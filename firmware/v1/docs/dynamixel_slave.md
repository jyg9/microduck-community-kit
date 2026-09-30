# Dynamixel 2.0 从站实现说明（imu_to_dxl v1）

本文档说明本固件如何把 `imu_to_dxl` 板做成一个 **Dynamixel Protocol 2.0 从站**，
以及它必须满足主机（`microduck` 的 `duck-control/src/bus.rs`，经 `rustypot` 1.6.0）的哪些行为。
逐字节参考向量可在 `host/tools/test_protocols.c` 与 `host/bus.py` 的 `_self_test()` 中核对。

实现文件：`src/dxl2.c`（协议）、`src/crc16.c`（CRC）、`src/stuffing.c`（字节填充）、
`src/dev.c`（寄存器镜像）。

---

## 1. 帧格式（与 rustypot 1.6.0 完全一致）

### 1.1 指令帧（主机 → 节点）

```
FF FF FD 00 | ID | LEN_L LEN_H | INST | PARAM... | CRC_L CRC_H
LEN = len(PARAM) + 3          （INST + PARAM + CRC）
整帧 = 7 + LEN
```

### 1.2 状态帧（节点 → 主机）

```
FF FF FD 00 | ID | LEN_L LEN_H | 0x55 | ERR | PARAM... | CRC_L CRC_H
LEN = 1(0x55) + 1(ERR) + len(PARAM) + 2(CRC) = len(PARAM) + 4
整帧 = 7 + LEN
```

`0x55` 是状态帧标记（指令帧该位置是 INST）。`rustypot` 的
`StatusPacketV2::from_bytes` 硬性要求 `data[7] == 0x55`，并检查 `LEN == len(frame) - 7`。

### 1.3 CRC-16

ROBOTIS SDK 的 `update_crc`：多项式 `0x8005`、初值 0、**高位在前**（不反射）、无最终异或。
**CRC 覆盖整帧（含 `FF FF FD 00` 帧头）直到 CRC 字段之前**。
`src/crc16.c` 的实现与 rustypot 的表驱动实现对同一组向量输出一致
（例如 ping id=2 → `0x7219`），见 `host/tools/test_crc16.c`。

### 1.4 字节填充（容易漏掉，但真机会炸）

协议 2.0 规定：**参数区**出现 `FF FF FD` 时，要在其后**插入一个 `0xFD`**，
并且 **`LEN` 与 CRC 都按填充后的字节计算**；接收方只在 CRC 校验通过后才去掉填充。

这不只是规范细节：`microduck` 用的 `rustypot` 1.6.0 在解析状态帧时会调用
`remove_stuffing(&data[8..len-2])`（ERR + 参数区）。12 字节 IMU 块里出现
`FF FF`（例如 `gyro_x = -1`）后跟一个 `0xFD` 字节完全可能，所以**不填充就会被解析错位**。

本实现的规则（与官方 DynamixelSDK `addStuffing` 一致）：

* 模式扫描只在**原始数据**上进行，插入的 `0xFD` 不会再触发新的匹配；
* `FF FF FF FD` 仍然要插一个字节（`FF FF` 后缀在多余 `FF` 之间保持有效）。

`src/stuffing.c` 的单元测试直接使用 rustypot 自己的向量：

| 输入 | 输出 |
|---|---|
| `01 02 FF FD 03` | 不变（没有 `FF FF FD`） |
| `FF FF FD 07` | `FF FF FD FD 07` |
| `FF FF FD FD` | `FF FF FD FD FD` |
| `FF FF FF FD` | `FF FF FF FD FD` |

指令方向同理：**节点解析指令前必须去掉参数区的填充**，否则 `LEN` 对应的参数长度会算错。
`src/dxl2.c` 在 CRC 校验通过后对 `f[8 .. len-3]` 做一次 `stuffing_remove`，
指令与状态两类帧共用同一段代码（状态帧的这段正是 ERR+参数）。

---

## 2. 指令集支持

| INST | 名称 | 实现 | 说明 |
|---|---|---|---|
| `0x01` | PING | ✅ | 返回 `型号(2) + 固件版本(1)`；广播 PING 也回答（用本机 ID） |
| `0x02` | READ | ✅ | `ADDR(2) + LEN(2)`；越界返回 RANGE，长度 0 返回 LENGTH |
| `0x03` | WRITE | ✅ | 广播写执行但不应答；只写只读区返回 ACCESS（整帧拒绝，不做部分写入） |
| `0x04` | REG_WRITE | ✅ | 暂存（最多 32 字节） |
| `0x05` | ACTION | ✅ | 执行暂存写 |
| `0x06` | FACTORY_RESET | ✅ | 恢复默认并写 Flash |
| `0x08` | REBOOT | ✅ | 先回应答，50 ms 后 `NVIC_SystemReset()` |
| `0x10` | CLEAR | ✅ | 清硬件错误状态（地址 70/71） |
| `0x82` | SYNC_READ | ✅ | 仅在 ID 列表包含本机 ID 时应答（**本机排在列表第一位时最先应答**，正是 microduck 依赖的顺序） |
| `0x83` | SYNC_WRITE | ✅ | 仅在列表包含本机 ID 时写入；**不应答** |
| `0x8A` | FAST_SYNC_READ | ✅ | 指令与 `0x82` 同形；应答是**所有设备共拼的一个聚合状态帧**（`ID=0xFE`，每台按 ID 列表顺序各发 `ERR + ID + DATA + 累积 CRC`，**不填充**，第 0 台额外发 8 字节前缀，`LEN = 1 + N×(X+4)`）。本机在第 k 位时**按前面 k 段的字节数**让位（不是 300 µs 槽，一段只有约 `(X+4)×10 µs`）；前导设备缺席则整包本就无效，本机静默（见 `imu_to_dxl_protocol.md` §3.2.1） |
| `0x92` | BULK_READ | ✅ | 参数布局为 `(ADDR,LEN) × N + ID × N`（`LEN` 共 7N 字节） |
| 其它 | — | 应答 `ERR=0x40`（Instruction Error），广播时静默 |

`STATUS_RETURN_LEVEL`（寄存器 68，默认 2）按协议生效：0 = 只有 PING 应答；
1 = PING + READ/SYNC_READ/FAST_SYNC_READ；2 = 全部。

### 错误位（与 rustypot 的解码一致）

| 位 | 值 | 含义 |
|---|---|---|
| bit1 | `0x02` | Access（访问只读区） |
| bit2 | `0x04` | Limit |
| bit3 | `0x08` | Length |
| bit4 | `0x10` | Range |
| bit5 | `0x20` | Checksum（CRC 错） |
| bit6 | `0x40` | Instruction（不支持的指令） |
| bit7 | `0x80` | Result Fail |

CRC 错误时：若帧的 ID 是本机 ID，回一帧 `ERR=0x20` 的状态帧（参数为空）；否则静默丢弃。

---

## 3. 寄存器镜像

节点维护一份 256 字节的寄存器镜像（`src/dev.c`），布局按 **XL330 控制表**，
这样任何按 XL330 写的工具（包括日志里出现过的寄存器断言）都能读到合理值。
镜像同时是遥测块的宿主：主机在地址 124 读到的前 12 字节就是 IMU 数据。

### 3.1 关键地址

| 地址 | 名称 | 我们的取值 | 说明 |
|---|---|---|---|
| 0~1 | Model Number | 1200 (`0x04B0`) | 与 XL330 相同，工具会按 XL330 的单位换算 |
| 6 | Firmware Version | `0x10`（v1.0） | |
| 7 | ID | 200（可写，持久化，clamp 1~252） | |
| 8 | Baud Rate | 3 = 1 Mbps | 与 `microduck` 的 `BAUD_RATE` 一致 |
| 9 | Return Delay Time | 0 | `microduck` 启动时会断言所有舵机为 0；本节点本来就 0 |
| 10/11/12/13 | Drive Mode / Operating Mode / Secondary ID / Protocol Type | 0 / 3 / 255 / 2 | 模仿 XL330 初始值 |
| 20 | Homing Offset | 0 | |
| 24 | Moving Threshold | 10 | |
| 31 | Temperature Limit | 70 | |
| 32/34 | Max/Min Voltage Limit | 70 / 35 | 数据手册初始值 |
| 36/38 | PWM / Current Limit | 885 / 1750 | |
| 44/48/52 | Velocity / Max Position / Min Position Limit | 445 / 4095 / 0 | |
| 60/62/63 | Startup Config / PWM Slope / Shutdown | 0 / 140 / 53 | 数据手册初始值 |
| 64 | Torque Enable | 0 | 可写，无实际作用（本节点不运动） |
| 68 | Status Return Level | 2 | |
| 76~90 | 速度/位置增益 | 1600 / 180 / 0 / 0 / 400 / 0 / 0 | 读取无害；写入只存在镜像里 |
| 98 | Bus Watchdog | 0 | |
| 100~119 | Goal PWM/Current/Velocity/Position 等 | 0 | 可写，无实际作用 |
| **120~121** | **Realtime Tick** | 实时（`ms & 0xFFFF`） | 每秒刷新 |
| 122/123 | Moving / Moving Status | 0 | |
| **124~143** | **IMU 遥测块（20 字节）** | 见 §4 | 每次读取前刷新 |
| 144~145 | Present Input Voltage | **0** | 本节点没有电池分压；`microduck` 的电压平均会过滤 0，不会把假值算进去 |
| 146 | Present Temperature | 片内温度传感器（ADC0 通道 16） | 每秒更新一次 |
| 147 | Backup Ready | 0 | |
| **148~167** | **厂商配置窗口** | 见 README §6 | 同一张表在飞特侧是 160~179 |
| 168~223 | Indirect Address | 默认 224~251 | 模仿 XL330 |
| 224~251 | Indirect Data | 0 | |

只读区（本节点会拒绝写入）：`0~6`（型号/信息/版本）、`69`、`70~71`、`120~123`、
`124~147`（遥测/电压/温度/backup）。

厂商窗口（148~167）整段**可写**，包括 `STATUS`(166)/保留(167)：
主机通常一次写 20 字节，若拒绝会连带把可用字段一起挡掉；这两个字节写入被忽略，
每次读之前 `dev_refresh()` 会用真实状态覆盖 `STATUS`。

### 3.2 与 XL330 的差异（诚实清单）

* **型号号</b>报 1200**：这是有意的——所有按 XL330 写的工具都会据此选择
  "1 圈 4096 计数、0.229 rev/min/count" 的换算；而本节点的 124 地址块是厂商自定义语义，
  真正解析它的是 `microduck` 里的 `SflpDecoder`，与型号号无关。
* `present_current`(126)、`present_velocity`(128)、`present_position`(132) 等地址被
  遥测块**覆盖**，读到的不是舵机语义的电流/速度/位置。这是官方板"地址 124 起 12 字节与舵机同址"
  设计的必然结果，也是 `microduck` 只把前 12 字节交给 `SflpDecoder` 的原因。
* 本节点不实现 Indirect Address/Data 的间接寻址语义（镜像里可读写，但不重定向）。
* `Torque Enable`、增益、Goal 系列寄存器只有镜像语义，没有任何执行机构。

---

## 4. 遥测块与刷新

20 字节布局见 `README.md` §3。**FeeTech 的人格在地址 56 只暴露前 15 字节**（12 控制 +
采样计数 + 状态位 + 保留 0，`src/telem_pack.c`），完整 20 字节块在 128；Dynamixel 侧
124 起 20 字节不变。原因见 `docs/bus_timing_borrow_plan.md`：真舵机在 56 只答 15 字节，
而一次 `sync_read` 的读长对所有设备必须相同。

刷新（`dev_refresh()`）**由主循环发布**，不再由读触发：

1. 主循环在"有新样本（`imu_samples()` 变化）或距上次超过 50 ms"时调用一次；
2. 刷新内容：型号/版本（自愈式重写，防止被写坏）、实时 tick、契约块与诊断块、
   144~145 电压、146 温度、身份窗、厂商窗口（含 `V_LAST_ERR`）；
3. 整段发布在一段极短临界区（`src/critical.h`）里完成，所以中断里读到的块是
   "整块新"或"整块旧"，不会读到半新半旧的四元数。

`sync_read`（一次事务）与单条 `READ` 看到的是同一份一致数据。控制环每 tick 只取前 12 字节，
正好是 `READ_ADDR = 124, READ_LEN = 12`（FeeTech 是 `56, 15`）。

---

## 4.1 应答时序：位次槽 + 中断/DMA

真舵机在 `sync_read` 里为**每个前导 ID** 等一个应答槽（实测 ≈295 µs，前导设备缺席也占一槽；
见 `docs/bus_timing_borrow_plan.md` §1.3）。因此：

* 帧的解析在 **USART 接收中断里**完成，应答交给 **DMA** 发送
  （USART0_TX = DMA0 CH3，USART1_TX = DMA0 CH6）——排在后面的设备只留 ≈295 µs；
* 本机在 ID 列表里的下标 k > 0 时，先用 `src/bus_arb.c` 让出 k 个槽
  （`BUS_SLOT_US = 300`）再发；等待期间若出现新的指令帧（本机 ID 或广播）则作废；
* 运行时 IMU 恒排第 0 位，所以热路径是"收完指令立即发"，T_resp 与主循环无关；
* 若同一个 UART 上还有调试输出（开发板的镜像口），日志行会在字节边界让路给协议帧。

---

## 5. `microduck` 侧依赖的行为（回归清单）

改固件时请对照这张表，这些都是 `duck-control/src/bus.rs` 与 `imu.rs` 真实依赖的点：

| 依赖 | 固件对应实现 |
|---|---|
| 一次 `sync_read(ids=[IMU, 舵机...], 124, 12)`，IMU 排第一且**最先应答** | `dxl2.c` 解析完广播帧后应答；下标 0 时不走仲裁、直接 DMA 发送 |
| 12 字节块 = 陀螺(6) + 四元数 xyz half(6) | `imu_sim.c` 打包（真驱动同布局）；FeeTech 侧同一份前 12 字节 |
| 单位：陀螺 17.5 mdps/LSB；四元数 IEEE half；`w = √(1-x²-y²-z²)` | `f32_to_half()` + `GYRO_COUNTS_PER_RAD_S` |
| 四元数字节全 0 表示"SFLP 未就绪"，主机保持上一有效值 | SIM_MODE 5；真驱动在融合未收敛时同样输出全 0 |
| 连续 25 个有效四元数样本才 `ready()` | 100 Hz 刷新，25 样本 ≈ 0.25 s |
| "块 12 字节与上一 tick 完全相同"算 stale | 模拟器每 10 ms 重新生成；SIM_MODE 6 专门制造相同的块 |
| 启动时对每个舵机 `read/write` return_delay_time(9)/baud_rate(8)/pwm_slope(62)/shutdown(63) | 本节点只对**本机 ID** 的读写负责，舵机地址表照 XL330 保留 |
| 60 Hz 级读超时 30 ms | 应答在中断里组帧并由 DMA 发送，T_resp 目标 ≤100 µs（实测受 295 µs 槽长约束） |
| FeeTech 侧：ACK STATUS 恒 0，缺席/故障由超时与 `V_LAST_ERR` 反映 | `fee.c` 恒回 0；内部错误记 `V_LAST_ERR`(179) |

---

## 6. 性能与缓冲

| 项 | 值 | 说明 |
|---|---|---|
| 最大帧长 | 512 字节（`DXL_MAX_FRAME`/`DXL_MAX_RESP`） | 覆盖 256 字节寄存器读 + 填充 + 帧头 + CRC |
| 去填充后的体缓冲 | 264 字节（`DXL_MAX_BODY`） | ERR + 参数 |
| 解析超时 | 20 ms（`DXL_FRAME_TIMEOUT_MS`） | 帧内不再收到字节即复位状态机 |
| 间隙判据 | 帧进行中且字节间隙 > 500 µs（`BUS_GAP_US`）→ 丢弃残帧并计 `gaps` | 只统计"帧中途停顿"，空闲间隔不算 |
| 发送 | DMA（USART0_TX = DMA0 CH3 / USART1_TX = DMA0 CH6），中断里组帧后启动 | 在中断里自旋发 21 字节会饿死其它中断 |
| 自发自收抑制 | 发送期间丢弃 RX（`tx_busy`）+ 发送完成后 2 字节（`echo_skip`） | 取代原来的"发送后 2 ms 静默窗口"（那个窗口会把紧随的主机指令一起吞掉） |
| 1 Mbps 分帧 | USART1 在 APB1=60 MHz，`BRR=60`；USART0 在 APB2=120 MHz，`BRR=120` | 都是精确值 |

---

## 7. 手工验证方法（无 microduck 环境）

```bash
# 协议向量（不需要硬件）
host/tools/run_c_tests.sh
../.venv/bin/python host/bus.py

# 上板：两种协议 + 全部模拟模式 + 延迟/一致性统计
./build.sh smoke

# 单条命令（需要 pyserial）
../.venv/bin/python - <<'EOF'
import sys; sys.path.insert(0, "host")
from bus import Link
with Link("/dev/ttyACM0", 1_000_000, "dxl", 200, read_len=20) as l:
    print("info", l.device_info())
    print("cfg ", l.get_config())
    for _ in range(5):
        s = l.poll(); print(f"{s['gyro_dps']} {s['euler']} lat={s['latency_ms']:.2f}ms")
EOF
```

若把这段代码换到真机器人上：Dynamixel 用 124/12（或 124/20 拿诊断尾部），
FeeTech 用 **56/15**（契约块，与 15 颗舵机同一次 `sync_read`），诊断块在 128/20；
`host/bus.py` 的 `Link.read_contract()` 已经把这两种形状都封装好了。

真舵机侧的只读对照（应答槽长、契约块长度、寄存器底账）用
`host/tools/servo_probe.py` 复现，见 `docs/bus_timing_borrow_plan.md` §1。
