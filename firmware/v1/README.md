# imu_to_dxl v1 — GD32F303CC 固件

复刻官方未开源的 `imu_to_dxl` 板：把 IMU 姿态数据做成**电机总线上的一个从站节点**，
让机器人控制软件（`microduck` 的 `robotd`）用**同一次 sync_read** 同时拿到 IMU 与全部舵机状态，
主机侧不需要任何额外轮询或融合。

* 主控：GD32F303CC（LQFP48，256 KB Flash / 48 KB SRAM，12 MHz 晶振 → 120 MHz）
* 目标板：`elec_imu_to_dxl/imu_to_dxl`，**PCBA 已回板并烧写**（bootloader + slot A），
  节点 ID 200 挂在 1 Mbps 半双工伺服总线上（USART1，PA2/PA3）
* 真板传感器：**LSM6DSV16X**（SPI0：PA4=CS / PA5=SCK / PA6=MISO / PA7=MOSI，INT1/INT2 = PB0/PB1），
  由 `IMU_USE_SPI=1`（默认）选 `src/imu_spi.c` + `src/lsm6dsv16x.h`；`=0` 时回到模拟器 `src/imu_sim.c`
* 台架拓扑：PC 经 USB 串口适配器接**电机总线**（USART1，PA2/PA3；15 颗 HD-1910-C001：右腿 10-14、
  左腿 20-24、颈/头/嘴 30-34），适配器通常枚举成 `/dev/ttyACM0`（换 USB 口后先 `ls /dev/ttyACM*` 确认）；
  调试口是 CH340 的 `/dev/ttyUSB*`（USART0，PB6/PB7，115200，**只输出 `DBG_*` 日志、不承载任何协议**）
* 两种协议：**Dynamixel 2.0**（ID 200，地址 124）与**飞特 SCS/HLS**（ID 200，地址 56），节点自动识别
* 上位机：`host/`（本地网页版，3D 姿态 + 实时瀑布流 + 抖动/偏移统计）

> 相关文档
> * `docs/dynamixel_slave.md` —— Dynamixel 从站实现细节与寄存器表（含 2.0 字节填充）
> * `docs/flash_layout.md` —— Flash 分区 / bootloader / 总线升级协议设计（待实现）
> * `docs/lsm6dsv16x_driver_spec.md` —— 真 LSM6DSV16X 驱动规格（含 §8.1 CONVERGE 实现状态）
> * `docs/bus_timing_borrow_plan.md` —— 总线时序风险分析 + 借鉴计划（**已实施**；真机缺陷见 §4.6，真机验证见 §7）
> * `patches/README.md` —— 官方 `microduck` 仓库飞特补丁说明（补丁文件同目录；原名 `microduck_feetech_patch.md`，`2fc4731` 改名）
> * `../../software/microduck_feetech/` —— 打好该补丁的官方 `microduck` 完整源码（可直接编译运行 `robotd`）
> * `../../docs/飞特通讯协议说明.md` —— 飞特协议完整说明
> * `host/README.md` —— 上位机使用说明

---

## 1. 当前状态（诚实清单）

| 功能 | 状态 | 验证方式 |
|---|---|---|
| 系统时钟 120 MHz（12 MHz 晶振） | ✅ | 上板运行（LED 心跳 + 串口 banner） |
| Dynamixel 2.0 从站（PING/READ/WRITE/REG/SYNC_READ/SYNC_WRITE/**FAST_SYNC_READ 0x8A**/BULK_READ/RESET/REBOOT/CLEAR） | ✅ | `host/tools/test_protocols.c` 逐字节比对 rustypot 参考向量；`host/tools/rustypot_check/` 再用**真的 rustypot 1.8 解析器**对拍 |
| Dynamixel 2.0 `0x8A` 聚合应答（`ID=0xFE`、不填充、逐段累积 CRC、按字节数让位） | ✅ | C 测试（含 e-manual 32 字节向量与 `FF FF FD` 反证）+ 真机 `host/tools/fast_sync_read_check.py`（19 项，见 `docs/imu_to_dxl_protocol.md` §11(c2) 与 `docs/bus_timing_borrow_plan.md` §7.8）；真 XL330 同总线未验证，见 `docs/imu_to_dxl_protocol.md` §15.5 第 11 项 |
| Dynamixel 2.0 字节填充（`FF FF FD`） | ✅ | 同上 + `src/stuffing.c` 单元测试（rustypot 1.6 会解填充） |
| 飞特 SCS/HLS 从站（PING/READ/WRITE/REG/ACTION/RECOVERY/**REBOOT 0x08**/RESET/CAL/SYNC_READ/SYNC_WRITE） | ✅ | 同上 + `host/bus.py` 自检 |
| 协议自动识别 + 粘滞 + 可锁定 | ✅ | `host/tools/bus_smoke.py`（上板后可跑） |
| 模拟 IMU（7 种模式，四元数与陀螺自洽） | ✅（`IMU_USE_SPI=0`） | C 测试：陀螺/四元数导数一致性、半精度、安装矩阵往返 |
| **真 LSM6DSV16X SPI 驱动（`src/imu_spi.{c,h}` + `src/lsm6dsv16x.h`）** | ✅ 已实现 + 真机验证 | `bus_smoke.py` 真传感器路径 + `host/tools/sensor_probe.py`；见 §8.6 |
| **`IMU_USE_SPI` 构建开关（默认 1 = 真驱动，0 = 模拟器）** | ✅ | `src/board.h`、`CMakeLists.txt` |
| 寄存器镜像（XL330 控制表 + HLS 内存表 + 厂商窗口） | ✅ | C 测试 + 上位机读写 |
| 片上温度 / 实时 tick / 状态位 | ✅ | 寄存器读取 |
| 调试串口（可宏关闭、运行时可调级别） | ✅ | 上板（banner + 2 s 周期统计） |
| USART1 电机总线 + USART0 只输出日志的调试口 | ✅ | 见 §5；协议与升级只走总线（PC 侧通常 `/dev/ttyACM0`）；镜像口是默认关闭的台架选项 |
| 配置存 Flash（ID/波特率/模拟参数） | ✅ | `src/dev.c`（最后一页 + CRC16） |
| 上位机网页版（3D + 瀑布流 + 统计 + 录制 CSV） | ✅ | HTTP/WS 集成测试 + 前端桩测试（见 §7） |
| **bootloader（32 KB）+ A/B slot 启动决策** | ✅ 已实现 | `host/tools/test_boot.c`（决策表全表）+ 真机启动 A/B/回滚 |
| **电机总线串口升级（块 CRC + 两段镜像 CRC32 + trial/回滚）** | ✅ 已实现 | `host/tools/test_boot.c` + `host/tools/test_upgrade.py`（pty 端到端）+ 真机 3 次升级 |
| **Linux 升级工具（包校验 / 版本比较与提示 / 升级 / 回读）** | ✅ 已实现 | `host/upgrade.py`，见 `docs/upgrade.md` |
| **sync_read 位次仲裁（答在自己的槽里）** | ✅ 已实现 + 真机验证 | 真机 1/2/3 个前导 ID → +339/+676/+1001 µs；`test_protocols.c` 的 `bus_arb` 用例 |
| **中断解析 + DMA 应答（T_resp 有界）+ 原子遥测发布** | ✅ 已实现 + 真机验证 | 双协议冒烟 2555 次轮询、0 超时 0 坏帧；`bus_arb`/`telem_pack` 主机用例 |
| **FeeTech 15 字节契约块（56）+ 20 字节诊断块（128/124）** | ✅ 已实现 + 真机验证 | `bus_smoke.py`：56/15、byte14=0、计数递增、与 128 一致 |
| microduck 飞特补丁 | ✅ 已编译 + 单测（93 项） | `patches/microduck-feetech.patch` |

**已在真硬件上实测**：协议（两种）、bootloader 启动决策、总线升级（3 次，含 trial
自确认与 3 次未确认回滚）、升级前后协议冒烟全过 —— 记录见 `docs/upgrade.md` §8。
时序改造（位次仲裁 / 中断+DMA 应答 / 契约块）有一轮真机验证（`docs/bus_timing_borrow_plan.md` §7）；
真 PCBA + 真 LSM6DSV16X + 15 颗舵机的一轮见本文 §8.6 与计划文档 §7.6/§7.7。

**真硬件已验证（2026-09-25）**：真 LSM6DSV16X（`WHO_AM_I`=0x70、SPI0 mode 0 @7.5 MHz、
静止 996–997 mg、`|q|`=1.0000、四元数重力 vs 加速度 0.05–0.12°、陀螺零偏 (−0.97,+0.16,+0.73) dps）、
与 15 颗舵机共总线（16 ID `sync_read` 150/150 扭矩开/关、背靠背 200/200）、
真机联调修掉的三个固件缺陷（回声窗口 / µs 时钟回绕 / SFLP CONVERGE，见计划文档 §4.6）。

**仍未在真硬件上验证的部分**：绝对 T_resp（要示波器/逻辑分析仪，宿主 USB 时间戳只到 ~0.3 ms）；
本节点排在 `sync_read` ID 列表中间/末尾时的让位（运行时 200 固定第 0 位）；
满槽 96 KB 升级与擦除中途掉电演练。（主机侧 `robotd` 突发截断曾列在此处，**2026-10-02 已修**：
删 `tcdrain` + `BURST_IDLE` 2 → 4 ms，见 `docs/bus_timing_borrow_plan.md §7.7`。）

---

## 2. 硬件与引脚

| 引脚 | 功能 | 说明 |
|---|---|---|
| PA2 / PA3 | USART1 TX / RX | **电机总线**（单线半双工）。节点与 15 颗舵机共用，全部寄存器协议流量与现场升级只在这里（PC 侧 USB 串口适配器，通常 `/dev/ttyACM0`）。方向控制由硬件从 TX 自动产生（原理图里的 `TXD_EN`，Q1+R28/R29/C22），固件不控制任何 GPIO |
| PA4 | SPI0_NSS | IMU 片选（低有效）；`IMU_USE_SPI=1` 时由 `src/imu_spi.c` 驱动，传输间保持高 |
| PA5 / PA6 / PA7 | SPI0_SCK / MISO / MOSI | IMU SPI，`IMU_USE_SPI=1` 时是 LSM6DSV16X 总线（mode 0 @ 7.5 MHz）；`=0` 时浮空不用 |
| PB0 / PB1 | IMU INT1 / INT2 | LSM6DSV16X 上拉输入；驱动轮询，但已把 FIFO 水位/溢出路由到 INT1，留给将来中断或示波器 |
| PB6 / PB7 | USART0 TX / RX | **只输出调试日志**的串口（CH340，PC 侧 `/dev/ttyUSB*`，115200），不解析、不应答任何协议帧；`BUS_MIRROR_ENABLE=1` 时才是台架镜像口（见 §5） |
| PB5 | 状态 LED（开发板） | 1 Hz 心跳；5 s 没收到总线帧则 8 Hz 闪（提示"没有主机在说话"） |

时钟：`(HXTAL / 2) * 20 = 120 MHz`。**注意** GD32 官方 `system_gd32f30x.c` 默认按 8 MHz 晶振
硬编码 `RCU_PLL_MUL30`，在 12 MHz 板上会算成 180 MHz 而卡死；本工程自带一份可选时钟源的副本
（`src/system_gd32f30x.c`），`HXTAL_VALUE` 也必须通过 CMake 传成 12 MHz，否则波特率会错。

---

## 3. 遥测块（与官方板一致）

20 字节诊断块，两种协议的**内容**相同，但 FeeTech 的 56 处只暴露前 15 字节（见 §4）：

| 字节 | 内容 | 单位 |
|---|---|---|
| 0~5 | 陀螺 x/y/z，i16 小端 | ±500 dps，**17.5 mdps/LSB** |
| 6~11 | 四元数 x/y/z，IEEE half（binary16） | `w = √(1-x²-y²-z²)` |
| 12~17 | 加速度 x/y/z，i16 小端 | ±4 g，0.122 mg/LSB |
| 18 | 采样计数（u8，回绕） | 100 Hz |
| 19 | 状态位 | `0x01` SFLP 有效（本块带四元数）/ `0x02` 计数回绕 / `0x04` 陀螺饱和 / `0x08` 传感器错误 / `0x10` 模拟数据 / `0x20` 冻结 / `0x40` 配置未保存 / `0x80` 融合算法**正在运行**（`FUSION_OK`，不等于已收敛） |

控制环只读**前 12 字节**（陀螺 + 四元数 xyz）——这正是官方板"与舵机共用地址、塞进同一事务"的做法。

**FeeTech 侧的"契约块"（重要）**：真舵机在地址 56 起只答 **15 字节**
（位置/速度/负载/电压/温度/状态/移动/电流），而运行时的 `sync_read` 对 IMU 节点和 15 颗舵机
用的是**同一个长度**。所以本节点在 56 处也回答 15 字节：
`12 字节控制块 + 采样计数(12) + 状态位(13) + 保留 0(14)`（`src/telem_pack.c`），
**加速度搬到 128 别名的 20 字节诊断块**。选 15 而不是 20 是量出来的：每多 5 字节 × 16 个设备
就要多约 0.8 ms 总线时间（见 `docs/bus_timing_borrow_plan.md` §2、§7）。

**坐标系**：板子安装使 `躯干系 = [+raw_z, +raw_y, −raw_x]`（绕 Y 轴 +90°）。
固件上报的是**芯片系**数据，主机用 `microduck` 的 `DEFAULT_MOUNT = [1/√2, 0, 1/√2, 0]`
左乘/右乘还原（`q_trunk = q_raw ⊗ mount⁻¹`，`ω_trunk = mount · ω_raw`）。
调试时可用厂商寄存器 `REPORT_FRAME=1` 让固件直接上报躯干系（主机就不要再转）。

**已知偏差**：真板存在约 5° 的系统性 pitch 偏置，由 runtime 在源头校正（见 `microduck` 注释）。
上位机 3D 视图会显示重力与"世界竖直"的夹角，可用来测这个偏置。

---

## 4. 两种协议下的地址映射

| | Dynamixel 2.0 | 飞特 SCS/HLS |
|---|---|---|
| 帧结构 | `FF FF FD 00 ID LEN_L LEN_H INST ... CRC16`（ROBOTIS CRC-16，**含帧头**；状态帧带 `0x55` 标记；**参数区字节填充**） | `FF FF ID LEN INST [ADDR] DATA ~SUM`（8 位取反和；无填充） |
| 节点 ID | 200（寄存器 7，可改并持久化） | 200（寄存器 5，可改并持久化，禁止 0xFD） |
| 遥测块地址 | **124**（`present_pwm` 起 20 字节，前 12 是控制环要的） | **56**（`PRESENT_POSITION_L` 起 **15** 字节：12 控制 + 计数 + 状态 + 保留 0）；完整 20 字节诊断块在 **128** |
| 波特率 | 1 Mbps（寄存器 8 = 3） | 1 Mbps（寄存器 6 = 0） |
| 每次调度读的地址/长度 | `124` / 12（主机可读 20 拿诊断尾部） | `56` / **15**（与 15 颗舵机同一次 `sync_read`，长度必须对所有设备成立） |
| 厂商配置窗口 | **148~167**（20 字节） | **160~179**（同一张表） |
| 设备语义 | 状态返回级别(68)、只读区拒绝写入 | 应答状态级别(8，默认 1)、锁标志(55，默认 0=可持久化)、角度限位/死区/偏移等 |
| 只读信息 | 型号 1200（`0x04B0`）、固件版本 0x10 | 固件版本 1.0（寄存器 0/1）、型号 `0x4D49`（'I','M'，**故意不是舵机型号**） |
| 应答 STATUS 字节 | 按协议（`DXL_ERR_*`，rustypot 会解码） | **恒 0**（飞特工具会把非 0 当舵机故障）；最近一次内部错误放厂商窗 `V_LAST_ERR`(179) |
| 5 V / 诊断 | 144~146（电压、温度） | 128~147（诊断块 + 别名）；71~75 不参与契约，保持 0 |

飞特侧另外按厂商手册实现了两个容易被忽略的语义：寄存器 8（应答状态级别，写 0 后写指令执行但静默、
读指令仍应答）与寄存器 55（锁标志，加锁时写入仍生效但不持久化——手册里它是"是否掉电保存"，不是"是否允许写"）。
两项都有上板回归（`host/tools/bus_smoke.py` 的 `device semantics` 段）。

**协议自动识别**：一帧的第 3 字节为 `0xFD` → Dynamixel 2.0；否则 → 飞特。
成功应答一种协议后，该端口 100 ms 内只认这种协议（`BUS_PROTO_STICKY_MS`），
避免把数据里的 `FF FF <本机ID>` 误判；厂商寄存器 `PROTO_LOCK`（0 自动 / 1 仅 DXL / 2 仅飞特）可强制。

**位次与应答时序（实测，见 `docs/bus_timing_borrow_plan.md` §7）**：真舵机在 `sync_read`
里会为**每一个排在它前面的 ID** 等一个"应答槽"（实测 ≈295 µs，前导设备缺席也照样占一槽）。
本节点在 ID 列表里的下标 k > 0 时，会先让出 k 个槽再应答（`src/bus_arb.c`）；运行时 IMU 节点
固定排第 0 位，所以那条热路径是"收完指令立刻发"。应答的解析在 USART 中断里、
发送走 **DMA**（USART0_TX = DMA0 CH3，USART1_TX = DMA0 CH6），因为排在后面的设备
只留 ≈295 µs，主循环给不出这个上界。

**FeeTech `0x08` REBOOT（实测补充，见计划文档 §1.7）**：这颗 HD-1910（fw 3.46）**支持** 0x08——
**不应答**、约 **823 ms** 回来、ID/波特率/模式/锁/OFS 全保留、**RAM 增益(50/51)回落到 EEPROM(21/22)**
（正是 `RobotIo::reboot` 契约要的语义）。本节点的飞特人格据此实现 0x08（不应答地重启，
应用与 bootloader 共用 `dev_request_reboot()`）；主机补丁的 `reboot()` 也从"明确不支持"
改成"先写扭矩 0、发 0x08、不等应答立即返回"。

---

## 5. 调试口（只输出日志）与电机总线

**电机总线是 `USART1`（PA2/PA3）**：节点和 15 颗舵机共用它，单线半双工；所有寄存器协议流量
（Dynamixel 2.0 / 飞特）与**现场升级**都只在这条总线上发生，别处没有。PC 侧是 USB 串口适配器，
通常枚举成 **`/dev/ttyACM0`**（ACM 编号跟着适配器枚举走，换 USB 口/插别的设备后先
`ls /dev/ttyACM*` 确认）；`host/`、`./build.sh smoke` 与 `host/upgrade.py` 都默认用它。

**调试口是 `USART0`（PB6/PB7）**：CH340，PC 侧是 `/dev/ttyUSB*`，115200，**只输出日志**，
不承载任何协议——它不解析、也不应答协议帧，所以终端上敲的东西不会被当成总线帧。
日志行以 `#` 开头、ASCII 里没有 `0xFF`（主机分帧器可以安全地在文本里重新同步）。

* `BUS_MIRROR_ENABLE`（`src/board.h`、`CMakeLists.txt`）现在**默认 0**，上面就是默认行为。
* 只有在"开发板上电机总线还没接"的**台架**场景才把它设为 1：此时 `USART0` 临时兼作第二个
  协议口（1 Mbps，波特率编码见 §6 的 `MIRROR_BAUD`），方便一块板子上同时看调试文本与协议。
  注意这时的 `USART0` **什么都能做，包括升级**——协议层不区分端口。所以升级工具
  `--port` 默认指向电机总线（`auto` 会打印它选了哪个设备，见 `docs/upgrade.md` §4.5），
  不要指到调试口上去。
* 升级端口由**工具**负责选对：默认 `--port auto` **逐个探测** `ttyACM0-1`、
  `ttyUSB0-5`、`ttyS2` 里存在的设备，取第一个真的能应答节点的那个并打印它选了谁
  （存在的 `ttyUSB*` 不会被按名字排除：它既可能是总线适配器，也可能是调试口，
  按"能不能应答"判断）；同时**拒绝和另一个主机抢端口**（机器人上那是 robotd，
  工具会打印 stop/upgrade/start 三行提示）。见 `docs/upgrade.md` §4.5。

`./build.sh` 现在会在配置之后打印**有效的缓存选项**（`APP_SLOT APP_VERSION_STR BOOT_ENABLE
DBG_ENABLE BUS_MIRROR_ENABLE IMU_USE_SPI`），`BUS_MIRROR_ENABLE=1` 时还会加一条 NOTE。
CMake 的缓存会跨源码更新保留旧值，所以老的 `build/` 目录可能仍是台架镜像构建：
如果摘要里是 1，用 `./build.sh clean`（或 `-DBUS_MIRROR_ENABLE=0`）重新配置。

**调试口的物理引脚**（这块 PCBA 上是一个 4 pin 排针 **H1**；网表见 `hardware/imu_to_dxl`）：

| H1 引脚 | 信号 | 说明 |
|---|---|---|
| 1 | `+3.3V` | |
| 2 | `GND` | |
| 3 | `DEBUG_RX` | **PB7**，经串阻 R10，R7 上拉到 3.3V |
| 4 | `DEBUG_TX` | **PB6**，经串阻 R11 |

**PC 侧适配器的 TX 必须接到 H1-3**。只接 GND + TX 的"两根线"接法能读到日志（TX 一路是通的），
但节点收不到任何字节——日志里 `uart0 rx=0` 就是它的表现，`BUS_MIRROR_ENABLE=1` 的台架镜像模式
也就无从谈起（这一点在真板上实测确认过）。

---

## 6. IMU：真驱动与模拟器

`IMU_USE_SPI`（`src/board.h`、`CMakeLists.txt`，默认 **1**）选择传感器来源：

* **1 = 真 LSM6DSV16X**：`src/imu_spi.c` + `src/imu_spi.h` + `src/lsm6dsv16x.h`，
  SPI0（PA4–PA7），mode 0 @ 7.5 MHz，芯片自己做 SFLP 融合；`src/imu.c` 是门面并拥有 10 ms 采样网格。
* **0 = 模拟器**：`src/imu_sim.c` 用**同一条角速度**既积分四元数又生成陀螺计数，
  保证上报的陀螺与四元数严格自洽（这样才能真正验证上位机的 3D 视图与瀑布流，而不是看随机数）。

模拟器模式（仅 `IMU_USE_SPI=0` 的构建生效）：

| `SIM_MODE` | 名称 | 用途 |
|---|---|---|
| 0 | static | 静止 + LSB 级噪声：看抖动底噪 |
| 1 | sine | 三轴正弦（默认）：看 3D 姿态与瀑布流是否正常 |
| 2 | drift | 慢速偏航 + 注入陀螺偏置：看"偏移/漂移" |
| 3 | noise | 宽带陀螺噪声：看抖动与统计 |
| 4 | spin | 快速自转 |
| 5 | sflp-wait | 陀螺正常但四元数字节全 0：验证"保持上一有效值 / ready 门控" |
| 6 | frozen | 每次读出完全相同：验证 `microduck` 的 stale 检测 |

厂商窗口（20 字节，DXL 148 / 飞特 160）：

| 偏移 | 名称 | 读/写 | 说明 |
|---|---|---|---|
| 0~1 | MAGIC | RW | `0x4D49`（'I','M'）；写窗口时用于判断有效性 |
| 2 | CMD | W | 1 保存配置 / 2 恢复出厂 / 3 重启 / 4 重启运动模型（执行后自动清 0） |
| 3 | SIM_MODE | RW | 上表 0~6 |
| 4~5 | SIM_AMP | RW | 幅度千分比 100~5000（默认 1000） |
| 6~7 | SIM_FREQ | RW | 频率千分比 100~5000（默认 1000） |
| 8 | REPORT_FRAME | RW | 0 芯片系（默认，与真板一致）/ 1 躯干系 |
| 9 | PROTO_LOCK | RW | 0 自动 / 1 仅 Dynamixel / 2 仅飞特 |
| 10~15 | GYRO_BIAS_X/Y/Z | RW | i16，直接叠加到上报的陀螺原始计数（注入偏移或在线标定） |
| 16 | MIRROR_BAUD | RW | USART0（调试口 / 台架镜像口）波特率编码（飞特编码表）。`BUS_MIRROR_ENABLE=0` 时 USART0 固定 115200 调试口；此寄存器与电机总线（USART1）无关 |
| 17 | DBG_LEVEL | RW | 0 关 / 1 仅错误 / 2 启动+2 s 统计（默认）/ 3 每帧跟踪 |
| 18 | STATUS | R | bit0 配置未保存 / bit1 来自 Flash / bit2 使用模拟数据 |
| 19 | LAST_ERR | R | 最近一次被拒绝的协议操作（`DXL_ERR_*`）。应答 STATUS 恒 0，所以原因只能从这里问 |

配置（ID / 波特率 / 模拟参数）写入后**先等寄存器写入平静 500 ms，再等总线安静 1.5 s**，
才落 Flash 最后一页（`0x0803F800`，`CFG_FLASH_ENABLE=0` 可关）——
因为擦一页 Flash 会让内核（以及 USART 中断）停顿几十毫秒，在机器人持续轮询时保存会吃掉一个控制周期。
显式的 `CMD=1`（保存）是立即写，主机需要容忍那一次读超时。`CMD=2` 恢复出厂。

厂商窗口的 `STATUS`(18) 与 `LAST_ERR`(19) 可以写（整窗口 20 字节一次写入不会被拒），但写入无效——
主循环发布寄存器镜像时（`dev_refresh()`）会用真实状态覆盖它们。**寄存器镜像现在由主循环发布，
读取本身不再触发刷新**：读发生在 USART 中断里（见 §4 的时序段），那里不能做 ADC 转换或重写厂商窗。
发布在一段很短的临界区里完成，所以中断读到的是"整块新"或"整块旧"，不会读到半新半旧的四元数。

---

## 7. 编译、烧录与测试

```bash
cd v1

# 1) 构建（arm-none-eabi-gcc + cmake，无需网络）
./build.sh

# 2) 烧录（J-Link，设备名必须是 GD32F303CC，不能带封装后缀 T6）
./build.sh flash              # APP_SLOT 镜像（开发板构型，会覆盖 bootloader 区）
./build.sh flash-pair a       # 量产构型：一次擦除 + bootloader + slot A（推荐）
./build.sh flash-boot         # 只更新 bootloader（整片擦除）
./build.sh flash-slot b       # 只替换 slot B（不擦除，保留 bootloader）
./build.sh erase / reset

# 3) 全部主机测试（纯 gcc + Python，不需要硬件）
./build.sh test
#    -> CRC 对拍、两个协议从站、bootloader 决策表/flash 策略/升级状态机（9500+ 断言）
#    -> 升级包格式 + 升级工具 pty 端到端（173 断言）+ 编解码/服务自检
host/tools/run_c_tests.sh                          # 只跑 C 部分
../.venv/bin/python host/tools/test_upgrade.py     # 只跑升级工具端到端

# 4) 上板协议冒烟测试（需要串口；真/模拟两条路径，脚本读 TELEM_FLAG_SIMULATED 自动选）
./build.sh smoke
BUS_PORT=/dev/ttyACM0 BUS_BAUD=1000000 ./build.sh smoke      # 电机总线口（先 ls /dev/ttyACM* 确认编号）
BUS_PORT=/dev/ttyUSB0 BUS_BAUD=1000000 ./build.sh smoke      # 仅台架镜像构建（BUS_MIRROR_ENABLE=1）

# 4b) 真舵机只读探针（PING/READ/SYNC_READ，绝不写；见 docs/bus_timing_borrow_plan.md）
./host/tools/py.sh host/tools/servo_probe.py --port /dev/ttyACM0 --repeat 21

# 4c) 真 LSM6DSV16X 只读表征探针（四元数/加速度/重力夹角/零偏/SFLP 重启；见 §8.6）
./host/tools/py.sh host/tools/sensor_probe.py --port /dev/ttyACM0

# 4d) Fast Sync Read（0x8A）验收：本机在第 0 位 / 第 1 位 / 未被点名三种位置
./host/tools/py.sh host/tools/fast_sync_read_check.py --port /dev/ttyACM0
./host/tools/py.sh host/tools/fast_sync_read_check.py --self-test       # 不需要硬件

# 4e) 与官方 rustypot 解析器对拍（需要 Rust 工具链，故不在 ./build.sh test 里）
cd host/tools/rustypot_check && cargo run --release && cd -

# 5) 现场升级（详见 docs/upgrade.md）
./build.sh status  build/gd32f303cc_imu_to_dxl_slot_b.ipkg    # 需不需要升级？
./build.sh upgrade build/gd32f303cc_imu_to_dxl_slot_b.ipkg --yes
./host/upgrade.py verify <pkg>                                # 只校验包

# 6) 打开上位机（浏览器访问 http://127.0.0.1:8081）
./build.sh host
```

可选编译项（`EXTRA_CMAKE_ARGS` 或 `-D` 传入）：

| 变量 | 默认 | 说明 |
|---|---|---|
| `APP_SLOT` | 0 | 0 = 链接到 0x08000000（开发板，无 bootloader）；1/2 = A/B slot（并设置 VTOR） |
| `BOOT_ENABLE` | 1 | 1 = 同时构建 bootloader 与 A/B 两个 slot 镜像 |
| `APP_VERSION_STR` | 1.0.0 | 打进镜像头与升级包的版本号（major.minor.patch） |
| `DBG_ENABLE` | 1 | 0 = 完全去掉调试输出代码 |
| `BUS_MIRROR_ENABLE` | 0 | **默认 0**：USART0 只做调试口（只输出日志）。1 = 台架选项（开发板尚未接电机总线时让 USART0 兼作协议口），升级会话仍只在电机总线上开放 |
| `IMU_USE_SPI` | 1 | 1 = 真 LSM6DSV16X（`src/imu_spi.{c,h}` + `src/lsm6dsv16x.h`）；0 = 模拟器（`src/imu_sim.c`） |
| `STATUS_LED_ENABLE` | 1 | 0 = 无 PB5 心跳灯 |
| `CFG_FLASH_ENABLE` | 1 | 0 = 不写 Flash（配置只存 RAM） |
| `TEMP_SENSOR_ENABLE` | 1 | 0 = 不读片内温度（固定 30 °C） |
| `SYS_CLK_SOURCE` | HXTAL | IRC8M 可绕过晶振问题 |
| `SYS_CLK_HXTAL_HZ` | 12000000 | 晶振频率，同时定义 `HXTAL_VALUE` |

当前占用（Release，`-Os`，`IMU_USE_SPI=1` 真机构建，2026-09-25 实测）：

| 目标 | FLASH | RAM |
|---|---|---|
| 应用（开发镜像，`APP_SLOT=0`） | **24036 B / 254 KB** | **8288 B / 48896 B (16.95%)** |
| 应用（slot 镜像，96 KB 槽） | **24600 B / 96 KB (25.0%)** | 8288 B |
| **bootloader** | **18764 B / 32 KB (57.3%)** | 8288 B + 8 B（`.boot_ram`） |

（以上为 2026-09-25 `./build.sh` 构建（`IMU_USE_SPI=1`）的实测大小；RAM 百分比按链接脚本的
48896 B `RAM` 区算，另有顶部 256 B `BOOTRAM` 握手区。P0/P1 时序改造那一轮的旧数字是
dev 26.1 KB / slot 26.7 KB (28%) / boot 18.7 KB (58%)，见 `docs/bus_timing_borrow_plan.md` §7.5。）

比改造前各多约 1.6–1.8 KB flash、1.1 KB RAM：应答要走 DMA（每个端口一份应答缓冲 +
一份延迟缓冲）、要 µs 级时基（DWT）、要位次状态机与契约块打包，外加真 LSM6DSV16X 驱动
（SPI0 与 CONVERGE 状态）。bootloader 也带同一套（boot 模式下协议行为与应用完全一致）。

`.boot_ram` 是 SRAM 顶部 256 字节里唯一的 8 字节握手区（NOINIT，启动代码不清），
所以 RAM 区在链接脚本里被拆成 `RAM`（48896 B）+ `BOOTRAM`（256 B）两块，并有
`ASSERT(_sp <= 0x2000BF00)` 防止栈越界踩到它。

> ⚠️ J-Link 与串口设备（电机总线 `/dev/ttyACM*`、调试口 `/dev/ttyUSB*`）在受限沙箱里不可见；
> 烧录与串口测试需要在能访问 USB 的环境执行。

### 7.1 A/B 构型与现场升级（一句话版）

量产构型是 **32 KB bootloader + 两个 96 KB 应用槽（A/B）**：bootloader 校验两个槽的
镜像头（魔数/长度/entry/两段 CRC32/板号），按 `boot_slot` 与 trial 状态选一个跳转；
新镜像先以 **trial** 启动，30 s 内没有自我确认就在 3 次尝试后**自动回滚**。
升级通过电机总线完成（复用两种协议，不新增帧类型；调试口 USART0 不参与升级，见 §5；
机器人上这条总线是 `/dev/ttyS2`，升级前先 `sudo systemctl stop robotd` 让出总线），
由 `host/upgrade.py` 驱动：

```
verify  → 校验升级包（容器 CRC32 + 镜像头 + 两段 CRC32，坏包不上线）
status  → 读节点身份窗（模式/槽/版本/镜像 CRC/启动状态）+ 版本比较与提示
upgrade → VCMD_BOOT 进 bootloader → 分块写（块 CRC + 序号 + 回读）→ UPG_END 校验
          → 重启进新镜像（trial）→ 30 s 后自确认或回滚
```

完整命令、寄存器、失败模式与实测数据见 **`docs/upgrade.md`**；设计与取舍见 `docs/flash_layout.md`。

---

## 8. 上位机

`host/` 是纯本地工具，**无第三方前端依赖**（3D 用自写 WebGL，曲线/瀑布流用 canvas）：

* 3D：板卡模型按接收到的四元数实时旋转，红色 X / 绿色 Y / 蓝色 Z 轴，地面网格，
  实测重力矢量与世界竖直对比，+Z 指向的球面轨迹（看抖动），roll/pitch/yaw 数字；
* 瀑布流 1：roll/pitch/yaw 与躯干系陀螺 x/y/z 两条滚动曲线（可锁定 Y 轴）；
* 瀑布流 2：可选量的**密度热力图**（x 轴时间滚动、y 轴数值分箱、颜色=该列命中次数）；
* 统计：每轴均值/标准差/峰峰值/最小二乘漂移斜率/极值；采样率、往返时延（中位/p95）、
  采样计数连续性、stale 连续次数与最大次数、坏帧/超时、原始状态位解码；
* 控制：串口/波特率/协议/读取方式/读取长度/轮询率、暂停、CSV 录制下载，
  以及固件厂商窗口的全部设置（模拟模式/幅度/频率/偏置/上报坐标系/调试级别/保存/恢复出厂/重启）。

自检证据：`host/tools/test_server.py` 覆盖 HTTP + WebSocket 握手、坏串口报错、暂停往返；
加 `--device /dev/ttyACM0`（电机总线）后还会在真板上验证**整链路**：连接 → 100 Hz 实时样本
（所有前端字段齐全、四元数与重力单位性）→ 统计消息（采样率 100 Hz、0 超时/0 坏帧）→ 断开。
（该轮记录里的时延 2.48 ms 是台架镜像口 `/dev/ttyUSB0` 上的数字。）
前端另有桩 DOM/WebGL 测试（协议帧解析、异常帧不崩、CSV 列数、统计单元、热力图出墨、3D 矩阵数值）。

---

## 8.5 上板验证结果（GD32F303CC 开发板 + 模拟 IMU，台架镜像口 `/dev/ttyUSB0`，1 Mbps）

固件烧录后 `host/tools/bus_smoke.py` **两种协议全部通过，0 超时、0 坏帧**：

| 项目 | Dynamixel 2.0 | 飞特 SCS |
|---|---|---|
| 型号 / 固件 | 1200 / `0x10` | **`0x4D49`（'I','M'）/ 1.0** |
| 厂商窗口 | `magic=0x4D49 mode=sine amp=1000 freq=1000 frame=chip` | 同 |
| 遥测 | 20 字节块在 124，`cnt` 递增 | **56 处 15 字节契约块**（12 控制 + 计数 + 状态 + 保留 0）+ 128 处 20 字节诊断块 |
| `read` 与 `sync_read` | 各 146 / 157 个样本 | 同 |
| 持续轮询 | **365 Hz，时延中位 2.71 ms、p95 2.78 ms** | **391 Hz，中位 2.53 ms、p95 2.57 ms** |
| 超时 / 坏帧 | 0 / 0 | 0 / 0 |
| 四元数、重力单位性 | `|q|=1.00000`，`|g|=1.00000` | 同 |
| 陀螺与四元数导数一致性 | **中位 2.77°** | **中位 2.08°** |
| 采样计数连续性 | 0 次跳变 | 0 次跳变 |
| 7 种模拟模式 | static/sine/drift/noise/spin/sflp-wait/frozen 现象全部符合预期 | 同 |

（`frozen` 模式确认每次读出**完全相同的块**——这正是 `microduck` stale 检测要抓的故障；
`sflp-wait` 确认四元数字节全 0 且 `SFLP_VALID=0`。）

**P0/P1 时序改造的上板验证**（细节与数据见 `docs/bus_timing_borrow_plan.md` §7）：

| 项目 | 结果 |
|---|---|
| 位次仲裁（运行时不走的那条路） | `sync_read` 的 ID 列表里放 0/1/2/3 个前导 ID（缺席）→ 往返 +0 / +339 / +676 / +1001 µs（≈330 µs/槽，含主循环派发抖动） |
| 延迟缓冲计数 | 固件统计 `bus held` 与主机发出的 60 次 k>0 事务**完全对上** |
| 15 字节契约块 | `56/15` 读回 12 控制 + 计数 + 状态，`byte14 == 0`，且计数与 128 处的诊断块一致、逐帧递增 |
| 中断 + DMA 应答 | 两种协议各自 2555 次轮询、**0 超时 0 坏帧**；Dynamixel 侧 40 个被字节填充的帧全部正确解出 |
| 间隙判据 / 回声门 | `dropped=0`、`echo` 计数正常；空闲间隔不再被误记成 `gaps` |
| 调试口与总线共用 UART0 | 有协议帧要发时日志行在字节边界让路（`uart_port_write` 逐字节检查），协议帧不会被文本插断 |

另外两项针对性的上板回归：

* **Dynamixel 2.0 字节填充在真实链路上验证**：把陀螺偏置设成让原始计数恰好出现
  `FF FF FD`（`gyro_bias=(-1,-3,0)` + static 模式），60 次读取中 **40 帧确实被填充**，
  40 帧全部被主机按 rustypot 的规则正确解填充并解出原始计数。
  （如果不做填充，microduck 侧会解析错位——这是真机会踩的坑。）
* **飞特设备语义**：寄存器 8（应答状态级别）默认 1；写 0 后写指令执行但静默、读指令仍应答；
  寄存器 55（锁标志）加锁后写入仍生效于 RAM（`register 24 = 123` 读回）。

上板过程发现并修好的三个真问题（都已回归）：
1. **厂商窗口整段写入被拒**：`STATUS`/保留字节原本被标成只读，导致 20 字节整窗口写入返回 ACCESS；
   现在这两字节可写但被忽略。
2. **协议粘滞窗口过长**：原来 1 s，主机切换协议时前 1 s 的帧被忽略；改为 100 ms（仍能挡住数据里的
   `FF FF <本机ID>` 误判），冒烟测试也加了协议切换前的静默等待。
3. **配置存 Flash 会打断总线**：擦页让内核停顿几十毫秒，机器人持续轮询时会读超时；
   现在自动保存要等"写入平静 500 ms + 总线安静 1.5 s"，显式保存才立即执行。

## 8.6 真硬件验证（真 PCBA + 真 LSM6DSV16X + 15 颗舵机，2026-09-25）

`IMU_USE_SPI=1` 构建，`flash-pair a` 烧 bootloader + slot A；节点 ID 200 经 `/dev/ttyACM1`
上真总线；`BUS_PORT=/dev/ttyACM1 ./build.sh smoke` 与
`host/tools/py.sh host/tools/sensor_probe.py --port /dev/ttyACM1` 全过。

| 项目 | 实测值 |
|---|---|
| LSM6DSV16X 身份 / SPI | `WHO_AM_I` = **0x70**，SPI0 **mode 0 @ 7.5 MHz**（`SPI_PSC_16`，APB2 = 120 MHz） |
| 量程 / 速率 | 陀螺 ±500 dps @ 120 Hz 高性能，加速度 ±4 g @ 120 Hz，SFLP game rotation vector @ 120 Hz |
| FIFO | continuous，只装 SFLP 字，水位 12 字 |
| 静止加速度模长 | **996–997 mg**（300 样本均值 997.4 mg） |
| 四元数范数 | **1.0000**（300/300） |
| 四元数重力 vs 加速度方向 | **0.05–0.12°**（mean 0.058 / max 0.115） |
| 陀螺零偏 / 每轴噪声 std | **(−0.97, +0.16, +0.73) dps** / **0.017–0.031 dps**；`gyro_bias` = **(+56, −9, −42) raw counts** |
| SFLP 重启 | `VCMD_RESET_SIM` 后 **9–13 ms** 四元数归零，**1.51–1.52 s** 后恢复非零 |
| 真驱动冒烟 | Dynamixel **1260 Hz / 3851 次轮询**（中位 0.75 ms）、FeeTech **1789 Hz / 5372 次轮询**（中位 0.52 ms），**0 超时 / 0 坏帧** |
| 与 15 颗舵机共总线 | 16 ID（200 + 10–14/20–24/30–34）一次 `sync_read`（地址 56、长 15）：**150/150 扭矩关、150/150 扭矩开、200/200 背靠背**；往返中位 8.74 / p95 8.86 / max 8.95 ms |
| 节点侧 | 每秒约 47/50 次读被应答，`echo=0 dropped=0 err=0` |

**真机联调修掉的三个固件缺陷**（现象/根因/修复/证据见 `docs/bus_timing_borrow_plan.md` §4.6）：
`uart_port.c` 的固定回声跳过吃掉下一帧帧头（改成时间窗口）、`us_time.c` 的 `us_now()`
每 35.8 s 回绕（SysTick 重锁存，真 modulo-2^32）、`imu_spi.c` 的 SFLP 重启发布未收敛姿态
（新增 CONVERGE 状态，1500 ms）。


## 9. 目录结构

```
v1/
├── CMakeLists.txt / build.sh          # 构建、烧录、测试入口
├── cmake/gd32_toolchain.cmake         # arm-none-eabi 交叉工具链
├── linker/gd32f303cc_{app,slot_a,slot_b}.ld   # slot 脚本含 .app_header + .boot_ram
├── linker/gd32f303cc_boot.ld          # bootloader（32 KB @ 0x08000000）
├── scripts/jlink_*.jlink
├── src/
│   ├── board.h                         # 引脚、时钟、协议/寄存器常量、编译开关
│   ├── main.c                          # 主循环（非阻塞）、心跳、统计、重启
│   ├── system_gd32f30x.c / systick.*    # 时钟（12 MHz 晶振）与 1 kHz 节拍
│   ├── gd32f30x_it.c                   # SysTick + USART0/1 接收 + TX DMA 完成中断
│   ├── uart_port.*                     # USART：中断收字节（回调给 bus）+ DMA 发送
│   ├── us_time.*                       # µs 时基（DWT 周期计数器）
│   ├── bus_arb.*                       # sync_read 位次仲裁（纯逻辑，可主机测试）
│   ├── telem_pack.*                    # FeeTech 15 字节契约块打包（纯逻辑）
│   ├── critical.h                      # 极短临界区（发布寄存器镜像用）
│   ├── dbg.*                           # 调试口（宏可关、级别可调、防插帧）
│   ├── bus.*                           # 两个端口 + 协议自适应 + 粘滞 + 响应
│   ├── dxl2.*                          # Dynamixel 2.0 从站
│   ├── fee.*                           # 飞特 SCS/HLS 从站
│   ├── crc16.* / stuffing.*            # CRC-16 与 2.0 字节填充
│   ├── dev.*                           # 寄存器镜像、配置持久化、片内温度
│   ├── imu.* / imu_sim.*               # 传感器抽象 + 模拟器（IMU_USE_SPI=0）
│   ├── imu_spi.* / lsm6dsv16x.h        # 真 LSM6DSV16X 驱动（SPI0，IMU_USE_SPI=1）
│   ├── imu_math.*                      # 四元数/半精度工具
│   ├── app_header.*                    # 64 字节应用头（打包工具与 bootloader 共用）
│   ├── boot_crc32.*                    # 按位 CRC32（与 zlib/Python 一致）
│   ├── boot_flash.*                    # flash 抽象；主机测试用 256 KB RAM 后备
│   ├── boot_upgrade.*                  # 总线升级状态机（块 CRC/序号/整镜像 CRC）
│   ├── boot_regs.*                     # boot 模式寄存器空间（复用 dev_* API）
│   ├── boot_decide.*                   # 启动决策（纯函数，可主机测试）
│   ├── boot_ram.*                      # SRAM 握手字（NOINIT，跨软复位）
│   ├── boot_main.c                     # bootloader 主循环、跳转、boot 模式服务
│   ├── cfg_blob.*                      # 配置页 v2（设备配置 + 启动状态）
│   └── dev_cfg.c                       # 工厂默认值（应用与 bootloader 共用）
├── host/                              # 上位机（server.py + web/ + bus.py + package.py + upgrade.py）
├── host/tools/                        # C/Python 测试、boot_node_sim、servo_probe、sensor_probe、py.sh、run_c_tests.sh
├── docs/                              # 协议、Flash/boot、升级、降本路线文档
├── patches/                           # 飞特模块与 microduck 补丁
└── docs/refs/                          # 参考实现（rustypot 源码、XL330 控制表）
```

