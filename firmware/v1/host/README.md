# imu_to_dxl 上位机测试软件

本目录是 `imu_to_dxl` 节点的 PC 侧工具：**本地网页版**上位机（3D 姿态 + 实时瀑布流 +
抖动/偏移统计），外加两个命令行测试工具。前端零第三方依赖（自写 WebGL + canvas），
不需要联网、不需要构建步骤。

```
host/
├── bus.py               # 协议编解码 + 串口链路（Dynamixel 2.0 / 飞特），可单独自检
├── server.py            # aiohttp 服务：静态页面 + /ws 实时推送 + /api/ports
├── web/
│   ├── index.html       # 单页界面
│   ├── app.js           # WebGL 3D + canvas 曲线/瀑布流 + 统计（约 1800 行，无依赖）
│   └── style.css
└── tools/
    ├── bus_smoke.py     # 上板协议冒烟测试（两种协议 + 全部模拟模式）
    ├── fast_sync_read_check.py  # 上板 0x8A 验收（第 0 位 / 第 1 位 / 未点名）
    ├── rustypot_check/  # 与官方 rustypot 1.8 解析器对拍 0x8A（需要 Rust）
    ├── scope_traffic.py # 示波器用：单一已知帧、固定速率地重复（见第 4 节）
    ├── test_server.py   # HTTP + WebSocket 集成自检（不需要硬件）
    ├── test_protocols.c # 两个协议从站 + 模拟器的宿主单元测试（纯 gcc）
    ├── test_crc16.c     # CRC 与 rustypot/ROBOTIS 参考向量比对
    └── run_c_tests.sh   # 以上两个 C 测试的入口
```

## 1. 依赖

只需要 `pyserial` 与 `aiohttp`（Python 3.10+）：

```bash
# 在v1目录下执行
uv venv --python 3.12 
source .venv/bin/activate
uv pip install pyserial aiohttp
./build.sh host

# 已安装，可直接用：
source .venv/bin/activate
./build.sh host
```

C 测试只需要 `gcc`；协议自检 `bus.py` 在缺 pyserial 时也能跑（只有实链路部分需要它）。
`tools/rustypot_check/` 额外需要 Rust 工具链与 crates.io 缓存（它跑的是真的 rustypot 1.8，
不是它的复制品），因此**不**在 `./build.sh test` 里，改 `0x8A` 路径后手动执行：

```bash
cd tools/rustypot_check && cargo run --release        # 打印 5 项断言结果
```

## 2. 使用

```bash
cd v1

# 打开上位机（浏览器访问 http://127.0.0.1:8081；页面里再选串口/协议/波特率）
./build.sh host
# 等价于：
../.venv/bin/python host/server.py --port /dev/ttyACM0 --baud 1000000 --protocol dxl

# 先不上板、只验证工具链
../.venv/bin/python host/bus.py                  # 协议向量自检
../.venv/bin/python host/tools/test_server.py    # HTTP/WS 自检
host/tools/run_c_tests.sh                        # 固件源码的宿主单元测试

# 上板协议冒烟测试（会遍历两种协议与 7 种模拟模式，并还原配置）
./build.sh smoke
BUS_PORT=/dev/ttyACM1 BUS_BAUD=1000000 ./build.sh smoke   # 换端口（默认 /dev/ttyACM0）
```

`/dev/ttyACM0` 是**电机总线**：本节点和 15 颗舵机挂在同一条单线总线上，上位机、
冒烟测试、升级工具都走它。ACM 编号跟着适配器枚举走，换 USB 口/插别的设备后先用
`ls /dev/ttyACM*` 确认；机器人上的总线是 SoC 串口 `/dev/ttyS2`，而升级前要先停 robotd
（它独占总线，工具会拒绝和别人抢端口，并打印 stop/upgrade/start 提示）。升级工具的
`--port auto`（默认）会**逐个探测** `ttyACM0-1`、`ttyUSB0-5`、`ttyS2` 里存在的设备，
取第一个真能应答节点的那个——所以 USB-TTL 上的总线适配器也能被自动找到。
**调试口（USART0，通常是 CH340 的 `/dev/ttyUSB*`）默认只说 DBG_ 日志**，不解析协议帧，
探测它只会得到"没有应答"。

服务默认只监听 `127.0.0.1:8081`（`--host 0.0.0.0` 可对外，注意这是无鉴权的调试工具）。

## 3. 界面说明

| 面板 | 内容 |
|---|---|
| 控制栏 | 串口下拉（来自 `/api/ports`）、波特率、协议（Dynamixel 2.0 / 飞特）、读取方式（`sync_read` / 单条 `read`）、读取长度（12 = 控制块，20 = 含诊断尾部）、轮询率、连接/断开、暂停、连接状态与型号/固件版本 |
| 板卡设置 | 模拟模式、幅度、频率、陀螺零偏（原始计数）、上报坐标系（芯片/躯干）、调试级别，以及"应用 / 保存到 Flash / 恢复出厂 / 重启 / 重启运动模型" |
| 3D 视图 | 板卡模型按姿态旋转、XYZ 轴、地面网格、实测重力 vs 世界竖直、+Z 球面轨迹、roll/pitch/yaw |
| 瀑布流（曲线） | roll/pitch/yaw 与**躯干系**陀螺 x/y/z 两条滚动曲线，可锁定 Y 轴 |
| 瀑布流（热力图） | 可选量（gyro_x/y/z、gravity_x/y、roll、pitch）的时间-数值密度图：颜色 = 该时间列落在该数值箱的样本数，用来一眼看出抖动带宽与偏移量 |
| 统计 | 每轴均值/标准差/峰峰值/最小二乘漂移斜率/极值；采样率、时延中位/p95、采样计数连续性、stale 当前/最大、坏帧/超时、状态位解码；CSV 录制下载 |

关键概念：

* **抖动**看标准差/峰峰值与热力图的"墨迹厚度"；
* **偏移**看均值与漂移斜率（`drift` 模式配合注入的陀螺零偏很容易复现）；
* **stale** 指节点连续返回**完全相同**的 12 字节块（`microduck` 用它发现"总线有应答但姿态冻结"），
  固件的 `SIM_MODE=6 frozen` 专门制造这种情况；
* 陀螺与重力都是**躯干系**；如果固件被设成 `REPORT_FRAME=1`（躯干系上报），
  上位机会按配置自动不再做安装矩阵变换（读厂商窗口的该字段）。

## 4. 命令行工具

### `bus_smoke.py`（上板）

按协议逐项检查：型号/固件寄存器、PING、厂商窗口、20 字节诊断块（飞特在 128）、
**共享块**（飞特 56/15 的契约块：计数与状态在 12/13、byte14 为 0、计数递增且与 128 一致）、
`read` 与 `sync_read` 两种取数、持续轮询（采样率/时延 p95/超时/坏帧）、陀螺范围、
四元数与重力单位性、采样计数连续性、**陀螺与四元数导数一致性**（这是最有价值的端到端检查），
然后逐个切换 7 种模拟模式验证各自应有的现象，最后还原原配置。退出码非 0 表示有失败项。

### `scope_traffic.py`（示波器，只读；`write` 模式除外）

要把总线波形打上示波器，需要的是**一个已知帧、按固定速率无限重复**——`servo_probe.py` 是固定
序列、`bus_smoke.py` 是判 pass/fail、`robotd` 是每 tick 都不同的 50 Hz 混合流量，都不合适。

```bash
# 最小帧：对一颗舵机 PING，20 Hz，一直发（Ctrl-C 停）
tools/py.sh tools/scope_traffic.py --port /dev/ttyACM0 --mode ping --id 20 --hz 20
# 机器人真实的一个 tick：广播 SYNC_READ 全部 16 个设备（含 IMU 节点）
tools/py.sh tools/scope_traffic.py --port /dev/ttyACM0 --mode sync --hz 20
# 启动时的短读：15 个关节、每台 2 字节
tools/py.sh tools/scope_traffic.py --port /dev/ttyACM0 --mode sync-pos --hz 20
# 广播 SYNC_WRITE：把每台舵机写回它**当前**位置（帧是真的，但不会动）
tools/py.sh tools/scope_traffic.py --port /dev/ttyACM0 --mode write --hz 5 --confirm-motion-safe
# 什么都不发，用来量空闲电平和上拉
tools/py.sh tools/scope_traffic.py --port /dev/ttyACM0 --mode idle
```

启动时它会打印一份**时序速查表**（1 Mbps、8N1：1 bit = 1 µs，1 字节 = 10 µs，空闲高电平）：
PING 请求/应答各 6 字节（60 µs，应答在请求结束后约 0.5 ms）；`SYNC_READ` 16 个 id 是 24 字节
（240 µs）请求，每台回 21 字节（210 µs），槽长约 295 µs、按 id 顺序应答，整串到最后一台约
**4.9 ms**；`SYNC_WRITE` 15 台是 53 字节（530 µs）、广播不应答。

探哪里：任意舵机接头的**数据线 + GND**（半双工单线，空闲为高）。触发取第一个起始位的下降沿；
20 µs/div 看两个字节，500 µs/div 看完整一串。要量"请求结束 → 第一台应答"的转向时间，
以及"应答之间"的槽间距。

两点注意：**跑之前确认端口只有它一个占用者**（`fuser /dev/ttyACM0`；robotd 用 TIOCEXCL
打开，但 TIOCEXCL 不会拒绝已经打开的 fd，第二个进程会偷走一半字节）；脚本的 `--hz` 是
主机侧 sleep 节拍（毫秒级抖动，示波器触发无所谓），而**总线上每一帧的位宽是硬件时钟**，
那才是要量的东西。

### `servo_probe.py`（真舵机，只读）

对一颗真的飞特舵机做**只读**调查：身份/底账（固件版本、END、型号、模式、P/E 增益、锁）、
地址 56 的 12/15/20/21 字节读、`sync_read` 里前导 ID 缺席时的应答槽长（线性 ≈295 µs/个）、
坏校验帧是否应答。用途见 `docs/bus_timing_borrow_plan.md`；它**不会写任何寄存器**。

### `servo_reboot_probe.py`（真舵机，会重启舵机）

实测 `0x08` REBOOT：不写 EEPROM、不使能扭矩、不带载；只把 **RAM** 增益 50/51 写成可辨识的值
（之后还原），重启后再读一遍全部状态，用来确认"不应答 / 约 800 ms 回来 / EEPROM 设置保留 /
RAM 增益回落到 EEPROM"。输出即 `docs/bus_timing_borrow_plan.md` §1.7 的数据。

### `test_protocols.c`（宿主）

把真实固件源码（`dxl2.c`、`fee.c`、`imu_sim.c`、`crc16.c`、`stuffing.c`）用 gcc
配一个桩寄存器层编译，断言：

* 与 rustypot 逐字节相同的响应帧（PING/SYNC_READ/READ 的完整字节序列）；
* CRC 错误、越界、不支持指令的错误位；
* `FF FF FD` 字节填充与指令方向解填充；
* 飞特 PING/READ/SYNC_READ/SYNC_WRITE 的完整字节序列与校验和；
* 半精度编码、安装矩阵往返、陀螺尺度、模拟器模式（sine 的陀螺/四元数一致性、
  static 的重力、sflp-wait 的全 0、frozen 的完全相同块）。

### 前端自检

`test_server.py` 覆盖服务端；前端在开发时用桩 DOM/WebGL/WebSocket 驱动真实 `app.js`
（协议解析、异常帧、CSV、统计单元、热力图出墨、3D 矩阵数值），该桩脚本不在仓库内。

## 5. 排错

| 现象 | 处理 |
|---|---|
| 打不开串口 | 确认**电机总线**口存在（通常 `/dev/ttyACM0`，换 USB 口后先 `ls /dev/ttyACM*` 确认编号）、当前用户有权限（`dialout` 组）、没有别的程序占用；`/dev/ttyUSB*` 是 CH340 调试口，**不是总线**，连节点不能用它；受限沙箱里 `/dev` 可能不可见，需要在宿主机执行 |
| 一直 timeout | 端口选错 / 波特率不对（默认 1 Mbps）/ 协议选错（节点会自动识别，但主机得先发对一种）；用 `DBG_LEVEL=3` 看节点是否收到字节 |
| 有大量坏帧 | 检查是否与调试文本混流（文本行以 `#` 开头，帧解析会跳过）、线缆质量、1 Mbps 下 USB-TTL 是否支持 |
| 数据不动 | 节点可能处于 `SIM_MODE=6 frozen`（这是故意用来测 stale 的）或 `5 sflp-wait`；改成 `sine` 看 |
| 3D 姿态方向不对 | 确认 `REPORT_FRAME` 与界面假设一致（默认芯片系）、安装角是否为官方 `绕 Y +90°` |
