# HLS 舵机浏览器调试工具

这是一个面向飞特 HLS 系列串行总线舵机的浏览器调试软件，放在
`飞特通讯协议/hls_servo_debugger` 下。后端使用 Python + Flask + pyserial，
前端为纯 HTML/CSS/JavaScript，无需前端构建工具。

工具以 `FTServo_Linux-main/src/HLSCL.cpp`、`SCS.cpp` 和官方
《磁编码 HLS 舵机-内存表解析》为依据，覆盖 HLS 舵机的指令、内存表、控制模式、
同步读写和原始协议调试。

## 功能范围

- 串口连接：`/dev/ttyACM1` 等设备选择、常用波特率、超时设置、自动探测 ID1。
- PING / 扫描：单个 Ping、按 ID 范围扫描、可读取版本。
- 实时反馈：一次读取 56~70 共 15 字节，显示位置/速度/负载/电压/温度/状态/移动/目标位置/电流，并绘制趋势图。
- 位置控制：位置伺服模式、扭矩开关、目标位置、速度、加速度、目标电流、转矩限制、角度限制、异步写（REG_WRITE）+ REG_ACTION、中位校准。
- 速度/电流/PWM：恒速模式、恒流模式、PWM 开环模式，以及对应写入方法。
- 参数配置：按 HLS 内存表自动生成 0~86 字段的读取/写入界面，支持枚举、位域、正负号字段编码。
- 同步读写：SYNC_READ(0x82)、SYNC_WRITE(0x83)、同步位置写、同步速度写。
- 高级调试：任意地址/长度 READ/WRITE、REG_WRITE、REG_ACTION、RESET、RECOVERY、CAL、写锁控制、原始十六进制帧收发。
- 日志：显示 TX/RX/错误帧，便于核对校验和和异常。
- 内存表参考：完整字段、地址、范围、单位、方向位说明。

HLS 参考库中的 `WritePosEx`、`RegWritePosEx`、`SyncWritePosEx`、
`SyncWriteSpe`、`ServoMode`、`WheelMode`、`EleMode`、`WriteSpe`、
`WriteEle`、`EnableTorque`、`unLockEprom`、`LockEprom`、
`CalibrationOfs`、`FeedBack`、`ReadPos/ReadSpeed/...` 均有对应前端操作或
API 入口。

## 目录结构

```text
hls_servo_debugger/
├── pyproject.toml
├── README.md
├── run.sh
├── hls_debugger/
│   ├── __init__.py
│   ├── __main__.py
│   ├── protocol.py        # FT-SCS/HLS 协议、串口收发、同步读写、应用层指令
│   ├── memory_table.py    # HLS 内存表、数据编解码、反馈解码
│   ├── server.py          # Flask API 和 Web 服务入口
│   └── selftest.py        # 无硬件协议自检
│   └── static/
│       ├── index.html
│       ├── style.css
│       └── app.js
├── hw_test.py             # 真机只读连通性测试（PING/READ，不产生动作）
└── .venv/                 # uv 创建的虚拟环境（本地生成）
```

## 安装与运行

### 方式一：使用 `run.sh`

```bash
cd microduck-community-kit/software/hls_servo_debugger
./run.sh
```

脚本会自动用 `uv` 创建 `.venv` 并安装 Flask、pyserial，然后启动服务。

### 方式二：手动使用 uv

```bash
cd microduck-community-kit/software/hls_servo_debugger

uv venv .venv --python python3
uv pip install 'Flask>=2.2,<3.0' 'pyserial>=3.5'

uv python -m hls_debugger
```

启动后浏览器访问：

```text
http://127.0.0.1:8765/
```

默认会自动打开浏览器；使用 `--no-browser` 可禁止自动打开：

```bash
.venv/bin/python -m hls_debugger --no-browser --port 8765
```

## 快速上手

1. 给舵机接好电源和串口总线，确认设备确实是 `/dev/ttyACM1`。
2. 打开页面，在“连接与扫描”中选择串口，波特率先选 `1000000`，点击“连接”。
3. 点击“Ping”确认 ID 1 是否在线。若失败，依次尝试 115200、500000、250000、57600、38400，或使用“自动探测 ID1”。
4. 点开“实时反馈”，点击“读取一次/开始轮询”，观察位置、速度、电压、温度、电流。
5. 点开“位置控制”，先“打开扭矩”，确认机械安全后写目标位置。
   初次测试建议：
   - 目标位置：`2048`（约 180°）或接近当前位置；
   - 速度：`60`（约 43.9 RPM）；
   - 加速度：`10~30`；
   - 目标电流：从 `200`（约 1.3 A）左右开始，根据负载调整。
6. 修改 ID、波特率、角度限制等 EPROM 参数前，建议先关闭扭矩，必要时关闭写入锁。
   修改 ID 后需要重新连接/扫描新 ID；修改波特率后需要重新连接新波特率。

## 内存表关键换算

| 字段 | 换算 |
|---|---|
| 位置 42/56/67 | 1 计数 = 0.087°，4096 计数/圈，BIT15 方向 |
| 速度 46/58 | 1 计数 = 0.732 RPM，BIT15 方向 |
| 负载 60 | 1 计数 = 0.1% 占空比，BIT10 方向 |
| 电压 62 | 1 计数 = 0.1 V |
| 电流 44/69 | 1 计数 = 6.5 mA，BIT15 方向 |
| 加速度 41 | 1 计数 = 8.7 °/s²，0 = 最大 |
| 转矩 16/48 | 1 计数 = 0.1% |
| Kp/Kd 50/51 | 位置模式 Kp 刻度 1/8，Kd 刻度 1/4 |

特别提醒：HLS 位置是 **4096 计数/圈**，不是 32768 计数/圈。
早期资料若按 32768 计算会有 8 倍误差。

## 与参考库的对应关系

| 参考库方法 | 工具入口 | HTTP API |
|---|---|---|
| `Ping(ID)` | Ping / 扫描 | `POST /api/ping` |
| `Read/readByte/readWord` | 通用读、参数配置、实时反馈 | `POST /api/read` |
| `genWrite/writeByte/writeWord` | 通用写、参数配置 | `POST /api/write`、`POST /api/write_field` |
| `regWrite` | 异步写 | `POST /api/reg_write` |
| `RegWriteAction` | 执行异步写 | `POST /api/reg_action` |
| `WritePosEx` | 位置控制 | `POST /api/move` |
| `RegWritePosEx` | 异步写位置 | `POST /api/move` (`reg=true`) |
| `SyncWritePosEx` | 同步位置写 | `POST /api/sync_move` |
| `SyncWriteSpe` | 同步速度写 | `POST /api/sync_wheel` |
| `ServoMode/WheelMode/EleMode` | 模式按钮 | `POST /api/mode` |
| `WriteSpe` | 恒速模式 | `POST /api/wheel` |
| `WriteEle` | 恒流模式 | `POST /api/electric` |
| `EnableTorque` | 扭矩开关 | `POST /api/torque` |
| `unLockEprom/LockEprom` | 写锁控制 | `POST /api/lock` |
| `CalibrationOfs` | 中位校准 | `POST /api/calibrate` |
| `FeedBack` | 实时反馈 | `POST /api/feedback` |
| `Reset` | RESET | `POST /api/reset` |
| `Recal` | CAL | `POST /api/calibrate` |
| 原始协议 | 原始帧收发 | `POST /api/raw` |

## 故障排查

- **爆出“打开串口失败”**：检查设备名、USB 线、供电、`dialout` 权限。
- **一直等待应答超时**：优先检查波特率是否匹配；HLS 默认 1 Mbps。
- **Ping 收到但读写失败**：确认 ID 是否为总线地址/副 ID；确认目标设备不是只响应主 ID。
- **写 EPROM 不保存**：写 55 号锁标志为 0（关闭写入锁）后再写，然后重新上电验证。
- **修改 ID/波特率后连不上**：换用新 ID/新波特率重新连接。
- **同步写没反应**：SYNC_WRITE 是广播，没有应答；先确认舵机处于正确模式，且每个 ID 的数据长度一致。
- **位置角度不对**：按 4096 计数/圈换算；扭矩关掉后位置可能可以手动转动，这是正常现象。
- **扫描少发现舵机（已修）**：早期扫描范围默认只到 ID 20，把 ID>20 的舵机整段漏掉
  （实测 10~14、20、21 共七台只报 6 台，漏的正是 21）。现已把界面、`/api/scan`、
  `HLSBus.scan()` 的默认上限全部改为 **253**，默认全总线扫描约 12 秒。
  注意扫描区间是**闭区间**：结束 ID 必须 ≥ 最大舵机 ID。

## 安全提示

调试软件可以通过浏览器直接驱动舵机。所有写入都会立即发送到总线，
请务必：
- 固定好机械结构，避免堵转、飞车或夹伤；
- 初次使用低速度、低加速度、低电流；
- 随时准备断开舵机电源；
- **注意：写目标位置会自动打开扭矩**（实测本固件行为），所以"关闭扭矩"后
  再写位置会重新带电；需要卸力时请在最后一次位置写入之后再点"关闭扭矩"；
- 不要把 `RESET`、`CAL`、改 ID/波特率等操作当成常规测试按钮反复点击。

## 无硬件自检

不连接舵机也可以运行协议自检，验证帧构造、校验和、串口读取状态机、
同步读解析、内存表编解码和反馈解码：

```bash
uv python -m hls_debugger.selftest
```

## 真机连通性测试

`hw_test.py` 用真实串口完成只读验证（只发 PING 和 READ，不发任何运动指令）：

```bash
uv python hw_test.py --port /dev/ttyACM1
```

它会打印设备信息、各波特率下的 ID 扫描、版本/ID/波特率/模式/扭矩/写锁，
以及 56~70 号 15 字节实时反馈的完整解码。若首个波特率无应答，会自动遍历
`1000000 / 115200 / 500000 / 250000 / 128000 / 76800 / 57600 / 38400`。


### 现场注意

- 波特率索引 0 = 1 Mbps，与本工具默认一致；改波特率后需按新速率重连。
- 锁标志为 1 时写 EPROM 掉电不保存，**改 ID/波特率等需先解写锁（55 号写 0）**。

