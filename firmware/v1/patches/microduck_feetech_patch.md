# microduck 电机总线：Dynamixel XL330 → FeeTech SCS/HLS 补丁说明

本补丁把 `duck-control` 的电机总线从 Dynamixel XL330（`rustypot`）切换到 FeeTech
SCS/HLS（HD-1910-C001），复用已经写好并测试过的 `feetech` 协议模块（v2 按厂商内存表
修正后逐字节复制，未改其一行代码）。

- 验证命令（在 worktree 中，设置 `CARGO_HOME` 到工作区内以绕过只读的 `~/.cargo`）：
  `cargo test -p duck-control`
- 结果：`test result: ok. 93 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 0.03s`
- `cargo check -p duck-control -p robotd` 通过、无新增 warning；补丁 `git apply --check` 可干净应用到
  `HEAD`，把补丁应用到干净副本后 `duck-control/src/bus.rs` 与本目录编辑结果逐字节一致。
- `git diff --stat`：12 files changed, 1800 insertions(+), 380 deletions(-)
  （其中 `duck-control/src/feetech.rs` 为原样新增 520 行，SHA-256 与
  `v1/patches/feetech.rs` 逐字节一致：
  `583acdad93116655f203ef72b2c02379601d3571958bbb3429e76d6a9b5e1137`；
  本次 `reboot()` 实现只改 `duck-control/src/bus.rs`（含单测）与 `io.rs` 的文档）。

## 一、逐文件改动

1. **`duck-control/src/feetech.rs`（新增）**
   从 `v1/patches/feetech.rs` 原样复制，未改一字。提供帧构造/解析、
   `FrameReader`、`reg` 寄存器地址、换算常量与 14 个单测。

2. **`duck-control/src/lib.rs`**
   在 `fall` 与 `imu` 之间加入 `pub mod feetech;`，与既有模块列表同风格。

3. **`duck-control/src/bus.rs`（核心重写，保留原文档语气与结构）**
   - `DynamixelIo` → `FeetechIo`；`open` 直接使用 `serialport`，不再经过 `rustypot`。
   - `Xl330Controller` 调用全部替换为 `feetech::*` + 手写收发。
   - 新增内部 `Transport` trait（`Send`，`write_all/read/flush`）与 `SerialTransport`
     包装真实串口。这是为了让 `set_torque`、乱序收包等失败路径在没有硬件的单测里可驱动。
   - 每个 tick 一次 `sync_read([IMU_ID, JOINT_IDS...], reg::PRESENT_POSITION_L=56, len=15)`。
   - **乱序收包与故障汇总**：`drain_replies()` 按 ack 里的 id 把回复放回请求顺序，而不是假设到达顺序。
     这是相对 Dynamixel 版本最重要的正确性修正。**非零 ack status 只记录该 id 与状态码，不再中断收集**：
     一台舵机报错不会掩盖其余 14 台（重复帧既不覆盖槽位也不重复计数；畸形/校验错帧、未知 id、长度
     不符请求仍按原规则跳过/报错）。`sync_read_blocks()` 在所有被请求 id 都已“有答复”（数据或故障）时
     立即结束，否则等到线路空闲 `BURST_IDLE=2 ms` 为止——它同时就是串口读超时，一个回复时隙即使设备
     缺席也约占 295 µs，2 ms ≈ 6 个时隙裕量，因此单台缺席只付一次 2 ms 而不是旧的 30 ms。收集结束后，
     若仍有 id 缺失或有非零状态，返回**同一条**描述性 `Err`（如
     `sync_read: id(s) 11 faulted status 0x08; id(s) 20 missing`）。故障仍然让该 tick 失败是刻意的：
     上游 `robotd` 会 `coast.sample(None)` 沿用上一份好样本并累加 `consecutive_errors`；本补丁只去掉
     30 ms 卡顿与中途 abort，**不**改成“用旧值顶替并返回 Ok”。`READ_TIMEOUT=30 ms` 现在只作为单台
     寻址命令（torque/gain/寄存器读写）的预算，由 `exchange()` 轮询耗尽。
   - `assemble_tick()`：slot 0 为 IMU 节点，slot 1.. 为 `JOINT_IDS` 顺序的舵机；纯函数，便于单测。
   - `imu_block_from_slot()`：只取 slot 0 的前 12 字节交给 `SflpDecoder`。IMU 节点在 FeeTech
     人格下同一地址（56）回答的是 **15 字节契约块**（2026-09-24 起）：前 12 字节是 SFLP 控制块、
     12 是采样计数、13 是状态位、14 保留 0——与真舵机在 56..70 的块长一致，因为一次 `sync_read`
     的读长对所有设备必须相同。原始加速度在 **128** 的 20 字节诊断块里，不属于本 tick 的读长
     （本 tick 读 15 字节，`imu_block_from_slot()` 只取前 12）。
   - `decode_servo_block()`：解析 15 字节舵机块（见下表），负载字段刻意不解码——`Sensors`
     没有 load 字段，不发明新字段。
   - `write()`：`sync_write` 到 `GOAL_POSITION_L=42`，每舵机 2 字节，由
     `feetech::position_from_rad`（符号-幅值）生成；广播写不产生 ack，因此不等待。
   - `set_torque()`：逐 id `write` 到 `TORQUE_ENABLE=40`，每台都写、失败最后汇总上报，保留
     原行为与注释；文档说明 FeeTech 总线对每条指令都回 ack，Dynamixel 的 `return_delay_time`
     预算问题不再适用。
   - `check_registers()`：只读并纠正 `reg::ID=5`（应与 `JOINT_IDS` 一致）与
     `reg::BAUD_RATE=6`（应为 0 = 1 Mbps）；不再断言 XL330 的
     `return_delay_time/pwm_slope/shutdown`。
   - `present_positions()`：一次 15 台、长度 2 的 `sync_read`（启动时只取位置，读更短、突发更短）。
   - `set_gain()`：**实写 RAM 增益寄存器**（v2 已解决旧版 no-op）：
     - 每关节写 **RAM 50（`Kp`，0x32）** 一个字节，`clamp(kp, 0, 254)`，`kp` 原样通过、
       不做换算（厂商表注明比例系数 1/8）。
     - 同时写 **RAM 51（`Kd`，0x33）= 0**：出厂 D 值非零，留着会在同一 `kp` 下给舵机自己的
       位置环加阻尼（与 Dynamixel 版把 D 置零同理；厂商表注明微分系数 1/4）。
     - **不写 Ki（52，0x34）**：厂商表明确位置伺服模式下 无效（属于恒速环），不为“对称”而写 0。
     - **不写 EEPROM 21/22（0x15/0x16）**：那是上电默认值，RAM 写立即生效，每次启动写 EEPROM
       只是白烧闪存；也因此 **无需处理 55（锁标志）**——该位只决定 EEPROM 写是否掉电保存。
     - 沿用 crate 既有形状：**每台都写、失败最后汇总上报**。
     - 文档明确 `deploy/robotd.toml` 的 `gain = 200` 已在该 0..254 范围的顶端，必须在真机重调。
   - `reboot()`：**真机验证后改为真正实现**（FeeTech 指令 `0x08`，厂商 STM32 HAL 的
     `INST.h` 写作 `INST_REBOOT`，其 `SCS.c::Reboot()` 也是发完即 flush、不读应答）。
     先用寻址写把该 id 的 **torque_enable(40) 写 0**，再单播一个无参数的 `0x08`，
     **不等任何应答**（本来就没有，等只会每台舵机烧掉一次 `READ_TIMEOUT` 并卡住控制环），
     帧一上线路就返回：不 sleep、不 ping。若舵机已失联，`write_register` 按周边代码的方式
     上报错误（写失败不吞）。**绝不能**把它映射到 `RESET(0x0A)`：那是恢复出厂设置，
     会清掉 id/波特率/限位；`0x08` 才是重启。
     真机实测（HD-1910-C001，固件 3.46，1 Mbps，2026-09-24）：`0x08` 存在且生效，
     舵机不回应答（请求按设计超时）；**823 ms 后**重新应答 PING（每 20 ms 轮询一次；
     厂商手册约 800 ms）。回来后 `ID`、波特率码、运行模式、扭矩状态、锁、EEPROM P/D(21/22)
     与 OFS 均不变，但 **RAM 从 EEPROM 重载**：曾把 RAM 位置增益(50/51)设为 99/88，
     重启后变回 EEPROM 的 32/40——正是 `RobotIo::reboot` 契约所说的“扭矩关闭、RAM 回到
     EEPROM 默认值，由调用方写回增益”。`robotd` 本就在 `reboot_motors` 前调用
     `set_torque(false)`，实现里再写一次 40 是为满足厂商手册的先后要求。
     实现位置：`bus.rs` 的 `REBOOT_INST = 0x08` + `impl RobotIo for FeetechIo::reboot`。
     补丁中没有任何“重启后再等待/重新采纳”的 settle 常量需要改（tick 靠读失败自然滑过这
     ~0.8 s）；`io.rs` 的 trait 文档已同步为 `0x08` 与 823 ms 的事实。
   - `slow_sensors()`：**折叠进 tick**，返回最近一次成功 tick 的电压/温度样本。
   - 单测：重写 3 个 rustypot/Dynamixel 相关测试；新增乱序重组、IMU/SFLP 与舵机块不混淆、
     电压温度折叠、`set_torque` 全写并只报静默 id、`set_gain` 命中 50/51 且 clamp、
     `reboot` 先写 torque=0 再发恰好一个 `0x08`、以及“不等应答（不触发读超时）”两个测试；
     原地保留 `StaleImuTracker` 的全部测试。

4. **`duck-control/Cargo.toml`**
   删除 `rustypot = "1.6.0"` 及其说明注释；保留 `serialport = { version = "4.8",
   default-features = false }` 与 `libudev` 的交叉编译理由注释（措辞更新）。

5. **`Cargo.lock`**
   cargo 自动移除 `rustypot` 及其依赖项（84 行删除），按“诚实”原则一并纳入补丁。

6. **`duck-control/src/model.rs`**
   `EXPECTED_REGISTERS` 表删除，替换为 `EXPECTED_BAUD_CODE = 0` 与说明（id 的期望值按关节
   逐个给，不放进表；v2 补充 HLS 波特率寄存器范围为 **0..7**）。顺带把
   `DynamixelIo::bus_voltage` 等失效引用改成 `FeetechIo`。

7. **`duck-control/src/io.rs`**
   仅文档：`Sensors` 的 IMU 来源改为“共享舵机总线”；`reboot` 说明 FeeTech 用无应答的
   `0x08`、离总线约 823 ms（厂商手册 ~800 ms）后回来、RAM 增益回到 EEPROM 默认值；
   `slow_sensors` 说明现已折叠进 tick。trait 方法签名不变。

8. **`duck-control/src/imu.rs`、`safety.rs`**
   仅注释：IMU 板“共享舵机总线”；执行器行程 ±π 的说明补上 12-bit XL330 与 15-bit
   符号-幅值 FeeTech 的差异，边界数值未改。

9. **`robotd/src/main.rs`**
   `BusIo` 类型别名与两处 `::open` 改为 `FeetechIo`，注释同步；无行为改动。

10. **`robotd-params/src/registry.rs`、`robotd/src/soc.rs`**
    仅文案：端口描述与“不碰总线”的注释改为 FeeTech/舵机总线。

## 二、FeeTech 寄存器与换算决策（块 56..70，低字节在前）

| 地址 | 字段 | 处理 |
|---|---|---|
| 56/57 | present_position | 15-bit 符号-幅值：`position_raw_to_counts(le_u16)` × `RAD_PER_COUNT`(2π/4096, 0.087°/count) |
| 58/59 | present_speed | `le_i16` × `RAD_PER_SEC_PER_SPEED_COUNT`（0.732 rpm/count） |
| 60/61 | present_load | **保留未用**（`Sensors` 无 load 字段，不发明；bit10 为方向位） |
| 62 | present_voltage | `u8` × 0.1 V/count，参与均值；0 值过滤 |
| 63 | present_temperature | `u8`，整 °C，逐关节保留 |
| 65 | 舵机状态 | 错误位：bit0 电压 / bit1 磁编码 / bit2 温度 / bit3 电流（当前未解码） |
| 66（块内偏移 10） | moving | 未使用 |
| 69/70 | present_current | `le_i16` × 6.5 mA/count，取绝对值填入 `currents_ma`（bit15 方向位） |
| 42 | goal_position | `sync_write` 2 字节/舵机，广播不 ack；单位 0.087°、bit15 方向 |
| 40 | torque_enable | 逐 id `write`，`0x01`/`0x00`，要 ack；值 2 = 阻尼输出 |
| 50 / 51 | Kp / Kd | `set_gain` 逐 id `write`，单字节；不写 Ki(52)、不写 EEPROM(21/22) |
| 5 / 6 | id / baud_rate | 启动检查；baud code 0 = 1 Mbps，寄存器范围 0..7 |

**关键决策**
- 位置必须走 `position_from_rad`/`position_rad`（符号-幅值），不能把弧度直接 `as i32` 成计数。
- **位置标度修正（v2）**：厂商内存表（`飞特通讯协议/HLS系列舵机内存表.html`
  “磁编码HLS舵机-内存表解析”§2.3/§2.4）把 `目标位置`/`当前位置` 的单位写作 **0.087°**，
  即 **4096 counts/圈**，字段为 `-32767..32767`、bit15 为方向位，因此量程是多圈（约 ±8 圈），
  不是“15 bit 一圈”。本文件旧版采用二手资料里的 **32768 counts/圈**，与厂商表矛盾：
  这会让每个关节角度差 **8 倍**。v2 已改为 4096，旧结论作废。
- 一次 15 字节能同时拿到位置/速度/负载/电压/温度/moving/电流，因此电压温度不再另开事务；
  `slow_sensors()` 改为返回最近 tick 的缓存样本（最多落后一个 tick，对电池/温度无影响；
  tick 失败时保留上一份好样本，而不是把一次丢包当成电池掉电）。
- `set_gain()` 的地址已由厂商表落定（50/51），不再是缺口；剩下的是**标度与整定**问题：
  寄存器比例 1/8、范围 0..254，而 `deploy/robotd.toml` 的增益是按 XL330 标度调的，必须真机重调。

### v2 巡检发现（本补丁现在依赖的厂商内存表事实）

- **地址 8 = 应答状态级别**：初值 1，范围 0..1。0 = 除读/PING 外不应答；1 = 对所有指令应答。
  本补丁按 1 工作：每条写指令都等 ack，`set_torque`/`set_gain` 的“逐台写、汇总失败”依赖它。
- **地址 55 = 锁标志**：初值 1，**只管持久化，不拒绝写入**（写 0 关锁→写 EEPROM 地址掉电保存；
  写 1 开锁→写 EEPROM 掉电不保存）。因此 RAM 写（50/51）根本不需要解锁，本补丁也不写 55。
- **地址 60 = 当前负载**：0.1% 占空比，**bit10 为方向位**（不是 bit15）。本补丁不解码它。
- **地址 65 = 舵机状态**：错误位 bit0 电压 / bit1 磁编码 / bit2 温度 / bit3 电流。当前未解码，
  仅作为后续排障依据记录。
- **地址 6 = 波特率**：HLS 范围 **0..7**（0=1 Mbps…7=38400），不是任意分频；
  `EXPECTED_BAUD_CODE = 0` 保持不变，注释已补范围。
- **地址 40 = 扭矩开关**：0 关 / 1 开 / **2 = 阻尼输出**（本补丁只用 0/1）。

## 三、已验证 vs 必须上真机测量

**已验证（本机编译 + 单元测试，无硬件）**
- `cargo test -p duck-control` 93 passed / 0 failed；`cargo check -p duck-control -p robotd` 通过。
- `feetech` 模块 14 个协议测试通过，且模块文件与源文件逐字节一致（SHA-256 见上）。
- 位置换算按厂商表 4096 counts/圈、0.087°/count、bit15 方向、多圈量程（约 ±8 圈）自测通过。
- 乱序 SYNC_READ 回复按 id 重组为请求顺序（`drain_replies` 级与 `sync_read_blocks` 级各一测）。
- 单台缺席时突发在 `BURST_IDLE`（2 ms）内结束而不是 30 ms，且错误信息点名缺席 id；一台报非零
  status 时其余回复仍被收齐、错误信息点名该 id 与状态码、不把它再报成 missing。
- slot 0 用 SFLP 解码、舵机块不会进入 SFLP 解码器（含 `FF FF FD` 数据字节不误触发重同步）。
- 15 字节块足以覆盖每个字段；位置符号-幅值往返；速度 0.732 rpm/count。
- `set_torque` 在一台静默时仍写满全部关节并汇总报告；`set_gain` 每关节写 50(Kp, clamp 到
  0..254) 与 51(Kd=0)、不碰 52/21/22；`reboot` 先写 torque_enable(40)=0，再对该 id 恰好发出
  一个 `0x08`（不是 `RESET`），且不等应答——用会静默的 `Transport` 驱动也不会报错或花掉
  `READ_TIMEOUT`。

**必须在真机上测量/确认（本补丁无法验证）**
- 关节正方向、每关节零点/`DEFAULT_POSITION` 对 C001 是否成立（XL330 数值无意义）。
- 位置/速度/电流/电压/温度的绝对标度与符号（尤其 `RAD_PER_COUNT=2π/4096`、`0.732`、`6.5`）。
- Kp/Kd 的实际手感与稳定裕度：1/8、1/4 标度下 `gain=200` 是否过大（见未决事项 2/3）。
- ID 与波特率检查在真实总线上是否如预期；多机应答时序、`BURST_IDLE=2 ms` 的突发预算与
  `READ_TIMEOUT=30 ms` 的单台寻址预算在真机上是否足够。
- IMU 节点 slot 0 的 20 字节块与 15 字节截断在实际节点上的对齐。
- `write`/`set_torque`/`set_gain` 的真实总线耗时与舵机响应。

## 四、未决事项（本补丁刻意不解决）

1. **零点与 `DEFAULT_POSITION`**：C001 的零点/装配方向必须重新测量；XL330 的数值不适用。
2. **增益整定（地址已定，标度待调）**：Kp=RAM 50、Kd=RAM 51 已按厂商表实写；
   但比例 1/8、微分 1/4、范围 0..254 是**文档值**，`deploy/robotd.toml` 的
   `gain = 200` 位于范围顶端、`gain_limp = 50` 为其 1/4，都必须上真机重新整定；
   在此之前 Kp/Kd 的绝对好坏无法在本仓库判定。
3. **`deploy/robotd.toml` 增益值**（gain=200、gain_limp=50、limp_fall_pose_gain=160、
   standing_gain_ratio=0.8）是按 XL330 的 0..16383 标度调的，**本补丁刻意不改**，必须实机重调。
4. **电池满/空电压**（`BATTERY_FULL_V=8.2`、`BATTERY_EMPTY_V=6.6`）是在 XL330 + NP-F550
   上测的，换舵机后需复测。
5. **IMU 节点的 FeeTech 人格**：地址 56 块的文档见 `飞特通讯协议/飞特通讯协议说明.md`，
   实现见 `v1/src/fee.c`；真实 FeeTech 舵机在 56..63 也返回位置/速度/负载，
   所以主机必须知道哪个 id 是 IMU（当前固定为 `IMU_DXL_ID=200`，且从舵机列表中排除）。
6. **`robot.rebootMotors` 现在会真正重启舵机**：`FeetechIo::reboot` 先写 torque=0，再单播
   `0x08` 后立即返回、不等应答；`robotd` 会把该次请求记为成功，随后靠 tick 的读失败滑过
   约 0.8 s 的离线窗口（823 ms 实测），并因 gain 缓存已失效而在下次 `apply` 写回增益。
   仍需真机确认的是端到端手感：重启后姿态如何、一次重启多少台、以及 `rebootMotors` 与
   `init`/Start 的先后是否符合预期（本补丁只验证了指令与 823 ms 窗口本身）。
7. **设计文档未同步**：`docs/design/robotd-design.md` §2.1 与 `docs/design/simulation.md`
   仍描述 Dynamixel/`rustypot` 总线与 124/144 地址，本补丁未重写（避免半改造成误导）。
   `model.rs`/`io.rs` 等代码内注释已同步到 FeeTech。
8. **负载/状态/电流方向位未解码**：地址 60 的 bit10、地址 65 的错误位、地址 69 的 bit15
   已在文档记录，但 `Sensors` 目前没有对应字段，故未实现。
