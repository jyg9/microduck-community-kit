# microduck 电机总线：Dynamixel XL330 → FeeTech SCS/HLS 补丁说明

本补丁把 `duck-control` 的电机总线从 Dynamixel XL330（`rustypot`）切换到 FeeTech
SCS/HLS（HD-1910-C001），复用 `v1/patches/feetech.rs` 里那份已经写好并测试过的
`feetech` 协议模块。

> **快照声明（2026-10-02）。** 这份补丁是**移植完成那一刻**的样子，**不随本树更新**。基线固定在
> 上游 `f0d934e`，`git apply` 到那棵树上仍然干净；但 `software/microduck_feetech/` 在那之后又做了
> 三件事，所以**补丁已不再等于那棵树**：
>
> 1. **去掉每笔串口事务的 `flush()`**——`serialport` 的 `flush` 是 `tcdrain`，在这块板子上固定
>    **~12 ms**（与帧长和有无应答都无关），控制环因此只有 36.3 Hz；删掉后回到 **50.0 Hz**。
> 2. **`BURST_IDLE` 2 ms → 4 ms**（实测 13.5 万个相邻应答间隔的最大值是 1.94 ms，2 ms 只剩
>    3% 余量）。
> 3. **同步上游 `main` 的 10 个提交**，并在 `bus.rs` / `model.rs` / `robotd-design.md` 里留下三处
>    `4656690`/`OFS_L` 的说明（上游那个会**写** EEPROM 的零点检查刻意没有带进来）。
>
> **冲突时以 `software/microduck_feetech/` 的代码为准**；补丁只用来回答"这次移植改了什么"。
> 同一个文件集合：补丁口径 **16 文件 +2341/−622**，**本目录今天是 +2364/−623**（差在 `bus.rs`
> +9、`model.rs` +8、`robotd-design.md` +6/−1）。**反向应用已经不干净**（恰好会在这三个文件上
> 失败），所以别用它回到官方总线。

- **基线**：上游 `pollen-robotics/microduck` `main` 的 `f0d934e`（2026-09-28）。上一版补丁的
  基线是 `5620aa2`（2026-09-08），两者相隔 1130 个提交，`duck-control/src/bus.rs` 与
  `robotd/src/main.rs` 都被上游改动过，因此补丁不是机械重放，而是按新代码重新移植
  （见第三节）。
- **验证命令**（`CARGO_HOME` 指向工作区内，绕过只读的 `~/.cargo`）：
  `cargo check -p duck-control -p robotd`、`cargo test -p duck-control`
- **结果**：`cargo test -p duck-control` **108 passed / 0 failed**（0.92 s，其中 1 s 是
  收养流程里那次 `REBOOT_SETTLE` 等待）；`cargo check -p duck-control -p robotd` 通过、
  无新增 warning；`cargo test -p robotd-params` **93 passed / 0 failed**。同一 workspace 的
  `cargo check --workspace --all-targets` 在本机因缺系统 gstreamer（`mediad`/`uyvy` 的
  `gstreamer-sys`）失败，与本补丁无关。
- **本机 `cargo test -p robotd` 有 2 个单测失败**（`an_unloadable_policy_holds_the_pose_and_
  reports_why`、`setting_a_skill_keeps_what_the_call_left_out`），原因是本机 ONNX Runtime /
  模型文件的差异（报错分别是 `/opt/robot/policies/current/velstand.onnx` 不存在、
  `Protobuf parsing failed`），**不是本补丁**：在干净的 `f0d934e` 上不打补丁直接跑，这两个用例
  以同样的信息失败。
- **补丁自检**：`git apply --check` 可干净应用到干净的 `f0d934e`（2026-10-02 复核）；把它应用到
  干净副本后，16 个文件里 **13 个**与本目录**逐字节一致**——另 3 个正是上面快照声明里的那三处
  后续改动（`duck-control/src/bus.rs`、`duck-control/src/model.rs`、`docs/design/robotd-design.md`）。
- `git diff --stat`：16 files changed, 2341 insertions(+), 622 deletions(-)
  （其中 `duck-control/src/feetech.rs` 为新增 556 行，SHA-256
  `0d3cfe86b4f80b689630f6fb3c39957d1e3940d8554a32d0c1122450cdea6c5a`，
  与本目录 `v1/patches/feetech.rs` 逐字节一致）。
- **真机首跑（2026-09-30，15 舵机 + IMU 节点）结果见第六节**：总线与 50 Hz 环路跑通、
  15 个关节跟随目标，并因此修了两处启动缺陷；同时暴露了一个必须按关节整定的增益问题。

## 一、逐文件改动

1. **`duck-control/src/feetech.rs`（新增）**
   从 `v1/patches/feetech.rs` 复制。提供帧构造/解析、`FrameReader`、`reg` 寄存器地址、
   换算常量与 15 个单测。本次相对上一版有改动：新增 `sign_magnitude(raw, direction_bit)`，
   `position_raw_to_counts` 改为调用它，并删除 `le_i16`（见第三节第 2 条）。

2. **`duck-control/src/lib.rs`**
   在 `fall` 与 `imu` 之间加入 `pub mod feetech;`，与既有模块列表同风格。

3. **`duck-control/src/bus.rs`（核心重写，保留原文档语气与结构）**
   - `DynamixelIo` → `FeetechIo`；`open` 直接使用 `serialport`，不再经过 `rustypot`。
   - `Xl330Controller` 调用全部替换为 `feetech::*` + 手写收发。
   - 新增内部 `Transport` trait（`Send`，`write_all/read/flush`）与 `SerialTransport`
     包装真实串口。这是为了让 `set_torque`、乱序收包、收养流程等失败路径在没有硬件的
     单测里可驱动。
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
     寻址命令（torque/gain/寄存读写/PING）的预算，由 `exchange()`/`ping()` 轮询耗尽。
   - `assemble_tick()`：slot 0 为 IMU 节点，slot 1.. 为 `JOINT_IDS` 顺序的舵机；纯函数，便于单测。
   - `imu_block_from_slot()`：只取 slot 0 的前 12 字节交给 `SflpDecoder`。IMU 节点在 FeeTech
     人格下同一地址（56）回答的是 **15 字节契约块**：前 12 字节是 SFLP 控制块、12 是采样计数、
     13 是状态位、14 保留 0——与真舵机在 56..70 的块长一致，因为一次 `sync_read`
     的读长对所有设备必须相同。原始加速度在 **128** 的 20 字节诊断块里，不属于本 tick 的读长
     （本 tick 读 15 字节，`imu_block_from_slot()` 只取前 12）。
   - `decode_servo_block()`：解析 15 字节舵机块（见下表）。**位置、速度、电流都是符号-幅值**
     （BIT15 方向位，`feetech::sign_magnitude`），负载是 BIT10 方向位但不解码——`Sensors`
     没有 load 字段，不发明新字段。
   - `write()`：`sync_write` 到 `GOAL_POSITION_L=42`，每舵机 2 字节，由
     `feetech::position_from_rad`（符号-幅值）生成；广播写不产生 ack，因此不等待。
   - `set_torque()`：逐 id `write` 到 `TORQUE_ENABLE=40`，每台都写、失败最后汇总上报，保留
     原行为与注释；文档说明 FeeTech 总线对每条指令都回 ack，Dynamixel 的 `return_delay_time`
     预算问题不再适用。
   - `read_register`/`write_register`：单寄存器读写，走 `exchange()`（等下一条 ack）。
     新增 `write_eeprom()`：**先写 `LOCK(55)=0` 解锁 → 写值 → 等 `EEPROM_SETTLE` →
     写回锁**。改 ID 时回锁发给**新 ID**（见第三节第 3 条）。
   - `ping(id) -> Ok(bool)`：发 `instruction_no_addr(id, PING)`，超时按“没有应答”返回
     `Ok(false)`（席位缺失是问题本身，不是错误）。
   - `check_registers()` / `check_registers_of(id)`：逐台只读并纠正 `reg::ID=5`（应与
     `JOINT_IDS` 一致）与 `reg::BAUD_RATE=6`（应为 `EXPECTED_BAUD_CODE=0` = 1 Mbps）；
     两处都是 EEPROM 地址，所以都走 `write_eeprom()`。不再断言 XL330 的
     `return_delay_time/pwm_slope/shutdown`。
   - `missing_servos()`：15 次 PING 的普查，返回不应答的 id（上游新增功能，协议无关，原样移植）。
   - `replacement_target(missing)`：只有一个 id 缺失时才返回它（上游新增功能，原样移植）。
   - `adopt_replacement(id)`：**按飞特出厂默认值适配过的收养流程**（见第三节第 4 条）——
     PING 出厂 ID 1 → 解锁并改写 ID → 回锁 → `check_registers_of()` → 写 torque 0 →
     发 `0x08` REBOOT → 等 `REBOOT_SETTLE` → PING 新 ID → 读 `舵机状态(65)` 记录是否仍有
     错误位。
   - `present_positions()`：一次 15 台、长度 2 的 `sync_read`（启动时只取位置，读更短、突发更短）。
   - `set_gain()`：**实写 RAM 增益寄存器**：
     - 每关节写 **RAM 50（`Kp`，0x32）** 一个字节，`clamp(kp, 0, 254)`，`kp` 原样通过、
       不做换算（厂商表注明比例系数 1/8）。
     - 同时写 **RAM 51（`Kd`，0x33）= 0**：出厂 D 值非零，留着会在同一 `kp` 下给舵机自己的
       位置环加阻尼（与 Dynamixel 版把 D 置零同理；厂商表注明微分系数 1/4）。
     - **不写 Ki（52，0x34）**：厂商表明确位置伺服模式下无效（属于恒速环），不为“对称”而写 0。
     - **不写 EEPROM 21/22（0x15/0x16）**：那是上电默认值，RAM 写立即生效，每次启动写 EEPROM
       只是白烧闪存；也因此**无需处理 55（锁标志）**——该位只决定 EEPROM 写是否掉电保存。
     - 沿用 crate 既有形状：**每台都写、失败最后汇总上报**。
     - 文档明确 `gain` 在该 0..254 范围里的位置：本分支已把它与派生值一起改成
       FeeTech 标度，见第 12 条与第六节发现 3。
   - `reboot()`：FeeTech 指令 `0x08`（厂商 STM32 HAL 的 `INST.h` 写作 `INST_REBOOT`，其
     `SCS.c::Reboot()` 也是发完即 flush、不读应答）。先用寻址写把该 id 的
     **torque_enable(40) 写 0**，再单播一个无参数的 `0x08`，**不等任何应答**（本来就没有，
     等只会每台舵机烧掉一次 `READ_TIMEOUT` 并卡住控制环），帧一上线路就返回：不 sleep、不 ping。
     若舵机已失联，`write_register` 按周边代码的方式上报错误（写失败不吞）。**绝不能**把它映射到
     `RESET(0x0A)`：那是恢复出厂设置，会清掉 id/波特率/限位；`0x08` 才是重启。
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
     原地保留 `StaleImuTracker` 的全部测试。本次新增：速度/电流符号-幅值解码、
     `replacement_target` 单缺才成立、以及整条收养流程的字节级断言
     （`FreshServo` 假总线：解锁→改址→回锁→校验→torque 0→`0x08`→回来后 PING 新 ID）。

4. **`duck-control/Cargo.toml`**
   删除 `rustypot`（上游此时已升到 1.8.0，为的是 protocol 2.0 的 fast sync read 0x8A——
   飞特总线没有这条指令）及其说明注释，改为一段说明为什么本 crate 自带 `feetech`；
   保留 `sha2`、`[dev-dependencies]` 与 `serialport = { version = "4.8",
   default-features = false }` 及交叉编译理由注释。

5. **`Cargo.lock`**
   cargo 自动移除 `rustypot` 及其独占依赖（84 行删除），按“诚实”原则一并纳入补丁。

6. **`duck-control/src/model.rs`**
   `EXPECTED_REGISTERS` 表删除，替换为 `EXPECTED_BAUD_CODE = 0` 与说明（id 的期望值按关节
   逐个给，不放进表；补充 HLS 波特率寄存器范围为 **0..7**）。`FACTORY_BAUD_RATE` 删除：
   厂商表写明出厂就是 1 Mbps，与总线同速，所以“换一个波特率去找新舵机”这件事不存在；
   `FACTORY_ID = 1` 保留（厂商默认站号）。上游新增的 `factory_defaults_are_unused_on_the_bus`
   改为不比较波特率，`the_shutdown_mask_clears_the_input_voltage_bit` 随
   `EXPECTED_REGISTERS` 一起删除，换成 `the_baud_code_and_the_port_speed_agree`。
   顺带把 `DynamixelIo::bus_voltage` 等失效引用改成 `FeetechIo`。

7. **`duck-control/src/io.rs`**
   仅文档：`Sensors` 的 IMU 来源改为“共享舵机总线”；`reboot` 说明 FeeTech 用无应答的
   `0x08`、离总线约 823 ms（厂商手册 ~800 ms）后回来、RAM 增益回到 EEPROM 默认值；
   `slow_sensors` 说明现已折叠进 tick。trait 方法签名不变。

8. **`duck-control/src/imu.rs`、`safety.rs`**
   仅注释：IMU 板“共享舵机总线”；执行器行程 ±π 的说明补上 12-bit XL330 与 15-bit
   符号-幅值 FeeTech 的差异，边界数值未改。

9. **`robotd/src/main.rs`**
   `BusIo` 类型别名与两处 `::open` 改为 `FeetechIo`（`open(port)` 只带端口，不再传
   `bus.fast_sync_read`）；`bus.fast_sync_read == false` 时的 journal 文案改为“在飞特总线上
   没有效果”。上游新增的 `adopt_missing_servo()`（普查 → 单缺 → 收养）原样保留，
   只把“factory defaults (id 1, 57600 baud)”的日志改成“(id 1, 1 Mbps)”。

10. **`robotd-params/src/registry.rs`、`robotd/src/soc.rs`**
     文案：端口描述与“不碰总线”的注释改为 FeeTech/舵机总线；`bus.fast_sync_read` 的描述
     改为“自 Dynamixel 总线保留；对飞特舵机没有效果”。

11. **`deploy/robotd.toml`、`docs/design/robotd-design.md`、`docs/design/simulation.md`**
     文档：`fast_sync_read` 的说明重写为“保留键、无效果”；设计文档 §1.1 顶部加“本树带飞特
     补丁”的提示块，总线图改成 `FeetechIo` / FeeTech SCS/HLS，fast-sync-read 段落改成飞特
     广播 `sync_read` 的说法，`DynamixelIo` 全部改名；`simulation.md` 同步。
     其余仍描述 XL330 寄存器与 `rustypot` 的段落见第五节未决事项 7。

12. **`robotd-params/src/lib.rs`（+ `deploy/robotd.toml`、两处引用旧值的注释）：增益默认值**
     把配置默认从原型值改成 FeeTech 标度——`policy.gain` **200 → 32**（厂商 EEPROM 默认值）、
     `safety.gain_limp` **50 → 8**（原来的 1/4）、`safety.limp_fall_pose_gain` **160 → 26**
     （原来的 0.8），`standing_gain_ratio = 0.8` 不变；`deploy/robotd.toml` 里那几个注释掉的
     默认值与说明同步，`robotd/src/main.rs` 与 `bus.rs` 引用旧数字的注释一并改。
     依据是第六节发现 3 的实测（200 → 5 A / 74 °C 自激振荡，32 → 110 mA 且温度不升）。
     `robotd/src/control.rs` 的 `Tuning::default()` **刻意保持 200**：它是测试用的“原型镜像”
     （测试名就是 `the_defaults_match_the_prototype`），生产路径从 params 构造它。

## 二、FeeTech 寄存器与换算决策（块 56..70，低字节在前）

| 地址 | 字段 | 处理 |
|---|---|---|
| 56/57 | present_position | 15-bit 符号-幅值：`position_raw_to_counts(le_u16)` × `RAD_PER_COUNT`(2π/4096, 0.087°/count) |
| 58/59 | present_speed | **15-bit 符号-幅值**（BIT15 方向位）× `RAD_PER_SEC_PER_SPEED_COUNT`（0.732 rpm/count） |
| 60/61 | present_load | **保留未用**（`Sensors` 无 load 字段，不发明；方向位是 **BIT10**，与其它字段不同） |
| 62 | present_voltage | `u8` × 0.1 V/count，参与均值；0 值过滤 |
| 63 | present_temperature | `u8`，整 °C，逐关节保留 |
| 65 | 舵机状态 | 错误位：bit0 电压 / bit1 磁编码 / bit2 温度 / bit3 电流（未解码，只读来记录收养后是否仍有错误） |
| 66（块内偏移 10） | moving | 未使用 |
| 69/70 | present_current | **15-bit 符号-幅值**（BIT15 方向位）× 6.5 mA/count，取绝对值填入 `currents_ma` |
| 42 | goal_position | `sync_write` 2 字节/舵机，广播不 ack；单位 0.087°、bit15 方向 |
| 40 | torque_enable | 逐 id `write`，`0x01`/`0x00`，要 ack；值 2 = 阻尼输出 |
| 50 / 51 | Kp / Kd（RAM） | `set_gain` 逐 id `write`，单字节；不写 Ki(52)、不写 EEPROM(21/22) |
| 55 | 锁标志（LOCK） | `write_eeprom` 用：`0` 解锁 → 写 EEPROM 地址 → `1` 回锁；出厂值 1 |
| 5 / 6 | id / baud_rate | 启动检查；baud code 0 = 1 Mbps，寄存器范围 0..7 |

**关键决策**
- 位置、速度、电流**三者都必须走符号-幅值**，不能把弧度/计数直接 `as i16`。速度与电流在上一版
  补丁里按 `i16` 读，本次已改（见第三节第 2 条，含厂商证据）。
- **位置标度（v2 起）**：厂商内存表（`飞特通讯协议/HLS系列舵机内存表.html`
  “磁编码HLS舵机-内存表解析”§2.3/§2.4）把 `目标位置`/`当前位置` 的单位写作 **0.087°**，
  即 **4096 counts/圈**，字段为 `-32767..32767`、bit15 为方向位，因此量程是多圈（约 ±8 圈），
  不是“15 bit 一圈”。旧版采用二手资料里的 **32768 counts/圈**，与厂商表矛盾：
  这会让每个关节角度差 **8 倍**。v2 已改为 4096，旧结论作废。
- 一次 15 字节能同时拿到位置/速度/负载/电压/温度/moving/电流，因此电压温度不再另开事务；
  `slow_sensors()` 改为返回最近 tick 的缓存样本（最多落后一个 tick，对电池/温度无影响；
  tick 失败时保留上一份好样本，而不是把一次丢包当成电池掉电）。
- `set_gain()` 的地址已由厂商表落定（50/51），不再是缺口；剩下的是**标度与整定**问题：
  寄存器比例 1/8、范围 0..254，而 `deploy/robotd.toml` 的增益是按 XL330 标度调的，必须真机重调。

### 厂商内存表事实（本补丁依赖）

- **地址 5 = 主ID**：初值 **1**，0..253，总线上唯一。
- **地址 6 = 波特率**：初值 **0** = 1 Mbps，范围 0..7（0=1Mbps，1=500k，2=250k，3=128k，
  4=115200，5=76800，6=57600，7=38400）；出厂串口配置即“波特率默认为1Mbps，默认通信地址
  （站号）为1”。
- **地址 8 = 应答状态级别**：初值 1，0..1。0 = 除读/PING 外不应答；1 = 对所有指令应答。
  本补丁按 1 工作：每条写指令都等 ack，`set_torque`/`set_gain`/`write_eeprom` 的“逐台写、
  汇总失败”依赖它。
- **地址 55 = 锁标志**：初值 1，**只管持久化，不拒绝写入**（写 0 关锁→写 EEPROM 地址掉电保存；
  写 1 开锁→写 EEPROM 掉电不保存）。因此 RAM 写（50/51）不需要解锁；而 **EEPROM 写（id/波特率）
  必须先解锁**，否则下次上电就丢——见第三节第 3 条。
- **地址 60 = 当前负载**：0.1% 占空比，**bit10 为方向位**（不是 bit15）。本补丁不解码它。
- **地址 65 = 舵机状态**：错误位 bit0 电压 / bit1 磁编码 / bit2 温度 / bit3 电流。解码未实现，
  只用于收养后记录。
- **地址 40 = 扭矩开关**：0 关 / 1 开 / **2 = 阻尼输出**（本补丁只用 0/1）。

## 三、相对上一版补丁（基线 `5620aa2`）的改动

1. **基线追到 `f0d934e`，并保留上游新增功能。** 上游在这 1130 个提交里给总线加了两件事：
   替代舵机自动收养（`missing_servos`/`replacement_target`/`adopt_replacement`）与
   `bus.fast_sync_read` 键，还把 `rustypot` 升到 1.8.0 以使用 fast sync read。
   移植时不删除功能，只按飞特协议重做：
   - `bus.fast_sync_read` **保留为配置键但没有效果**：它选的是 protocol 2.0 的 0x8A（所有设备的
     答复合并进一个状态包），而飞特广播 `sync_read`(0x82) 本来就是一次请求、每台各自回一个状态包，
     没有更慢的变体可退。删掉这个键会让已有 `robotd.toml` 落到“未知键”告警路径上，所以选择保留 +
     在登记表/`deploy/robotd.toml`/设计文档里写明无效，`open_bus` 只在它为 `false` 时说一句。
   - 收养流程按飞特出厂默认值重写（第 4 条）。
2. **速度/电流改成符号-幅值（真错误修复）。** 上一版按 `feetech::le_i16` 读 58/59 与 69/70，
   但厂商自己的 `FTServo_Linux-main/src/HLSCL.cpp` 里 `ReadSpeed`/`ReadLoad`/`ReadCurrent` 与
   `ReadPos` 是同一种写法：`if (v & (1<<bit)) v = -(v & ~(1<<bit))`（速度/电流 BIT15，负载 BIT10）；
   本仓库真机用过的 `software/hls_servo_debugger/`（`README.md` 写明“速度 46/58 … BIT15 方向”、
   “电流 44/69 … BIT15 方向”，代码用 `encode_signed_magnitude(..., 15)`）与厂商内存表的
   “BIT15为方向位”也都一致。按 `i16` 读的实际后果：方向位为 1、幅值很小时会读成接近负满量程的数——
   电流经 `.abs()` 后约 **327 倍**偏大，速度在观测向量里同样是错的，而且只在“方向位为 1 的那一半”
   出现，表现为时好时坏。现在 `feetech::sign_magnitude(raw, bit)` 统一处理三种字段，
   `le_i16` 已删除（它在本 crate 里已无正确用途，留着就是下次踩坑的入口）。
3. **EEPROM 写要解锁，改 ID 后回锁要发给新 ID。** 上一版 `check_registers` 直接写 ID(5)/波特率(6)，
   二者都是 EEPROM 地址，而锁标志出厂为 1——“写入EPROM地址的值掉电不保存”。结果是纠正看起来成功、
   读回也对，但**下次上电就丢**，舵机再次消失，且没有任何线索。现在统一走 `write_eeprom()`：
   `55=0` → 写值 → `EEPROM_SETTLE` → `55=1`（厂商 `HLSCL::unLockEprom`/`LockEprom` 就是这个顺序）。
   新发现的一层：**改 ID 之后回锁必须发给新 ID**。原样写回旧 ID 时，真实舵机（和本补丁新增的
   `FreshServo` 假总线一样）已经不应答老地址了，于是回锁无人应答——既报一次超时、又把舵机留在
   未锁状态。`write_eeprom` 因此在 `addr == reg::ID` 时把回锁发给写入的值。
   写 ID 本身仍然发给旧 ID 并等 ack：厂商 `SCS::Ack` 会读回复里的 id，不等于被寻址 id 就报
   `ERR_SLAVE_ID`，而厂商工具正是这样改 ID 的，说明回复带的是旧 id。
4. **收养流程按飞特默认值重写。** 上游的 XL330 版本先 ping 1 Mbps，再 `reopen(57600)` 找“出厂
   波特率下的新舵机”，再改 ID、改波特率、重启、回读 `hardware_error_status(70)`。飞特这边：
   - 出厂就是 **ID 1 + 1 Mbps**（厂商表），与总线同速，所以**没有第二个波特率可试**，
     `reopen`/`FACTORY_BAUD_RATE` 整个删除，搜索就是一次 `ping(FACTORY_ID)`。
   - 改 ID 走 `write_eeprom`（解锁/回锁）。
   - 重启后的健康检查读 **舵机状态(65)** 而不是 XL330 的 70，错误位含义记在上表。
   - `REBOOT_SETTLE` 取 **900 ms**：真机实测 823 ms 回来、厂商手册 ~800 ms，而 XL330 版本的
     500 ms 对着这个数字是不够的，早 ping 会把“还在启动”误判成“改址失败”。
5. **`EXPECTED_REGISTERS`/`FACTORY_BAUD_RATE` 与相应单测的调整**，见第一节第 6 条；
   新增的三条单测（符号-幅值、收养流程字节序、`replacement_target`）见第一节第 3 条末尾。

## 四、已验证 vs 必须上真机测量

**真机已验证（2026-09-30，15 舵机 + 节点，详见第六节）**
- 一条 1 Mbps 总线上 16 个设备：`check_registers` 15/15 通过，id 与波特率无需纠正。
- 50 Hz 环路 **3858 ticks / 0 missed**；策略加载并驱动；`robot init` 扭矩 + 2 s 归位成功。
- **15 个关节实测≈目标（最大偏差 0.02 rad）**：位置符号-幅值编码与 4096 counts/圈在真机上成立。
- 72-byte 之外的每 tick 15 字节突发在真机上无丢失（干净端口下 20 次突发零丢失、零畸形帧）；
  `BURST_IDLE = 2 ms` 足够（16 条回复最远 ~4.9 ms 到齐，帧间隔 ~300 µs）。
- IMU 节点在 **56** 地址按 15 字节契约块应答（前 12 字节给 SFLP 解码器），与舵机块同长。
- 电池电压、逐关节温度、四元数都是真值。

**已验证（本机编译 + 单元测试，无硬件）**
- `cargo test -p duck-control` **108 passed / 0 failed**；`cargo check -p duck-control -p robotd`
  通过、无新增 warning；`cargo test -p robotd-params` 通过。
- 补丁 `git apply --check` 可干净应用到 `f0d934e`；应用到干净副本后，16 个文件里 **13 个**与本目录
  逐字节一致——另 3 个就是文首快照声明里的那三处后续改动（`bus.rs`、`model.rs`、`robotd-design.md`）。
- `feetech` 模块 15 个协议测试通过，且模块文件与 `v1/patches/feetech.rs` 逐字节一致
  （SHA-256 见文首）。
- 位置/速度/电流三种符号-幅值解码（含与 `i16` 结果的**不等**断言，避免测试蒙对）；
  位置换算按厂商表 4096 counts/圈、0.087°/count、bit15 方向、多圈量程（约 ±8 圈）自测通过。
- 乱序 SYNC_READ 回复按 id 重组为请求顺序（`drain_replies` 级与 `sync_read_blocks` 级各一测）。
- 单台缺席时突发在 `BURST_IDLE`（2 ms）内结束而不是 30 ms，且错误信息点名缺席 id；一台报非零
  status 时其余回复仍被收齐、错误信息点名该 id 与状态码、不把它再报成 missing。
- slot 0 用 SFLP 解码、舵机块不会进入 SFLP 解码器（含 `FF FF FD` 数据字节不误触发重同步）。
- 15 字节块足以覆盖每个字段；位置符号-幅值往返；速度 0.732 rpm/count。
- `set_torque` 在一台静默时仍写满全部关节并汇总报告；`set_gain` 每关节写 50(Kp, clamp 到
  0..254) 与 51(Kd=0)、不碰 52/21/22；`reboot` 先写 torque_enable(40)=0，再对该 id 恰好发出
  一个 `0x08`（不是 `RESET`），且不等应答——用会静默的 `Transport` 驱动也不会报错或花掉
  `READ_TIMEOUT`。
- 收养流程的完整字节序：PING 出厂 ID → `55=0`(旧 id) → `5=新id`(旧 id) → `55=1`(**新 id**) →
  读 5/6 校验 → `40=0` → `0x08` → 等 900 ms → PING 新 id → 读 65；没有出厂 ID 时返回
  `Ok(false)`，非关节 id 返回错误。

**必须在真机上测量/确认（本补丁无法验证）**
- 关节正方向、每关节零点/`DEFAULT_POSITION` 对 C001 是否成立（XL330 数值无意义）。
  位置标度本身已在真机上验证（15/15 跟随目标），剩下的是**每关节的装配零点与正方向**。
- 速度/电流/电压/温度的绝对标度（尤其 `0.732`、`6.5`）。位置与三者的**符号-幅值编码**
  已有厂商库 + 本项目调试器两份证据；真机上只直接验过位置，速度/电流的绝对值仍未核对。
- Kp/Kd 的稳定裕度：**已确认 200 过大、32 在台面上稳定**（见第六节发现 3），默认值已随之
  改为 32/8/26；负载下的整定仍未做（见未决事项 2/3）。
- 多机应答时序与单台寻址预算：`BUS_PROTO_STICKY_MS`、`READ_TIMEOUT = 30 ms` 单台寻址预算
  在真机上够用（普查 15 次 ping 全通），但**只有 1 Mbps、只有这一套硬件**。
- `write`/`set_torque`/`set_gain`/`write_eeprom` 的真实总线耗时与舵机响应。
- **收养流程整体**：出厂 ID 1 是否真的应答（新舵机上电后是否已进入 1 Mbps 模式）、改 ID 的
  ack 里带的是旧 id 还是新 id、`0x08` 后 900 ms 是否足够、改 ID 后 EEPROM 是否真的保住
  （断电复测）。这一条只能上真机，且步骤是破坏性的（会改舵机 ID），建议拿一台备用舵机做。
  真机上**部分**观察过：`0x08` 不应答、823 ms 回来（2026-09-24 的探针记录）已确认；
  改 ID 与解锁/回锁的持久性仍未验证。

## 五、未决事项（本补丁刻意不解决）

1. **零点与 `DEFAULT_POSITION`**：C001 的零点/装配方向必须重新测量；XL330 的数值不适用。
2. **增益整定（地址已定，标度已实测）**：Kp=RAM 50、Kd=RAM 51 按厂商表实写；比例 1/8、
   微分 1/4、范围 0..254 是**文档值**。真机首跑已确认 200 会让 `left_hip_yaw` 自激振荡
   （峰值电流 5 A、74 °C），**厂商默认值 32 在台面上安静**（峰值 110 mA、温度不升，见第六节
   发现 3），本分支的默认值因此改为 32（派生值 8/26 按原比例缩）。这仍不是整定终值：
   稳定裕度、随负载的表现、以及 Kd 是否该保持 0，都要按关节在真实负载下重测。
3. **增益默认值：本分支已改**（`gain = 32`、`gain_limp = 8`、`limp_fall_pose_gain = 26`，
   `standing_gain_ratio = 0.8` 不变，见第一节第 12 条）。原值 200/50/160 是按 XL330 的
   0..16383 标度调的，在飞特 0..254 上是顶格。**32 只有台面证据**（悬空、单个关节、
   无负载），`8`/`26` 只是按原来 1/4 与 0.8 的比例跟着缩，**都没有真机整定过**：
   整机负载下的手感、站立与起步的稳定裕度仍要按关节重调。
   `software/hls_servo_debugger` 的初始化会写同一个寄存器（EEPROM 21/22 + RAM 50/51），
   两处的值要一致。
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
7. **设计文档只同步了协议与命名**：`docs/design/robotd-design.md` §1.1 已加“本树带飞特补丁”
   提示块、总线图、`FeetechIo` 命名与 fast-sync-read 段落，`simulation.md` 与
   `deploy/robotd.toml` 同步；但该文件里仍按 XL330 叙述的段落（`return_delay_time` 的
   每设备周转预算、寄存器检查清单、自动收养是“新 XL330”、§3.4 与 `rustypot` 相关的测量）
   **没有重写**——半改只会更误导，提示块与本文是权威。其余提到 Dynamixel/XL330 的位置：
   `robotd-params/src/lib.rs` 的参数文档注释、`duck-control/src/safety.rs` 的注释。
8. **负载/状态/电流方向位未解码**：地址 60 的 bit10、地址 65 的错误位已在文档记录，
   但 `Sensors` 目前没有对应字段，故未实现。
9. **`bus.fast_sync_read` 作为无效键保留**：这是刻意的兼容取舍（见第三节第 1 条），
   不是遗漏；若将来决定彻底删除，需要同时改 `robotd-params` 的注册表、默认值与
   `deploy/robotd.toml`，并接受已有配置文件会走“未知键”告警路径。

## 六、真机首跑记录（2026-09-30）

**场地**：15×HD-1910-C001 + `imu_to_dxl` 节点挂在同一条 `/dev/ttyACM0`（1 Mbps，
`1a86:55d3`），舵机悬空；Ubuntu x86-64，`robotd` 0.15.0 release（本补丁），
`walk` 槽指向 `2026-09-04_22-59-06_velocity.onnx`。**第一次把补丁跑在真机上。**

**跑通的**：

* `check_registers` 15/15 通过（journal：`motor registers already correct`）——id 与波特率
  （code 0 = 1 Mbps）与总线一致，无需纠正。
* 控制环路 **50.0 / 50.0 Hz，3858 ticks，0 missed**，策略加载成功；
  `robotctl robot init` 打开扭矩并 2 s 归位，journal：`at the home pose; the policy has the robot`。
* **15 个关节全部跟随目标**（`robotctl monitor --json`）：实测与目标最大偏差 **0.02 rad**，
  14/15 关节电流 0–13 mA。这条同时**验证了位置标度**：命令行与回读走同一套
  `position_from_rad`/`position_rad`，若 4096 counts/圈 或符号-幅值写错，关节会跑到 8 倍
  开外并顶死，实测不会。
* 电池 7.8–8.1 V、逐关节温度、IMU 四元数都是真值（节点状态位 `0x81` = 融合有效）。

**发现的问题**

1. **同一个 tty 被第二个进程占用**（已修文档，非代码缺陷）。台面上一个残留的
   `hls_debugger` GUI 先打开了端口；`robotd` 随后用 `TIOCEXCL` 打开**成功**——`TIOCEXCL`
   只让*之后*的 `open()` 失败，不拒绝已经存在的 fd。于是两个进程各读走一半字节：
   每个 tick 随机报几个 id missing（`200, 23, 24, 30..34` 这种连续段），看起来像节点挂了、
   甚至像人格被锁死。实测节点当时两套协议都答（DXL 10/10、FeeTech 5/5），
   `V_PROTO_LOCK`=0、配置未判脏，**排除了人格锁死**。清理进程后同一份代码
   **20 次 16-id 突发零丢失、零畸形帧**。
   操作含义：上车前先确认端口只有 robotd 一个持有者（`fuser /dev/ttyACM0`）。
2. **端口刚打开后的第一笔事务会丢**（已修）。连续 4 次冷启动，**每一次**都丢掉普查里
   第一个 id（20）的 PING：15 台里少一台 → 被当成“换了舵机” → 进收养流程找出厂 id 1 →
   失败 → 等下一轮，实测白等十几秒。修法是 `FeetechIo::open` 打开端口后等
   `OPEN_SETTLE = 400 ms`（`host/bus.py` 早就有同一个常数 `OPEN_SETTLE_S = 0.35`，
   注释写明就是为这块板子测的），并给普查加一次重试（同一个 id ping 两次才算缺失——
   缺失会触发**破坏性**的改址动作，不能凭一次丢帧判定）。改后 4/4 冷启动首次即通过。
3. **`left_hip_yaw`（id 20）在 `gain = 200` 下自激振荡，降到 32 后消失**（真机整定项，
   见未决事项 2/3）。这是本次最有价值的一条，前后对比可复现：

   | | `gain = 200`（`deploy/robotd.toml` 的出厂值） | `gain = 32`（厂商 RAM 50 的默认值） |
   |---|---|---|
   | id 20 峰值电流 | 4966–5388 mA | **110 mA** |
   | id 20 平均电流 | ~4000 mA | **20 mA** |
   | id 20 自报速度峰值 | 4.37 rad/s（目标不动） | 0.307 rad/s |
   | id 20 位置误差 | 0.02 rad | 0.0078 rad |
   | id 20 壳温 | 升到 **74 °C** 且持续上升 | 48→49 °C，**通电 6 s 内不变** |
   | 其余 14 关节 | 电流近 0（正常） | 电流近 0（正常） |

   人耳现象与数据一致：200 时“抖得厉害、发热严重”，32 时安静。机理：`set_gain` 把
   RAM 50(Kp) 写成 `gain`，而 200 在 FeeTech 的 0..254 寄存器里是**顶格**（这个数值来自
   XL330 的 0..16383 标度），同时 RAM 51(Kd) 被写 0、去掉了舵机自带阻尼；厂商 EEPROM 默认
   P/D 是 **32/40**（本次回读确认，补丁不写这两个地址）。寄存器层面 id 20 与其它 14 台
   **完全一致**（OFS=0、mode=4“位置（新固件）”、最小/最大角度限制=0/0 即多圈模式、
   误差字 65=0），所以不是接线、零点或配置差异。回读 RAM 50 确认新值已写入（200 → 32）。
   这仍是**台面单点**结论：32 只是厂商默认，真正的稳定裕度与手感要按关节在负载下整定。
   补丁因此把默认值改成 `gain = 32`（派生 `gain_limp = 8`、`limp_fall_pose_gain = 26`，
   即原来的 1/4 与 0.8 比例），见第一节第 12 条与未决事项 3。
4. **IMU 小板平放在台面时 `fallen=true`**：`safety.gravity ≈ [-0.99, 0.11, -0.01]`。
   宿主按“装在机器人身上”的 90° 安装矩阵解释，台面平放自然对不上。
   这是台面姿态问题，不是固件或安装矩阵的缺陷。
5. **冷启动第一笔 `sync_read` 偶尔丢节点应答**：4 次冷启动里 2 次各丢 1 个 tick
   （只有 id 200 missing，随后 3000+ tick 干净）。机制未确认（怀疑节点仲裁的首次延迟
   或首帧与舵机应答碰撞）；影响是启动时一个 tick，自愈，暂不处理，记录在此。
