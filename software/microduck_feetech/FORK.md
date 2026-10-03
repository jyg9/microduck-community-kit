# microduck + 飞特舵机（FeeTech SCS/HLS）适配说明

这个目录是**官方 [microduck](https://github.com/pollen-robotics/microduck) 仓库的完整源码**
（`main` 分支，移植基线上游提交 `f0d934e`，2026-09-28），上面已经打好了把电机总线从
Dynamixel XL330 换成**飞特 HD-1910-C001（SCS/HLS 协议）**的补丁；2026-10-02 又同步了上游
`main` 的 10 个提交（见文末「已同步上游 `main`」）。可以直接编译、运行 `robotd` 等程序，
不需要再手工改一行代码。

配套硬件与固件（IMU 小板 `imu_to_dxl`）在本仓库的 <a href="../../hardware/imu_to_dxl">`hardware/imu_to_dxl`</a>、
<a href="../../firmware/v1">`firmware/v1/`</a> 下。

## 补丁在哪、怎么核对

* 补丁本身：<a href="../../firmware/v1/patches/microduck-feetech.patch">`firmware/v1/patches/microduck-feetech.patch`</a>
  （对上一条基线 `f0d934e` 可干净应用。**它是移植当时的快照，已不等于本目录**——本目录后来又删了
  `tcdrain`、把 `BURST_IDLE` 放宽到 4 ms，并同步了上游 `main`，所以**反向应用会在 3 个文件上失败，
  别拿它回退到官方总线**；冲突时以本目录为准）
* 逐文件说明、厂商寄存器依据、真机首跑记录、必须上真机确认的清单、未决事项：
  <a href="../../firmware/v1/patches/microduck_feetech_patch.md">`firmware/v1/patches/microduck_feetech_patch.md`</a>
* 协议模块的单文件副本：<a href="../../firmware/v1/patches/feetech.rs">`firmware/v1/patches/feetech.rs`</a>
  （与 `duck-control/src/feetech.rs` 逐字节一致）

主要改动一句话：`duck-control` 的 `DynamixelIo` → `FeetechIo`，不再依赖 `rustypot`，
自己实现飞特帧收发，坐标/速度/电流按厂商的**符号-幅值**编码解码；增益默认值改成飞特标度。

## 编译与自测

需要 Rust 工具链与系统 `serialport` 依赖。**本目录只是源码，不含构建产物**（`target/`）。

```bash
cd software/microduck_feetech

cargo check -p duck-control -p robotd   # 只编译总线与守护进程
cargo test -p duck-control              # 总线协议与控制的单元测试，不需要硬件
cargo test -p robotd-params             # 参数默认值（含增益）
```

已知情况（2026-09-29/30，本机 Ubuntu x86-64）：

* `cargo test -p duck-control`：**108 passed / 0 failed**
* `cargo test -p robotd-params`：**93 passed / 0 failed**
* `cargo test -p robotd`：有 2 个用例失败，原因是本机 ONNX Runtime / 模型文件差异，**与总线无关**
  （在未打补丁的同一基线提交上同样失败），详见补丁说明第一节
* `cargo check --workspace --all-targets`：需要系统 gstreamer（`mediad`/`uyvy`），
  本机没装，因此只验证了 `duck-control`、`robotd-params` 与 `robotd`

## 上车前必须知道

这套补丁已在 **15 舵机 + IMU 节点**的真机上跑过（2026-09-30，记录见补丁说明第六节）：
总线、50 Hz 环路、寄存器检查、归位、15 个关节跟随目标都通过。但下面三件事必须自己确认：

> **其中「50 Hz 环路」一项已被 2026-10-02 的实测推翻**，原因与本目录为它做的修复见文末
> 「改动量：与上游逐文件对比」一节。以实测为准。

1. **增益默认值已按飞特标度改过，但只有台面证据，仍要按关节整定。** 默认 `gain` 从原型的
   200 改成了 **32**（厂商 EEPROM 默认值），派生值 `gain_limp = 8`、`limp_fall_pose_gain = 26`
   按原来的 1/4 与 0.8 比例跟着缩。原因是实测：`gain = 200` 时 `left_hip_yaw` 自激振荡——
   电流 2.4–5.4 A、壳温 74 °C、剧烈抖动；改成 32 后峰值 110 mA、通电 6 s 温度不升。
   200 是 FeeTech Kp 寄存器 0..254 的顶格（原值按 XL330 的 0..16383 标度调），而 32 是在
   **悬空、无负载**的台面上测的：负载下的稳定裕度、站立与起步手感仍要自己重调。
   `software/hls_servo_debugger` 的初始化会写同一个寄存器（EEPROM 21/22 + RAM 50/51），
   两处的值要一致。
2. **一次只让一个进程持有串口。** robotd 用 `TIOCEXCL` 打开端口，但 `TIOCEXCL` 只拦*之后*的
   `open()`，不会拒绝已经打开的 fd。若另一个工具（例如调试器 GUI）先占着同一个 tty，
   两个进程会各读走一半字节，现象是每个 tick 随机丢几个舵机、看起来像节点挂了。
   上车前用 `fuser /dev/ttyACM0` 确认只有 robotd。
3. **台面上 IMU 小板平放时 robotd 会报 `fallen`。** 宿主按“装在机器人身上”的 90° 安装矩阵
   解释姿态，平放的板子自然对不上——这是台面姿态问题，不是固件或安装矩阵的缺陷。

关节零点/正方向、速度/电流的绝对标度仍未实机核对；收养新舵机（改 ID）的流程是有破坏性的
操作，建议先用备用舵机验证。完整清单见补丁说明第四、五节。

## 与上游的关系

* 上游仓库：<https://github.com/pollen-robotics/microduck>（本项目只是复刻配套，非官方）
* 与上游的**行为差异**不止总线：增益默认值（`policy.gain`、`safety.gain_limp`、
  `safety.limp_fall_pose_gain`）也按飞特舵机改过，详见补丁说明第一节第 12 条。
* 上游的 `docs/design/*.md` 里仍有按 XL330 叙述的段落，本目录只同步了协议与命名的部分，
  未重写的范围在补丁说明未决事项 7 中列出；**冲突时以本目录代码为准**（补丁说明是移植当时的
  快照，见 `firmware/v1/patches/microduck_feetech_patch.md` 顶部的快照声明）。

## 改动量：与上游逐文件对比

方法：把上游基线 `f0d934e` 用 `git archive` 导出一份干净树，与本目录（去掉 `target/`、
`dist/`、`staged/`、`build/` 等未跟踪产物）逐文件对比。

三个口径要分开（2026-10-03 用 git 的 `--numstat` 复核，方法可复现）：

| 口径 | 结果 |
|---|---|
| **补丁本身**（16 个文件，不含本页） | **+2341 / −622** ← 下表就是它 |
| **本目录今天**，同样这 16 个文件 | **+2364 / −623** |
| 本目录**整棵树** | **+2835 / −705，33 个文件**（= 上面 16 个 + 上游同步在补丁范围外的 16 个 + 本页） |

差的 23 行全部落在 3 个文件上（`bus.rs` +9、`model.rs` +8、`robotd-design.md` +6/−1），内容就是
`BURST_IDLE` 2 → 4 ms 的实测注释、`tcdrain` 删除的说明，以及同步上游时留下的
`4656690`/`OFS_L` 三处注释。**补丁的行数与本目录的行数不可相互推导**：它停在移植完成的那一刻
（见 `firmware/v1/patches/microduck_feetech_patch.md` 顶部的快照声明）。**下表是补丁的口径**
（`bus.rs` 记 1621/397，本目录现在是 1630/397）：

| 文件 | + | − | 性质 |
|---|---|---|---|
| `duck-control/src/bus.rs` | 1621 | 397 | 总线层重写（帧收发、时序） |
| `duck-control/src/feetech.rs` | 556 | 0 | **新文件**：零依赖协议模块 |
| `FORK.md` | 75 | 0 | 本页（**不计入补丁**，测量时的行数） |
| `duck-control/src/model.rs` | 48 | 49 | ID / 寄存器 / 出厂默认值 |
| `docs/design/robotd-design.md` | 25 | 37 | 文档 |
| `robotd-params/src/lib.rs` | 21 | 5 | 配置项 |
| `deploy/robotd.toml` | 18 | 14 | 配置项与注释 |
| `robotd/src/main.rs` | 14 | 11 | 启动接线 |
| `duck-control/src/io.rs` | 13 | 10 | **仅注释**（trait 签名未变） |
| `duck-control/Cargo.toml` | 7 | 8 | **移除 rustypot** |
| `robotd-params/src/registry.rs` | 6 | 2 | 配置项 |
| `duck-control/src/safety.rs` | 6 | 1 | **仅注释** |
| `docs/design/simulation.md` | 3 | 2 | 文档 |
| `duck-control/src/imu.rs` | 1 | 1 | 注释 |
| `robotd/src/soc.rs` | 1 | 1 | 注释 |
| `duck-control/src/lib.rs` | 1 | 0 | `pub mod feetech;` |
| `Cargo.lock` | 0 | 84 | rustypot 依赖树消失 |

`bus.rs` + `feetech.rs` 占新增行的 **90%**；除掉这三个文件，其余 **14 个文件合计只有
+164 / −225**，其中一半以上是注释或文档，而 `Cargo.lock` 是净减少。

### 这样切分是收得住的

* **接缝只有一个 trait。** `RobotIo` 的方法签名一个都没改（`io.rs` 的改动全是文档）。trait
  **之上**的一切——robotd 的控制环、`policy`、`obs`、`safety`、`kinematics`、`odometry`、
  IPC 协议、其余 6 个守护进程——**一行没碰**。
* **依赖是减少的**：`rustypot` 整棵树消失，换成一个零依赖、自带测试、且能与
  <a href="../../firmware/v1/host/bus.py">`firmware/v1/host/bus.py`</a> 逐帧对拍的模块。
* **动机留在代码里**：小改动绝大多数是在记录"为什么这条 Dynamixel 假设要变"，不是静默替换。

### 代价，按严重度

1. **`EXPECTED_REGISTERS` 从 4 个寄存器缩到 1 个波特率码 + 逐关节 ID 校验。** 丢掉的两项是
   上游的硬结论：`return_delay_time`（上游算过 500 µs/设备 × 16 = 8 ms/tick = 40% 预算）与
   `shutdown = 52`（把输入电压位清零，否则满电 2S 电池会让 15 个舵机自己 latch 掉）。理由是对的
   ——C001 没有已知地址，猜一个等于往真机写随机寄存器——但**结果是这两项没有替代保护**。这是
   整个移植里**唯一一处功能性净减少**，待查厂商内存表里有无等价物。
   附一条措辞更正：删 `return_delay_time` 的理由写的是"没有 per-device turnaround 预算要钉"，
   但实测相邻应答间隔最大 **1.94 ms**、整 burst 4.9 ms——**turnaround 是存在的**，只是比 XL330
   钉的值更小所以不需要限制；结论成立，理由要改。
2. **自己写组帧替掉了经过验证的库**：校验和、符号-幅值编码、`sync_read`/`sync_write`、组帧解析
   从此由本目录负责。
3. **本目录曾经落后上游 10 个提交**（19 文件 / +356 −84），**2026-10-02 已同步**（见文末
   「已同步上游 `main`」）。其中只有一个碰 `duck-control/`：`4656690`（收养的膝写入
   `homing_offset`），冲突只有一行 import。**它的机制刻意没有带进来**，见那一节。
4. **这个文件里的偏离是承重的。** 2026-10-02 实测：每笔事务多付一次 `tcdrain`（`flush()`，
   固定 **~12 ms**，与帧长和有无应答都无关），每 tick 两笔 → 控制环上限 ~41 Hz、实测
   **36.3 Hz**、`missed` 占 48%、更新健康门判不健康。删掉后同一块板子上 **50.0 Hz / missed 0**。
   `bus.rs` 里的 `BURST_IDLE` 判据也从 2 ms 放宽到 4 ms（实测 13.5 万个相邻应答间隔的最大值是
   **1.94 ms**，2 ms 只剩 3% 余量）。

### 两条结论

* **没有硬件理由就不新增偏离。** 这个文件里的每一处自主决定都要上真机量。
  一个具体的"不做"：IMU 的 staleness 判据（`StaleImuTracker` 比较 12 字节 payload）看起来不如
  比较节点在**块偏移 12** 处给出的采样计数器精确，但那**是上游的设计**——上游 `READ_LEN = 12`，
  counter 在 20 字节块的偏移 18，它**结构上收不到**这个字节；本目录因为读 15 字节才顺带收到。
  改它是一处新偏离，而且上游做不到同等改动，所以它属于上游 PR 而不是本目录补丁。实测也支持
  不动：节点侧修掉调试控制台的 2 秒冻结后，counter 重复 **0.00%**（7394 次读），残余的 0.27%
  payload 重复从未触到 `STALE_RUN_WARN = 25`（`orientation is frozen` 告警出现 **0** 次）。
* **上游的 10 个提交已在 2026-10-02 同步进来；`4656690` 的机制刻意不带**，理由见下一节。

## 已同步上游 `main`（2026-10-02）

从基线 `f0d934e` 起，上游 `main` 有 **10 个提交 / 19 个文件 / +356 −84**，已全部并入本目录，
净效果 **+311 / −83**。19 个文件里只有两个属于 `duck-control`，其余是文档、脚本与其它守护进程。

其中真正需要判断的只有一个：**`4656690 duck-control: an adopted knee gets its homing offset`**。
它把"每颗舵机应有的零点偏移"加进寄存器检查，值来自 `microduck_runtime` 的
`setup_motor_rpi.py`——**膝 (13, 23) 是 −512，其余为 0**——而那个检查会**写**它认为不对的寄存器。

**本目录不带这个机制**：

* 那两个膝上的 `-512` 是 **XL330 的标定值**。HLS/HD-1910-C001 的对应寄存器是 `OFS_L`
  （地址 31，`duck-control/src/feetech::reg` 里已定义但未使用），而 C001 的零点**从来没有在
  真机上测过**。
* 这个检查是**写**操作。用未测量的期望值去"纠正"，会把每颗舵机现有的零点静默改成 0——
  它正好是「上车前必须知道」一节里那件未决事项，而且会**掩盖**问题而不是暴露它。

所以这次同步在 `duck-control/src/bus.rs`（`check_registers_of`）与 `model.rs` 里留下的是
**说明性注释而不是代码**，`docs/design/robotd-design.md` 里那句也标注了"上游有、本目录不带"。
等零点标定做完（测出各关节的 `OFS_L` 期望值），再把上游那个检查按 C001 的数值补回来。

顺带白拿的两条与硬件无关的修复：

* **`e463144`**：`padd` / `mediad` 不再 `Wants=robotd.service`。在这之前，`mediad` 因摄像头
  问题每 5 秒重启一次，每次重启都把 `robotd` 拉起来——`systemctl stop robotd` 五秒后失效。
  本目录现在也带上了；但**停 `robotd` 仍然是必须的**（它用 `TIOCEXCL` 持有总线），
  只是不必再先停 `mediad`。
* **`16f7061`**：`robotctl logs`，一个不需要 sudo 的 journal 尾巴（`configd` 侧 +5 行）。

