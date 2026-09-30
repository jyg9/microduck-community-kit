# microduck + 飞特舵机（FeeTech SCS/HLS）适配说明

这个目录是**官方 [microduck](https://github.com/pollen-robotics/microduck) 仓库的完整源码**
（`main` 分支，基线提交 `f0d934e`，2026-09-28），上面已经打好了把电机总线从
Dynamixel XL330 换成**飞特 HD-1910-C001（SCS/HLS 协议）**的补丁。可以直接编译、运行
`robotd` 等程序，不需要再手工改一行代码。

配套硬件与固件（IMU 小板 `imu_to_dxl`）在本仓库的 <a href="../../hardware/imu_to_dxl">`hardware/imu_to_dxl`</a>、
<a href="../../firmware/v1">`firmware/v1/`</a> 下。

## 补丁在哪、怎么核对

* 补丁本身：<a href="../../firmware/v1/patches/microduck-feetech.patch">`firmware/v1/patches/microduck-feetech.patch`</a>
  （对上一条基线 `git apply` 可干净应用，反向亦可）
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
  未重写的范围在补丁说明未决事项 7 中列出；**冲突时以补丁说明为准**。
* 需要回到官方总线时：在本目录里 `git apply -R ../../firmware/v1/patches/microduck-feetech.patch`
  （补丁文件在本仓库的 `firmware/v1/patches/`，不在本目录内）。
