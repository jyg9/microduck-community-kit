# microduck + 飞特舵机（FeeTech SCS/HLS）适配说明

这个目录是**官方 [microduck](https://github.com/pollen-robotics/microduck) 仓库的完整源码**
（`main` 分支，基线提交 `f0d934e`，2026-09-28），上面已经打好了把电机总线从
Dynamixel XL330 换成**飞特 HD-1910-C001（SCS/HLS 协议）**的补丁。可以直接编译、运行
`robotd` 等程序，不需要再手工改一行代码。

配套硬件与固件（IMU 小板 `imu_to_dxl`）在本仓库的 <a href="../../hardware/imu_to_dxl">`hardware/imu_to_dxl`</a>、
<a href="../../firmware/v1">`firmware/v1/`</a> 下。

## 补丁在哪、怎么核对

* 补丁本身：<a href="../../firmware/v1/patches/microduck-feetech.patch">`firmware/v1/patches/microduck-feetech.patch`</a>
  （对上一条基线 `git apply` 可干净应用）
* 逐文件说明、厂商寄存器依据、已验证 / 必须上真机确认的清单、未决事项：
  <a href="../../firmware/v1/patches/microduck_feetech_patch.md">`firmware/v1/patches/microduck_feetech_patch.md`</a>
* 协议模块的单文件副本：<a href="../../firmware/v1/patches/feetech.rs">`firmware/v1/patches/feetech.rs`</a>
  （与 `duck-control/src/feetech.rs` 逐字节一致）

主要改动一句话：`duck-control` 的 `DynamixelIo` → `FeetechIo`，不再依赖 `rustypot`，
自己实现飞特帧收发，坐标/速度/电流按厂商的**符号-幅值**编码解码。

## 编译与自测

需要 Rust 工具链与系统 `serialport` 依赖。**本目录只是源码，不含构建产物**（`target/`）。

```bash
cd software/microduck_feetech

# 只编译总线与守护进程
cargo check -p duck-control -p robotd

# 总线协议与控制的单元测试（不需要任何硬件）
cargo test -p duck-control
```

已知情况（2026-09-29，本机 Ubuntu x86-64）：

* `cargo test -p duck-control`：**108 passed / 0 failed**
* `cargo test -p robotd-params`：**93 passed / 0 failed**
* `cargo test -p robotd`：有 2 个用例失败，原因是本机 ONNX Runtime / 模型文件差异，**与总线无关**
  （在未打补丁的同一基线提交上同样失败），详见补丁说明第一节
* `cargo check --workspace --all-targets`：需要系统 gstreamer（`mediad`/`uyvy`），
  本机没装，因此只验证了 `duck-control` 与 `robotd`

## 上车前必须知道

这套补丁的协议层有厂商库与本项目调试器两份证据，但**没有在装了 15 台舵机的真机上跑过**：
关节正方向、零点、增益（`deploy/robotd.toml` 里的 `gain = 200` 是按 XL330 标度调的）、
电压/温度/电流标度都需要实机复测。收养新舵机（改 ID）的流程是有破坏性的操作，
建议先用备用舵机验证。清单见补丁说明第四、五节。

## 与上游的关系

* 上游仓库：<https://github.com/pollen-robotics/microduck>（本项目只是复刻配套，非官方）
* 上游的 `docs/design/*.md` 里仍有按 XL330 叙述的段落，本目录只同步了协议与命名的部分，
  未重写的范围在补丁说明未决事项 7 中列出；**冲突时以补丁说明为准**。
* 需要回到官方总线时：在本目录里 `git apply -R ../../firmware/v1/patches/microduck-feetech.patch`
  （补丁文件在本仓库的 `firmware/v1/patches/`，不在本目录内）。
