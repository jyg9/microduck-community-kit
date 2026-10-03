# 用于Microduck复刻的一些PCB补充

> 这不是官方Microduck仓库，<a href="https://github.com/pollen-robotics/microduck">官方仓库</a>在这里，感谢他们开源。本项目是复刻所需硬件PCB补充。

微信讨论群：

<img src="./docs/wechat.jpg" alt="3d view" width="200"/>

主要包括：
- **imu_to_dxl**：<a href="hardware/imu_to_dxl">身体传感器板</a>、<a href="firmware/v1">对应的固件</a>和<a href="firmware/v1/host">上位机测试软件</a>。特点：高度接近原版设计、高可靠设计、固件可在线升级。
- **banana_pcb**：<a href="hardware/banana_pcb">电池上方的转接小板</a>
- **dxl_hub**：<a href="hardware/dxl_hub">鸭鸭身体中的集线器板</a>
- **飞特舵机调试软件**：<a href="software/hls_servo_debugger">飞特舵机调试软件</a>
- **microduck_feetech**：<a href="software/microduck_feetech">官方机器人软件（robotd 等）的飞特舵机适配版</a>——官方 `microduck` 的完整源码，基于 `f0d934e`（2026-09-28）打上飞特总线补丁，并在 2026-10-02 同步了上游 `main` 的 10 个提交，可直接编译运行；


接线图：
```text
                               🦆 头部 (Head)
                                    │
                                    │ 舵机线
                                    ▲
 🔋 电池 (Battery)            ┌───────────┐
      │                      │           │
      │ 香蕉头                │  dxl_hub  │
      ▼                      │  (腹部)    │
 🍌 banana_pcb ─────────────>│           │
      (正负电源线)             └─┬───┬───┬─┘
                                │   │   │
                 ┌──────────────┘   │   └──────────────┐
                 │ 舵机线            │ 舵机线            │ 舵机线
                 ▼                  ▼                  ▼
            🦿 左腿            ⚖️ IMU            🦿 右腿
                             (imu_to_dxl，跨部内)
```

如需要编译固件，克隆的时候需要增加--recurse-submodules参数，否则不会下载GDLib：
```bash
git clone --recurse-submodules https://github.com/jyg9/microduck-community-kit.git
```

固件升级方法：
- 1、SFTP复制本项目<a href="firmware/v1/host">主机工具</a>下的`upgrade.py`、`package.py`、`bus.py`到目标电脑。目标电脑可也是鸭子后台，也可以是电脑接USB转舵机TTL。
- 2、从项目Releases下载编译好的固件，两个文件`gd32f303cc_imu_to_dxl_slot_a.ipkg`和`gd32f303cc_imu_to_dxl_slot_b.ipkg`，同样复制到目标电脑。
- 3、执行升级指令：
```bash
# 看目前节点版本号等信息，需不需要升级，提示缺依赖使用pip3 install <包名>安装
./upgrade.py status build/gd32f303cc_imu_to_dxl_slot_a.ipkg

# 升级指令，使用自动识别串口
./upgrade.py upgrade gd32f303cc_imu_to_dxl_slot_a.ipkg

# 升级指令，指定一些参数（高级，与上面二选一）
./upgrade.py upgrade gd32f303cc_imu_to_dxl_slot_a.ipkg --port /dev/ttyS2 --baud 1000000 --protocol auto --yes

# 如果提示slot_a占用，更换上面文件名为slot_b，目前脚本还不是十分智能

# 再检查升级后节点版本，版本号一致就是升级成功
./upgrade.py status
```

## 其他说明
 - 项目主要使用了中国国内易于采购的飞特1910舵机，同时兼容原版XL330
 - 本项目在Linux平台开发，不能保证其他平台兼容性
 - 硬件是纯手工绘制，软件全部由AI编写和调试，已经消耗1G+ Token
 - imu_to_dxl如有需要，可在<a href="https://item.taobao.com/item.htm?ft=t&id=1085185543605">我的店铺</a>购买。
 - 如果觉得项目不错，欢迎小额打赏😀，或者点个星⭐再走？

<img src="./docs/alipay.jpg" alt="3d view" width="100"/>
