# 用于Microduck复刻的一些PCB补充

> 这不是官方Microduck仓库，<a href="https://github.com/pollen-robotics/microduck">官方仓库</a>在这里，感谢他们开源。本项目是复刻所需硬件PCB补充。

主要包括：
- **imu_to_dxl**：<a href="hardware/imu_to_dxl">身体传感器板</a>、<a href="firmware/v1">对应的固件</a>和<a href="firmware/v1/host">上位机测试软件</a>。特点：高度接近原版设计、高可靠设计、固件可在线升级。其中，飞特1910舵机官方代码<a href="firmware/v1/patches">参考补丁</a>在这里
- **banana_pcb**：<a href="hardware/banana_pcb">电池上方的转接小板</a>
- **dxl_hub**：<a href="hardware/dxl_hub">鸭鸭身体中的集线器板</a>
- **飞特舵机调试软件**：<a href="software/hls_servo_debugger">飞特舵机调试软件</a>
- **microduck_feetech**：<a href="software/microduck_feetech">官方机器人软件（robotd 等）的飞特舵机适配版</a>——基于官方 `microduck` 仓库当前 `main`（`f0d934e`）打上飞特总线补丁的完整源码，可直接编译运行；补丁与逐文件说明见 <a href="firmware/v1/patches/microduck_feetech_patch.md">firmware/v1/patches</a>

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

## 其他说明
 - 项目主要使用了中国国内易于采购的飞特1910舵机，同时兼容原版XL330
 - 本项目在Linux平台开发，不能保证其他平台兼容性
 - 硬件是纯手工绘制，软件全部由AI编写和调试
 - imu_to_dxl如有需要，可在<a href="https://item.taobao.com/item.htm?ft=t&id=1085185543605">我的店铺</a>购买。（PS：第一版安装孔做小了0.1mm🙃，不过不太影响安装，仓库内设计文件已经修复）
 - 如果觉得项目不错，欢迎小额打赏😀，感谢各位老板支持。

<img src="./docs/alipay.jpg" alt="3d view" width="100"/>
