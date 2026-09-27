# 用于Microduck复刻的一些PCB补充

> 这不是官方Microduck仓库，<a href="https://github.com/pollen-robotics/microduck">官方仓库</a>在这里，感谢他们开源。本项目是复刻所需硬件PCB补充。

主要包括：
- **imu_to_dxl**：<a href="hardware/imu_to_dxl">身体传感器板</a>、<a href="firmware/v1">对应的固件</a>和<a href="firmware/v1/host">上位机测试软件</a>。特点：与原版保持一致、高可靠设计、固件可在线升级。
- **banana_pcb**：<a href="hardware/banana_pcb">电池上方的转接小板</a>，需要引出正负电源线焊接到下面dxl_hub板子上。
- **dxl_hub**：<a href="hardware/dxl_hub">鸭鸭身体中的集线器板</a>
- **飞特舵机调试软件**：<a href="software/hls_servo_debugger">飞特舵机调试软件</a>

接线图：
                          🦆 MicroDuck 头部 (Head)
                                │
                                │ 1x 舵机线 (向下)
                                ▼
 ┌───────────────┐        ┌───────────┐        ┌───────────────┐
 │ 🔋 电池       │        │           │        │ 🦿 左腿       │
 │ (后背上方)    │        │  dxl_hub  │<=======| 1x 舵机线     │
 └──────┬────────┘        │  (腹部)   │        └───────────────┘
        │                 │           │
        │ 电源输入        │  舵机单串 │        ┌───────────────┐
        ▼                 │  口总线   │<=======| 🦿 右腿       │
 ┌───────────────┐        │  并联分配 │        │ 1x 舵机线     │
 │  banana_pcb   │=======>│  各路电源 │        └───────────────┘
 │  (电池转接板) │ 2x电源 │           │
 └───────────────┘ 线     └─────▲─────┘
                                │
                                │ 1x 舵机线 (向上)
                                │
 ┌───────────────┐              │
 │ imu_to_dxl    │==============┘
 │ (跨部内，仿舵机小板) 
 └───────────────┘

如需要编译固件，克隆的时候需要增加--recurse-submodules参数，否则不会下载GDLib：
```bash
git clone --recurse-submodules https://github.com/jyg9/microduck-community-kit.git
```

## 其他说明
 - 项目使用了飞特舵机1910，同时兼容原版XL330
 - 不是最终版本。目前我的外壳还没到，未组装整机调试，后续可能会有更新
 - 本项目在Linux平台开发，不能保证其他平台兼容性
 - 硬件是纯手工绘制，软件全部由AI编写和调试
 - imu_to_dxl样品委外贴片开机费用成本较高，还剩余一些imu_to_dxl，如有需要可在<a href="https://item.taobao.com/item.htm?ft=t&id=1085185543605">我的店铺购</a>买，一起分摊一下开发成本。（第一版安装孔做小了0.1mm🙃，不过不太影响安装，目前设计文件已经修复）
 - 如果觉得项目不错，欢迎小额打赏😀，感谢各位老板支持。

<img src="./docs/alipay.jpg" alt="3d view" width="100"/>
