# Microduck的一些PCB补充

> 这不是官方Microduck仓库，<a href="https://github.com/pollen-robotics/microduck">官方仓库</a>，感谢他们开源。本项目是复刻所需硬件PCB补充。

主要包括：
- **imu_to_dxl**：<a href="hardware/imu_to_dxl">身体传感器板</a>、<a href="firmware/v1">对应的固件</a>和<a href="firmware/v1/host">上位机测试软件</a>
- **banana_pcb**：<a href="hardware/banana_pcb">电池上方的转接小板</a>
- **dxl_hub**：<a href="hardware/dxl_hub">鸭鸭身体中的集线器板</a>
- **飞特舵机调试软件**：<a href="software/hls_servo_debugger">飞特舵机调试软件</a>

克隆的时候，固件编译需要增加--recurse-submodules参数，否则不会下载GDLib
```bash
git clone --recurse-submodules https://github.com/jyg9/microduck-community-kit.git
```

## 其他说明
 - 本项目在Linux平台开发，其他平台暂未做适配，可能存在兼容性问题
 - imu_to_dxl样品委外贴片成本接近1千元，还剩余一些imu_to_dxl（v0.1版本），可在<a href="https://item.taobao.com/item.htm?ft=t&id=1085185543605">我的店铺购</a>买，谢谢。
