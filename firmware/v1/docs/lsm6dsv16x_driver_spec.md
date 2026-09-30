# LSM6DSV16X 驱动规格（imu_to_dxl v1 / GD32F303CC）

面向实现：把 ST **LSM6DSV16X**（SPI0，PA4=CS/PA5=SCK/PA6=MISO/PA7=MOSI，INT1=PB0，
INT2=PB1）接到现有 100 Hz 主循环上，产出与 `src/imu.h` 一致的 20 字节块。

  本文档每个寄存器号/位域/常数都给出 `§节号 / Table 号`。凡数据手册文本无法确认的，
  一律标注 **【不确定/需查原图】**，不猜。
* 交叉参考（**不作为数字依据**）：`src/imu.h` / `src/imu_sim.c` / `src/imu_math.h`
  （20 字节块与 half 编码的既有约定）、`src/board.h`（引脚与标志位）；
  `TELEM_FLAG_*` 的语义见 `docs/imu_to_dxl_protocol.md` §6.4。
* 目标 20 字节块（`src/imu.h`、`board.h TELEM_LEN=20`）：

| 字节 | 内容 |
|---|---|
| 0..6 | `gyro x/y/z` int16 LE，**chip frame**，±500 dps @ **17.5 mdps/LSB** |
| 6..12 | `quat x/y/z` IEEE binary16，`w = √(1−x²−y²−z²)` |
| 12..18 | `accel x/y/z` int16 LE，**chip frame**，±4 g @ **0.122 mg/LSB** |
| 18 | `counter` u8 |
| 19 | `flags`（`TELEM_FLAG_*`） |

---

## 1. 身份与接口

### 1.1 身份

| 项 | 值 | 出处 |
|---|---|---|
| `WHO_AM_I` 地址 | **0x0F** | Table 24（寄存器地址映射表） |
| `WHO_AM_I` 期望值 | **0x70** | §9.13 "Its value is fixed at 70h"；Table 49 位图 `01110000`；Table 24 默认列 `01110000` |
| 辅助 SPI 的同名寄存器 `SPI2_WHO_AM_I (0Fh)` | 也是 **0x70** | §11.1 "Its value is fixed at 70h" |
| 读类型 | R（只读） | Table 24 / §9.13 |

**注意**：主接口与辅助 SPI（SPI2）的 `WHO_AM_I` 都是 `0x70`，因此 `WHO_AM_I` **不能**
区分"读到的是主接口还是辅接口"。本板只接主接口（mode 1，见 §1.5），读到的就是主接口页。

### 1.2 SPI 电气与帧协议

| 项 | 值 / 结论 | 出处 |
|---|---|---|
| 角色 | SPI **从机**（MCU 为主机），全双工 4 线 | §5.1.3 "The SPI on the LSM6DSV16X is a bus slave" |
| 帧宽 / 位序 | 8 bit，**MSB first** | §5.1.3 bit0=RW、bit1-7=AD(6:0)、bit8-15=DI/DO(7:0) |
| SPI 模式 | **Mode 3**（CPOL=1, CPHA=1）：SPC 空闲为高；SDI/SDO 在 SPC 
**下降沿**驱动、**上升沿**采样 | §5.1.3 明文字；GD32 宏 `SPI_CK_PL_HIGH_PH_2EDGE`。Mode 0 亦被支持（Table 7 给出 mode 0 的 `tsu(CS)`），但本设计用 Mode 3 |
| 最短帧 | **16 个 SPC 脉冲**（1 字节地址 + 1 字节数据）；多字节按 8 位块追加 | §5.1.3；§5.1.3.1；§5.1.3.2 |
| 读位 | bit0 = **1**（READ） | §5.1.3.1 "bit 0: READ bit. The value is 1." |
| 写位 | bit0 = **0**（WRITE） | §5.1.3.2 "bit 0: WRITE bit. The value is 0." |
| 地址字段 | bit1-7 = `AD(6:0)`，**7 位**（0x00–0x7F） | §5.1.3 |
| 地址自增 | `CTRL3(0x12).IF_INC = 1` 时多字节访问地址**逐块自增**；`=0` 时地址保持 | §5.1.3；§9.16 / Table 57 |
| **dummy byte** | **不需要**。文本中 **没有** dummy byte 的说明（全文 grep "dummy" 无命中）：读帧 bit8-15 直接就是 `DO(7:0)`，"the chip drives SDO at the start of bit 8" | §5.1.3；§5.1.3.1 与 Figure 11/12 |
| 最大 SPC 频率 | **10 MHz** | Table 7 `fc(SPC)` max 10 MHz |
| 时序余量 | `tc(SPC) ≥ 100 ns`、`thigh/tlow ≥ 45 ns`、`tsu(CS) ≥ 5 ns`（mode 3）、`th(CS) ≥ 20 ns`、`tv(SO) 15–25 ns`、`tdis(SO) ≤ 50 ns`、`Cload ≤ 100 pF` | Table 7 |
| 3 线 / 4 线 | `IF_CFG(0x03).SIM`：**0 = 4 线**（本板）、1 = 3 线 | §9.3 / Table 30 |
| CS 极性/含义 | CS 低有效；"1: SPI idle mode / I²C…; 0: SPI communication mode" | §3.1 / Table 2（Pin 12） |

**【不确定/需查原图】**：Figure 10/11/12 的位级波形在文本抽取中只剩标签，未逐条比对。
Mode 3 的结论来自 §5.1.3 的文字（"SPC … is stopped high when CS is high"、"driven at the
falling edge of SPC and should be captured at the rising edge"），**台面仍需用逻辑分析仪
在 Mode 3 下确认一次**（见 §9）。

**SPI 时钟选择（本板数字）**：`system_gd32f30x.c` 设 AHB=SYSCLK、**APB2 = AHB/1 = 120 MHz**
（`RCU_APB2_CKAHB_DIV1`），SPI0 挂在 APB2。GD32 分频只有 2 的幂：

| `spi_parameter_struct.prescale` | SPI0 SCLK | 结论 |
|---|---|---|
| `SPI_PSC_32` | **3.75 MHz** | 点亮/排障档，最稳 |
| `SPI_PSC_16` | **7.5 MHz** | **推荐工作档**（10 MHz 上限内，留 25% 余量） |
| `SPI_PSC_8` | 15 MHz | **超规格，禁止** |

一次寄存器读只需 2 字节（地址+数据），一次 12 字节突发读 13 字节；7.5 MHz 下
13 字节 ≈ 13.9 µs，对 100 Hz 主循环无影响。

### 1.3 上电、复位与首次访问

| 步骤 | 内容 | 依据 |
|---|---|---|
| 上电等待 | **建议 ≥ 10 ms** 后才做第一次寄存器访问 | **【不确定/需查原图】** 数据手册**未**给出 POR/boot 时间；§4.3 Table 6 只给了温度稳定时间 500 µs。10 ms 是本规范的保守取值，必须台面核对（§9） |
| 软件复位 | `CTRL3 (0x12) = 0x01`（`SW_RESET=1`） | §9.16 / Table 57："Software reset, resets all control registers to their default value. This bit is **automatically cleared**." |
| 复位等待 | 轮询 `CTRL3(0x12)` 直到 **bit0 == 0**，超时 **50 ms** → 报错 | 同上（自清位），具体时长数据手册未给 **【不确定/需查原图】** |
| 复位影响范围 | `SW_RESET` 把**所有控制寄存器**恢复默认；但 **`PIN_CTRL (0x02)` 与 `IF_CFG (0x03)` 不被软件复位复位** | §9.2 "This register is not reset during the software reset procedure"；§9.3 同句；§9.16 |
| `BOOT` 位 | `CTRL3(0x12).BOOT`（bit7）= "Reboots memory content"，自清；**本驱动不用**（上电已自动装载出厂校准） | §9.16 / Table 57；§8 尾注 "automatically restored when the device is powered up" |
| 全局复位 | `FUNC_CFG_ACCESS(0x01).SW_POR`（bit2）= "Global reset of the device"；**本驱动不用** | §9.1 / Table 25–26 |
| 复位后顺序 | 必须先 `SW_RESET`，再写 `IF_CFG`/`PIN_CTRL`（否则被复位冲掉） | 由上两条 |

### 1.4 IF_INC 与块更新（BDU）

| 位 | 地址 | 含义 | 本驱动取值 | 出处 |
|---|---|---|---|---|
| `BDU` | `CTRL3(0x12)` bit6 | 1 = 输出寄存器在 LSB/MSB 读完前不更新（块数据更新） | **1** | §9.16 / Table 57 |
| `IF_INC` | `CTRL3(0x12)` bit2 | 1 = 多字节访问地址自增 | **1** | §9.16 / Table 57 |
| `CTRL3` 复位默认值 | — | **0x44**（= `BDU=1` + `IF_INC=1`，其余为 0） | 显式写 `0x44` 兜底（Board 断电后取值仍应是 0x44，显式写一次便于校验） | Table 24 默认列 `01000100`；Table 56 位图 |

`CTRL3` 位图（Table 56，bit7→bit0）：
`BOOT | BDU | 0(必须0) | 0(必须0) | 0(必须0) | IF_INC | 0(必须0) | SW_RESET`

### 1.5 引脚（mode 1，主 4 线 SPI）

板级网表（`board.h`：`IMU_CS_PIN=PA4`、`IMU_SCK_PIN=PA5`、`IMU_MISO_PIN=PA6`、
`IMU_MOSI_PIN=PA7`、`IMU_INT1_PIN=PB0`、`IMU_INT2_PIN=PB1`）对照 Table 2：

| 芯片脚 | 芯片信号 | 板级连接 |
|---|---|---|
| 12 | `CS` | PA4（**GPIO 推挽，软件片选，低有效**） |
| 13 | `SCL / SPC` | PA5（`GPIO_MODE_AF_PP`） |
| 1 | `SDO/SA0` | PA6（`GPIO_MODE_AF_PP`） |
| 14 | `SDA / SDI` | PA7（`GPIO_MODE_AF_PP`） |
| 4 | `INT1` | PB0 |
| 9 | `INT2` | PB1 |

**NSS 注意事项（GD32）**：`main.c board_gpio_init()` 现在把 PA4 配为推挽输出并置高、
PA5/6/7 为浮空输入。驱动必须把 PA5/6/7 改成 `GPIO_MODE_AF_PP` + `GPIO_OSPEED_50MHZ`，
`spi_parameter_struct.nss = SPI_NSS_SOFT`，并在 `spi_enable()` 后调用
`spi_nss_internal_high(SPI0)`（软件 NSS 内部电平必须为高，主机才会发时钟），
片选仍由 PA4 的 GPIO 控制。

### 1.6 读写原语（可直接落 C）

```c
/* 读：一次 CS 低 → 地址字节(0x80|addr) → 连续收 len 字节。无 dummy byte。 */
static void lsm_rd(uint8_t addr, uint8_t *buf, uint16_t len)
{
    uint16_t i;
    gpio_bit_reset(IMU_SPI_GPIO, IMU_CS_PIN);          /* CS 低 */
    (void)spi_i2s_data_receive(SPI0);                  /* 清 RBNE 残留 */
    spi_i2s_data_transmit(SPI0, (uint16_t)(0x80U | addr));
    while (RESET == spi_i2s_flag_get(SPI0, SPI_FLAG_RBNE)) { }   /* 地址字节回读，丢弃 */
    (void)spi_i2s_data_receive(SPI0);
    for (i = 0U; i < len; i++) {
        spi_i2s_data_transmit(SPI0, 0x00U);
        while (RESET == spi_i2s_flag_get(SPI0, SPI_FLAG_RBNE)) { }
        buf[i] = (uint8_t)spi_i2s_data_receive(SPI0);
    }
    while (RESET != spi_i2s_flag_get(SPI0, SPI_FLAG_TRANS)) { }  /* 等最后一拍移完 */
    gpio_bit_set(IMU_SPI_GPIO, IMU_CS_PIN);            /* CS 高 */
}

/* 写：一次 CS 低 → 地址字节(addr & 0x7F) → 数据字节。 */
static void lsm_wr(uint8_t addr, uint8_t val)
{
    gpio_bit_reset(IMU_SPI_GPIO, IMU_CS_PIN);
    (void)spi_i2s_data_receive(SPI0);
    spi_i2s_data_transmit(SPI0, (uint16_t)(addr & 0x7FU));
    while (RESET == spi_i2s_flag_get(SPI0, SPI_FLAG_TBE)) { }
    (void)spi_i2s_data_receive(SPI0);
    spi_i2s_data_transmit(SPI0, (uint16_t)val);
    while (RESET == spi_i2s_flag_get(SPI0, SPI_FLAG_TBE)) { }
    while (RESET != spi_i2s_flag_get(SPI0, SPI_FLAG_TRANS)) { }
    gpio_bit_set(IMU_SPI_GPIO, IMU_CS_PIN);
}
```

所用 GD32 API（`GD32F30x_Firmware_Library_V3.0.3/…/gd32f30x_spi.h`）：
`spi_struct_para_init` / `spi_init` / `spi_enable` / `spi_nss_internal_high` /
`spi_i2s_data_transmit` / `spi_i2s_data_receive` / `spi_i2s_flag_get`。

### 1.7 ⚠ 页别名陷阱（最容易写错的地方）

`FUNC_CFG_ACCESS(0x01).EMB_FUNC_REG_ACCESS`（**bit7**，§9.1 / Table 25-26）
= 1 时，**同一批地址 0x02…0x73 会指向完全不同的寄存器**。本驱动涉及的重叠地址：

| 地址 | 主页（`EMB_FUNC_REG_ACCESS=0`） | 嵌入式功能页（=1） |
|---|---|---|
| 0x02 | `PIN_CTRL` | `PAGE_SEL` |
| 0x07 | `FIFO_CTRL1` | `EMB_FUNC_EXEC_STATUS` |
| 0x08 | `FIFO_CTRL2` | `PAGE_ADDRESS` |
| 0x09 | `FIFO_CTRL3` | `PAGE_VALUE` |
| 0x0A | `FIFO_CTRL4` | `EMB_FUNC_INT1` |
| 0x17 | `CTRL8` | `PAGE_RW` |
| **0x5E** | **`MD1_CFG`** | **`SFLP_ODR`** |
| 0x66 | `UI_SPI2_SHARED_1` | `EMB_FUNC_INIT_A` |

出处：Table 24（主页映射）与 Table 262（嵌入式功能页映射）。
**实现要求**：写 `SFLP_ODR(0x5E)` / `EMB_FUNC_EN_A(0x04)` / `EMB_FUNC_FIFO_EN_A(0x44)` /
`EMB_FUNC_INIT_A(0x66)` 之前先写 `FUNC_CFG_ACCESS=0x80`，写完后**必须回读
`FUNC_CFG_ACCESS` 确认 == 0x80**，收尾再写回 `0x00`；页切换失败是最可能的静默 bug。

---

## 2. 寄存器映射表（本驱动触及的全部寄存器）

### 2.1 主页（`FUNC_CFG_ACCESS(0x01) = 0x00`）

| 名称 | 地址 | 复位/默认值 | 本驱动用到的位域 | 出处 |
|---|---|---|---|---|
| `FUNC_CFG_ACCESS` | 0x01 | `0x00` | bit7 `EMB_FUNC_REG_ACCESS`（1=进嵌入式功能页） | §9.1 / Table 25-26；Table 24 |
| `PIN_CTRL` | 0x02 | `0x23` | 不改（SDO 上拉等；**不被 SW_RESET 复位**） | §9.2 / Table 27-28；Table 24 |
| `IF_CFG` | 0x03 | `0x00` | bit2 `SIM`（0=4线）、bit4 `H_LACTIVE`、bit3 `PP_OD`、bit0 `I2C_I3C_disable` | §9.3 / Table 29-30；Table 24 |
| `FIFO_CTRL1` | 0x07 | `0x00` | bit7:0 `WTM[7:0]`（水位，1 LSB = 1 个字 = TAG 1B + 6B） | §9.5 / Table 33-34 |
| `FIFO_CTRL2` | 0x08 | `0x00` | bit7 `STOP_ON_WTM`（0=不限深）；bit1:0 `UNCOMPR_RATE` | §9.6 / Table 35-36 |
| `FIFO_CTRL3` | 0x09 | `0x00` | bit7:4 `BDR_GY[3:0]`、bit3:0 `BDR_XL[3:0]`（方案 A 全 0） | §9.7 / Table 37-38 |
| `FIFO_CTRL4` | 0x0A | `0x00` | bit2:0 `FIFO_MODE[2:0]`（110=continuous，000=bypass） | §9.8 / Table 39-40 |
| `INT1_CTRL` | 0x0D | `0x00` | bit3 `INT1_FIFO_TH`、bit4 `INT1_FIFO_OVR`、bit1 `INT1_DRDY_G`、bit0 `INT1_DRDY_XL` | §9.11 / Table 45-46 |
| `INT2_CTRL` | 0x0E | `0x00` | bit3 `INT2_FIFO_TH`、bit4 `INT2_FIFO_OVR` 等（本驱动置 0） | §9.12 / Table 47-48 |
| `WHO_AM_I` | 0x0F | **`0x70`** | 只读全字节 | §9.13 / Table 49 |
| `CTRL1` | 0x10 | `0x00` | bit6:4 `OP_MODE_XL[2:0]`、bit3:0 `ODR_XL[3:0]` | §9.14 / Table 50-52 |
| `CTRL2` | 0x11 | `0x00` | bit6:4 `OP_MODE_G[2:0]`、bit3:0 `ODR_G[3:0]` | §9.15 / Table 53-55 |
| `CTRL3` | 0x12 | **`0x44`** | bit7 `BOOT`、bit6 `BDU`、bit2 `IF_INC`、bit0 `SW_RESET` | §9.16 / Table 56-57 |
| `CTRL4` | 0x13 | `0x00` | bit3 `DRDY_MASK`（可选：滤波稳定前屏蔽 DRDY） | §9.17 / Table 58-59 |
| `CTRL6` | 0x15 | `0x00` | bit6:4 `LPF1_G_BW[2:0]`、bit3:0 `FS_G[3:0]` | §9.19 / Table 62-63 |
| `CTRL7` | 0x16 | `0x00` | bit0 `LPF1_G_EN`（本驱动保持 0 = LPF1 关） | §9.20 / Table 65-66 |
| `CTRL8` | 0x17 | `0x00` | bit1:0 `FS_XL[1:0]`、bit7:5 `HP_LPF2_XL_BW`、bit3 `XL_DualC_EN` | §9.21 / Table 67-68 |
| `CTRL9` | 0x18 | `0x00` | bit4 `HP_SLOPE_XL_EN`、bit3 `LPF2_XL_EN`、bit0 `USR_OFF_ON_OUT`（全部保持 0） | §9.22 / Table 70-71 |
| `FIFO_STATUS1` | 0x1B | output | bit7:0 `DIFF_FIFO[7:0]` | §9.25 / Table 76-77 |
| `FIFO_STATUS2` | 0x1C | output | bit7 `FIFO_WTM_IA`、bit6 `FIFO_OVR_IA`、bit5 `FIFO_FULL_IA`、bit4 `COUNTER_BDR_IA`、bit3 `FIFO_OVR_LATCHED`、bit0 `DIFF_FIFO_8` | §9.26 / Table 78-79 |
| `ALL_INT_SRC` | 0x1D | output | bit7 `EMB_FUNC_IA`（仅诊断用） | §9.27 / Table 80-81 |
| `STATUS_REG` | 0x1E | output | bit1 `GDA`、bit0 `XLDA`、bit2 `TDA` | §9.28 / Table 82-83 |
| `OUT_TEMP_L/H` | 0x20/0x21 | output | 16 bit 二补码，`T = raw/256 + 25 °C` | §9.29 / Table 84-86；Table 6 |
| `OUTX_L_G`…`OUTZ_H_G` | 0x22–0x27 | output | gyro X/Y/Z，16 bit 二补码 LE | §9.30–9.32 / Table 87-95；Table 24 |
| `OUTX_L_A`…`OUTZ_H_A` | 0x28–0x2D | output | accel X/Y/Z，16 bit 二补码 LE | §9.33–9.35 / Table 96-104；Table 24 |
| `MD1_CFG` | 0x5E | `0x00` | bit1 `INT1_EMB_FUNC`（本驱动不用；**与 `SFLP_ODR` 同地址，见 §1.7**） | §9.65 / Table 175-176 |
| `FIFO_DATA_OUT_TAG` | 0x78 | output | bit7:3 `TAG_SENSOR[4:0]`、bit2:1 `TAG_CNT[1:0]`、bit0 保留 | §9.84 / Table 216-217 |
| `FIFO_DATA_OUT_X_L`…`Z_H` | 0x79–0x7E | output | 每个 FIFO 字的 6 字节载荷 | §9.85–9.87 / Table 219-224；Table 24 |

### 2.2 嵌入式功能页（`FUNC_CFG_ACCESS(0x01) = 0x80`）

| 名称 | 地址 | 复位/默认值 | 本驱动用到的位域 | 出处 |
|---|---|---|---|---|
| `PAGE_SEL` | 0x02 | `0x01` | bit7:4 `PAGE_SEL[3:0]`（本驱动不碰：默认 page 0） | §13.1 / Table 263-264；Table 262 |
| `EMB_FUNC_EN_A` | 0x04 | `0x00` | **bit1 `SFLP_GAME_EN`**（1=开 SFLP 六轴游戏旋转向量） | §13.2 / Table 265-266 |
| `EMB_FUNC_EXEC_STATUS` | 0x07 | output | bit1 `EMB_FUNC_EXEC_OVR`、bit0 `EMB_FUNC_ENDOP`（诊断） | §13.4 / Table 269-270 |
| `EMB_FUNC_STATUS` | 0x12 | output | **没有 SFLP 位**（只有 FSM_LC/SIGMOT/TILT/STEP_DET） | §13.13 / Table 287-288 |
| `EMB_FUNC_FIFO_EN_A` | 0x44 | `0x00` | **bit1 `SFLP_GAME_FIFO_EN`**；bit5 `SFLP_GBIAS_FIFO_EN`、bit4 `SFLP_GRAVITY_FIFO_EN` | §13.17 / Table 295-296 |
| `SFLP_ODR` | 0x5E | **`0x5B`** | bit6 与 bit1:0 **必须为 1**；bit5:3 `SFLP_GAME_ODR[2:0]` | §13.30 / Table 323-324 |
| `EMB_FUNC_INIT_A` | 0x66 | `0x00` | **bit1 `SFLP_GAME_INIT`**（1=重新初始化 SFLP） | §13.35 / Table 335-336 |

**不需要**的页机制：`PAGE_RW(0x17)` / `PAGE_ADDRESS(0x08)` / `PAGE_VALUE(0x09)`
（§13.16、§13.5、§13.6、§14 尾部的"写/读过程示例"）只用于**高级功能页**（page 0/1/2，
如 `SFLP_GAME_GBIAS*`）。SFLP 游戏旋转向量的使能/速率/FIFO 批处理**全部在上表
2.2 的直接寻址寄存器里**，本驱动不需要进入高级功能页。

### 2.3 只读、不要写的寄存器

`RESERVED 0x04-0x05`（主页）、`0x1F`、`0x3C-0x3F`、`0x4C-0x4E`、`0x60-0x61`、
`0x6C-0x6E`、`0x76-0x77`；以及所有位图标为 "This bit must be set to 0/1" 的位
（§8 尾注 + 各寄存器表脚注）。Table 24 脚注：**"Reserved registers must not be
changed. Writing to those registers may cause permanent damage to the device."**

---

## 3. 量程与 ODR 编码

### 3.1 数据手册灵敏度常数（用于校验我们的假设）

| 参数 | 量程 | 灵敏度（typ.） | 出处 |
|---|---|---|---|
| `LA_So` | ±2 g | 0.061 mg/LSB | Table 3 |
| `LA_So` | **±4 g** | **0.122 mg/LSB** | Table 3 |
| `LA_So` | ±8 g | 0.244 mg/LSB | Table 3 |
| `LA_So` | ±16 g | 0.488 mg/LSB | Table 3 |
| `G_So` | ±125 dps | 4.375 mdps/LSB | Table 3 |
| `G_So` | ±250 dps | 8.75 mdps/LSB | Table 3 |
| `G_So` | **±500 dps** | **17.50 mdps/LSB** | Table 3 |
| `G_So` | ±1000 dps | 35 mdps/LSB | Table 3 |
| `G_So` | ±2000 dps | 70 mdps/LSB | Table 3 |
| `G_So` | ±4000 dps | 140 mdps/LSB | Table 3 |

**结论**：`board.h` 的 `IMU_GYRO_MDPS_PER_LSB = 17.5f` 与 `IMU_ACCEL_MG_PER_LSB = 0.122f`
与 Table 3 完全一致，**换算路径一行都不用改**，主机 `imu.rs` 的
`GYRO_RAD_PER_LSB = 0.0175·π/180` 也继续成立。

**⚠ 必须知道的数字坑（不是错，是量程标称与 LSB 满量程不一致）**：

* 加速度：`0.122 mg/LSB × 32768 = 3997.7 mg ≈ ±4 g` → 理想匹配。
* 陀螺：`17.5 mdps/LSB × 32768 = 573.4 dps`，而标称满量程是 **±500 dps**
  （500 dps ÷ 17.5 mdps = **28571 counts**）。
  数据手册**未说明** 28571…32767 counts 区间是否仍线性。
  → `TELEM_FLAG_GYRO_SAT` 建议在 **|count| ≥ 28571**（= 标称 ±500 dps）时置位，
  而不是 32767；这个区间行为列入 §9 台面核对。

### 3.2 ODR 编码

加速度 ODR（Table 52，`ODR_XL[3:0]`）：0000=PD、0001=1.875（仅低功耗）、0010=7.5、
0011=15、0100=30、0101=60、**0110=120**、0111=240、1000=480、1001=960、1010=1.92k、
1011=3.84k、1100=7.68k Hz。
陀螺 ODR（Table 55，`ODR_G[3:0]`）：0000=PD、0010=7.5、0011=15、0100=30、0101=60、
**0110=120**、0111=240、1000=480、1001=960、1010=1.92k、1011=3.84k、1100=7.68k Hz。

**120 Hz 是两个通道都支持的档位**（`Table 3` 的 `LA_ODR`/`G_ODR` 列表里也有 120）。
不需要退档，也不需要 104/208 Hz 折中。

### 3.3 本驱动的精确编码值

| 寄存器 | 值 | 位域解码 | 效果 | 出处 |
|---|---|---|---|---|
| `CTRL1 (0x10)` | **`0x06`** | `OP_MODE_XL=000`（高性能）、`ODR_XL=0110` | accel 120 Hz，高性能（抗混叠滤波器有效，噪声 60 µg/√Hz） | §9.14 / Table 50-52；§6.2；Table 3 `An` |
| `CTRL8 (0x17)` | **`0x01`** | `FS_XL=01`、`HP_LPF2_XL_BW=000`、`XL_DualC_EN=0` | **±4 g**（0.122 mg/LSB） | §9.21 / Table 67-68 |
| `CTRL2 (0x11)` | **`0x06`** | `OP_MODE_G=000`（高性能）、`ODR_G=0110` | gyro 120 Hz，高性能（速率噪声 2.8 mdps/√Hz） | §9.15 / Table 53-55；§6.4；Table 3 `Rn` |
| `CTRL6 (0x15)` | **`0x02`** | `FS_G=0010`、`LPF1_G_BW=000` | **±500 dps**（17.50 mdps/LSB），LPF1 未使能 | §9.19 / Table 62-63 |
| `CTRL7 (0x16)` | `0x00` | `LPF1_G_EN=0` | 陀螺数字 LPF1 关闭 → 输出带宽只剩 LPF2：**49.4 Hz @ ODR=120 Hz** | §9.20 / Table 65-66；§6.9.2 / Table 21 |
| `CTRL9 (0x18)` | `0x00` | `HP_SLOPE_XL_EN=0`（低通路径）、`LPF2_XL_EN=0`（一级数字滤波输出） | accel 带宽 = **ODR/2 = 60 Hz**（不启用 HPF/LPF2，用户偏移块旁路） | §9.22 / Table 70-71；Table 69 脚注 1 |

**ODR=120 Hz vs 100 Hz 主循环的取舍（必须明说）**：

* 120 Hz 与 100 Hz 之比 = 1.2（非整数）。100 Hz 主循环每格平均拿到 1.2 个新样本，
  也就是**约 17 % 的样本被丢弃**：发布的最新样本"年龄"在 0…8.3 ms 之间抖动（均值 ≈4 ms）。
  后果：同一 20 字节块里的 quat（FIFO 里最新的一帧）与 gyro/accel（输出寄存器里的最新一帧）
  **可能相差最多 ~8.3 ms**。板子不做积分，所以这不会算错任何量；它只影响高速动态下的
  配对精度。若台面证明不能接受 → 走方案 B（三路都进 FIFO，按 `TAG_CNT` 配对，§5.3）。
* 备选 240 Hz：主循环每格约 2.4 个样本，可插值对齐到 10 ms 整点。代价是数字滤波带宽随
  ODR 翻倍、噪声 RMS 随之上升（Table 3 脚注 9：`Noise RMS related to BW = ODR/2`），
  而且 SFLP_ODR 也要同步提到 240 Hz（`SFLP_ODR = 0x63`）。
* 备选 60 Hz：**低于发布率，会出现重复样本**，只在"只要能看见姿态"的场景才考虑。
* 本规范**推荐 120 Hz**（与 SFLP_ODR 默认值一致，见 §4.2，且两通道同速率）。
* 由此得到的实际信号带宽（实现时用于带宽预算）：**accel 60 Hz**（ODR/2，CTRL9 默认）、
  **gyro 49.4 Hz**（LPF1 关，只剩 LPF2，Table 21 @ 120 Hz）。

---

## 4. SFLP 低功耗融合输出

### 4.1 使能路径

| 事实 | 出处 |
|---|---|
| SFLP 提供 6 轴（加速度+陀螺）**游戏旋转向量**（四元数）、重力向量、陀螺零偏 | §2.8；§1 Overview |
| 使能：`EMB_FUNC_EN_A(0x04).SFLP_GAME_EN = 1` | §2.8 "The SFLP block is enabled by setting the SFLP_GAME_EN bit to 1 of the EMB_FUNC_EN_A (04h)"；§13.2 |
| 重新初始化：`EMB_FUNC_INIT_A(0x66).SFLP_GAME_INIT = 1` | §2.8；§13.35 |
| 四元数**只存在于 FIFO**：`The X, Y, Z quaternion components are stored in FIFO`；FIFO 内容列表含 "SFLP output data (quaternion, gyroscope bias, gravity vector)" | §1 Overview；§6.12 |
| 批处理使能：`EMB_FUNC_FIFO_EN_A(0x44).SFLP_GAME_FIFO_EN = 1` | §13.17 / Table 295-296 |
| **不存在** SFLP 输出寄存器、也没有 SFLP 状态/中断位 | `EMB_FUNC_STATUS(0x12)` 无 SFLP 位（§13.13）；`EMB_FUNC_INT1(0x0A)` 无 SFLP 位（§13.7）；`SFLP_GAME_GBIAS*(6E-73h)` 只是**输入**偏置的临时寄存器（§15.1.1 "temporary register for gbias setting procedure"） |

**推论（本驱动的核心设计前提）**：SFLP 四元数**只能**从 FIFO 取，且**没有**
"融合就绪"中断/状态位 → 只能用 FIFO 水位/字数判断。任何"读 SFLP 输出寄存器"或
"等 EMB_FUNC 中断"的设计都是错的。

### 4.2 输出速率

`SFLP_ODR(0x5E)` 的 `SFLP_GAME_ODR[2:0]`（§13.30 / Table 324）：
`000`=15、`001`=30、`010`=60、**`011`=120（默认）**、`100`=240、`101`=480 Hz。

`SFLP_ODR(0x5E)` 位图（Table 323，bit7→bit0）：
`0(必须0) | 1(必须1) | SFLP_GAME_ODR_2 | _1 | _0 | 0(必须0) | 1(必须1) | 1(必须1)`

| `SFLP_GAME_ODR` | 速率 | `SFLP_ODR(0x5E)` 字节值 |
|---|---|---|
| `011` | **120 Hz（推荐，= 复位默认）** | **`0x5B`** |
| `100` | 240 Hz | `0x63` |
| `010` | 60 Hz | `0x53` |

数据手册**未**规定"加速度/陀螺 ODR 必须 ≥ SFLP ODR"这类约束
（全文 grep "SFLP" 无此句）。但 SFLP 的输入就是 accel+gyro 数据流，逻辑上
`ODR_XL = ODR_G ≥ SFLP_ODR`，本规范取 **三者同为 120 Hz**。
**【不确定/需查原图】**：若把 accel/gyro ODR 设成 60 Hz 而 SFLP_ODR=120 Hz，
行为未文档化 → 不要这么做，并列入 §9 台面验证。

### 4.3 FIFO 配置要求（必须项）

| 项 | 值 | 依据 |
|---|---|---|
| FIFO 模式 | **continuous（`FIFO_MODE=110`）**，`FIFO_CTRL4(0x0A)=0x06` | §6.12；§6.12.3；§9.8 / Table 40。**数据手册没有规定 SFLP 必须用哪种模式** → 选 continuous 的理由是"满时新样本覆盖旧样本"，任何一次主循环抖动都不会锁死输出（FIFO 模式 `001` 会停在满） |
| 水位 | `WTM = 2` → `FIFO_CTRL1(0x07)=0x02` | §9.5；理由见 §5 |
| 深度限制 | `STOP_ON_WTM = 0`（`FIFO_CTRL2(0x08)=0x00`） | §9.6 / Table 36 |
| SFLP 批处理 | `EMB_FUNC_FIFO_EN_A(0x44)=0x02` | §13.17 |
| accel/gyro 批处理 | **方案 A：关**（`FIFO_CTRL3(0x09)=0x00`，BDR 默认 0） | §9.7 / Table 38（0000 = not batched） |
| 压缩 | 关（`FIFO_CTRL2(0x08).FIFO_COMPR_RT_EN=0` 且 `EMB_FUNC_EN_B(0x05).FIFO_COMPR_EN=0`） | §6.12.8 尾段；§13.3。开启压缩会改变 FIFO 字格式，第一版不做 |

**精确寄存器写序列（按顺序）**：

| # | 地址（页） | 值 | 目的 |
|---|---|---|---|
| 1 | `0x0A`（主页） | `0x00` | FIFO → bypass（清空 FIFO，§6.12.2 "To reset FIFO content, bypass mode should be selected"） |
| 2 | `0x08`（主页） | `0x00` | `STOP_ON_WTM=0`，压缩关 |
| 3 | `0x09`（主页） | `0x00` | BDR_XL/BDR_GY 不批处理（方案 A） |
| 4 | `0x07`（主页） | `0x02` | 水位 = 2 个字 |
| 5 | `0x0A`（主页） | `0x06` | FIFO → continuous |
| 6 | `0x01`（主页） | `0x80` | 进入嵌入式功能页 |
| 7 | `0x5E`（嵌入页） | `0x5B` | `SFLP_GAME_ODR=011` → 120 Hz |
| 8 | `0x44`（嵌入页） | `0x02` | `SFLP_GAME_FIFO_EN=1` → 四元数写入 FIFO |
| 9 | `0x66`（嵌入页） | `0x02` | `SFLP_GAME_INIT=1` → 重新初始化算法 |
| 10 | `0x04`（嵌入页） | `0x02` | `SFLP_GAME_EN=1` → 启动 SFLP |
| 11 | `0x01`（主页） | `0x00` | 回到主页（**并回读校验**，见 §1.7） |

### 4.4 FIFO 字与 tag 编码

| 事实 | 值 | 出处 |
|---|---|---|
| 一个字长 | **7 字节** = 1 字节 TAG + 6 字节固定数据 | §6.12.8 "each FIFO word is composed of 7 bytes" |
| TAG 寄存器 | `FIFO_DATA_OUT_TAG (0x78)` | §9.84 / Table 216 |
| 数据寄存器 | `FIFO_DATA_OUT_X_L(0x79)`, `X_H(0x7A)`, `Y_L(0x7B)`, `Y_H(0x7C)`, `Z_L(0x7D)`, `Z_H(0x7E)` | Table 24；§9.85–9.87 |
| TAG 位布局 | bit7:3 = `TAG_SENSOR[4:0]`；bit2:1 = `TAG_CNT[1:0]`（时间槽）；bit0 = `-`（保留） | §9.84 / Table 216-217 |
| **SFLP 游戏旋转向量的 `TAG_SENSOR`** | **`0x13`**（Table 218 "SFLP game rotation vector"） | §9.84 / Table 218 |
| **原始 tag 字节值** | **`0x13 << 3 = 0x98`**（bit2:1 是 TAG_CNT，bit0 保留 → 实际可能读到 `0x98/0x9A/0x9C/0x9E`，必须屏蔽） | 由 Table 216 位布局 + Table 218 得出 |
| 判定写法（C） | `if ((tag_byte & 0xF8U) == 0x98U) { /* SFLP 四元数 */ }` | 同上 |
| 其它相关 tag（Table 218） | `0x00` FIFO empty、**`0x01` Gyroscope NC、`0x02` Accelerometer NC**、`0x12` Step counter、`0x16` SFLP gyro bias、`0x17` SFLP gravity、`0x1A` MLC result… | Table 218 |
| 字数统计 | `DIFF_FIFO[8:0]` = FIFO 中**未读字数**（每字 1B TAG + 6B） | §6.12.8；§9.25/§9.26 |

**【不确定/需查原图】**：Table 216 的列顺序在文本抽取里是
`TAG_SENSOR_4 | _3 | _2 | _1 | _0 | TAG_CNT_1 | TAG_CNT_0 | -`，即 5 位 tag 在**高 5 位**
（bit7:3）。公开的 ST 驱动示例（基于 `lsm6dsv16x-pid`）把驱动解出的 `f_data.tag`
与宏 `LSM6DSV16X_SFLP_GAME_ROTATION_VECTOR_TAG`（= `0x13`）比较，这与"原始字节 = `0x13<<3`"
自洽；但**文本抽取里没有 Figure 级的位标尺**（原图未抽取），因此台面必须打印**原始
tag 字节**确认是 `0x98`（而不是 `0x13`）——见 §9。
**若把 `0x98` 写成 `0x13`，驱动会一个四元数都取不到。**

### 4.5 3 个 half-float 分量的打包

| 字节序 | 寄存器 | 内容 |
|---|---|---|
| 1 | `0x78` | TAG（`0x98` / `0x9A` / `0x9C` / `0x9E`；bit2:1 是 TAG_CNT） |
| 2 | `0x79` `FIFO_DATA_OUT_X_L` | **q_x LSB** |
| 3 | `0x7A` `FIFO_DATA_OUT_X_H` | **q_x MSB** |
| 4 | `0x7B` `FIFO_DATA_OUT_Y_L` | **q_y LSB** |
| 5 | `0x7C` `FIFO_DATA_OUT_Y_H` | **q_y MSB** |
| 6 | `0x7D` `FIFO_DATA_OUT_Z_L` | **q_z LSB** |
| 7 | `0x7E` `FIFO_DATA_OUT_Z_H` | **q_z MSB** |

* **字节序**：每个分量是 16 bit **小端**（`_L` 在前、`_H` 在后），与输出寄存器族
  （`OUTX_L_G(0x22)`=D7:0、`OUTX_H_G(0x23)`=D15:8，§9.30 / Table 87-88）同一风格；
  FIFO 数据寄存器同样按 `X_L/H → Y_L/H → Z_L/H` 成对排列（§9.85–9.87 / Table 219-224）。
* **16 bit 对齐 / padding**：**没有 padding**。6 字节 = 3 × 16 bit，天然按 16 bit 对齐，
  **不需要**像 "TAG+1 pad" 之类的处理。`DIFF_FIFO` 的步进固定 7 字节。
* **只有 x/y/z，没有 w**：`The X, Y, Z quaternion components are stored in FIFO`（§1 Overview）。
  w 必须自行重建：**`w = √(1 − x² − y² − z²)`**——这正是 `src/imu.h` 与
  `imu.rs` 的既有契约（且 ST 官方 SFLP 示例也是这么做的）。
* **数值格式**：IEEE 754 **binary16**（`S` 1 位 / `E` 5 位 / `F` 10 位），
  即 `SEEEEEFFFFFFFFFF`。
  数据手册**明确写出该格式的地方是 SFLP 陀螺零偏寄存器**：
  `SFLP_GAME_GBIASX_L/H` "The value is expressed as half-precision floating-point format:
  SEEEEEFFFFFFFFFF"（§15.1.1，Table 351）。
  **【不确定/需查原图】**：**FIFO 数据寄存器本身的描述（Table 219-224）只写
  "FIFO X-axis output"，没有写格式**。因此"FIFO 里的四元数是 binary16"这条依赖：
  (a) §1 Overview "quaternion components stored in FIFO"，
  (b) §15.1.1 对 SFLP 输出量的 binary16 定义，
  (c) 仓库既有约定 `imu_math.h`（`f32_to_half` 注释："The LSM6DSV16X ships the SFLP
  quaternion in this format"）。
  **台面必须实测核对**（§9）：静止时 q_x/y/z 应约为 0x0000（≈0），
  倾斜 90° 时某分量应接近 ±1.0 的 half 编码（`0x3C00` / `0xBC00`）。
* **解码**：把 16 位拼成 `uint16_t h = (uint16_t)L | ((uint16_t)H << 8)`，再走
  `half_to_f32()`（`src/imu_math.c` 已有）。
* **chirality/定序**：解出的 (x,y,z) **原样**填入块字节 6..11，**板上不做任何旋转**
  （chip frame 直出，mount 由主机 `imu.rs` 施加）。`imu_sim.c` 的
  `q_raw = quat_mul(s_q, s_mount)`（chip→world）就是这个语义的参照。

### 4.6 FIFO 溢出 / 水位 / 满 标志的读取与清除

| 标志 | 位 | 读/清规则 | 出处 |
|---|---|---|---|
| `DIFF_FIFO[8:0]` | `FIFO_STATUS1(0x1B)` bit7:0 + `FIFO_STATUS2(0x1C)` bit0 | 只读，反映当前未读字数 | §9.25 / Table 77；§9.26 / Table 79 |
| `FIFO_WTM_IA` | `FIFO_STATUS2(0x1C)` bit7 | "FIFO filling is equal to or greater than WTM" | §9.26 / Table 79；§6.12.3 |
| `FIFO_OVR_IA` | `FIFO_STATUS2(0x1C)` bit6 | "1: FIFO is completely filled"（至少最旧样本被覆盖） | §9.26 / Table 79；§6.12.3 |
| `FIFO_FULL_IA` | `FIFO_STATUS2(0x1C)` bit5 | "1: FIFO will be full at the next ODR" | §9.26 / Table 79 |
| `FIFO_OVR_LATCHED` | `FIFO_STATUS2(0x1C)` bit3 | **"This bit is reset when this register is read."** | §9.26 / Table 79 |
| `COUNTER_BDR_IA` | `FIFO_STATUS2(0x1C)` bit4 | **"This bit is reset when these registers are read."** | §9.26 / Table 79 |
| FIFO 清空（软复位 FIFO） | `FIFO_CTRL4(0x0A)` | 写 bypass `0x00` 即清空，再写 continuous `0x06` 恢复 | §6.12.2 "To reset FIFO content, bypass mode should be selected"；§6.12.3 |

**【不确定/需查原图】**：数据手册**只说** `FIFO_OVR_LATCHED` 与 `COUNTER_BDR_IA`
"读该寄存器即清"，**没有说** `FIFO_WTM_IA` / `FIFO_OVR_IA` / `FIFO_FULL_IA`
如何清除。经验上水位位在 FIFO 排空到低于 WTM 后自动复位、溢出位需要读完 FIFO 数据
（或 bypass 一次）才复位，但**不能当依据**。驱动必须做的保守做法：
只把这三个位当作**只读诊断**，恢复动作一律"bypass → 重新 continuous"，
并在 §9 中验证它们的行为。

### 4.7 SFLP 初始化 / 收敛时间与收敛前输出

| 参数 | 值 | 出处 |
|---|---|---|
| 静态精度 | yaw 0.5 °/5 min；pitch/roll 1.5 ° | Table 1 |
| 低动态精度 | yaw 0.7 °/5 min；pitch/roll 0.5 ° | Table 1 |
| 高动态精度 | yaw 5.9 °/5 min；pitch/roll 1.6 ° | Table 1 |
| **校准时间（Calibration time）** | **0.8 s**（"Time required to reach steady state"） | Table 1 脚注 1 |
| **姿态稳定时间（Orientation stabilization time）** | **0.7 s** | Table 1 |

**收敛前芯片到底输出什么？——数据手册没有说。**
**【不确定/需查原图】** 两个可能：FIFO 里根本不写 SFLP 字（`DIFF_FIFO` 不增长），
或者写 `0x0000`（half 的 +0.0，即 x=y=z=0）。**两种都能被我们的判据吃下**，
所以驱动的做法不需要先验知识：

1. `SFLP_GAME_EN=1` 之后，从驱动内部记 `t_sflp_enable_ms`；
2. 在 `t < 800 ms + 700 ms = 1500 ms` 窗口内，**丢弃**所有 SFLP 字（无论内容），
   块字节 6..11 发**全 0**，`SFLP_VALID`/`FUSION_OK` 清 0；
3. 窗口结束后的第一个 SFLP 字解码出 `(x,y,z)`；仅当
   `|q|² = x²+y²+z²` 有意义（`0 < |q| ≤ 1`，即 `w = √(1−|q|²)` 为实数）
   才认为有效 → 填块、置 `SFLP_VALID`；
4. 若解码结果是 NaN/Inf（half 的 `0x7C00`–`0x7FFF` 等）或 `|q| > 1`，视为未收敛，
   继续发全 0。
   （全 0 的 half 是 `0x0000`，正好与主机"全 0 = 还没写表 / 保持上一帧"的语义同构，
   这是 `imu_sim.c` 的 `SIM_SFLP_WAIT` 模式已经钉住的行为。）

---

## 5. FIFO 与中断

### 5.1 推荐配置（100 Hz 轮询主循环）

| 项 | 推荐值 | 理由 |
|---|---|---|
| FIFO 模式 | **continuous**，`FIFO_CTRL4(0x0A)=0x06` | 满时覆盖最旧，主循环抖动不会丢新数据、不会锁死 |
| 水位 | **`WTM = 2`**，`FIFO_CTRL1(0x07)=0x02` | 120 Hz 下一个字 8.33 ms，100 Hz 轮询每格 1~2 个字；`WTM=2` 让水位位在正常节拍下稳定置位又不会提前太多。`WTM=1` 每次单字就置位（中断最频繁、最灵敏）；`WTM≥4`（≥33 ms）会引入半格以上延迟，只在纯中断驱动时才值得 |
| 深度限制 | `STOP_ON_WTM=0` | 保留全部 FIFO 作为抖动缓冲 |
| FIFO 深度 | **1.5 KB 未压缩** → ≈ **219 个 7 字节字**（1536/7） | §6.12 "embeds 1.5 KB of data in FIFO (up to 4.5 KB with the compression feature enabled)"。**数据手册未给出确切可用字数** → §9 核对 |
| 抖动裕量 | 1.2 字/10 ms → 219 字 ≈ **1.8 s** 不用排空才会溢出 | 由上一行推算；也解释了为什么 `imu_tick()` 的 100 ms 积压丢弃上限完全安全 |
| 批处理 | 方案 A：只批 SFLP（`FIFO_CTRL3=0x00`） | FIFO 里只有 `0x98` 一种 tag，解析最简单 |

### 5.2 INT1 / INT2 路由

`INT1_CTRL(0x0D)` 位图（Table 45，bit7→bit0）：
`0(必须0) | INT1_CNT_BDR | INT1_FIFO_FULL | INT1_FIFO_OVR | INT1_FIFO_TH | 0(必须0) | INT1_DRDY_G | INT1_DRDY_XL`

| 需求 | `INT1_CTRL(0x0D)` 值 |
|---|---|
| **仅 FIFO 水位**（推荐） | **`0x08`**（bit3 `INT1_FIFO_TH`） |
| 水位 + 溢出 | `0x18`（bit3 + bit4 `INT1_FIFO_OVR`） |
| 水位 + 溢出 + 两路 DRDY | `0x1B`（+ bit1 `INT1_DRDY_G` + bit0 `INT1_DRDY_XL`） |
| 全关（纯轮询） | `0x00` |

`INT2_CTRL(0x0E)`：同样有 `INT2_FIFO_TH`(bit3)/`INT2_FIFO_OVR`(bit4)/`INT2_DRDY_G`(bit1)/
`INT2_DRDY_XL`(bit0)；本驱动 **`0x00`**，PB1 空闲备用。
`MD1_CFG(0x5E)` **不需要写**：它只路由 tap/WU/FF/6D/SHUB/`INT1_EMB_FUNC`，
而 `EMB_FUNC_STATUS(0x12)` 里**没有 SFLP 位**（§13.13），所以
"用 EMB_FUNC 中断表示四元数就绪"这条路**不存在**。

`IF_CFG(0x03)` 决定 INT 引脚电气特性：`H_LACTIVE`(bit4) 0=高有效、1=低有效；
`PP_OD`(bit3) 0=推挽、1=开漏（§9.3 / Table 30）。默认 `0x00` = 推挽、高有效，
PB0/PB1 在 `main.c` 里已是上拉输入，可直接接。

### 5.3 只轮询 `FIFO_STATUS` 够不够？——**够，且推荐**

* 裕量：见 §5.1 表（219 字 ÷ 1.2 字/10 ms ≈ 1.8 s），10 ms 周期内**不可能**溢出；
* 一次轮询只需 2 字节读：`0x1B` 起连读 `FIFO_STATUS1`+`FIFO_STATUS2`（`IF_INC=1`
  自动递增），比一次 EXTI 中断 + 中断里读寄存器更便宜、更可预测；
* 主循环本来就每 10 ms 跑一次（`main.c` 的 `imu_tick(now)`），
  与总线时序（`docs/bus_timing_borrow_plan.md` 的 300 µs 回复窗口）相比，
  多这一笔 SPI 完全无压力（2 字节 @7.5 MHz ≈ 2.4 µs）；
* 结论：**INT1 只作为可选的"低延迟唤醒/掉帧告警"**。若实现：
  `PB0` 配 EXTI0 下降沿/上升沿（按 `H_LACTIVE`），ISR 里只 `s_fifo_irq = 1;`，
  **不在 ISR 里做 SPI**（既有约定：ISR 只置标志、主循环消费），
  主循环消费该标志；EXTI 优先级必须低于 USART1 的总线中断。

**方案 B（可选升级，仅当 ±8.3 ms 时间戳抖动不可接受）**：
`FIFO_CTRL3(0x09)=0x66`（`BDR_GY=0110`、`BDR_XL=0110`，都 120 Hz），
`WTM` 提到 `3`（一个批处理槽 = GYRO+XL+SFLP 三个字）；此时 FIFO 里会出现
`TAG_SENSOR` = `0x01`（Gyroscope NC）/`0x02`（Accelerometer NC）/`0x13` 三种字，
用 `TAG_CNT` 配对，`DIFF_FIFO` 增长 3.6 字/10 ms（219 字 ≈ **0.6 s** 裕量，
仍远大于 `imu_tick()` 的 100 ms 积压上限）。
代价：解析复杂度上升（§6 的分支要按 tag 分派）。

---

## 6. 数据格式与字节序

### 6.1 原始输出寄存器块

| 通道 | 起始地址 | 长度 | 顺序 | 格式 | 出处 |
|---|---|---|---|---|---|
| gyro | **0x22** (`OUTX_L_G`) | 6 字节 | `X_L,X_H,Y_L,Y_H,Z_L,Z_H` | 每轴 16 bit **二补码小端** | §9.30–9.32 / Table 87-95；Table 24 |
| accel | **0x28** (`OUTX_L_A`) | 6 字节 | `X_L,X_H,Y_L,Y_H,Z_L,Z_H` | 每轴 16 bit **二补码小端** | §9.33–9.35 / Table 96-104；Table 24 |
| 温度（可选） | 0x20 (`OUT_TEMP_L`) | 2 字节 | `L,H` | 16 bit 二补码，`256 LSB/°C`，25 °C 时 **0 LSB (typ.)** | §9.29 / Table 84-86；§4.3 / Table 6 |

字面依据（X 轴为例）：
`OUTX_L_G (22h)` = D7…D0；`OUTX_H_G (23h)` = D15…D8（§9.30 / Table 87, 88）→
**X_L 在前、小端**。Y/Z 同构（0x24/0x25、0x26/0x27；accel 0x28…0x2D）。

### 6.2 换算：不需要额外缩放

`gyro int16 = (int16_t)((uint16_t)b[0] | ((uint16_t)b[1] << 8))`，
`accel` 同理 —— **直接就是块里要的 int16 计数**，无需移位、无需乘除、无需换轴
（chip frame 直出，与 `imu.h` 的 "chip frame, raw counts" 一致）。

| 量 | 每 LSB | 换算（仅用于日志/健康判断） | 出处 |
|---|---|---|---|
| gyro | 17.5 mdps | `dps = count × 0.0175`；`rad/s = count × 0.0175 × π/180` | Table 3 |
| accel | 0.122 mg | `g = count × 0.000122`；`m/s² = count × 0.000122 × 9.80665` | Table 3 |
| 温度 | 256 LSB/°C | `°C = raw/256 + 25` | §4.3 / Table 6 |

12 字节 gyro + 12 字节 accel 可各用一次突发读完成：
`lsm_rd(0x22, buf, 6)`、`lsm_rd(0x28, buf, 6)`；
若想一次读完（含温度）用 `lsm_rd(0x20, buf, 14)`（0x20…0x2D 连续，
`IF_INC=1` 自动递增，Table 24 中该区间无空洞）。

---

## 7. 推荐的初始化序列（可直接翻成 C）

前置：`board_gpio_init()` 之后，驱动先把 PA5/6/7 改为 `GPIO_MODE_AF_PP` +
`GPIO_OSPEED_50MHZ`，PA4 保持推挽输出高；开 `RCU_GPIOA` / `RCU_AF` / `RCU_SPI0`；
`spi_struct_para_init()` → `device_mode=SPI_MASTER`、`trans_mode=SPI_TRANSMODE_FULLDUPLEX`、
`frame_size=SPI_FRAMESIZE_8BIT`、`nss=SPI_NSS_SOFT`、`endian=SPI_ENDIAN_MSB`、
**`clock_polarity_phase=SPI_CK_PL_HIGH_PH_2EDGE`**、`prescale=SPI_PSC_16`（7.5 MHz）；
`spi_init()` → `spi_enable()` → `spi_nss_internal_high(SPI0)`。

下表每一行的"值"都是 §1–§4 推出的**最终字节值**。`(主)` = `FUNC_CFG_ACCESS=0x00` 页，
`(嵌)` = `FUNC_CFG_ACCESS=0x80` 页。

| # | 地址 | 值 | 页 | 目的 / 说明 |
|---|---|---|---|---|
| 1 | — | — | — | `delay_ms(10)`：上电后首次访问前的等待（数据手册未规定，见 §1.3） |
| 2 | `0x12` | `0x01` | 主 | `CTRL3.SW_RESET=1` 软复位 |
| 3 | `0x12` | 读 | 主 | 轮询直到 **bit0==0**（自清），每 1 ms 一次，超时 **50 ms** → 失败：`SENSOR_ERR` |
| 4 | `0x0F` | 读 | 主 | **`WHO_AM_I` 必须 == `0x70`**，否则 `SENSOR_ERR` 并停止初始化 |
| 5 | `0x12` | `0x44` | 主 | `BDU=1` + `IF_INC=1`（= 复位默认值，显式写便于第 6 步回读校验） |
| 6 | `0x12` | 读 | 主 | 回读校验 == `0x44`（`BDU`/`IF_INC` 生效） |
| 7 | `0x03` | `0x01` | 主 | `IF_CFG`：`SIM=0`（4 线）、`I2C_I3C_disable=1`（关 I²C/I3C 防误触发）。**必须在 SW_RESET 之后写**（本寄存器不被复位，§1.3） |
| 8 | `0x10` | `0x06` | 主 | `CTRL1`：`OP_MODE_XL=000` 高性能 + `ODR_XL=0110` = **120 Hz** |
| 9 | `0x17` | `0x01` | 主 | `CTRL8`：`FS_XL=01` = **±4 g** |
| 10 | `0x11` | `0x06` | 主 | `CTRL2`：`OP_MODE_G=000` 高性能 + `ODR_G=0110` = **120 Hz** |
| 11 | `0x15` | `0x02` | 主 | `CTRL6`：`FS_G=0010` = **±500 dps**，`LPF1_G_BW=000` |
| 12 | `0x16` | `0x00` | 主 | `CTRL7`：`LPF1_G_EN=0`（陀螺 LPF1 关闭 → 输出带宽只剩 LPF2 = 49.4 Hz @120 Hz，§3.3） |
| 13 | `0x0A` | `0x00` | 主 | `FIFO_CTRL4` = bypass → 清空 FIFO（§6.12.2） |
| 14 | `0x08` | `0x00` | 主 | `FIFO_CTRL2`：`STOP_ON_WTM=0`、压缩关、`UNCOMPR_RATE=00` |
| 15 | `0x09` | `0x00` | 主 | `FIFO_CTRL3`：XL/GY 不批处理（方案 A；方案 B 用 `0x66`） |
| 16 | `0x07` | `0x02` | 主 | `FIFO_CTRL1`：**水位 WTM=2 字** |
| 17 | `0x0A` | `0x06` | 主 | `FIFO_CTRL4`：`FIFO_MODE=110` = **continuous** |
| 18 | `0x0D` | `0x08` | 主 | `INT1_CTRL`：`INT1_FIFO_TH=1`（水位→INT1/PB0）。想加溢出告警写 `0x18`；纯轮询写 `0x00` |
| 19 | `0x0E` | `0x00` | 主 | `INT2_CTRL`：全关（PB1 备用） |
| 20 | `0x01` | `0x80` | 主 | `FUNC_CFG_ACCESS.EMB_FUNC_REG_ACCESS=1` → **进入嵌入式功能页** |
| 21 | `0x01` | 读 | — | 回读确认 == `0x80`（页切换校验，见 §1.7） |
| 22 | `0x5E` | `0x5B` | **嵌** | `SFLP_ODR`：`SFLP_GAME_ODR=011` = **120 Hz**（+ bit6/bit1/bit0 强制位） |
| 23 | `0x44` | `0x02` | **嵌** | `EMB_FUNC_FIFO_EN_A`：`SFLP_GAME_FIFO_EN=1` → 四元数写入 FIFO |
| 24 | `0x66` | `0x02` | **嵌** | `EMB_FUNC_INIT_A`：`SFLP_GAME_INIT=1` → 重新初始化 SFLP 算法 |
| 25 | `0x04` | `0x02` | **嵌** | `EMB_FUNC_EN_A`：`SFLP_GAME_EN=1` → **启动 SFLP** |
| 26 | `0x66` | 读 | **嵌** | （可选）确认 `SFLP_GAME_INIT` 已自清；未自清则重试 |
| 27 | `0x01` | `0x00` | 主 | 回到主页；**回读确认 == `0x00`** |
| 28 | — | — | — | 记 `t_sflp_enable_ms = now_ms`；进入 §8 状态机；**1500 ms 内发全 0 四元数**（§4.7） |

**必须先做、不能省的读回校验**：`WHO_AM_I`（步 4）、`CTRL3`（步 6）、
`FUNC_CFG_ACCESS` 两次（步 21、步 27）。其余寄存器可选回读（成本 ≈ 2.4 µs/次）。
任何一步写后回读不一致 → 重试 3 次 → 仍不一致则置 `SENSOR_ERR` 并转 FAULT 态。

---

## 8. 驱动状态机与错误处理

### 8.1 状态机

```
POWER_ON ──(≥10ms)──> PROBE ──WHO_AM_I==0x70──> RESET ──SW_RESET自清──> CONFIG
   ▲                     │ 失败                                                │
   │                     ▼                                                    ▼
   └──── RECOVER <── FAULT <──(连续3次错)── RUN <──(1500ms窗口过 + 第一个有效字)── CONVERGE
```

| 状态 | 进入动作 | 退出条件 |
|---|---|---|
| `PROBE` | 读 `WHO_AM_I` | == `0x70` → `RESET`；否则重试 3 次 → `FAULT` |
| `RESET` | 写 `CTRL3=0x01`，轮询 bit0==0（超时 50 ms） | 自清 → `CONFIG`；超时 → `FAULT` |
| `CONFIG` | §7 的步 5…27 全部写 + 读回校验 | 全部通过 → `CONVERGE`；任一校验失败 → `FAULT` |
| `CONVERGE` | 记 `t0`；每 tick 排空并**丢弃** FIFO 里的 SFLP 字；块 6..11 = 全 0；`SFLP_VALID`/`FUSION_OK` 清 0 | `now-t0 ≥ 1500 ms` **且** 收到一个可解码的 SFLP 字 → `RUN`；`now-t0 > 5000 ms` 仍无字 → `FAULT` |
| `RUN` | 正常排空 + 发布（见 §8.2） | 连续 3 次 SPI 读失败 / 读回不一致 / `FIFO_OVR_IA` 且排空后仍异常 → `FAULT`；`imu_restart()` → `CONFIG`（或只回 `CONVERGE`，见下） |
| `RECOVER` | `FIFO_CTRL4(0x0A)=0x00`（bypass 清 FIFO）→ 重写 §7 步 14…15（`FIFO_CTRL2/3`）与步 16（水位 `WTM=2`）→ `FIFO_CTRL4=0x06`；**不**重写量程/ODR（保留 SFLP 收敛状态） | 一次成功 → `RUN`（`t_sflp_enable_ms` 保留） |
| `FAULT` | SPI 每 100 ms 重试 `WHO_AM_I`；块 6..11 = 全 0；`SENSOR_ERR=1`、`SFLP_VALID=0`、`FUSION_OK=0` | `WHO_AM_I` 恢复 → 完整重跑 `CONFIG` |

`imu_restart()` 的**实际**调用点（已核对 `src/dev.c`）：写 vendor 寄存器 `V_SIM_MODE`
（`dev.c:228`）、写 `V_REPORT_FRAME`（`dev.c:244`）、vendor 命令 `VCMD_RESET_SIM`
（`dev.c:284`）。语义：清融合状态 → 重写 `EMB_FUNC_INIT_A(0x66)=0x02` 重新初始化
SFLP → 回到 `CONVERGE`（**不**重跑 `SW_RESET`，不动 `WHO_AM_I`）。

> **实现状态（2026-09-25，真 PCBA 实测）：`CONVERGE` 已落地。** 代码在
> `src/imu_spi.c`（`imu_spi_begin_converge()` / `imu_spi_end_converge()` / `s_converging`），
> 窗口常量 `LSM_SFLP_CONVERGE_MS = 1500`（`src/lsm6dsv16x.h`，0.8 s 校准 + 0.7 s 稳定）。
> 与上面状态表/§4.7 的两点实测偏差，以代码为准：
> * §4.7/§9.1#10 问"收敛前芯片输出什么"——**实测芯片继续输出格式良好的 SFLP 字**
>   （`sflp_words` 约 +115.5/s、重启期间无停顿），既不静默也不写全 0。所以"等第一个字"
>   不等于"等可信姿态"；窗口内驱动**丢弃**所有 SFLP 字、发布零四元数字节并清
>   `TELEM_FLAG_SFLP_VALID`，窗口结束再 flush FIFO（bypass → continuous）。
> * 真机没有独立的 `FAULT`/`RECOVER` 状态：`WHO_AM_I` 不匹配只是 `SENSOR_ERR` + 每 1 s 重探
>   （`IMU_SPI_REPROBE_MS`），不进入单独状态机。
>
> 实测时间：`VCMD_RESET_SIM` 后 **9–13 ms** 四元数字节归零；**1.51–1.52 s** 后恢复非零
> （冒烟上限 2.5 s）。真机联调背景见 `docs/bus_timing_borrow_plan.md` §4.6.3。

### 8.2 100 Hz tick 的排空流程

```c
void imu_impl_tick(uint32_t now_ms)          /* 与 main.c 的 10 ms 网格对齐 */
{
    uint8_t st[2], tag[7];
    uint16_t n;

    if (state != RUN && state != CONVERGE) { return; }

    /* 1) FIFO 状态：0x1B 起连读 2 字节（IF_INC=1 自动递增） */
    if (lsm_rd(0x1B, st, 2) != 0) { err_cnt++; ... return; }
    n = (uint16_t)st[0] | ((uint16_t)(st[1] & 0x01U) << 8);   /* DIFF_FIFO[8:0] */

    if (st[1] & 0x40U) { overrun_cnt++; }        /* FIFO_OVR_IA：只记账，不当场恢复 */
    if (n > IMU_FIFO_MAX_DRAIN) { n = IMU_FIFO_MAX_DRAIN; }   /* 上限保护，防跑飞 */

    /* 2) 逐字排空：每字按 7 字节一次读完（0x78..0x7E）——
           "读 TAG 是否推进读指针" 需按 §9 #7 实测确认，未确认前不要只读 1 字节 */
    for (i = 0; i < n; i++) {
        if (lsm_rd(0x78, tag, 7) != 0) { err_cnt++; return; }
        if ((tag[0] & 0xF8U) == 0x98U) {          /* SFLP game rotation vector */
            qx = half_to_f32((uint16_t)tag[1] | ((uint16_t)tag[2] << 8));
            qy = half_to_f32((uint16_t)tag[3] | ((uint16_t)tag[4] << 8));
            qz = half_to_f32((uint16_t)tag[5] | ((uint16_t)tag[6] << 8));
            have_sflp = 1;
        }
        /* 方案 A：其它 tag（本应不出现）直接丢弃；
           方案 B：分派 raw tag 0x08（TAG_SENSOR 0x01 = gyro NC）/ 0x10（0x02 = accel NC） */
    }

    /* 3) 100 Hz 网格上取一次 gyro+accel（可合成 0x20 起 14 字节一次读完） */
    lsm_rd(0x22, rawg, 6);   /* → int16 gyro[3]  = X_L,X_H,Y_L,Y_H,Z_L,Z_H */
    lsm_rd(0x28, rawa, 6);   /* → int16 accel[3] 同上 */

    /* 4) 发布：quat 有效才写 6..11，否则全 0；counter 每发布一次 +1 */
}
```

* `IMU_FIFO_MAX_DRAIN` 建议 `16`：正常只需 1~2，遇到长停顿最多补 12 步
  （`imu.c` 的 100 ms 积压上限）→ 16 足够，超过说明状态异常。
* **丢帧计数**：`n == 0` 连续超过 20 个 tick（200 ms）→ 说明 SFLP 停了 → `FAULT`。
* **不重排、不插值**、不丢最新：取本次排空到的**最后一个** `0x98` 字为准
  （最新姿态），`counter` 每个 tick 只 +1（保持 100 Hz 契约）。
* `imu_samples()` 的语义保持"发布一次 +1"，与仿真器一致（`main.c` 用它决定
  `dev_refresh()` 的时机）。

### 8.3 错误处理要点

| 场景 | 检测 | 动作 | 标志 |
|---|---|---|---|
| SPI 完全无响应（MISO 常量 0x00/0xFF） | `WHO_AM_I != 0x70` | 重试 3 次 → `FAULT`；四元数全 0 | `SENSOR_ERR` |
| `SW_RESET` 不自清 | 轮询 50 ms 超时 | `FAULT` | `SENSOR_ERR` |
| 页切换失败 | 回读 `FUNC_CFG_ACCESS != 0x80/0x00` | 重试 3 次（重发页切换）→ `FAULT` | `SENSOR_ERR` |
| FIFO 溢出 | `FIFO_STATUS2.FIFO_OVR_IA` 或 `FIFO_OVR_LATCHED` | 计数；若单 tick 内 `DIFF_FIFO==219`（满）→ `RECOVER`：bypass → continuous | `SENSOR_ERR`（连续 3 次） |
| SFLP 停止出字 | 200 ms 内没有任何 `0x98` 字 | `FAULT` → 重跑 `CONFIG` | `SENSOR_ERR` |
| 收敛窗口内 | `now - t_sflp_enable_ms < 1500 ms` | 发全 0，丢弃 SFLP 字 | `SFLP_VALID=0`、`FUSION_OK=0` |
| 陀螺接近标称满量程 | 任一轴 `abs(count) ≥ 28571`（= ±500 dps） | 只置标志，不丢样本 | `GYRO_SAT` |
| `counter` 回绕 | `counter == 0xFF` 后清零 | 与 `imu_sim.c` 一致：置位一个 tick | `CNT_WRAP` |
| 配置脏 | `dev.c` 的 `s_cfg_dirty` | 由 dev 层自己置位 | `CFG_DIRTY` |

### 8.4 `dev_cfg.gyro_bias` 怎么接（**必须先做决定的地方**）

现状（已核对源码）：`dev_cfg_t.gyro_bias[3]` 是 **i16 原始计数**，`dev.h:37` 的注释是
**`raw counts added to the gyro output`**（加在 gyro 输出上的注入量）；写 vendor 窗口
`V_BIAS_X_L…`（`board.h`，DXL 148+10 = 158）时**只** `s_cfg_seq++`（`dev.c:252-256`），
**不会**调用 `imu_restart()`；`imu_sim.c` 也确实是 `+ cfg->gyro_bias[i]`。
换句话说：**这个字段是"注入量"，不是"标定减除量"**。

V16X 路径上有两条互斥的接法，必须显式选一条并写进代码注释：

| 方案 | 做法 | 优点 | 代价 / 风险 |
|---|---|---|---|
| **A（推荐 v1）** | 保持 `dev.h` 的既定语义：`gyro_out = raw + gyro_bias[i]`（**加**）；芯片 SFLP 不感知（quat 不受影响）；`gyro_bias` **每个 tick 重新读**（`dev_cfg()` 的 3 个 i16，零成本） | 不需要进高级功能页；`dev.c`/`dev.h` 一行不用改；与 `imu_sim.c` 语义完全一致，`bus_smoke.py` 的对比基线可比 | **零偏只作用在 gyro 字节上，不影响融合质量**；做标定时要把"实测零偏的相反数"写进该字段（因为它是加项） |
| B（增强） | 通过高级功能页把零偏写进芯片：`SFLP_GAME_GBIASX/Y/Z_L/H`（0x6E–0x73，page 0，binary16），让 **SFLP 自己扣零偏** | 融合质量更好 | 需要 §14 尾部的页写过程；**单位未在数据手册文本中给出**（只写 "SFLP game algorithm X-axis gbias" + binary16 格式，§15.1.1）→ **【不确定/需查原图】** 必须先台面标定出"count → gbias 数值"的换算 |

**方案 B 的页写过程（摘自 §14 尾部"Write procedure example"）**：
1. `FUNC_CFG_ACCESS(0x01) = 0x80`（`EMB_FUNC_REG_ACCESS=1`）
2. `PAGE_RW(0x17) = 0x40`（`PAGE_WRITE=1`）
3. `PAGE_SEL(0x02) = 0x01`（`PAGE_SEL[3:0]=0000` → page 0，且 bit0 强制 1；**默认值就是 0x01**）
4. `PAGE_ADDRESS(0x08) = 0x6E`（目标的低字节地址）
5. `PAGE_VALUE(0x09) = <byte>`（写数据；连续寄存器只需重复第 5 步）
6. `PAGE_RW(0x17) = 0x00`（`PAGE_WRITE=0`）
7. `FUNC_CFG_ACCESS(0x01) = 0x00`

**无论选哪条，都不要依赖 `gyro_bias` 变更会触发 `imu_restart()`——它不会**（`dev.c`
只在 sim_mode / report_frame / `VCMD_RESET_SIM` 三处调用）。这也是本规范给
`imu.c`/新驱动的实现提示：若走方案 B，需要自己在 tick 里比较 `dev_cfg_seq()`
或轮询 `gyro_bias[]` 来决定是否重写 gbias 寄存器。

### 8.5 与 `TELEM_FLAG_*` 的映射（块字节 19，`board.h`）

| 标志 | 值 | 本驱动语义 |
|---|---|---|
| `TELEM_FLAG_SFLP_VALID` | `0x01` | `RUN` 态且当前块的四元数来自**有效** SFLP 字（非全 0、`0 < \|q\| ≤ 1`）时置位；`CONVERGE`/`FAULT` 时清 0 |
| `TELEM_FLAG_CNT_WRAP` | `0x02` | `counter` 由 `0xFF` 回绕到 `0x00` 的那一个 tick 置位（与 `imu_sim.c` 相同） |
| `TELEM_FLAG_GYRO_SAT` | `0x04` | 任一轴 `abs(gyro count) ≥ 28571` 时置位（§3.1 的量程标称坑） |
| `TELEM_FLAG_SENSOR_ERR` | `0x08` | `PROBE`/`RESET`/`CONFIG` 失败、连续 3 次 SPI 错、FIFO 溢出恢复、SFLP 停止出字 |
| `TELEM_FLAG_SIMULATED` | `0x10` | **必须清 0**（本驱动 `imu_is_simulated()` 返回 0；`dev.c` 会按它清 `VSTAT_SIM_ACTIVE`） |
| `TELEM_FLAG_FROZEN` | `0x20` | **不用**（仅 `imu_sim.c` 的 `SIM_FROZEN` 模式） |
| `TELEM_FLAG_CFG_DIRTY` | `0x40` | 由 `dev.c` 维护，驱动不碰 |
| `TELEM_FLAG_FUSION_OK` | `0x80` | 与 `SFLP_VALID` 同置同清；`CONVERGE`/`FAULT` 必清 0 |

**硬约束（来自 `imu.h` / `imu.rs`，不是可选项）**：收敛前与故障时，
块字节 6..11 **必须是全 0**。线上只有 x/y/z（w 由主机重建），所以唯一合法的
"无效"表示就是 `x=y=z=0`；只要发出任何一个能解出非零 x/y/z 的值，主机就会把它
当成一帧**有效姿态**（`imu.rs` 的 `all_zero_quaternion_bytes_hold_the_last_good_value`
钉住了"全 0 = 保持上一帧好值"）。

---

## 9. 待验证清单

### 9.1 只能在真实硬件上确认的项（每条都写成可执行的测量）

| # | 待验证事实 | 具体测量方法 | 通过判据 |
|---|---|---|---|
| 1 | **SPI 模式**：数据手册文字指向 Mode 3，但 Figure 10–12 的位标尺在文本抽取中丢失 | 逻辑分析仪同时抓 SCK/MOSI/MISO，先试 `SPI_CK_PL_HIGH_PH_2EDGE`（Mode 3），失败再试 `SPI_CK_PL_LOW_PH_1EDGE`（Mode 0） | 连续 1000 次读 `WHO_AM_I` 全为 `0x70`，无一位错 |
| 2 | **没有 dummy byte** 是否成立 | 用 `lsm_rd(0x0F,buf,1)` 直接读；若返回错值，再插入 1 个 dummy 字节对比 | 直读得到 `0x70`；加 dummy 反而得 `0x00`/上一字节 |
| 3 | **SPI 时钟上限** | 依次 `SPI_PSC_32`(3.75M) → `PSC_16`(7.5M)，每次连续读 `WHO_AM_I` 1000 次并统计误码；可选试 `PSC_8`(15M，超规格) | `PSC_16` 下 0 误码 → 用 7.5 MHz；有误码则退 3.75 MHz |
| 4 | **上电到首次可访问的等待时间**（数据手册未给 POR/boot 时间，规范暂取 10 ms） | 上电后从 0 ms 起每 1 ms 读一次 `WHO_AM_I`，记录第一次 == `0x70` 的时刻 | 得到实测值并写回 §1.3 |
| 5 | **`SW_RESET` 自清时间** | 写 `CTRL3=0x01` 后用 DWT/µs 计时轮询 bit0 | 记录实测（规范暂定 50 ms 超时；若 >50 ms 要改） |
| 6 | **FIFO tag 字节真值**（最高风险项：`0x98` 还是 `0x13`） | 静止状态排空 FIFO，把每个字的 tag 原始字节 `0x78` 打印成 hex（不做任何移位） | 必须看到 `0x98/0x9A/0x9C/0x9E`（高 5 位 = `0x13`）；若看到 `0x13` 则 §4.4 的位布局理解错了，必须改判定 |
| 7 | **FIFO 字长固定 7 字节 / 读 TAG 是否推进读指针** | 读一次 `0x78..0x7E`（7B）后看 `DIFF_FIFO` 是否恰好 -1；再试"只读 0x78 一字节"看是否也 -1 | 确认"必须整字 7 字节读一次"，否则读指针不前进（会重复读同一个字） |
| 8 | **SFLP 四元数是 IEEE binary16**（FIFO 寄存器描述未写格式） | 静止水平：`q_x,q_y,q_z` 的 16 位原始值应接近 `0x0000`；绕某轴倾斜 90°：对应分量应接近 `0x3C00`（+1.0）或 `0xBC00`（−1.0）；再手算 `w=√(1−x²−y²−z²)` 校验 `\|q\|≈1` | 三种姿态都能对上 half 编码；`\|w\| ≤ 1` 恒成立 |
| 9 | **SFLP 分量顺序确实是 x,y,z** | 分别绕芯片 X/Y/Z 轴做 +90° 静态旋转，看哪个分量变化 | 与 §4.5 的 `0x79=x,0x7B=y,0x7D=z` 一致 |
| 10 | **收敛前芯片输出什么**（全 0 字 或 不写 FIFO） | 使能 SFLP 后立刻以 10 ms 周期记录 `DIFF_FIFO` 与 tag 内容 2 s | 确定是"不写"还是"写 0x0000"；据此简化/保留 §4.7 的窗口逻辑 |
| 11 | **收敛时间 0.8 s / 0.7 s 是否覆盖实际** | 使能后记录第一个"`\|q\|` 稳定且 `\|q\|≈1`"的时刻（静止水平） | 实测 ≤ 1500 ms；超了就把 §7 步 28 的窗口调大 |
| 12 | **`EMB_FUNC_INIT_A(0x66).SFLP_GAME_INIT` 是否自清** | 写 1 后立即回读 | 期望 0；若不自清，需在驱动里手动清（并写回本规范） |
| 13 | **只批 SFLP（BDR_XL=GY=0）时 FIFO 是否按 120 字/s 增长** | 保持 §7 配置，10 s 内累计 `DIFF_FIFO` 增量 | ≈1200 字（允许 ±2%）；若明显偏低说明 SFLP 字还依赖 BDR 设置 → 改用 §5.3 方案 B |
| 14 | **accel/gyro ODR 与 SFLP_ODR 的约束**（数据手册未写） | 试 `ODR_XL=ODR_G=60 Hz` + `SFLP_ODR=120 Hz`，看四元数是否仍按 120 字/s 出 | 若不按 120 出 → 三者必须同速率，写入规范 |
| 15 | **FIFO 实际可用字数**（规范按 1536/7≈219 推算） | 填满 FIFO（continuous 会覆盖），在 `FIFO_FULL_IA` 首次置位时读 `DIFF_FIFO` | 记录实测最大字数并写回 §5.1 |
| 16 | **`FIFO_WTM_IA` / `FIFO_OVR_IA` / `FIFO_FULL_IA` 的清除条件**（数据手册只写了 `FIFO_OVR_LATCHED` 与 `COUNTER_BDR_IA`） | 故意 200 ms 不排空制造溢出，然后分别试：(a) 读 `0x1C`；(b) 排空 FIFO；(c) bypass→continuous | 记录哪种操作让每个位复位；据此确定 §8.3 的恢复动作 |
| 17 | **陀螺 28571…32767 counts 区间是否线性**（±500 dps 标称 vs 17.5 mdps/LSB 的 573 dps） | 用转台从 400 dps 加到 600 dps，记录 count 曲线 | 若 28571 以上就不再增长 → 满量程就是 28571，`GYRO_SAT` 阈值确定；若继续线性 → 记录 32767 对应的实际 dps |
| 18 | **INT1 极化与 EXTI 边沿**（`IF_CFG.H_LACTIVE=0` 默认高有效） | 逻辑分析仪抓 PB0 波形 + `INT1_CTRL=0x08` | 确认有效电平/宽度；据此定 EXTI 触发边沿 |
| 19 | **`DRDY_MASK` 是否影响 FIFO 批处理**（数据手册 §9.17 说它屏蔽 DRDY，未提 FIFO） | 开 `CTRL4=0x08` 与不开各测一次 FIFO 增长 | 若 FIFO 也被屏蔽 → 不要开 `DRDY_MASK` |
| 20 | **温度刻度**（`T = raw/256 + 25`，Table 6：25 °C 时 0 LSB typ.，Toff ±15 °C） | 恒温箱 0/25/50 °C 各静置 5 min 读 `0x20/0x21` | 线性斜率 ≈256 LSB/°C；偏差 ≤ ±15 °C |
| 21 | **`SFLP_GAME_GBIAS*` 的单位与换算**（数据手册只给 binary16 格式，未给物理单位；§8.4 方案 B 需要） | 临时置 `EMB_FUNC_FIFO_EN_A(0x44)` 的 bit5（`SFLP_GBIAS_FIFO_EN=1`），写一个已知 gbias（如 +1000 counts @17.5 mdps/LSB ≈ 17.5 dps）到 `0x6E–0x73`，读 `TAG_SENSOR=0x16`（原始字节 `0xB0`）的 FIFO 字看回读值 | 得到 "gbias 数值 ↔ dps" 的换算系数；据此确定 `gyro_bias[count] → gbias[half]` 的公式 |

### 9.2 文中已显式标注的"不确定/需查原图"汇总

| 位置 | 事项 | 现状 |
|---|---|---|
| §1.2 | Figure 10–12 位级波形未抽取出来 | 依据 §5.1.3 文字取 Mode 3；表 9.1#1 核对 |
| §1.2 | dummy byte | 文本无该词，判定"不需要"；表 9.1#2 核对 |
| §1.3 | 上电 boot 等待时间、`SW_RESET` 完成时间 | 数据手册均未给；暂取 10 ms / 50 ms 超时；表 9.1#4/#5 |
| §4.4 | Table 216 的 tag 位标尺 | 依"5 位 tag 在 bit7:3"→ 原始字节 `0x98`；表 9.1#6 核对（**最高风险**） |
| §4.5 | FIFO 数据寄存器的数值格式 | FIFO 寄存器描述未写；依 §1 Overview + §15.1.1 的 binary16 定义 + 仓库约定；表 9.1#8 核对 |
| §4.6 | `FIFO_WTM_IA`/`FIFO_OVR_IA`/`FIFO_FULL_IA` 的清除条件 | 数据手册只写了另两位；表 9.1#16 核对 |
| §4.7 | 收敛前输出、实际收敛时间 | 数据手册只给 0.8 s + 0.7 s；表 9.1#10/#11 核对 |
| §4.2 | accel/gyro ODR 与 SFLP_ODR 的约束关系 | 数据手册未写；本规范取三者同为 120 Hz；表 9.1#14 核对 |
| §4.3 | SFLP 是否强制某种 FIFO 模式 | 数据手册未写；选 continuous；表 9.1#13 核对 |
| §3.1 | 陀螺 ±500 dps 与 17.5 mdps/LSB 的满量程不一致 | 数据手册两个数都写了但没解释；表 9.1#17 核对 |
| §5.1 | FIFO 确切可用字数 | 只给 1.5 KB/4.5 KB 总量；按 1536/7≈219 推算；表 9.1#15 核对 |
| §8.4 | `SFLP_GAME_GBIAS*` 的物理单位 | 数据手册只给 binary16 格式，没给单位；方案 B 前必须实测；表 9.1#21 核对 |

---

## 附：主机契约的"不许变"清单（实现时逐条自检）

1. 20 字节块布局与 `board.h TELEM_LEN=20` 一致；前 12 字节是 microduck 每 tick 读的部分。
2. `quat x/y/z` 是 **chip frame** 的 SFLP 游戏旋转向量，板上**不做**安装旋转。
3. 四元数只发 x/y/z 的 binary16；w 由主机 `√(1−x²−y²−z²)` 重建。
4. 未收敛 / 故障 → 字节 6..11 **全 0**（主机保持上一帧好值，且不计入 `quat_samples`）。
5. `counter` 每个发布 tick +1，回绕到 0 时置 `TELEM_FLAG_CNT_WRAP`。
6. `TELEM_FLAG_SIMULATED` 必须为 0；`TELEM_FLAG_FROZEN` 不用。
7. `IMU_SAMPLE_HZ = 100` 不改：`imu_tick()` 的 10 ms 网格是外层契约。
8. `bus_smoke.py` 的 `quat_consistency` 中位数阈值 `< 8.0 deg` 必须仍然通过。
