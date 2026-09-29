/*!
    \file    lsm6dsv16x.h
    \brief   LSM6DSV16X register map - only what src/imu_spi.c touches

    Every address, bit field and constant below is from the ST datasheet
    DS13510 Rev 4 ("LSM6DSV16X: 6-axis IMU with embedded sensor fusion, AI and
    Qvar", STMicroelectronics).  Section/table numbers are quoted so a
    disagreement with the silicon can be settled against the source.

    Two address spaces exist, switched by FUNC_CFG_ACCESS (01h):

      * the main page (Table 24, "Registers address map"), and
      * the embedded-functions page (Table 262), reachable only while
        EMB_FUNC_REG_ACCESS is 1.  `SFLP_ODR` is 5Eh in *that* page, which
        collides with MD1_CFG (5Eh) on the main page - hence LSM_EMB_ADDR().

    Read/write framing: the primary interface is 4-wire SPI (IF_CFG.SIM = 0),
    address in bit 7 of the first byte (1 = read), MSB first, auto-increment
    through CTRL3.IF_INC.
*/

#ifndef LSM6DSV16X_H
#define LSM6DSV16X_H

#include <stdint.h>

/* ── identity ───────────────────────────────────────────────────────────── */

#define LSM_WHO_AM_I            0x0F    /* §9.13: fixed at 70h */
#define LSM_WHO_AM_I_VALUE      0x70

/* ── main-page registers (Table 24) ─────────────────────────────────────── */

#define LSM_FUNC_CFG_ACCESS     0x01
#define LSM_EMB_REG_ACCESS      0x80    /* §9.1: 1 = embedded page selected */
#define LSM_PIN_CTRL            0x02
#define LSM_IF_CFG              0x03
#define LSM_FIFO_CTRL1          0x07
#define LSM_FIFO_CTRL2          0x08
#define LSM_FIFO_CTRL3          0x09
#define LSM_FIFO_CTRL4          0x0A
#define LSM_INT1_CTRL           0x0D
#define LSM_INT2_CTRL           0x0E
#define LSM_CTRL1               0x10    /* accelerometer ODR + operating mode */
#define LSM_CTRL2               0x11    /* gyroscope ODR + operating mode     */
#define LSM_CTRL3               0x12    /* BDU / IF_INC / SW_RESET            */
#define LSM_CTRL6               0x15    /* gyroscope full scale               */
#define LSM_CTRL8               0x17    /* accelerometer full scale           */
#define LSM_FIFO_STATUS1        0x1B
#define LSM_FIFO_STATUS2        0x1C
#define LSM_OUTX_L_G            0x22
#define LSM_OUTX_L_A            0x28
#define LSM_FIFO_DATA_OUT_TAG   0x78

/* IF_CFG (03h, §9.3, Table 30).  Bit 0 disables I2C/I3C: the part is wired for
   4-wire SPI and only CS selects the interface, so leaving I2C enabled only
   creates a way to latch the wrong bus.  Not cleared by SW_RESET. */
#define LSM_IF_CFG_I2C_I3C_DIS  0x01

/* CTRL3 (12h, §9.16, Table 57): default 44h = BDU | IF_INC, which is what a
   coherent multi-byte read needs.  SW_RESET is self-clearing. */
#define LSM_CTRL3_BDU           0x40
#define LSM_CTRL3_IF_INC        0x04
#define LSM_CTRL3_SW_RESET      0x01
#define LSM_CTRL3_DEFAULT       0x44
#define LSM_CTRL3_BOOT          0x80

/* CTRL1/CTRL2 layout (§9.14/§9.15, Tables 50/53): [0, OP_MODE[2:0], ODR[3:0]].
   OP_MODE 000 = high-performance; ODR 0110 = 120 Hz (Tables 52/55). */
#define LSM_ODR_120HZ_HP        0x06

/* CTRL6 (15h, Table 63): FS_G 0010 = ±500 dps  -> 17.50 mdps/LSB  (§4.6.1)
   CTRL8 (17h, Table 68): FS_XL 01   = ±4 g     -> 0.122 mg/LSB     (§4.6.1) */
#define LSM_FS_G_500DPS         0x02
#define LSM_FS_XL_4G            0x01

/* FIFO_CTRL4 (0Ah, Table 40) FIFO_MODE[2:0]: 110 = continuous mode, where a
   full FIFO overwrites the oldest word.  For a 100 Hz poller that is the right
   failure mode: a stalled main loop loses history instead of wedging the FIFO,
   and the watermark/overrun flags still say it happened. */
#define LSM_FIFO_MODE_CONTINUOUS 0x06
#define LSM_FIFO_MODE_BYPASS     0x00

/* FIFO_CTRL1 (07h, Table 34): watermark in words (1 word = tag + 6 bytes).
   12 words = 100 ms of SFLP at 120 Hz - deep enough that a normal main-loop
   stall never gets near it, shallow enough to be an early warning. */
#define LSM_FIFO_WTM_WORDS      12U
#define LSM_FIFO_WORD_LEN       7U

/* INT1_CTRL (0Dh, Table 46): route the watermark and overrun signals to INT1
   (PB0) so the pin is meaningful even though the driver polls. */
#define LSM_INT1_FIFO_TH        0x08
#define LSM_INT1_FIFO_OVR       0x10
#define LSM_INT1_FIFO_FULL      0x20
#define LSM_INT1_ROUTE          (LSM_INT1_FIFO_TH | LSM_INT1_FIFO_OVR)

/* FIFO_STATUS2 (1Ch, Table 79): DIFF_FIFO_8 in bit 0, overrun flags above it.
   Reading the register clears the latched overrun. */
#define LSM_FIFO_ST2_DIFF8      0x01
#define LSM_FIFO_ST2_OVR_LATCH  0x08
#define LSM_FIFO_ST2_OVR_IA     0x40
#define LSM_FIFO_ST2_WTM_IA     0x80

/* FIFO_DATA_OUT_TAG (78h, Table 217/218): TAG_SENSOR[4:0] sits in bits 7:3 and
   TAG_CNT[1:0] in bits 2:1, so the *byte* for a sensor is its Table 218 code
   shifted left by 3 - comparing the byte to 0x13 would never match. */
#define LSM_TAG_SENSOR(tag)     ((uint8_t)((tag) >> 3))
#define LSM_TAG_SFLP_GAME       0x13    /* Table 218: SFLP game rotation vector */
#define LSM_TAG_SFLP_GBIAS      0x16
#define LSM_TAG_SFLP_GRAVITY    0x17

/* ── embedded-functions page (Table 262) ────────────────────────────────── */

#define LSM_EMB_FUNC_EN_A       0x04    /* bit 1 = SFLP_GAME_EN (§13.2)        */
#define LSM_EMB_FUNC_FIFO_EN_A  0x44    /* bit 1 = SFLP_GAME_FIFO_EN (§13.17)  */
#define LSM_EMB_FUNC_INIT_A     0x66    /* bit 1 = SFLP_GAME_INIT (§13.32)     */
#define LSM_EMB_SFLP_ODR        0x5E    /* §13.30, Table 324                   */

/* The datasheet's Table 295 wraps the column headers over three lines and the
   text extraction puts the SFLP_GAME_FIFO_EN column next to a reserved 0, so
   the bit cannot be read off the extracted table with confidence.  The
   positions below were confirmed against ST's own register definition
   (`lsm6dsv16x_reg.h`, `lsm6dsv16x_emb_func_fifo_en_a_t`, field order is
   bit0-first: not_used0, sflp_game_fifo_en, not_used1[2],
   sflp_gravity_fifo_en, sflp_gbias_fifo_en, step_counter_fifo_en,
   mlc_fifo_en).  Writing 0x01 instead of 0x02 is silent: the SFLP runs, the
   FIFO simply never receives a word - measured 0 sflp_words for 8 s on the
   real PCBA before the fix. */
#define LSM_SFLP_GAME_EN        0x02
#define LSM_SFLP_GAME_FIFO_EN   0x02
#define LSM_SFLP_GAME_INIT      0x02

/* SFLP_ODR (5Eh) is [0, 1, SFLP_GAME_ODR[2:0], 0, 1, 1]; 011 = 120 Hz, which
   is also its reset value.  Reset value 5Bh. */
#define LSM_SFLP_ODR_120HZ      0x5B

/* §2.8 Table 1: SFLP calibration 0.8 s + orientation stabilisation 0.7 s, so
   the quaternion is only meaningful ~1.5 s after the algorithm is enabled.

   The chip does *not* signal that window by staying quiet: measured on the real
   part, it keeps emitting well-formed SFLP words at ~115.5/s straight through a
   `SFLP_GAME_INIT` pulse.  So "wait for the first word" is not "wait for a
   trustworthy attitude", and the driver has to hold the window itself - that is
   what LSM_SFLP_CONVERGE_MS is for (see src/imu_spi.c's CONVERGE state). */
#define LSM_SFLP_CONVERGE_MS    1500U

/*! \brief register address in the embedded-functions page (for documentation:
    the caller must have set EMB_FUNC_REG_ACCESS first). */
#define LSM_EMB_ADDR(a)         (a)

#endif /* LSM6DSV16X_H */
