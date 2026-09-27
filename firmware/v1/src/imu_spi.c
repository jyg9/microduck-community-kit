/*!
    \file    imu_spi.c
    \brief   LSM6DSV16X over SPI0: the real sensor block

    Replaces the simulator when IMU_USE_SPI is set (board.h).  The chip does the
    fusion: SFLP is enabled and its game rotation vector is batched into the
    FIFO, so the MCU only has to drain the FIFO and copy gyro/accelerometer
    counts - no floating-point integration on the 120 MHz M4F, and the attitude
    is bit-for-bit what the original board produced.

    Data path, per 10 ms sample (src/imu.c drives the grid):

      1. FIFO_STATUS1/2    -> how many (tag + 6 byte) words are waiting
      2. burst read from FIFO_DATA_OUT_TAG, 7 bytes per word; keep the newest
         word whose TAG_SENSOR is 0x13 (SFLP game rotation vector)
      3. burst read OUTX_L_G / OUTX_L_A (6 bytes each, BDU protects the pair)
      4. compose the 20-byte block in chip frame - exactly the wire contract
         from src/imu.h

    Register values and the reasoning behind them live in src/lsm6dsv16x.h with
    datasheet citations.  Deliberate choices worth repeating here:

      * FIFO_CTRL3 = 0: the FIFO carries *only* SFLP words.  Gyro and accel are
        read from their output registers, which is cheaper and keeps the drain
        loop trivial (fixed 7-byte stride).
      * FIFO mode is continuous, so falling behind loses the oldest history
        instead of wedging; the latched overrun flag counts that as an event.
      * The quaternion is passed through as raw binary16.  The host owns the
        norm/plausibility rules (duck-control/src/imu.rs); the node only reports
        "no SFLP word yet" by leaving the six bytes zero, which is the documented
        "keep your last good value" signal.
*/

#include "imu_spi.h"

#include <math.h>
#include <string.h>

#include "board.h"
#include "dbg.h"
#include "dev.h"
#include "imu_math.h"
#include "lsm6dsv16x.h"
#include "systick.h"

/* Words drained per tick.  32 words = 266 ms of SFLP at 120 Hz: far more than
   one tick can need, small enough that the burst buffer stays on the stack. */
#define IMU_SPI_DRAIN_MAX   32U
#define IMU_SPI_BURST_MAX   (IMU_SPI_DRAIN_MAX * LSM_FIFO_WORD_LEN)

/* Re-probe a missing chip at this rate instead of every tick. */
#define IMU_SPI_REPROBE_MS  1000U

/* Datasheet-mandated settling: SW_RESET and the BOOT reload both need the
   internal filters to restart before the first register access is meaningful.
   10 ms is what the vendor driver uses and is 100x the typical figure. */
#define IMU_SPI_RESET_MS    10U

static uint8_t      s_block[TELEM_LEN];
static imu_sample_t s_sample;
static uint32_t     s_samples;
static uint8_t      s_counter;
static uint8_t      s_quat_valid;

static uint8_t      s_present;
static uint8_t      s_whoami;
static uint32_t     s_sflp_words;
static uint32_t     s_overruns;
static uint32_t     s_bad_tags;
static uint32_t     s_probe_fails;
static uint32_t     s_next_probe_ms;

/* SFLP convergence window.

   The chip keeps emitting *well-formed* SFLP words while its fusion is still
   settling - a `SFLP_GAME_INIT` pulse does not blank the output, it only
   restarts the algorithm behind it (measured on the real part: `sflp_words`
   keeps climbing at ~116/s straight through a restart).  So "wait for the first
   word" is not the same as "wait for a trustworthy attitude", and publishing
   one during the window would hand the host an attitude the datasheet says is
   not yet steady (0.8 s calibration + 0.7 s stabilisation, §2.8 Table 1).

   While this flag is set the driver publishes the documented signal instead:
   gyro and accelerometer are real and current, the six quaternion bytes are
   zero and TELEM_FLAG_SFLP_VALID is clear - which is exactly what the host's
   `SflpDecoder` reads as "hold your last good value", and what keeps its
   25-good-sample `ready()` gate from being satisfied by settling data. */
static uint8_t      s_converging;
static uint32_t     s_converge_until_ms;

/* ── SPI0 transport ─────────────────────────────────────────────────────── */

static void cs_low(void)
{
    gpio_bit_reset(IMU_SPI_GPIO, IMU_CS_PIN);
}

static void cs_high(void)
{
    gpio_bit_set(IMU_SPI_GPIO, IMU_CS_PIN);
}

static uint8_t spi_xfer(uint8_t out)
{
    uint32_t guard = 0U;

    while ((RESET == spi_i2s_flag_get(IMU_SPI, SPI_FLAG_TBE)) && (guard++ < 100000U)) {
    }
    spi_i2s_data_transmit(IMU_SPI, out);
    guard = 0U;
    while ((RESET == spi_i2s_flag_get(IMU_SPI, SPI_FLAG_RBNE)) && (guard++ < 100000U)) {
    }
    return (uint8_t)spi_i2s_data_receive(IMU_SPI);
}

/*! \brief read `n` bytes starting at `reg` (auto-increment through IF_INC). */
static void lsm_read(uint8_t reg, uint8_t *buf, uint16_t n)
{
    uint16_t i;

    cs_low();
    (void)spi_xfer((uint8_t)(reg | 0x80U));
    for (i = 0U; i < n; i++) {
        buf[i] = spi_xfer(0x00U);
    }
    cs_high();
}

static uint8_t lsm_read8(uint8_t reg)
{
    uint8_t v = 0U;

    lsm_read(reg, &v, 1U);
    return v;
}

static void lsm_write(uint8_t reg, uint8_t value)
{
    cs_low();
    (void)spi_xfer((uint8_t)(reg & 0x7FU));
    (void)spi_xfer(value);
    cs_high();
}

/*! \brief select the embedded-functions register page (FUNC_CFG_ACCESS). */
static void lsm_emb_enter(void)
{
    lsm_write(LSM_FUNC_CFG_ACCESS, LSM_EMB_REG_ACCESS);
}

static void lsm_emb_leave(void)
{
    lsm_write(LSM_FUNC_CFG_ACCESS, 0x00U);
}

/*! \brief read-modify-write one embedded-page register. */
static void lsm_emb_or(uint8_t reg, uint8_t bits)
{
    uint8_t v;

    lsm_emb_enter();
    v = lsm_read8(reg);
    lsm_write(reg, (uint8_t)(v | bits));
    lsm_emb_leave();
}

/*! \brief read back everything the driver configured, for the console.

    A silent SFLP is the one failure mode that looks like success everywhere
    else (WHO_AM_I answers, gyro counts move), so the state that decides whether
    the fusion block is actually running is worth printing rather than
    inferring.  Normal builds pay for it once, at init. */
static void imu_spi_dump(const char *tag)
{
    uint8_t e[4];
    uint8_t sflp;

    lsm_emb_enter();
    sflp = lsm_read8(LSM_EMB_SFLP_ODR);
    lsm_read(LSM_EMB_FUNC_EN_A, e, 2U);          /* EN_A (04h), EN_B (05h)    */
    e[2] = lsm_read8(LSM_EMB_FUNC_INIT_A);
    e[3] = lsm_read8(LSM_EMB_FUNC_FIFO_EN_A);
    lsm_emb_leave();

    DBG_INFO("%s ctrl1=%02X ctrl2=%02X ctrl3=%02X ctrl6=%02X ctrl8=%02X",
             tag, (unsigned)lsm_read8(LSM_CTRL1), (unsigned)lsm_read8(LSM_CTRL2),
             (unsigned)lsm_read8(LSM_CTRL3), (unsigned)lsm_read8(LSM_CTRL6),
             (unsigned)lsm_read8(LSM_CTRL8));
    DBG_INFO("%s fifo_ctrl1=%02X ctrl2=%02X ctrl3=%02X ctrl4=%02X int1=%02X "
             "func_cfg_access=%02X",
             tag, (unsigned)lsm_read8(LSM_FIFO_CTRL1),
             (unsigned)lsm_read8(LSM_FIFO_CTRL2), (unsigned)lsm_read8(LSM_FIFO_CTRL3),
             (unsigned)lsm_read8(LSM_FIFO_CTRL4), (unsigned)lsm_read8(LSM_INT1_CTRL),
             (unsigned)lsm_read8(LSM_FUNC_CFG_ACCESS));
    DBG_INFO("%s emb en_a=%02X en_b=%02X init_a=%02X fifo_en_a=%02X sflp_odr=%02X "
             "(want en_a bit1, fifo_en_a bit0, sflp_odr %02X)",
             tag, (unsigned)e[0], (unsigned)e[1], (unsigned)e[2], (unsigned)e[3],
             (unsigned)sflp, (unsigned)LSM_SFLP_ODR_120HZ);
}

/* ── block composition ──────────────────────────────────────────────────── */

static void put16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFU);
    p[1] = (uint8_t)(v >> 8);
}

static void set_block_quat(const uint16_t q[3])
{
    put16(&s_block[6],  q[0]);
    put16(&s_block[8],  q[1]);
    put16(&s_block[10], q[2]);
    s_sample.quat[0] = q[0];
    s_sample.quat[1] = q[1];
    s_sample.quat[2] = q[2];
}

/*! \brief start the convergence window: publish zero quaternion bytes and stop
    accepting SFLP words until the fusion has settled. */
static void imu_spi_begin_converge(void)
{
    static const uint16_t zero[3] = { 0U, 0U, 0U };

    s_converging = 1U;
    s_converge_until_ms = systick_get_ms() + LSM_SFLP_CONVERGE_MS;
    s_quat_valid = 0U;
    set_block_quat(zero);
}

/*! \brief leave the convergence window, dropping everything the FIFO collected
    while the fusion was settling - none of those words describe the attitude
    we are about to publish. */
static void imu_spi_end_converge(void)
{
    lsm_write(LSM_FIFO_CTRL4, LSM_FIFO_MODE_BYPASS);   /* resets the FIFO */
    lsm_write(LSM_FIFO_CTRL4, LSM_FIFO_MODE_CONTINUOUS);
    s_converging = 0U;
}

/* ── init ───────────────────────────────────────────────────────────────── */

static void imu_spi_pins(void)
{
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_AF);
    rcu_periph_clock_enable(RCU_SPI0);

    /* CS is ours, driven high between transfers */
    gpio_init(IMU_SPI_GPIO, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, IMU_CS_PIN);
    gpio_bit_set(IMU_SPI_GPIO, IMU_CS_PIN);

    /* SCK and MOSI are outputs from the master; MISO is the chip's answer.
       The dev board's board_gpio_init() leaves all three floating as a safe
       default for a simulated build - this is where they become a bus. */
    gpio_init(IMU_SPI_GPIO, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ,
              IMU_SCK_PIN | IMU_MOSI_PIN);
    gpio_init(IMU_SPI_GPIO, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, IMU_MISO_PIN);

    /* INT1/INT2 (PB0/PB1) stay inputs with a pull-up: the driver polls, but the
       chip is configured to raise INT1 on FIFO watermark/overrun, so a future
       interrupt path or a scope hook has something to look at. */
    gpio_init(IMU_INT1_GPIO, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ,
              IMU_INT1_PIN | IMU_INT2_PIN);
}

static void imu_spi_bus_init(void)
{
    spi_parameter_struct spi;

    spi_i2s_deinit(IMU_SPI);
    spi_struct_para_init(&spi);
    spi.device_mode          = SPI_MASTER;
    spi.trans_mode           = SPI_TRANSMODE_FULLDUPLEX;
    spi.frame_size           = SPI_FRAMESIZE_8BIT;
    spi.nss                  = SPI_NSS_SOFT;
    spi.endian               = SPI_ENDIAN_MSB;
    spi.clock_polarity_phase = SPI_CK_PL_LOW_PH_1EDGE;   /* SPI mode 0 */
    /* APB2 = 120 MHz; /16 = 7.5 MHz, inside the 10 MHz the part allows and
       slow enough to survive a jumper wire on the bench. */
    spi.prescale              = SPI_PSC_16;
    spi_init(IMU_SPI, &spi);
    spi_enable(IMU_SPI);
}

/*! \brief reset, probe and configure the chip.
    \retval 1 the chip answered WHO_AM_I and is configured and running. */
static int imu_spi_configure(void)
{
    uint8_t who;
    uint8_t ctrl3;
    uint32_t guard;

    /* software reset; IF_CFG is deliberately not reset by it */
    lsm_write(LSM_CTRL3, LSM_CTRL3_SW_RESET);
    delay_1ms(IMU_SPI_RESET_MS);
    for (guard = 0U; guard < 20U; guard++) {
        ctrl3 = lsm_read8(LSM_CTRL3);
        if (0U == (ctrl3 & LSM_CTRL3_SW_RESET)) {
            break;
        }
        delay_1ms(1U);
    }

    who = lsm_read8(LSM_WHO_AM_I);
    s_whoami = who;
    if (LSM_WHO_AM_I_VALUE != who) {
        /* Nothing is listening, or the part is not in SPI mode: leave CS high
           and the bus idle, and let the retry path try again later. */
        return 0;
    }

    /* SPI only: the interface is selected by CS, so the I2C/I3C decoder can
       only ever be a way to latch the wrong bus (IF_CFG survives SW_RESET). */
    lsm_write(LSM_IF_CFG, LSM_IF_CFG_I2C_I3C_DIS);

    /* BDU + IF_INC: coherent 6-byte gyro/accel reads */
    lsm_write(LSM_CTRL3, LSM_CTRL3_BDU | LSM_CTRL3_IF_INC);

    /* full scales that match the wire contract */
    lsm_write(LSM_CTRL6, LSM_FS_G_500DPS);
    lsm_write(LSM_CTRL8, LSM_FS_XL_4G);

    /* FIFO: only SFLP words, continuous (newest wins), watermark 12 words */
    lsm_write(LSM_FIFO_CTRL1, (uint8_t)LSM_FIFO_WTM_WORDS);
    lsm_write(LSM_FIFO_CTRL2, 0x00U);
    lsm_write(LSM_FIFO_CTRL3, 0x00U);
    lsm_write(LSM_FIFO_CTRL4, LSM_FIFO_MODE_BYPASS);
    lsm_write(LSM_INT1_CTRL, LSM_INT1_ROUTE);

    /* SFLP: enable the algorithm and its FIFO batching at 120 Hz */
    lsm_emb_enter();
    lsm_write(LSM_EMB_SFLP_ODR, LSM_SFLP_ODR_120HZ);
    lsm_emb_or(LSM_EMB_FUNC_EN_A, LSM_SFLP_GAME_EN);
    lsm_emb_or(LSM_EMB_FUNC_FIFO_EN_A, LSM_SFLP_GAME_FIFO_EN);
    lsm_emb_leave();

    /* flush whatever the FIFO collected before it was configured, then run */
    lsm_write(LSM_FIFO_CTRL4, LSM_FIFO_MODE_CONTINUOUS);

    /* sensors last: they are what starts producing words */
    lsm_write(LSM_CTRL1, LSM_ODR_120HZ_HP);
    lsm_write(LSM_CTRL2, LSM_ODR_120HZ_HP);

    s_quat_valid = 0U;
    set_block_quat(s_sample.quat);   /* publishes the zeroed bytes too */

    DBG_INFO("lsm6dsv16x: who=0x%02X id ok, spi0 7.5 MHz, gyro +-500dps/120Hz, "
             "xl +-4g/120Hz, sflp 120Hz, wtm %u words",
             (unsigned)who, (unsigned)LSM_FIFO_WTM_WORDS);
    imu_spi_dump("lsm6dsv16x:");

    /* The algorithm has just been started; it needs the same settling window a
       restart does before its attitude means anything. */
    imu_spi_begin_converge();
    return 1;
}

void imu_spi_init(void)
{
    memset(s_block, 0, sizeof(s_block));
    memset(&s_sample, 0, sizeof(s_sample));
    s_samples = 0U;
    s_counter = 0U;
    s_quat_valid = 0U;
    s_sflp_words = 0U;
    s_overruns = 0U;
    s_bad_tags = 0U;
    s_probe_fails = 0U;
    s_present = 0U;
    s_whoami = 0U;
    s_converging = 0U;
    s_converge_until_ms = 0U;
    s_next_probe_ms = systick_get_ms() + IMU_SPI_REPROBE_MS;

    imu_spi_pins();
    imu_spi_bus_init();

    if (imu_spi_configure()) {
        s_present = 1U;
    } else {
        s_probe_fails++;
        DBG_ERROR("lsm6dsv16x: WHO_AM_I=0x%02X (want 0x%02X) - sensor absent",
                  (unsigned)s_whoami, (unsigned)LSM_WHO_AM_I_VALUE);
    }
    imu_spi_step();
}

void imu_spi_restart(void)
{
    if (0U == s_present) {
        return;
    }
    /* SFLP_GAME_INIT restarts the fusion; the attitude is meaningless until it
       converges again, so hold the quaternion at zero for the whole window. */
    lsm_emb_or(LSM_EMB_FUNC_INIT_A, LSM_SFLP_GAME_INIT);
    imu_spi_begin_converge();
    DBG_INFO("lsm6dsv16x: SFLP restarted, quaternion held at zero for %u ms",
             (unsigned)LSM_SFLP_CONVERGE_MS);
}

/* ── the 10 ms step ─────────────────────────────────────────────────────── */

/*! \brief drain the FIFO and keep the newest SFLP quaternion word.
    \retval 1 a quaternion word was consumed. */
static int imu_spi_drain_sflp(void)
{
    static uint8_t buf[IMU_SPI_BURST_MAX];
    uint8_t st[2];
    uint16_t words;
    uint16_t n;
    uint16_t i;
    uint16_t q[3];
    int got = 0;

    lsm_read(LSM_FIFO_STATUS1, st, 2U);
    words = (uint16_t)(((uint16_t)(st[1] & LSM_FIFO_ST2_DIFF8) << 8) | st[0]);

    /* The latched overrun bit is cleared by the read above, so it counts one
       event per overrun rather than one per poll. */
    if (0U != (st[1] & LSM_FIFO_ST2_OVR_LATCH)) {
        s_overruns++;
    }
    if (0U == words) {
        return 0;
    }

    n = (words > (uint16_t)IMU_SPI_DRAIN_MAX) ? (uint16_t)IMU_SPI_DRAIN_MAX
                                              : words;
    lsm_read(LSM_FIFO_DATA_OUT_TAG, buf, (uint16_t)(n * LSM_FIFO_WORD_LEN));

    /* Oldest first; the last SFLP word in the batch is the newest attitude. */
    for (i = 0U; i < n; i++) {
        const uint8_t *w = &buf[i * LSM_FIFO_WORD_LEN];

        if (LSM_TAG_SFLP_GAME != LSM_TAG_SENSOR(w[0])) {
            s_bad_tags++;
            continue;
        }
        q[0] = (uint16_t)((uint16_t)w[1] | ((uint16_t)w[2] << 8));
        q[1] = (uint16_t)((uint16_t)w[3] | ((uint16_t)w[4] << 8));
        q[2] = (uint16_t)((uint16_t)w[5] | ((uint16_t)w[6] << 8));
        s_sflp_words++;
        got = 1;
    }

    if (0U != got) {
        /* All-zero components are how the chip says "nothing computed yet";
           that is the documented signal for the host to keep its last value. */
        if ((0U == q[0]) && (0U == q[1]) && (0U == q[2])) {
            s_quat_valid = 0U;
            memset(&s_block[6], 0, 6U);
            s_sample.quat[0] = 0U;
            s_sample.quat[1] = 0U;
            s_sample.quat[2] = 0U;
            got = 0;
        } else {
            s_quat_valid = 1U;
            set_block_quat(q);
        }
    }
    return got;
}

void imu_spi_step(void)
{
    const dev_cfg_t *cfg = dev_cfg();
    uint8_t raw_g[6];
    uint8_t raw_a[6];
    uint8_t flags = 0U;
    int16_t gyro[3];
    int16_t accel[3];
    int i;

    if (0U == s_present) {
        /* keep trying: a bench wiring fix should recover without a power cycle */
        if ((int32_t)(systick_get_ms() - s_next_probe_ms) >= 0) {
            s_next_probe_ms = systick_get_ms() + IMU_SPI_REPROBE_MS;
            if (imu_spi_configure()) {
                s_present = 1U;
                DBG_INFO("lsm6dsv16x: sensor came up on retry (who=0x%02X)",
                         (unsigned)s_whoami);
            } else {
                s_probe_fails++;
            }
        }
    }

    if (0U != s_present) {
        if (0U != s_converging) {
            /* Deliberately not draining: the FIFO wraps in continuous mode and
               every word in it was produced before the fusion settled.  The
               window is bounded, and it is flushed when it ends. */
            if ((int32_t)(systick_get_ms() - s_converge_until_ms) >= 0) {
                imu_spi_end_converge();
                DBG_INFO("lsm6dsv16x: SFLP converged after %u ms, publishing attitude",
                         (unsigned)LSM_SFLP_CONVERGE_MS);
            }
        } else {
            (void)imu_spi_drain_sflp();
            if (0U == s_sflp_words) {
                /* The fusion block is the one thing that can be silent while
                   every other read looks healthy; say so once, with the state
                   that decides it, instead of reporting "present" forever. */
                static uint8_t warned;
                static uint32_t started_ms;

                if (0U == started_ms) {
                    started_ms = systick_get_ms();
                }
                if ((0U == warned)
                    && ((uint32_t)(systick_get_ms() - started_ms) > LSM_SFLP_CONVERGE_MS)) {
                    warned = 1U;
                    DBG_ERROR("lsm6dsv16x: no SFLP word after %u ms - fusion not running",
                              (unsigned)LSM_SFLP_CONVERGE_MS);
                    imu_spi_dump("lsm6dsv16x!");
                }
            }
        }
        lsm_read(LSM_OUTX_L_G, raw_g, 6U);
        lsm_read(LSM_OUTX_L_A, raw_a, 6U);
        for (i = 0; i < 3; i++) {
            gyro[i]  = (int16_t)((uint16_t)raw_g[2 * i] | ((uint16_t)raw_g[2 * i + 1] << 8));
            accel[i] = (int16_t)((uint16_t)raw_a[2 * i] | ((uint16_t)raw_a[2 * i + 1] << 8));
        }
    } else {
        gyro[0] = gyro[1] = gyro[2] = 0;
        accel[0] = accel[1] = accel[2] = 0;
    }

    /* register-injected gyro bias, same hook the simulator offers */
    for (i = 0; i < 3; i++) {
        int32_t v = (int32_t)gyro[i] + (int32_t)cfg->gyro_bias[i];
        if (v > 32767) {
            v = 32767;
            flags |= TELEM_FLAG_GYRO_SAT;
        } else if (v < -32768) {
            v = -32768;
            flags |= TELEM_FLAG_GYRO_SAT;
        }
        gyro[i] = (int16_t)v;
    }
    if ((32767 == gyro[0]) || (-32768 == gyro[0])
        || (32767 == gyro[1]) || (-32768 == gyro[1])
        || (32767 == gyro[2]) || (-32768 == gyro[2])) {
        flags |= TELEM_FLAG_GYRO_SAT;
    }

    /* Trunk-frame mode is a bench convenience: the host normally applies the
       mounting rotation itself, and the default (chip frame) is what the
       original board reported. */
    if (0U != cfg->report_frame) {
        quat_t mount;
        float w_in[3];
        float w_out[3];
        quat_t q_chip;

        mount.w = MOUNT_QW; mount.x = MOUNT_QX;
        mount.y = MOUNT_QY; mount.z = MOUNT_QZ;
        for (i = 0; i < 3; i++) {
            w_in[i] = (float)gyro[i];
        }
        quat_rotate(mount, w_in, w_out);
        for (i = 0; i < 3; i++) {
            gyro[i] = (int16_t)lroundf_i32(w_out[i]);
        }
        q_chip.x = half_to_f32(s_sample.quat[0]);
        q_chip.y = half_to_f32(s_sample.quat[1]);
        q_chip.z = half_to_f32(s_sample.quat[2]);
        /* X/Y/Z only: the scalar part is reconstructed, which is the whole
           point of the on-the-wire format. */
        {
            float w2 = 1.0f - (q_chip.x * q_chip.x + q_chip.y * q_chip.y
                               + q_chip.z * q_chip.z);
            q_chip.w = (w2 > 0.0f) ? sqrtf(w2) : 1.0f;
        }
        if (0U != s_quat_valid) {
            quat_t q_trunk = quat_mul(q_chip, quat_conj(mount));
            uint16_t h[3];

            h[0] = f32_to_half(q_trunk.x);
            h[1] = f32_to_half(q_trunk.y);
            h[2] = f32_to_half(q_trunk.z);
            set_block_quat(h);
        }
    }

    put16(&s_block[0], (uint16_t)gyro[0]);
    put16(&s_block[2], (uint16_t)gyro[1]);
    put16(&s_block[4], (uint16_t)gyro[2]);
    put16(&s_block[12], (uint16_t)accel[0]);
    put16(&s_block[14], (uint16_t)accel[1]);
    put16(&s_block[16], (uint16_t)accel[2]);

    s_sample.gyro[0] = gyro[0];
    s_sample.gyro[1] = gyro[1];
    s_sample.gyro[2] = gyro[2];
    s_sample.accel[0] = accel[0];
    s_sample.accel[1] = accel[1];
    s_sample.accel[2] = accel[2];

    /* FUSION_OK means "the algorithm is running", SFLP_VALID means "this block
       carries a quaternion".  Between them they reproduce the chip's own
       start-up: the algorithm is on for the ~1.5 s it needs to converge, with
       the six quaternion bytes left at zero, which is exactly the state the
       host's "keep your last good value" rule was written for. */
    if (0U != s_present) {
        flags |= TELEM_FLAG_FUSION_OK;
    }
    if (0U != s_quat_valid) {
        flags |= TELEM_FLAG_SFLP_VALID;
    }
    if (0U == s_present) {
        flags |= TELEM_FLAG_SENSOR_ERR;
    }
    if (0xFFU == s_counter) {
        flags |= TELEM_FLAG_CNT_WRAP;
    }

    s_block[18] = s_counter++;
    s_block[19] = flags;
    s_sample.counter = s_block[18];
    s_sample.flags = flags;
    s_sample.quat_valid = s_quat_valid;
    s_samples++;
}

const uint8_t *imu_spi_block(void)
{
    return s_block;
}

const imu_sample_t *imu_spi_sample(void)
{
    return &s_sample;
}

uint32_t imu_spi_sample_count(void)
{
    return s_samples;
}

uint8_t imu_spi_present(void)
{
    return s_present;
}

uint8_t imu_spi_whoami(void)
{
    return s_whoami;
}

uint32_t imu_spi_sflp_words(void)
{
    return s_sflp_words;
}

uint32_t imu_spi_fifo_overruns(void)
{
    return s_overruns;
}

uint32_t imu_spi_bad_tags(void)
{
    return s_bad_tags;
}

uint32_t imu_spi_probe_fails(void)
{
    return s_probe_fails;
}
