/*!
    \file    imu.c
    \brief   sensor facade: one 20-byte block, two possible sources

    Everything above this file (the register image, the protocol slaves, the
    host) sees only the block described in src/imu.h.  Which source produces it
    is a build option (IMU_USE_SPI in board.h):

      * IMU_USE_SPI = 1 - src/imu_spi.c drives a real LSM6DSV16X on SPI0 and
        takes its SFLP game rotation vector out of the FIFO;
      * IMU_USE_SPI = 0 - src/imu_sim.c integrates the quaternion from the same
        angular velocity it reports, which is how the whole stack was developed
        and is still what the bench smoke test's simulation modes use.

    The 10 ms sampling grid lives here rather than in either source, so both
    produce samples on exactly the same cadence and neither has to know how the
    main loop calls it.
*/

#include "imu.h"

#include "imu_math.h"
#include "systick.h"

#if IMU_USE_SPI
#include "imu_spi.h"
#else
#include "imu_sim.h"
#endif

/* ── the FT6 quaternion sequence ──────────────────────────────────────────
   Kept here, in the facade, rather than in either backend: imu.c owns the
   sampling grid, and both backends already know when they produced a *new*
   attitude.  Putting it here covers both backends and every simulator mode
   without either source learning about the FT6 contract.

   "New" has to mean new, though.  The simulator recomputes its liveness every
   10 ms step, so `quat_valid` is a per-step fact there.  The SPI backend's
   `s_quat_valid` is a *latch*: it is set when a nonzero SFLP word is consumed
   and left alone when the FIFO has nothing (imu_spi_drain_sflp() returns early
   on words == 0).  Counting that latch per tick would advance the sequence -
   and refresh s_quat_ms - while the fusion is stalled, which is exactly the
   failure the FT6 counter exists to expose.  So the SPI side counts consumed
   SFLP words instead, and the all-zero test that FT6 applies to `ready` stays
   in imu_quat_ready(): the FT6 node advances its counter even for an all-zero
   word and lets the ready bit carry that meaning. */
static uint32_t s_quat_seq;
static uint32_t s_quat_ms;
#if IMU_USE_SPI
static uint32_t s_sflp_words;
#endif

/*! \brief 1 when this 10 ms step consumed a new attitude.
    SPI: a new SFLP word left the FIFO (the same signal the FT6 node counts).
    Simulator: the block it just published is a live one. */
static int quat_produced(void)
{
#if IMU_USE_SPI
    uint32_t words = imu_spi_sflp_words();

    if (words == s_sflp_words) {
        return 0;
    }
    s_sflp_words = words;
    return 1;
#else
    return (0U != imu_latest()->quat_valid) ? 1 : 0;
#endif
}

/*! \brief 1 when the six quaternion bytes carry an attitude at all.  All-zero
           components are how the chip says "nothing computed yet" - the FT6
           node's compact_snapshot() applies the same test to `ready`. */
static int quat_bytes_live(void)
{
    const uint8_t *b = imu_block_bytes();

    return !((0U == b[6]) && (0U == b[7]) && (0U == b[8])
             && (0U == b[9]) && (0U == b[10]) && (0U == b[11]));
}

/*! \brief true while the block comes from the simulator rather than a chip. */
int imu_is_simulated(void)
{
#if IMU_USE_SPI
    return 0;
#else
    return 1;
#endif
}

void imu_init(void)
{
#if IMU_USE_SPI
    imu_spi_init();
#else
    imu_sim_init();
#endif
}

void imu_tick(uint32_t now_ms)
{
    static uint32_t next_ms;
    uint8_t steps = 0U;

    if (0U == next_ms) {
        next_ms = now_ms + IMU_SAMPLE_PERIOD_MS;
    }

    while ((int32_t)(now_ms - next_ms) >= 0) {
#if IMU_USE_SPI
        imu_spi_step();
#else
        imu_sim_step_10ms();
#endif
        next_ms += IMU_SAMPLE_PERIOD_MS;
        steps++;

        /* A new quaternion arrived.  The tick time recorded is the *sample's*
           own grid time, not now_ms: after a stall the backlog must look old,
           not fresh, or the age the FT6 status byte publishes would be a lie. */
        if (quat_produced()) {
            s_quat_seq++;
            s_quat_ms = next_ms - IMU_SAMPLE_PERIOD_MS;
        }

        if (steps >= 10U) {
            /* stalled longer than 100 ms (breakpoint, erase, ...): drop the
               backlog instead of taking a burst of steps */
            next_ms = now_ms + IMU_SAMPLE_PERIOD_MS;
            break;
        }
    }
}

void imu_restart(void)
{
#if IMU_USE_SPI
    imu_spi_restart();
    /* the FIFO is dropped, so the word counter restarts with it: comparing
       against a stale high-water mark would swallow the first new attitude */
    s_sflp_words = imu_spi_sflp_words();
#else
    imu_sim_restart();
#endif
    /* a restarted fusion has no attitude until it converges again */
    s_quat_seq = 0U;
    s_quat_ms = 0U;
}

const uint8_t *imu_block_bytes(void)
{
#if IMU_USE_SPI
    return imu_spi_block();
#else
    return imu_sim_block();
#endif
}

const imu_sample_t *imu_latest(void)
{
#if IMU_USE_SPI
    return imu_spi_sample();
#else
    return imu_sim_sample();
#endif
}

uint8_t imu_flags(void)
{
    return imu_latest()->flags;
}

uint32_t imu_samples(void)
{
#if IMU_USE_SPI
    return imu_spi_sample_count();
#else
    return imu_sim_sample_count();
#endif
}

/* ── the FT6 contract's sequence, age and readiness ─────────────────────── */

uint32_t imu_quat_seq(void)
{
    return s_quat_seq;
}

uint32_t imu_quat_ms(void)
{
    return s_quat_ms;
}

uint8_t imu_quat_ready(void)
{
    /* the backend's own liveness first (a latched-but-dead SPI link fails this),
       then the all-zero test the FT6 node applies to `ready` */
    if (0U == imu_latest()->quat_valid) {
        return 0U;
    }
    return (uint8_t)(quat_bytes_live() ? 1U : 0U);
}

uint16_t imu_quat_age_ms(void)
{
    uint32_t age;

    if (0U == imu_quat_ready()) {
        return 65535U;
    }
    /* unsigned arithmetic wraps correctly across the 49-day ms rollover */
    age = systick_get_ms() - s_quat_ms;
    return (age > 65535U) ? 65535U : (uint16_t)age;
}

#if FEE_IMU_PERSONALITY_FT6
/* ── the FeeTech-side extra chip-frame rotation ─────────────────────────── */

static uint16_t get16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static void put16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFU);
    p[1] = (uint8_t)(v >> 8);
}

void imu_fee_control_block(uint8_t out[TELEM_LEN])
{
    const uint8_t *in = imu_block_bytes();
    quat_t c;
    quat_t q;
    float g[3];
    float gr[3];
    float x, y, z;
    int i;

    /* the tail (accelerometer, counter, flags) is never rotated */
    for (i = TELEM_CTRL_LEN; i < TELEM_LEN; i++) {
        out[i] = in[i];
    }

    c.w = FEE_IMU_PREMOUNT_QW;
    c.x = FEE_IMU_PREMOUNT_QX;
    c.y = FEE_IMU_PREMOUNT_QY;
    c.z = FEE_IMU_PREMOUNT_QZ;
    c = quat_normalize(c);

    /* gyro: i16 little endian, sensor frame.  The host rotates it by its own
       mount, so pre-rotate here by C: q_rotate(C, g) = C g C^-1. */
    for (i = 0; i < 3; i++) {
        g[i] = (float)(int16_t)get16(&in[2 * i]);
    }
    quat_rotate(c, g, gr);
    for (i = 0; i < 3; i++) {
        put16(&out[2 * i], (uint16_t)(int16_t)lroundf_i32(clampf(gr[i], -32768.0f, 32767.0f)));
    }

    /* quaternion: x/y/z only, IEEE binary16.  The receiver rebuilds w >= 0, so
       an all-zero triple means "no attitude" and must stay that way. */
    x = half_to_f32(get16(&in[6]));
    y = half_to_f32(get16(&in[8]));
    z = half_to_f32(get16(&in[10]));
    if ((0.0f == x) && (0.0f == y) && (0.0f == z)) {
        put16(&out[6], 0U);
        put16(&out[8], 0U);
        put16(&out[10], 0U);
        return;
    }

    {
        float w2 = 1.0f - (x * x + y * y + z * z);
        q.w = (w2 > 0.0f) ? sqrtf(w2) : 1.0f;
        q.x = x;
        q.y = y;
        q.z = z;
    }
    q = quat_normalize(quat_mul(q, c));
    /* Only x/y/z go on the wire and the receiver always rebuilds w >= 0.  If the
       product came out with w < 0, publish -q, which is the same rotation. */
    if (q.w < 0.0f) {
        q.w = -q.w; q.x = -q.x; q.y = -q.y; q.z = -q.z;
    }
    put16(&out[6], f32_to_half(q.x));
    put16(&out[8], f32_to_half(q.y));
    put16(&out[10], f32_to_half(q.z));
}
#endif
