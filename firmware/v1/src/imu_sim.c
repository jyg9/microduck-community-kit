/*!
    \file    imu_sim.c
    \brief   the stand-in LSM6DSV16X: integrates a quaternion and encodes the
             same block the real board does

    Everything here is in the *chip* frame, like the real board: the mounting
    rotation is baked into the reported quaternion and the reported gyro is
    un-rotated, so the host's `mount` handling is exercised exactly as it will
    be on hardware.
*/

#include "imu_sim.h"

#include <math.h>
#include <string.h>

#include "board.h"
#include "dev.h"
#include "imu.h"
#include "imu_math.h"

/* 1 rad/s in gyro counts: 57295.78 mdps / 17.5 mdps per LSB = 3274.04 */
#define GYRO_COUNTS_PER_RAD_S   (57295.7795f / IMU_GYRO_MDPS_PER_LSB)
#define ACCEL_COUNTS_PER_MG     (1.0f / IMU_ACCEL_MG_PER_LSB)
#define DEG2RAD                 0.017453293f

#define SIM_STEP_S              0.01f

/* ±500 dps full scale -> ±28571 counts, so the clamp is only a guard */
#define GYRO_COUNT_MAX          32767.0f
#define GYRO_COUNT_MIN         (-32768.0f)

static quat_t        s_q;          /* trunk -> world */
static quat_t        s_mount;      /* chip  -> trunk (board.h MOUNT_*) */
static float         s_t;          /* simulated seconds */
static uint32_t      s_rng;
static uint32_t      s_samples;
static uint8_t       s_counter;
static uint8_t       s_block[TELEM_LEN];
static imu_sample_t  s_sample;

static void sim_publish(void)
{
    s_sample.gyro[0]  = (int16_t)((uint16_t)s_block[0] | ((uint16_t)s_block[1] << 8));
    s_sample.gyro[1]  = (int16_t)((uint16_t)s_block[2] | ((uint16_t)s_block[3] << 8));
    s_sample.gyro[2]  = (int16_t)((uint16_t)s_block[4] | ((uint16_t)s_block[5] << 8));
    s_sample.quat[0]  = (uint16_t)((uint16_t)s_block[6]  | ((uint16_t)s_block[7] << 8));
    s_sample.quat[1]  = (uint16_t)((uint16_t)s_block[8]  | ((uint16_t)s_block[9] << 8));
    s_sample.quat[2]  = (uint16_t)((uint16_t)s_block[10] | ((uint16_t)s_block[11] << 8));
    s_sample.accel[0] = (int16_t)((uint16_t)s_block[12] | ((uint16_t)s_block[13] << 8));
    s_sample.accel[1] = (int16_t)((uint16_t)s_block[14] | ((uint16_t)s_block[15] << 8));
    s_sample.accel[2] = (int16_t)((uint16_t)s_block[16] | ((uint16_t)s_block[17] << 8));
    s_sample.counter  = s_block[18];
    s_sample.flags    = s_block[19];
    s_sample.quat_valid = (uint8_t)((s_sample.flags & TELEM_FLAG_SFLP_VALID) != 0U);
}

static void put16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFU);
    p[1] = (uint8_t)(v >> 8);
}

/* xorshift32: deterministic, so a bench session is reproducible */
static float rnd01(void)
{
    s_rng ^= s_rng << 13;
    s_rng ^= s_rng >> 17;
    s_rng ^= s_rng << 5;
    return (float)(s_rng & 0xFFFFFFU) / 16777216.0f;
}

static float noise(float amplitude_counts)
{
    if (amplitude_counts <= 0.0f) {
        return 0.0f;
    }
    return (rnd01() * 2.0f - 1.0f) * amplitude_counts;
}

static void fill_frozen(void)
{
    /* Identical bytes on every read: a board that answers but never refreshes.
       microduck's stale-block tracker has to notice this, so it is a mode
       rather than an accident. Counter deliberately does not advance. */
    memset(s_block, 0, TELEM_LEN);
    put16(&s_block[12], 0);
    put16(&s_block[14], 0);
    put16(&s_block[16], (uint16_t)lroundf_i32(1000.0f * ACCEL_COUNTS_PER_MG));
    s_block[18] = 0U;
    s_block[19] = (uint8_t)(TELEM_FLAG_SIMULATED | TELEM_FLAG_FROZEN);
    sim_publish();
}

void imu_sim_step_10ms(void)
{
    const dev_cfg_t *cfg = dev_cfg();
    float amp = (float)cfg->sim_amp / 1000.0f;
    float frq = (float)cfg->sim_freq / 1000.0f;
    float omega[3] = { 0.0f, 0.0f, 0.0f };
    float gyro_noise = 0.7f;         /* counts rms */
    float accel_noise = 2.0f;
    uint8_t flags = TELEM_FLAG_SIMULATED | TELEM_FLAG_FUSION_OK;
    int quat_valid = 1;
    float w_raw[3], f_trunk[3], f_raw[3];
    quat_t q_raw;
    int16_t gyro[3], accel[3];
    float down[3] = { 0.0f, 0.0f, -1.0f };
    float g_trunk[3];
    int i;

    if ((float)amp <= 0.0f) {
        amp = 1.0f;
    }
    if ((float)frq <= 0.0f) {
        frq = 1.0f;
    }

    switch (cfg->sim_mode) {
    case SIM_SINE: {
        const float two_pi = 6.2831853f;
        float ar = 20.0f * DEG2RAD * amp;   /* roll  */
        float ap = 12.0f * DEG2RAD * amp;   /* pitch */
        float ay = 30.0f * DEG2RAD * amp;   /* yaw   */
        float fr = 0.50f * frq;
        float fp = 0.33f * frq;
        float fy = 0.20f * frq;
        /* the derivative of the analytic angles, so gyro and quaternion agree */
        omega[0] = ar * two_pi * fr * cosf(two_pi * fr * s_t);
        omega[1] = ap * two_pi * fp * cosf(two_pi * fp * s_t + 0.7f);
        omega[2] = ay * two_pi * fy * cosf(two_pi * fy * s_t + 1.3f);
        gyro_noise = 1.0f;
        break;
    }
    case SIM_DRIFT:
        /* slow yaw plus the register-injected gyro bias: the waterfall shows
           the offset as a constant floor and the quaternion drifts away */
        omega[2] = 0.30f * amp;
        gyro_noise = 1.0f;
        break;
    case SIM_NOISE:
        gyro_noise = 12.0f;   /* ~210 mdps rms */
        accel_noise = 8.0f;
        break;
    case SIM_SPIN:
        omega[2] = 3.0f * amp;
        gyro_noise = 1.0f;
        break;
    case SIM_SFLP_WAIT:
        /* gyro alive, SFLP table still empty: all-zero quaternion bytes */
        gyro_noise = 1.0f;
        quat_valid = 0;
        flags &= (uint8_t)~TELEM_FLAG_FUSION_OK;
        break;
    case SIM_FROZEN:
        fill_frozen();
        s_samples++;
        return;
    case SIM_STATIC:
    default:
        break;
    }

    s_t += SIM_STEP_S;
    quat_integrate(&s_q, omega, SIM_STEP_S);

    /* raw gyro = mount⁻¹ · ω_trunk, plus the injected bias and noise.  In
       trunk-report mode the rotation is skipped so the gyro stays in the same
       frame as the quaternion. */
    if (cfg->report_frame != 0U) {
        w_raw[0] = omega[0];
        w_raw[1] = omega[1];
        w_raw[2] = omega[2];
    } else {
        quat_rotate_inv(s_mount, omega, w_raw);
    }
    for (i = 0; i < 3; i++) {
        float c = w_raw[i] * GYRO_COUNTS_PER_RAD_S
                + (float)cfg->gyro_bias[i]
                + noise(gyro_noise);
        c = clampf(c, GYRO_COUNT_MIN, GYRO_COUNT_MAX);
        gyro[i] = (int16_t)lroundf_i32(c);
    }

    if (cfg->report_frame != 0U) {
        q_raw = s_q;                       /* bench convenience: trunk frame */
    } else {
        q_raw = quat_mul(s_q, s_mount);    /* what the chip would report */
    }

    /* specific force in mg: the accelerometer measures -g (i.e. +1 g up at rest) */
    quat_rotate_inv(s_q, down, g_trunk);
    for (i = 0; i < 3; i++) {
        f_trunk[i] = -g_trunk[i] * 1000.0f;
    }
    if (cfg->report_frame != 0U) {
        memcpy(f_raw, f_trunk, sizeof(f_raw));
    } else {
        quat_rotate_inv(s_mount, f_trunk, f_raw);
    }
    for (i = 0; i < 3; i++) {
        float c = f_raw[i] * ACCEL_COUNTS_PER_MG + noise(accel_noise);
        c = clampf(c, GYRO_COUNT_MIN, GYRO_COUNT_MAX);
        accel[i] = (int16_t)lroundf_i32(c);
    }

    put16(&s_block[0], (uint16_t)gyro[0]);
    put16(&s_block[2], (uint16_t)gyro[1]);
    put16(&s_block[4], (uint16_t)gyro[2]);
    if (quat_valid) {
        put16(&s_block[6],  f32_to_half(q_raw.x));
        put16(&s_block[8],  f32_to_half(q_raw.y));
        put16(&s_block[10], f32_to_half(q_raw.z));
        flags |= TELEM_FLAG_SFLP_VALID;
    } else {
        memset(&s_block[6], 0, 6);
    }
    put16(&s_block[12], (uint16_t)accel[0]);
    put16(&s_block[14], (uint16_t)accel[1]);
    put16(&s_block[16], (uint16_t)accel[2]);

    if (s_counter == 0xFFU) {
        flags |= TELEM_FLAG_CNT_WRAP;
    }
    s_block[18] = s_counter++;
    s_block[19] = flags;
    s_samples++;
    sim_publish();
}

void imu_sim_restart(void)
{
    s_q = quat_identity();
    s_t = 0.0f;
    s_counter = 0U;
    imu_sim_step_10ms();
}

void imu_sim_init(void)
{
    s_mount.w = MOUNT_QW;
    s_mount.x = MOUNT_QX;
    s_mount.y = MOUNT_QY;
    s_mount.z = MOUNT_QZ;
    s_rng = 0x1234567U;
    s_samples = 0U;
    imu_sim_restart();
}

const uint8_t *imu_sim_block(void)
{
    return s_block;
}

const imu_sample_t *imu_sim_sample(void)
{
    return &s_sample;
}

uint32_t imu_sim_sample_count(void)
{
    return s_samples;
}
