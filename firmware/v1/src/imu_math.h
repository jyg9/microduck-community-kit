/*!
    \file    imu_math.h
    \brief   small float/quaternion toolkit used by the IMU simulation and by
             the host-side-compatible frame conversions

    Conventions match microduck's `duck-control/src/imu.rs` exactly, because the
    whole point of this firmware is to be indistinguishable from the original
    board on the wire and on the host:
      * quaternions are scalar-first `[w, x, y, z]`,
      * `q_rotate(q, v)`   = q v q⁻¹   (body -> world for a trunk->world quat),
      * `q_rotate_inv(q,v)` = q⁻¹ v q  (world -> body),
      * quaternion components are shipped as IEEE binary16, x/y/z only.
*/

#ifndef IMU_MATH_H
#define IMU_MATH_H

#include <stdint.h>

typedef struct { float w, x, y, z; } quat_t;

/* ── quaternion algebra ─────────────────────────────────────────────────── */

static inline quat_t quat_identity(void)
{
    quat_t q = { 1.0f, 0.0f, 0.0f, 0.0f };
    return q;
}

quat_t quat_mul(quat_t a, quat_t b);
quat_t quat_conj(quat_t q);
quat_t quat_normalize(quat_t q);
float  quat_norm(quat_t q);

/*! rotate a vector by q: v' = q v q⁻¹ */
void quat_rotate(quat_t q, const float v[3], float out[3]);
/*! rotate a vector by q⁻¹: v' = q⁻¹ v q */
void quat_rotate_inv(quat_t q, const float v[3], float out[3]);
/*! q <- q + 0.5 * q * [0, omega] * dt, renormalised.  omega is in rad/s. */
void quat_integrate(quat_t *q, const float omega[3], float dt);

/* ── IEEE 754 binary16 ──────────────────────────────────────────────────── */

/*! float -> half, round to nearest even.  The LSM6DSV16X ships the SFLP
    quaternion in this format, so the simulator has to produce it the same way
    the chip does or the host would see different rounding than on hardware. */
uint16_t f32_to_half(float v);
/*! half -> float; used by the host-side test script and by self tests. */
float half_to_f32(uint16_t h);

/* ── misc ───────────────────────────────────────────────────────────────── */

float clampf(float v, float lo, float hi);
int32_t lroundf_i32(float v);

#endif /* IMU_MATH_H */
