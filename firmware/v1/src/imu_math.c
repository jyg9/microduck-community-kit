/*!
    \file    imu_math.c
    \brief   quaternion and binary16 helpers - see imu_math.h
*/

#include "imu_math.h"

#include <math.h>

quat_t quat_mul(quat_t a, quat_t b)
{
    quat_t r;
    r.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
    r.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
    r.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
    r.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;
    return r;
}

quat_t quat_conj(quat_t q)
{
    quat_t r;
    r.w = q.w;
    r.x = -q.x;
    r.y = -q.y;
    r.z = -q.z;
    return r;
}

float quat_norm(quat_t q)
{
    return sqrtf(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
}

quat_t quat_normalize(quat_t q)
{
    float n = quat_norm(q);
    if (n > 0.0f) {
        q.w /= n;
        q.x /= n;
        q.y /= n;
        q.z /= n;
    } else {
        q = quat_identity();
    }
    return q;
}

/* v' = v + 2 w (qv × v) + 2 qv × (qv × v)   -- the same expansion microduck's
   `rotate()` uses, so both sides round identically. */
void quat_rotate(quat_t q, const float v[3], float out[3])
{
    float t[3], c[3];
    t[0] = 2.0f * (q.y * v[2] - q.z * v[1]);
    t[1] = 2.0f * (q.z * v[0] - q.x * v[2]);
    t[2] = 2.0f * (q.x * v[1] - q.y * v[0]);
    c[0] = q.y * t[2] - q.z * t[1];
    c[1] = q.z * t[0] - q.x * t[2];
    c[2] = q.x * t[1] - q.y * t[0];
    out[0] = v[0] + q.w * t[0] + c[0];
    out[1] = v[1] + q.w * t[1] + c[1];
    out[2] = v[2] + q.w * t[2] + c[2];
}

void quat_rotate_inv(quat_t q, const float v[3], float out[3])
{
    float t[3], c[3];
    t[0] = 2.0f * (q.y * v[2] - q.z * v[1]);
    t[1] = 2.0f * (q.z * v[0] - q.x * v[2]);
    t[2] = 2.0f * (q.x * v[1] - q.y * v[0]);
    c[0] = q.y * t[2] - q.z * t[1];
    c[1] = q.z * t[0] - q.x * t[2];
    c[2] = q.x * t[1] - q.y * t[0];
    out[0] = v[0] - q.w * t[0] + c[0];
    out[1] = v[1] - q.w * t[1] + c[1];
    out[2] = v[2] - q.w * t[2] + c[2];
}

void quat_integrate(quat_t *q, const float omega[3], float dt)
{
    quat_t dq;
    quat_t w;

    w.w = 0.0f;
    w.x = omega[0];
    w.y = omega[1];
    w.z = omega[2];

    dq = quat_mul(*q, w);
    q->w += 0.5f * dq.w * dt;
    q->x += 0.5f * dq.x * dt;
    q->y += 0.5f * dq.y * dt;
    q->z += 0.5f * dq.z * dt;
    *q = quat_normalize(*q);
}

uint16_t f32_to_half(float v)
{
    union { float f; uint32_t u; } bits;
    uint32_t sign, mant, half;
    int32_t  exp, shift, rem, halfway;

    bits.f = v;
    sign = (bits.u >> 16) & 0x8000u;
    exp  = (int32_t)((bits.u >> 23) & 0xFFu) - 127 + 15;
    mant = bits.u & 0x7FFFFFu;

    if (exp >= 0x1F) {
        /* overflow / inf / nan -> inf */
        return (uint16_t)(sign | 0x7C00u);
    }
    if (exp <= 0) {
        if (exp < -10) {
            return (uint16_t)sign;           /* underflows to zero */
        }
        mant |= 0x800000u;
        shift = 14 - exp;
        half = mant >> shift;
        rem = (int32_t)(mant & ((1u << shift) - 1u));
        halfway = 1 << (shift - 1);
        if (rem > halfway || (rem == halfway && (half & 1u))) {
            half++;
        }
        return (uint16_t)(sign | half);
    }

    half = sign | ((uint32_t)exp << 10) | (mant >> 13);
    rem = (int32_t)(mant & 0x1FFFu);
    if (rem > 0x1000 || (rem == 0x1000 && (half & 1u))) {
        half++;                               /* may carry into the exponent */
    }
    return (uint16_t)half;
}

float half_to_f32(uint16_t h)
{
    uint32_t sign = (h & 0x8000u) ? 0x80000000u : 0u;
    uint32_t exp  = (h >> 10) & 0x1Fu;
    uint32_t frac = h & 0x3FFu;
    union { float f; uint32_t u; } bits;

    if (exp == 0) {
        if (frac == 0) {
            bits.u = sign;
            return bits.f;
        }
        /* subnormal */
        exp = 1;
        while ((frac & 0x400u) == 0u) {
            frac <<= 1;
            exp--;
        }
        frac &= 0x3FFu;
        bits.u = sign | ((exp + (127 - 15)) << 23) | (frac << 13);
        return bits.f;
    }
    if (exp == 0x1Fu) {
        bits.u = sign | 0x7F800000u | (frac << 13);
        return bits.f;
    }
    bits.u = sign | ((exp + (127 - 15)) << 23) | (frac << 13);
    return bits.f;
}

float clampf(float v, float lo, float hi)
{
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return v;
}

int32_t lroundf_i32(float v)
{
    return (int32_t)((v >= 0.0f) ? (v + 0.5f) : (v - 0.5f));
}
