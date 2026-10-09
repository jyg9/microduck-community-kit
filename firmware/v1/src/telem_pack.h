/*!
    \file    telem_pack.h
    \brief   the wire forms of the sensor block, per protocol map

    The lump of bytes the runtime reads every tick is the *first 12* of the
    20-byte block, in both maps.  What follows is where the two maps differ, and
    it is not cosmetic:

      * FeeTech 56..70 is a real servo's block - position, speed, load, voltage,
        temperature, status, moving, current - and it is 15 bytes long.  The
        runtime reads exactly that span from the IMU node and every servo in one
        `sync_read`, so the node has to put its sample counter and status bits
        *inside* those 15 bytes (bytes 12 and 13) and leave byte 14 at zero, the
        way a real servo's block is shaped.  The raw accelerometer does not fit
        and lives at the 128 alias instead (measured: the servo answers a
        20-byte read at 56 too, but only 56..70 is its documented block, and a
        20-byte shared read costs ~5 bytes x 16 devices = ~0.8 ms of bus time).

        Which counter and which status bits go in there depends on the FeeTech
        *personality* (docs/imu_to_dxl_protocol.md §16).  This repository's own
        contract puts the sample counter and its TELEM_FLAG_* byte there; the
        FT6 contract puts the quaternion sequence and its own status byte there,
        and moves the native 20-byte block from 128 to 196 to make room for a
        124..143 diagnostic window.  Both packers are compiled; only the call
        site in src/dev.c is switched.

      * Dynamixel 124 is `present_pwm` onwards, where the XL330 control table has
        room up to 143, so the full 20-byte block (accelerometer, counter, flags
        at 12/18/19) stays as it is - in both personalities.

    Keeping the packing here - pure, no register image, no hardware - is what
    lets the host tests assert the FeeTech layout byte by byte.
*/

#ifndef TELEM_PACK_H
#define TELEM_PACK_H

#include <stdint.h>

#include "board.h"

/*! \brief FeeTech 56..70: 12 control bytes, counter, status, reserved 0. */
void telem_pack_fee15(const uint8_t *telem, uint8_t *out);

/* ── the FT6 contract's two wire forms (docs/imu_to_dxl_protocol.md §16) ──
   Built in every personality, not only the FT6 one: they are pure functions of
   their arguments, and keeping them compiled lets one host test binary assert
   both layouts byte by byte.  What changes with the personality is which packer
   src/dev.c calls, and nothing else. */

/*! \brief FT6 contract block at 56..70: 12 control bytes, then the quaternion
           sequence (u8) and the FT6 status byte, then a reserved zero.  The
           native counter/flags bytes are deliberately NOT used here. */
void telem_pack_fee15_ft6(const uint8_t *telem, uint8_t *out,
                          uint8_t quat_seq8, uint8_t status);

/*! \brief FT5/FT6 diagnostic block, 20 bytes: 12 control bytes, quaternion
           sequence (u32 LE), age (u16 LE, saturated), ready, schema = 1. */
void telem_pack_fee_diag20(const uint8_t *telem, uint8_t *out,
                           uint32_t quat_seq, uint16_t age_ms, uint8_t ready);

/*! \brief the FT6 contract status byte.  Mirrors the FT6 node's
           ft_imu_compact_status(): bit0 not-ready (the opposite polarity of
           this repository's own TELEM_FLAG_SFLP_VALID), bit1 stale, bit2 error,
           bit3 reader-slow, bits 4..7 always zero. */
uint8_t telem_fee_ft6_status(uint8_t ready, uint32_t age_ms,
                             uint32_t new_samples, uint8_t sensor_err);

#endif /* TELEM_PACK_H */
