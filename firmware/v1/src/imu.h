/*!
    \file    imu.h
    \brief   the board's sensor block, independent of where the samples come from

    v1 fills this from src/imu_sim.c because the development board has no SPI
    IMU.  A real driver (LSM6DSV16X over SPI0, later LSM6DS3TR-C + on-MCU
    fusion) only has to produce the same 20-byte block.

    Wire layout (also the register image at DXL address 124 / FeeTech 56):

      | bytes | contents                                                   |
      |-------|------------------------------------------------------------|
      | 0..6  | gyro x/y/z, i16 little endian, ±500 dps @ 17.5 mdps/LSB    |
      | 6..12 | quaternion x/y/z, IEEE binary16, w = √(1 - x² - y² - z²)   |
      | 12..18| raw accelerometer x/y/z, i16 LE, ±4 g @ 0.122 mg/LSB       |
      | 18    | sample counter, u8, wraps                                  |
      | 19    | status flags (TELEM_FLAG_*)                                |

    The first 12 bytes are the block microduck consumes every tick; the extra
    8 are the board's diagnostic tail (raw accelerometer, counter, flags).

    Byte 18 is the *native* sample counter and byte 19 the native flags; in the
    FT6 personality the contract block at 56..70 does not publish them - it
    carries the quaternion sequence and the FT6 status byte instead (see
    docs/imu_to_dxl_protocol.md §16).  They stay in the 20-byte block either
    way, which is what the 124/20 diagnostic block and the 196 alias expose.
*/

#ifndef IMU_H
#define IMU_H

#include <stdint.h>

#include "board.h"

typedef struct {
    int16_t  gyro[3];      /* chip frame, raw counts */
    uint16_t quat[3];      /* chip frame, x/y/z binary16 */
    uint8_t  quat_valid;
    int16_t  accel[3];     /* chip frame, raw counts */
    uint8_t  counter;
    uint8_t  flags;
} imu_sample_t;

/*! \brief one-off init: register image defaults, ADC for the on-chip
           temperature sensor, simulation state. */
void imu_init(void);

/*! \brief advance the sensor by wall-clock time.  Safe to call every main-loop
           iteration: the sample itself is produced on a fixed 10 ms grid. */
void imu_tick(uint32_t now_ms);

/*! \brief restart the motion model / fusion (vendor command, or after a
           configuration change that invalidates the current attitude). */
void imu_restart(void);

/*! \brief the current 20-byte block, wire order. */
const uint8_t *imu_block_bytes(void);

/*! \brief the latest decoded sample (for the debug console). */
const imu_sample_t *imu_latest(void);

uint8_t  imu_flags(void);
uint32_t imu_samples(void);
int      imu_is_simulated(void);

/* ── quaternion sequence: the FT6 contract's counter ──────────────────────
   The FT6 node counts *new quaternion words*, not samples: its counter only
   advances when the FIFO yields a fresh SFLP attitude.  The host reads that
   byte as "the IMU is still producing data" and refuses the block after three
   identical reads, so a counter that advances while the fusion has stalled is
   worse than no counter at all - it retires the one signal that would have
   caught it.  The facade owns the counting because both backends already know
   when they produced a new attitude (see src/imu.c). */

/*! \brief quaternion sequence: increments only when the backend produced a new
           quaternion, and never when the attitude source has stopped. */
uint32_t imu_quat_seq(void);

/*! \brief millisecond tick of the sample that produced the current quaternion
           sequence value, or 0 before the first one. */
uint32_t imu_quat_ms(void);

/*! \brief 1 when the current block carries a usable quaternion: the backend
           says so and the six quaternion bytes are not all zero. */
uint8_t  imu_quat_ready(void);

/*! \brief milliseconds since the sample that produced the current quaternion
           sequence value, saturated to 65535; 65535 when not ready.  O(1), and
           safe to call from the USART interrupt. */
uint16_t imu_quat_age_ms(void);

#if FEE_IMU_PERSONALITY_FT6
/*! \brief the 20-byte block as the *FeeTech* side should see it: the current
           block with the personality's extra chip-frame rotation applied to the
           twelve control bytes (see FEE_IMU_PREMOUNT_* in board.h), and the
           diagnostic tail (accelerometer, counter, flags) copied unchanged.

           Applied above the backends on purpose: the Dynamixel map and the 196
           native alias must keep the raw chip frame, so the rotation belongs to
           the FT6 FeeTech projection and not to the sensor sources.  An
           all-zero quaternion stays all zero - "no attitude yet" must not be
           turned into a fabricated one by rotating it. */
void imu_fee_control_block(uint8_t out[TELEM_LEN]);
#endif

#endif /* IMU_H */
