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

#endif /* IMU_H */
