/*!
    \file    imu_spi.h
    \brief   real LSM6DSV16X sensor block over SPI0 (see src/imu_spi.c)

    Same contract as src/imu_sim.c: one call per 10 ms sample produces the
    20-byte block in chip frame.  Selected by the IMU_USE_SPI build option
    (board.h); `imu.c` is the facade that picks this or the simulator.
*/

#ifndef IMU_SPI_H
#define IMU_SPI_H

#include <stdint.h>

#include "imu.h"

/*! \brief bring SPI0 up, reset and configure the chip, and take a first sample.
    Safe to call again after a failure: it re-probes. */
void imu_spi_init(void);

/*! \brief one 10 ms step: drain the SFLP FIFO, read gyro/accel, republish. */
void imu_spi_step(void);

/*! \brief re-initialise the SFLP algorithm (vendor RESTART command) and hold
    the quaternion at zero until it converges again. */
void imu_spi_restart(void);

const uint8_t *imu_spi_block(void);
const imu_sample_t *imu_spi_sample(void);
uint32_t imu_spi_sample_count(void);

/* ── observability, for the debug console ───────────────────────────────── */

/*! \brief 1 when a WHO_AM_I probe last matched. */
uint8_t imu_spi_present(void);
/*! \brief last WHO_AM_I byte read (0 when the bus never answered). */
uint8_t imu_spi_whoami(void);
/*! \brief total SFLP quaternion words consumed. */
uint32_t imu_spi_sflp_words(void);
/*! \brief FIFO words dropped because the poller fell behind (overrun). */
uint32_t imu_spi_fifo_overruns(void);
/*! \brief FIFO words discarded for carrying a tag the driver does not expect. */
uint32_t imu_spi_bad_tags(void);
/*! \brief retries of a failed init/probe. */
uint32_t imu_spi_probe_fails(void);

#endif /* IMU_SPI_H */
