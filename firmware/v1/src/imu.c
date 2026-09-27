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

#include "systick.h"

#if IMU_USE_SPI
#include "imu_spi.h"
#else
#include "imu_sim.h"
#endif

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
#else
    imu_sim_restart();
#endif
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
