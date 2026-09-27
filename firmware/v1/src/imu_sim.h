/*!
    \file    imu_sim.h
    \brief   simulated LSM6DSV16X + SFLP block, used until the real PCBA exists

    The simulator is not a random number generator: it integrates the quaternion
    from the same angular velocity it reports as gyro, so the two halves of the
    block stay mutually consistent - which is exactly what makes it useful for
    validating the host tool, the 3D view and the waterfall plots.
*/

#ifndef IMU_SIM_H
#define IMU_SIM_H

#include <stdint.h>

#include "imu.h"

/* Motion models, also the value of the vendor SIM_MODE register. */
#define SIM_STATIC      0   /* resting, LSB-level noise                              */
#define SIM_SINE        1   /* roll/pitch/yaw sinusoids (default bench stimulus)      */
#define SIM_DRIFT       2   /* slow yaw + injected gyro bias: shows offset/drift      */
#define SIM_NOISE       3   /* static with wide-band gyro noise: shows jitter         */
#define SIM_SPIN        4   /* fast constant yaw                                      */
#define SIM_SFLP_WAIT   5   /* gyro live but quaternion bytes all zero (SFLP not up)  */
#define SIM_FROZEN      6   /* identical block every read (staleness / frozen test)   */
#define SIM_MODE_MAX    SIM_FROZEN

void imu_sim_init(void);
void imu_sim_restart(void);
void imu_sim_step_10ms(void);
const uint8_t *imu_sim_block(void);
const imu_sample_t *imu_sim_sample(void);
uint32_t imu_sim_sample_count(void);

#endif /* IMU_SIM_H */
