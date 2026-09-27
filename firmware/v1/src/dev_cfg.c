/*!
    \file    dev_cfg.c
    \brief   the factory defaults for the device configuration

    Split out of `dev.c` so that the bootloader - which does not link the
    application's IMU/simulation layer - can still come up with the same node ID
    and baud rate when the configuration page is empty.
*/

#include "dev.h"

#include <string.h>

#include "imu_sim.h"

void dev_cfg_set_defaults(dev_cfg_t *cfg)
{
    memset(cfg, 0, sizeof(*cfg));
    cfg->dxl_id = DXL_ID_DEFAULT;
    cfg->dxl_baud_code = DXL_BAUD_CODE_1M;
    cfg->fee_id = FEE_ID_DEFAULT;
    cfg->fee_baud_code = FEE_BAUD_CODE_1M;
    /* SINE out of the box: the bench link has something to show in the 3D view
       and in the waterfall without any host-side setup */
    cfg->sim_mode = SIM_SINE;
    cfg->sim_amp = 1000U;
    cfg->sim_freq = 1000U;
    cfg->report_frame = 0U;
    cfg->proto_lock = PROTO_NONE;
    cfg->mirror_baud_code = FEE_BAUD_CODE_1M;
    cfg->dbg_level = 2U;
}

