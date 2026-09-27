/*!
    \file    boot_regs.h
    \brief   the bootloader's register space

    The bootloader deliberately does not link `dev.c`: there is no IMU to
    publish, no simulation model, no 256-byte control table to maintain.  What
    it does provide is the *same C API* (`dev_cfg`, `dev_image`, `dev_read`,
    `dev_write`, `dev_request_reboot`, ...), so `dxl2.c`, `fee.c` and `bus.c`
    are compiled once and behave identically in both images - the host sees the
    same framing, the same ID, the same auto-detection.

    Visible registers in boot mode:

      * identity (both maps): model number, firmware version byte, ID, baud,
        status-return/ACK level - enough for a normal `PING`/`READ` to work;
      * `UID_WIN_ADDR` 180..191: read-only mode/slot/version/CRC of what the
        bootloader is about to run (see board.h), which is how the host tool
        tells "in the bootloader" from "running the application";
      * `UPGRADE_WIN_ADDR` 208..255: the upgrade session (boot_upgrade.c).

    Anything else is out of range: a bootloader has no business pretending to be
    a servo, and refusing is cheaper than explaining.
*/

#ifndef BOOT_REGS_H
#define BOOT_REGS_H

#include <stdint.h>

#include "board.h"
#include "boot_decide.h"
#include "boot_upgrade.h"
#include "cfg_blob.h"

/*! \brief load the configuration page, default what is missing, build the
           register images. */
void boot_regs_init(void);

/*! \brief publish the boot decision in the identity window. */
void boot_regs_set_decision(const boot_decide_in_t *in, const boot_decide_out_t *out);

/*! \brief publish one slot's header fields in the identity window. */
void boot_regs_set_slot_info(uint8_t slot, uint16_t version, uint16_t flags,
                             uint32_t imgcrc);

/*! \brief the production upgrade callbacks (configuration page updates), so a
           test or another integrator can chain them instead of replacing them. */
const upg_hooks_t *boot_regs_hooks(void);

/*! \brief the blob the bootloader owns (device config + boot state). */
cfg_blob_t *boot_regs_blob(void);

/*! \brief persist the current blob (used for boot_stay / trial updates). */
int boot_regs_save(void);

/*! \brief 1 once a reset was requested (UPG_REBOOT / INST_REBOOT) and the
           response has had time to leave. */
int boot_regs_reboot_pending(uint32_t now_ms);

/* ── the API dxl2.c / fee.c / bus.c expect (see dev.h for the contracts) ── */

const dev_cfg_t *dev_cfg(void);
const uint8_t *dev_image(uint8_t proto);
uint8_t dev_read(uint8_t proto, uint16_t addr, uint16_t len, uint8_t *out);
uint8_t dev_write(uint8_t proto, uint16_t addr, uint16_t len, const uint8_t *in);
void dev_refresh(void);
void dev_factory_reset(void);
void dev_request_reboot(uint32_t now_ms);
void dev_clear_hw_error(void);
uint32_t dev_dxl_baud(void);
uint32_t dev_fee_baud(void);
uint32_t dev_mirror_baud(void);
uint32_t dev_cfg_seq(void);

#endif /* BOOT_REGS_H */
