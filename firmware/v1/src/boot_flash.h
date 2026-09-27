/*!
    \file    boot_flash.h
    \brief   the only place that touches the FMC (and the RAM stand-in the host
             tests use instead)

    Two layers:

      * `boot_flash_read/write/erase_page` - raw access, no policy.  Used by
        `cfg_blob.c` for the configuration page and by `boot_upgrade.c` for the
        application slots through the checked wrappers below.

      * `boot_flash_slot_*` - the *only* functions the upgrade state machine is
        allowed to program flash with.  They refuse anything that is not inside
        slot A or slot B, so "erase the bootloader" or "erase the configuration
        page" is not expressible: a corrupted session cannot brick the board
        (`UPG_ERRCODE = UPG_ERR_RANGE` instead).

    With `IMU_TO_DXL_HOST_TEST` the same API is backed by a 256 KB RAM image
    with real flash semantics (erase -> 0xFF, program can only clear bits), so
    the state machine, its tests and the pty node simulator can exercise the
    real code paths on a PC.
*/

#ifndef BOOT_FLASH_H
#define BOOT_FLASH_H

#include <stdint.h>

#include "board.h"

/*! \brief copy `len` bytes out of flash (flash is memory mapped, so this is a
           memcpy; it exists so that the host build has one place to redirect). */
int boot_flash_read(uint32_t addr, void *dst, uint32_t len);

/*! \brief pointer to flash content, for in-place CRC computation. */
const uint8_t *boot_flash_ptr(uint32_t addr);

/*! \brief erase the 2 KB page containing `addr`.
    \retval 0 on success, -1 when `addr` is not in flash or not page aligned */
int boot_flash_erase_page(uint32_t addr);

/*! \brief program one 32-bit word.  The target bits must be erased (0xFFFF).
    \retval 0 on success, -1 on a misaligned / out-of-range / non-erased write */
int boot_flash_program_word(uint32_t addr, uint32_t word);

/* ── the checked slot-only layer used by the upgrade state machine ───────── */

/*! \brief base address of a slot (SLOT_A / SLOT_B), 0 when the index is bad. */
uint32_t boot_flash_slot_base(uint32_t slot);

/*! \brief erase the pages covering `[offset, offset+len)` inside a slot.
    \retval 0 on success, -1 when it would leave the slot or the arguments are
            not page aligned */
int boot_flash_slot_erase(uint32_t slot, uint32_t offset, uint32_t len);

/*! \brief program `len` bytes (multiple of 4, 4-byte aligned) into a slot.
    \retval 0 on success, -1 on any policy or hardware failure */
int boot_flash_slot_program(uint32_t slot, uint32_t offset, const void *src,
                            uint32_t len);

/*! \brief the CRC-covered region of a slot starts at `boot_flash_slot_base()`. */
const uint8_t *boot_flash_slot_ptr(uint32_t slot);

/* ── host-test / node-simulator backing store ───────────────────────────── */

#ifdef IMU_TO_DXL_HOST_TEST
/*! \brief erase the whole simulated flash to 0xFF and zero the counters. */
void boot_flash_test_reset(void);
/*! \brief write raw bytes into the simulated flash (no flash semantics). */
void boot_flash_test_load(uint32_t addr, const void *src, uint32_t len);
/*! \brief the whole simulated flash, for dumping a slot after a session. */
uint8_t *boot_flash_test_base(void);
uint32_t boot_flash_test_erase_count(void);
uint32_t boot_flash_test_program_count(void);
uint32_t boot_flash_test_reject_count(void);
#endif

#endif /* BOOT_FLASH_H */
