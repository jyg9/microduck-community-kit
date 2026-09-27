/*!
    \file    boot_ram.h
    \brief   the application -> bootloader handshake word in SRAM

    The application cannot re-enter the bootloader by itself (the bootloader is
    only reachable through the reset vector), so a vendor command writes a magic
    value here and calls `NVIC_SystemReset()`.  The bootloader checks the word
    before it looks at the flash headers, so it works even when the application
    it was just asked to leave is the only valid image.

    Where it lives matters: `g_boot_ram` is linked into its own `.boot_ram`
    section, placed by `linker/gd32f303cc_*.ld` in the last 256 bytes of SRAM and *outside*
    the `.bss` range the startup code clears.  A normal `.bss`/`.data` variable
    would be zeroed by the reset path before `main()` ever runs.

    SRAM on this part has no VBAT domain, so the word only survives a warm reset
    (soft reset / watchdog).  It deliberately does NOT survive power loss; the
    persisted `cfg_boot_t.boot_stay` flag is what covers that case.
*/

#ifndef BOOT_RAM_H
#define BOOT_RAM_H

#include <stdint.h>

#include "board.h"

typedef struct {
    uint32_t magic;     /* BOOT_RAM_MAGIC while a boot-mode entry is requested */
    uint32_t arg;       /* reserved for the bootloader (0 today) */
} boot_ram_t;

/*! \brief the shared word; the linker places it at BOOT_RAM_ADDR in both the
           application and the bootloader image. */
extern boot_ram_t g_boot_ram;

/*! \brief application side: ask the bootloader to take over after the reset. */
void boot_ram_request(uint32_t arg);

/*! \brief bootloader side: read the request and clear it (so that a later
           cold reset is not mistaken for a request).
    \retval 1 when the magic was present, 0 otherwise */
int boot_ram_consume(uint32_t *arg);

#endif /* BOOT_RAM_H */
