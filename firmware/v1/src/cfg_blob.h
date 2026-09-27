/*!
    \file    cfg_blob.h
    \brief   the configuration page (last 2 KB) as one blob, shared by the
             application and the bootloader

    Layout (`CFG_VERSION = 2`):

      magic 'AIM1' | version | len | dev_cfg_t cfg | cfg_boot_t boot | crc16 | pad

    The CRC-16 (ROBOTIS flavour, `crc16_dxl()`) covers everything before the
    `crc` field.  A blob whose magic / version / length / CRC does not check out
    is ignored and the defaults are used - which is also how the application
    survives a power cut in the middle of a save, and how the bootloader
    survives a power cut in the middle of a trial counter update.

    `cfg_boot_t` is the part the bootloader owns: which slot was last confirmed,
    which slot is on trial, how many times it has been tried, whether a host
    asked for boot mode, and a monotonic ping-pong counter.  The application
    reads it at startup (to know whether it must confirm itself) and writes it
    when it has run long enough.
*/

#ifndef CFG_BLOB_H
#define CFG_BLOB_H

#include <stdint.h>

#include "board.h"
#include "dev.h"

#define CFG_MAGIC       0x314D4941UL    /* "AIM1" */
#define CFG_VERSION     2U

typedef struct {
    uint8_t boot_slot;      /* SLOT_A / SLOT_B / SLOT_NONE: last confirmed slot */
    uint8_t trial_slot;     /* SLOT_A / SLOT_B / SLOT_NONE: image on trial     */
    uint8_t attempts;       /* boots of the trial image so far                 */
    uint8_t boot_stay;      /* 1 = the bootloader must wait for a host         */
    uint8_t pingpong;       /* +1 on every committed image (freshness)         */
    uint8_t reserved[3];
} cfg_boot_t;

typedef struct {
    uint32_t   magic;
    uint16_t   version;
    uint16_t   len;         /* sizeof(dev_cfg_t) */
    dev_cfg_t  cfg;
    cfg_boot_t boot;
    uint16_t   crc;
    uint16_t   pad;
} cfg_blob_t;

/* the blob is programmed as 32-bit words, so it must be word sized */
typedef char cfg_blob_is_word_sized[(sizeof(cfg_blob_t) % 4U == 0U) ? 1 : -1];

/*! \brief defaults: no confirmed slot, nothing on trial, not staying in boot. */
void cfg_boot_defaults(cfg_boot_t *b);

/*! \brief read the configuration page.
    \retval 1 when a valid v2 blob was found (and copied to `out`),
            0 otherwise (the page is empty, v1, or corrupt) */
int cfg_blob_load(cfg_blob_t *out);

/*! \brief erase and rewrite the configuration page.  \retval 0 on success. */
int cfg_blob_save(const cfg_blob_t *b);

/*! \brief the configuration page address, for host-side dumping/tests. */
uint32_t cfg_blob_addr(void);

#endif /* CFG_BLOB_H */
