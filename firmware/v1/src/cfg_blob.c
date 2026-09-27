/*!
    \file    cfg_blob.c
    \brief   see cfg_blob.h
*/

#include "cfg_blob.h"

#include <stddef.h>
#include <string.h>

#include "boot_flash.h"
#include "crc16.h"

void cfg_boot_defaults(cfg_boot_t *b)
{
    memset(b, 0, sizeof(*b));
    b->boot_slot = SLOT_NONE;
    b->trial_slot = SLOT_NONE;
    b->attempts = 0U;
    b->boot_stay = 0U;
    b->pingpong = 0U;
}

uint32_t cfg_blob_addr(void)
{
    return CFG_FLASH_ADDR;
}

/* CRC over everything before the `crc` field */
static uint16_t blob_crc(const cfg_blob_t *b)
{
    return crc16_dxl((const uint8_t *)b, (uint16_t)offsetof(cfg_blob_t, crc));
}

int cfg_blob_load(cfg_blob_t *out)
{
    cfg_blob_t b;

    if (NULL == out) {
        return 0;
    }
    if (0 != boot_flash_read(CFG_FLASH_ADDR, &b, (uint32_t)sizeof(b))) {
        return 0;
    }
    if (b.magic != CFG_MAGIC || b.version != CFG_VERSION
        || b.len != (uint16_t)sizeof(dev_cfg_t)) {
        return 0;
    }
    if (blob_crc(&b) != b.crc) {
        return 0;
    }
    memcpy(out, &b, sizeof(b));
    return 1;
}

int cfg_blob_save(const cfg_blob_t *b)
{
    cfg_blob_t w;
    uint32_t words;
    uint32_t i;

    if (NULL == b) {
        return -1;
    }
    memcpy(&w, b, sizeof(w));
    w.magic = CFG_MAGIC;
    w.version = CFG_VERSION;
    w.len = (uint16_t)sizeof(dev_cfg_t);
    w.pad = 0U;
    w.crc = blob_crc(&w);

    if (0 != boot_flash_erase_page(CFG_FLASH_ADDR)) {
        return -1;
    }
    words = (uint32_t)sizeof(w) / 4U;
    for (i = 0U; i < words; i++) {
        uint32_t word;
        memcpy(&word, (const uint8_t *)&w + i * 4U, 4U);
        if (0 != boot_flash_program_word(CFG_FLASH_ADDR + i * 4U, word)) {
            return -1;
        }
    }
    return 0;
}
