/*!
    \file    boot_crc32.c
    \brief   see boot_crc32.h
*/

#include "boot_crc32.h"

#include <stddef.h>

uint32_t boot_crc32_update(uint32_t crc, const void *data, uint32_t len)
{
    const uint8_t *p = (const uint8_t *)data;
    uint32_t i;

    crc = ~crc;
    while (len-- != 0U) {
        crc ^= (uint32_t)(*p++);
        for (i = 0U; i < 8U; i++) {
            /* branch-free: mask is 0xFFFFFFFF when bit 0 is set, else 0 */
            crc = (crc >> 1) ^ (0xEDB88320UL & (uint32_t)(-(int32_t)(crc & 1U)));
        }
    }
    return ~crc;
}

uint32_t boot_crc32(const void *data, uint32_t len)
{
    return boot_crc32_update(0U, data, len);
}
