/*!
    \file    crc16.c
    \brief   see crc16.h
*/

#include "crc16.h"

uint16_t crc16_dxl_update(uint16_t crc, uint8_t byte)
{
    uint8_t i;

    crc ^= (uint16_t)((uint16_t)byte << 8);
    for (i = 0U; i < 8U; i++) {
        if (0U != (crc & 0x8000U)) {
            crc = (uint16_t)((uint16_t)(crc << 1) ^ 0x8005U);
        } else {
            crc = (uint16_t)(crc << 1);
        }
    }
    return crc;
}

uint16_t crc16_dxl(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0U;
    uint16_t i;

    for (i = 0U; i < len; i++) {
        crc = crc16_dxl_update(crc, data[i]);
    }
    return crc;
}
