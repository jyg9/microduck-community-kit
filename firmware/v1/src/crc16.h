/*!
    \file    crc16.h
    \brief   CRC-16 used by Dynamixel protocol 2.0

    Poly 0x8005, init 0x0000, MSB first, no final XOR - the same "CRC-16/IBM"
    flavour the ROBOTIS SDKs use ("update_crc").  rustypot's table starts
    `0x0000, 0x8005, 0x800F, ...`, which is exactly this bitwise algorithm, and
    the CRC is computed over the whole packet including the `FF FF FD 00`
    header (rustypot: `crc(&bytes)`).
*/

#ifndef CRC16_H
#define CRC16_H

#include <stdint.h>

uint16_t crc16_dxl_update(uint16_t crc, uint8_t byte);
uint16_t crc16_dxl(const uint8_t *data, uint16_t len);

#endif /* CRC16_H */
