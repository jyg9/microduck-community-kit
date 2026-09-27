/*!
    \file    boot_crc32.h
    \brief   CRC-32 (IEEE 802.3, the zlib/Python `zlib.crc32` flavour)

    poly 0x04C11DB7, init 0xFFFFFFFF, reflected in/out, xorout 0xFFFFFFFF.
    Implemented bit by bit (no 1 KB table) because the bootloader has a 32 KB
    budget and the only caller runs once per boot: 96 KB costs ~25 ms at
    120 MHz, which is nothing next to the 2 s boot window.

    Host tests compare this against `zlib.crc32` for a set of vectors
    (`host/tools/test_crc32.c`), and the packaging tool (`host/package.py`)
    computes the same value from Python, so all three have to agree.
*/

#ifndef BOOT_CRC32_H
#define BOOT_CRC32_H

#include <stdint.h>

/*! \brief CRC-32 of a buffer, from scratch. */
uint32_t boot_crc32(const void *data, uint32_t len);

/*! \brief continue a CRC over another segment.
    \param[in] crc: the value returned for the previous segment (0 = start)
    \retval the CRC of the concatenation, which is what
            `app_image_crc_combined()` and `zlib.crc32(second, first)` both mean. */
uint32_t boot_crc32_update(uint32_t crc, const void *data, uint32_t len);

#endif /* BOOT_CRC32_H */
