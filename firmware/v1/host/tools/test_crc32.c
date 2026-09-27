/*!
    \file    test_crc32.c
    \brief   the bootloader's bit-by-bit CRC-32 against Python's zlib.crc32

    The upgrade path has three independent implementations of the same checksum:
    the node (`src/boot_crc32.c`), the packaging tool (`host/package.py`,
    `zlib.crc32`) and the host tests.  If they ever disagree, `UPG_END` reports a
    CRC failure on an image that is actually fine - which looks exactly like a
    hardware fault.  Hence this test: the golden values below were produced by
    `zlib.crc32`, and the pty end-to-end test checks the other direction.

    Build and run through `host/tools/run_c_tests.sh`, or by hand:

        gcc -std=gnu99 -Wall -Wextra -o /tmp/t test_crc32.c ../../src/boot_crc32.c
        /tmp/t
*/

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "boot_crc32.h"

static int failures;

static void check(const char *name, uint32_t got, uint32_t want)
{
    if (got != want) {
        printf("  FAIL %-24s got 0x%08X want 0x%08X\n", name,
               (unsigned)got, (unsigned)want);
        failures++;
    } else {
        printf("  ok   %-24s 0x%08X\n", name, (unsigned)got);
    }
}

int main(void)
{
    uint8_t buf[4096];
    uint8_t ff[1024];
    uint32_t i;
    uint32_t crc;

    printf("CRC-32 (zlib/Python flavour) golden vectors\n");

    check("empty", boot_crc32(NULL, 0U), 0x00000000U);
    check("'123456789'", boot_crc32("123456789", 9U), 0xCBF43926U);

    memset(ff, 0xFF, sizeof(ff));
    check("0xFF x 1024", boot_crc32(ff, sizeof(ff)), 0xB83AFFF4U);

    for (i = 0U; i < sizeof(buf); i++) {
        buf[i] = (uint8_t)(i & 0xFFU);
    }
    /* bytes(range(256)) * 4 is the first 1024 bytes of this buffer */
    check("0..255 x 4", boot_crc32(buf, 1024U), 0xB70B4C26U);

    /* a 4096-byte pseudo-random block, reproduced from the generator the golden
       value came from (xorshift32 seeded with 7 in the Python reference) */
    {
        uint32_t x = 7U;
        for (i = 0U; i < sizeof(buf); i++) {
            x ^= x << 13;
            x ^= x >> 17;
            x ^= x << 5;
            buf[i] = (uint8_t)(x >> 24);
        }
    }
    crc = boot_crc32(buf, 1000U);
    check("random 1000 (segment 1)", crc, 0xE09A8515U);
    check("random 4096 (one shot)", boot_crc32(buf, sizeof(buf)), 0x0C5D0E0FU);
    check("random 4096 (continued)",
          boot_crc32_update(crc, &buf[1000], (uint32_t)sizeof(buf) - 1000U),
          0x0C5D0E0FU);

    /* continuation must be exactly "crc of the concatenation" */
    check("concat == update",
          boot_crc32_update(boot_crc32("1234", 4U), "56789", 5U),
          boot_crc32("123456789", 9U));

    /* the two image segments the node hashes, with the header hole between */
    {
        uint8_t image[0x300];
        uint32_t low;
        uint32_t combined;

        for (i = 0U; i < sizeof(image); i++) {
            image[i] = (uint8_t)(0xA5U ^ i);
        }
        low = boot_crc32(image, 0x200U);
        combined = boot_crc32_update(low, &image[0x240], 0x300U - 0x240U);
        check("image combined (0x200|0x240..)",
              combined, 0x9CBB078EU);
    }

    if (failures) {
        printf("%d FAILURE(S)\n", failures);
        return 1;
    }
    printf("all CRC-32 vectors match zlib.crc32\n");
    return 0;
}
