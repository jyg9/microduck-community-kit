/*!
    \file    test_crc16.c
    \brief   host-side check that the firmware's CRC-16 matches the ROBOTIS /
             rustypot reference

    Build and run:
        gcc -Wall -Wextra -o /tmp/test_crc16 test_crc16.c ../../src/crc16.c && /tmp/test_crc16

    The vectors are the ones rustypot 0.6 asserts in `protocol/v2.rs` tests, so a
    pass here means the firmware produces byte-identical instructions and status
    packets to the library microduck actually uses.
*/

#include <stdio.h>
#include <string.h>

#include "../../src/crc16.h"

static int failures;

static void expect_crc(const char *name, const unsigned char *data, unsigned len, unsigned expect)
{
    unsigned got = crc16_dxl(data, (unsigned short)len);
    if (got != expect) {
        printf("FAIL %-28s crc=0x%04X want 0x%04X\n", name, got, expect);
        failures++;
    } else {
        printf("ok   %-28s crc=0x%04X\n", name, got);
    }
}

/* CRC is taken over the whole packet including the header, minus the CRC bytes */
static void expect_packet(const char *name, const unsigned char *pkt, unsigned total)
{
    unsigned got = crc16_dxl(pkt, (unsigned short)(total - 2));
    unsigned want = (unsigned)pkt[total - 2] | ((unsigned)pkt[total - 1] << 8);
    if (got != want) {
        printf("FAIL %-28s crc=0x%04X packet says 0x%04X\n", name, got, want);
        failures++;
    } else {
        printf("ok   %-28s crc=0x%04X\n", name, got);
    }
}

int main(void)
{
    /* rustypot: create_ping_packet, id 2 */
    static const unsigned char ping[] = { 0xFF, 0xFF, 0xFD, 0x00, 0x02, 0x03, 0x00, 0x01, 0x19, 0x72 };
    /* rustypot: create_read_packet, id 1, addr 0x2B, len 2 */
    static const unsigned char rd[] = { 0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x07, 0x00, 0x02, 0x2B, 0x00, 0x02, 0x00, 0x2E, 0xCD };
    /* rustypot: create_write_packet, id 1, addr 116, u32 512 */
    static const unsigned char wr[] = { 0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x09, 0x00, 0x03, 0x74, 0x00, 0x00, 0x02, 0x00, 0x00, 0xCA, 0x89 };
    /* rustypot: create_sync_read_packet, ids 1,2, addr 132, len 4 */
    static const unsigned char sr[] = { 0xFF, 0xFF, 0xFD, 0x00, 0xFE, 0x09, 0x00, 0x82, 0x84, 0x00, 0x04, 0x00, 0x01, 0x02, 0xCE, 0xFA };
    /* rustypot: create_sync_write_packet, ids 1,2, addr 116 */
    static const unsigned char sw[] = { 0xFF, 0xFF, 0xFD, 0x00, 0xFE, 0x11, 0x00, 0x83, 0x74, 0x00, 0x04, 0x00,
                                        0x01, 0x96, 0x00, 0x00, 0x00, 0x02, 0xAA, 0x00, 0x00, 0x00, 0x82, 0x87 };
    /* rustypot: parse_status_packet (a real XL430 read answer) */
    static const unsigned char st[] = { 0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x08, 0x00, 0x55, 0x00, 0xA6, 0x00, 0x00, 0x00, 0x8C, 0xC0 };
    /* rustypot: test_crc uses this raw byte string */
    static const unsigned char raw[] = { 0xFF, 0xFF, 0xFD, 0x00, 0x2A, 0x03, 0x00, 0x01 };

    expect_packet("ping id=2", ping, sizeof(ping));
    expect_packet("read id=1 addr=0x2B", rd, sizeof(rd));
    expect_packet("write id=1 addr=116", wr, sizeof(wr));
    expect_packet("sync_read [1,2] addr=132", sr, sizeof(sr));
    expect_packet("sync_write [1,2] addr=116", sw, sizeof(sw));
    expect_packet("status read answer", st, sizeof(st));
    expect_crc("raw rustypot test_crc", raw, sizeof(raw), 0xD216);

    if (failures != 0) {
        printf("\n%d FAILURE(S)\n", failures);
        return 1;
    }
    printf("\nall CRC vectors match rustypot/ROBOTIS\n");
    return 0;
}
