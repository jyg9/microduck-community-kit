/*!
    \file    test_boot.c
    \brief   host tests for the A/B bootloader: header, decision, flash policy,
             upgrade state machine, protocol integration

    Everything here runs the *real* firmware sources on a PC:

      * `src/boot_flash.c` is compiled with `IMU_TO_DXL_HOST_TEST`, which backs
        the FMC with a 256 KB RAM image that behaves like flash (page erase ->
        0xFF, programming can only clear bits, and a write over non-erased data
        is refused like the real FMC refuses it);
      * `src/boot_upgrade.c`, `src/boot_regs.c`, `src/boot_decide.c`,
        `src/app_header.c` and `src/cfg_blob.c` are the same objects that go into
        the bootloader;
      * the protocol-level tests drive `src/dxl2.c` / `src/fee.c` byte by byte,
        so framing, byte stuffing, the CRC and the state machine are exercised
        together - not just the register window.

    Build and run through `host/tools/run_c_tests.sh`.
*/

#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app_header.h"
#include "boot_crc32.h"
#include "boot_decide.h"
#include "boot_flash.h"
#include "boot_regs.h"
#include "boot_upgrade.h"
#include "board.h"
#include "crc16.h"
#include "dbg.h"
#include "dxl2.h"
#include "fee.h"

/* ── test harness ───────────────────────────────────────────────────────── */

static int      g_checks;
static int      g_failures;
static uint32_t g_now_ms = 1000U;

/* boot_regs.c only needs a millisecond clock; the pty simulator uses the same
   indirection, and a test can jump time forward by writing g_now_ms */
uint32_t systick_get_ms(void);

uint32_t systick_get_ms(void)
{
    return g_now_ms;
}

/* the console is switchable and must stay out of the tests */
void dbg_log(uint8_t level, const char *fmt, ...);
void dbg_hex(uint8_t level, const char *tag, const uint8_t *data, uint16_t len);

void dbg_log(uint8_t level, const char *fmt, ...)
{
    (void)level;
    (void)fmt;
}

void dbg_hex(uint8_t level, const char *tag, const uint8_t *data, uint16_t len)
{
    (void)level;
    (void)tag;
    (void)data;
    (void)len;
}

static void check(int ok, const char *fmt, ...)
{
    va_list ap;

    g_checks++;
    if (ok) {
        return;
    }
    g_failures++;
    printf("  FAIL ");
    va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);
    printf("\n");
}

static void check_eq_u32(uint32_t got, uint32_t want, const char *what)
{
    check(got == want, "%s: got %u (0x%X), want %u (0x%X)", what,
          (unsigned)got, (unsigned)got, (unsigned)want, (unsigned)want);
}

/* same, with a printf-style label (for messages that carry the error code) */
static void check_u32_fmt(uint32_t got, uint32_t want, const char *fmt, ...)
{
    va_list ap;
    char label[160];

    va_start(ap, fmt);
    vsnprintf(label, sizeof(label), fmt, ap);
    va_end(ap);
    check_eq_u32(got, want, label);
}

/* ── image helpers ──────────────────────────────────────────────────────── */

/* A synthetic but structurally valid slot image: a vector table (MSP + Thumb
   entry), the 64-byte header hole, and a payload that depends on `seed` so
   crc_high is not trivially the same everywhere. */
static uint8_t *make_image(uint8_t *buf, uint8_t slot, uint32_t size,
                           uint16_t version, uint8_t seed)
{
    uint32_t base = (SLOT_A == slot) ? SLOT_A_BASE : SLOT_B_BASE;
    uint32_t i;

    memset(buf, 0xFF, size);
    memset(buf, 0, 16U);
    /* initial MSP inside SRAM, entry point inside the slot with the Thumb bit */
    {
        uint32_t sp = 0x2000BF00U;
        uint32_t entry = base + 0x241U;
        memcpy(&buf[0], &sp, 4U);
        memcpy(&buf[4], &entry, 4U);
    }
    for (i = 0x240U; i < size; i++) {
        buf[i] = (uint8_t)((i * 31U + seed) & 0xFFU);
    }
    return buf;
}

/* Put a valid image into a slot of the simulated flash. */
static void load_slot(uint8_t slot, uint16_t version, uint32_t size, uint8_t seed)
{
    static uint8_t buf[SLOT_SIZE];
    uint32_t base = boot_flash_slot_base(slot);
    int rc;

    make_image(buf, slot, size, version, seed);
    rc = app_header_fill(buf, size, version, 0U, 0U);
    check(rc == APP_HDR_OK, "app_header_fill(slot %c) failed: %s",
          (SLOT_A == slot) ? 'A' : 'B', app_hdr_strerror(rc));
    boot_flash_test_load(base, buf, size);
}

/* A slot image held in RAM, ready to be sent over the upgrade window. */
static uint8_t *build_upgrade_image(uint8_t *buf, uint8_t slot, uint32_t size,
                                    uint16_t version)
{
    int rc = app_header_fill(make_image(buf, slot, size, version, 0x5AU),
                             size, version, 0U, APP_HDR_FLAG_TRIAL);
    check(rc == APP_HDR_OK, "fill upgrade image: %s", app_hdr_strerror(rc));
    return buf;
}

/* ── upgrade window helpers ─────────────────────────────────────────────── */

static void upg_write_expect(uint8_t off, const uint8_t *p, uint8_t n,
                            uint8_t want)
{
    uint8_t got = upg_write(off, p, n, g_now_ms);
    check(got == want, "upg_write(off=%u n=%u): got err 0x%02X want 0x%02X",
          (unsigned)off, (unsigned)n, got, want);
}

static void upg_put(uint8_t off, const uint8_t *p, uint8_t n)
{
    upg_write_expect(off, p, n, 0U);
}

static void upg_open_session(uint8_t target, uint32_t imglen, uint32_t imgcrc,
                            uint16_t ver)
{
    uint8_t tmp[4];

    tmp[0] = (uint8_t)(UPGRADE_MAGIC & 0xFFU);
    tmp[1] = (uint8_t)(UPGRADE_MAGIC >> 8);
    upg_put(U_MAGIC_L, tmp, 2U);
    upg_put(U_TARGET, &target, 1U);
    tmp[0] = (uint8_t)(imglen & 0xFFU);
    tmp[1] = (uint8_t)((imglen >> 8) & 0xFFU);
    tmp[2] = (uint8_t)((imglen >> 16) & 0xFFU);
    tmp[3] = (uint8_t)((imglen >> 24) & 0xFFU);
    upg_put(U_IMGLEN_0, tmp, 4U);
    tmp[0] = (uint8_t)(imgcrc & 0xFFU);
    tmp[1] = (uint8_t)((imgcrc >> 8) & 0xFFU);
    tmp[2] = (uint8_t)((imgcrc >> 16) & 0xFFU);
    tmp[3] = (uint8_t)((imgcrc >> 24) & 0xFFU);
    upg_put(U_IMGCRC_0, tmp, 4U);
    tmp[0] = (uint8_t)(ver & 0xFFU);
    tmp[1] = (uint8_t)(ver >> 8);
    upg_put(U_VER_L, tmp, 2U);
}

static void upg_cmd(uint8_t cmd)
{
    upg_put(U_CMD, &cmd, 1U);
    /* the node always clears the command register */
    check_eq_u32(upg_read(U_CMD), 0U, "UPG_CMD reads back 0");
}

static uint8_t upg_status(void)
{
    return upg_read(U_STATUS);
}

/* Send one block with a corrupted CRC, to prove the node refuses it. */
static uint8_t upg_send_block_badcrc(uint16_t seq, uint32_t off, const uint8_t *data,
                                    uint8_t len)
{
    uint8_t win[UPGRADE_WIN_LEN];
    uint16_t crc = (uint16_t)(crc16_dxl(data, len) ^ 0x5A5AU);

    memset(win, 0, sizeof(win));
    win[U_SEQ_L] = (uint8_t)(seq & 0xFFU);
    win[U_SEQ_H] = (uint8_t)(seq >> 8);
    win[U_OFF_0] = (uint8_t)(off & 0xFFU);
    win[U_OFF_1] = (uint8_t)((off >> 8) & 0xFFU);
    win[U_OFF_2] = (uint8_t)((off >> 16) & 0xFFU);
    win[U_OFF_3] = (uint8_t)((off >> 24) & 0xFFU);
    win[U_BLKLEN] = len;
    win[U_BLKCRC_L] = (uint8_t)(crc & 0xFFU);
    win[U_BLKCRC_H] = (uint8_t)(crc >> 8);
    if (0U != upg_write(U_SEQ_L, &win[U_SEQ_L],
                        (uint8_t)(U_BLKCRC_H - U_SEQ_L + 1U), g_now_ms)) {
        return 1U;
    }
    return upg_write(U_DATA_OFF, data, len, g_now_ms);
}

/* Send one 4..16 byte block the way the host tool does: a control frame and a
   data frame. */
static uint8_t upg_send_block(uint16_t seq, uint32_t off, const uint8_t *data,
                             uint8_t len)
{
    uint8_t win[UPGRADE_WIN_LEN];
    uint8_t rc;

    memset(win, 0, sizeof(win));
    win[U_SEQ_L] = (uint8_t)(seq & 0xFFU);
    win[U_SEQ_H] = (uint8_t)(seq >> 8);
    win[U_OFF_0] = (uint8_t)(off & 0xFFU);
    win[U_OFF_1] = (uint8_t)((off >> 8) & 0xFFU);
    win[U_OFF_2] = (uint8_t)((off >> 16) & 0xFFU);
    win[U_OFF_3] = (uint8_t)((off >> 24) & 0xFFU);
    win[U_BLKLEN] = len;
    {
        uint16_t crc = crc16_dxl(data, len);
        win[U_BLKCRC_L] = (uint8_t)(crc & 0xFFU);
        win[U_BLKCRC_H] = (uint8_t)(crc >> 8);
    }
    rc = upg_write(U_SEQ_L, &win[U_SEQ_L],
                   (uint8_t)(U_BLKCRC_H - U_SEQ_L + 1U), g_now_ms);
    if (rc != 0U) {
        return rc;
    }
    return upg_write(U_DATA_OFF, data, len, g_now_ms);
}

/* Send a whole image (must be a multiple of 16 bytes; the tail is sent as the
   remainder, which the packaging tool guarantees is a multiple of 4). */
static void upg_send_image(const uint8_t *img, uint32_t size)
{
    uint32_t off = 0U;
    uint16_t seq = 0U;

    while (off < size) {
        uint32_t left = size - off;
        uint8_t len = (uint8_t)((left >= 16U) ? 16U : left);

        if (0U != (len % 4U)) {
            check(0, "image size %u is not a multiple of 4", (unsigned)size);
            return;
        }
        check(0U == upg_send_block(seq, off, &img[off], len),
              "block seq=%u off=%u rejected (status 0x%02X err %u)",
              (unsigned)seq, (unsigned)off, (unsigned)upg_status(),
              (unsigned)upg_read(U_ERRCODE));
        off += len;
        seq++;
    }
}

/* ── protocol frame helpers (mirroring host/tools/test_protocols.c) ─────── */

static dxl_slave_t s_dxl_slave;
static fee_slave_t s_fee_slave;

static uint16_t dxl_frame(uint8_t id, uint8_t inst, const uint8_t *params,
                         uint16_t nparams, uint8_t *out)
{
    uint16_t total = (uint16_t)(nparams + 10U);
    uint16_t crc;

    out[0] = 0xFFU; out[1] = 0xFFU; out[2] = 0xFDU; out[3] = 0x00U;
    out[4] = id;
    out[5] = (uint8_t)(nparams + 3U);
    out[6] = 0x00U;
    out[7] = inst;
    if (nparams != 0U) {
        memcpy(&out[8], params, nparams);
    }
    crc = crc16_dxl(out, (uint16_t)(total - 2U));
    out[total - 2U] = (uint8_t)(crc & 0xFFU);
    out[total - 1U] = (uint8_t)(crc >> 8);
    return total;
}

static uint16_t dxl_feed(const uint8_t *frame, uint16_t len, uint8_t *resp,
                        uint16_t resp_max)
{
    uint16_t rlen = 0U;
    uint16_t i;

    for (i = 0U; i < len; i++) {
        uint16_t got = dxl_slave_feed(&s_dxl_slave, frame[i], g_now_ms, resp, resp_max);
        if (0U != got) {
            rlen = got;
        }
    }
    return rlen;
}

/* One DXL INST_WRITE, returns the response length (0 = none).
   `addr` is a register address, i.e. UPGRADE_WIN_ADDR + window offset. */
static uint16_t dxl_write(const uint8_t *data, uint16_t len, uint8_t addr,
                         uint8_t *resp, uint16_t resp_max)
{
    uint8_t params[64];
    uint8_t frame[96];
    uint16_t flen;

    params[0] = addr;
    params[1] = 0U;
    memcpy(&params[2], data, len);
    flen = dxl_frame(200U, DXL_INST_WRITE, params, (uint16_t)(len + 2U), frame);
    return dxl_feed(frame, flen, resp, resp_max);
}

static uint16_t fee_frame(uint8_t id, uint8_t inst, const uint8_t *params,
                         uint8_t nparams, uint8_t *out)
{
    uint8_t sum = 0U;
    uint8_t i;
    uint16_t total = (uint16_t)(nparams + 6U);

    out[0] = 0xFFU; out[1] = 0xFFU; out[2] = id;
    out[3] = (uint8_t)(nparams + 2U);
    out[4] = inst;
    if (nparams != 0U) {
        memcpy(&out[5], params, nparams);
    }
    for (i = 2U; i < (uint8_t)(5U + nparams); i++) {
        sum = (uint8_t)(sum + out[i]);
    }
    out[5U + nparams] = (uint8_t)(~sum);
    return total;
}

static uint16_t fee_feed(const uint8_t *frame, uint16_t len, uint8_t *resp,
                        uint16_t resp_max)
{
    uint16_t rlen = 0U;
    uint16_t i;

    for (i = 0U; i < len; i++) {
        uint16_t got = fee_slave_feed(&s_fee_slave, frame[i], g_now_ms, resp, resp_max);
        if (0U != got) {
            rlen = got;
        }
    }
    return rlen;
}

static uint16_t fee_write(const uint8_t *data, uint8_t len, uint8_t addr,
                         uint8_t *resp, uint16_t resp_max)
{
    uint8_t params[64];
    uint8_t frame[96];
    uint16_t flen;

    params[0] = addr;
    memcpy(&params[1], data, len);
    flen = fee_frame(200U, FEE_INST_WRITE, params, (uint8_t)(len + 1U), frame);
    return fee_feed(frame, flen, resp, resp_max);
}

/* the target used by upgrade_over_dxl (set by each test before calling it) */
static uint8_t upg_target_of_current_test = SLOT_B;

/* Feed a whole slot image through real DXL frames, 16 bytes at a time. */
static void upgrade_over_dxl(const uint8_t *img, uint32_t size)
{
    uint8_t resp[DXL_MAX_RESP];
    uint8_t win[UPGRADE_WIN_LEN];
    uint8_t tmp[8];
    uint32_t off = 0U;
    uint16_t seq = 0U;

    tmp[0] = (uint8_t)(UPGRADE_MAGIC & 0xFFU);
    tmp[1] = (uint8_t)(UPGRADE_MAGIC >> 8);
    dxl_write(tmp, 2U, UPGRADE_WIN_ADDR + U_MAGIC_L, resp, sizeof(resp));
    tmp[0] = upg_target_of_current_test;
    dxl_write(tmp, 1U, UPGRADE_WIN_ADDR + U_TARGET, resp, sizeof(resp));
    memcpy(tmp, &size, 4U);
    dxl_write(tmp, 4U, UPGRADE_WIN_ADDR + U_IMGLEN_0, resp, sizeof(resp));
    tmp[0] = 0U; tmp[1] = 0U; tmp[2] = 0U; tmp[3] = 0U; tmp[4] = 0U; tmp[5] = 0U;
    dxl_write(tmp, 6U, UPGRADE_WIN_ADDR + U_VER_L, resp, sizeof(resp));
    {
        uint32_t crc = app_image_crc_combined(img, size);
        memcpy(tmp, &crc, 4U);
        dxl_write(tmp, 4U, UPGRADE_WIN_ADDR + U_IMGCRC_0, resp, sizeof(resp));
    }
    tmp[0] = UPG_CMD_BEGIN;
    dxl_write(tmp, 1U, UPGRADE_WIN_ADDR + U_CMD, resp, sizeof(resp));
    check(upg_status() == UPG_ST_ERASED, "BEGIN over DXL: status 0x%02X err %u",
          (unsigned)upg_status(), (unsigned)upg_read(U_ERRCODE));

    while (off < size) {
        uint32_t left = size - off;
        uint8_t len = (uint8_t)((left >= 16U) ? 16U : left);
        uint16_t crc = crc16_dxl(&img[off], len);

        memset(win, 0, sizeof(win));
        win[0] = (uint8_t)(seq & 0xFFU);
        win[1] = (uint8_t)(seq >> 8);
        win[2] = (uint8_t)(off & 0xFFU);
        win[3] = (uint8_t)((off >> 8) & 0xFFU);
        win[4] = (uint8_t)((off >> 16) & 0xFFU);
        win[5] = (uint8_t)((off >> 24) & 0xFFU);
        /* seq/off/blklen/blkcrc are contiguous: 213..221 */
        win[6] = len;
        win[7] = (uint8_t)(crc & 0xFFU);
        win[8] = (uint8_t)(crc >> 8);
        dxl_write(&win[0], 9U, UPGRADE_WIN_ADDR + U_SEQ_L, resp, sizeof(resp));
        dxl_write(&img[off], len, UPGRADE_WIN_ADDR + U_DATA_OFF, resp, sizeof(resp));
        check(upg_status() == UPG_ST_BLOCK_OK, "DXL block %u: status 0x%02X err %u",
              (unsigned)seq, (unsigned)upg_status(),
              (unsigned)upg_read(U_ERRCODE));
        check_eq_u32(upg_read(U_ACK_SEQ_L) | ((uint32_t)upg_read(U_ACK_SEQ_H) << 8),
                     seq, "DXL ACK_SEQ follows");
        off += len;
        seq++;
    }

    tmp[0] = UPG_CMD_END;
    dxl_write(tmp, 1U, UPGRADE_WIN_ADDR + U_CMD, resp, sizeof(resp));
}

/* ── tests ──────────────────────────────────────────────────────────────── */

static void test_app_header_layout(void)
{
    printf("app header layout\n");
    check_eq_u32(sizeof(app_header_t), APP_HDR_SIZE, "sizeof(app_header_t)");
    check_eq_u32(APP_HDR_OFF, 0x200U, "APP_HDR_OFF");
    check_eq_u32(offsetof(app_header_t, magic), 0U, "offsetof(magic)");
    check_eq_u32(offsetof(app_header_t, header_version), 4U, "offsetof(header_version)");
    check_eq_u32(offsetof(app_header_t, image_version), 6U, "offsetof(image_version)");
    check_eq_u32(offsetof(app_header_t, image_size), 8U, "offsetof(image_size)");
    check_eq_u32(offsetof(app_header_t, crc_low), 12U, "offsetof(crc_low)");
    check_eq_u32(offsetof(app_header_t, crc_high), 16U, "offsetof(crc_high)");
    check_eq_u32(offsetof(app_header_t, build_unix), 20U, "offsetof(build_unix)");
    check_eq_u32(offsetof(app_header_t, entry), 24U, "offsetof(entry)");
    check_eq_u32(offsetof(app_header_t, board_id), 28U, "offsetof(board_id)");
    check_eq_u32(offsetof(app_header_t, proto_version), 30U, "offsetof(proto_version)");
    check_eq_u32(offsetof(app_header_t, flags), 32U, "offsetof(flags)");
    check_eq_u32(offsetof(app_header_t, reserved), 34U, "offsetof(reserved)");
    check_eq_u32(APP_VERSION, APP_VERSION_MAKE(FW_VERSION_MAJOR, FW_VERSION_MINOR, 0),
                 "APP_VERSION");
    check_eq_u32(APP_VERSION_MAJOR(APP_VERSION_MAKE(2, 15, 255)), 2U, "ver major");
    check_eq_u32(APP_VERSION_MINOR(APP_VERSION_MAKE(2, 15, 255)), 15U, "ver minor");
    check_eq_u32(APP_VERSION_PATCH(APP_VERSION_MAKE(2, 15, 255)), 255U, "ver patch");
}

static void test_header_check(void)
{
    static uint8_t buf[SLOT_SIZE];
    app_header_t h;
    uint32_t size = 0x800U;
    uint8_t tmp;
    uint8_t size_field[4];

    printf("app header validation\n");

    make_image(buf, SLOT_A, size, APP_VERSION, 1U);
    check_eq_u32((uint32_t)app_header_fill(buf, size, APP_VERSION, 1234U,
                                           APP_HDR_FLAG_TRIAL), APP_HDR_OK,
                 "fill a good image");
    check_eq_u32((uint32_t)app_header_check(buf, SLOT_SIZE, &h), APP_HDR_OK, "check ok");
    check_eq_u32(h.image_size, size, "image_size recorded");
    check_eq_u32(h.image_version, APP_VERSION, "image_version recorded");
    check_eq_u32(h.build_unix, 1234U, "build_unix recorded");
    check(1 == app_header_entry_in_slot(&h, SLOT_A_BASE, SLOT_SIZE),
          "entry inside slot A");
    check(0 == app_header_entry_in_slot(&h, SLOT_B_BASE, SLOT_SIZE),
          "entry outside slot B");

    /* size checks */
    check_eq_u32((uint32_t)app_header_check(buf + 1, SLOT_SIZE - 1U, NULL),
                 APP_HDR_ERR_MAGIC, "misaligned base sees no magic");
    check_eq_u32((uint32_t)app_header_check(buf, UPG_MIN_IMGLEN - 4U, NULL),
                 APP_HDR_ERR_SIZE, "undersized buffer");

    /* every failure branch */
    {
        uint8_t save[APP_HDR_SIZE];
        memcpy(save, &buf[APP_HDR_OFF], APP_HDR_SIZE);

        buf[APP_HDR_OFF] ^= 0xFFU;
        check_eq_u32((uint32_t)app_header_check(buf, SLOT_SIZE, NULL),
                     APP_HDR_ERR_MAGIC, "bad magic");
        memcpy(&buf[APP_HDR_OFF], save, APP_HDR_SIZE);

        buf[APP_HDR_OFF + 4] = 9U;
        check_eq_u32((uint32_t)app_header_check(buf, SLOT_SIZE, NULL),
                     APP_HDR_ERR_VERSION, "bad header version");
        buf[APP_HDR_OFF + 4] = APP_HDR_VERSION;

        memcpy(size_field, &buf[APP_HDR_OFF + 8], 4U);
        memset(&buf[APP_HDR_OFF + 8], 0, 4U);
        check_eq_u32((uint32_t)app_header_check(buf, SLOT_SIZE, NULL),
                     APP_HDR_ERR_SIZE, "image_size below the minimum");
        memset(&buf[APP_HDR_OFF + 8], 0xFF, 4U);
        check_eq_u32((uint32_t)app_header_check(buf, SLOT_SIZE, NULL),
                     APP_HDR_ERR_SIZE, "image_size above the slot");
        {
            uint32_t bad_size = size + 2U;
            memcpy(&buf[APP_HDR_OFF + 8], &bad_size, 4U);
        }
        check_eq_u32((uint32_t)app_header_check(buf, SLOT_SIZE, NULL),
                     APP_HDR_ERR_ALIGN, "unaligned image_size");
        memcpy(&buf[APP_HDR_OFF + 8], size_field, 4U);

        memcpy(&tmp, &buf[4], 1U);
        buf[4] = 0x01U;
        check_eq_u32((uint32_t)app_header_check(buf, SLOT_SIZE, NULL),
                     APP_HDR_ERR_ENTRY, "entry does not match the vector table");
        memcpy(&buf[4], &tmp, 1U);

        buf[APP_HDR_OFF + 12] ^= 0x01U;
        check_eq_u32((uint32_t)app_header_check(buf, SLOT_SIZE, NULL),
                     APP_HDR_ERR_CRC_LOW, "crc_low mismatch");
        buf[APP_HDR_OFF + 12] ^= 0x01U;

        buf[APP_HDR_OFF + 16] ^= 0x01U;
        check_eq_u32((uint32_t)app_header_check(buf, SLOT_SIZE, NULL),
                     APP_HDR_ERR_CRC_HIGH, "crc_high mismatch");
        buf[APP_HDR_OFF + 16] ^= 0x01U;

        buf[APP_HDR_OFF + 28] = 0x7BU;
        check_eq_u32((uint32_t)app_header_check(buf, SLOT_SIZE, NULL),
                     APP_HDR_ERR_BOARD, "wrong board id");
        buf[APP_HDR_OFF + 28] = (uint8_t)APP_BOARD_ID;

        buf[APP_HDR_OFF + 30] = 0x7BU;
        check_eq_u32((uint32_t)app_header_check(buf, SLOT_SIZE, NULL),
                     APP_HDR_ERR_PROTO, "wrong proto version");
        buf[APP_HDR_OFF + 30] = (uint8_t)APP_PROTO_VERSION;

        buf[APP_HDR_OFF + 32] |= APP_HDR_FLAG_NO_BOOT;
        check_eq_u32((uint32_t)app_header_check(buf, SLOT_SIZE, &h),
                     APP_HDR_ERR_NO_BOOT, "no-boot flag");
        check(0 != (h.flags & APP_HDR_FLAG_NO_BOOT), "flags still reported");
        buf[APP_HDR_OFF + 32] &= (uint8_t)~APP_HDR_FLAG_NO_BOOT;

        check_eq_u32((uint32_t)app_header_check(buf, SLOT_SIZE, NULL), APP_HDR_OK,
                     "restored image checks out again");
    }

    /* the two CRC segments must be independent of the header contents, which is
       what lets the packaging tool and the node agree */
    {
        uint32_t low = app_image_crc_low(buf, size);
        uint32_t high = app_image_crc_high(buf, size);
        uint8_t save = buf[APP_HDR_OFF + 40];

        buf[APP_HDR_OFF + 40] ^= 0xFFU;
        check_eq_u32(app_image_crc_low(buf, size), low, "crc_low ignores the header");
        check_eq_u32(app_image_crc_high(buf, size), high, "crc_high ignores the header");
        buf[APP_HDR_OFF + 40] = save;
    }
}

static void test_boot_decision(void)
{
    boot_decide_in_t in;
    boot_decide_out_t out;

    printf("boot decision table\n");

#define DECIDE(...)                                                            \
    do {                                                                       \
        memset(&in, 0, sizeof(in));                                            \
        in.boot_slot = SLOT_NONE;                                              \
        in.trial_slot = SLOT_NONE;                                             \
        __VA_ARGS__;                                                           \
        boot_decide(&in, &out);                                                \
    } while (0)

    DECIDE();
    check(out.action == BOOT_ACT_STAY && out.slot == SLOT_NONE,
          "no valid slot -> stay in the bootloader");

    DECIDE(in.valid[SLOT_A] = 1U);
    check(out.action == BOOT_ACT_RUN && out.slot == SLOT_A, "only A valid -> run A");

    DECIDE(in.valid[SLOT_B] = 1U);
    check(out.action == BOOT_ACT_RUN && out.slot == SLOT_B, "only B valid -> run B");

    DECIDE(in.valid[SLOT_A] = 1U; in.valid[SLOT_B] = 1U; in.boot_slot = SLOT_A);
    check(out.action == BOOT_ACT_RUN && out.slot == SLOT_A, "confirmed slot A wins");

    DECIDE(in.valid[SLOT_A] = 1U; in.valid[SLOT_B] = 1U; in.boot_slot = SLOT_B);
    check(out.action == BOOT_ACT_RUN && out.slot == SLOT_B, "confirmed slot B wins");

    DECIDE(in.valid[SLOT_A] = 1U; in.valid[SLOT_B] = 1U;
           in.version[SLOT_A] = APP_VERSION_MAKE(2, 0, 0);
           in.version[SLOT_B] = APP_VERSION_MAKE(1, 0, 0));
    check(out.action == BOOT_ACT_RUN && out.slot == SLOT_A, "no history: newer A wins");

    DECIDE(in.valid[SLOT_A] = 1U; in.valid[SLOT_B] = 1U;
           in.version[SLOT_A] = APP_VERSION_MAKE(1, 0, 0);
           in.version[SLOT_B] = APP_VERSION_MAKE(2, 0, 0));
    check(out.action == BOOT_ACT_RUN && out.slot == SLOT_B, "no history: newer B wins");

    DECIDE(in.valid[SLOT_A] = 1U; in.valid[SLOT_B] = 1U; in.pingpong = 0U);
    check(out.action == BOOT_ACT_RUN && out.slot == SLOT_A, "tie: pingpong even -> A");

    DECIDE(in.valid[SLOT_A] = 1U; in.valid[SLOT_B] = 1U; in.pingpong = 1U);
    check(out.action == BOOT_ACT_RUN && out.slot == SLOT_B, "tie: pingpong odd -> B");

    DECIDE(in.valid[SLOT_A] = 1U; in.valid[SLOT_B] = 1U;
           in.trial_slot = SLOT_B; in.attempts = 0U; in.boot_slot = SLOT_A);
    check(out.action == BOOT_ACT_RUN_TRIAL && out.slot == SLOT_B && out.attempts == 1U,
          "trial boot 1/3 runs the trial slot");

    DECIDE(in.valid[SLOT_A] = 1U; in.valid[SLOT_B] = 1U;
           in.trial_slot = SLOT_B; in.attempts = 2U; in.boot_slot = SLOT_A);
    check(out.action == BOOT_ACT_RUN_TRIAL && out.attempts == BOOT_ATTEMPTS_MAX,
          "trial boot 3/3 is the last chance");

    DECIDE(in.valid[SLOT_A] = 1U; in.valid[SLOT_B] = 1U;
           in.trial_slot = SLOT_B; in.attempts = BOOT_ATTEMPTS_MAX; in.boot_slot = SLOT_A);
    check(out.action == BOOT_ACT_ROLLBACK && out.slot == SLOT_A && out.clear_trial == 1U
              && out.attempts == 0U,
          "trial exhausted -> roll back to the confirmed slot");

    DECIDE(in.valid[SLOT_A] = 1U; in.valid[SLOT_B] = 1U;
           in.trial_slot = SLOT_B; in.attempts = BOOT_ATTEMPTS_MAX;
           in.boot_slot = SLOT_NONE);
    check(out.action == BOOT_ACT_ROLLBACK && out.slot == SLOT_A,
          "trial exhausted with no history -> the other slot");

    DECIDE(in.valid[SLOT_A] = 1U; in.valid[SLOT_B] = 1U;
           in.trial_slot = SLOT_A; in.attempts = BOOT_ATTEMPTS_MAX; in.boot_slot = SLOT_B);
    check(out.action == BOOT_ACT_ROLLBACK && out.slot == SLOT_B,
          "trial A exhausted -> back to B");

    DECIDE(in.valid[SLOT_A] = 1U; in.trial_slot = SLOT_B; in.attempts = 0U);
    check(out.action == BOOT_ACT_RUN && out.slot == SLOT_A,
          "a corrupt trial slot is ignored, not attempted");

#undef DECIDE
}

static void test_flash_policy(void)
{
    uint8_t canary[16];
    uint8_t buf[16];
    uint32_t i;

    printf("flash write policy\n");
    boot_flash_test_reset();
    memset(canary, 0x5A, sizeof(canary));
    memset(buf, 0xA5, sizeof(buf));

    /* the bootloader region and the configuration page are protected by
       construction: only slot-relative offsets are expressible */
    check(boot_flash_slot_erase(SLOT_A, 0U, SLOT_SIZE) == 0, "erase all of slot A");
    check_eq_u32(boot_flash_test_erase_count(), SLOT_SIZE / FLASH_PAGE_SIZE,
                 "erased exactly the slot pages");
    check(boot_flash_slot_erase(SLOT_B, 0U, SLOT_SIZE) == 0, "erase all of slot B");
    check_eq_u32(boot_flash_test_erase_count(),
                 (SLOT_SIZE / FLASH_PAGE_SIZE) * 2U, "both slots for 96 pages total");

    check(boot_flash_slot_erase(SLOT_A, SLOT_SIZE - FLASH_PAGE_SIZE, FLASH_PAGE_SIZE * 2U)
              != 0, "an erase that would leave the slot is refused");
    check(boot_flash_slot_erase(SLOT_A, FLASH_PAGE_SIZE / 2U, FLASH_PAGE_SIZE) != 0,
          "an unaligned erase is refused");
    check(boot_flash_slot_erase(7U, 0U, FLASH_PAGE_SIZE) != 0, "an unknown slot is refused");
    check(boot_flash_slot_erase(SLOT_NONE, 0U, 0U) != 0, "a zero-length erase is refused");
    check(boot_flash_slot_program(SLOT_A, 2U, buf, 16U) != 0,
          "an unaligned program is refused");
    check(boot_flash_slot_program(SLOT_A, 0U, buf, 6U) != 0,
          "a program that is not a whole number of words is refused");
    check(boot_flash_slot_program(SLOT_A, SLOT_SIZE - 8U, buf, 16U) != 0,
          "a program that would leave the slot is refused");

    /* the boot region (0x08000000..0x08007FFF) and the config page must still be
       erased: nothing above is able to reach them */
    boot_flash_read(BOOT_BASE, canary, sizeof(canary));
    for (i = 0U; i < sizeof(canary); i++) {
        check(canary[i] == 0xFFU, "byte %u of the bootloader region untouched", (unsigned)i);
    }
    boot_flash_read(CFG_FLASH_ADDR, canary, sizeof(canary));
    for (i = 0U; i < sizeof(canary); i++) {
        check(canary[i] == 0xFFU, "byte %u of the configuration page untouched", (unsigned)i);
    }

    /* programming works, and a second program over live data is refused like the
       real FMC refuses it */
    check(boot_flash_slot_program(SLOT_A, 0x100U, buf, 16U) == 0, "program into slot A");
    boot_flash_read(SLOT_A_BASE + 0x100U, canary, sizeof(canary));
    check(memcmp(canary, buf, sizeof(buf)) == 0, "read back matches");
    memset(buf, 0xFF, sizeof(buf));
    check(boot_flash_slot_program(SLOT_A, 0x100U, buf, 16U) != 0,
          "programming 1-bits over erased-away bits is refused");
}

/* Hook recorders: the state machine must tell the integrator about commits and
   reboots instead of touching the configuration page itself. */
static int      s_commit_calls;
static uint8_t  s_commit_slot;
static int      s_reboot_calls;
static int      s_block_calls;

static void hook_commit(uint8_t slot)
{
    s_commit_calls++;
    s_commit_slot = slot;
    /* chain into the production behaviour, which is what actually records the
       trial slot and erases/programs the configuration page */
    if (NULL != boot_regs_hooks()->on_commit) {
        boot_regs_hooks()->on_commit(slot);
    }
}

static void hook_reboot(void)
{
    s_reboot_calls++;
    if (NULL != boot_regs_hooks()->on_reboot) {
        boot_regs_hooks()->on_reboot();
    }
}

static void hook_block(uint32_t offset, uint8_t len, uint16_t seq, int retransmit)
{
    (void)offset;
    (void)len;
    (void)seq;
    (void)retransmit;
    s_block_calls++;
}

static const upg_hooks_t s_test_hooks = { hook_commit, hook_reboot, hook_block };

static void boot_reset(uint8_t run_slot, uint8_t other_bootable)
{
    upg_ctx_t ctx;

    boot_flash_test_reset();
    g_now_ms = 1000U;
    s_commit_calls = 0;
    s_reboot_calls = 0;
    s_block_calls = 0;
    boot_regs_init();
    upg_set_hooks(&s_test_hooks);
    ctx.run_slot = run_slot;
    ctx.other_bootable = other_bootable;
    upg_set_context(&ctx);
}

static void test_upgrade_session_rules(void)
{
    static uint8_t image[0x800];
    uint8_t tmp[4];
    uint8_t bad;

    printf("upgrade session rules\n");

    /* BEGIN without the magic word */
    boot_reset(SLOT_A, 1U);
    build_upgrade_image(image, SLOT_B, sizeof(image), APP_VERSION);
    upg_open_session(SLOT_B, sizeof(image), app_image_crc_combined(image, sizeof(image)),
                     APP_VERSION);
    memset(tmp, 0, 2U);
    upg_put(U_MAGIC_L, tmp, 2U);              /* clear the magic again */
    upg_cmd(UPG_CMD_BEGIN);
    check_eq_u32(upg_read(U_ERRCODE), UPG_ERR_MAGIC, "BEGIN without magic -> ERR_MAGIC");
    check_eq_u32(upg_status(), UPG_ST_ERROR | UPG_ERR_MAGIC, "status is error|magic");

    /* the slot the bootloader is about to run is protected while it is the only
       bootable image */
    boot_reset(SLOT_A, 0U);
    upg_open_session(SLOT_A, sizeof(image), app_image_crc_combined(image, sizeof(image)),
                     APP_VERSION);
    upg_cmd(UPG_CMD_BEGIN);
    check_eq_u32(upg_read(U_ERRCODE), UPG_ERR_TARGET, "targeting the last good slot");

    /* ... but allowed when the other slot can still boot */
    boot_reset(SLOT_A, 1U);
    upg_open_session(SLOT_A, sizeof(image), app_image_crc_combined(image, sizeof(image)),
                     APP_VERSION);
    upg_cmd(UPG_CMD_BEGIN);
    check_eq_u32(upg_read(U_ERRCODE), UPG_ERR_NONE, "targeting A is fine when B is valid");
    check_eq_u32(upg_status(), UPG_ST_ERASED, "status = erased");
    check_eq_u32(boot_flash_test_erase_count(),
                 (sizeof(image) + FLASH_PAGE_SIZE - 1U) / FLASH_PAGE_SIZE,
                 "erased ceil(imglen/page) pages");

    /* invalid lengths */
    boot_reset(SLOT_A, 1U);
    upg_open_session(SLOT_B, 0x100U, 0U, APP_VERSION);
    upg_cmd(UPG_CMD_BEGIN);
    check_eq_u32(upg_read(U_ERRCODE), UPG_ERR_LENGTH, "imglen below the minimum");

    boot_reset(SLOT_A, 1U);
    upg_open_session(SLOT_B, (uint32_t)SLOT_SIZE + 4U, 0U, APP_VERSION);
    upg_cmd(UPG_CMD_BEGIN);
    check_eq_u32(upg_read(U_ERRCODE), UPG_ERR_LENGTH, "imglen above the slot");

    boot_reset(SLOT_A, 1U);
    upg_open_session(SLOT_B, 0x801U, 0U, APP_VERSION);
    upg_cmd(UPG_CMD_BEGIN);
    check_eq_u32(upg_read(U_ERRCODE), UPG_ERR_LENGTH, "unaligned imglen");

    /* a bad target register */
    boot_reset(SLOT_A, 1U);
    upg_open_session(SLOT_B, 0x800U, 0U, APP_VERSION);
    bad = 5U;
    upg_put(U_TARGET, &bad, 1U);
    upg_cmd(UPG_CMD_BEGIN);
    check_eq_u32(upg_read(U_ERRCODE), UPG_ERR_TARGET, "target 5 -> ERR_TARGET");

    /* data before BEGIN */
    boot_reset(SLOT_A, 1U);
    upg_open_session(SLOT_B, sizeof(image), 0U, APP_VERSION);
    check(0U == upg_send_block(0U, 0U, image, 16U), "block frame is well formed");
    check_eq_u32(upg_read(U_ERRCODE), UPG_ERR_STATE, "block before BEGIN -> ERR_STATE");

    /* the window is bounds-checked */
    check_eq_u32(upg_write(UPGRADE_WIN_LEN - 2U, image, 8U, g_now_ms), DXL_ERR_RANGE,
                 "write past the window");
    check_eq_u32(upg_write(U_DATA_OFF, image, 6U, g_now_ms), DXL_ERR_LENGTH,
                 "a 6-byte block is not a whole number of words");
    check_eq_u32(upg_write(U_DATA_OFF, image, 0U, g_now_ms), DXL_ERR_LENGTH,
                 "a zero-length write");
}

static void test_upgrade_blocks(void)
{
    static uint8_t image[0x1000];
    uint8_t buf[16];
    uint8_t *flash;

    printf("upgrade block bookkeeping\n");

    boot_reset(SLOT_A, 1U);
    build_upgrade_image(image, SLOT_B, sizeof(image), APP_VERSION);
    upg_open_session(SLOT_B, sizeof(image), app_image_crc_combined(image, sizeof(image)),
                     APP_VERSION);
    upg_cmd(UPG_CMD_BEGIN);

    flash = boot_flash_test_base();

    /* a wrong block CRC must not touch flash */
    memcpy(buf, &image[0], 16U);
    check(0U == upg_send_block_badcrc(0U, 0U, buf, 16U), "bad-CRC block frame accepted");
    check_eq_u32(upg_read(U_ERRCODE), UPG_ERR_CRC, "bad block CRC -> ERR_CRC");
    check_eq_u32(upg_read(U_ACK_SEQ_L) | ((uint32_t)upg_read(U_ACK_SEQ_H) << 8),
                 0xFFFFU, "ACK_SEQ untouched after a bad block");
    check_eq_u32(flash[SLOT_B_BASE + 3U - FLASH_BASE_ADDR], 0xFFU,
                 "flash untouched by the bad block");

    /* a good block, then an out-of-order one */
    check(0U == upg_send_block(0U, 0U, &image[0], 16U), "block 0 accepted");
    check_eq_u32(upg_status(), UPG_ST_BLOCK_OK, "status = block ok");
    check_eq_u32(upg_read(U_ERRCODE), UPG_ERR_NONE, "no error");
    check_eq_u32(upg_bytes_written(), 16U, "bytes_written = 16");
    check_eq_u32(upg_blocks(), 1U, "blocks = 1");
    check(memcmp(&flash[SLOT_B_BASE - FLASH_BASE_ADDR], &image[0], 16U) == 0,
          "block 0 landed in slot B");

    check(0U == upg_send_block(5U, 0x240U, &image[0x240], 16U),
          "out-of-order frame accepted");
    check_eq_u32(upg_read(U_ERRCODE), UPG_ERR_RANGE, "sequence jump -> ERR_RANGE");
    check_eq_u32(upg_read(U_ACK_SEQ_L) | ((uint32_t)upg_read(U_ACK_SEQ_H) << 8), 0U,
                 "ACK_SEQ still 0");
    check(memcmp(&flash[SLOT_B_BASE + 0x240U - FLASH_BASE_ADDR], &image[0x240], 16U) != 0,
          "the out-of-order block was not programmed");
    check(0xFFU == flash[SLOT_B_BASE + 0x240U - FLASH_BASE_ADDR],
          "the out-of-order block left the slot erased");

    /* a retransmit of the accepted block is idempotent */
    check(0U == upg_send_block(0U, 0U, &image[0], 16U), "retransmit accepted");
    check_eq_u32(upg_retransmits(), 1U, "retransmit counted");
    check_eq_u32(upg_bytes_written(), 16U, "retransmit does not double count bytes");
    check_eq_u32(upg_blocks(), 2U, "retransmit counted as a block attempt");

    /* an offset that would leave the image */
    check(0U == upg_send_block(1U, 0x2000U, &image[16], 16U), "out-of-range frame accepted");
    check_eq_u32(upg_read(U_ERRCODE), UPG_ERR_RANGE, "offset past imglen -> ERR_RANGE");

    /* a block whose declared length disagrees with the window write */
    {
        uint8_t win[UPGRADE_WIN_LEN];
        uint16_t crc = crc16_dxl(&image[16], 8U);
        memset(win, 0, sizeof(win));
        win[U_SEQ_L] = 1U;
        win[2] = 16U;              /* offset 16 */
        win[U_BLKLEN] = 8U;
        win[U_BLKCRC_L] = (uint8_t)(crc & 0xFFU);
        win[U_BLKCRC_H] = (uint8_t)(crc >> 8);
        upg_put(U_SEQ_L, &win[U_SEQ_L], (uint8_t)(U_BLKCRC_H - U_SEQ_L + 1U));
        check(0U == upg_write(U_DATA_OFF, &image[16], 16U, g_now_ms), "16-byte data write");
        check_eq_u32(upg_read(U_ERRCODE), UPG_ERR_LENGTH,
                     "data length != declared BLKLEN -> ERR_LENGTH");
    }

    /* the whole window in one frame works too (control + data together) */
    boot_reset(SLOT_A, 1U);
    upg_open_session(SLOT_B, sizeof(image), app_image_crc_combined(image, sizeof(image)),
                     APP_VERSION);
    upg_cmd(UPG_CMD_BEGIN);
    {
        uint8_t win[UPGRADE_WIN_LEN];
        uint16_t crc = crc16_dxl(&image[0], 16U);
        memset(win, 0, sizeof(win));
        win[U_MAGIC_L] = (uint8_t)(UPGRADE_MAGIC & 0xFFU);
        win[U_MAGIC_H] = (uint8_t)(UPGRADE_MAGIC >> 8);
        win[U_SEQ_L] = 0U;
        win[U_OFF_0] = 0U;
        win[U_BLKLEN] = 16U;
        win[U_BLKCRC_L] = (uint8_t)(crc & 0xFFU);
        win[U_BLKCRC_H] = (uint8_t)(crc >> 8);
        memcpy(&win[U_DATA_OFF], &image[0], 16U);
        check(0U == upg_write(0U, win, UPGRADE_WIN_LEN, g_now_ms),
              "whole-window write accepted");
        check_eq_u32(upg_status(), UPG_ST_BLOCK_OK, "status = block ok after whole-window write");
        check_eq_u32(upg_bytes_written(), 16U, "the whole-window block was programmed");
    }
}

static void test_upgrade_end(void)
{
    static uint8_t image[0x1000];
    uint8_t win[UPGRADE_WIN_LEN];
    uint32_t nodecrc;

    printf("upgrade end: verification and commit\n");

    /* a complete session */
    boot_reset(SLOT_A, 1U);
    build_upgrade_image(image, SLOT_B, sizeof(image), APP_VERSION_MAKE(3, 2, 1));
    upg_target_of_current_test = SLOT_B;
    upg_open_session(SLOT_B, sizeof(image), app_image_crc_combined(image, sizeof(image)),
                     APP_VERSION_MAKE(3, 2, 1));
    upg_cmd(UPG_CMD_BEGIN);
    upg_send_image(image, sizeof(image));
    check_eq_u32(upg_bytes_written(), sizeof(image), "all bytes written");
    check_eq_u32(upg_blocks(), sizeof(image) / 16U, "one block per 16 bytes");
    upg_cmd(UPG_CMD_END);
    check_u32_fmt(upg_status(), UPG_ST_COMMITTED, "status = committed (err %u)",
                  (unsigned)upg_read(U_ERRCODE));
    check_eq_u32(s_commit_calls, 1, "on_commit called once");
    check_eq_u32(s_commit_slot, SLOT_B, "on_commit for slot B");
    nodecrc = (uint32_t)upg_read(U_NODE_CRC_0) | ((uint32_t)upg_read(U_NODE_CRC_1) << 8)
              | ((uint32_t)upg_read(U_NODE_CRC_2) << 16)
              | ((uint32_t)upg_read(U_NODE_CRC_3) << 24);
    check_eq_u32(nodecrc, app_image_crc_combined(image, sizeof(image)),
                 "node CRC equals the host CRC");

    /* the image in flash must now be a bootable image with the right version */
    {
        app_header_t h;
        int rc = app_header_check(boot_flash_slot_ptr(SLOT_B), SLOT_SIZE, &h);
        check(rc == APP_HDR_OK, "the committed slot validates: %s", app_hdr_strerror(rc));
        check_eq_u32(h.image_version, APP_VERSION_MAKE(3, 2, 1), "version in the header");
        check_eq_u32(h.image_size, sizeof(image), "size in the header");
        check(0 != (h.flags & APP_HDR_FLAG_TRIAL), "the package was marked trial");
    }

    /* a tampered image must fail verification and must NOT be committed */
    boot_reset(SLOT_A, 1U);
    build_upgrade_image(image, SLOT_B, sizeof(image), APP_VERSION);
    upg_open_session(SLOT_B, sizeof(image), app_image_crc_combined(image, sizeof(image)),
                     APP_VERSION);
    upg_cmd(UPG_CMD_BEGIN);
    upg_send_image(image, sizeof(image));
    {
        /* smash one byte in the middle of the slot, as a bit rot / interrupted
           write would */
        uint8_t *flash = boot_flash_test_base();
        flash[SLOT_B_BASE + 0x800U - FLASH_BASE_ADDR] ^= 0x01U;
    }
    upg_cmd(UPG_CMD_END);
    check_eq_u32(upg_status(), UPG_ST_ERROR | UPG_ERR_VERIFY, "tampered image -> ERR_VERIFY");
    check_eq_u32(s_commit_calls, 0, "nothing committed");
    check_eq_u32(upg_read(U_NODE_CRC_0) | ((uint32_t)upg_read(U_NODE_CRC_1) << 8)
                     | ((uint32_t)upg_read(U_NODE_CRC_2) << 16)
                     | ((uint32_t)upg_read(U_NODE_CRC_3) << 24),
                 app_image_crc_combined(boot_flash_slot_ptr(SLOT_B), sizeof(image)),
                 "the node reports the CRC it actually computed");

    /* END without BEGIN */
    boot_reset(SLOT_A, 1U);
    upg_open_session(SLOT_B, sizeof(image), 0U, APP_VERSION);
    upg_cmd(UPG_CMD_END);
    check_eq_u32(upg_read(U_ERRCODE), UPG_ERR_STATE, "END before BEGIN -> ERR_STATE");

    /* a host CRC that disagrees with the image */
    boot_reset(SLOT_A, 1U);
    build_upgrade_image(image, SLOT_B, sizeof(image), APP_VERSION);
    upg_open_session(SLOT_B, sizeof(image), 0xDEADBEEFU, APP_VERSION);
    upg_cmd(UPG_CMD_BEGIN);
    upg_send_image(image, sizeof(image));
    upg_cmd(UPG_CMD_END);
    check_eq_u32(upg_read(U_ERRCODE), UPG_ERR_VERIFY, "wrong UPG_IMGCRC -> ERR_VERIFY");
    check_eq_u32(s_commit_calls, 0, "nothing committed after a CRC mismatch");

    /* version cross-check between UPG_VER and the header */
    boot_reset(SLOT_A, 1U);
    build_upgrade_image(image, SLOT_B, sizeof(image), APP_VERSION_MAKE(1, 0, 0));
    upg_open_session(SLOT_B, sizeof(image), app_image_crc_combined(image, sizeof(image)),
                     APP_VERSION_MAKE(9, 9, 9));
    upg_cmd(UPG_CMD_BEGIN);
    upg_send_image(image, sizeof(image));
    upg_cmd(UPG_CMD_END);
    check_eq_u32(upg_read(U_ERRCODE), UPG_ERR_VERIFY,
                 "UPG_VER disagreeing with the header -> ERR_VERIFY");

    /* an image marked no-boot must be refused even with a correct CRC */
    boot_reset(SLOT_A, 1U);
    build_upgrade_image(image, SLOT_B, sizeof(image), APP_VERSION);
    {
        app_header_t h;
        memcpy(&h, &image[APP_HDR_OFF], sizeof(h));
        h.flags |= APP_HDR_FLAG_NO_BOOT;
        memcpy(&image[APP_HDR_OFF], &h, sizeof(h));
    }
    upg_open_session(SLOT_B, sizeof(image), app_image_crc_combined(image, sizeof(image)),
                     APP_VERSION);
    upg_cmd(UPG_CMD_BEGIN);
    upg_send_image(image, sizeof(image));
    upg_cmd(UPG_CMD_END);
    check_eq_u32(upg_read(U_ERRCODE), UPG_ERR_VERIFY, "no-boot image -> ERR_VERIFY");
    check_eq_u32(s_commit_calls, 0, "no-boot image not committed");

    /* abort resets the session */
    boot_reset(SLOT_A, 1U);
    build_upgrade_image(image, SLOT_B, sizeof(image), APP_VERSION);
    upg_open_session(SLOT_B, sizeof(image), app_image_crc_combined(image, sizeof(image)),
                     APP_VERSION);
    upg_cmd(UPG_CMD_BEGIN);
    check(0U == upg_send_block(0U, 0U, image, 16U), "block accepted before abort");
    upg_cmd(UPG_CMD_ABORT);
    check_eq_u32(upg_status(), UPG_ST_IDLE, "status idle after abort");
    check_eq_u32(upg_read(U_ACK_SEQ_L) | ((uint32_t)upg_read(U_ACK_SEQ_H) << 8), 0U,
                 "ACK_SEQ reset");
    check_eq_u32(upg_bytes_written(), 0U, "byte counter reset");
    check(0U == upg_send_block(0U, 0U, image, 16U), "well-formed frame after abort");
    check_eq_u32(upg_read(U_ERRCODE), UPG_ERR_STATE, "blocks after abort -> ERR_STATE");

    /* confirm is an application command: the bootloader refuses it */
    boot_reset(SLOT_A, 1U);
    upg_cmd(UPG_CMD_CONFIRM);
    check_eq_u32(upg_read(U_ERRCODE), UPG_ERR_STATE, "UPG_CONFIRM in boot mode -> ERR_STATE");

    /* reboot is acknowledged and asks for a reset through the hook */
    boot_reset(SLOT_A, 1U);
    check(0 == upg_reboot_pending(g_now_ms), "no reboot pending yet");
    upg_cmd(UPG_CMD_REBOOT);
    check_eq_u32(s_reboot_calls, 1, "on_reboot called");
    check(0 == upg_reboot_pending(g_now_ms), "reboot waits for the response to leave");
    g_now_ms += 100U;
    check(1 == upg_reboot_pending(g_now_ms), "reboot pending after the delay");

    /* readback: the host can re-read what it wrote */
    boot_reset(SLOT_A, 1U);
    build_upgrade_image(image, SLOT_B, sizeof(image), APP_VERSION);
    upg_open_session(SLOT_B, sizeof(image), app_image_crc_combined(image, sizeof(image)),
                     APP_VERSION);
    upg_cmd(UPG_CMD_BEGIN);
    upg_send_image(image, sizeof(image));
    memset(win, 0, sizeof(win));
    win[U_OFF_0] = 0x40U;
    win[U_OFF_1] = 0x02U;          /* offset 0x240 */
    win[U_BLKLEN] = 16U;
    upg_put(U_OFF_0, &win[U_OFF_0], 5U);
    upg_cmd(UPG_CMD_READBACK);
    {
        uint8_t back[16];
        uint8_t i;
        for (i = 0U; i < 16U; i++) {
            back[i] = upg_read((uint8_t)(U_DATA_OFF + i));
        }
        check(memcmp(back, &image[0x240], 16U) == 0, "readback returns the image bytes");
    }
}

static void test_cfg_blob_and_trial(void)
{
    cfg_blob_t blob;
    cfg_blob_t loaded;
    boot_decide_in_t in;
    boot_decide_out_t out;

    printf("configuration page and the trial/rollback loop\n");

    boot_reset(SLOT_A, 1U);

    /* default state is "nothing confirmed, nothing on trial" */
    blob = *boot_regs_blob();
    check_eq_u32(blob.boot.boot_slot, SLOT_NONE, "default boot_slot");
    check_eq_u32(blob.boot.trial_slot, SLOT_NONE, "default trial_slot");

    /* commit slot B through the upgrade state machine, then re-load the page */
    {
        static uint8_t image[0x800];
        build_upgrade_image(image, SLOT_B, sizeof(image), APP_VERSION);
        upg_open_session(SLOT_B, sizeof(image), app_image_crc_combined(image, sizeof(image)),
                         APP_VERSION);
        upg_cmd(UPG_CMD_BEGIN);
        upg_send_image(image, sizeof(image));
        upg_cmd(UPG_CMD_END);
        check_eq_u32(upg_status(), UPG_ST_COMMITTED, "committed");
    }
    blob = *boot_regs_blob();
    check_eq_u32(blob.boot.trial_slot, SLOT_B, "B is on trial");
    check_eq_u32(blob.boot.attempts, 0U, "attempts reset");
    check_eq_u32(blob.boot.pingpong, 1U, "pingpong advanced");
    check_eq_u32(blob.boot.boot_stay, 0U, "boot_stay cleared on commit");

    {
        uint8_t uid[UID_WIN_LEN];
        check_eq_u32(dev_read(PROTO_DXL, UID_WIN_ADDR, UID_WIN_LEN, uid), 0U,
                     "read the identity window after the commit");
        check_eq_u32(uid[UID_INFO_VERSION], UID_INFO_VERSION_1, "boot state published");
        check_eq_u32(uid[UID_TRIAL_SLOT], SLOT_B, "identity window shows the trial slot");
        check_eq_u32(uid[UID_BOOT_SLOT], SLOT_NONE, "nothing confirmed yet");
        check((uid[UID_FLAGS] & APP_HDR_FLAG_TRIAL) != 0U,
              "the effective trial flag is set");
    }

    check(1 == cfg_blob_load(&loaded), "the page still loads");
    check(0 == memcmp(&loaded.boot, &blob.boot, sizeof(blob.boot)),
          "boot state survived the erase/program cycle");
    check(0 == memcmp(&loaded.cfg, &blob.cfg, sizeof(blob.cfg)),
          "device config survived too");

    /* the trial loop: B boots, fails to confirm, and rolls back to A */
    memset(&in, 0, sizeof(in));
    in.valid[SLOT_A] = 1U;
    in.valid[SLOT_B] = 1U;
    in.version[SLOT_A] = APP_VERSION;
    in.version[SLOT_B] = APP_VERSION;
    in.boot_slot = blob.boot.boot_slot;
    in.trial_slot = blob.boot.trial_slot;
    in.attempts = blob.boot.attempts;
    in.pingpong = blob.boot.pingpong;

    boot_decide(&in, &out);
    check(out.action == BOOT_ACT_RUN_TRIAL && out.slot == SLOT_B, "attempt 1 boots B");
    in.attempts = out.attempts;

    boot_decide(&in, &out);
    check(out.action == BOOT_ACT_RUN_TRIAL && out.attempts == 2U, "attempt 2 boots B");
    in.attempts = out.attempts;

    boot_decide(&in, &out);
    check(out.action == BOOT_ACT_RUN_TRIAL && out.attempts == 3U, "attempt 3 boots B");
    in.attempts = out.attempts;

    boot_decide(&in, &out);
    check(out.action == BOOT_ACT_ROLLBACK && out.slot == SLOT_A && out.clear_trial,
          "attempt 4 rolls back to A");

    /* nothing on trial and nothing confirmed: the newer image wins */
    in.trial_slot = SLOT_NONE;
    in.attempts = 0U;
    in.boot_slot = SLOT_NONE;
    in.version[SLOT_A] = APP_VERSION;
    in.version[SLOT_B] = APP_VERSION_MAKE(FW_VERSION_MAJOR, FW_VERSION_MINOR + 1U, 0U);
    boot_decide(&in, &out);
    check(out.action == BOOT_ACT_RUN && out.slot == SLOT_B, "newer image wins");

    /* a v1 blob (the layout before the boot state was added) must be ignored */
    boot_reset(SLOT_A, 1U);
    {
        uint8_t v1[64];
        uint16_t crc;
        memset(v1, 0, sizeof(v1));
        /* magic "AIM1", version 1, len sizeof(dev_cfg_t), then a crc16 */
        v1[0] = 0x41U; v1[1] = 0x49U; v1[2] = 0x4DU; v1[3] = 0x31U;
        v1[4] = 1U; v1[5] = 0U;
        v1[6] = (uint8_t)sizeof(dev_cfg_t); v1[7] = 0U;
        crc = crc16_dxl(v1, 8U);
        v1[8] = (uint8_t)(crc & 0xFFU);
        v1[9] = (uint8_t)(crc >> 8);
        boot_flash_test_load(CFG_FLASH_ADDR, v1, sizeof(v1));
    }
    check(0 == cfg_blob_load(&loaded), "a v1 blob is rejected");
    boot_regs_init();
    check_eq_u32(boot_regs_blob()->boot.boot_slot, SLOT_NONE, "defaults after a v1 blob");
    check_eq_u32(boot_regs_blob()->cfg.dxl_id, DXL_ID_DEFAULT, "default dxl id after v1");
}

static void test_identity_window(void)
{
    uint8_t buf[UID_WIN_LEN];
    uint8_t tmp;

    printf("identity window and out-of-range writes\n");

    boot_reset(SLOT_A, 1U);
    load_slot(SLOT_B, APP_VERSION_MAKE(4, 5, 6), 0x1000U, 0x11U);
    {
        boot_decide_in_t in;
        boot_decide_out_t out;
        memset(&in, 0, sizeof(in));
        in.valid[SLOT_B] = 1U;
        in.version[SLOT_B] = APP_VERSION_MAKE(4, 5, 6);
        in.boot_slot = SLOT_NONE;
        in.trial_slot = SLOT_NONE;
        boot_decide(&in, &out);
        check(out.slot == SLOT_B, "decision picks B");
        boot_regs_set_decision(&in, &out);
        boot_regs_set_slot_info(out.slot, in.version[out.slot], 0x0002U, 0xCAFEBABEU);
    }

    check_eq_u32(dev_read(PROTO_DXL, UID_WIN_ADDR, UID_WIN_LEN, buf), 0U,
                 "read the identity window");
    check_eq_u32((uint32_t)(buf[UID_MAGIC_L] | ((uint16_t)buf[UID_MAGIC_H] << 8)),
                 UID_MAGIC, "identity magic");
    check_eq_u32(buf[UID_MODE], UID_MODE_BOOT, "mode = bootloader");
    check_eq_u32(buf[UID_SLOT_RUN], SLOT_B, "slot = B");
    check_eq_u32(buf[UID_FLAGS], 0x02U, "flags published");
    check_eq_u32((uint32_t)(buf[UID_VER_L] | ((uint16_t)buf[UID_VER_H] << 8)),
                 APP_VERSION_MAKE(4, 5, 6), "version published");
    check_eq_u32((uint32_t)buf[UID_IMGCRC_0] | ((uint32_t)buf[UID_IMGCRC_1] << 8)
                     | ((uint32_t)buf[UID_IMGCRC_2] << 16)
                     | ((uint32_t)buf[UID_IMGCRC_3] << 24),
                 0xCAFEBABEU, "image CRC published");
    check_eq_u32(buf[UID_INFO_VERSION], UID_INFO_VERSION_1, "boot state is published");
    check_eq_u32(buf[UID_BOOT_SLOT], SLOT_NONE, "confirmed slot from the config page");
    check_eq_u32(buf[UID_TRIAL_SLOT], SLOT_NONE, "trial slot from the config page");
    check_eq_u32(buf[UID_ATTEMPTS], 0U, "trial attempts from the config page");
    check_eq_u32(buf[UID_BOOT_STAY], 0U, "boot_stay from the config page");

    /* PING must report the bootloader model, not the application's */
    {
        uint8_t resp[DXL_MAX_RESP];
        uint8_t frame[16];
        uint16_t flen;

        flen = dxl_frame(200U, DXL_INST_PING, NULL, 0U, frame);
        check(dxl_feed(frame, flen, resp, sizeof(resp)) > 0U, "PING is answered");
        check_eq_u32(resp[8], 0U, "no error byte");
        check_eq_u32((uint32_t)(resp[9] | ((uint16_t)resp[10] << 8)), BOOT_MODEL_NUMBER,
                     "boot PING model number");
        check(resp[11] == 0U, "boot PING firmware version byte is 0");
        check(BOOT_MODEL_NUMBER != DXL_MODEL_NUMBER, "boot model differs from the app's");
    }
    /* ... and the FeeTech model lives in registers 3..4 */
    {
        uint8_t info[4];
        check_eq_u32((uint32_t)dev_read(PROTO_FEE, 3U, 4U, info), 0U, "read FeeTech model");
        check_eq_u32((uint32_t)(info[0] | ((uint16_t)info[1] << 8)), BOOT_FEE_MODEL_NUMBER,
                     "boot FeeTech model");
        check((uint32_t)(info[0] | ((uint16_t)info[1] << 8)) != FEE_MODEL_NUMBER,
              "boot FeeTech model differs from the app's");
    }

    /* the identity window is read-only, the session window is not */
    tmp = 0xA5U;
    check_eq_u32(dev_write(PROTO_DXL, UID_WIN_ADDR, 1U, &tmp), 0U,
                 "a write to the identity window is accepted and ignored");
    check_eq_u32(dev_read(PROTO_DXL, UID_WIN_ADDR, 1U, buf), 0U, "re-read");
    check_eq_u32(buf[0], (uint8_t)(UID_MAGIC & 0xFFU), "identity window unchanged");

    check_eq_u32(dev_write(PROTO_DXL, 0U, 4U, &tmp), DXL_ERR_RANGE,
                 "a write outside the two windows is refused");
    check_eq_u32(dev_write(PROTO_DXL, UPGRADE_WIN_ADDR, UPGRADE_WIN_LEN + 1U, &tmp),
                 DXL_ERR_RANGE, "a write past the session window is refused");
    check_eq_u32(dev_write(PROTO_DXL, U_DATA_OFF, 4U, &tmp), DXL_ERR_RANGE,
                 "dev_write takes absolute addresses, not window offsets");
}

static void test_upgrade_over_the_protocols(void)
{
    static uint8_t image[0x1000];
    uint8_t resp[DXL_MAX_RESP];
    uint8_t params[64];

    printf("full upgrade through real DXL and FeeTech frames\n");

    /* --- Dynamixel --- */
    boot_reset(SLOT_A, 1U);
    dxl_slave_init(&s_dxl_slave);
    build_upgrade_image(image, SLOT_B, sizeof(image), APP_VERSION_MAKE(1, 1, 0));
    upgrade_over_dxl(image, sizeof(image));
    check_u32_fmt(upg_status(), UPG_ST_COMMITTED, "DXL upgrade committed (err %u)",
                  (unsigned)upg_read(U_ERRCODE));
    check_eq_u32(s_commit_slot, SLOT_B, "DXL upgrade committed to slot B");
    check(APP_HDR_OK == app_header_check(boot_flash_slot_ptr(SLOT_B), SLOT_SIZE, NULL),
          "DXL upgrade produced a valid header");

    /* --- FeeTech, including the byte-level retry the protocol needs --- */
    boot_reset(SLOT_A, 1U);
    fee_slave_init(&s_fee_slave);
    {
        uint32_t off = 0U;
        uint16_t seq = 0U;
        uint8_t tmp[8];
        uint32_t crc;

        crc = app_image_crc_combined(image, sizeof(image));
        /* open the session */
        tmp[0] = (uint8_t)(UPGRADE_MAGIC & 0xFFU);
        tmp[1] = (uint8_t)(UPGRADE_MAGIC >> 8);
        check(fee_write(tmp, 2U, UPGRADE_WIN_ADDR + U_MAGIC_L, resp, sizeof(resp)) > 0U, "fee magic ack");
        tmp[0] = SLOT_B;
        check(fee_write(tmp, 1U, UPGRADE_WIN_ADDR + U_TARGET, resp, sizeof(resp)) > 0U, "fee target ack");
        memcpy(tmp, &(uint32_t){ (uint32_t)sizeof(image) }, 4U);
        check(fee_write(tmp, 4U, UPGRADE_WIN_ADDR + U_IMGLEN_0, resp, sizeof(resp)) > 0U, "fee imglen ack");
        memcpy(tmp, &crc, 4U);
        check(fee_write(tmp, 4U, UPGRADE_WIN_ADDR + U_IMGCRC_0, resp, sizeof(resp)) > 0U, "fee imgcrc ack");
        tmp[0] = (uint8_t)(APP_VERSION_MAKE(1, 1, 0) & 0xFFU);
        tmp[1] = (uint8_t)(APP_VERSION_MAKE(1, 1, 0) >> 8);
        check(fee_write(tmp, 2U, UPGRADE_WIN_ADDR + U_VER_L, resp, sizeof(resp)) > 0U, "fee version ack");
        tmp[0] = UPG_CMD_BEGIN;
        check(fee_write(tmp, 1U, UPGRADE_WIN_ADDR + U_CMD, resp, sizeof(resp)) > 0U, "fee begin ack");
        check_eq_u32(upg_status(), UPG_ST_ERASED, "fee BEGIN erased the slot");

        while (off < sizeof(image)) {
            uint32_t left = sizeof(image) - off;
            uint8_t len = (uint8_t)((left >= 16U) ? 16U : left);
            uint16_t blkcrc = crc16_dxl(&image[off], len);

            params[0] = (uint8_t)(seq & 0xFFU);
            params[1] = (uint8_t)(seq >> 8);
            params[2] = (uint8_t)(off & 0xFFU);
            params[3] = (uint8_t)((off >> 8) & 0xFFU);
            params[4] = (uint8_t)((off >> 16) & 0xFFU);
            params[5] = (uint8_t)((off >> 24) & 0xFFU);
            params[6] = len;
            params[7] = (uint8_t)(blkcrc & 0xFFU);
            params[8] = (uint8_t)(blkcrc >> 8);
            check(fee_write(params, 9U, UPGRADE_WIN_ADDR + U_SEQ_L, resp, sizeof(resp)) > 0U,
                  "fee block control ack");
            check(fee_write(&image[off], len, UPGRADE_WIN_ADDR + U_DATA_OFF, resp, sizeof(resp)) > 0U,
                  "fee data ack (seq %u)", (unsigned)seq);
            check_u32_fmt(upg_status(), UPG_ST_BLOCK_OK, "fee block %u ok (err %u)",
                          (unsigned)seq, (unsigned)upg_read(U_ERRCODE));
            off += len;
            seq++;
        }
        tmp[0] = UPG_CMD_END;
        check(fee_write(tmp, 1U, UPGRADE_WIN_ADDR + U_CMD, resp, sizeof(resp)) > 0U,
              "fee end ack");
        check_u32_fmt(upg_status(), UPG_ST_COMMITTED, "fee upgrade committed (err %u)",
                      (unsigned)upg_read(U_ERRCODE));
        check(APP_HDR_OK == app_header_check(boot_flash_slot_ptr(SLOT_B), SLOT_SIZE, NULL),
              "fee upgrade produced a valid header");
    }
}

static void test_full_slot_upgrade(void)
{
    static uint8_t image[SLOT_SIZE];
    uint32_t size = SLOT_SIZE;

    printf("upgrade a full 96 KB slot image\n");

    boot_reset(SLOT_A, 0U);
    build_upgrade_image(image, SLOT_B, size, APP_VERSION_MAKE(2, 0, 0));
    upg_open_session(SLOT_B, size, app_image_crc_combined(image, size),
                     APP_VERSION_MAKE(2, 0, 0));
    upg_cmd(UPG_CMD_BEGIN);
    upg_send_image(image, size);
    upg_cmd(UPG_CMD_END);
    check_u32_fmt(upg_status(), UPG_ST_COMMITTED, "full slot committed (err %u)",
                  (unsigned)upg_read(U_ERRCODE));
    check_eq_u32(upg_blocks(), size / 16U, "one block per 16 bytes");
    /* one FMC word per 4 image bytes, plus the configuration page the commit
       rewrites (the boot state lives there) */
    check_eq_u32(boot_flash_test_program_count(),
                 size / 4U + (uint32_t)(sizeof(cfg_blob_t) / 4U),
                 "one FMC word per 4 bytes, plus the config page");
    check(0U == memcmp(boot_flash_slot_ptr(SLOT_B), image, size), "flash matches the image");
    check(APP_HDR_OK == app_header_check(boot_flash_slot_ptr(SLOT_B), SLOT_SIZE, NULL),
          "full slot validates");
    /* the other slot must be untouched */
    {
        const uint8_t *a = boot_flash_slot_ptr(SLOT_A);
        uint32_t i;
        for (i = 0U; i < SLOT_SIZE; i++) {
            if (a[i] != 0xFFU) {
                check(0, "slot A was modified at offset %u", (unsigned)i);
                break;
            }
        }
    }
}

int main(void)
{
    printf("bootloader host tests\n");

    test_app_header_layout();
    test_header_check();
    test_boot_decision();
    test_flash_policy();
    test_upgrade_session_rules();
    test_upgrade_blocks();
    test_upgrade_end();
    test_cfg_blob_and_trial();
    test_identity_window();
    test_upgrade_over_the_protocols();
    test_full_slot_upgrade();

    printf("%d checks, %d failure(s)\n", g_checks, g_failures);
    if (g_failures) {
        printf("BOOT TESTS FAILED\n");
        return 1;
    }
    printf("all bootloader tests pass\n");
    return 0;
}
