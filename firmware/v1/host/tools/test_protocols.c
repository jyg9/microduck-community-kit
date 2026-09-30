/*!
    \file    test_protocols.c
    \brief   host-side tests for the two protocol slaves and for the simulator's
             frame conventions

    The PCBA is not back yet and the SPI IMU is missing, so the protocol layer is
    verified here instead: the real dxl2.c / fee.c / imu_sim.c sources are
    compiled with gcc against a stubbed register layer, and the expected response
    packets were generated independently (python, table-driven ROBOTIS CRC) and
    cross-checked against rustypot 0.6's own test vectors in test_crc16.c.

    Build & run:
        ./run_c_tests.sh
*/

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define IMU_TO_DXL_HOST_TEST 1

#include "../../src/board.h"
#include "../../src/bus_arb.h"
#include "../../src/crc16.h"
#include "../../src/dev.h"
#include "../../src/dxl2.h"
#include "../../src/fee.h"
#include "../../src/imu.h"
#include "../../src/imu_math.h"
#include "../../src/imu_sim.h"
#include "../../src/lsm6dsv16x.h"
#include "../../src/stuffing.h"
#include "../../src/telem_pack.h"

/* ── stubbed register layer ─────────────────────────────────────────────── */

static uint8_t   g_dxl[REG_SPACE_SIZE];
static uint8_t   g_fee[REG_SPACE_SIZE];
static dev_cfg_t g_cfg;
static int       g_write_calls;
static uint16_t  g_write_addr;
static uint16_t  g_write_len;
static uint8_t   g_write_data[REG_SPACE_SIZE];
static uint8_t   g_noted_err;

const uint8_t *dev_image(uint8_t proto)
{
    return (PROTO_DXL == proto) ? g_dxl : g_fee;
}

const dev_cfg_t *dev_cfg(void)
{
    return &g_cfg;
}

uint8_t dev_read(uint8_t proto, uint16_t addr, uint16_t len, uint8_t *out)
{
    const uint8_t *img = dev_image(proto);
    if (0U == len) {
        return DXL_ERR_LENGTH;
    }
    if (((uint32_t)addr + len) > REG_SPACE_SIZE) {
        return DXL_ERR_RANGE;
    }
    memcpy(out, &img[addr], len);
    return 0U;
}

uint8_t dev_write(uint8_t proto, uint16_t addr, uint16_t len, const uint8_t *in)
{
    uint8_t *img = (uint8_t *)dev_image(proto);
    if (0U == len) {
        return DXL_ERR_LENGTH;
    }
    if (((uint32_t)addr + len) > REG_SPACE_SIZE) {
        return DXL_ERR_RANGE;
    }
    memcpy(&img[addr], in, len);
    g_write_calls++;
    g_write_addr = addr;
    g_write_len = len;
    memcpy(g_write_data, in, len);
    return 0U;
}

void dev_factory_reset(void) { }
static uint32_t g_reboot_calls;
void dev_request_reboot(uint32_t now_ms) { (void)now_ms; g_reboot_calls++; }
void dev_clear_hw_error(void) { }
void dev_note_error(uint8_t proto, uint8_t err)
{
    (void)proto;
    if (0U != err) {
        g_noted_err = err;
    }
}

void dbg_log(uint8_t level, const char *fmt, ...)
{
    (void)level; (void)fmt;
}
void dbg_hex(uint8_t level, const char *tag, const uint8_t *data, uint16_t len)
{
    (void)level; (void)tag; (void)data; (void)len;
}

/* ── test plumbing ──────────────────────────────────────────────────────── */

static int g_failures;
static const char *g_case;

static void check(int cond, const char *what)
{
    if (!cond) {
        printf("  FAIL [%s] %s\n", g_case, what);
        g_failures++;
    }
}

static void check_bytes(const uint8_t *got, const uint8_t *want, uint16_t n, const char *what)
{
    if (memcmp(got, want, n) != 0) {
        uint16_t i;
        printf("  FAIL [%s] %s\n    got :", g_case, what);
        for (i = 0U; i < n; i++) { printf(" %02X", got[i]); }
        printf("\n    want:");
        for (i = 0U; i < n; i++) { printf(" %02X", want[i]); }
        printf("\n");
        g_failures++;
    }
}

static void reset_dev(void)
{
    memset(g_dxl, 0, sizeof(g_dxl));
    memset(g_fee, 0, sizeof(g_fee));
    memset(&g_cfg, 0, sizeof(g_cfg));
    g_cfg.dxl_id = 200U;
    g_cfg.fee_id = 200U;
    g_cfg.sim_amp = 1000U;
    g_cfg.sim_freq = 1000U;
    g_dxl[68] = 2U;                  /* Dynamixel status return level */
    g_fee[8] = 1U;                   /* FeeTech ACK status level: answer everything */
    /* dxl2.c answers PING from the register image (not from a compile-time
       constant) so the bootloader can report BOOT_MODEL_NUMBER with the same
       code; the stub image therefore has to carry this node's identity. */
    g_dxl[0] = (uint8_t)(DXL_MODEL_NUMBER & 0xFFU);
    g_dxl[1] = (uint8_t)(DXL_MODEL_NUMBER >> 8);
    g_dxl[6] = FW_VERSION_BYTE;
    g_fee[3] = (uint8_t)(FEE_MODEL_NUMBER & 0xFFU);
    g_fee[4] = (uint8_t)(FEE_MODEL_NUMBER >> 8);
    for (uint8_t i = 0U; i < 20U; i++) {
        g_dxl[DXL_TELEM_ADDR + i] = i;
        g_fee[FEE_TELEM_ADDR + i] = i;
    }
    g_write_calls = 0;
    g_write_addr = 0U;
    g_write_len = 0U;
}

/* Builds an instruction exactly like rustypot's `to_bytes`: the parameter
   region is byte-stuffed and LEN counts the stuffed bytes. */
static uint16_t dxl_frame(uint8_t id, uint8_t inst, const uint8_t *params, uint16_t n,
                          uint8_t *out)
{
    uint8_t stuffed[DXL_MAX_BODY + (DXL_MAX_BODY / 3U) + 4U];
    uint16_t sn = stuffing_add(params, n, stuffed);
    uint16_t len = (uint16_t)(sn + 3U);
    uint16_t crc;

    out[0] = 0xFF; out[1] = 0xFF; out[2] = 0xFD; out[3] = 0x00;
    out[4] = id;
    out[5] = (uint8_t)(len & 0xFFU);
    out[6] = (uint8_t)(len >> 8);
    out[7] = inst;
    if (sn != 0U) { memcpy(&out[8], stuffed, sn); }
    crc = crc16_dxl(out, (uint16_t)(8U + sn));
    out[8U + sn] = (uint8_t)(crc & 0xFFU);
    out[9U + sn] = (uint8_t)(crc >> 8);
    return (uint16_t)(10U + sn);
}

/* Parses a status frame the way rustypot does: CRC first, then de-stuff the
   body.  Returns 0 on success and fills *err / *params. */
static int dxl_parse_status_frame(const uint8_t *f, uint16_t total,
                                  uint8_t *err, uint8_t *params, uint16_t *nparams)
{
    uint16_t len;
    uint16_t crc_read, crc_calc;
    uint8_t plain[DXL_MAX_BODY];
    uint16_t pn;

    if (total < 11U || f[0] != 0xFF || f[1] != 0xFF || f[2] != 0xFD || f[3] != 0x00) {
        return -1;
    }
    if (f[7] != 0x55) {
        return -2;
    }
    len = (uint16_t)((uint16_t)f[5] | ((uint16_t)f[6] << 8));
    if ((uint16_t)(7U + len) != total) {
        return -3;
    }
    crc_read = (uint16_t)((uint16_t)f[total - 2U] | ((uint16_t)f[total - 1U] << 8));
    crc_calc = crc16_dxl(f, (uint16_t)(total - 2U));
    if (crc_read != crc_calc) {
        return -4;
    }
    pn = stuffing_remove(&f[8], (uint16_t)(len - 3U), plain);
    if (pn < 1U) {
        return -5;
    }
    *err = plain[0];
    *nparams = (uint16_t)(pn - 1U);
    if (*nparams != 0U && params != NULL) {
        memcpy(params, &plain[1], *nparams);
    }
    return 0;
}

static uint16_t dxl_run(dxl_slave_t *s, uint8_t id, uint8_t inst,
                        const uint8_t *params, uint16_t n, uint8_t *out)
{
    uint8_t frame[DXL_MAX_FRAME];
    uint16_t flen = dxl_frame(id, inst, params, n, frame);
    uint16_t got = 0U;
    for (uint16_t i = 0U; i < flen; i++) {
        got = dxl_slave_feed(s, frame[i], 1000U, out, DXL_MAX_RESP);
    }
    return got;
}

static uint16_t fee_frame(uint8_t id, uint8_t inst, const uint8_t *params, uint8_t n,
                          uint8_t *out)
{
    /* params include ADDR (when the instruction has one); LEN counts
       INST + params + SUM */
    uint8_t len = (uint8_t)(n + 2U);
    uint8_t sum = 0U;
    out[0] = 0xFF; out[1] = 0xFF; out[2] = id; out[3] = len; out[4] = inst;
    if (n != 0U) { memcpy(&out[5], params, n); }
    for (uint16_t i = 2U; i < (uint16_t)(5U + n); i++) { sum = (uint8_t)(sum + out[i]); }
    out[5U + n] = (uint8_t)(~sum);
    return (uint16_t)(6U + n);
}

static uint16_t fee_run(fee_slave_t *s, uint8_t id, uint8_t inst,
                        const uint8_t *params, uint8_t n, uint8_t *out)
{
    uint8_t frame[FEE_MAX_FRAME];
    uint16_t flen = fee_frame(id, inst, params, n, frame);
    uint16_t got = 0U;
    for (uint16_t i = 0U; i < flen; i++) {
        got = fee_slave_feed(s, frame[i], 1000U, out, DXL_MAX_RESP);
    }
    return got;
}

/* ── Dynamixel ──────────────────────────────────────────────────────────── */

static void test_dxl_ping(void)
{
    static const uint8_t want[] = {
        0xFF, 0xFF, 0xFD, 0x00, 0xC8, 0x07, 0x00, 0x55, 0x00, 0xB0, 0x04, 0x10, 0xF4, 0x46
    };
    dxl_slave_t s;
    uint8_t out[DXL_MAX_RESP];
    uint16_t n;

    g_case = "dxl ping";
    reset_dev();
    dxl_slave_init(&s);
    n = dxl_run(&s, 200U, DXL_INST_PING, NULL, 0U, out);
    check(n == sizeof(want), "response length");
    if (n == sizeof(want)) {
        check_bytes(out, want, n, "ping response bytes");
    }

    g_case = "dxl ping broadcast";
    dxl_slave_init(&s);
    n = dxl_run(&s, DXL_BROADCAST_ID, DXL_INST_PING, NULL, 0U, out);
    check(n == sizeof(want), "broadcast ping also answers");

    g_case = "dxl ping other id";
    dxl_slave_init(&s);
    n = dxl_run(&s, 7U, DXL_INST_PING, NULL, 0U, out);
    check(n == 0U, "a ping for another id is ignored");
}

static void test_dxl_read(void)
{
    static const uint8_t want[] = {
        0xFF, 0xFF, 0xFD, 0x00, 0xC8, 0x10, 0x00, 0x55, 0x00,
        0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0xD1, 0x49
    };
    dxl_slave_t s;
    uint8_t out[DXL_MAX_RESP];
    uint8_t params[4] = { 124U, 0U, 12U, 0U };
    uint16_t n;

    g_case = "dxl read 12-byte block";
    reset_dev();
    dxl_slave_init(&s);
    n = dxl_run(&s, 200U, DXL_INST_READ, params, 4U, out);
    check(n == sizeof(want), "read response length");
    if (n == sizeof(want)) {
        check_bytes(out, want, n, "read response bytes");
    }

    g_case = "dxl read range error";
    dxl_slave_init(&s);
    params[0] = 250U; params[1] = 0U; params[2] = 12U; params[3] = 0U;
    n = dxl_run(&s, 200U, DXL_INST_READ, params, 4U, out);
    check(n == 11U, "range error status length");
    if (n >= 9U) {
        check(out[8] == DXL_ERR_RANGE, "range error bit");
    }
}

static void test_dxl_sync_read(void)
{
    dxl_slave_t s;
    uint8_t out[DXL_MAX_RESP];
    uint8_t params[16];
    uint16_t n;

    g_case = "dxl sync_read lists us first";
    reset_dev();
    dxl_slave_init(&s);
    params[0] = 124U; params[1] = 0U; params[2] = 12U; params[3] = 0U;
    params[4] = 200U; params[5] = 10U; params[6] = 11U;
    n = dxl_run(&s, DXL_BROADCAST_ID, DXL_INST_SYNC_READ, params, 7U, out);
    check(n == 23U, "sync_read answers when listed");
    if (n == 23U) {
        check(out[4] == 200U, "answers with our id");
        check(out[5] == 16U && out[6] == 0U, "status LEN = 12 + 4");
        check(out[7] == 0x55, "status marker 0x55 (rustypot requires it)");
        check(out[8] == 0U, "error byte 0");
        check(out[9] == 0U && out[20] == 11U, "12-byte block payload");
    }

    g_case = "dxl sync_read without us";
    dxl_slave_init(&s);
    params[4] = 10U; params[5] = 11U;
    n = dxl_run(&s, DXL_BROADCAST_ID, DXL_INST_SYNC_READ, params, 6U, out);
    check(n == 0U, "silent when not listed");

    g_case = "dxl sync_write for others is ignored";
    dxl_slave_init(&s);
    {
        uint8_t sw[16];
        uint8_t frame[DXL_MAX_FRAME];
        uint16_t flen;
        uint16_t got = 0U;
        sw[0] = 116U; sw[1] = 0U; sw[2] = 4U; sw[3] = 0U;
        sw[4] = 10U; sw[5] = 1U; sw[6] = 0U; sw[7] = 0U; sw[8] = 0U;
        flen = dxl_frame(DXL_BROADCAST_ID, DXL_INST_SYNC_WRITE, sw, 9U, frame);
        for (uint16_t i = 0U; i < flen; i++) {
            got = dxl_slave_feed(&s, frame[i], 1000U, out, DXL_MAX_RESP);
        }
        check(got == 0U, "no response");
        check(g_write_calls == 0, "no write happened");
    }
}

/* ── Fast Sync Read (0x8A) ─────────────────────────────────────────────── */

/* The aggregate-packet parser, transcribed from rustypot 1.8's
   `parse_fast_sync_read_status_with_error` (src/dynamixel_protocol/v2.rs) - the
   reference consumer this firmware has to match byte for byte.  It is here so a
   test can check the whole packet a Fast Sync Read produces and not only the
   block this node sends, and it is exercised on the protocol 2.0 e-manual
   example below, which is what makes it a transcription rather than a parser
   written to agree with the implementation under test.

   Returns 0 and fills out/errs (out may be NULL) on success. */
static int fast_ref_parse(const uint8_t *data, uint16_t total,
                          const uint8_t *ids, uint16_t nids, uint16_t xlen,
                          uint8_t *out, uint8_t *errs, uint16_t *nvalues)
{
    uint16_t block_size = (uint16_t)(xlen + 4U);
    uint16_t payload;
    uint16_t i;

    if (total != (uint16_t)(8U + (nids * block_size))) {
        return -1;
    }
    if (total < 8U || data[4] != DXL_BROADCAST_ID || data[7] != DXL_STATUS_MARKER) {
        return -2;
    }
    payload = (uint16_t)((uint16_t)data[5] | ((uint16_t)data[6] << 8));
    if (payload != (uint16_t)(total - 7U)) {
        return -3;
    }
    for (i = 0U; i < nids; i++) {
        uint16_t block = (uint16_t)(8U + (i * block_size));
        uint16_t crc_at = (uint16_t)(block + 2U + xlen);
        uint16_t read_crc = (uint16_t)((uint16_t)data[crc_at]
                                       | ((uint16_t)data[crc_at + 1U] << 8));
        if (read_crc != crc16_dxl(data, crc_at)) {
            return -4;
        }
        if (data[block + 1U] != ids[i]) {
            return -5;
        }
        if (NULL != out) {
            memcpy(&out[i * xlen], &data[block + 2U], xlen);
        }
        if (NULL != errs) {
            errs[i] = data[block];
        }
    }
    *nvalues = nids;
    return 0;
}

/* microduck's joint order (duck-control/src/model.rs `JOINT_IDS`), so the id list
   the tests build is the one the runtime builds. */
static const uint8_t g_joint_ids[15] = {
    20U, 21U, 22U, 23U, 24U, 30U, 31U, 32U, 33U, 34U, 10U, 11U, 12U, 13U, 14U
};

/* ROBOTIS protocol 2.0 e-manual, "Fast Sync Read (0x8A)" example: ids 3, 7 and 4
   answer a read of present position (addr 132, X = 4) with 166, 2079 and 1023.
   The same 32 bytes are rustypot 1.8's FAST_SYNC_READ_STATUS test vector. */
static const uint8_t g_fast_example[32] = {
    0xFF, 0xFF, 0xFD, 0x00, 0xFE, 0x19, 0x00, 0x55,
    0x00, 0x03, 0xA6, 0x00, 0x00, 0x00, 0x84, 0x08,
    0x00, 0x07, 0x1F, 0x08, 0x00, 0x00, 0x16, 0xCA,
    0x00, 0x04, 0xFF, 0x03, 0x00, 0x00, 0xD1, 0x9E
};

/*! \brief the 8 bytes the device heading the id list sends:
           `FF FF FD 00 FE LEN_L LEN_H 0x55`, LEN covering the whole packet. */
static void fast_write_prefix(uint8_t *pkt, uint16_t total)
{
    pkt[0] = DXL_HEADER_0;
    pkt[1] = DXL_HEADER_1;
    pkt[2] = DXL_HEADER_2;
    pkt[3] = DXL_HEADER_3;
    pkt[4] = DXL_BROADCAST_ID;
    pkt[5] = (uint8_t)(total & 0xFFU);
    pkt[6] = (uint8_t)(total >> 8);
    pkt[7] = DXL_STATUS_MARKER;
}

/*! \brief append a block to an aggregate packet the way a real servo does:
           ERROR, ID, DATA, then the CRC of everything including this block. */
static uint16_t fast_synth_block(uint8_t *pkt, uint16_t at, uint8_t id,
                                 uint8_t seed, uint16_t xlen)
{
    uint16_t i;
    uint16_t crc;

    pkt[at] = 0U;
    pkt[at + 1U] = id;
    for (i = 0U; i < xlen; i++) {
        pkt[at + 2U + i] = (uint8_t)(seed + i);
    }
    crc = crc16_dxl(pkt, (uint16_t)(at + 2U + xlen));
    pkt[at + 2U + xlen] = (uint8_t)(crc & 0xFFU);
    pkt[at + 3U + xlen] = (uint8_t)(crc >> 8);
    return (uint16_t)(at + 4U + xlen);
}

/*! \brief one received byte as src/bus.c handles it: the fast-sync feeder first,
           the frame parser after. */
static uint16_t dxl_bus_feed(dxl_slave_t *s, uint8_t byte, uint8_t *out)
{
    uint16_t n = dxl_slave_fast_rx(s, byte, out, DXL_MAX_RESP);

    if (0U == n) {
        n = dxl_slave_feed(s, byte, 1000U, out, DXL_MAX_RESP);
    }
    return n;
}

static void test_dxl_fast_ref(void)
{
    static const uint8_t ids[3] = { 3U, 7U, 4U };
    uint8_t vals[12];
    uint8_t errs[3];
    uint16_t nv = 0U;
    uint16_t i;

    g_case = "fast sync read: reference parser on the e-manual packet";
    check(fast_ref_parse(g_fast_example, sizeof(g_fast_example), ids, 3U, 4U,
                         vals, errs, &nv) == 0, "the e-manual example parses");
    check(nv == 3U, "three blocks");
    check(vals[0] == 0xA6U && vals[1] == 0x00U && vals[2] == 0x00U && vals[3] == 0x00U,
          "id 3 = 166");
    check(vals[4] == 0x1FU && vals[5] == 0x08U, "id 7 = 2079");
    check(vals[8] == 0xFFU && vals[9] == 0x03U, "id 4 = 1023");
    for (i = 0U; i < 3U; i++) {
        check(errs[i] == 0U, "error byte is its own field");
    }
    /* The parser has to be strict, or agreeing with it means nothing: the
       negative cases are rustypot's own. */
    {
        static const uint8_t wrong_order[3] = { 3U, 4U, 7U };
        uint8_t flipped[32];
        check(fast_ref_parse(g_fast_example, 32U, wrong_order, 3U, 4U, vals, errs, &nv) != 0,
              "ids in an order we did not ask for are rejected");
        memcpy(flipped, g_fast_example, sizeof(flipped));
        flipped[10] ^= 0x01U;
        check(fast_ref_parse(flipped, 32U, ids, 3U, 4U, vals, errs, &nv) != 0,
              "one flipped data byte is rejected");
        check(fast_ref_parse(g_fast_example, 24U, ids, 3U, 4U, vals, errs, &nv) != 0,
              "a packet one block short is rejected");
    }
}

static void test_dxl_fast_sync_read(void)
{
    dxl_slave_t s;
    uint8_t out[DXL_MAX_RESP];
    uint8_t params[4U + 16U];
    uint8_t pkt[8U + (16U * 16U)];
    uint8_t ids[16];
    uint8_t vals[16U * 12U];
    uint8_t errs[16];
    uint16_t nv = 0U;
    uint16_t n;
    uint16_t at;
    uint16_t i;

    /* microduck's tick read: id list [IMU, servos...] at address 124, 12 bytes -
       the node is entry 0, which is the case the runtime exercises. */
    g_case = "dxl fast_sync_read lists us first";
    reset_dev();
    dxl_slave_init(&s);
    params[0] = 124U; params[1] = 0U; params[2] = 12U; params[3] = 0U;
    params[4] = 200U;
    for (i = 0U; i < 15U; i++) {
        params[5U + i] = g_joint_ids[i];
    }
    n = dxl_run(&s, DXL_BROADCAST_ID, DXL_INST_FAST_SYNC_READ, params, 20U, out);
    check(n == 24U, "prefix + our 12-byte block, nothing else");
    {
        /* The instruction rustypot 1.8 actually generates for this read.  It is
           pinned here and asserted against rustypot's own output in
           host/tools/rustypot_check, so these two agree on the bytes the node is
           handed as well as on the bytes it answers with. */
        static const uint8_t want_instr[30] = {
            0xFF, 0xFF, 0xFD, 0x00, 0xFE, 0x17, 0x00, 0x8A, 0x7C, 0x00,
            0x0C, 0x00, 0xC8, 0x14, 0x15, 0x16, 0x17, 0x18, 0x1E, 0x1F,
            0x20, 0x21, 0x22, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x5F, 0x95
        };
        uint8_t instr[DXL_MAX_FRAME];
        uint16_t ilen = dxl_frame(DXL_BROADCAST_ID, DXL_INST_FAST_SYNC_READ,
                                  params, 20U, instr);
        check(ilen == 30U, "the instruction is 30 bytes");
        if (ilen == 30U) {
            check_bytes(instr, want_instr, 30U, "rustypot's own instruction bytes");
        }
    }
    {
        /* The exact 24 bytes the firmware puts on the wire for microduck's tick
           read with the register image this test installs (g_dxl[124 + i] = i).
           host/tools/rustypot_check feeds the same vector to the real rustypot
           parser, so both halves of that check are pinned to one number. */
        static const uint8_t want24[24] = {
            0xFF, 0xFF, 0xFD, 0x00, 0xFE, 0x01, 0x01, 0x55,
            0x00, 0xC8, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05,
            0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x8E, 0xA1
        };
        if (n == 24U) {
            check_bytes(out, want24, 24U, "the block, byte for byte");
        }
    }
    if (n == 24U) {
        check(out[0] == 0xFFU && out[1] == 0xFFU && out[2] == 0xFDU && out[3] == 0x00U,
              "header");
        check(out[4] == DXL_BROADCAST_ID, "the aggregate answers as 0xFE");
        check((uint16_t)(out[5] | ((uint16_t)out[6] << 8)) == 257U,
              "LEN covers all 16 blocks (1 + 16 * 16)");
        check(out[7] == DXL_STATUS_MARKER, "0x55 status marker");
        check(out[8] == 0U, "error byte");
        check(out[9] == 200U, "our own id inside the block");
        check(out[10] == 0U && out[21] == 11U, "the 12 telemetry bytes");
        {
            uint16_t want = crc16_dxl(out, 22U);
            check(out[22] == (uint8_t)(want & 0xFFU) && out[23] == (uint8_t)(want >> 8),
                  "CRC covers the prefix and our block");
        }
    }

    /* Complete that packet the way the fifteen servos behind us would, and read
       it back with the reference parser: this is what robotd gets to see. */
    g_case = "dxl fast_sync_read: the completed packet parses";
    memcpy(pkt, out, 24U);
    at = 24U;
    for (i = 0U; i < 15U; i++) {
        at = fast_synth_block(pkt, at, g_joint_ids[i], (uint8_t)(0x10U + i), 12U);
    }
    check(at == 8U + (16U * 16U), "16 blocks, 264 bytes");
    for (i = 0U; i < 16U; i++) {
        ids[i] = (0U == i) ? 200U : g_joint_ids[i - 1U];
    }
    check(fast_ref_parse(pkt, at, ids, 16U, 12U, vals, errs, &nv) == 0,
          "the aggregate packet of 16 devices parses");
    check(nv == 16U, "16 values back");
    for (i = 0U; i < 12U; i++) {
        check(vals[i] == (uint8_t)i, "the IMU block comes back first, byte for byte");
    }
    check(errs[0] == 0U && errs[15] == 0U, "error bytes line up with the blocks");

    g_case = "dxl fast_sync_read without us";
    dxl_slave_init(&s);
    for (i = 0U; i < 16U; i++) {
        params[4U + i] = (uint8_t)(10U + i);
    }
    n = dxl_run(&s, DXL_BROADCAST_ID, DXL_INST_FAST_SYNC_READ, params, 20U, out);
    check(n == 0U, "silent when the id list does not name us");

    g_case = "dxl fast_sync_read mid-list waits for the block ahead";
    reset_dev();
    dxl_slave_init(&s);
    params[4] = 10U; params[5] = 200U; params[6] = 11U;   /* we are entry 1 */
    n = dxl_run(&s, DXL_BROADCAST_ID, DXL_INST_FAST_SYNC_READ, params, 7U, out);
    check(n == 0U, "nothing goes out before the device ahead has finished");
    fast_write_prefix(pkt, 49U);         /* LEN = 1 + 3 * 16 */
    at = fast_synth_block(pkt, 8U, 10U, 0x40U, 12U);     /* -> 24 */
    for (i = 0U; i < 24U; i++) {
        n = dxl_bus_feed(&s, pkt[i], out);
        if ((i + 1U) < 24U) {
            check(n == 0U, "still waiting while the block ahead is arriving");
        }
    }
    check(n == 16U, "our block goes out on the byte that completes the one ahead");
    if (n == 16U) {
        check(out[0] == 0U && out[1] == 200U, "ERROR + our id, no prefix of our own");
        check(out[2] == 0U && out[13] == 11U, "our 12 telemetry bytes");
        memcpy(&pkt[24], out, 16U);
        at = fast_synth_block(pkt, 40U, 11U, 0x80U, 12U);
        check(at == 8U + (3U * 16U), "a complete 3-block packet");
        {
            static const uint8_t ids3[3] = { 10U, 200U, 11U };
            check(fast_ref_parse(pkt, at, ids3, 3U, 12U, vals, errs, &nv) == 0,
                  "the packet with our block in the middle parses");
            for (i = 0U; i < 12U; i++) {
                check(vals[12U + i] == (uint8_t)i, "our block landed in entry 1");
            }
            check(errs[1] == 0U, "and carries no error");
        }
    }

    g_case = "dxl fast_sync_read mid-list drops a stream that is not ours";
    dxl_slave_init(&s);
    (void)dxl_run(&s, DXL_BROADCAST_ID, DXL_INST_FAST_SYNC_READ, params, 7U, out);
    check(dxl_bus_feed(&s, 0x00U, out) == 0U, "a byte that cannot start the packet");
    /* the obligation is gone: the real prefix and block now pass unremarked */
    fast_write_prefix(pkt, 49U);         /* LEN = 1 + 3 * 16 */
    at = fast_synth_block(pkt, 8U, 10U, 0x40U, 12U);
    n = 0U;
    for (i = 0U; i < at; i++) {
        n = dxl_bus_feed(&s, pkt[i], out);
    }
    check(n == 0U, "no block is sent into a packet we already gave up on");

    g_case = "dxl fast_sync_read waits out a gap before the first block";
    reset_dev();
    dxl_slave_init(&s);
    params[0] = 124U; params[1] = 0U; params[2] = 12U; params[3] = 0U;
    params[4] = 10U; params[5] = 200U; params[6] = 11U;
    (void)dxl_run(&s, DXL_BROADCAST_ID, DXL_INST_FAST_SYNC_READ, params, 7U, out);
    /* What the >BUS_GAP_US rule does when the device ahead has not spoken yet:
       it must NOT throw the obligation away - that device is allowed its own
       return delay time, and on a factory-fresh servo that is 500 us. */
    dxl_slave_reset(&s);
    fast_write_prefix(pkt, 49U);
    at = fast_synth_block(pkt, 8U, 10U, 0x40U, 12U);
    n = 0U;
    for (i = 0U; i < at; i++) {
        n = dxl_bus_feed(&s, pkt[i], out);
    }
    check(n == 16U, "the block still goes out after the gap");

    g_case = "dxl fast_sync_read drops a stream that breaks mid-packet";
    dxl_slave_init(&s);
    (void)dxl_run(&s, DXL_BROADCAST_ID, DXL_INST_FAST_SYNC_READ, params, 7U, out);
    fast_write_prefix(pkt, 49U);
    at = fast_synth_block(pkt, 8U, 10U, 0x40U, 12U);
    check(dxl_bus_feed(&s, pkt[0], out) == 0U, "the prefix has started");
    dxl_slave_reset(&s);            /* the stream broke after it started */
    n = 0U;
    for (i = 1U; i < at; i++) {
        n = dxl_bus_feed(&s, pkt[i], out);
    }
    check(n == 0U, "no block is sent into a broken aggregate");

    g_case = "dxl fast_sync_read does not byte-stuff";
    reset_dev();
    dxl_slave_init(&s);
    g_dxl[124] = 0xFFU; g_dxl[125] = 0xFFU; g_dxl[126] = 0xFDU;
    params[4] = 200U;                    /* a list of one: LEN = 1 + 16 */
    n = dxl_run(&s, DXL_BROADCAST_ID, DXL_INST_FAST_SYNC_READ, params, 5U, out);
    check(n == 24U, "prefix + block, no growth");
    if (n == 24U) {
        check(out[10] == 0xFFU && out[11] == 0xFFU && out[12] == 0xFDU,
              "FF FF FD survives verbatim in the payload");
    }

    g_case = "dxl fast_sync_read status return level 0";
    dxl_slave_init(&s);
    g_dxl[68] = 0U;
    n = dxl_run(&s, DXL_BROADCAST_ID, DXL_INST_FAST_SYNC_READ, params, 5U, out);
    check(n == 0U, "silent, like every other read");
    g_dxl[68] = 2U;

    g_case = "dxl fast_sync_read range error";
    dxl_slave_init(&s);
    params[0] = 250U; params[1] = 0U; params[2] = 12U; params[3] = 0U;
    n = dxl_run(&s, DXL_BROADCAST_ID, DXL_INST_FAST_SYNC_READ, params, 5U, out);
    check(n == 24U, "the block keeps its size");
    if (n == 24U) {
        check(out[8] == DXL_ERR_RANGE, "range error in the block's error byte");
        check(out[10] == 0U && out[21] == 0U, "with a zeroed payload");
    }

    g_case = "dxl fast_sync_read malformed and impossible lists";
    dxl_slave_init(&s);
    n = dxl_run(&s, DXL_BROADCAST_ID, DXL_INST_FAST_SYNC_READ, params, 3U, out);
    check(n == 0U, "fewer than four parameters is silence");
    dxl_slave_init(&s);
    params[0] = 124U; params[1] = 0U; params[2] = 0xF0U; params[3] = 0x01U;  /* X = 496 */
    n = dxl_run(&s, DXL_BROADCAST_ID, DXL_INST_FAST_SYNC_READ, params, 5U, out);
    check(n == 0U, "a read longer than the register space is silence");
    {
        /* 260 ids of 260 bytes each: the aggregate LEN would not fit in 16 bits,
           so the packet cannot exist and this node does not join it */
        uint8_t many[4U + 260U];
        dxl_slave_init(&s);
        many[0] = 124U; many[1] = 0U; many[2] = 0x00U; many[3] = 0x01U;  /* X = 256 */
        for (i = 0U; i < 260U; i++) {
            many[4U + i] = (0U == i) ? 200U : (uint8_t)(30U + i);
        }
        n = dxl_run(&s, DXL_BROADCAST_ID, DXL_INST_FAST_SYNC_READ, many, 4U + 260U, out);
        check(n == 0U, "a LEN that cannot be expressed is silence");
    }
}

static void test_dxl_errors(void)
{
    dxl_slave_t s;
    uint8_t out[DXL_MAX_RESP];
    uint8_t frame[DXL_MAX_FRAME];
    uint8_t params[4] = { 124U, 0U, 12U, 0U };
    uint16_t flen;
    uint16_t n = 0U;

    g_case = "dxl crc error";
    reset_dev();
    dxl_slave_init(&s);
    flen = dxl_frame(200U, DXL_INST_READ, params, 4U, frame);
    frame[flen - 1U] ^= 0xA5U;
    for (uint16_t i = 0U; i < flen; i++) {
        n = dxl_slave_feed(&s, frame[i], 1000U, out, DXL_MAX_RESP);
    }
    check(n == 11U, "crc error status length");
    if (n >= 9U) {
        check(out[8] == DXL_ERR_CRC, "checksum error bit");
    }

    g_case = "dxl unsupported instruction";
    dxl_slave_init(&s);
    n = dxl_run(&s, 200U, 0x77U, NULL, 0U, out);
    check(n == 11U, "instruction error status length");
    if (n >= 9U) {
        check(out[8] == DXL_ERR_INSTRUCTION, "instruction error bit");
    }

    g_case = "dxl write ack";
    reset_dev();
    dxl_slave_init(&s);
    {
        uint8_t wp[3] = { 9U, 0U, 5U };   /* return delay time = 5 */
        n = dxl_run(&s, 200U, DXL_INST_WRITE, wp, 3U, out);
        check(n == 11U, "write status length");
        if (n >= 9U) {
            check(out[8] == 0U, "write accepted");
        }
        check(g_write_calls == 1, "dev_write called");
        check(g_write_addr == 9U && g_write_len == 1U, "write landed on address 9");
        check(g_write_data[0] == 5U, "written value");
    }
}

/* ── Dynamixel 2.0 byte stuffing ───────────────────────────────────────── */

static void test_stuffing(void)
{
    uint8_t out[64];
    uint16_t n;

    g_case = "stuffing: rustypot vectors";

    {
        static const uint8_t in[] = { 1, 2, 0xFF, 0xFD, 3 };
        n = stuffing_add(in, sizeof(in), out);
        check(n == sizeof(in) && memcmp(out, in, sizeof(in)) == 0, "no pattern -> untouched");
    }
    {
        static const uint8_t in[] = { 0xFF, 0xFF, 0xFD, 7 };
        static const uint8_t want[] = { 0xFF, 0xFF, 0xFD, 0xFD, 7 };
        n = stuffing_add(in, sizeof(in), out);
        check(n == sizeof(want) && memcmp(out, want, n) == 0, "FF FF FD -> extra FD");
    }
    {
        static const uint8_t in[] = { 0xFF, 0xFF, 0xFD, 0xFD };
        static const uint8_t want[] = { 0xFF, 0xFF, 0xFD, 0xFD, 0xFD };
        n = stuffing_add(in, sizeof(in), out);
        check(n == sizeof(want) && memcmp(out, want, n) == 0, "scan restarts after the stuffed byte");
    }
    {
        static const uint8_t in[] = { 0xFF, 0xFF, 0xFF, 0xFD };
        static const uint8_t want[] = { 0xFF, 0xFF, 0xFF, 0xFD, 0xFD };
        n = stuffing_add(in, sizeof(in), out);
        check(n == sizeof(want) && memcmp(out, want, n) == 0, "FF FF suffix survives extra FFs");
    }
    {
        static const uint8_t in[] = { 0xFF, 0xFF, 0xFD, 0xFD, 5, 0xFF, 0xFF, 0xFD };
        uint8_t round[64];
        uint16_t sn = stuffing_add(in, sizeof(in), out);
        uint16_t rn = stuffing_remove(out, sn, round);
        check(rn == sizeof(in) && memcmp(round, in, sizeof(in)) == 0,
              "add/remove round trip");
    }
}

static void test_dxl_status_stuffing(void)
{
    dxl_slave_t s;
    uint8_t out[DXL_MAX_RESP];
    uint8_t params[4] = { 124U, 0U, 12U, 0U };
    uint8_t err = 0xAAU;
    uint8_t got[32];
    uint16_t nparams = 0U;
    uint16_t n;

    g_case = "dxl status stuffing";
    reset_dev();
    /* a block whose bytes contain FF FF FD: this is exactly the case that made
       rustypot 1.6 add de-stuffing, and a node that does not stuff breaks it */
    g_dxl[DXL_TELEM_ADDR + 0] = 0xFFU;
    g_dxl[DXL_TELEM_ADDR + 1] = 0xFFU;
    g_dxl[DXL_TELEM_ADDR + 2] = 0xFDU;
    g_dxl[DXL_TELEM_ADDR + 3] = 0x7FU;
    for (int i = 4; i < 12; i++) { g_dxl[DXL_TELEM_ADDR + i] = (uint8_t)i; }

    dxl_slave_init(&s);
    n = dxl_run(&s, 200U, DXL_INST_READ, params, 4U, out);
    check(n == 24U, "one stuffing byte makes the frame 24 bytes (23 + 1)");
    check(dxl_parse_status_frame(out, n, &err, got, &nparams) == 0, "status frame parses after de-stuffing");
    check(err == 0U, "error byte 0");
    check(nparams == 12U, "12 parameter bytes recovered");
    check(got[0] == 0xFFU && got[1] == 0xFFU && got[2] == 0xFDU && got[3] == 0x7FU,
          "payload identical after the round trip");

    g_case = "dxl instruction de-stuffing";
    /* WRITE with addr = 0xFFFF puts FF FF in the parameter region, and a 0xFD
       data byte completes the pattern; the node must still parse the frame and
       answer with a range error rather than dropping it as malformed. */
    {
        uint8_t wp[3] = { 0xFFU, 0xFFU, 0xFDU };
        dxl_slave_init(&s);
        n = dxl_run(&s, 200U, DXL_INST_WRITE, wp, 3U, out);
        check(n == 11U, "range error status");
        if (n >= 11U) {
            check(dxl_parse_status_frame(out, n, &err, got, &nparams) == 0, "parses");
            check(err == DXL_ERR_RANGE, "range error bit set");
        }
    }
}

/* ── FeeTech ────────────────────────────────────────────────────────────── */

static void test_fee_ping_read(void)
{
    static const uint8_t want_ping[] = { 0xFF, 0xFF, 0xC8, 0x02, 0x00, 0x35 };
    static const uint8_t want_read[] = {
        0xFF, 0xFF, 0xC8, 0x0E, 0x00, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05,
        0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0xE7
    };
    static const uint8_t want_20[] = {
        0xFF, 0xFF, 0xC8, 0x16, 0x00, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
        0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13, 0x63
    };
    fee_slave_t s;
    uint8_t out[DXL_MAX_RESP];
    uint8_t params[4];
    uint16_t n;

    g_case = "fee ping";
    reset_dev();
    fee_slave_init(&s);
    n = fee_run(&s, 200U, FEE_INST_PING, NULL, 0U, out);
    check(n == sizeof(want_ping), "ping ack length");
    if (n == sizeof(want_ping)) {
        check_bytes(out, want_ping, n, "ping ack bytes");
    }

    g_case = "fee read 12";
    fee_slave_init(&s);
    params[0] = 56U;
    params[1] = 12U;
    n = fee_run(&s, 200U, FEE_INST_READ, params, 2U, out);
    check(n == sizeof(want_read), "read ack length");
    if (n == sizeof(want_read)) {
        check_bytes(out, want_read, n, "read ack bytes");
    }

    g_case = "fee read 20";
    fee_slave_init(&s);
    params[1] = 20U;
    n = fee_run(&s, 200U, FEE_INST_READ, params, 2U, out);
    check(n == sizeof(want_20), "20-byte read ack length");
    if (n == sizeof(want_20)) {
        check_bytes(out, want_20, n, "20-byte read ack bytes");
    }

    g_case = "fee ping other id";
    fee_slave_init(&s);
    n = fee_run(&s, 7U, FEE_INST_PING, NULL, 0U, out);
    check(n == 0U, "ignored");
}

static void test_fee_sync(void)
{
    fee_slave_t s;
    uint8_t out[DXL_MAX_RESP];
    uint8_t params[16];
    uint16_t n;

    g_case = "fee sync_read lists us";
    reset_dev();
    fee_slave_init(&s);
    params[0] = 56U; params[1] = 12U; params[2] = 200U; params[3] = 1U;
    n = fee_run(&s, FEE_BROADCAST_ID, FEE_INST_SYNC_READ, params, 4U, out);
    check(n == 18U, "sync_read ack length (12 + 6)");
    if (n == 18U) {
        check(out[2] == 200U, "answers with our id");
        check(out[3] == 14U, "LEN = data + 2");
        check(out[4] == 0U, "status ok");
        check(out[5] == 0U && out[16] == 11U, "payload");
    }

    g_case = "fee sync_read without us";
    fee_slave_init(&s);
    params[2] = 1U; params[3] = 2U;
    n = fee_run(&s, FEE_BROADCAST_ID, FEE_INST_SYNC_READ, params, 4U, out);
    check(n == 0U, "silent when not listed");

    g_case = "fee sync_write for us is applied, not answered";
    fee_slave_init(&s);
    {
        uint8_t sw[8];
        sw[0] = 40U;          /* torque enable */
        sw[1] = 1U;           /* 1 byte per id */
        sw[2] = 200U;         /* id */
        sw[3] = 1U;           /* value */
        sw[4] = 1U;           /* id 1 */
        sw[5] = 0U;
        n = fee_run(&s, FEE_BROADCAST_ID, FEE_INST_SYNC_WRITE, sw, 6U, out);
        check(n == 0U, "no ack for sync write");
        check(g_write_calls == 1, "write applied");
        check(g_write_addr == 40U && g_write_data[0] == 1U, "address/value");
    }

    g_case = "fee bad checksum";
    fee_slave_init(&s);
    {
        uint8_t frame[FEE_MAX_FRAME];
        uint16_t flen = fee_frame(200U, FEE_INST_PING, NULL, 0U, frame);
        uint16_t got = 0U;
        frame[flen - 1U] ^= 0xFFU;
        for (uint16_t i = 0U; i < flen; i++) {
            got = fee_slave_feed(&s, frame[i], 1000U, out, DXL_MAX_RESP);
        }
        check(got == 0U, "no response to a bad checksum");
    }

    g_case = "fee ack level 0 silences writes";
    reset_dev();
    g_fee[8] = 0U;                   /* vendor table: 0 = only read/PING answered */
    fee_slave_init(&s);
    {
        uint8_t wp[2] = { 40U, 1U }; /* torque enable */
        uint8_t rp[2] = { 56U, 12U };
        n = fee_run(&s, 200U, FEE_INST_WRITE, wp, 2U, out);
        check(n == 0U, "no ack for a write at level 0");
        check(g_write_addr == 40U && g_write_data[0] == 1U, "but the write still happened");
        n = fee_run(&s, 200U, FEE_INST_READ, rp, 2U, out);
        check(n == 18U, "reads are still answered at level 0");
    }

    g_case = "fee write ack";
    reset_dev();
    fee_slave_init(&s);
    {
        uint8_t wp[2] = { 5U, 42U };   /* id = 42 */
        n = fee_run(&s, 200U, FEE_INST_WRITE, wp, 2U, out);
        check(n == 6U, "write ack length");
        if (n == 6U) {
            check(out[4] == 0U, "status ok");
        }
        check(g_write_addr == 5U && g_write_data[0] == 42U, "id register written");
    }
}

/* ── simulator frame conventions ────────────────────────────────────────── */

static void decode_block_as_host(const uint8_t *b, quat_t *q_trunk,
                                 float omega_trunk[3], float accel_mg[3])
{
    quat_t mount = { MOUNT_QW, MOUNT_QX, MOUNT_QY, MOUNT_QZ };
    quat_t mount_inv = { MOUNT_QW, -MOUNT_QX, -MOUNT_QY, -MOUNT_QZ };
    float v[3];
    int16_t g[3];
    uint16_t h[3];
    int i;

    for (i = 0; i < 3; i++) {
        g[i] = (int16_t)((uint16_t)b[2 * i] | ((uint16_t)b[2 * i + 1] << 8));
    }
    v[0] = (float)g[0] * IMU_GYRO_MDPS_PER_LSB / 1000.0f;   /* deg/s, then radians */
    v[1] = (float)g[1] * IMU_GYRO_MDPS_PER_LSB / 1000.0f;
    v[2] = (float)g[2] * IMU_GYRO_MDPS_PER_LSB / 1000.0f;
    for (i = 0; i < 3; i++) {
        v[i] *= (float)(M_PI / 180.0);
    }
    /* microduck: gyro = rotate(mount, gyro_sensor) */
    quat_rotate(mount, v, omega_trunk);

    for (i = 0; i < 3; i++) {
        h[i] = (uint16_t)b[6 + 2 * i] | ((uint16_t)b[7 + 2 * i] << 8);
    }
    {
        float x = half_to_f32(h[0]);
        float y = half_to_f32(h[1]);
        float z = half_to_f32(h[2]);
        float norm = x * x + y * y + z * z;
        quat_t qraw;
        qraw.w = sqrtf(fmaxf(0.0f, 1.0f - norm));
        qraw.x = x;
        qraw.y = y;
        qraw.z = z;
        /* microduck: q_trunk = q_raw * mount_inv */
        *q_trunk = quat_mul(qraw, mount_inv);
    }

    for (i = 0; i < 3; i++) {
        int16_t a = (int16_t)((uint16_t)b[12 + 2 * i] | ((uint16_t)b[13 + 2 * i] << 8));
        v[i] = (float)a * IMU_ACCEL_MG_PER_LSB;
    }
    quat_rotate(mount, v, accel_mg);
}

static float angle_between(const float a[3], const float b[3])
{
    float dot = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    float na = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    float nb = sqrtf(b[0] * b[0] + b[1] * b[1] + b[2] * b[2]);
    if (na <= 0.0f || nb <= 0.0f) {
        return 999.0f;
    }
    dot /= (na * nb);
    if (dot > 1.0f) { dot = 1.0f; }
    if (dot < -1.0f) { dot = -1.0f; }
    return acosf(dot);
}

/* ── the reply slot, the FeeTech 15-byte block, the gap reset ───────────── */

static void test_telem_pack(void)
{
    uint8_t telem[TELEM_LEN];
    uint8_t blk[FEE_BLOCK_LEN];
    uint8_t i;

    g_case = "fee 56..70 block";
    for (i = 0U; i < TELEM_LEN; i++) {
        telem[i] = (uint8_t)(0xA0U + i);
    }
    telem[TELEM_CNT] = 0x37U;
    telem[TELEM_STATUS] = 0x81U;
    telem_pack_fee15(telem, blk);

    check(FEE_BLOCK_LEN == 15U, "the block is the 15 bytes a servo answers at 56");
    check(FEE_BLOCK_LEN <= (FEE_TELEM_ALT_ADDR - FEE_TELEM_ADDR),
          "and it stops before the diagnostic alias");
    check(memcmp(blk, telem, TELEM_CTRL_LEN) == 0, "first 12 bytes are the control block");
    check(blk[FEE_BLOCK_CNT] == 0x37U, "sample counter lands at byte 12, inside the read");
    check(blk[FEE_BLOCK_STATUS] == 0x81U, "status lands at byte 13");
    check(blk[FEE_BLOCK_RESERVED] == 0U, "byte 14 is reserved and zero");
}

static void test_sync_index(void)
{
    fee_slave_t f;
    dxl_slave_t d;
    uint8_t out[DXL_MAX_RESP];
    uint8_t fp[5];
    uint8_t dp[6];

    g_case = "sync_read index - fee";
    reset_dev();
    fp[0] = 56U; fp[1] = 15U;
    fp[2] = 7U; fp[3] = 200U; fp[4] = 1U;
    fee_slave_init(&f);
    (void)fee_run(&f, FEE_BROADCAST_ID, FEE_INST_SYNC_READ, fp, 5U, out);
    check(f.sync_named == 1U && f.sync_index == 1U, "named second -> one reply slot");

    fee_slave_init(&f);
    fp[2] = 200U; fp[3] = 7U; fp[4] = 1U;
    (void)fee_run(&f, FEE_BROADCAST_ID, FEE_INST_SYNC_READ, fp, 5U, out);
    check(f.sync_named == 1U && f.sync_index == 0U, "named first -> no slot");

    fee_slave_init(&f);
    fp[2] = 7U; fp[3] = 8U; fp[4] = 1U;
    (void)fee_run(&f, FEE_BROADCAST_ID, FEE_INST_SYNC_READ, fp, 5U, out);
    check(f.sync_named == 0U, "not named -> no slot bookkeeping");

    g_case = "sync_read index - dxl";
    dp[0] = (uint8_t)DXL_TELEM_ADDR; dp[1] = 0U; dp[2] = 15U; dp[3] = 0U;
    dp[4] = 7U; dp[5] = 200U;
    dxl_slave_init(&d);
    (void)dxl_run(&d, DXL_BROADCAST_ID, DXL_INST_SYNC_READ, dp, 6U, out);
    check(d.sync_named == 1U && d.sync_index == 1U, "named second -> one reply slot");

    dxl_slave_init(&d);
    dp[4] = 200U; dp[5] = 7U;
    (void)dxl_run(&d, DXL_BROADCAST_ID, DXL_INST_SYNC_READ, dp, 6U, out);
    check(d.sync_named == 1U && d.sync_index == 0U, "named first -> no slot");
}

static void test_bus_arb(void)
{
    bus_arb_t a;

    g_case = "reply slot timing";
    bus_arb_init(&a, BUS_SLOT_US);
    bus_arb_set_id(&a, 200U);
    check(bus_arb_step(&a, 0U) == 0U, "idle is not ready");

    bus_arb_start(&a, 1000U, 0U);
    check(bus_arb_step(&a, 1000U) == 1U, "index 0 is ready immediately");

    bus_arb_start(&a, 1000U, 3U);
    check(bus_arb_step(&a, 1000U + BUS_SLOT_US - 1U) == 0U, "not before the slot ends");
    check(bus_arb_step(&a, 1000U + BUS_SLOT_US) == 0U, "one slot down, two to go");
    check(bus_arb_step(&a, 1000U + 2U * BUS_SLOT_US) == 0U, "two slots down");
    check(bus_arb_step(&a, 1000U + 3U * BUS_SLOT_US) == 1U, "three slots -> ready");
    check(bus_arb_step(&a, 1000U + 3U * BUS_SLOT_US + 7U) == 1U, "ready stays ready");
    bus_arb_cancel(&a);
    check(bus_arb_step(&a, 900000U) == 0U, "cancelled");

    /* An absent device still costs exactly one slot (measured on the real
       bus), so a long jump covers several slots at once. */
    bus_arb_start(&a, 0U, 3U);
    check(bus_arb_step(&a, 3U * BUS_SLOT_US) == 1U, "three expired slots at once");

    g_case = "reply slot abort";
    bus_arb_start(&a, 0U, 2U);
    bus_arb_rx(&a, 0xFFU); bus_arb_rx(&a, 0xFFU); bus_arb_rx(&a, 200U);
    check(a.state == BUS_ARB_IDLE, "unicast to us cancels the wait");

    bus_arb_start(&a, 0U, 2U);
    bus_arb_rx(&a, 0xFFU); bus_arb_rx(&a, 0xFFU); bus_arb_rx(&a, FEE_BROADCAST_ID);
    check(a.state == BUS_ARB_IDLE, "broadcast cancels the wait");

    bus_arb_start(&a, 0U, 2U);
    bus_arb_rx(&a, 0xFFU); bus_arb_rx(&a, 0xFFU); bus_arb_rx(&a, 7U);
    check(a.state == BUS_ARB_WAIT, "another device's reply does not cancel it");

    bus_arb_start(&a, 0U, 2U);
    bus_arb_rx(&a, 0xFFU); bus_arb_rx(&a, 0xFFU); bus_arb_rx(&a, 0xFDU);
    bus_arb_rx(&a, 0x00U); bus_arb_rx(&a, 200U);
    check(a.state == BUS_ARB_IDLE, "dxl unicast to us cancels the wait");

    bus_arb_start(&a, 0U, 2U);
    bus_arb_rx(&a, 0xFFU); bus_arb_rx(&a, 0xFFU); bus_arb_rx(&a, 0xFDU);
    bus_arb_rx(&a, 0x00U); bus_arb_rx(&a, 9U);
    check(a.state == BUS_ARB_WAIT, "dxl reply from another device does not cancel it");
}

static void test_slave_gap_reset(void)
{
    fee_slave_t f;
    dxl_slave_t d;
    uint8_t out[DXL_MAX_RESP];
    uint8_t frame[FEE_MAX_FRAME];
    uint16_t n;

    g_case = "inter-byte gap reset - fee";
    reset_dev();
    fee_slave_init(&f);
    (void)fee_frame(200U, FEE_INST_PING, NULL, 0U, frame);
    (void)fee_slave_feed(&f, frame[0], 1000U, out, DXL_MAX_RESP);
    (void)fee_slave_feed(&f, frame[1], 1000U, out, DXL_MAX_RESP);
    check(f.n == 2U && f.active == 0U, "half a frame is buffered");
    fee_slave_reset(&f);
    check(f.n == 0U && f.need == 0U && f.active == 0U, "the gap rule drops it");
    n = fee_run(&f, 200U, FEE_INST_PING, NULL, 0U, out);
    check(n == 6U, "the next frame is parsed normally");

    g_case = "inter-byte gap reset - dxl";
    dxl_slave_init(&d);
    {
        uint8_t dframe[DXL_MAX_FRAME];
        (void)dxl_frame(200U, DXL_INST_PING, NULL, 0U, dframe);
        (void)dxl_slave_feed(&d, dframe[0], 1000U, out, DXL_MAX_RESP);
        (void)dxl_slave_feed(&d, dframe[1], 1000U, out, DXL_MAX_RESP);
        check(d.n == 2U, "half a frame is buffered");
        dxl_slave_reset(&d);
        check(d.n == 0U && d.active == 0U, "the gap rule drops it");
    }
    n = dxl_run(&d, 200U, DXL_INST_PING, NULL, 0U, out);
    check(n == 14U, "the next frame is parsed normally (PING: model+version)");
}

static void test_fee_reboot(void)
{
    fee_slave_t s;
    uint8_t out[DXL_MAX_RESP];
    uint16_t n;

    g_case = "fee 0x08 reboot";
    reset_dev();
    fee_slave_init(&s);
    g_reboot_calls = 0U;
    n = fee_run(&s, 200U, FEE_INST_REBOOT, NULL, 0U, out);
    check(n == 0U, "no answer to 0x08 (the real servo is silent; measured)");
    check(g_reboot_calls == 1U, "the node asked for a reboot");

    fee_slave_init(&s);
    g_reboot_calls = 0U;
    n = fee_run(&s, FEE_BROADCAST_ID, FEE_INST_REBOOT, NULL, 0U, out);
    check(n == 0U && g_reboot_calls == 1U, "broadcast 0x08 reboots silently");

    fee_slave_init(&s);
    g_reboot_calls = 0U;
    n = fee_run(&s, 9U, FEE_INST_REBOOT, NULL, 0U, out);
    check(n == 0U && g_reboot_calls == 0U, "0x08 addressed to another id is ignored");

    /* and the Dynamixel personality keeps answering then resetting (its
       protocol has a status packet for REBOOT, unlike FeeTech) */
    {
        dxl_slave_t d;
        g_case = "dxl 0x08 reboot";
        dxl_slave_init(&d);
        g_reboot_calls = 0U;
        n = dxl_run(&d, 200U, DXL_INST_REBOOT, NULL, 0U, out);
        check(n > 0U, "Dynamixel REBOOT answers first");
        check(g_reboot_calls == 1U, "and then asks for the reset");
    }
}

static void test_sim_gyro_scale(void)
{
    const uint8_t *b;
    int32_t gz;
    float dps;

    g_case = "sim gyro scale";
    reset_dev();
    g_cfg.sim_mode = SIM_SPIN;      /* 3 rad/s about trunk z */
    g_cfg.report_frame = 1U;        /* trunk frame: no mount rotation to undo */
    imu_sim_init();
    for (int i = 0; i < 10; i++) { imu_sim_step_10ms(); }
    b = imu_sim_block();
    gz = (int16_t)((uint16_t)b[4] | ((uint16_t)b[5] << 8));
    /* 3 rad/s = 57295.78 mdps/s -> / 17.5 mdps per LSB = 9822 counts */
    check(gz > 9700 && gz < 9950, "3 rad/s is ~9822 raw counts (scale caught a 1000x bug once)");
    dps = (float)gz * IMU_GYRO_MDPS_PER_LSB / 1000.0f;
    check(fabsf(dps - 171.887f) < 3.0f, "3 rad/s = 171.9 deg/s");
}

static void test_half(void)
{
    g_case = "half precision";
    check(f32_to_half(0.0f) == 0x0000U, "0");
    check(f32_to_half(1.0f) == 0x3C00U, "1");
    check(f32_to_half(-1.0f) == 0xBC00U, "-1");
    check(f32_to_half(0.5f) == 0x3800U, "0.5");
    check(fabsf(half_to_f32(0x3555U) - 0.333f) < 1e-3f, "0x3555 ~ 0.333");
    check(fabsf(half_to_f32(0x3000U) - 0.125f) < 1e-6f, "0x3000 = 0.125");
    check(fabsf(half_to_f32(f32_to_half(-0.75f)) + 0.75f) < 1e-6f, "round trip -0.75");
}

/*! \brief the SFLP FIFO tag is a 5-bit code in bits 7:3 of the tag byte.

    Table 217/218 of DS13510: TAG_SENSOR[4:0] sits in bits 7:3 and TAG_CNT[1:0]
    in bits 2:1, so the SFLP game rotation vector arrives as 0x98 / 0x9A / 0x9C /
    0x9E - the 5-bit code 0x13 shifted left by three.  The first implementation
    compared the *raw byte* against 0x13, which matched nothing and silently
    consumed no quaternion at all: the sensor answered, the FIFO drained, and
    the block simply stayed zero.  These cases pin the shift down on the host.
*/
static void test_lsm_fifo_tag(void)
{
    g_case = "lsm6dsv16x FIFO tag";

    /* the 5-bit code itself (Table 218) and its raw byte for TAG_CNT = 0 */
    check(LSM_TAG_SFLP_GAME == 0x13U, "SFLP game TAG_SENSOR code is 0x13");
    check((uint8_t)(LSM_TAG_SFLP_GAME << 3) == 0x98U, "TAG_CNT=0 raw byte is 0x98");

    /* every TAG_CNT variant still decodes to the same 5-bit sensor code */
    check(LSM_TAG_SENSOR(0x98) == LSM_TAG_SFLP_GAME, "0x98 -> SFLP game (TAG_CNT=0)");
    check(LSM_TAG_SENSOR(0x9A) == LSM_TAG_SFLP_GAME, "0x9A -> SFLP game (TAG_CNT=1)");
    check(LSM_TAG_SENSOR(0x9C) == LSM_TAG_SFLP_GAME, "0x9C -> SFLP game (TAG_CNT=2)");
    check(LSM_TAG_SENSOR(0x9E) == LSM_TAG_SFLP_GAME, "0x9E -> SFLP game (TAG_CNT=3)");

    /* gyro/accel-frame words must never be mistaken for the quaternion */
    check(LSM_TAG_SENSOR(0x0A) != LSM_TAG_SFLP_GAME, "0x0A (gyro NC) is not SFLP game");
    check(LSM_TAG_SENSOR(0x0C) != LSM_TAG_SFLP_GAME, "0x0C (gyro NC, TAG_CNT=1) is not SFLP");
    check(LSM_TAG_SENSOR(0x14) != LSM_TAG_SFLP_GAME, "0x14 (accel NC) is not SFLP game");
    check(LSM_TAG_SENSOR(0x16) != LSM_TAG_SFLP_GAME, "0x16 (SFLP gbias) is not SFLP game");

    /* the original mistake: comparing the raw byte to 0x13 fails */
    check(LSM_TAG_SENSOR(0x13) != LSM_TAG_SFLP_GAME, "raw 0x13 would never match");
}

static void test_sim_conventions(void)
{
    uint8_t blk_a[TELEM_LEN];
    uint8_t blk_b[TELEM_LEN];
    quat_t qa, qb;
    float wa[3], wb[3], aa[3], ab[3];
    float accel_rest[3];

    g_case = "sim static: gravity and accel in the trunk frame";
    reset_dev();
    g_cfg.sim_mode = SIM_STATIC;
    g_cfg.report_frame = 0U;
    imu_sim_init();
    for (int i = 0; i < 40; i++) { imu_sim_step_10ms(); }
    memcpy(blk_a, imu_sim_block(), TELEM_LEN);
    decode_block_as_host(blk_a, &qa, wa, aa);
    /* at rest the accelerometer must read ~1 g along trunk +z */
    check(fabsf(aa[0]) < 20.0f && fabsf(aa[1]) < 20.0f, "accel x/y ~ 0 mg");
    check(fabsf(aa[2] - 1000.0f) < 30.0f, "accel z ~ +1 g");
    check(fabsf(quat_norm(qa) - 1.0f) < 1e-4f, "trunk quaternion is unit");
    {
        float g_trunk[3];
        float down[3] = { 0.0f, 0.0f, -1.0f };
        quat_rotate_inv(qa, down, g_trunk);
        check(angle_between(g_trunk, down) < 0.05f, "upright gravity is [0,0,-1]");
    }

    g_case = "sim sine: gyro matches the quaternion derivative";
    g_cfg.sim_mode = SIM_SINE;
    imu_sim_init();
    for (int i = 0; i < 200; i++) { imu_sim_step_10ms(); }
    memcpy(blk_a, imu_sim_block(), TELEM_LEN);
    decode_block_as_host(blk_a, &qa, wa, aa);
    imu_sim_step_10ms();
    memcpy(blk_b, imu_sim_block(), TELEM_LEN);
    decode_block_as_host(blk_b, &qb, wb, ab);
    {
        /* omega from the quaternion delta: q' = q * exp(0.5 * w * dt), so
           q⁻¹ * q' carries the *trunk-frame* rate microduck consumes */
        quat_t dq = quat_mul(quat_conj(qa), qb);
        float w_rec[3];
        if (dq.w < 0.0f) {           /* keep the short way round */
            dq.w = -dq.w; dq.x = -dq.x; dq.y = -dq.y; dq.z = -dq.z;
        }
        w_rec[0] = 2.0f * dq.x / 0.01f;
        w_rec[1] = 2.0f * dq.y / 0.01f;
        w_rec[2] = 2.0f * dq.z / 0.01f;
        check(angle_between(w_rec, wb) < 0.15f
              || (sqrtf(w_rec[0] * w_rec[0] + w_rec[1] * w_rec[1] + w_rec[2] * w_rec[2]) < 1e-3f
                  && sqrtf(wb[0] * wb[0] + wb[1] * wb[1] + wb[2] * wb[2]) < 1e-3f),
              "gyro is consistent with the quaternion it ships with");
    }

    g_case = "sim accel at rest in the chip frame needs the mount to come back";
    g_cfg.sim_mode = SIM_STATIC;
    imu_sim_init();
    for (int i = 0; i < 40; i++) { imu_sim_step_10ms(); }
    memcpy(blk_a, imu_sim_block(), TELEM_LEN);
    decode_block_as_host(blk_a, &qa, wa, accel_rest);
    check(fabsf(accel_rest[2] - 1000.0f) < 30.0f, "mount rotation applied by the host");

    g_case = "sim sflp-wait leaves the quaternion empty";
    g_cfg.sim_mode = SIM_SFLP_WAIT;
    g_cfg.report_frame = 0U;
    imu_sim_init();
    for (int i = 0; i < 10; i++) { imu_sim_step_10ms(); }
    memcpy(blk_a, imu_sim_block(), TELEM_LEN);
    check(blk_a[6] == 0U && blk_a[7] == 0U && blk_a[8] == 0U && blk_a[9] == 0U
          && blk_a[10] == 0U && blk_a[11] == 0U, "all-zero quaternion bytes");
    check((blk_a[19] & TELEM_FLAG_SFLP_VALID) == 0U, "SFLP valid flag clear");
    check((blk_a[19] & TELEM_FLAG_SIMULATED) != 0U, "simulated flag set");

    g_case = "sim frozen repeats itself";
    g_cfg.sim_mode = SIM_FROZEN;
    imu_sim_init();
    for (int i = 0; i < 5; i++) { imu_sim_step_10ms(); }
    memcpy(blk_a, imu_sim_block(), TELEM_LEN);
    check(blk_a[19] == (TELEM_FLAG_SIMULATED | TELEM_FLAG_FROZEN), "frozen flags");
    for (int i = 0; i < 30; i++) { imu_sim_step_10ms(); }
    memcpy(blk_b, imu_sim_block(), TELEM_LEN);
    check(memcmp(blk_a, blk_b, TELEM_LEN) == 0, "identical block on every read");

    g_case = "sim trunk-frame report mode";
    g_cfg.sim_mode = SIM_STATIC;
    g_cfg.report_frame = 1U;
    imu_sim_init();
    for (int i = 0; i < 40; i++) { imu_sim_step_10ms(); }
    memcpy(blk_a, imu_sim_block(), TELEM_LEN);
    {
        /* in trunk mode the host must NOT apply the mount: gravity comes out
           directly from the raw quaternion */
        quat_t mount_inv = { MOUNT_QW, -MOUNT_QX, -MOUNT_QY, -MOUNT_QZ };
        uint16_t hx = (uint16_t)blk_a[6] | ((uint16_t)blk_a[7] << 8);
        float x = half_to_f32(hx);
        float y = half_to_f32((uint16_t)blk_a[8] | ((uint16_t)blk_a[9] << 8));
        float z = half_to_f32((uint16_t)blk_a[10] | ((uint16_t)blk_a[11] << 8));
        quat_t qraw = { sqrtf(fmaxf(0.0f, 1.0f - (x * x + y * y + z * z))), x, y, z };
        quat_t wrong = quat_mul(qraw, mount_inv);
        float down[3] = { 0.0f, 0.0f, -1.0f };
        float g_wrong[3];
        (void)wrong;
        quat_rotate_inv(qraw, down, g_wrong);
        check(angle_between(g_wrong, down) < 0.05f, "trunk-mode quaternion is already trunk");
    }
}

int main(void)
{
    printf("protocol + simulator host tests\n");
    test_half();
    test_lsm_fifo_tag();
    test_dxl_ping();
    test_dxl_read();
    test_dxl_sync_read();
    test_dxl_fast_ref();
    test_dxl_fast_sync_read();
    test_dxl_errors();
    test_stuffing();
    test_dxl_status_stuffing();
    test_fee_ping_read();
    test_fee_sync();
    test_fee_reboot();
    test_telem_pack();
    test_sync_index();
    test_bus_arb();
    test_slave_gap_reset();
    test_sim_gyro_scale();
    test_sim_conventions();

    if (g_failures != 0) {
        printf("\n%d FAILURE(S)\n", g_failures);
        return 1;
    }
    printf("all host protocol tests pass\n");
    return 0;
}
