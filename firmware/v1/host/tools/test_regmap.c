/*!
    \file    test_regmap.c
    \brief   host tests for the register images: the native golden, the FT6
             personality, and the read-time contract refresh

    Unlike test_protocols.c (which stubs the register layer), this binary runs
    the *real* `src/dev.c` together with `src/dev_cfg.c`, `src/cfg_blob.c`,
    `src/app_header.c`, `src/boot_flash.c` (RAM-backed in host mode),
    `src/imu.c` + `src/imu_sim.c` and `src/telem_pack.c`.  The 256-byte images
    asserted here are therefore the bytes the firmware builds, without a board.

    Two personalities share this one test binary, selected by the same
    `FEE_IMU_PERSONALITY_FT6` define the firmware uses:

      * `run_c_tests.sh` builds it twice, once per personality;
      * the native build must reproduce `--dump` output captured from the code
        *before* the personality switch existed, byte for byte (`GOLDEN_*`);
      * the FT6 build must do the same for the Dynamixel image - that is the
        "FT6 must not disturb protocol 2.0" requirement, asserted, not reviewed.

    `--dump` prints the two images as hex and nothing else, so the golden can be
    regenerated deliberately with
        host/tools/run_c_tests.sh --regen-regmap-golden
    (see the script) and reviewed as a diff.
*/

#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "board.h"
#include "boot_flash.h"
#include "dev.h"
#include "imu.h"
#include "imu_math.h"
#include "imu_sim.h"
#include "telem_pack.h"

#include <math.h>

/* ── platform stubs (the same ones test_boot.c uses) ────────────────────── */

static uint32_t g_now_ms;

uint32_t systick_get_ms(void);
uint32_t systick_get_ms(void)
{
    return g_now_ms;
}

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

/* ── the golden images ──────────────────────────────────────────────────── */

/* The Dynamixel image is personality-independent by definition, so this golden
   is asserted in both builds - in the FT6 build it *is* the "protocol 2.0 is
   untouched" requirement.  The FeeTech golden only applies to the native build:
   the FT6 map is asserted structurally below. */
static const char GOLDEN_DXL[] =
    "B0040000000010C803000003FF02000000000000000000000A00000000000046"
    "460023007503D60600000000BD010000FF0F0000000000000000000000008C35"
    "0000000002000000000000004006B40000000000900100000000000000000000"
    "000000000000000000000000000000000000000000000000DC050000D80231FF"
    "70002330CA399D2F10E2090B64FD319100001E00494D0001E803E80300000000"
    "0000000000020400E000E100E200E300E400E500555000FF8000100000000001"
    "FFFF0000EE00EF00F000F100F200F300F400F500F600F700F800F900FA00FB00"
    "0000000000000000000000000000000000000000000000000000000000000000";
#if !FEE_IMU_PERSONALITY_FT6
static const char GOLDEN_FEE_NATIVE[] =
    "010000494DC800FE010000FF0F00000000000000000000000000000000000000"
    "000000000000000000000000000000000000000000000000D80231FF70002330"
    "CA399D2F31910000000000000000000000000000000000000000000000000000"
    "0000000000000000000000000000000000000000000000000000000000000000"
    "D80231FF70002330CA399D2F10E2090B64FD3191000000000000000000000000"
    "494D0001E803E803000000000000000000020400555000FF8000100000000001"
    "FFFF000000000000000000000000000000000000000000000000000000000000"
    "0000000000000000000000000000000000000000000000000000000000000000";
#endif

/* ── plumbing ───────────────────────────────────────────────────────────── */

static int g_failures;
static const char *g_case;

static void check(int cond, const char *what)
{
    if (!cond) {
        printf("  FAIL [%s] %s\n", g_case, what);
        g_failures++;
    }
}

static void hex_line(const uint8_t *img, char *out)
{
    static const char digits[] = "0123456789ABCDEF";
    int i;

    for (i = 0; i < REG_SPACE_SIZE; i++) {
        out[2 * i] = digits[(img[i] >> 4) & 0xF];
        out[2 * i + 1] = digits[img[i] & 0xF];
    }
    out[2 * REG_SPACE_SIZE] = '\0';
}

static int hex_eq(const uint8_t *img, const char *golden)
{
    char got[2 * REG_SPACE_SIZE + 1];

    if ('\0' == golden[0]) {
        return 1;                 /* not captured yet: nothing to compare */
    }
    hex_line(img, got);
    return (0 == strcmp(got, golden));
}

static void dump_image(const char *tag, const uint8_t *img)
{
    char line[2 * REG_SPACE_SIZE + 1];

    hex_line(img, line);
    printf("%s %s\n", tag, line);
}

/* ── the firmware's own start-up and publish sequence ───────────────────── */

/* src/main.c: dev_init(); imu_init(); dev_refresh(); then one 10 ms step and a
   publish per loop iteration.  Reproduced exactly, because the golden is only
   meaningful if the schedule that produced it is the schedule a board runs. */
static void boot(void)
{
    boot_flash_test_reset();
    g_now_ms = 1000U;
    dev_init();
    imu_init();
    dev_refresh();
}

static void step_ms(uint32_t n)
{
    uint32_t i;

    for (i = 0U; i < n; i++) {
        g_now_ms += IMU_SAMPLE_PERIOD_MS;
        imu_tick(g_now_ms);
        dev_refresh();
    }
}

/* Advance the clock without touching the sensor: the main loop is stalled (a
   breakpoint, an erase, a hung I2C read), but the USART keeps answering. */
#if FEE_IMU_PERSONALITY_FT6

static void stall_ms(uint32_t n)
{
    g_now_ms += n;
}

#endif

static const uint8_t *fee_img(void)
{
    return dev_image(PROTO_FEE);
}

static const uint8_t *dxl_img(void)
{
    return dev_image(PROTO_DXL);
}

#if FEE_IMU_PERSONALITY_FT6

static void read_fee15(uint8_t *out)
{
    (void)dev_read(PROTO_FEE, FEE_TELEM_ADDR, FEE_BLOCK_LEN, out);
}

#endif

/* ── 1. the images the node publishes ───────────────────────────────────── */

/* `boot()` runs exactly once per process: the firmware never re-initialises,
   and `imu_tick()` keeps its 10 ms grid in a function-local static, so a second
   boot would rewind the clock under a grid that is still ahead of it.  The
   tests below therefore share one start-up and one timeline, like a board. */
#if !FEE_IMU_PERSONALITY_FT6

static void test_golden_native(void)
{
    g_case = "golden: native images";

    check(hex_eq(dxl_img(), GOLDEN_DXL),
          "the Dynamixel image 0..255 is byte-identical to the baseline");
    check(hex_eq(fee_img(), GOLDEN_FEE_NATIVE),
          "the FeeTech image 0..255 is byte-identical to the baseline");
}

#endif

static void test_dxl_independent_of_personality(void)
{
    g_case = "personality: Dynamixel image";

    /* In the FT6 build GOLDEN_DXL is the *native* image, so this is the
       requirement itself: switching the FeeTech personality must not move one
       byte of protocol 2.0. */
    check(hex_eq(dxl_img(), GOLDEN_DXL),
          "the Dynamixel image does not depend on FEE_IMU_PERSONALITY");
}

/* ── 2. the FT6 FeeTech map ─────────────────────────────────────────────── */

#if FEE_IMU_PERSONALITY_FT6

static void test_ft6_map(void)
{
    uint8_t buf[TELEM_LEN];
    uint8_t ctrl[TELEM_LEN];
    const uint8_t *fee;
    const uint8_t *blk;
    int i;

    g_case = "ft6: register map";
    fee = fee_img();
    blk = imu_block_bytes();
    /* the FeeTech projection of the block: the chip frame plus this
       personality's pre-rotation (identity by default) */
    imu_fee_control_block(ctrl);

    /* identity 0..6, matching kissqy's fixed table {6,0,0,0,0xf2,FT_IMU_ID,0} */
    check(fee[0] == 6U, "register 0 reports firmware major 6");
    check(fee[1] == 0U, "register 1 reports firmware minor 0");
    check(fee[2] == 0U, "register 2 stays zero");
    check(fee[3] == 0x00U && fee[4] == 0xF2U, "registers 3/4 report model 0xF200");
    check(fee[5] == (uint8_t)FEE_ID_DEFAULT, "register 5 reports the node id");
    check(fee[6] == 0U, "register 6 stays zero (baud code 0 at 1 Mbps)");

    /* 56..70: 12 control bytes identical to native, then the FT6 counter and
       status, then a reserved zero */
    (void)dev_read(PROTO_FEE, FEE_TELEM_ADDR, FEE_BLOCK_LEN, buf);
    check(memcmp(buf, ctrl, TELEM_CTRL_LEN) == 0, "bytes 56..67 are the control block");
    check(buf[FEE_BLOCK_RESERVED] == 0U, "byte 70 is reserved and zero");
    check((buf[FEE_BLOCK_STATUS] & 0xF0U) == 0U, "status bits 4..7 are always zero");
    check((buf[FEE_BLOCK_STATUS] & 0x07U) == 0U, "a live simulator reports ready");
    check(buf[FEE_BLOCK_CNT] == (uint8_t)imu_quat_seq(),
          "byte 68 is the quaternion sequence, not the sample counter");

    /* 124..143: the FT5/FT6 diagnostic block.  Note it *overlaps* the native
       128..147 alias in 128..143, which is exactly why the native 20-byte block
       has to move to 196 in this personality. */
    (void)dev_read(PROTO_FEE, FEE_DIAG_ADDR, FEE_DIAG_LEN, buf);
    check(memcmp(buf, ctrl, TELEM_CTRL_LEN) == 0, "124..135 are the control block");
    check(memcmp(&fee[FEE_DIAG_ADDR], buf, FEE_DIAG_LEN) == 0,
          "the whole 124..143 window is the diagnostic block");
    check(buf[19] == FEE_DIAG_SCHEMA, "byte 143 is schema = 1");
    check((buf[18] & (uint8_t)~1U) == 0U, "byte 142 is ready 0 or 1");
    check(((uint32_t)buf[12] | ((uint32_t)buf[13] << 8)
           | ((uint32_t)buf[14] << 16) | ((uint32_t)buf[15] << 24)) == imu_quat_seq(),
          "bytes 136..139 are the sequence, u32 little endian");
    check(((uint32_t)buf[16] | ((uint32_t)buf[17] << 8)) == (uint32_t)imu_quat_age_ms(),
          "bytes 140..141 are the age, u16 little endian");

    /* the native 20-byte block moved to 196..215 and still holds the whole
       diagnostic tail */
    check(memcmp(&fee[FEE_TELEM_NATIVE_ADDR], blk, TELEM_LEN) == 0,
          "196..215 carry the native 20-byte block");
    check(memcmp(&fee[FEE_TELEM_NATIVE_ADDR], &dxl_img()[DXL_TELEM_ADDR], TELEM_LEN) == 0,
          "and it is the same block the Dynamixel map serves at 124");

    /* the windows the FT6 layout leaves alone: 144..147 is the only part of the
       old native alias that is not covered by the diagnostic block */
    for (i = FEE_DIAG_ADDR + FEE_DIAG_LEN; i <= 147; i++) {
        check(fee[i] == 0U, "144..147 are unused in the FT6 layout");
        if (fee[i] != 0U) { break; }
    }
    for (i = 71; i <= 123; i++) {
        check(fee[i] == 0U, "71..123 stay empty");
        if (fee[i] != 0U) { break; }
    }
    for (i = 216; i < REG_SPACE_SIZE; i++) {
        check(fee[i] == 0U, "216..255 stay empty");
        if (fee[i] != 0U) { break; }
    }
}

/* ── 3. the contract block is refreshed when it is read ─────────────────── */

static void set_sim_mode(uint8_t mode)
{
    check(0U == dev_write(PROTO_FEE, (uint16_t)(FEE_VENDOR_ADDR + V_SIM_MODE), 1U, &mode),
          "SIM_MODE is writable through the vendor window");
    step_ms(1U);
}

static void test_read_time_refresh(void)
{
    uint8_t first[FEE_BLOCK_LEN];
    uint8_t second[FEE_BLOCK_LEN];
    uint8_t saved[2];
    uint8_t throwaway[TELEM_LEN];

    g_case = "ft6: contract refresh at read";

    /* this runs before any other test has touched dev_read(), so the very first
       contract read of the session is the one asserted below */
    /* the first read must not claim the reader is slow (nothing to compare) */
    read_fee15(first);
    check((first[FEE_BLOCK_STATUS] & FEE_ST_READER_SLOW) == 0U,
          "the first contract read does not set READER_SLOW");

    /* a new quaternion per 10 ms step: three steps must move the sequence */
    step_ms(3U);
    read_fee15(second);
    check(second[FEE_BLOCK_CNT] != first[FEE_BLOCK_CNT],
          "the sequence advances between reads while the sensor is live");
    check((second[FEE_BLOCK_STATUS] & 0x07U) == 0U, "and the block is ready");

    /* five or more new samples between two reads is the FT6 READER_SLOW rule */
    step_ms(5U);
    read_fee15(first);
    check((first[FEE_BLOCK_STATUS] & FEE_ST_READER_SLOW) != 0U,
          "five unread samples set READER_SLOW");
    check((first[FEE_BLOCK_STATUS] & 0x07U) == 0U,
          "READER_SLOW alone does not make the block unreachable");

    /* A stalled main loop: the ISR keeps answering, but the age must be bounded
       at transmission, otherwise the node reports freshness forever.  This is
       the whole reason the status byte exists. */
    stall_ms(100U);
    read_fee15(second);
    check((second[FEE_BLOCK_STATUS] & FEE_ST_STALE) != 0U,
          "a 100 ms stall sets STALE at read time");
    check((second[FEE_BLOCK_STATUS] & 0x07U) != 0U,
          "and the host's ready test (bits 0..2) now refuses the block");
    check((second[FEE_BLOCK_STATUS] & 0xF0U) == 0U, "while bits 4..7 stay zero");
    check(second[FEE_BLOCK_RESERVED] == 0U, "and byte 70 stays zero");

    /* the diagnostic block carries the same age, refreshed the same way */
    (void)dev_read(PROTO_FEE, FEE_DIAG_ADDR, FEE_DIAG_LEN, throwaway);
    check((((uint32_t)throwaway[16] | ((uint32_t)throwaway[17] << 8))) >= 100U,
          "the 124/20 age reflects the stall too");
    check(throwaway[19] == 1U, "124/20 keeps schema = 1");

    /* reading the Dynamixel map must not publish into the FeeTech image */
    step_ms(1U);
    saved[0] = fee_img()[FEE_TELEM_ADDR + FEE_BLOCK_CNT];
    saved[1] = fee_img()[FEE_TELEM_ADDR + FEE_BLOCK_STATUS];
    stall_ms(100U);
    (void)dev_read(PROTO_DXL, DXL_TELEM_ADDR, TELEM_LEN, throwaway);
    (void)dev_read(PROTO_DXL, FEE_TELEM_ADDR, FEE_BLOCK_LEN, throwaway);
    check(fee_img()[FEE_TELEM_ADDR + FEE_BLOCK_CNT] == saved[0]
          && fee_img()[FEE_TELEM_ADDR + FEE_BLOCK_STATUS] == saved[1],
          "a Dynamixel read does not refresh the FeeTech contract block");

    /* frozen simulator: the block stops advancing and says so */
    set_sim_mode((uint8_t)SIM_FROZEN);
    step_ms(2U);
    read_fee15(first);
    step_ms(2U);
    read_fee15(second);
    check(first[FEE_BLOCK_CNT] == second[FEE_BLOCK_CNT],
          "a frozen block stops advancing the sequence");
    check((second[FEE_BLOCK_STATUS] & 0x07U) != 0U,
          "and reports not-ready or stale");
    check((second[FEE_BLOCK_STATUS] & 0xF0U) == 0U, "with bits 4..7 still zero");

    /* and it recovers when the sensor comes back */
    set_sim_mode((uint8_t)SIM_SINE);
    step_ms(3U);
    read_fee15(second);
    check(second[FEE_BLOCK_CNT] != first[FEE_BLOCK_CNT],
          "the sequence advances again once the sensor recovers");
    check((second[FEE_BLOCK_STATUS] & 0x07U) == 0U, "and the block is ready again");
}

#else /* native */

static void test_native_contract(void)
{
    uint8_t buf[FEE_BLOCK_LEN];

    g_case = "native: contract block";

    (void)dev_read(PROTO_FEE, FEE_TELEM_ADDR, FEE_BLOCK_LEN, buf);
    check(memcmp(buf, imu_block_bytes(), TELEM_CTRL_LEN) == 0, "56..67 are the control block");
    check(buf[FEE_BLOCK_CNT] == imu_block_bytes()[TELEM_CNT],
          "byte 68 is the native sample counter");
    check(buf[FEE_BLOCK_STATUS] == imu_block_bytes()[TELEM_STATUS],
          "byte 69 is the native flags byte");
    check(buf[FEE_BLOCK_RESERVED] == 0U, "byte 70 is reserved and zero");
    check(memcmp(&fee_img()[FEE_TELEM_ALT_ADDR], imu_block_bytes(), TELEM_LEN) == 0,
          "128..147 keep the native 20-byte block");
    check(fee_img()[124] == 0U, "124 is unused in the native layout");
}

#endif

#if FEE_IMU_PERSONALITY_FT6

/* The FT6 host applies the mounting rotation itself, so the node must report
   chip frame: trunk-frame mode (this repository's bench convenience) would
   rotate the attitude twice.  The write is accepted and ignored. */
static void test_ft6_forces_chip_frame(void)
{
    uint8_t one = 1U;

    g_case = "ft6: report_frame";
    check(0U == dev_cfg()->report_frame, "chip frame is the only mode at start-up");
    check(0U == dev_write(PROTO_FEE, (uint16_t)(FEE_VENDOR_ADDR + V_REPORT_FRAME), 1U, &one),
          "writing V_REPORT_FRAME is accepted");
    step_ms(1U);
    check(0U == dev_cfg()->report_frame, "but the ft6 personality stays in chip frame");
    check(fee_img()[FEE_VENDOR_ADDR + V_REPORT_FRAME] == 0U,
          "and the vendor window reads back what is actually reported");
}

#endif /* FEE_IMU_PERSONALITY_FT6 */

#if FEE_IMU_PERSONALITY_FT6

/* The FeeTech-side pre-rotation (FEE_IMU_PREMOUNT_*).  Whatever it is set to,
   three things must hold: the rotation seen by a host is exactly
   receiver(published) = receiver(raw) * C, the diagnostic tail is untouched,
   and the raw chip frame still reaches the Dynamixel map and the 196 alias. */
static quat_t rx_quat(const uint8_t *p)
{
    float x = half_to_f32((uint16_t)((uint16_t)p[6] | ((uint16_t)p[7] << 8)));
    float y = half_to_f32((uint16_t)((uint16_t)p[8] | ((uint16_t)p[9] << 8)));
    float z = half_to_f32((uint16_t)((uint16_t)p[10] | ((uint16_t)p[11] << 8)));
    quat_t q;

    q.w = sqrtf(fmaxf(0.0f, 1.0f - (x * x + y * y + z * z)));
    q.x = x; q.y = y; q.z = z;
    return quat_normalize(q);
}

/*! \brief the same rotation, allowing for the q / -q ambiguity */
static int same_rotation(quat_t a, quat_t b)
{
    float plus = (a.w - b.w) * (a.w - b.w) + (a.x - b.x) * (a.x - b.x)
               + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z);
    float minus = (a.w + b.w) * (a.w + b.w) + (a.x + b.x) * (a.x + b.x)
                + (a.y + b.y) * (a.y + b.y) + (a.z + b.z) * (a.z + b.z);

    /* the published components are binary16, so allow a little rounding: the
       threshold is ~1 degree, far below any real composition error */
    return (fminf(plus, minus) < 1e-4f);
}

static void test_ft6_premount(void)
{
    const uint8_t *raw = imu_block_bytes();
    uint8_t out[TELEM_LEN];
    uint8_t buf[FEE_BLOCK_LEN];
    quat_t c;
    int identity;

    g_case = "ft6: chip-frame pre-rotation";
    imu_fee_control_block(out);
    (void)dev_read(PROTO_FEE, FEE_TELEM_ADDR, FEE_BLOCK_LEN, buf);

    c.w = FEE_IMU_PREMOUNT_QW;
    c.x = FEE_IMU_PREMOUNT_QX;
    c.y = FEE_IMU_PREMOUNT_QY;
    c.z = FEE_IMU_PREMOUNT_QZ;
    c = quat_normalize(c);
    /* the macros are floating constants, so this is a runtime comparison */
    identity = (fabsf(c.w - 1.0f) < 1e-6f) && (fabsf(c.x) < 1e-6f)
               && (fabsf(c.y) < 1e-6f) && (fabsf(c.z) < 1e-6f);

    check(memcmp(&out[TELEM_CTRL_LEN], &raw[TELEM_CTRL_LEN], TELEM_LEN - TELEM_CTRL_LEN) == 0,
          "the diagnostic tail is copied, not rotated");
    check(memcmp(buf, out, TELEM_CTRL_LEN) == 0,
          "the contract block publishes exactly the rotated control bytes");
    if (identity) {
        check(memcmp(out, raw, TELEM_CTRL_LEN) == 0,
              "the identity pre-rotation publishes the raw chip frame");
    } else {
        check(memcmp(out, raw, TELEM_CTRL_LEN) != 0,
              "a configured pre-rotation changes the published frame");
    }

    /* the whole point: a host decoding the published bytes sees raw * C */
    check(same_rotation(rx_quat(out), quat_mul(rx_quat(raw), c)),
          "the published attitude is the raw chip frame composed with the pre-rotation");

    /* gyro: C g C^-1, compared axis by axis */
    {
        float g[3], want[3], got[3];
        int i;

        for (i = 0; i < 3; i++) {
            uint16_t u = (uint16_t)((uint16_t)out[2 * i] | ((uint16_t)out[2 * i + 1] << 8));
            got[i] = (float)(int16_t)u;
            u = (uint16_t)((uint16_t)raw[2 * i] | ((uint16_t)raw[2 * i + 1] << 8));
            g[i] = (float)(int16_t)u;
        }
        quat_rotate(c, g, want);
        for (i = 0; i < 3; i++) {
            check(fabsf(got[i] - want[i]) < 1.5f, "the published gyro is C g C^-1");
        }
    }

    /* the raw chip frame still reaches the Dynamixel map and the 196 alias */
    check(memcmp(&fee_img()[FEE_TELEM_NATIVE_ADDR], raw, TELEM_LEN) == 0,
          "196 still carries the raw chip frame");
    check(memcmp(&dxl_img()[DXL_TELEM_ADDR], raw, TELEM_LEN) == 0,
          "and so does Dynamixel 124");

    /* "no attitude yet" must survive the rotation: SIM_SFLP_WAIT publishes six
       zero quaternion bytes, and rotating those would fabricate an attitude */
    set_sim_mode((uint8_t)SIM_SFLP_WAIT);
    step_ms(3U);
    (void)dev_read(PROTO_FEE, FEE_TELEM_ADDR, FEE_BLOCK_LEN, buf);
    check((buf[6] | buf[7] | buf[8] | buf[9] | buf[10] | buf[11]) == 0U,
          "an all-zero quaternion stays all zero after the pre-rotation");
    set_sim_mode((uint8_t)SIM_SINE);
    step_ms(3U);
}

#endif /* FEE_IMU_PERSONALITY_FT6 */

/* ── main ───────────────────────────────────────────────────────────────── */

int main(int argc, char **argv)
{
    if (argc > 1 && 0 == strcmp(argv[1], "--dump")) {
        boot();
        step_ms(50U);
        dump_image("DXL", dxl_img());
        dump_image("FEE", fee_img());
        return 0;
    }

    printf("register image + personality tests (%s)\n",
#if FEE_IMU_PERSONALITY_FT6
           "ft6"
#else
           "native"
#endif
    );

    boot();
    step_ms(50U);

#if FEE_IMU_PERSONALITY_FT6
    test_dxl_independent_of_personality();
    test_read_time_refresh();
    test_ft6_map();
    test_ft6_forces_chip_frame();
    test_ft6_premount();
#else
    test_dxl_independent_of_personality();
    test_golden_native();
    test_native_contract();
#endif

    if (0 != g_failures) {
        printf("\n%d FAILURE(S)\n", g_failures);
        return 1;
    }
    printf("ok\n");
    return 0;
}
