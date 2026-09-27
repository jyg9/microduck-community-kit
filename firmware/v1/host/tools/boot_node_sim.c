/*!
    \file    boot_node_sim.c
    \brief   a PC stand-in for the node in boot mode, so the Linux upgrade tool
             can be tested end to end without hardware

    It links the *real* bootloader modules - `boot_regs.c`, `boot_upgrade.c`,
    `boot_flash.c` (RAM-backed), `cfg_blob.c`, `app_header.c` - plus the real
    protocol slaves `dxl2.c` / `fee.c`, and talks them over a serial device or
    a file descriptor.  The only thing it re-implements is `src/bus.c`'s
    two-byte-prefix protocol detection, because `bus.c` is written against the
    GD32 UART driver (the rule is copied verbatim below and marked).

    Usage (normally driven by `host/tools/test_upgrade.py`):

      boot_node_sim --fd 7 --dump-dir /tmp/sim [--run-slot a] [--other-bootable]
                    [--preload-slot b:1.0.0] [--seconds 60]

    On exit it writes `slot_a.bin`, `slot_b.bin` and `cfg.bin` into the dump
    directory, so the test can check what actually landed in "flash".
*/

#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include "app_header.h"
#include "boot_flash.h"
#include "boot_regs.h"
#include "boot_upgrade.h"
#include "board.h"
#include "bus.h"
#include "dbg.h"
#include "dxl2.h"
#include "fee.h"

static int      s_fd = -1;
static uint32_t s_now_ms;
static int      s_verbose;

/* ── the clock the node sees ────────────────────────────────────────────── */

uint32_t systick_get_ms(void);
uint32_t systick_get_ms(void)
{
    return s_now_ms;
}

static uint32_t mono_ms(void)
{
    struct timespec ts;

    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)((uint64_t)ts.tv_sec * 1000U + (uint64_t)(ts.tv_nsec / 1000000));
}

/* ── console off ────────────────────────────────────────────────────────── */

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

/* ── the real firmware's logging hooks, observed by the test ───────────── */

static int     s_blocks;
static int     s_commits;
static uint8_t s_commit_slot;

static void on_block(uint32_t offset, uint8_t len, uint16_t seq, int retransmit)
{
    (void)offset;
    (void)len;
    (void)seq;
    (void)retransmit;
    s_blocks++;
}

static void on_commit(uint8_t slot)
{
    s_commits++;
    s_commit_slot = slot;
    if (NULL != boot_regs_hooks()->on_commit) {
        boot_regs_hooks()->on_commit(slot);
    }
}

static void on_reboot(void)
{
    if (NULL != boot_regs_hooks()->on_reboot) {
        boot_regs_hooks()->on_reboot();
    }
}

static const upg_hooks_t s_hooks = { on_commit, on_reboot, on_block };

/* ── serial plumbing ───────────────────────────────────────────────────── */

static void set_raw(int fd)
{
    struct termios t;

    if (tcgetattr(fd, &t) != 0) {
        return;
    }
    cfmakeraw(&t);
    cfsetispeed(&t, B115200);
    cfsetospeed(&t, B115200);
    (void)tcsetattr(fd, TCSANOW, &t);
}

static int open_device(const char *path)
{
    int fd = open(path, O_RDWR | O_NOCTTY);

    if (fd < 0) {
        fprintf(stderr, "sim: cannot open %s: %s\n", path, strerror(errno));
        return -1;
    }
    set_raw(fd);
    return fd;
}

static void write_all(const uint8_t *buf, uint16_t len)
{
    uint16_t off = 0U;

    while (off < len) {
        ssize_t n = write(s_fd, &buf[off], (size_t)(len - off));
        if (n <= 0) {
            if (errno == EINTR) {
                continue;
            }
            return;
        }
        off = (uint16_t)(off + (uint16_t)n);
    }
}

/* ── preloading a slot, so "the last good image" is protected ──────────── */

static void preload_slot(const char *spec)
{
    /* "b:1.0.0" */
    char copy[32];
    char *colon;
    uint8_t slot = SLOT_A;
    unsigned major = 1U;
    unsigned minor = 0U;
    unsigned patch = 0U;
    static uint8_t image[SLOT_SIZE];
    uint32_t size = 0x1200U;
    uint32_t i;
    uint32_t base;
    uint16_t version;
    int rc;

    snprintf(copy, sizeof(copy), "%s", spec);
    colon = strchr(copy, ':');
    if (NULL != colon) {
        *colon = '\0';
        if (3 == sscanf(colon + 1, "%u.%u.%u", &major, &minor, &patch)) {
            /* fine */
        }
    }
    slot = ('b' == copy[0] || 'B' == copy[0]) ? SLOT_B : SLOT_A;
    version = APP_VERSION_MAKE(major, minor, patch);
    base = boot_flash_slot_base(slot);

    memset(image, 0xFF, size);
    memset(image, 0, 16U);
    {
        uint32_t sp = 0x2000BF00U;
        uint32_t entry = base + 0x241U;
        memcpy(&image[0], &sp, 4U);
        memcpy(&image[4], &entry, 4U);
    }
    for (i = 0x240U; i < size; i++) {
        image[i] = (uint8_t)((i * 7U + slot) & 0xFFU);
    }
    rc = app_header_fill(image, size, version, 0U, 0U);
    if (APP_HDR_OK != rc) {
        fprintf(stderr, "sim: preload failed: %s\n", app_hdr_strerror(rc));
        return;
    }
    boot_flash_test_load(base, image, size);
    printf("SIM: preloaded slot %c v%u.%u.%u (%u bytes)\n",
           (slot == SLOT_A) ? 'A' : 'B', major, minor, patch, (unsigned)size);
}

static void dump_flash(const char *dir)
{
    static const struct {
        const char *name;
        uint32_t addr;
        uint32_t len;
    } parts[] = {
        { "slot_a.bin", SLOT_A_BASE, SLOT_SIZE },
        { "slot_b.bin", SLOT_B_BASE, SLOT_SIZE },
        { "cfg.bin", CFG_FLASH_ADDR, CFG_FLASH_SIZE },
    };
    const uint8_t *base = boot_flash_test_base();
    char path[512];
    size_t i;

    for (i = 0U; i < sizeof(parts) / sizeof(parts[0]); i++) {
        FILE *handle;

        snprintf(path, sizeof(path), "%s/%s", dir, parts[i].name);
        handle = fopen(path, "wb");
        if (NULL == handle) {
            fprintf(stderr, "sim: cannot write %s: %s\n", path, strerror(errno));
            continue;
        }
        (void)fwrite(base + (parts[i].addr - FLASH_BASE_ADDR), 1U, parts[i].len, handle);
        (void)fclose(handle);
    }
    printf("SIM: dumped slot_a.bin slot_b.bin cfg.bin to %s\n", dir);
}

/* ── protocol dispatch: the rule from src/bus.c, inlined ────────────────── */

static dxl_slave_t s_dxl;
static fee_slave_t s_fee;

static void feed(uint8_t byte)
{
    static uint8_t  last_proto;
    static uint32_t last_ms;
    static uint8_t  out[DXL_MAX_RESP];
    uint8_t allow_dxl = 1U;
    uint8_t allow_fee = 1U;
    uint16_t n;

    if (PROTO_NONE != last_proto
        && (uint32_t)(s_now_ms - last_ms) < BUS_PROTO_STICKY_MS) {
        allow_dxl = (uint8_t)(PROTO_DXL == last_proto);
        allow_fee = (uint8_t)(PROTO_FEE == last_proto);
    }

    if (0U != allow_dxl) {
        n = dxl_slave_feed(&s_dxl, byte, s_now_ms, out, (uint16_t)sizeof(out));
        if (0U != n) {
            if (s_verbose) {
                fprintf(stderr, "sim: dxl -> %u bytes\n", (unsigned)n);
            }
            write_all(out, n);
            last_proto = PROTO_DXL;
            last_ms = s_now_ms;
            return;
        }
    }
    if (0U != allow_fee) {
        n = fee_slave_feed(&s_fee, byte, s_now_ms, out, (uint16_t)sizeof(out));
        if (0U != n) {
            write_all(out, n);
            last_proto = PROTO_FEE;
            last_ms = s_now_ms;
        }
    }
}

/* ── main ──────────────────────────────────────────────────────────────── */

static void usage(void)
{
    fprintf(stderr,
            "usage: boot_node_sim (--fd N | --device PATH) [--dump-dir DIR]\n"
            "                     [--run-slot a|b] [--other-bootable]\n"
            "                     [--preload slot:ver] [--seconds N]\n");
}

int main(int argc, char **argv)
{
    const char *dump_dir = NULL;
    const char *device = NULL;
    const char *preload = NULL;
    uint8_t run_slot = SLOT_NONE;
    uint8_t other_bootable = 0U;
    uint32_t seconds = 60U;
    uint32_t started;
    upg_ctx_t ctx;
    int i;

    for (i = 1; i < argc; i++) {
        if (0 == strcmp(argv[i], "--fd") && i + 1 < argc) {
            s_fd = atoi(argv[++i]);
        } else if (0 == strcmp(argv[i], "--device") && i + 1 < argc) {
            device = argv[++i];
        } else if (0 == strcmp(argv[i], "--dump-dir") && i + 1 < argc) {
            dump_dir = argv[++i];
        } else if (0 == strcmp(argv[i], "--run-slot") && i + 1 < argc) {
            run_slot = ('b' == argv[++i][0]) ? SLOT_B : SLOT_A;
        } else if (0 == strcmp(argv[i], "--other-bootable")) {
            other_bootable = 1U;
        } else if (0 == strcmp(argv[i], "--preload") && i + 1 < argc) {
            preload = argv[++i];
        } else if (0 == strcmp(argv[i], "--verbose")) {
            s_verbose = 1;
        } else if (0 == strcmp(argv[i], "--seconds") && i + 1 < argc) {
            seconds = (uint32_t)atoi(argv[++i]);
        } else {
            usage();
            return 2;
        }
    }
    if (NULL == device && s_fd < 0) {
        usage();
        return 2;
    }
    if (NULL != device) {
        s_fd = open_device(device);
        if (s_fd < 0) {
            return 1;
        }
    }

    s_now_ms = mono_ms();
    boot_flash_test_reset();
    if (NULL != preload) {
        preload_slot(preload);
    }
    boot_regs_init();
    upg_set_hooks(&s_hooks);

    ctx.run_slot = run_slot;
    ctx.other_bootable = other_bootable;
    upg_set_context(&ctx);

    /* identity window: boot mode, the slot the bootloader would run */
    {
        boot_decide_in_t in;
        boot_decide_out_t out;
        memset(&in, 0, sizeof(in));
        in.boot_slot = SLOT_NONE;
        in.trial_slot = SLOT_NONE;
        in.valid[SLOT_A] = (uint8_t)((SLOT_NONE != run_slot && SLOT_A == run_slot)
                                     || 0U != other_bootable);
        in.valid[SLOT_B] = (uint8_t)((SLOT_NONE != run_slot && SLOT_B == run_slot)
                                     || 0U != other_bootable);
        boot_decide(&in, &out);
        boot_regs_set_decision(&in, &out);
        boot_regs_set_slot_info(run_slot, APP_VERSION, 0U, 0U);
    }

    dxl_slave_init(&s_dxl);
    fee_slave_init(&s_fee);
    printf("SIM: ready run_slot=%d other_bootable=%u\n", (int)run_slot,
           (unsigned)other_bootable);
    fflush(stdout);

    started = mono_ms();
    while ((mono_ms() - started) < seconds * 1000U) {
        fd_set rfds;
        struct timeval tv;
        int ready;
        uint8_t buf[256];
        ssize_t n;

        FD_ZERO(&rfds);
        FD_SET(s_fd, &rfds);
        tv.tv_sec = 0;
        tv.tv_usec = 20000;
        ready = select(s_fd + 1, &rfds, NULL, NULL, &tv);
        if (ready < 0) {
            if (errno == EINTR) {
                continue;
            }
            break;
        }
        if (0 == ready) {
            continue;
        }
        n = read(s_fd, buf, sizeof(buf));
        if (n <= 0) {
            if (0 == n) {
                break;          /* peer closed the pty: the session is over */
            }
            if (errno == EINTR || errno == EAGAIN) {
                continue;
            }
            break;
        }
        if (s_verbose) {
            fprintf(stderr, "sim: read %d bytes: %02X %02X %02X %02X\n", (int)n,
                    (unsigned)buf[0], (unsigned)((n > 1) ? buf[1] : 0),
                    (unsigned)((n > 2) ? buf[2] : 0),
                    (unsigned)((n > 3) ? buf[3] : 0));
        }
        for (i = 0; i < (int)n; i++) {
            s_now_ms = mono_ms();
            feed(buf[i]);
        }
    }

    printf("SIM: blocks=%d commits=%d commit_slot=%d status=0x%02X errcode=%u "
           "bytes=%u retransmits=%u\n",
           s_blocks, s_commits, (int)s_commit_slot, (unsigned)g_upg.status,
           (unsigned)g_upg.errcode, (unsigned)upg_bytes_written(),
           (unsigned)upg_retransmits());
    if (NULL != dump_dir) {
        dump_flash(dump_dir);
    }
    fflush(stdout);
    if (s_fd >= 0) {
        (void)close(s_fd);
    }
    return 0;
}
