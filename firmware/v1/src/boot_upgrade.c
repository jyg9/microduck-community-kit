/*!
    \file    boot_upgrade.c
    \brief   see boot_upgrade.h
*/

#include "boot_upgrade.h"

#include <string.h>

#include "app_header.h"
#include "boot_crc32.h"
#include "boot_flash.h"
#include "crc16.h"

upg_t g_upg;

static const upg_hooks_t *s_hooks;

/* ── little-endian helpers ──────────────────────────────────────────────── */

static uint16_t get16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static uint32_t get32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8)
           | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void put16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFU);
    p[1] = (uint8_t)(v >> 8);
}

static void put32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFFU);
    p[1] = (uint8_t)((v >> 8) & 0xFFU);
    p[2] = (uint8_t)((v >> 16) & 0xFFU);
    p[3] = (uint8_t)((v >> 24) & 0xFFU);
}

/* ── window bookkeeping ─────────────────────────────────────────────────── */

/* Read-only bytes: the node owns them, a host write is ignored (the same
   convention the application's vendor window uses for its status byte). */
static int ro_off(uint8_t off)
{
    if (U_STATUS == off || U_RESERVED == off) {
        return 1;
    }
    if (off >= U_ACK_SEQ_L && off <= U_ERRCODE) {
        return 1;
    }
    if (off >= U_NODE_CRC_0 && off <= U_NODE_CRC_3) {
        return 1;
    }
    return 0;
}

void upg_refresh(void)
{
    put16(&g_upg.win[U_ACK_SEQ_L], g_upg.ack_seq);
    g_upg.win[U_STATUS] = g_upg.status;
    g_upg.win[U_ERRCODE] = g_upg.errcode;
    put32(&g_upg.win[U_NODE_CRC_0], g_upg.nodecrc);
}

uint8_t upg_read(uint8_t off)
{
    if (off >= UPGRADE_WIN_LEN) {
        return 0U;
    }
    upg_refresh();
    return g_upg.win[off];
}

void upg_init(void)
{
    memset(&g_upg, 0, sizeof(g_upg));
    g_upg.ack_seq = 0xFFFFU;
    g_upg.status = UPG_ST_IDLE;
    g_upg.target = SLOT_NONE;
    g_upg.ctx.run_slot = SLOT_NONE;
    upg_refresh();
}

void upg_set_context(const upg_ctx_t *ctx)
{
    if (NULL != ctx) {
        g_upg.ctx = *ctx;
    }
}

void upg_set_hooks(const upg_hooks_t *hooks)
{
    s_hooks = hooks;
}

uint32_t upg_bytes_written(void)
{
    return g_upg.bytes_written;
}

uint32_t upg_blocks(void)
{
    return g_upg.blocks;
}

uint32_t upg_retransmits(void)
{
    return g_upg.retransmits;
}

int upg_reboot_pending(uint32_t now_ms)
{
    if (0U == g_upg.reboot_pending) {
        return 0;
    }
    return ((int32_t)(now_ms - g_upg.reboot_at_ms) >= 0) ? 1 : 0;
}

/* ── failure bookkeeping ────────────────────────────────────────────────── */

static void fail(uint8_t errcode)
{
    g_upg.status = (uint8_t)(UPG_ST_ERROR | errcode);
    g_upg.errcode = errcode;
}

/* ── commands ───────────────────────────────────────────────────────────── */

static void cmd_begin(void)
{
    uint32_t imglen = get32(&g_upg.win[U_IMGLEN_0]);
    uint32_t imgcrc = get32(&g_upg.win[U_IMGCRC_0]);
    uint16_t ver = get16(&g_upg.win[U_VER_L]);
    uint8_t target = g_upg.win[U_TARGET];
    uint32_t pages;
    uint32_t i;

    g_upg.bytes_written = 0U;
    g_upg.blocks = 0U;
    g_upg.retransmits = 0U;
    g_upg.erase_pages = 0U;
    g_upg.nodecrc = 0U;

    if (0U == g_upg.armed) {
        fail(UPG_ERR_MAGIC);
        return;
    }
    if (target > SLOT_B) {
        fail(UPG_ERR_TARGET);
        return;
    }
    /* Never erase the only bootable image: if the slot the bootloader selected
       is the last one that works, the host has to upgrade the other one. */
    if (target == g_upg.ctx.run_slot && 0U == g_upg.ctx.other_bootable) {
        fail(UPG_ERR_TARGET);
        return;
    }
    if (imglen < UPG_MIN_IMGLEN || imglen > SLOT_SIZE || 0U != (imglen % 4U)) {
        fail(UPG_ERR_LENGTH);
        return;
    }
    pages = (imglen + FLASH_PAGE_SIZE - 1U) / FLASH_PAGE_SIZE;
    if (pages * FLASH_PAGE_SIZE > SLOT_SIZE) {
        fail(UPG_ERR_RANGE);
        return;
    }

    for (i = 0U; i < pages; i++) {
        if (0 != boot_flash_slot_erase(target, i * FLASH_PAGE_SIZE, FLASH_PAGE_SIZE)) {
            fail(UPG_ERR_FLASH);
            return;
        }
        g_upg.erase_pages++;
    }

    g_upg.active = 1U;
    g_upg.target = target;
    g_upg.imglen = imglen;
    g_upg.imgcrc = imgcrc;
    g_upg.ver = ver;
    g_upg.ack_seq = 0xFFFFU;
    g_upg.status = UPG_ST_ERASED;
    g_upg.errcode = UPG_ERR_NONE;
    put16(&g_upg.win[U_SEQ_L], 0U);
    put32(&g_upg.win[U_OFF_0], 0U);
    g_upg.win[U_BLKLEN] = 0U;
    put16(&g_upg.win[U_BLKCRC_L], 0U);
}

/*! \brief program one received block.
    \param[in] n block length in bytes; already checked to be 4/8/12/16 */
static void run_block(uint8_t n)
{
    uint16_t seq = get16(&g_upg.win[U_SEQ_L]);
    uint32_t off = get32(&g_upg.win[U_OFF_0]);
    uint16_t blkcrc = get16(&g_upg.win[U_BLKCRC_L]);
    uint16_t calc;
    uint8_t back[UPGRADE_DATA_LEN];
    int retransmit;

    if (0U == g_upg.active) {
        fail(UPG_ERR_STATE);
        return;
    }
    if (0U != g_upg.win[U_BLKLEN] && g_upg.win[U_BLKLEN] != n) {
        fail(UPG_ERR_LENGTH);
        return;
    }
    if (0U != (off % 4U) || (off + (uint32_t)n) > g_upg.imglen) {
        fail(UPG_ERR_RANGE);
        return;
    }
    calc = crc16_dxl(&g_upg.win[U_DATA_OFF], n);
    if (calc != blkcrc) {
        fail(UPG_ERR_CRC);
        return;
    }

    if (seq == (uint16_t)(g_upg.ack_seq + 1U)) {
        retransmit = 0;
    } else if (seq == g_upg.ack_seq) {
        retransmit = 1;
        g_upg.retransmits++;
    } else {
        /* out of order: refuse without touching flash, the host resynchronises
           from UPG_ACK_SEQ */
        fail(UPG_ERR_RANGE);
        return;
    }

    if (0 != boot_flash_slot_program(g_upg.target, off, &g_upg.win[U_DATA_OFF], n)) {
        fail(UPG_ERR_FLASH);
        return;
    }
    /* read back: a flash cell that refused to program is the one failure the
       FMC does not always report */
    if (0 != boot_flash_read(boot_flash_slot_base(g_upg.target) + off, back, n)
        || 0 != memcmp(back, &g_upg.win[U_DATA_OFF], n)) {
        fail(UPG_ERR_FLASH);
        return;
    }

    g_upg.blocks++;
    if (0 == retransmit) {
        g_upg.ack_seq = seq;
        g_upg.bytes_written += n;
    }
    g_upg.status = UPG_ST_BLOCK_OK;
    g_upg.errcode = UPG_ERR_NONE;
    if (NULL != s_hooks && NULL != s_hooks->on_block) {
        s_hooks->on_block(off, n, seq, retransmit);
    }
}

static void cmd_end(void)
{
    const uint8_t *img;
    app_header_t h;
    uint32_t crc;

    if (0U == g_upg.active) {
        fail(UPG_ERR_STATE);
        return;
    }
    img = boot_flash_slot_ptr(g_upg.target);
    if (NULL == img) {
        fail(UPG_ERR_TARGET);
        return;
    }

    crc = app_image_crc_combined(img, g_upg.imglen);
    g_upg.nodecrc = crc;
    if (crc != g_upg.imgcrc) {
        /* note: the image header is *not* rewritten by the node.  The host
           already sent a complete, valid header (the header is excluded from
           both CRCs), and flash can only clear bits - rewriting it here could
           not set a bit the host had left at 0.  The node verifies instead of
           writes, which is strictly safer. */
        fail(UPG_ERR_VERIFY);
        return;
    }

    if (APP_HDR_OK != app_header_check(img, SLOT_SIZE, &h)) {
        fail(UPG_ERR_VERIFY);
        return;
    }
    if (h.image_size != g_upg.imglen) {
        fail(UPG_ERR_VERIFY);
        return;
    }
    if (0U != g_upg.ver && h.image_version != g_upg.ver) {
        fail(UPG_ERR_VERIFY);
        return;
    }
    if (!app_header_entry_in_slot(&h, boot_flash_slot_base(g_upg.target), SLOT_SIZE)) {
        fail(UPG_ERR_VERIFY);
        return;
    }

    g_upg.status = UPG_ST_VERIFIED;
    g_upg.errcode = UPG_ERR_NONE;

    /* The image is on trial from here: the bootloader gives it BOOT_ATTEMPTS_MAX
       boots to confirm itself (the application writes the confirmation into the
       configuration page), then falls back to the other slot.  The trial state
       lives in the configuration page, not in the image header, so a package
       that was built as "confirmed" still gets the safety net. */
    if (NULL != s_hooks && NULL != s_hooks->on_commit) {
        s_hooks->on_commit(g_upg.target);
    }
    g_upg.status = UPG_ST_COMMITTED;
}

static void cmd_abort(void)
{
    g_upg.active = 0U;
    g_upg.target = SLOT_NONE;
    g_upg.ack_seq = 0U;
    g_upg.bytes_written = 0U;
    g_upg.blocks = 0U;
    g_upg.retransmits = 0U;
    g_upg.imglen = 0U;
    g_upg.imgcrc = 0U;
    g_upg.nodecrc = 0U;
    g_upg.ver = 0U;
    g_upg.status = UPG_ST_IDLE;
    g_upg.errcode = UPG_ERR_NONE;
    put16(&g_upg.win[U_SEQ_L], 0U);
    put32(&g_upg.win[U_OFF_0], 0U);
    g_upg.win[U_BLKLEN] = 0U;
    put16(&g_upg.win[U_BLKCRC_L], 0U);
    put32(&g_upg.win[U_IMGLEN_0], 0U);
    put32(&g_upg.win[U_IMGCRC_0], 0U);
}

static void cmd_readback(void)
{
    uint32_t off = get32(&g_upg.win[U_OFF_0]);
    uint8_t len = g_upg.win[U_BLKLEN];

    if (0U == g_upg.active && 0U == g_upg.armed) {
        fail(UPG_ERR_STATE);
        return;
    }
    if (g_upg.target > SLOT_B) {
        fail(UPG_ERR_TARGET);
        return;
    }
    if (len < 1U || len > UPGRADE_DATA_LEN) {
        fail(UPG_ERR_LENGTH);
        return;
    }
    if (0U != (off % 4U) || (off + len) > SLOT_SIZE) {
        fail(UPG_ERR_RANGE);
        return;
    }
    if (0 != boot_flash_read(boot_flash_slot_base(g_upg.target) + off,
                             &g_upg.win[U_DATA_OFF], len)) {
        fail(UPG_ERR_RANGE);
        return;
    }
    g_upg.errcode = UPG_ERR_NONE;
}

static void run_command(uint8_t cmd, uint32_t now_ms)
{
    switch (cmd) {
    case UPG_CMD_BEGIN:
        cmd_begin();
        break;
    case UPG_CMD_DATA:
        if (0U == g_upg.active) {
            fail(UPG_ERR_STATE);
        } else {
            g_upg.status = UPG_ST_RECEIVING;
            g_upg.errcode = UPG_ERR_NONE;
        }
        break;
    case UPG_CMD_END:
        cmd_end();
        break;
    case UPG_CMD_ABORT:
        cmd_abort();
        break;
    case UPG_CMD_CONFIRM:
        /* An application-side command: the bootloader has nothing to confirm. */
        fail(UPG_ERR_STATE);
        break;
    case UPG_CMD_REBOOT:
        if (NULL != s_hooks && NULL != s_hooks->on_reboot) {
            s_hooks->on_reboot();
        }
        g_upg.reboot_pending = 1U;
        g_upg.reboot_at_ms = now_ms + 50U;
        break;
    case UPG_CMD_READBACK:
        cmd_readback();
        break;
    default:
        fail(UPG_ERR_STATE);
        break;
    }
}

/* ── the write entry point ──────────────────────────────────────────────── */

uint8_t upg_write(uint8_t off, const uint8_t *p, uint8_t n, uint32_t now_ms)
{
    uint8_t i;
    uint8_t cmd = UPG_CMD_NONE;
    uint8_t cmd_written = 0U;
    uint8_t magic_written = 0U;
    uint8_t blk_len = 0U;

    if (NULL == p || 0U == n) {
        return DXL_ERR_LENGTH;
    }
    if (((uint16_t)off + (uint16_t)n) > UPGRADE_WIN_LEN) {
        return DXL_ERR_RANGE;
    }

    for (i = 0U; i < n; i++) {
        uint8_t o = (uint8_t)(off + i);
        if (ro_off(o)) {
            continue;
        }
        g_upg.win[o] = p[i];
        if (U_CMD == o) {
            cmd = p[i];
            cmd_written = 1U;
        }
        if (U_MAGIC_L == o || U_MAGIC_H == o) {
            magic_written = 1U;
        }
    }

    /* The magic word opens (or closes) the session - but only a write that
       actually covers those two bytes may do so.  A 16-byte data frame at
       offset 32 must not be able to drop the session it belongs to. */
    if (0U != magic_written) {
        if (get16(&g_upg.win[U_MAGIC_L]) == UPGRADE_MAGIC) {
            g_upg.armed = 1U;
        } else {
            g_upg.armed = 0U;
            g_upg.active = 0U;
        }
    }

    /* A write that reaches the data window carries a block.  The host sends the
       block as its own frame (address 240), but a write of the whole window in
       one frame works too. */
    if (off <= U_DATA_OFF && ((uint16_t)off + (uint16_t)n) >= (U_DATA_OFF + UPG_BLK_MIN)) {
        uint16_t avail = (uint16_t)((uint16_t)off + (uint16_t)n) - U_DATA_OFF;
        if (avail > UPGRADE_WIN_LEN - U_DATA_OFF) {
            avail = UPGRADE_WIN_LEN - U_DATA_OFF;
        }
        blk_len = (uint8_t)avail;
    }

    if (0U != cmd_written && UPG_CMD_NONE != cmd) {
        run_command(cmd, now_ms);
    }
    g_upg.win[U_CMD] = 0U;      /* the host always reads back 0 */

    if (0U != blk_len) {
        if (blk_len < UPG_BLK_MIN || blk_len > UPG_BLK_MAX
            || 0U != (blk_len % 4U)) {
            g_upg.win[U_BLKLEN] = 0U;
            return DXL_ERR_LENGTH;
        }
        run_block(blk_len);
    }
    return 0U;
}
