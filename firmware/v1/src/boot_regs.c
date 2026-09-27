/*!
    \file    boot_regs.c
    \brief   see boot_regs.h
*/

#include "boot_regs.h"

#include <string.h>

#include "boot_upgrade.h"
#include "systick.h"

static uint8_t    s_dxl[REG_SPACE_SIZE];
static uint8_t    s_fee[REG_SPACE_SIZE];
static uint8_t    s_uid[UID_WIN_LEN];
static cfg_blob_t s_blob;
static uint8_t    s_loaded;
static uint32_t   s_reboot_at_ms;
/* Last protocol error, published in the vendor window's V_LAST_ERR.  The reply
   status byte stays 0 in boot mode too, for the same reason as the application:
   FeeTech tooling reads a nonzero ack status as a servo fault. */
static uint8_t    s_last_err;

/* ── little-endian helpers ──────────────────────────────────────────────── */

static void put16(uint8_t *img, uint16_t a, uint16_t v)
{
    img[a] = (uint8_t)(v & 0xFFU);
    img[a + 1U] = (uint8_t)(v >> 8);
}

static void put32(uint8_t *img, uint16_t a, uint32_t v)
{
    img[a] = (uint8_t)(v & 0xFFU);
    img[a + 1U] = (uint8_t)((v >> 8) & 0xFFU);
    img[a + 2U] = (uint8_t)((v >> 16) & 0xFFU);
    img[a + 3U] = (uint8_t)((v >> 24) & 0xFFU);
}

/* ── upgrade state machine callbacks ────────────────────────────────────── */

static void on_commit(uint8_t slot)
{
    s_blob.boot.trial_slot = slot;
    s_blob.boot.attempts = 0U;
    s_blob.boot.pingpong = (uint8_t)(s_blob.boot.pingpong + 1U);
    /* A committed image means the host is done: do not hold the node in boot
       mode any more, so a power cut right after the upgrade still boots the
       trial image (and rolls back on its own if it never confirms). */
    s_blob.boot.boot_stay = 0U;
    (void)boot_regs_save();
}

static void on_reboot(void)
{
    if (0U != s_blob.boot.boot_stay) {
        s_blob.boot.boot_stay = 0U;
        (void)boot_regs_save();
    }
}

static const upg_hooks_t s_hooks = {
    on_commit,
    on_reboot,
    NULL            /* on_block: no console mid-session */
};

/* ── identity window ────────────────────────────────────────────────────── */

void boot_regs_set_decision(const boot_decide_in_t *in, const boot_decide_out_t *out)
{
    (void)in;
    memset(s_uid, 0, sizeof(s_uid));
    put16(s_uid, UID_MAGIC_L, UID_MAGIC);
    s_uid[UID_MODE] = UID_MODE_BOOT;
    s_uid[UID_SLOT_RUN] = (uint8_t)((SLOT_NONE != out->slot) ? out->slot : SLOT_NONE);
}

/*! \brief publish the header of a slot in the identity window (called by
           boot_main once the headers are validated). */
void boot_regs_set_slot_info(uint8_t slot, uint16_t version, uint16_t flags,
                             uint32_t imgcrc)
{
    s_uid[UID_SLOT_RUN] = (uint8_t)((slot <= SLOT_B) ? slot : SLOT_NONE);
    s_uid[UID_FLAGS] = (uint8_t)(flags & 0xFFU);
    put16(s_uid, UID_VER_L, version);
    put32(s_uid, UID_IMGCRC_0, imgcrc);
}

/* ── images ─────────────────────────────────────────────────────────────── */

static void images_build(void)
{
    memset(s_dxl, 0, sizeof(s_dxl));
    memset(s_fee, 0, sizeof(s_fee));

    put16(s_dxl, 0U, BOOT_MODEL_NUMBER);
    s_dxl[6] = 0U;                          /* firmware version 0 = bootloader */
    s_dxl[7] = s_blob.cfg.dxl_id;
    s_dxl[8] = s_blob.cfg.dxl_baud_code;
    s_dxl[13] = 2U;                         /* protocol 2.0 */
    s_dxl[68] = 2U;                         /* status return level */

    put16(s_fee, 3U, BOOT_FEE_MODEL_NUMBER);
    s_fee[5] = s_blob.cfg.fee_id;
    s_fee[6] = s_blob.cfg.fee_baud_code;
    s_fee[8] = 1U;                          /* ACK everything */

    dev_refresh();
}

void boot_regs_init(void)
{
    memset(&s_blob, 0, sizeof(s_blob));
    s_loaded = (uint8_t)cfg_blob_load(&s_blob);
    if (0U == s_loaded) {
        /* no usable configuration page: defaults, exactly like the application */
        dev_cfg_set_defaults(&s_blob.cfg);
        cfg_boot_defaults(&s_blob.boot);
    }

    if (s_blob.boot.boot_slot > SLOT_B) {
        s_blob.boot.boot_slot = SLOT_NONE;
    }
    if (s_blob.boot.trial_slot > SLOT_B) {
        s_blob.boot.trial_slot = SLOT_NONE;
    }

    memset(s_uid, 0, sizeof(s_uid));
    put16(s_uid, UID_MAGIC_L, UID_MAGIC);
    s_uid[UID_MODE] = UID_MODE_BOOT;
    s_uid[UID_SLOT_RUN] = SLOT_NONE;

    upg_init();
    upg_set_hooks(&s_hooks);
    images_build();
}

const upg_hooks_t *boot_regs_hooks(void)
{
    return &s_hooks;
}

cfg_blob_t *boot_regs_blob(void)
{
    return &s_blob;
}

int boot_regs_save(void)
{
    return cfg_blob_save(&s_blob);
}

/* ── the API used by dxl2.c / fee.c / bus.c ─────────────────────────────── */

const dev_cfg_t *dev_cfg(void)
{
    return &s_blob.cfg;
}

const uint8_t *dev_image(uint8_t proto)
{
    return (PROTO_DXL == proto) ? s_dxl : s_fee;
}

void dev_refresh(void)
{
    /* The boot state lives in the configuration page and changes as the host
       upgrades things, so it is published on every read rather than once. */
    s_uid[UID_INFO_VERSION] = UID_INFO_VERSION_1;
    s_uid[UID_BOOT_SLOT] = s_blob.boot.boot_slot;
    s_uid[UID_TRIAL_SLOT] = s_blob.boot.trial_slot;
    s_uid[UID_ATTEMPTS] = s_blob.boot.attempts;
    s_uid[UID_BOOT_STAY] = s_blob.boot.boot_stay;
    /* In the bootloader "on trial" means "the image that is about to run still
       has to confirm itself", i.e. a trial slot exists at all: the decision on
       the way out of boot mode will pick it (RUN_TRIAL) while it has attempts
       left. */
    s_uid[UID_FLAGS] = (uint8_t)((s_uid[UID_FLAGS] & (uint8_t)~APP_HDR_FLAG_TRIAL)
                                 | ((s_blob.boot.trial_slot <= SLOT_B)
                                    ? APP_HDR_FLAG_TRIAL : 0U));

    upg_refresh();
    memcpy(&s_dxl[UID_WIN_ADDR], s_uid, UID_WIN_LEN);
    memcpy(&s_fee[UID_WIN_ADDR], s_uid, UID_WIN_LEN);
    memcpy(&s_dxl[UPGRADE_WIN_ADDR], g_upg.win, UPGRADE_WIN_LEN);
    memcpy(&s_fee[UPGRADE_WIN_ADDR], g_upg.win, UPGRADE_WIN_LEN);

    /* last protocol error, so a host can ask why an upgrade write was refused
       (the reply's status byte stays 0 in boot mode as well) */
    s_dxl[DXL_VENDOR_ADDR + V_LAST_ERR] = s_last_err;
    s_fee[FEE_VENDOR_ADDR + V_LAST_ERR] = s_last_err;
}

uint8_t dev_read(uint8_t proto, uint16_t addr, uint16_t len, uint8_t *out)
{
    const uint8_t *img = (PROTO_DXL == proto) ? s_dxl : s_fee;

    if (0U == len) {
        return DXL_ERR_LENGTH;
    }
    if (((uint32_t)addr + (uint32_t)len) > REG_SPACE_SIZE) {
        return DXL_ERR_RANGE;
    }
    dev_refresh();
    memcpy(out, &img[addr], len);
    return 0U;
}

uint8_t dev_write(uint8_t proto, uint16_t addr, uint16_t len, const uint8_t *in)
{
    (void)proto;
    if (0U == len) {
        return DXL_ERR_LENGTH;
    }
    if (((uint32_t)addr + (uint32_t)len) > REG_SPACE_SIZE) {
        return DXL_ERR_RANGE;
    }
    /* the upgrade session window */
    if (addr >= UPGRADE_WIN_ADDR) {
        if (len > UPGRADE_WIN_LEN) {
            return DXL_ERR_RANGE;
        }
        return upg_write((uint8_t)(addr - UPGRADE_WIN_ADDR), in, (uint8_t)len,
                         systick_get_ms());
    }
    /* the identity window is read-only; a write is accepted and ignored, the
       same convention the application uses for its vendor status byte */
    if (addr >= UID_WIN_ADDR) {
        return 0U;
    }
    /* a bootloader has no control table */
    return DXL_ERR_RANGE;
}

void dev_factory_reset(void)
{
    /* restore the device configuration but keep the boot state: losing "which
       slot was confirmed" would be worse than any benefit of a full reset */
    dev_cfg_set_defaults(&s_blob.cfg);
    (void)boot_regs_save();
    images_build();
}

void dev_note_error(uint8_t proto, uint8_t err)
{
    (void)proto;
    if (0U != err) {
        s_last_err = err;
    }
}

void dev_request_reboot(uint32_t now_ms)
{
    on_reboot();
    s_reboot_at_ms = now_ms + 50U;
}

void dev_clear_hw_error(void)
{
    /* no hardware error latches in boot mode */
}

int boot_regs_reboot_pending(uint32_t now_ms)
{
    if (0U == s_reboot_at_ms) {
        return 0;
    }
    return ((int32_t)(now_ms - s_reboot_at_ms) >= 0) ? 1 : 0;
}

/* ── baud rates ─────────────────────────────────────────────────────────── */

uint32_t dev_dxl_baud(void)
{
    static const uint32_t tbl[7] = { 9600U, 57600U, 115200U, 1000000U,
                                     2000000U, 3000000U, 4000000U };
    uint8_t c = s_blob.cfg.dxl_baud_code;
    return (c <= 6U) ? tbl[c] : BUS_BAUD_DEFAULT;
}

uint32_t dev_fee_baud(void)
{
    static const uint32_t tbl[12] = { 1000000U, 500000U, 250000U, 128000U,
                                      115200U, 76800U, 57600U, 38400U,
                                      19200U, 14400U, 9600U, 4800U };
    uint8_t c = s_blob.cfg.fee_baud_code;
    return (c <= 11U) ? tbl[c] : BUS_BAUD_DEFAULT;
}

uint32_t dev_mirror_baud(void)
{
    static const uint32_t tbl[12] = { 1000000U, 500000U, 250000U, 128000U,
                                      115200U, 76800U, 57600U, 38400U,
                                      19200U, 14400U, 9600U, 4800U };
    uint8_t c = s_blob.cfg.mirror_baud_code;
    return (c <= 11U) ? tbl[c] : BUS_MIRROR_BAUD_DEFAULT;
}

uint32_t dev_cfg_seq(void)
{
    return 0U;      /* the bootloader never changes the bus configuration */
}
