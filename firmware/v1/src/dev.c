/*!
    \file    dev.c
    \brief   register images, configuration, persistence, on-chip temperature
*/

#include "dev.h"

#include <string.h>

#include "app_header.h"
#include "boot_ram.h"
#include "critical.h"
#include "dbg.h"
#include "cfg_blob.h"
#include "imu.h"
#include "imu_sim.h"
#include "crc16.h"
#include "systick.h"
#include "telem_pack.h"

#if TEMP_SENSOR_ENABLE
#define VREF_MV             3300.0f
#define TEMP_V25_MV         1430.0f
#define TEMP_SLOPE_MV_C     4.3f
#endif

/* ── state ──────────────────────────────────────────────────────────────── */

static uint8_t   s_dxl[REG_SPACE_SIZE];
static uint8_t   s_fee[REG_SPACE_SIZE];
static dev_cfg_t s_cfg;
static cfg_boot_t s_boot;
static uint8_t   s_uid[UID_WIN_LEN];
static uint32_t  s_own_imgcrc;
static uint32_t  s_cfg_seq;
static uint8_t   s_cfg_dirty;
static uint32_t  s_cfg_change_ms;
static uint32_t  s_reboot_at_ms;
static uint8_t   s_flash_loaded;
static uint32_t  s_boot_started_ms;
/* set for the duration of one write while the FeeTech EEPROM lock is on: the
   value still lands in the image, it is just not marked for persistence */
static uint8_t   s_cfg_locked;
/* last protocol error this node refused with, published in V_LAST_ERR: the ack
   status byte itself stays 0 (see src/fee.c) */
static uint8_t   s_last_err;

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

static uint16_t get16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

/* ── configuration ──────────────────────────────────────────────────────── */

static uint8_t dl_baud_code_valid(uint8_t c)
{
    return (uint8_t)(c <= 6U);
}

static uint8_t fee_baud_code_valid(uint8_t c)
{
    return (uint8_t)(c <= 11U);
}

void dev_cfg_defaults(void)
{
    /* the defaults themselves live in src/dev_cfg.c so the bootloader can use
       them without linking this file */
    dev_cfg_set_defaults(&s_cfg);
    /* The boot state needs its defaults too.  Leaving it as .bss zeros reads as
       "slot A is confirmed and slot A is on trial", which is not a default but a
       lie - and it made a freshly flashed board answer the identity window with
       a phantom trial. */
    cfg_boot_defaults(&s_boot);
}

const dev_cfg_t *dev_cfg(void)
{
    return &s_cfg;
}

uint32_t dev_cfg_seq(void)
{
    return s_cfg_seq;
}

/* write the typed configuration into the vendor window of both images */
static void vendor_sync(void)
{
    uint8_t *imgs[2];
    uint16_t bases[2];
    uint8_t status;
    uint8_t k;

    imgs[0] = s_dxl;  bases[0] = DXL_VENDOR_ADDR;
    imgs[1] = s_fee;  bases[1] = FEE_VENDOR_ADDR;

    status = 0U;
    if (s_cfg_dirty) {
        status |= VSTAT_CFG_DIRTY;
    }
    if (s_flash_loaded) {
        status |= VSTAT_CFG_FROM_FLASH;
    }
    if (imu_is_simulated()) {
        status |= VSTAT_SIM_ACTIVE;
    }

    for (k = 0U; k < 2U; k++) {
        uint8_t *w = &imgs[k][bases[k]];
        put16(w, V_MAGIC_L, VENDOR_MAGIC);
        w[V_SIM_MODE] = s_cfg.sim_mode;
        put16(w, V_SIM_AMP_L, s_cfg.sim_amp);
        put16(w, V_SIM_FREQ_L, s_cfg.sim_freq);
        w[V_REPORT_FRAME] = s_cfg.report_frame;
        w[V_PROTO_LOCK] = s_cfg.proto_lock;
        put16(w, V_BIAS_X_L, (uint16_t)((int32_t)s_cfg.gyro_bias[0] & 0xFFFF));
        put16(w, V_BIAS_Y_L, (uint16_t)((int32_t)s_cfg.gyro_bias[1] & 0xFFFF));
        put16(w, V_BIAS_Z_L, (uint16_t)((int32_t)s_cfg.gyro_bias[2] & 0xFFFF));
        w[V_MIRROR_BAUD] = s_cfg.mirror_baud_code;
        w[V_DBG_LEVEL] = s_cfg.dbg_level;
        w[V_STATUS] = status;
        w[V_LAST_ERR] = s_last_err;
        w[V_CMD] = VCMD_NONE;
    }
}

/* rebuild both register images from scratch: identity, defaults, vendor window */
static void images_build(void)
{
    uint8_t i;

    memset(s_dxl, 0, sizeof(s_dxl));
    memset(s_fee, 0, sizeof(s_fee));

    /* ---- Dynamixel: XL330 control table layout ---------------------------- */
    put16(s_dxl, 0U, DXL_MODEL_NUMBER);
    s_dxl[6] = FW_VERSION_BYTE;
    s_dxl[7] = s_cfg.dxl_id;
    s_dxl[8] = s_cfg.dxl_baud_code;
    s_dxl[9] = 0U;              /* return delay time - microduck forces 0 */
    s_dxl[10] = 0U;             /* drive mode */
    s_dxl[11] = 3U;             /* operating mode: position control */
    s_dxl[12] = 255U;           /* secondary id: off */
    s_dxl[13] = 2U;             /* protocol type: 2.0 */
    put32(s_dxl, 20U, 0U);      /* homing offset */
    put32(s_dxl, 24U, 10U);     /* moving threshold */
    s_dxl[31] = 70U;            /* temperature limit */
    put16(s_dxl, 32U, 70U);     /* max voltage limit */
    put16(s_dxl, 34U, 35U);     /* min voltage limit */
    put16(s_dxl, 36U, 885U);    /* pwm limit */
    put16(s_dxl, 38U, 1750U);   /* current limit */
    put32(s_dxl, 44U, 445U);    /* velocity limit */
    put32(s_dxl, 48U, 4095U);   /* max position limit */
    put32(s_dxl, 52U, 0U);      /* min position limit */
    s_dxl[60] = 0U;             /* startup configuration */
    s_dxl[62] = 140U;           /* pwm slope */
    s_dxl[63] = 53U;            /* shutdown mask */
    s_dxl[64] = 0U;             /* torque enable */
    s_dxl[68] = 2U;             /* status return level: answer everything */
    put16(s_dxl, 76U, 1600U);   /* velocity I gain */
    put16(s_dxl, 78U, 180U);    /* velocity P gain */
    put16(s_dxl, 80U, 0U);      /* position D gain */
    put16(s_dxl, 82U, 0U);      /* position I gain */
    put16(s_dxl, 84U, 400U);    /* position P gain */
    for (i = 0U; i < 28U; i++) {
        put16(s_dxl, (uint16_t)(168U + 2U * i), (uint16_t)(224U + i));
    }

    /* ---- FeeTech: HLS memory table layout --------------------------------- */
    /* 0/1 = firmware version (major, minor).  A real HD-1910 reports 3.46 here;
       reporting 0.0 said nothing at all, and "don't pretend to be a servo" is
       why the model below is an unmistakable value. */
    s_fee[0] = FW_VERSION_MAJOR;
    s_fee[1] = FW_VERSION_MINOR;
    put16(s_fee, 3U, FEE_MODEL_NUMBER);
    s_fee[5] = s_cfg.fee_id;
    s_fee[6] = s_cfg.fee_baud_code;
    s_fee[7] = 254U;            /* second id: off */
    /* ACK status level (vendor table, initial value 1): 1 = answer every
       instruction, 0 = only READ/PING (and SYNC_READ) answer. */
    s_fee[8] = 1U;
    put16(s_fee, 9U, 0U);       /* min angle limit */
    put16(s_fee, 11U, 4095U);   /* max angle limit */
    put16(s_fee, 31U, 0U);      /* OFS: mid calibration */
    s_fee[33] = 0U;             /* mode: position servo */
    /* EEPROM lock (55).  The vendor initial value is 1 ("locked": EEPROM writes
       are accepted but not persisted).  This node defaults to 0 instead, so a
       host that sets the ID or baud rate gets it saved without extra steps -
       a documented deviation, see docs and 飞特通讯协议说明.md. */
    s_fee[55] = 0U;

    vendor_sync();
}

/* pull the typed configuration back out of an image */
static void vendor_apply(uint8_t proto)
{
    uint8_t *img = (proto == PROTO_DXL) ? s_dxl : s_fee;
    uint16_t base = (proto == PROTO_DXL) ? DXL_VENDOR_ADDR : FEE_VENDOR_ADDR;
    uint8_t *w = &img[base];
    uint16_t v;
    uint8_t cmd;

    if (get16(&w[V_MAGIC_L]) != VENDOR_MAGIC) {
        return;                 /* untouched or corrupted window: ignore */
    }

    if (w[V_SIM_MODE] <= SIM_MODE_MAX) {
        if (w[V_SIM_MODE] != s_cfg.sim_mode) {
            s_cfg.sim_mode = w[V_SIM_MODE];
            imu_restart();
            s_cfg_seq++;
        }
    }
    v = get16(&w[V_SIM_AMP_L]);
    if (v < 100U) { v = 100U; }
    if (v > 5000U) { v = 5000U; }
    if (v != s_cfg.sim_amp) { s_cfg.sim_amp = v; s_cfg_seq++; }

    v = get16(&w[V_SIM_FREQ_L]);
    if (v < 100U) { v = 100U; }
    if (v > 5000U) { v = 5000U; }
    if (v != s_cfg.sim_freq) { s_cfg.sim_freq = v; s_cfg_seq++; }

    if (w[V_REPORT_FRAME] <= 1U && w[V_REPORT_FRAME] != s_cfg.report_frame) {
        s_cfg.report_frame = w[V_REPORT_FRAME];
        imu_restart();
        s_cfg_seq++;
    }
    if (w[V_PROTO_LOCK] <= PROTO_FEE && w[V_PROTO_LOCK] != s_cfg.proto_lock) {
        s_cfg.proto_lock = w[V_PROTO_LOCK];
        s_cfg_seq++;
    }
    for (uint8_t i = 0U; i < 3U; i++) {
        int16_t b = (int16_t)get16(&w[V_BIAS_X_L + 2U * i]);
        if (b != s_cfg.gyro_bias[i]) {
            s_cfg.gyro_bias[i] = b;
            s_cfg_seq++;
        }
    }
    if (fee_baud_code_valid(w[V_MIRROR_BAUD])
        && w[V_MIRROR_BAUD] != s_cfg.mirror_baud_code) {
        s_cfg.mirror_baud_code = w[V_MIRROR_BAUD];
        s_cfg_seq++;
    }
    if (w[V_DBG_LEVEL] <= 3U && w[V_DBG_LEVEL] != s_cfg.dbg_level) {
        s_cfg.dbg_level = w[V_DBG_LEVEL];
        s_cfg_seq++;
    }

    cmd = w[V_CMD];
    if (VCMD_NONE != cmd) {
        w[V_CMD] = VCMD_NONE;
        switch (cmd) {
        case VCMD_SAVE:
            dev_cfg_save_now();
            s_cfg_seq++;
            break;
        case VCMD_DEFAULTS:
            dev_factory_reset();
            s_cfg_seq++;
            break;
        case VCMD_REBOOT:
            s_reboot_at_ms = systick_get_ms() + 50U;
            break;
        case VCMD_RESET_SIM:
            imu_restart();
            break;
        case VCMD_CONFIRM:
            dev_boot_confirm(systick_get_ms());
            break;
        case VCMD_BOOT:
            dev_boot_request_boot(systick_get_ms());
            break;
        case VCMD_SWITCH_SLOT:
            dev_boot_request_switch(systick_get_ms());
            break;
        default:
            break;
        }
    }
}

/* persist id / baud changes coming in through the plain registers */
static uint8_t hook_ids(uint8_t proto)
{
    uint32_t seq0 = s_cfg_seq;

    if (PROTO_DXL == proto) {
        uint8_t id = s_dxl[7];
        if (id < 1U || id > 252U) {
            id = (id == 0U) ? 1U : 252U;
            s_dxl[7] = id;
        }
        if (id != s_cfg.dxl_id) { s_cfg.dxl_id = id; s_cfg_seq++; }
        if (dl_baud_code_valid(s_dxl[8]) && s_dxl[8] != s_cfg.dxl_baud_code) {
            s_cfg.dxl_baud_code = s_dxl[8];
            s_cfg_seq++;
        } else if (!dl_baud_code_valid(s_dxl[8])) {
            s_dxl[8] = s_cfg.dxl_baud_code;
        }
    } else {
        uint8_t id = s_fee[5];
        if (id > FEE_ID_MAX) {
            id = FEE_ID_MAX;
            s_fee[5] = id;
        }
        if (id != s_cfg.fee_id) { s_cfg.fee_id = id; s_cfg_seq++; }
        if (fee_baud_code_valid(s_fee[6]) && s_fee[6] != s_cfg.fee_baud_code) {
            s_cfg.fee_baud_code = s_fee[6];
            s_cfg_seq++;
        } else if (!fee_baud_code_valid(s_fee[6])) {
            s_fee[6] = s_cfg.fee_baud_code;
        }
    }

    if (s_cfg_seq != seq0) {
        if (0U == s_cfg_locked) {
            s_cfg_dirty = 1U;
            s_cfg_change_ms = systick_get_ms();
        }
        return 1U;
    }
    return 0U;
}

/* ── flash persistence: one blob holds the device config *and* the boot state */

int dev_cfg_from_flash(void)
{
#if CFG_FLASH_ENABLE
    cfg_blob_t blob;

    if (!cfg_blob_load(&blob)) {
        return 0;
    }
    s_cfg = blob.cfg;
    s_boot = blob.boot;
    s_flash_loaded = 1U;
    return 1;
#else
    return 0;
#endif
}

void dev_cfg_save_now(void)
{
    s_cfg_dirty = 0U;
#if CFG_FLASH_ENABLE
    {
        cfg_blob_t blob;

        memset(&blob, 0, sizeof(blob));
        blob.cfg = s_cfg;
        blob.boot = s_boot;
        (void)cfg_blob_save(&blob);
        s_flash_loaded = 1U;
    }
#endif
}

/* ── boot state, identity window and the upgrade handshake ──────────────── */

static uint8_t s_own_hdr_ok;

static void uid_publish(void)
{
    const app_header_t *h = app_own_header();

    memset(s_uid, 0, sizeof(s_uid));
    put16(s_uid, UID_MAGIC_L, UID_MAGIC);
    s_uid[UID_MODE] = UID_MODE_APP;
    if (NULL != h) {
        s_uid[UID_SLOT_RUN] = app_own_slot();
        s_uid[UID_FLAGS] = (uint8_t)(h->flags & 0xFFU);
        put16(s_uid, UID_VER_L, h->image_version);
        put32(s_uid, UID_IMGCRC_0, s_own_imgcrc);
    } else {
        s_uid[UID_SLOT_RUN] = SLOT_NONE;
        put16(s_uid, UID_VER_L, APP_VERSION);
    }
    if (0U == s_own_hdr_ok) {
        s_uid[UID_FLAGS] |= UID_FLAG_NO_HEADER;
    }

    /* The header flag is frozen at packaging time; what matters to the host is
       whether this image still has to confirm itself, which lives in the
       configuration page. */
    s_uid[UID_FLAGS] = (uint8_t)((s_uid[UID_FLAGS] & (uint8_t)~APP_HDR_FLAG_TRIAL)
                                 | ((s_boot.trial_slot == app_own_slot()
                                     && SLOT_NONE != app_own_slot())
                                    ? APP_HDR_FLAG_TRIAL : 0U));
    s_uid[UID_INFO_VERSION] = UID_INFO_VERSION_1;
    s_uid[UID_BOOT_SLOT] = s_boot.boot_slot;
    s_uid[UID_TRIAL_SLOT] = s_boot.trial_slot;
    s_uid[UID_ATTEMPTS] = s_boot.attempts;
    s_uid[UID_BOOT_STAY] = s_boot.boot_stay;
}

void dev_boot_init(void)
{
    const app_header_t *h = app_own_header();

    /* defence in depth: a configuration page that loaded but carries
       out-of-range slot numbers must not be believed either */
    if (s_boot.boot_slot > SLOT_B) {
        s_boot.boot_slot = SLOT_NONE;
    }
    if (s_boot.trial_slot > SLOT_B) {
        s_boot.trial_slot = SLOT_NONE;
    }

    s_boot_started_ms = systick_get_ms();
    s_own_hdr_ok = (uint8_t)(APP_HDR_OK == app_own_header_check());
    s_own_imgcrc = 0U;
    if (NULL != h && 0U != s_own_hdr_ok) {
        s_own_imgcrc = app_image_crc_combined(
            (const uint8_t *)(const void *)h - APP_HDR_OFF, h->image_size);
    }
    uid_publish();
}

uint8_t dev_boot_trial_slot(void)
{
    return s_boot.trial_slot;
}

uint8_t dev_boot_boot_slot(void)
{
    return s_boot.boot_slot;
}

uint8_t dev_boot_stay(void)
{
    return s_boot.boot_stay;
}

uint8_t dev_boot_pingpong(void)
{
    return s_boot.pingpong;
}

void dev_boot_confirm(uint32_t now_ms)
{
    uint8_t own = app_own_slot();

    (void)now_ms;
    if (SLOT_NONE == own) {
        return;                 /* development image: nothing to confirm */
    }
    s_boot.trial_slot = SLOT_NONE;
    s_boot.attempts = 0U;
    s_boot.boot_slot = own;
    s_boot.boot_stay = 0U;
    s_boot.pingpong = (uint8_t)(s_boot.pingpong + 1U);
    dev_cfg_save_now();
    uid_publish();
    DBG_INFO("trial confirmed: slot %c is now the boot slot (pingpong %u)",
             (SLOT_A == own) ? 'A' : 'B', (unsigned)s_boot.pingpong);
}

int dev_boot_tick(uint32_t now_ms, uint32_t frames_ok)
{
    uint32_t up;
    uint8_t own = app_own_slot();

    if (SLOT_NONE == own || s_boot.trial_slot != own) {
        return 0;
    }
    up = (uint32_t)(now_ms - s_boot_started_ms);
    if (up < BOOT_CONFIRM_MS) {
        return 0;
    }
    /* A host that never talks to us must not keep the image on trial forever -
       but give it twice as long, so a robot that is simply idle still confirms
       instead of rolling back. */
    if (0U == frames_ok && up < BOOT_CONFIRM_LONE_MS) {
        return 0;
    }
    dev_boot_confirm(now_ms);
    return 1;
}

void dev_boot_request_boot(uint32_t now_ms)
{
    /* Persisted flag first: it is what survives a power cut.  The RAM magic only
       has to survive the reset we are about to trigger. */
    s_boot.boot_stay = 1U;
    dev_cfg_save_now();
    boot_ram_request(0U);
    DBG_INFO("bootloader requested: staying in boot mode after the reset");
    dev_request_reboot(now_ms);
}

void dev_boot_request_switch(uint32_t now_ms)
{
    uint8_t own = app_own_slot();

    if (SLOT_NONE == own) {
        return;
    }
    s_boot.boot_slot = (uint8_t)((SLOT_A == own) ? SLOT_B : SLOT_A);
    s_boot.boot_stay = 0U;
    dev_cfg_save_now();
    boot_ram_request(1U);
    DBG_INFO("switching boot slot to %c", (SLOT_A == s_boot.boot_slot) ? 'A' : 'B');
    dev_request_reboot(now_ms);
}

/* ── on-chip temperature ────────────────────────────────────────────────── */

#if TEMP_SENSOR_ENABLE
static uint16_t s_temp_c = 30U;
static uint32_t s_temp_next_ms;

static void temp_hw_init(void)
{
    rcu_periph_clock_enable(RCU_ADC0);
    /* AHB/20 = 6 MHz keeps the ADC inside its 14 MHz limit at 120 MHz core */
    rcu_adc_clock_config(RCU_CKADC_CKAHB_DIV20);

    adc_deinit(ADC0);
    adc_mode_config(ADC_MODE_FREE);
    adc_special_function_config(ADC0, ADC_SCAN_MODE, DISABLE);
    adc_special_function_config(ADC0, ADC_CONTINUOUS_MODE, DISABLE);
    adc_data_alignment_config(ADC0, ADC_DATAALIGN_RIGHT);
    adc_channel_length_config(ADC0, ADC_ROUTINE_CHANNEL, 1U);
    adc_routine_channel_config(ADC0, 0U, ADC_CHANNEL_16, ADC_SAMPLETIME_239POINT5);
    adc_external_trigger_config(ADC0, ADC_ROUTINE_CHANNEL, ENABLE);
    adc_external_trigger_source_config(ADC0, ADC_ROUTINE_CHANNEL,
                                       ADC0_1_2_EXTTRIG_ROUTINE_NONE);
    adc_enable(ADC0);
    delay_1ms(1U);
    adc_calibration_enable(ADC0);
    adc_tempsensor_vrefint_enable();
}

static uint16_t temp_read(void)
{
    uint32_t guard = 0U;
    uint16_t raw;
    float mv, t;

    adc_software_trigger_enable(ADC0, ADC_ROUTINE_CHANNEL);
    while ((RESET == adc_flag_get(ADC0, ADC_FLAG_EOC)) && (guard++ < 200000U)) {
    }
    raw = adc_routine_data_read(ADC0);
    mv = ((float)raw * VREF_MV) / 4096.0f;
    /* GD32F30x datasheet: V25 = 1.43 V, 4.3 mV/°C */
    t = (TEMP_V25_MV - mv) / TEMP_SLOPE_MV_C + 25.0f;
    if (t < 0.0f) { t = 0.0f; }
    if (t > 125.0f) { t = 125.0f; }
    return (uint16_t)t;
}
#endif

uint16_t dev_temp_celsius(void)
{
#if TEMP_SENSOR_ENABLE
    uint32_t now = systick_get_ms();
    if ((int32_t)(now - s_temp_next_ms) >= 0) {
        s_temp_next_ms = now + 1000U;
        s_temp_c = temp_read();
    }
    return s_temp_c;
#else
    return 30U;
#endif
}

/* ── baud ───────────────────────────────────────────────────────────────── */

uint32_t dev_dxl_baud(void)
{
    static const uint32_t tbl[7] = { 9600U, 57600U, 115200U, 1000000U,
                                     2000000U, 3000000U, 4000000U };
    uint8_t c = s_cfg.dxl_baud_code;
    return (c <= 6U) ? tbl[c] : BUS_BAUD_DEFAULT;
}

uint32_t dev_fee_baud(void)
{
    static const uint32_t tbl[12] = { 1000000U, 500000U, 250000U, 128000U,
                                      115200U, 76800U, 57600U, 38400U,
                                      19200U, 14400U, 9600U, 4800U };
    uint8_t c = s_cfg.fee_baud_code;
    return (c <= 11U) ? tbl[c] : BUS_BAUD_DEFAULT;
}

uint32_t dev_mirror_baud(void)
{
    static const uint32_t tbl[12] = { 1000000U, 500000U, 250000U, 128000U,
                                      115200U, 76800U, 57600U, 38400U,
                                      19200U, 14400U, 9600U, 4800U };
    uint8_t c = s_cfg.mirror_baud_code;
    return (c <= 11U) ? tbl[c] : BUS_MIRROR_BAUD_DEFAULT;
}

/* ── register access ────────────────────────────────────────────────────── */

/* Registers 180..255 belong to the bootloader: the identity window is
   read-only everywhere, and the upgrade session window is only writable while
   the bootloader is in charge.  Refusing here is what tells a host "enter boot
   mode first" instead of silently accepting upgrade writes nothing will act on. */
static int ro_high(uint16_t a)
{
    return (a >= UID_WIN_ADDR) ? 1 : 0;
}

static int ro_dxl(uint16_t a)
{
    if (ro_high(a)) { return 1; }
    if (a <= 6U) { return 1; }                    /* model, info, firmware */
    if (a == 69U) { return 1; }                   /* registered instruction */
    if (a >= 70U && a <= 71U) { return 1; }       /* hardware error status */
    if (a >= 120U && a <= 123U) { return 1; }     /* tick, moving, moving status */
    if (a >= 124U && a <= 147U) { return 1; }     /* present block, volt, temp */
    /* The vendor window's status/reserved bytes (166/167) are deliberately NOT
       read-only: hosts write the whole 20-byte window in one go, and refusing
       that would make the usable fields unwritable.  Writes there are ignored -
       vendor_sync() rewrites STATUS from the real state on every read. */
    return 0;
}

static int ro_fee(uint16_t a)
{
    if (ro_high(a)) { return 1; }
    if (a <= 1U) { return 1; }                    /* firmware version */
    if (a >= 3U && a <= 4U) { return 1; }         /* model */
    if (a >= 56U && a <= 70U) { return 1; }       /* the 15-byte IMU block */
    if (a >= 128U && a <= 147U) { return 1; }     /* telemetry alias */
    /* vendor window 160..179: all writable, status/last-error ignored (see
       ro_dxl; vendor_sync() rewrites them from the real state) */
    return 0;
}

const uint8_t *dev_image(uint8_t proto)
{
    return (PROTO_DXL == proto) ? s_dxl : s_fee;
}

void dev_refresh(void)
{
    const uint8_t *blk = imu_block_bytes();
    uint8_t temp;
    uint32_t primask;

    /* The on-chip temperature is an ADC conversion plus a float divide (rate
       limited to 1 Hz inside dev_temp_celsius), so keep it out of the critical
       section below. */
    temp = (uint8_t)dev_temp_celsius();

    /* Publish under a short critical section.  dev_read() runs in the USART
       interrupt now - a frame's last byte has to become a reply within one
       reply slot - so it can otherwise preempt this half way through and hand a
       host a block with new gyro counts and an old counter, or a torn
       quaternion.  Everything below is a few dozen bytes of copying: ~1-2 us at
       120 MHz, well inside one byte time at 1 Mbps. */
    primask = irq_save();

    /* static identity, re-published rather than set once: the version registers
       are writable on a real servo too (writes are ignored through ro_*) */
    put16(s_dxl, 0U, DXL_MODEL_NUMBER);
    s_dxl[6] = FW_VERSION_BYTE;
    s_fee[0] = FW_VERSION_MAJOR;
    s_fee[1] = FW_VERSION_MINOR;
    put16(s_fee, 3U, FEE_MODEL_NUMBER);

    /* realtime tick: free-running ms, what a servo reports here */
    put16(s_dxl, 120U, (uint16_t)systick_get_ms());

    /* Telemetry.  FeeTech 56..70 keeps the 15-byte servo shape with the counter
       and status inside it, which is what the shared `sync_read` reads (15
       bytes from the IMU node and from every servo); the raw accelerometer does
       not fit there and lives in the 20-byte diagnostic block at 128.  The
       Dynamixel map keeps the whole 20-byte block at 124, where the XL330 table
       has room up to 143. */
    telem_pack_fee15(blk, &s_fee[FEE_TELEM_ADDR]);
    memcpy(&s_dxl[DXL_TELEM_ADDR], blk, TELEM_LEN);
    memcpy(&s_fee[FEE_TELEM_ALT_ADDR], blk, TELEM_LEN);

    /* This node has no pack divider, so it reports 0 V: microduck filters
       zeros out of the bus voltage average rather than believing a fake
       reading.  Temperature comes from the MCU's own sensor. */
    put16(s_dxl, 144U, 0U);
    s_dxl[146] = temp;

    /* identity window: mode / slot / version / image CRC, for the upgrade tool */
    memcpy(&s_dxl[UID_WIN_ADDR], s_uid, UID_WIN_LEN);
    memcpy(&s_fee[UID_WIN_ADDR], s_uid, UID_WIN_LEN);

    vendor_sync();

    irq_restore(primask);
}

/*! \brief remember the last protocol error for V_LAST_ERR.
    Called from the protocol handlers, which now run in interrupt context, so
    this is a single byte store and nothing more. */
void dev_note_error(uint8_t proto, uint8_t err)
{
    (void)proto;
    if (0U != err) {
        s_last_err = err;
    }
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
    /* No dev_refresh() here: this runs in the USART interrupt, where an ADC
       conversion and a vendor-window rewrite have no business.  The main loop
       publishes the image (dev_refresh) whenever a new sample exists, and
       writes land in the image directly, so a read never needs to wait. */
    memcpy(out, &img[addr], len);
    return 0U;
}

uint8_t dev_write(uint8_t proto, uint16_t addr, uint16_t len, const uint8_t *in)
{
    uint8_t *img = (PROTO_DXL == proto) ? s_dxl : s_fee;
    uint16_t i;

    if (0U == len) {
        return DXL_ERR_LENGTH;
    }
    if (((uint32_t)addr + (uint32_t)len) > REG_SPACE_SIZE) {
        return DXL_ERR_RANGE;
    }
    /* FeeTech's lock register (55) is about persistence, not permission: with
       the lock on, a write to the EEPROM half of the table takes effect in RAM
       but is not saved, so it reverts at power-on.  Mirror exactly that. */
    {
        uint8_t locked = (uint8_t)((PROTO_FEE == proto) && (0U != s_fee[55]) && (addr < 40U));
        s_cfg_locked = locked;
    }

    /* refuse the whole write if any byte is read-only - the same all-or-nothing
       behaviour a real servo shows, and much easier to reason about */
    for (i = 0U; i < len; i++) {
        uint16_t a = (uint16_t)(addr + i);
        int ro = (PROTO_DXL == proto) ? ro_dxl(a) : ro_fee(a);
        if (ro) {
            if (PROTO_DXL == proto) {
                s_cfg_locked = 0U;
                return DXL_ERR_ACCESS;
            }
            continue;           /* FeeTech: accepted, ignored */
        }
    }

    for (i = 0U; i < len; i++) {
        img[addr + i] = in[i];
    }

    (void)hook_ids(proto);
    s_cfg_locked = 0U;
    if (PROTO_DXL == proto) {
        if (!(addr + len <= DXL_VENDOR_ADDR || addr > (DXL_VENDOR_ADDR + 19U))) {
            vendor_apply(proto);
        }
    } else {
        if (!(addr + len <= FEE_VENDOR_ADDR || addr > (FEE_VENDOR_ADDR + 19U))) {
            vendor_apply(proto);
        }
    }
    return 0U;
}

/* ── lifecycle ──────────────────────────────────────────────────────────── */

void dev_init(void)
{
    s_cfg_seq = 0U;
    s_cfg_dirty = 0U;
    s_cfg_change_ms = 0U;
    s_reboot_at_ms = 0U;
    s_flash_loaded = 0U;

    dev_cfg_defaults();
    (void)dev_cfg_from_flash();
    images_build();

#if TEMP_SENSOR_ENABLE
    temp_hw_init();
#endif

    dev_boot_init();
}

#define CFG_SAVE_QUIET_MS   500U     /* let a burst of register writes settle */
#define CFG_SAVE_IDLE_MS    1500U    /* ... and wait for the bus to go quiet */

void dev_tick(uint32_t now_ms, uint32_t bus_idle_ms)
{
    if (s_cfg_dirty && CFG_FLASH_ENABLE) {
        if ((int32_t)(now_ms - s_cfg_change_ms) >= (int32_t)CFG_SAVE_QUIET_MS
            && bus_idle_ms >= CFG_SAVE_IDLE_MS) {
            dev_cfg_save_now();
        }
    }
}

int dev_reboot_pending(uint32_t now_ms)
{
    if (0U == s_reboot_at_ms) {
        return 0;
    }
    return ((int32_t)(now_ms - s_reboot_at_ms) >= 0) ? 1 : 0;
}

void dev_request_reboot(uint32_t now_ms)
{
    /* give the status packet time to leave before the core resets */
    s_reboot_at_ms = now_ms + 50U;
}

void dev_factory_reset(void)
{
    dev_cfg_defaults();
    images_build();
    imu_restart();
    s_cfg_seq++;
    dev_cfg_save_now();
}

void dev_clear_hw_error(void)
{
    s_dxl[70] = 0U;
    s_dxl[71] = 0U;
}
