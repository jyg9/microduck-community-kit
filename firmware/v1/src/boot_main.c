/*!
    \file    boot_main.c
    \brief   the bootloader: validate the slots, decide, jump - or stay and take
             an upgrade over the motor bus

    Flash map (board.h, docs/flash_layout.md):

      0x08000000  32 KB   this bootloader (only updatable through SWD)
      0x08008000  96 KB   slot A
      0x08020000  96 KB   slot B
      0x08038000  30 KB   unassigned
      0x0803F800   2 KB   configuration page (device config + boot state)

    Runs in one of two modes:

      * **boot decision** (the normal path): both slot headers are validated,
        `boot_decide()` picks one, a trial boot counts its attempt in the
        configuration page and then jumps to the application's reset vector.
      * **boot mode**: nothing bootable, or a host asked for it (persisted
        `boot_stay`, or the SRAM handshake word after `VCMD_BOOT`).  The bus is
        served with the *same* `dxl2.c` / `fee.c` / `bus.c` as the application,
        so an upgrade is driven with ordinary register writes.  With a bootable
        slot present the mode times out after BOOT_IDLE_EXIT_MS of silence, so a
        host that disappears cannot leave the robot stranded.

    Why the bootloader never updates itself: a self-updating bootloader needs a
    transport that does not depend on the application, and a mistake there is
    unrecoverable without a debugger.  It fits in 32 KB and is flashed once.
*/

#include "board.h"

#include <string.h>

#include "app_header.h"
#include "boot_decide.h"
#include "boot_flash.h"
#include "boot_ram.h"
#include "boot_regs.h"
#include "boot_upgrade.h"
#include "bus.h"
#include "dbg.h"
#include "systick.h"
#include "us_time.h"

#define LED_PERIOD_MS       250U

static boot_decide_in_t  s_in;
static boot_decide_out_t s_out;
static uint16_t          s_slot_version[2];
static uint16_t          s_slot_flags[2];
static uint32_t          s_slot_crc[2];
static uint32_t          s_last_frame_ms;

/* ── board bring-up ─────────────────────────────────────────────────────── */

static void board_gpio_init(void)
{
#if STATUS_LED_ENABLE
    rcu_periph_clock_enable(RCU_GPIOB);
    gpio_init(LED_GPIO, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, LED_PIN);
    gpio_bit_reset(LED_GPIO, LED_PIN);
#endif

    /* IMU chip select high: a fitted sensor must stay deselected while the
       application is not running */
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
    gpio_init(IMU_SPI_GPIO, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, IMU_CS_PIN);
    gpio_bit_set(IMU_SPI_GPIO, IMU_CS_PIN);
    gpio_init(IMU_SPI_GPIO, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ,
              IMU_SCK_PIN | IMU_MISO_PIN | IMU_MOSI_PIN);
    gpio_init(IMU_INT1_GPIO, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ,
              IMU_INT1_PIN | IMU_INT2_PIN);
}

static void led_tick(uint32_t now_ms)
{
#if STATUS_LED_ENABLE
    static uint32_t next_ms;
    static uint8_t on;

    if (0U == next_ms) {
        next_ms = now_ms + LED_PERIOD_MS;
    }
    if ((int32_t)(now_ms - next_ms) >= 0) {
        next_ms = now_ms + LED_PERIOD_MS;
        on = (uint8_t)(0U == on);
        if (on) {
            gpio_bit_set(LED_GPIO, LED_PIN);
        } else {
            gpio_bit_reset(LED_GPIO, LED_PIN);
        }
    }
#else
    (void)now_ms;
#endif
}

/* ── slot validation ────────────────────────────────────────────────────── */

static void normalize_boot_state(void)
{
    cfg_blob_t *blob = boot_regs_blob();

    if (blob->boot.boot_slot > SLOT_B) {
        blob->boot.boot_slot = SLOT_NONE;
    }
    if (blob->boot.trial_slot > SLOT_B) {
        blob->boot.trial_slot = SLOT_NONE;
    }
    if (blob->boot.attempts > BOOT_ATTEMPTS_MAX) {
        blob->boot.attempts = BOOT_ATTEMPTS_MAX;
    }
}

static void validate_slots(void)
{
    uint8_t slot;

    for (slot = SLOT_A; slot <= SLOT_B; slot++) {
        const uint8_t *img = boot_flash_slot_ptr(slot);
        app_header_t h;
        int rc;

        memset(&h, 0, sizeof(h));
        s_in.valid[slot] = 0U;
        s_in.version[slot] = 0U;
        s_slot_version[slot] = 0U;
        s_slot_flags[slot] = 0U;
        s_slot_crc[slot] = 0U;

        rc = app_header_check(img, SLOT_SIZE, &h);
        if (APP_HDR_OK == rc) {
            s_in.valid[slot] = 1U;
            s_in.version[slot] = h.image_version;
            s_slot_version[slot] = h.image_version;
            s_slot_flags[slot] = h.flags;
            s_slot_crc[slot] = app_image_crc_combined(img, h.image_size);
        }
        DBG_INFO("slot %c: %s%s ver=%u.%u.%u size=%u",
                 (SLOT_A == slot) ? 'A' : 'B',
                 (0U != s_in.valid[slot]) ? "valid" : "invalid",
                 ((0U != s_in.valid[slot]) && (0U != (h.flags & APP_HDR_FLAG_TRIAL)))
                     ? " trial" : "",
                 (unsigned)APP_VERSION_MAJOR(h.image_version),
                 (unsigned)APP_VERSION_MINOR(h.image_version),
                 (unsigned)APP_VERSION_PATCH(h.image_version),
                 (unsigned)h.image_size);
        if (0U == s_in.valid[slot]) {
            DBG_INFO("       reason: %s", app_hdr_strerror(rc));
        }
    }
}

static void load_state(void)
{
    cfg_blob_t *blob = boot_regs_blob();

    normalize_boot_state();
    s_in.boot_slot = blob->boot.boot_slot;
    s_in.trial_slot = blob->boot.trial_slot;
    s_in.attempts = blob->boot.attempts;
    s_in.pingpong = blob->boot.pingpong;
}

static void publish_identity(void)
{
    upg_ctx_t ctx;

    boot_regs_set_decision(&s_in, &s_out);
    if (SLOT_NONE != s_out.slot) {
        boot_regs_set_slot_info(s_out.slot, s_slot_version[s_out.slot],
                                s_slot_flags[s_out.slot], s_slot_crc[s_out.slot]);
    }
    ctx.run_slot = s_out.slot;
    ctx.other_bootable = 0U;
    if (SLOT_NONE != s_out.slot) {
        ctx.other_bootable = s_in.valid[(SLOT_A == s_out.slot) ? SLOT_B : SLOT_A];
    }
    upg_set_context(&ctx);
}

/* ── the jump ───────────────────────────────────────────────────────────── */

static void boot_shutdown_peripherals(void)
{
    uint8_t i;

    usart_interrupt_disable(BUS_USART, USART_INT_RBNE);
    usart_disable(BUS_USART);
#if BUS_MIRROR_ENABLE
    usart_interrupt_disable(DBG_USART, USART_INT_RBNE);
    usart_disable(DBG_USART);
#endif

    SysTick->CTRL = 0U;
    SysTick->LOAD = 0U;
    SysTick->VAL = 0U;

    for (i = 0U; i < 8U; i++) {
        NVIC->ICER[i] = 0xFFFFFFFFU;
        NVIC->ICPR[i] = 0xFFFFFFFFU;
    }
}

__attribute__((noreturn))
static void boot_jump(uint8_t slot)
{
    uint32_t base = boot_flash_slot_base(slot);
    app_header_t h;
    uint32_t sp;
    uint32_t entry;

    if (0U == base || APP_HDR_OK != app_header_check((const uint8_t *)(uintptr_t)base,
                                                    SLOT_SIZE, &h)) {
        DBG_ERROR("slot %c stopped being valid, staying in the bootloader",
                  (SLOT_A == slot) ? 'A' : 'B');
        for (;;) {
        }
    }
    sp = *(const volatile uint32_t *)(uintptr_t)base;
    entry = *(const volatile uint32_t *)(uintptr_t)(base + 4U);
    if (entry != h.entry || !app_header_entry_in_slot(&h, base, SLOT_SIZE)) {
        DBG_ERROR("slot %c entry %08X does not match the header, refusing",
                  (SLOT_A == slot) ? 'A' : 'B', (unsigned)entry);
        for (;;) {
        }
    }
    if (0U == sp || 0U != (sp & 3U)) {
        DBG_ERROR("slot %c has a bad stack pointer %08X", (SLOT_A == slot) ? 'A' : 'B',
                  (unsigned)sp);
        for (;;) {
        }
    }

    DBG_INFO("jumping to slot %c: sp=%08X entry=%08X v%u.%u.%u",
             (SLOT_A == slot) ? 'A' : 'B', (unsigned)sp, (unsigned)entry,
             (unsigned)APP_VERSION_MAJOR(h.image_version),
             (unsigned)APP_VERSION_MINOR(h.image_version),
             (unsigned)APP_VERSION_PATCH(h.image_version));

    boot_shutdown_peripherals();

    /* The application's Reset_Handler repeats the data/bss init and re-runs
       SystemInit(); interrupts must be enabled before the jump because a reset
       leaves PRIMASK clear but the application does not set it explicitly. */
    SCB->VTOR = base;
    __set_MSP(sp);
    __enable_irq();
    ((void (*)(void))(uintptr_t)entry)();

    for (;;) {              /* not reached */
    }
}

/* ── boot mode ──────────────────────────────────────────────────────────── */

static void clear_boot_stay(void)
{
    cfg_blob_t *blob = boot_regs_blob();

    if (0U != blob->boot.boot_stay) {
        blob->boot.boot_stay = 0U;
        (void)boot_regs_save();
        DBG_INFO("boot_stay cleared");
    }
}

/*! \brief serve the bus until the host reboots us or disappears.
    \retval never, unless the host vanished while a slot was bootable */
static void boot_mode_serve(void)
{
    uint8_t any_bootable = (uint8_t)(s_in.valid[SLOT_A] || s_in.valid[SLOT_B]);
    uint32_t now;

    s_last_frame_ms = systick_get_ms();
    DBG_INFO("boot mode: id dxl=%u fee=%u, serving %u byte blocks at reg %u",
             (unsigned)dev_cfg()->dxl_id, (unsigned)dev_cfg()->fee_id,
             (unsigned)UPGRADE_DATA_LEN, (unsigned)UPGRADE_WIN_ADDR);
    DBG_INFO("boot mode: %s", any_bootable
             ? "host may upgrade; leaving after 30 s of silence"
             : "no bootable image - waiting for an upgrade");

    for (;;) {
        now = systick_get_ms();
        bus_poll(now);

        /* "The host is still there" is about the bus.  The console's receive
           timestamps must not count: with the mirror off, bytes arriving on
           USART0 are somebody typing at the debug console (or line noise), and
           counting them would keep this node in boot mode for as long as that
           goes on instead of handing the robot back after BOOT_IDLE_EXIT_MS. */
        {
            uint8_t host_active = (uint8_t)(bus_ms_since_rx(BUS_PORT_MAIN, now) < 20U);
#if BUS_MIRROR_ENABLE
            /* the bench link is the only other thing that can be a host */
            host_active = (uint8_t)(host_active
                                    || (bus_ms_since_rx(BUS_PORT_MIRROR, now) < 20U));
#endif
            if (0U != host_active) {
                s_last_frame_ms = now;
            }
        }

        if (boot_regs_reboot_pending(now) || upg_reboot_pending(now)) {
            DBG_INFO("rebooting");
            delay_1ms(2U);
            NVIC_SystemReset();
        }

        {
            static uint32_t last_logged;
            if (last_logged != upg_blocks()) {
                last_logged = upg_blocks();
                if (0U == (last_logged % 128U)) {
                    DBG_INFO("upgrade: %u blocks, %u bytes, %u retransmits",
                             (unsigned)last_logged, (unsigned)upg_bytes_written(),
                             (unsigned)upg_retransmits());
                }
            }
        }

        if (any_bootable && (uint32_t)(now - s_last_frame_ms) > BOOT_IDLE_EXIT_MS) {
            DBG_INFO("no host for %u ms: leaving boot mode",
                     (unsigned)BOOT_IDLE_EXIT_MS);
            clear_boot_stay();
            return;
        }

        led_tick(now);
    }
}

/* ── main ───────────────────────────────────────────────────────────────── */

static int persist_trial_attempt(void)
{
    cfg_blob_t *blob = boot_regs_blob();

    blob->boot.trial_slot = s_in.trial_slot;
    blob->boot.attempts = s_out.attempts;
    if (0U != s_out.clear_trial) {
        blob->boot.trial_slot = SLOT_NONE;
        blob->boot.attempts = 0U;
    }
    return boot_regs_save();
}

int main(void)
{
    uint32_t arg;
    uint8_t force_boot;

    systick_config();
    us_time_init();
    board_gpio_init();

    boot_regs_init();
    bus_init();
    dbg_bind(bus_uart(BUS_PORT_MIRROR));
    dbg_init();
    bus_apply_ports();

    DBG_INFO("imu_to_dxl bootloader, build %s %s", __DATE__, __TIME__);
    DBG_INFO("boot: uart1=%u baud, mirror uart0=%u baud",
             (unsigned)uart_port_get_baud(bus_uart(BUS_PORT_MAIN)),
             (unsigned)uart_port_get_baud(bus_uart(BUS_PORT_MIRROR)));

    force_boot = (uint8_t)boot_ram_consume(&arg);
    load_state();
    if (0U != boot_regs_blob()->boot.boot_stay) {
        force_boot = 1U;
    }
    if (0U != force_boot) {
        DBG_INFO("boot mode requested (%s)", (0U != arg) ? "slot switch" : "host");
    }

    validate_slots();

    for (;;) {
        /* The decision is always computed, even when a host asked for boot mode:
           it is what tells the upgrade state machine which slot is the last
           known-good one it must not erase. */
        boot_decide(&s_in, &s_out);
        publish_identity();

        if (0U == force_boot && BOOT_ACT_STAY != s_out.action) {
            if (BOOT_ACT_RUN_TRIAL == s_out.action) {
                DBG_INFO("trial boot %u/%u of slot %c",
                         (unsigned)s_out.attempts, (unsigned)BOOT_ATTEMPTS_MAX,
                         (SLOT_A == s_out.slot) ? 'A' : 'B');
                if (0 != persist_trial_attempt()
                    && s_out.attempts >= BOOT_ATTEMPTS_MAX) {
                    /* the configuration page will not take the counter, so the
                       next reset would try again forever: fall back now */
                    DBG_ERROR("cannot remember the trial attempt, rolling back");
                    s_in.attempts = BOOT_ATTEMPTS_MAX;
                    boot_decide(&s_in, &s_out);
                    publish_identity();
                }
            } else if (0U != s_out.clear_trial) {
                (void)persist_trial_attempt();
            }
            boot_jump(s_out.slot);
        }
        if (BOOT_ACT_STAY == s_out.action) {
            DBG_ERROR("no bootable image in either slot");
        }

        boot_mode_serve();

        /* The host went away while a slot was bootable: clear the request and
           run the decision for real. */
        force_boot = 0U;
        load_state();
        validate_slots();
    }
}
