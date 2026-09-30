/*!
    \file    main.c
    \brief   imu_to_dxl v1: main loop

    The protocol responses no longer run here at all - a frame is parsed in the
    USART interrupt and its reply goes out over DMA, because a `sync_read` burst
    gives this node one reply slot (~295 us) to start answering.  What is left
    for the main loop is the sensor, the register image (dev_refresh), the
    deferred-reply timer, housekeeping, and one number worth watching: how long
    an iteration takes, since that is now the only thing that can delay a reply
    that is waiting for its slot.
*/

#include "board.h"

#include "app_header.h"
#include "bus.h"
#include "dbg.h"
#include "dev.h"
#include "imu.h"
#include "imu_math.h"
#include "imu_sim.h"
#if IMU_USE_SPI
#include "imu_spi.h"
#endif
#include "systick.h"
#include "us_time.h"

#define STATS_PERIOD_MS     2000U
#define LED_ACTIVE_MS       500U
#define LED_IDLE_MS         125U
#define LED_IDLE_AFTER_MS   5000U

#if STATUS_LED_ENABLE
static uint8_t s_led_on;
#endif

/* worst main-loop iteration since the last stats line; a long one is the only
   thing that can make a deferred reply miss its slot */
static uint32_t s_loop_max_us;

static void board_gpio_init(void)
{
#if STATUS_LED_ENABLE
    rcu_periph_clock_enable(RCU_GPIOB);
    gpio_init(LED_GPIO, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, LED_PIN);
    gpio_bit_reset(LED_GPIO, LED_PIN);
#endif

    /* IMU pins: chip select driven high so a fitted sensor stays deselected
       while no SPI driver is running.  SCK/MOSI/MISO are left as inputs. */
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
    uint32_t idle = bus_ms_since_rx(BUS_PORT_MIRROR, now_ms);
    uint32_t period = (idle > LED_IDLE_AFTER_MS) ? LED_IDLE_MS : LED_ACTIVE_MS;

    if (0U == next_ms) {
        next_ms = now_ms + period;
    }
    if ((int32_t)(now_ms - next_ms) >= 0) {
        next_ms = now_ms + period;
        s_led_on = (uint8_t)(0U == s_led_on);
        if (s_led_on) {
            gpio_bit_set(LED_GPIO, LED_PIN);
        } else {
            gpio_bit_reset(LED_GPIO, LED_PIN);
        }
    }
#else
    (void)now_ms;
#endif
}

/*! \brief how many transactions this node has answered on either port.

    Used as the "somebody is talking to us" signal that lets a trial image
    confirm itself after BOOT_CONFIRM_MS instead of waiting twice as long. */
static uint32_t bus_answered_total(void)
{
    bus_stats_t mirror;
    bus_stats_t main_port;

    bus_get_stats(BUS_PORT_MIRROR, &mirror);
    bus_get_stats(BUS_PORT_MAIN, &main_port);
    return mirror.answered + main_port.answered;
}

static const char *proto_name(uint8_t p)
{
    switch (p) {
    case PROTO_DXL: return "dxl2.0";
    case PROTO_FEE: return "feetech";
    default:        return "none";
    }
}

#if !IMU_USE_SPI
static const char *sim_name(uint8_t m)
{
    switch (m) {
    case SIM_STATIC:    return "static";
    case SIM_SINE:      return "sine";
    case SIM_DRIFT:     return "drift";
    case SIM_NOISE:     return "noise";
    case SIM_SPIN:      return "spin";
    case SIM_SFLP_WAIT: return "sflp-wait";
    case SIM_FROZEN:    return "frozen";
    default:            return "?";
    }
}
#endif

static void stats_print(uint32_t now_ms)
{
    static uint32_t next_ms;
    const imu_sample_t *s = imu_latest();
    const dev_cfg_t *cfg = dev_cfg();
    bus_stats_t m;
    bus_stats_t d;

    if (0U == next_ms) {
        next_ms = now_ms + STATS_PERIOD_MS;
    }
    if ((int32_t)(now_ms - next_ms) < 0) {
        return;
    }
    next_ms = now_ms + STATS_PERIOD_MS;

    bus_get_stats(BUS_PORT_MIRROR, &d);
    bus_get_stats(BUS_PORT_MAIN, &m);
    DBG_INFO("t=%u temp=%uC sensor=%s samples=%u qvalid=%u",
             (unsigned)(now_ms / 1000U), (unsigned)dev_temp_celsius(),
             imu_is_simulated() ? "sim" : "spi", (unsigned)imu_samples(),
             (unsigned)((s->flags & TELEM_FLAG_SFLP_VALID) != 0U));
#if IMU_USE_SPI
    DBG_INFO("  lsm6dsv16x who=0x%02X present=%u sflp_words=%u ovr=%u badtag=%u probe_fail=%u",
             (unsigned)imu_spi_whoami(), (unsigned)imu_spi_present(),
             (unsigned)imu_spi_sflp_words(), (unsigned)imu_spi_fifo_overruns(),
             (unsigned)imu_spi_bad_tags(), (unsigned)imu_spi_probe_fails());
#else
    DBG_INFO("  sim mode=%s amp=%u freq=%u",
             sim_name(cfg->sim_mode), (unsigned)cfg->sim_amp,
             (unsigned)cfg->sim_freq);
#endif
    DBG_INFO("  cfg id(dxl=%u fee=%u) baud(dxl=%u fee=%u console=%u) frame=%s lock=%s dirty=0x%02X",
             (unsigned)cfg->dxl_id, (unsigned)cfg->fee_id,
             (unsigned)uart_port_get_baud(bus_uart(BUS_PORT_MAIN)),
             (unsigned)dev_fee_baud(),
             (unsigned)uart_port_get_baud(bus_uart(BUS_PORT_MIRROR)),
             (cfg->report_frame ? "trunk" : "chip"), proto_name(cfg->proto_lock),
             (unsigned)dev_image(PROTO_DXL)[DXL_VENDOR_ADDR + V_STATUS]);
    DBG_INFO("  uart0 rx=%u tx=%u dropped=%u err=%u echo=%u frames=%u answered=%u proto=%s",
             (unsigned)d.rx_bytes, (unsigned)d.tx_bytes, (unsigned)d.rx_dropped,
             (unsigned)d.rx_errors, (unsigned)d.echoed, (unsigned)d.frames,
             (unsigned)d.answered, proto_name(d.last_proto));
    DBG_INFO("  uart1 rx=%u tx=%u dropped=%u err=%u echo=%u frames=%u answered=%u proto=%s",
             (unsigned)m.rx_bytes, (unsigned)m.tx_bytes, (unsigned)m.rx_dropped,
             (unsigned)m.rx_errors, (unsigned)m.echoed, (unsigned)m.frames,
             (unsigned)m.answered, proto_name(m.last_proto));
    DBG_INFO("  bus held=%u/%u lost=%u gaps=%u ignored=%u loop_max=%uus",
             (unsigned)d.deferred, (unsigned)m.deferred,
             (unsigned)(d.tx_lost + m.tx_lost), (unsigned)(d.gaps + m.gaps),
             (unsigned)(d.ignored + m.ignored), (unsigned)s_loop_max_us);
    s_loop_max_us = 0U;
    {
        /* integer millidegrees/s: nano.specs does not format floats unless
           -u _printf_float is linked, and the console does not need them */
        long gx = (long)lroundf_i32((float)s->gyro[0] * IMU_GYRO_MDPS_PER_LSB);
        long gy = (long)lroundf_i32((float)s->gyro[1] * IMU_GYRO_MDPS_PER_LSB);
        long gz = (long)lroundf_i32((float)s->gyro[2] * IMU_GYRO_MDPS_PER_LSB);
        DBG_INFO("  block gyro(mdps) %ld %ld %ld accel(mg) %ld %ld %ld",
                 gx, gy, gz,
                 (long)lroundf_i32((float)s->accel[0] * IMU_ACCEL_MG_PER_LSB),
                 (long)lroundf_i32((float)s->accel[1] * IMU_ACCEL_MG_PER_LSB),
                 (long)lroundf_i32((float)s->accel[2] * IMU_ACCEL_MG_PER_LSB));
        DBG_INFO("  block quat(xyzh) %04X %04X %04X flags 0x%02X cnt=%u",
                 (unsigned)s->quat[0], (unsigned)s->quat[1], (unsigned)s->quat[2],
                 (unsigned)s->flags, (unsigned)s->counter);
    }
}

int main(void)
{
    uint32_t last_seq;
    uint32_t now;
    uint32_t boot_check_ms = 0U;

#if (APP_SLOT == 1)
    SCB->VTOR = SLOT_A_BASE;
#elif (APP_SLOT == 2)
    SCB->VTOR = SLOT_B_BASE;
#endif

    systick_config();
    us_time_init();
    board_gpio_init();
    bus_init();
    dbg_bind(bus_uart(BUS_PORT_MIRROR));
    dbg_init();

    dev_init();
    imu_init();
    dev_refresh();

    last_seq = dev_cfg_seq();
    bus_apply_ports();

    DBG_INFO("%s fw %u.%u", FW_BUILD_NAME, FW_VERSION_MAJOR, FW_VERSION_MINOR);
    DBG_INFO("dxl id=%u (v2.0, block at %u), feetech id=%u (block at %u)",
             (unsigned)dev_cfg()->dxl_id, (unsigned)DXL_TELEM_ADDR,
             (unsigned)dev_cfg()->fee_id, (unsigned)FEE_TELEM_ADDR);
    DBG_INFO("bus uart1=%u baud, mirror uart0=%u baud, sensor=%s",
             (unsigned)uart_port_get_baud(bus_uart(BUS_PORT_MAIN)),
             (unsigned)uart_port_get_baud(bus_uart(BUS_PORT_MIRROR)),
             imu_is_simulated() ? "simulated" : "lsm6dsv16x/spi0");
#if IMU_USE_SPI
    DBG_INFO("sensor frame=%s, vendor bias reg %u; gyro +-500dps @ %.1f mdps/LSB",
             dev_cfg()->report_frame ? "trunk" : "chip",
             (unsigned)(DXL_VENDOR_ADDR + V_BIAS_X_L),
             (double)IMU_GYRO_MDPS_PER_LSB);
#else
    DBG_INFO("sim mode=%s - write vendor reg %u (magic I M, cmd) to change",
             sim_name(dev_cfg()->sim_mode), (unsigned)(DXL_VENDOR_ADDR + V_SIM_MODE));
#endif
    {
        const app_header_t *own = app_own_header();
        if (NULL != own) {
            DBG_INFO("image v%u.%u.%u in slot %c, header %s",
                     (unsigned)APP_VERSION_MAJOR(own->image_version),
                     (unsigned)APP_VERSION_MINOR(own->image_version),
                     (unsigned)APP_VERSION_PATCH(own->image_version),
                     (SLOT_A == app_own_slot()) ? 'A' : 'B',
                     (APP_HDR_OK == app_own_header_check()) ? "valid" : "invalid");
        } else {
            DBG_INFO("development image (no bootloader, no header)");
        }
        DBG_INFO("boot state: slot=%d trial=%d attempts=%u stay=%u pingpong=%u",
                 (int)dev_boot_boot_slot(), (int)dev_boot_trial_slot(),
                 (unsigned)0, (unsigned)dev_boot_stay(),
                 (unsigned)dev_boot_pingpong());
    }

    while (1) {
        uint32_t loop_start_us = us_now();
        uint32_t samples;
        uint32_t took_us;

        now = systick_get_ms();

        imu_tick(now);

        /* Publish the register image as soon as the sensor has a new sample,
           and a few times a second regardless so the fields that drift on their
           own (the MCU temperature, the free-running tick) stay current.  The
           reads themselves run in the USART interrupt and must not touch the
           ADC or the vendor window, which is why this lives here. */
        {
            static uint32_t last_samples;
            static uint32_t last_publish_ms;
            samples = imu_samples();
            if ((samples != last_samples) || ((uint32_t)(now - last_publish_ms) >= 50U)) {
                last_samples = samples;
                last_publish_ms = now;
                dev_refresh();
            }
        }

        bus_poll(now);
        {
            /* The automatic config save waits for both ports to be quiet: a
               flash page erase stalls the core for tens of ms, and on the real
               robot that would eat a control-loop tick. */
            uint32_t idle_a = bus_ms_since_rx(BUS_PORT_MIRROR, now);
            uint32_t idle_b = bus_ms_since_rx(BUS_PORT_MAIN, now);
            dev_tick(now, (idle_a < idle_b) ? idle_a : idle_b);
        }

        if (dev_cfg_seq() != last_seq) {
            last_seq = dev_cfg_seq();
            bus_apply_ports();
#if IMU_USE_SPI
            DBG_INFO("config applied: id=%u/%u frame=%s baud=%u console=%u",
                     (unsigned)dev_cfg()->dxl_id, (unsigned)dev_cfg()->fee_id,
                     dev_cfg()->report_frame ? "trunk" : "chip",
                     (unsigned)uart_port_get_baud(bus_uart(BUS_PORT_MAIN)),
                     (unsigned)uart_port_get_baud(bus_uart(BUS_PORT_MIRROR)));
#else
            DBG_INFO("config applied: id=%u/%u sim=%s baud=%u console=%u",
                     (unsigned)dev_cfg()->dxl_id, (unsigned)dev_cfg()->fee_id,
                     sim_name(dev_cfg()->sim_mode),
                     (unsigned)uart_port_get_baud(bus_uart(BUS_PORT_MAIN)),
                     (unsigned)uart_port_get_baud(bus_uart(BUS_PORT_MIRROR)));
#endif
        }

        /* Trial images confirm themselves once they have run long enough
           (docs/flash_layout.md §4.4); checked once a second, not every loop. */
        if ((int32_t)(now - boot_check_ms) >= 0) {
            boot_check_ms = now + 1000U;
            if (dev_boot_tick(now, bus_answered_total())) {
                DBG_INFO("this trial image is now confirmed");
            }
        }

        led_tick(now);
        stats_print(now);

        /* High-water mark of one iteration.  A deferred reply is sent from
           here, so this is the number that says whether the main loop can still
           keep the bus contract (the target is well under a reply slot). */
        took_us = us_elapsed(loop_start_us, us_now());
        if (took_us > s_loop_max_us) {
            s_loop_max_us = took_us;
        }

        if (dev_reboot_pending(now)) {
            DBG_INFO("reboot requested");
            delay_1ms(2U);
            NVIC_SystemReset();
        }
    }
}
