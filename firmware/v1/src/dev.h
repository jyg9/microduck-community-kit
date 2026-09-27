/*!
    \file    dev.h
    \brief   the node's register space: one flat 256-byte image per protocol,
             plus the typed configuration the firmware actually acts on

    The two protocols have different address maps (Dynamixel follows the XL330
    control table, FeeTech follows the HLS memory table), so there is one image
    per protocol.  The image is what the wire sees; the typed `dev_cfg_t` is
    what the firmware obeys.  Host writes land in the image first and are then
    pulled into the typed configuration by `dev_write`, which keeps multi-byte
    writes and spanning reads trivially correct.

    The IMU block is *aliased* into both images so one `sync_read` can fetch it
    together with the servos:
      * Dynamixel: address 124, 20 bytes (present_pwm .. position_trajectory)
      * FeeTech:   address 56, 20 bytes (present_position .. present_current)
                   and again at 128 (vendor window)
*/

#ifndef DEV_H
#define DEV_H

#include <stdint.h>

#include "board.h"

typedef struct {
    uint8_t  dxl_id;            /* Dynamixel ID (register 7)            */
    uint8_t  dxl_baud_code;     /* Dynamixel baud register (8): 3 = 1M  */
    uint8_t  fee_id;            /* FeeTech ID (register 5)              */
    uint8_t  fee_baud_code;     /* FeeTech baud register (6): 0 = 1M    */
    uint8_t  sim_mode;          /* SIM_*                                */
    uint16_t sim_amp;           /* per mille of the nominal amplitude   */
    uint16_t sim_freq;          /* per mille of the nominal frequency   */
    uint8_t  report_frame;      /* 0 = chip frame (default), 1 = trunk  */
    uint8_t  proto_lock;        /* PROTO_NONE = auto, else forced       */
    int16_t  gyro_bias[3];      /* raw counts added to the gyro output  */
    uint8_t  mirror_baud_code;  /* FeeTech baud code for the mirror port*/
    uint8_t  dbg_level;         /* 0 off, 1 boot+errors, 2 +stats, 3 +frames */
} dev_cfg_t;

void dev_init(void);

/*!
    \brief  background work: persist the configuration once the bus has been
            quiet for a while.
    \param[in] now_ms: millisecond tick
    \param[in] bus_idle_ms: how long since the last byte arrived on any bus port

    A flash page erase stalls the core (and therefore the USART interrupts) for
    tens of milliseconds, which would cost a control-loop tick if it happened
    while the robot is polling.  Deferring until the bus is quiet keeps the
    automatic save invisible; an explicit VCMD_SAVE still writes immediately and
    the host must tolerate one missed read.
*/
void dev_tick(uint32_t now_ms, uint32_t bus_idle_ms);

const dev_cfg_t *dev_cfg(void);
/*! \brief bumped on every configuration change; the bus uses it to re-apply
           port settings without dev.c having to know about the bus. */
uint32_t dev_cfg_seq(void);

/*! \brief the factory defaults, shared with the bootloader (which needs the
           same ID/baud without linking dev.c). */
void dev_cfg_set_defaults(dev_cfg_t *cfg);

void dev_cfg_defaults(void);
void dev_cfg_save_now(void);
int  dev_cfg_from_flash(void);

/* ── boot state and the upgrade handshake (application side) ────────────── */

/*! \brief read the persisted boot state and publish the identity window.
           Called by dev_init(). */
void dev_boot_init(void);

uint8_t dev_boot_trial_slot(void);      /* SLOT_A / SLOT_B / SLOT_NONE */
uint8_t dev_boot_boot_slot(void);       /* SLOT_A / SLOT_B / SLOT_NONE */
uint8_t dev_boot_stay(void);            /* 1 = the bootloader is holding  */
uint8_t dev_boot_pingpong(void);

/*! \brief VCMD_BOOT: ask the bootloader for a session and reset into it. */
void dev_boot_request_boot(uint32_t now_ms);
/*! \brief VCMD_SWITCH_SLOT: boot the other slot on the next reset. */
void dev_boot_request_switch(uint32_t now_ms);
/*! \brief VCMD_CONFIRM: accept the running image immediately. */
void dev_boot_confirm(uint32_t now_ms);

/*! \brief trial self-confirmation: after BOOT_CONFIRM_MS of healthy running
           (with at least one bus transaction) or BOOT_CONFIRM_LONE_MS without
           any host at all, record the image as the confirmed one.
    \param[in] frames_ok total bus transactions answered so far
    \retval 1 when this call confirmed the image */
int dev_boot_tick(uint32_t now_ms, uint32_t frames_ok);

/*! \brief 1 once a vendor "reboot" command's response has had time to leave. */
int dev_reboot_pending(uint32_t now_ms);

/*! \brief ask for a software reset after the current response is out. */
void dev_request_reboot(uint32_t now_ms);

/*! \brief restore factory defaults in RAM (and persist them). */
void dev_factory_reset(void);

/*! \brief clear the latched hardware error status (Dynamixel CLEAR). */
void dev_clear_hw_error(void);

/*! \brief refresh the dynamic fields of both images (telemetry, temperature,
           realtime tick, vendor status).

    Called by the *main loop*, not by every read: the reads run in the USART
    interrupt, where an ADC conversion and a vendor-window rewrite have no
    business.  It publishes under a short critical section, so an interrupt
    that arrives mid-publish sees either the old image or the new one. */
void dev_refresh(void);

/*! \brief remember the last protocol error for the vendor window's V_LAST_ERR.
    Called from the protocol handlers (interrupt context), so this is a plain
    byte store.  The reply's own status byte stays 0: FeeTech tooling reads a
    nonzero ack status as a servo fault. */
void dev_note_error(uint8_t proto, uint8_t err);

/*! \brief read/write through a protocol's address map.
    \retval 0 or a protocol error byte (DXL_ERR_*) */
uint8_t dev_read(uint8_t proto, uint16_t addr, uint16_t len, uint8_t *out);
uint8_t dev_write(uint8_t proto, uint16_t addr, uint16_t len, const uint8_t *in);

const uint8_t *dev_image(uint8_t proto);

/*! \brief on-chip temperature for the `present temperature` register. */
uint16_t dev_temp_celsius(void);

/*! \brief baud rates implied by the configured register codes. */
uint32_t dev_dxl_baud(void);
uint32_t dev_fee_baud(void);
uint32_t dev_mirror_baud(void);

#endif /* DEV_H */
