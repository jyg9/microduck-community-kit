/*!
    \file    boot_upgrade.h
    \brief   the bus upgrade state machine (both protocols share one window)

    The host drives this with ordinary register writes - `INST_WRITE` for
    Dynamixel, `INST_WRITE` for FeeTech - into the 48-byte window at
    `UPGRADE_WIN_ADDR` (208..255, see board.h and docs/flash_layout.md §5).
    There is no new frame type, no new checksum layer: `dxl2.c` / `fee.c`
    already validated the frame, and this module only has to be careful about
    *where* it programs flash.

    Safety properties, in the order they matter:

      1. only slot A and slot B are ever erased or programmed
         (`boot_flash_slot_*` refuses anything else), so a bad session cannot
         touch the bootloader or the configuration page;
      2. the slot the bootloader is about to run is refused as a target while
         the other slot is not bootable, so the last good image is not destroyed;
      3. every block is covered by its own CRC-16 and every image by a two
         segment CRC-32 that the node recomputes independently;
      4. the image header is only written after the whole image verified, and its
         `magic` word is programmed last, so a power cut in the middle of the
         header leaves an invalid header rather than a half-valid one;
      5. nothing is committed as bootable until the header passes
         `app_header_check()` again, read back from flash.
*/

#ifndef BOOT_UPGRADE_H
#define BOOT_UPGRADE_H

#include <stdint.h>

#include "board.h"

typedef struct {
    uint8_t run_slot;           /* slot the bootloader would boot, or SLOT_NONE */
    uint8_t other_bootable;     /* 1 = the *other* slot holds a valid image     */
} upg_ctx_t;

/*! \brief what the state machine calls when it needs to touch the
           configuration page.  The bootloader implements these in
           `boot_regs.c`; the host tests pass their own recorders. */
typedef struct {
    /*! \brief a complete image verified and was accepted: put `slot` on trial
               (trial_slot = slot, attempts = 0, pingpong + 1) and persist. */
    void (*on_commit)(uint8_t slot);
    /*! \brief UPG_REBOOT was accepted: clear `boot_stay` (if set) and persist,
               so the reset boots the decision instead of the bootloader. */
    void (*on_reboot)(void);
    /*! \brief called once per successful block, for the console log. */
    void (*on_block)(uint32_t offset, uint8_t len, uint16_t seq, int retransmit);
} upg_hooks_t;

typedef struct {
    uint8_t  win[UPGRADE_WIN_LEN];   /* the register window, as the host sees it */
    uint8_t  armed;                  /* magic present                            */
    uint8_t  active;                 /* BEGIN accepted, session in progress      */
    uint8_t  target;                 /* SLOT_A / SLOT_B                           */
    uint8_t  status;                 /* UPG_ST_*                                 */
    uint8_t  errcode;                /* UPG_ERR_*                                */
    uint16_t ack_seq;                /* last accepted block sequence             */
    uint16_t ver;                    /* image_version from the host              */
    uint32_t imglen;                 /* image length from the host               */
    uint32_t imgcrc;                 /* image CRC-32 from the host               */
    uint32_t nodecrc;                /* image CRC-32 the node computed           */
    uint32_t bytes_written;
    uint32_t blocks;
    uint32_t retransmits;
    uint32_t erase_pages;
    uint32_t reboot_at_ms;
    uint8_t  reboot_pending;
    upg_ctx_t ctx;
} upg_t;

/*! \brief the single upgrade session of this node. */
extern upg_t g_upg;

/*! \brief reset the window and drop any session. */
void upg_init(void);

/*! \brief tell the state machine which slot must be protected. */
void upg_set_context(const upg_ctx_t *ctx);

/*! \brief install the configuration-page callbacks (may be NULL). */
void upg_set_hooks(const upg_hooks_t *hooks);

/*! \brief the host wrote `n` bytes into the window at `off`.
    \param[in] now_ms millisecond tick (for the delayed reboot command)
    \retval 0, or a protocol error byte (DXL_ERR_*) for an illegal write */
uint8_t upg_write(uint8_t off, const uint8_t *p, uint8_t n, uint32_t now_ms);

/*! \brief recompute the read-only window bytes (status, ack, errcode, CRC). */
void upg_refresh(void);

/*! \brief one byte of the window, as a `READ` should return it. */
uint8_t upg_read(uint8_t off);

/*! \brief 1 once a `UPG_REBOOT` response has had time to leave the node. */
int upg_reboot_pending(uint32_t now_ms);

/*! \brief session bookkeeping for the console/stats. */
uint32_t upg_bytes_written(void);
uint32_t upg_blocks(void);
uint32_t upg_retransmits(void);

#endif /* BOOT_UPGRADE_H */
