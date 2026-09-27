/*!
    \file    boot_decide.h
    \brief   the boot decision, as a pure function so it can be tested on a PC

    Inputs: which slots are bootable (their header validated), their versions,
    and the persisted boot state.  Output: what to run and what to persist.
    Nothing here touches hardware, which is why `host/tools/test_boot.c` can
    walk the whole decision table (docs/flash_layout.md §4.3).

    Rules in one sentence: a valid slot always beats an invalid one; a slot on
    trial wins while it has attempts left; after BOOT_ATTEMPTS_MAX failed trial
    boots the trial is dropped and the previously confirmed slot runs; with two
    valid slots and no history the higher version wins, ties broken by the
    ping-pong counter.
*/

#ifndef BOOT_DECIDE_H
#define BOOT_DECIDE_H

#include <stdint.h>

#include "board.h"

typedef struct {
    uint8_t  valid[2];      /* per slot: 1 = header valid and bootable */
    uint16_t version[2];    /* image_version of each slot               */
    uint8_t  boot_slot;     /* SLOT_A / SLOT_B / SLOT_NONE              */
    uint8_t  trial_slot;    /* SLOT_A / SLOT_B / SLOT_NONE              */
    uint8_t  attempts;      /* trial boots already attempted            */
    uint8_t  pingpong;      /* freshness counter                        */
} boot_decide_in_t;

#define BOOT_ACT_STAY       0U  /* no bootable slot: wait for a host */
#define BOOT_ACT_RUN        1U  /* run `slot`                        */
#define BOOT_ACT_RUN_TRIAL  2U  /* run `slot`, count the attempt     */
#define BOOT_ACT_ROLLBACK   3U  /* drop the trial, run the old slot  */

typedef struct {
    uint8_t action;         /* BOOT_ACT_*                       */
    uint8_t slot;           /* SLOT_A / SLOT_B / SLOT_NONE      */
    uint8_t attempts;       /* value to persist for next boot   */
    uint8_t clear_trial;    /* 1 = forget the trial slot        */
} boot_decide_out_t;

/*! \brief fill `out` from `in`; always succeeds. */
void boot_decide(const boot_decide_in_t *in, boot_decide_out_t *out);

#endif /* BOOT_DECIDE_H */
