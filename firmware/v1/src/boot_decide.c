/*!
    \file    boot_decide.c
    \brief   see boot_decide.h
*/

#include "boot_decide.h"

static uint8_t slot_ok(uint8_t slot)
{
    return (uint8_t)(slot <= SLOT_B);
}

void boot_decide(const boot_decide_in_t *in, boot_decide_out_t *out)
{
    uint8_t a_ok = in->valid[SLOT_A];
    uint8_t b_ok = in->valid[SLOT_B];
    uint8_t other;

    out->action = BOOT_ACT_STAY;
    out->slot = SLOT_NONE;
    out->attempts = in->attempts;
    out->clear_trial = 0U;

    if (0U == a_ok && 0U == b_ok) {
        return;                             /* nothing to run */
    }
    if (0U == b_ok) {
        out->action = BOOT_ACT_RUN;
        out->slot = SLOT_A;
        return;
    }
    if (0U == a_ok) {
        out->action = BOOT_ACT_RUN;
        out->slot = SLOT_B;
        return;
    }

    /* both slots are bootable from here on */

    /* 1. an image on trial gets its attempts, then is dropped */
    if (slot_ok(in->trial_slot) && 0U != in->valid[in->trial_slot]) {
        if (in->attempts < BOOT_ATTEMPTS_MAX) {
            out->action = BOOT_ACT_RUN_TRIAL;
            out->slot = in->trial_slot;
            out->attempts = (uint8_t)(in->attempts + 1U);
            return;
        }
        out->action = BOOT_ACT_ROLLBACK;
        out->clear_trial = 1U;
        out->attempts = 0U;
        if (slot_ok(in->boot_slot) && in->boot_slot != in->trial_slot) {
            out->slot = in->boot_slot;
        } else {
            out->slot = (uint8_t)((in->trial_slot == SLOT_A) ? SLOT_B : SLOT_A);
        }
        return;
    }

    /* 2. the slot that was confirmed last time wins */
    if (slot_ok(in->boot_slot)) {
        out->action = BOOT_ACT_RUN;
        out->slot = in->boot_slot;
        return;
    }

    /* 3. no history: the newer image wins, ties go to the ping-pong parity */
    if (in->version[SLOT_A] > in->version[SLOT_B]) {
        out->action = BOOT_ACT_RUN;
        out->slot = SLOT_A;
        return;
    }
    if (in->version[SLOT_B] > in->version[SLOT_A]) {
        out->action = BOOT_ACT_RUN;
        out->slot = SLOT_B;
        return;
    }
    other = (uint8_t)(in->pingpong & 1U);
    out->action = BOOT_ACT_RUN;
    out->slot = other;
}
