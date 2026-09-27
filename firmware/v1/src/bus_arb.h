/*!
    \file    bus_arb.h
    \brief   "answer in your own slot": the sync_read reply ordering rule

    A real HD-1910 does not answer a `sync_read` the moment it has parsed the
    instruction; it waits one reply slot for *every* id that precedes its own in
    the id list, whether that device answers or not.  Measured on fw 3.46: one
    absent id ahead costs +0.302 ms, two +0.595, three +0.882 - so a slot is
    ~295 us, and an absent device consumes exactly one slot rather than a long
    timeout.  (This is why the peer design's "1 ms of bus silence means the
    device is absent" rule is wrong: waiting 1 ms when the slot is 295 us puts
    our reply one whole slot late, on top of whatever comes after us.)

    The runtime puts the IMU node first, where no arbitration is needed.  With a
    debug tool that moves it into the middle, transmitting immediately would
    collide with the devices ahead of us on the single-wire bus - destroying
    their replies, not just ours - so the deferred response waits slots x
    BUS_SLOT_US and is cancelled if a new instruction frame addressed to us
    arrives first.

    This module is deliberately free of any hardware dependency: it takes
    `now_us` from the caller, which is what makes it testable on the host (see
    host/tools/test_protocols.c).
*/

#ifndef BUS_ARB_H
#define BUS_ARB_H

#include <stdint.h>

#define BUS_ARB_IDLE    0U
#define BUS_ARB_WAIT    1U
#define BUS_ARB_READY   2U

/*! What a new instruction frame looks like while we are waiting: a FeeTech
    frame carries the id in the third byte, a Dynamixel 2.0 frame starts
    `FF FF FD 00` and carries the id in the fifth.  0xFD is never a valid
    FeeTech id (the protocol reserves it for exactly this reason), so the third
    byte alone tells the two apart. */
typedef struct {
    uint8_t  state;         /* BUS_ARB_*                                  */
    uint8_t  slots_left;    /* preceding ids still to make room for       */
    uint8_t  our_id;        /* abort when an instruction names us ...     */
    uint8_t  rx_state;      /* ... or the broadcast id                    */
    uint32_t slot_us;
    uint32_t deadline_us;
} bus_arb_t;

void bus_arb_init(bus_arb_t *a, uint32_t slot_us);
void bus_arb_set_id(bus_arb_t *a, uint8_t our_id);
void bus_arb_start(bus_arb_t *a, uint32_t now_us, uint8_t slots);
void bus_arb_cancel(bus_arb_t *a);

/*! \brief sniff one received byte; cancels the wait when a new instruction
           frame addressed to us (or broadcast) begins. */
void bus_arb_rx(bus_arb_t *a, uint8_t byte);

/*! \brief advance the slot timer.
    \retval 1 once the response may be transmitted (state stays READY until the
            caller sends it and calls bus_arb_cancel()) */
uint8_t bus_arb_step(bus_arb_t *a, uint32_t now_us);

#endif /* BUS_ARB_H */
