/*!
    \file    bus.h
    \brief   the two bus ports, protocol auto-detection, and the reply-slot rule

    Port 0 is USART0 (PB6/PB7) - the debug console.  It only ever prints DBG_*
    lines; `BUS_MIRROR_ENABLE` (board.h) turns it into a second bus port for
    bench work on a board that has no motor bus wired yet.
    Port 1 is USART1 (PA2/PA3) - the real motor bus, half duplex, whose TX
    enable is generated in hardware from TX (`TXD_EN` in the schematic), so the
    firmware only has to remember that it hears its own transmission.  The
    register protocol *and* the field upgrade run here and nowhere else.

    Auto-detection: a Dynamixel 2.0 frame starts `FF FF FD`, a FeeTech frame is
    `FF FF <id>` with id <= 0xFC, so the third byte decides.  After a frame is
    accepted the port sticks to that protocol for BUS_PROTO_STICKY_MS, which is
    what keeps a stray `FF FF <our id>` inside a servo payload from being taken
    for a FeeTech frame.  A register (vendor V_PROTO_LOCK) can force one.

    Timing (the numbers are measured, see docs/bus_timing_borrow_plan.md):

      * the receive path runs in the USART interrupt and the reply goes out over
        DMA, because a `sync_read` burst gives this node ~295 us from the last
        bit of the instruction to its own first bit - one reply slot, which is
        all the devices behind it wait before they start talking;
      * a `sync_read` that names us at index k>0 is deferred by k slots
        (src/bus_arb.c), so a debug tool may move the IMU node anywhere in the
        id list without our reply colliding with the devices ahead of us.
*/

#ifndef BUS_H
#define BUS_H

#include <stdint.h>

#include "board.h"
#include "uart_port.h"

#define BUS_PORT_MIRROR     0U
#define BUS_PORT_MAIN       1U
#define BUS_PORT_COUNT      2U

/* How long a port stays with the protocol it last answered.  Long enough to
   cover the frame in progress (a DXL payload can contain `FF FF <our id>` and
   must not be taken for a FeeTech frame), short enough that a host switching
   protocols is not ignored for a second.  Measured need: the bench smoke test
   switches protocols back to back. */
#define BUS_PROTO_STICKY_MS 100U

/* One reply slot lives in board.h (BUS_SLOT_US): the slot rule is part of the
   wire contract, and src/bus_arb.c - which is deliberately free of any bus or
   hardware dependency - needs it too.

   A frame whose bytes are further apart than this is not one frame: drop the
   partial frame and re-sync on the next FF FF.  Well above the ~300 us slot of
   a live burst, far below the ~20 ms between host transactions. */
#define BUS_GAP_US          500U
/* The echo guard is not a byte count and does not live here: it is a *time*
   window after our own transmission completes (`UART_ECHO_WINDOW_BYTES` in
   src/uart_port.h), because whether our reply comes back at all depends on the
   transceiver.  A fixed byte count sat here once and silently ate the next
   request's `FF FF` on the real PCBA - see docs/bus_timing_borrow_plan.md §4.6.1. */

typedef struct {
    uint32_t rx_bytes;
    uint32_t tx_bytes;
    uint32_t rx_dropped;
    uint32_t rx_errors;
    uint32_t echoed;
    uint32_t frames;
    uint32_t answered;
    uint32_t responses;
    uint32_t bad_crc;
    uint32_t bad_len;
    uint32_t ignored;
    uint32_t deferred;      /* sync_read replies held for their slot  */
    uint32_t tx_lost;       /* replies dropped because TX was still busy */
    uint32_t gaps;          /* partial frames dropped by the gap rule */
    uint8_t  last_proto;
    uint8_t  enabled;
} bus_stats_t;

void bus_init(void);

/*! \brief housekeeping on the ports: send a reply whose slot has come up, and
           nothing else - frames are parsed in the USART interrupt. */
void bus_poll(uint32_t now_ms);

/*! \brief re-apply baud rates after a configuration change. */
void bus_apply_ports(void);

void bus_get_stats(uint8_t index, bus_stats_t *out);
uint8_t bus_last_proto(uint8_t index);
uint32_t bus_ms_since_rx(uint8_t index, uint32_t now_ms);

/*! \brief the underlying port, so the console can write to it. */
uart_port_t *bus_uart(uint8_t index);

#endif /* BUS_H */
