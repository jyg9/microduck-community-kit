/*!
    \file    bus.c
    \brief   see bus.h

    Two things here are timing-critical and shaped by measurement rather than
    taste (docs/bus_timing_borrow_plan.md):

      * frames are parsed in the USART interrupt, not in the main loop.  A
        `sync_read` burst gives this node one reply slot (~295 us on the real
        bus) to start answering; the main loop cannot promise that, and the
        interrupt costs a few hundred cycles per byte.
      * a `sync_read` that names us at index k>0 is deferred by k reply slots
        (src/bus_arb.c).  The runtime always puts the IMU node first, where no
        arbitration happens - this is for debug tools that move it into the
        middle, where transmitting immediately would talk over the devices ahead
        of us and destroy *their* replies on the single-wire bus.
      * a Fast Sync Read (0x8A) has no reply slots to count: the devices build
        one packet between them and each block is shorter than a slot, so the
        block of a node that is not first is timed by the bytes ahead of it
        (src/dxl2.c, `dxl_slave_fast_rx`).  It goes out from the same interrupt
        path, one byte after the block before it ends.
*/

#include "bus.h"

#include <stddef.h>
#include <string.h>

#include "bus_arb.h"
#include "dbg.h"
#include "dev.h"
#include "dxl2.h"
#include "fee.h"
#include "systick.h"
#include "us_time.h"

typedef struct {
    uart_port_t  port;
    dxl_slave_t  dxl;
    fee_slave_t  fee;
    bus_arb_t    arb;
    uint8_t      last_proto;
    uint32_t     last_proto_ms;
    uint8_t      enabled;
    const char  *name;
    /* One reply buffer per port: DMA reads it while the interrupt may already
       be assembling the next frame's reply. */
    uint8_t      tx[DXL_MAX_RESP];
    uint8_t      defer[DXL_MAX_RESP];
    uint16_t     defer_len;
    uint8_t      defer_pending;
    /* inter-byte gap rule */
    uint32_t     last_us;
    uint8_t      have_last;
    /* statistics */
    uint32_t     gaps;
    uint32_t     tx_lost;
    uint32_t     deferred;
} bus_port_t;

static bus_port_t s_ports[BUS_PORT_COUNT];

static void proto_allow(const bus_port_t *bp, uint8_t lock, uint32_t now_ms,
                        uint8_t *allow_dxl, uint8_t *allow_fee)
{
    if (PROTO_DXL == lock) {
        *allow_dxl = 1U;
        *allow_fee = 0U;
        return;
    }
    if (PROTO_FEE == lock) {
        *allow_dxl = 0U;
        *allow_fee = 1U;
        return;
    }
    if ((PROTO_NONE != bp->last_proto)
        && ((uint32_t)(now_ms - bp->last_proto_ms) < BUS_PROTO_STICKY_MS)) {
        *allow_dxl = (uint8_t)(PROTO_DXL == bp->last_proto);
        *allow_fee = (uint8_t)(PROTO_FEE == bp->last_proto);
        return;
    }
    *allow_dxl = 1U;
    *allow_fee = 1U;
}

/* ── transmit ───────────────────────────────────────────────────────────── */

static void bus_note_proto(bus_port_t *bp, uint8_t proto, uint32_t now_ms)
{
    bp->last_proto = proto;
    bp->last_proto_ms = now_ms;
}

static void bus_emit(bus_port_t *bp, uint8_t proto, uint16_t len, uint32_t now_us)
{
    const dxl_slave_t *dxl = &bp->dxl;
    const fee_slave_t *fee = &bp->fee;
    uint8_t named = (PROTO_DXL == proto) ? dxl->sync_named : fee->sync_named;
    uint8_t index = (PROTO_DXL == proto) ? dxl->sync_index : fee->sync_index;
    uint8_t inst = (PROTO_DXL == proto) ? dxl->last_inst : fee->last_inst;
    uint8_t sync_inst = (PROTO_DXL == proto) ? DXL_INST_SYNC_READ : FEE_INST_SYNC_READ;

    bus_note_proto(bp, proto, systick_get_ms());

    /* A newer frame supersedes a reply still waiting for its slot: the host has
       moved on, and the old sync_read is not worth answering any more. */
    if (0U != bp->defer_pending) {
        bp->defer_pending = 0U;
        bus_arb_cancel(&bp->arb);
    }

    if ((inst == sync_inst) && (0U != named) && (index > 0U)) {
        memcpy(bp->defer, bp->tx, len);
        bp->defer_len = len;
        bp->defer_pending = 1U;
        bp->deferred++;
        bus_arb_set_id(&bp->arb, (PROTO_DXL == proto) ? dev_cfg()->dxl_id
                                                      : dev_cfg()->fee_id);
        bus_arb_start(&bp->arb, now_us, index);
        DBG_TRACE("%s: sync_read reply held for %u slots", bp->name, (unsigned)index);
        return;
    }

    /* Our own slot: the runtime puts this node first, so this is the hot path -
       the devices behind us wait one reply slot and no more. */
    if (0U == uart_port_write_dma(&bp->port, bp->tx, len)) {
        bp->tx_lost++;
    }
}

/* A Fast Sync Read block whose turn has come.  Its slotting is done in dxl2.c -
   it counts the bytes of the blocks ahead of us, which is the only thing that
   works when a block is ~160 us and a 0x82 reply slot is ~295 us - so this is a
   plain send and the deferral above must not see it. */
static void bus_emit_now(bus_port_t *bp, uint8_t proto, uint16_t len)
{
    bus_note_proto(bp, proto, systick_get_ms());

    if (0U != bp->defer_pending) {
        bp->defer_pending = 0U;
        bus_arb_cancel(&bp->arb);
    }
    if (0U == uart_port_write_dma(&bp->port, bp->tx, len)) {
        bp->tx_lost++;
    }
}

/* ── receive (interrupt context) ────────────────────────────────────────── */

static void bus_on_byte(uint8_t index, uint8_t byte)
{
    bus_port_t *bp = &s_ports[index];
    uint32_t now_us = us_now();
    uint32_t now_ms = systick_get_ms();
    uint8_t allow_dxl;
    uint8_t allow_fee;
    uint16_t n;

    /* A frame whose bytes are further apart than BUS_GAP_US is not one frame:
       drop what we have so the next FF FF re-syncs cleanly instead of the tail
       of a truncated frame being glued to the next one.  Only a stall in the
       middle of a frame counts - the gap between two transactions is the whole
       point of a request/response bus. */
    if ((0U != bp->have_last) && (us_elapsed(bp->last_us, now_us) > BUS_GAP_US)) {
        if ((0U != bp->dxl.active) || (0U != bp->fee.active)) {
            bp->gaps++;
        }
        dxl_slave_reset(&bp->dxl);
        fee_slave_reset(&bp->fee);
    }
    bp->last_us = now_us;
    bp->have_last = 1U;

    bus_arb_rx(&bp->arb, byte);

    proto_allow(bp, dev_cfg()->proto_lock, now_ms, &allow_dxl, &allow_fee);

    if (0U != allow_dxl) {
        /* Fast Sync Read status bytes first: while this node owes a block, the
           bytes arriving are the blocks of the devices ahead of it, and they are
           what its own CRC has to cover.  The feeder answers from here, in the
           interrupt, on the byte that completes the block before ours - the same
           latency budget as every other reply. */
        n = dxl_slave_fast_rx(&bp->dxl, byte, bp->tx, (uint16_t)sizeof(bp->tx));
        if (0U != n) {
            bus_emit_now(bp, PROTO_DXL, n);
            return;
        }
        n = dxl_slave_feed(&bp->dxl, byte, now_ms, bp->tx, (uint16_t)sizeof(bp->tx));
        if (0U != n) {
            bus_emit(bp, PROTO_DXL, n, now_us);
            return;
        }
    }
    if (0U != allow_fee) {
        n = fee_slave_feed(&bp->fee, byte, now_ms, bp->tx, (uint16_t)sizeof(bp->tx));
        if (0U != n) {
            bus_emit(bp, PROTO_FEE, n, now_us);
        }
    }
}

/* ── init ───────────────────────────────────────────────────────────────── */

void bus_init(void)
{
    uint8_t k;

    memset(s_ports, 0, sizeof(s_ports));

    /* port 0: the debug console.  With the mirror off it is not fed to the
       protocol slaves at all (see the loop below), so nothing typed at it - and
       nothing a stray tool sends to it - can be taken for a bus frame. */
    s_ports[BUS_PORT_MIRROR].name = "uart0";
    s_ports[BUS_PORT_MIRROR].enabled = 1U;
    uart_port_init(&s_ports[BUS_PORT_MIRROR].port, DBG_USART, BUS_PORT_MIRROR,
                   (BUS_MIRROR_ENABLE ? dev_mirror_baud() : DBG_BAUD_DEFAULT),
                   0U);
    dxl_slave_init(&s_ports[BUS_PORT_MIRROR].dxl);
    fee_slave_init(&s_ports[BUS_PORT_MIRROR].fee);
    bus_arb_init(&s_ports[BUS_PORT_MIRROR].arb, BUS_SLOT_US);
    s_ports[BUS_PORT_MIRROR].last_proto = PROTO_NONE;

    /* port 1: the real motor bus, single wire, hears its own TX */
    s_ports[BUS_PORT_MAIN].name = "uart1";
    s_ports[BUS_PORT_MAIN].enabled = 1U;
    uart_port_init(&s_ports[BUS_PORT_MAIN].port, BUS_USART, BUS_PORT_MAIN,
                   dev_dxl_baud(), 1U);
    dxl_slave_init(&s_ports[BUS_PORT_MAIN].dxl);
    fee_slave_init(&s_ports[BUS_PORT_MAIN].fee);
    bus_arb_init(&s_ports[BUS_PORT_MAIN].arb, BUS_SLOT_US);
    s_ports[BUS_PORT_MAIN].last_proto = PROTO_NONE;

    /* With the mirror off, USART0 is a pure console: never answer protocol
       frames on it, so a terminal typing at the debug port cannot be mistaken
       for a bus. */
    for (k = 0U; k < BUS_PORT_COUNT; k++) {
        if (BUS_MIRROR_ENABLE || (BUS_PORT_MAIN == k)) {
            uart_port_set_on_byte(&s_ports[k].port, bus_on_byte);
        }
    }
}

void bus_apply_ports(void)
{
    uint32_t main_baud;

    if (BUS_MIRROR_ENABLE) {
        uart_port_set_baud(&s_ports[BUS_PORT_MIRROR].port, dev_mirror_baud());
    } else {
        uart_port_set_baud(&s_ports[BUS_PORT_MIRROR].port, DBG_BAUD_DEFAULT);
    }

    /* the served baud follows whichever protocol the host is speaking */
    main_baud = (PROTO_FEE == s_ports[BUS_PORT_MAIN].last_proto)
                    ? dev_fee_baud() : dev_dxl_baud();
    uart_port_set_baud(&s_ports[BUS_PORT_MAIN].port, main_baud);
}

/* ── main-loop housekeeping ─────────────────────────────────────────────── */

void bus_poll(uint32_t now_ms)
{
    uint8_t i;

    (void)now_ms;

    for (i = 0U; i < BUS_PORT_COUNT; i++) {
        bus_port_t *bp = &s_ports[i];

        if (0U == bp->defer_pending) {
            continue;
        }
        if (0U != bus_arb_step(&bp->arb, us_now())) {
            if (0U == uart_port_write_dma(&bp->port, bp->defer, bp->defer_len)) {
                bp->tx_lost++;
            }
            bp->defer_pending = 0U;
            bus_arb_cancel(&bp->arb);
        }
    }
}

/* ── accessors ──────────────────────────────────────────────────────────── */

void bus_get_stats(uint8_t index, bus_stats_t *out)
{
    bus_port_t *bp;

    if (index >= BUS_PORT_COUNT) {
        memset(out, 0, sizeof(*out));
        return;
    }
    bp = &s_ports[index];
    out->rx_bytes   = bp->port.rx_bytes;
    out->tx_bytes   = bp->port.tx_bytes;
    out->rx_dropped = bp->port.rx_dropped;
    out->rx_errors  = bp->port.rx_errors;
    out->echoed     = bp->port.echoed;
    out->frames     = bp->dxl.frames + bp->fee.frames;
    out->answered   = bp->dxl.answered + bp->fee.answered;
    out->responses  = bp->dxl.responses + bp->fee.responses;
    out->bad_crc    = bp->dxl.bad_crc;
    out->bad_len    = bp->dxl.bad_len + bp->fee.bad_len + bp->fee.bad_sum;
    out->ignored    = bp->dxl.ignored + bp->fee.ignored;
    out->deferred   = bp->deferred;
    out->tx_lost    = bp->tx_lost;
    out->gaps       = bp->gaps;
    out->last_proto = bp->last_proto;
    out->enabled    = bp->enabled;
}

uint8_t bus_last_proto(uint8_t index)
{
    return (index < BUS_PORT_COUNT) ? s_ports[index].last_proto : PROTO_NONE;
}

uint32_t bus_ms_since_rx(uint8_t index, uint32_t now_ms)
{
    if (index >= BUS_PORT_COUNT) {
        return 0xFFFFFFFFU;
    }
    return uart_port_ms_since_rx(&s_ports[index].port, now_ms);
}

uart_port_t *bus_uart(uint8_t index)
{
    return (index < BUS_PORT_COUNT) ? &s_ports[index].port : NULL;
}
