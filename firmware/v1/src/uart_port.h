/*!
    \file    uart_port.h
    \brief   one USART, with the receive path in the interrupt and the reply
             path on DMA

    Both are forced by the bus contract, not by taste (see
    docs/bus_timing_borrow_plan.md): a `sync_read` burst leaves this node one
    reply slot - ~295 us measured - to get its first byte on the wire, and a
    frame's last byte has to turn into a reply without waiting for the main
    loop.  Spinning out a 21-byte reply (210 us at 1 Mbps) inside the interrupt
    would starve every other interrupt, so the reply is handed to DMA.

    The main loop no longer reads received bytes at all; it only advances the
    deferred-reply timer (src/bus_arb.c) and refreshes the register image.
*/

#ifndef UART_PORT_H
#define UART_PORT_H

#include <stdint.h>

/* How long after our own transmission is reported done a received byte is
   treated as the tail of our own frame rather than as traffic, in byte times.

   DMA reports "finished" while the last byte is still shifting out, and the
   auto-direction buffer has an RC tail, so *something* may come back.  Three
   byte times (~30 us at 1 Mbps) sit well inside the ~85 us the next device
   leaves before it starts talking, and far below the milliseconds between a
   host's instructions - so this window can never swallow the start of a frame
   addressed to us, which a fixed *byte count* could (and did: on this PCBA the
   buffer does not loop our reply back at all, so a 2-byte skip ate the next
   request's `FF FF` and halved the answer rate - see
   docs/bus_timing_borrow_plan.md §4.6). */
#define UART_ECHO_WINDOW_BYTES  3U

typedef struct uart_port uart_port_t;

/*! \brief one received byte, in interrupt context. */
typedef void (*uart_rx_fn)(uint8_t index, uint8_t byte);

struct uart_port {
    uint32_t    periph;
    uint32_t    baud;
    /* single-wire buses hear their own transmission; those bytes are dropped
       here rather than being mistaken for traffic */
    uint8_t     echo;
    uint8_t     index;
    /* set by bus_init(); NULL means "nobody wants received bytes" (a pure
       debug console) */
    uart_rx_fn  on_byte;
    /* a DMA reply is in flight */
    uint8_t     tx_busy;
    /* us timestamp (us_now()) until which received bytes are our own echo */
    uint32_t    echo_until_us;
    uint16_t    tx_len;
    /* statistics */
    uint32_t    rx_bytes;
    uint32_t    tx_bytes;
    uint32_t    rx_dropped;
    uint32_t    rx_errors;
    uint32_t    echoed;
    uint32_t    last_rx_ms;
};

void uart_port_init(uart_port_t *p, uint32_t periph, uint8_t index,
                    uint32_t baud, uint8_t echo);
void uart_port_set_on_byte(uart_port_t *p, uart_rx_fn fn);
void uart_port_set_baud(uart_port_t *p, uint32_t baud);
uint32_t uart_port_get_baud(const uart_port_t *p);

/*! \brief hand a reply to the DMA channel that feeds this USART.
    \retval 1 queued, 0 refused because the previous transfer is still running
            (the caller counts that as a lost reply - impossible on a
            request/response bus, where the host waits for our answer) */
int uart_port_write_dma(uart_port_t *p, const uint8_t *data, uint16_t len);

/*! \brief blocking write, for the debug console only: refused while a DMA reply
           is in flight so a log line cannot corrupt a protocol frame. */
void uart_port_write(uart_port_t *p, const uint8_t *data, uint16_t len);

/*! \brief DMA full-transfer-finish body; the per-channel handlers call it.
           USART0 TX is DMA0 channel 3, USART1 TX is DMA0 channel 6
           (GD32F30x user manual, figure 10-4). */
void uart_port_tx_complete(uint32_t periph);

uint8_t uart_port_tx_busy(const uart_port_t *p);

uint32_t uart_port_ms_since_rx(const uart_port_t *p, uint32_t now_ms);

/*! \brief shared RX interrupt body; the per-peripheral handlers call this. */
void uart_port_isr(uint32_t periph);

#endif /* UART_PORT_H */
