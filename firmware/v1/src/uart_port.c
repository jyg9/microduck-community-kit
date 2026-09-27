/*!
    \file    uart_port.c
    \brief   USART driver: PB6/PB7 (USART0, remapped) and PA2/PA3 (USART1)

    RX is interrupt driven and the parsing happens there too: at 1 Mbps a byte
    arrives every 10 us, and a frame's last byte has to become a reply within
    one reply slot (~295 us measured), which the main loop cannot promise.

    TX is DMA: USART0_TX is DMA0 channel 3 and USART1_TX is DMA0 channel 6
    (GD32F30x user manual figure 10-4).  The alternative - spinning on TBE in
    the interrupt - would block for the whole 21-byte frame (210 us).
*/

#include "uart_port.h"

#include <stddef.h>

#include "board.h"
#include "systick.h"
#include "us_time.h"

#define UART_PORTS  2U

static uart_port_t *s_ports[UART_PORTS];

/*! \brief the echo window in microseconds for one byte time at this baud. */
static uint32_t echo_window_us(uint32_t baud)
{
    uint32_t byte_us;

    if (0U == baud) {
        baud = BUS_BAUD_DEFAULT;
    }
    byte_us = 10000000U / baud;          /* 10 bits per byte, rounded down */
    if (0U == byte_us) {
        byte_us = 1U;
    }
    return byte_us * UART_ECHO_WINDOW_BYTES;
}

static uart_port_t *port_of(uint32_t periph)
{
    return (USART0 == periph) ? s_ports[0] : s_ports[1];
}

static dma_channel_enum tx_channel_of(uint32_t periph)
{
    return (USART0 == periph) ? DMA_CH3 : DMA_CH6;
}

static IRQn_Type tx_irqn_of(uint32_t periph)
{
    return (USART0 == periph) ? DMA0_Channel3_IRQn : DMA0_Channel6_IRQn;
}

/* ── init ───────────────────────────────────────────────────────────────── */

static void port_pins(uint32_t periph)
{
    if (USART0 == periph) {
        rcu_periph_clock_enable(RCU_GPIOB);
        rcu_periph_clock_enable(RCU_AF);
        rcu_periph_clock_enable(RCU_USART0);
        gpio_pin_remap_config(GPIO_USART0_REMAP, ENABLE);
        gpio_init(DBG_GPIO, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, DBG_TX_PIN);
        gpio_init(DBG_GPIO, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, DBG_RX_PIN);
    } else {
        rcu_periph_clock_enable(RCU_GPIOA);
        rcu_periph_clock_enable(RCU_USART1);
        /* PA2/PA3 is the USART1 default mapping - no remap */
        gpio_init(BUS_GPIO, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, BUS_TX_PIN);
        gpio_init(BUS_GPIO, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, BUS_RX_PIN);
    }
}

void uart_port_init(uart_port_t *p, uint32_t periph, uint8_t index,
                    uint32_t baud, uint8_t echo)
{
    p->periph = periph;
    p->baud = baud;
    p->echo = echo;
    p->echo_until_us = 0U;
    p->tx_busy = 0U;
    p->tx_len = 0U;
    p->on_byte = NULL;
    p->rx_bytes = 0U;
    p->tx_bytes = 0U;
    p->rx_dropped = 0U;
    p->rx_errors = 0U;
    p->echoed = 0U;
    p->last_rx_ms = 0U;
    p->index = index;

    port_pins(periph);
    rcu_periph_clock_enable(RCU_DMA0);

    usart_deinit(periph);
    usart_baudrate_set(periph, baud);
    usart_word_length_set(periph, USART_WL_8BIT);
    usart_stop_bit_set(periph, USART_STB_1BIT);
    usart_parity_config(periph, USART_PM_NONE);
    usart_hardware_flow_rts_config(periph, USART_RTS_DISABLE);
    usart_hardware_flow_cts_config(periph, USART_CTS_DISABLE);
    usart_receive_config(periph, USART_RECEIVE_ENABLE);
    usart_transmit_config(periph, USART_TRANSMIT_ENABLE);
    usart_enable(periph);

    if (USART0 == periph) {
        s_ports[0] = p;
        nvic_irq_enable(USART0_IRQn, 1U, 0U);
    } else {
        s_ports[1] = p;
        nvic_irq_enable(USART1_IRQn, 1U, 0U);
    }
    usart_interrupt_enable(periph, USART_INT_RBNE);
    nvic_irq_enable(tx_irqn_of(periph), 1U, 0U);
}

void uart_port_set_on_byte(uart_port_t *p, uart_rx_fn fn)
{
    p->on_byte = fn;
}

void uart_port_set_baud(uart_port_t *p, uint32_t baud)
{
    if ((baud == p->baud) || (0U != p->tx_busy)) {
        return;
    }
    /* Only safe with the line idle; the caller re-applies port settings from
       the main loop, never from inside the ISR or mid-frame. */
    usart_disable(p->periph);
    usart_baudrate_set(p->periph, baud);
    usart_enable(p->periph);
    p->baud = baud;
    p->echo_until_us = 0U;
}

uint32_t uart_port_get_baud(const uart_port_t *p)
{
    return p->baud;
}

/* ── rx ─────────────────────────────────────────────────────────────────── */

void uart_port_isr(uint32_t periph)
{
    uart_port_t *p = port_of(periph);
    uint32_t guard = 0U;

    if (NULL == p) {
        (void)usart_data_receive(periph);
        return;
    }

    while ((RESET != usart_flag_get(periph, USART_FLAG_RBNE)) && (guard++ < 64U)) {
        uint8_t b = (uint8_t)usart_data_receive(periph);

        p->rx_bytes++;
        p->last_rx_ms = systick_get_ms();

        if (0U != p->echo) {
            /* Our own reply may be on the wire: either it is still being sent
               (tx_busy), or it has just finished and the receiver is still
               seeing the tail of the last byte through the direction buffer.
               Both are bounded in *time*: a fixed byte count is not, because
               whether the reply comes back at all depends on the transceiver -
               see UART_ECHO_WINDOW_BYTES. */
            if ((0U != p->tx_busy)
                || ((int32_t)(p->echo_until_us - us_now()) > 0)) {
                p->echoed++;
                continue;
            }
        }

        if (NULL != p->on_byte) {
            p->on_byte(p->index, b);
        } else {
            p->rx_dropped++;
        }
    }

    if (RESET != usart_flag_get(periph, USART_FLAG_ORERR)) {
        usart_flag_clear(periph, USART_FLAG_ORERR);
        (void)usart_data_receive(periph);
        p->rx_errors++;
    }
}

uint32_t uart_port_ms_since_rx(const uart_port_t *p, uint32_t now_ms)
{
    return (uint32_t)(now_ms - p->last_rx_ms);
}

/* ── tx ─────────────────────────────────────────────────────────────────── */

int uart_port_write_dma(uart_port_t *p, const uint8_t *data, uint16_t len)
{
    dma_parameter_struct cfg;
    dma_channel_enum ch = tx_channel_of(p->periph);
    uint32_t guard = 0U;

    if ((0U == len) || (0U != p->tx_busy)) {
        return 0;
    }

    /* The previous reply's last byte may still be shifting out; starting a new
       DMA before TC would cut it off.  One byte time at 1 Mbps, and only ever
       after the host has already come back with a new instruction. */
    while ((RESET == usart_flag_get(p->periph, USART_FLAG_TC)) && (guard++ < 20000U)) {
    }

    p->tx_busy = 1U;
    p->tx_len = len;

    dma_channel_disable(DMA0, ch);
    dma_interrupt_flag_clear(DMA0, ch, DMA_INT_FLAG_FTF);
    dma_deinit(DMA0, ch);
    cfg.direction = DMA_MEMORY_TO_PERIPHERAL;
    cfg.memory_addr = (uint32_t)data;
    cfg.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
    cfg.memory_width = DMA_MEMORY_WIDTH_8BIT;
    cfg.number = len;
    cfg.periph_addr = (uint32_t)(&USART_DATA(p->periph));
    cfg.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
    cfg.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
    cfg.priority = DMA_PRIORITY_ULTRA_HIGH;
    dma_init(DMA0, ch, &cfg);
    dma_circulation_disable(DMA0, ch);
    dma_memory_to_memory_disable(DMA0, ch);
    dma_interrupt_enable(DMA0, ch, DMA_INT_FTF);
    usart_dma_transmit_config(p->periph, USART_TRANSMIT_DMA_ENABLE);
    dma_channel_enable(DMA0, ch);
    return 1;
}

void uart_port_tx_complete(uint32_t periph)
{
    uart_port_t *p = port_of(periph);
    dma_channel_enum ch = tx_channel_of(periph);

    if (NULL == p) {
        return;
    }
    if (SET == dma_interrupt_flag_get(DMA0, ch, DMA_INT_FLAG_FTF)) {
        dma_interrupt_flag_clear(DMA0, ch, DMA_INT_FLAG_FTF);
    }
    dma_channel_disable(DMA0, ch);
    usart_dma_transmit_config(periph, USART_TRANSMIT_DMA_DISABLE);

    p->tx_bytes += p->tx_len;
    p->tx_busy = 0U;
    if (0U != p->echo) {
        p->echo_until_us = us_now() + echo_window_us(p->baud);
    }
}

uint8_t uart_port_tx_busy(const uart_port_t *p)
{
    return p->tx_busy;
}

void uart_port_write(uart_port_t *p, const uint8_t *data, uint16_t len)
{
    uint16_t i;

    if (0U == len) {
        return;
    }

    for (i = 0U; i < len; i++) {
        /* A protocol reply always wins: on the bench the debug console and the
           bus share USART0, and a host that starts talking in the middle of a
           log line must get its answer, not a truncated frame interleaved with
           text.  The reply's own DMA waits for TC first, so the byte in flight
           finishes cleanly and this loop simply stops. */
        if (0U != p->tx_busy) {
            return;
        }
        while (RESET == usart_flag_get(p->periph, USART_FLAG_TBE)) {
        }
        usart_data_transmit(p->periph, (uint16_t)data[i]);
    }
    while (RESET == usart_flag_get(p->periph, USART_FLAG_TC)) {
    }

    p->tx_bytes += len;
}
