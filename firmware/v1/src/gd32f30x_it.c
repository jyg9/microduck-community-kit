/*!
    \file    gd32f30x_it.c
    \brief   interrupt handlers
*/

#include "gd32f30x_it.h"

#include "uart_port.h"
#include "us_time.h"

/*!
    \brief      this function handles SysTick exception

    Also re-latches the microsecond clock: us_now() is CYCCNT relative to the
    last millisecond tick, which is what keeps it monotonic across the 35.8 s
    cycle-counter wrap (see src/us_time.c).
*/
void SysTick_Handler(void)
{
    us_time_tick();
    delay_decrement();
}

/*!
    \brief      USART0 (debug console / bench bus mirror) receive
*/
void USART0_IRQHandler(void)
{
    uart_port_isr(USART0);
}

/*!
    \brief      USART1 (motor bus) receive
*/
void USART1_IRQHandler(void)
{
    uart_port_isr(USART1);
}

/*!
    \brief      DMA0 channel 3: USART0 transmit finished

    The reply itself is assembled in the USART interrupt and handed to DMA, so
    that the interrupt is not stuck spinning out 21 bytes (210 us at 1 Mbps)
    while the next byte is 10 us away.
*/
void DMA0_Channel3_IRQHandler(void)
{
    uart_port_tx_complete(USART0);
}

/*!
    \brief      DMA0 channel 6: USART1 (motor bus) transmit finished
*/
void DMA0_Channel6_IRQHandler(void)
{
    uart_port_tx_complete(USART1);
}
