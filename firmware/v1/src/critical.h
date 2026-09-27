/*!
    \file    critical.h
    \brief   shortest-possible interrupt masking, for publishing the register
             image while the USART interrupt may be reading it

    The receive path runs in the USART interrupt now (a frame's last byte has to
    become a reply within one reply slot, ~295 us measured), so `dev_read()` can
    preempt `dev_refresh()` half way through updating an image - which would
    hand a host a block with new gyro samples and an old counter, or a torn
    quaternion.  Publishing is a few dozen bytes of copying, so masking
    interrupts around it is cheaper than double buffering, and short enough not
    to cost a received byte at 1 Mbps (a byte is 10 us apart; this is ~1-2 us).

    Never use these around flash writes or ADC conversions.
*/

#ifndef CRITICAL_H
#define CRITICAL_H

#include "board.h"

static inline uint32_t irq_save(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static inline void irq_restore(uint32_t primask)
{
    if (0U == (primask & 0x1U)) {
        __enable_irq();
    }
}

#endif /* CRITICAL_H */
