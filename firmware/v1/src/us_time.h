/*!
    \file    us_time.h
    \brief   microsecond monotonic clock, for the parts of the bus contract that
             the 1 kHz SysTick cannot see

    Two of those exist (see docs/bus_timing_borrow_plan.md):

      * the reply slot: a real servo waits one slot (~295 us measured on an
        HD-1910) for every id that precedes it in a `sync_read` list, present or
        not, so answering in our own position is a sub-millisecond decision;
      * the inter-byte gap: a frame that stalls for more than ~500 us between
        bytes is not one frame and must be dropped before the next one starts.

    The Cortex-M4's DWT cycle counter gives both.  At 120 MHz CYCCNT wraps every
    ~35.8 s, which is why the microsecond clock is *latched to the millisecond
    tick* instead of being a bare `CYCCNT / 120`:

      * a bare division wraps at 35.8e6, not at 2^32, so `deadline - now` stops
        being a wrap-safe signed comparison once every 35.8 s;
      * a deadline that is suddenly ~35 s in the future is not a small error -
        it is how the echo window came to discard every received byte and leave
        the node silent on a live bus (measured: `echo` tracking `rx` 1:1 while
        `frames`/`answered` froze, ~48% of bytes discarded over a run, and
        `loop_max` reporting 4259175904 us).

    us_time_tick() is what fixes that, and this is its only reason to exist.
*/

#ifndef US_TIME_H
#define US_TIME_H

#include <stdint.h>

/*! \brief start the cycle counter and the millisecond phase.  Idempotent; safe
           to call from init. */
void us_time_init(void);

/*! \brief re-latch the cycle counter and advance the millisecond part.  Called
           from the 1 kHz SysTick handler; this is what makes us_now() monotonic
           across the 35.8 s cycle wrap. */
void us_time_tick(void);

/*! \brief free-running microseconds, wrapping at 2^32 (~71.6 min) so a signed
           difference stays meaningful across the wrap. */
uint32_t us_now(void);

/*! \brief wrap-safe `now - since`. */
uint32_t us_elapsed(uint32_t since, uint32_t now);

/*! \brief 1 when `now` has reached `deadline` (both from us_now()). */
static inline int us_due(uint32_t deadline, uint32_t now)
{
    return ((int32_t)(now - deadline) >= 0) ? 1 : 0;
}

#endif /* US_TIME_H */
