/*!
    \file    us_time.c
    \brief   see us_time.h
*/

#include "us_time.h"

#ifdef IMU_TO_DXL_HOST_TEST
/* The host protocol tests never drive the bus in real time; provide inert
   versions so the module can still be linked if a test list grows. */
void us_time_init(void) { }
void us_time_tick(void) { }
uint32_t us_now(void) { return 0U; }
uint32_t us_elapsed(uint32_t since, uint32_t now) { (void)since; return now; }
#else

#include "board.h"

/* One millisecond of core cycles, and the clamp that goes with it: if the
   SysTick handler is ever late, the sub-millisecond part must saturate rather
   than borrow a millisecond and report a time that has not happened yet. */
#define US_TIME_MAX_SUB_US  999U

static uint32_t s_cycles_per_us = 1U;

/* The millisecond part and the cycle count latched at that millisecond.  Both
   are written only by us_time_tick() (SysTick, the highest-priority exception)
   and read by us_now(), which masks interrupts for the pair so it cannot see a
   millisecond that does not belong to the latch it read. */
static volatile uint32_t s_ms;
static volatile uint32_t s_cycles_at_ms;

void us_time_init(void)
{
    /* TRCENA powers the debug block; CYCCNT starts counting core cycles from
       the moment it is enabled.  Both writes are ignored until TRCENA is set. */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    s_cycles_per_us = SystemCoreClock / 1000000U;
    if (0U == s_cycles_per_us) {
        s_cycles_per_us = 1U;
    }

    s_ms = 0U;
    s_cycles_at_ms = DWT->CYCCNT;
}

void us_time_tick(void)
{
    s_cycles_at_ms = DWT->CYCCNT;
    s_ms++;
}

uint32_t us_now(void)
{
    uint32_t primask = __get_PRIMASK();
    uint32_t ms;
    uint32_t base;
    uint32_t cycles;
    uint32_t sub;

    __disable_irq();
    ms = s_ms;
    base = s_cycles_at_ms;
    cycles = DWT->CYCCNT;
    if (0U == (primask & 0x1U)) {
        __enable_irq();
    }

    /* CYCCNT wraps every 35.8 s and so does this difference; the clamp covers
       the one case the subtraction cannot, a SysTick handler delayed past a
       whole millisecond. */
    sub = (cycles - base) / s_cycles_per_us;
    if (sub > US_TIME_MAX_SUB_US) {
        sub = US_TIME_MAX_SUB_US;
    }
    return (ms * 1000U) + sub;
}

uint32_t us_elapsed(uint32_t since, uint32_t now)
{
    return (uint32_t)(now - since);
}

#endif /* IMU_TO_DXL_HOST_TEST */
