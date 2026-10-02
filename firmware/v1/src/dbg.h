/*!
    \file    dbg.h
    \brief   debug console on the mirror port, macro-switchable

    The console shares USART0 with the bench bus mirror, so:
      * every line is prefixed with '#' - the host tool skips lines that do not
        start with 0xFF, and ASCII text cannot contain 0xFF, so frame parsing
        stays unambiguous;
      * a line is dropped when a frame was seen in the last 20 ms, which keeps
        text out of the middle of a request/response exchange;
      * the level is a runtime register (vendor window V_DBG_LEVEL), and
        DBG_ENABLE=0 removes the whole thing at compile time.
*/

#ifndef DBG_H
#define DBG_H

#include <stdint.h>

#include "uart_port.h"

#define DBG_LVL_ERROR   1U      /* failures only                          */
#define DBG_LVL_INFO    2U      /* boot banner, periodic health (default)  */
#define DBG_LVL_TRACE   3U      /* every accepted frame                    */

/*! \brief attach the port the console writes to (the mirror port). */
void dbg_bind(uart_port_t *port);

/*! \brief enable the console; called once the ports exist. */
void dbg_init(void);

void dbg_log(uint8_t level, const char *fmt, ...);
void dbg_hex(uint8_t level, const char *tag, const uint8_t *data, uint16_t len);

#if DBG_ENABLE
#define DBG_ERROR(...)  dbg_log(DBG_LVL_ERROR, __VA_ARGS__)
#define DBG_INFO(...)   dbg_log(DBG_LVL_INFO, __VA_ARGS__)
#define DBG_TRACE(...)  dbg_log(DBG_LVL_TRACE, __VA_ARGS__)
#else
#define DBG_ERROR(...)  ((void)0)
#define DBG_INFO(...)   ((void)0)
#define DBG_TRACE(...)  ((void)0)
/* No `dbg_hex` macro here. It is a real function with its own `#if DBG_ENABLE`
   body in dbg.c, so a function-like macro of the same name rewrites that very
   definition and `DBG_ENABLE=0` does not compile:
       src/dbg.c:67: error: expected identifier or '(' before 'void'
   The body guard already removes the code, and there are no call sites — the
   three variadic macros above are the interface. */
#endif

#endif /* DBG_H */
