/*!
    \file    dbg.c
    \brief   see dbg.h
*/

#include "dbg.h"

#if DBG_ENABLE
#include <stdarg.h>
#include <stdio.h>
#endif

#include "board.h"
#include "dev.h"
#include "systick.h"

static uart_port_t *s_port;

void dbg_bind(uart_port_t *port)
{
    s_port = port;
}

void dbg_init(void)
{
    /* the port pointer is bound by the bus before use; nothing else to do */
}

void dbg_log(uint8_t level, const char *fmt, ...)
{
#if DBG_ENABLE
    char buf[160];
    va_list ap;
    int n;

    if (NULL == s_port) {
        return;
    }
    if (level > dev_cfg()->dbg_level) {
        return;
    }
    /* keep text out of a live exchange: a request was seen very recently.
       (The receive path itself runs in the interrupt now, so "no byte for the
       last 20 ms" is the whole guard; there is no ring to inspect.) */
    if (uart_port_ms_since_rx(s_port, systick_get_ms()) < 20U) {
        return;
    }

    va_start(ap, fmt);
    n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n <= 0) {
        return;
    }
    if (n > (int)sizeof(buf) - 1) {
        n = (int)sizeof(buf) - 1;
    }
    uart_port_write(s_port, (const uint8_t *)"# ", 2U);
    uart_port_write(s_port, (const uint8_t *)buf, (uint16_t)n);
    uart_port_write(s_port, (const uint8_t *)"\r\n", 2U);
#else
    (void)level;
    (void)fmt;
#endif
}

void dbg_hex(uint8_t level, const char *tag, const uint8_t *data, uint16_t len)
{
#if DBG_ENABLE
    static const char hex[] = "0123456789ABCDEF";
    char line[128];
    uint16_t i;
    uint16_t n = 0U;

    if (NULL == s_port || level > dev_cfg()->dbg_level) {
        return;
    }
    for (i = 0U; i < len && n < (uint16_t)(sizeof(line) - 4U); i++) {
        line[n++] = hex[(data[i] >> 4) & 0x0FU];
        line[n++] = hex[data[i] & 0x0FU];
        line[n++] = ' ';
    }
    line[n] = '\0';
    dbg_log(level, "%s (%u) %s", tag, (unsigned)len, line);
#else
    (void)level;
    (void)tag;
    (void)data;
    (void)len;
#endif
}
