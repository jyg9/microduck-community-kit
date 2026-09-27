/*!
    \file    bus_arb.c
    \brief   see bus_arb.h
*/

#include "bus_arb.h"

#include "board.h"
#include "us_time.h"

void bus_arb_init(bus_arb_t *a, uint32_t slot_us)
{
    a->state = BUS_ARB_IDLE;
    a->slots_left = 0U;
    a->our_id = 0U;
    a->rx_state = 0U;
    a->slot_us = (0U != slot_us) ? slot_us : BUS_SLOT_US;
    a->deadline_us = 0U;
}

void bus_arb_set_id(bus_arb_t *a, uint8_t our_id)
{
    a->our_id = our_id;
}

void bus_arb_cancel(bus_arb_t *a)
{
    a->state = BUS_ARB_IDLE;
    a->slots_left = 0U;
    a->rx_state = 0U;
}

void bus_arb_start(bus_arb_t *a, uint32_t now_us, uint8_t slots)
{
    a->rx_state = 0U;
    if (0U == slots) {
        a->state = BUS_ARB_READY;
        a->slots_left = 0U;
        return;
    }
    a->state = BUS_ARB_WAIT;
    a->slots_left = slots;
    a->deadline_us = now_us + a->slot_us;
}

static int bus_arb_names_us(const bus_arb_t *a, uint8_t id)
{
    return ((id == a->our_id) || (id == FEE_BROADCAST_ID)) ? 1 : 0;
}

void bus_arb_rx(bus_arb_t *a, uint8_t byte)
{
    if (BUS_ARB_WAIT != a->state) {
        a->rx_state = 0U;
        return;
    }

    switch (a->rx_state) {
    case 0U:
        a->rx_state = (FEE_HEADER_0 == byte) ? 1U : 0U;
        break;
    case 1U:
        a->rx_state = (FEE_HEADER_0 == byte) ? 2U : 0U;
        break;
    case 2U:
        a->rx_state = 0U;
        if (DXL_HEADER_2 == byte) {
            a->rx_state = 3U;               /* Dynamixel 2.0: id comes later */
        } else if (bus_arb_names_us(a, byte)) {
            bus_arb_cancel(a);
        }
        break;
    case 3U:
        a->rx_state = (DXL_HEADER_3 == byte) ? 4U : 0U;
        break;
    default:
        a->rx_state = 0U;
        if (bus_arb_names_us(a, byte)) {
            bus_arb_cancel(a);
        }
        break;
    }
}

uint8_t bus_arb_step(bus_arb_t *a, uint32_t now_us)
{
    while ((BUS_ARB_WAIT == a->state) && us_due(a->deadline_us, now_us)) {
        if (a->slots_left > 0U) {
            a->slots_left--;
        }
        if (0U == a->slots_left) {
            a->state = BUS_ARB_READY;
        } else {
            a->deadline_us += a->slot_us;
        }
    }
    return (BUS_ARB_READY == a->state) ? 1U : 0U;
}
