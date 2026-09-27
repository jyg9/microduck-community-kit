/*!
    \file    stuffing.c
    \brief   see stuffing.h
*/

#include "stuffing.h"

uint16_t stuffing_max_len(uint16_t n)
{
    return (uint16_t)(n + (n / 3U) + 2U);
}

uint16_t stuffing_add(const uint8_t *in, uint16_t n, uint8_t *out)
{
    uint16_t i;
    uint16_t written = 0U;
    uint8_t run = 0U;          /* length of the FF FF FD prefix matched so far */

    for (i = 0U; i < n; i++) {
        uint8_t b = in[i];
        out[written++] = b;
        switch (run) {
        case 0U:
            run = (0xFFU == b) ? 1U : 0U;
            break;
        case 1U:
            run = (0xFFU == b) ? 2U : 0U;
            break;
        case 2U:
            if (0xFFU == b) {
                run = 2U;              /* FF FF FF keeps an FF FF suffix */
            } else if (0xFDU == b) {
                out[written++] = 0xFDU; /* the stuffing byte */
                run = 0U;
            } else {
                run = 0U;
            }
            break;
        default:
            run = (0xFFU == b) ? 1U : 0U;
            break;
        }
    }
    return written;
}

uint16_t stuffing_remove(const uint8_t *in, uint16_t n, uint8_t *out)
{
    uint16_t i;
    uint16_t written = 0U;
    uint8_t run = 0U;

    for (i = 0U; i < n; i++) {
        uint8_t b = in[i];

        if (3U == run) {
            run = 0U;
            if (0xFDU == b) {
                continue;              /* inserted by the sender: drop it */
            }
        }

        switch (run) {
        case 0U:
            run = (0xFFU == b) ? 1U : 0U;
            break;
        case 1U:
            run = (0xFFU == b) ? 2U : 0U;
            break;
        case 2U:
            if (0xFFU == b) {
                run = 2U;
            } else if (0xFDU == b) {
                run = 3U;
            } else {
                run = 0U;
            }
            break;
        default:
            run = (0xFFU == b) ? 1U : 0U;
            break;
        }

        out[written++] = b;
    }
    return written;
}
