/*!
    \file    telem_pack.c
    \brief   see telem_pack.h
*/

#include "telem_pack.h"

#include <string.h>

void telem_pack_fee15(const uint8_t *telem, uint8_t *out)
{
    memcpy(out, telem, TELEM_CTRL_LEN);
    out[FEE_BLOCK_CNT] = telem[TELEM_CNT];
    out[FEE_BLOCK_STATUS] = telem[TELEM_STATUS];
    out[FEE_BLOCK_RESERVED] = 0U;
}
