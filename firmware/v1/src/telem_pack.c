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

/* The FT6 forms.  Byte-for-byte parity with kissqy's ft_imu.c
   (compact_snapshot / snapshot) is the whole point, so resist tidying these:
   the offsets, the little-endian order and the schema byte are the contract. */

void telem_pack_fee15_ft6(const uint8_t *telem, uint8_t *out,
                          uint8_t quat_seq8, uint8_t status)
{
    memcpy(out, telem, TELEM_CTRL_LEN);
    out[FEE_BLOCK_CNT] = quat_seq8;
    out[FEE_BLOCK_STATUS] = status;
    out[FEE_BLOCK_RESERVED] = 0U;
}

void telem_pack_fee_diag20(const uint8_t *telem, uint8_t *out,
                           uint32_t quat_seq, uint16_t age_ms, uint8_t ready)
{
    memcpy(out, telem, TELEM_CTRL_LEN);
    out[12] = (uint8_t)(quat_seq);
    out[13] = (uint8_t)(quat_seq >> 8);
    out[14] = (uint8_t)(quat_seq >> 16);
    out[15] = (uint8_t)(quat_seq >> 24);
    out[16] = (uint8_t)(age_ms);
    out[17] = (uint8_t)(age_ms >> 8);
    out[18] = (0U != ready) ? 1U : 0U;
    out[19] = FEE_DIAG_SCHEMA;
}

uint8_t telem_fee_ft6_status(uint8_t ready, uint32_t age_ms,
                             uint32_t new_samples, uint8_t sensor_err)
{
    /* bit0: NOT_READY - the opposite polarity of this repository's own
       TELEM_FLAG_SFLP_VALID, which is why the two contracts cannot share a
       byte.  bit1 is only meaningful once the block is ready at all: a
       not-ready block is already refused, and kissqy gates it the same way. */
    uint8_t st = (0U != ready) ? 0U : (uint8_t)FEE_ST_NOT_READY;

    if ((0U != ready) && (age_ms > (uint32_t)FEE_ST_FRESH_MS)) {
        st |= (uint8_t)FEE_ST_STALE;
    }
    if (0U != sensor_err) {
        st |= (uint8_t)FEE_ST_ERR;
    }
    if (new_samples >= (uint32_t)FEE_ST_SLOW_SAMPLES) {
        st |= (uint8_t)FEE_ST_READER_SLOW;
    }
    return st;                    /* bits 4..7 are never set */
}
