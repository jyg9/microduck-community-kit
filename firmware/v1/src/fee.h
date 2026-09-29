/*!
    \file    fee.h
    \brief   FeeTech SCS/HLS protocol slave (the HD-1910-C001 family bus)

    Framing, taken from `FTServo_Linux-main/src/SCS.cpp` (`writeBuf`, `Ack`,
    `Read`, `syncReadPacketTx`, `syncWrite`) and `INST.h`:

      instruction: FF FF ID LEN INST [ADDR] [DATA...] ~SUM
                   LEN counts INST + ADDR + DATA + SUM
      status:      FF FF ID LEN STATUS [DATA...] ~SUM
                   LEN counts STATUS + DATA + SUM
      SUM          = ID + LEN + INST + ADDR + sum(DATA)   (ADDR = 0 when the
                     instruction carries none, which is how PING is summed)

    The node presents the HLS memory table (HLSCL.h) so it looks like a servo
    to FeeTech tooling, except that its status area (0x38..0x4B) carries the IMU
    block - see docs/imu_to_dxl_protocol.md §7 and docs/飞特通讯协议说明.md.
*/

#ifndef FEE_H
#define FEE_H

#include <stdint.h>

#include "board.h"

#define FEE_MAX_FRAME   260U
#define FEE_PEND_MAX    32U

typedef struct {
    uint8_t  buf[FEE_MAX_FRAME];
    uint16_t n;
    uint16_t need;
    uint32_t last_ms;
    uint8_t  active;
    uint8_t  pend_valid;
    uint8_t  pend_addr;
    uint8_t  pend_len;
    uint8_t  pend_data[FEE_PEND_MAX];
    /* Where the last sync_read put us in its id list.  bus.c uses this to hold
       the reply for its own slot instead of transmitting immediately (see
       src/bus_arb.h); sync_named is 0 when that sync_read did not name us. */
    uint8_t  sync_named;
    uint8_t  sync_index;
    /* statistics */
    uint32_t frames;
    uint32_t answered;
    uint32_t bad_sum;
    uint32_t bad_len;
    uint32_t ignored;
    uint32_t responses;
    uint8_t  last_inst;
} fee_slave_t;

void fee_slave_init(fee_slave_t *s);

/*! \brief drop a partial frame (long inter-byte gap) but keep the statistics. */
void fee_slave_reset(fee_slave_t *s);

uint16_t fee_slave_feed(fee_slave_t *s, uint8_t byte, uint32_t now_ms,
                        uint8_t *out, uint16_t out_max);

#endif /* FEE_H */
