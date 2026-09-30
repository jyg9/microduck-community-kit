/*!
    \file    dxl2.h
    \brief   Dynamixel protocol 2.0 slave

    Byte-exact against the consumer that matters, rustypot 0.6 (`protocol/v2.rs`),
    which microduck uses through `Xl330Controller`:

      instruction: FF FF FD 00 ID LEN_L LEN_H INST PARAM... CRC_L CRC_H
                   LEN = len(PARAM) + 3
      status:      FF FF FD 00 ID LEN_L LEN_H 0x55 ERR PARAM... CRC_L CRC_H
                   LEN = len(PARAM) + 4

    The CRC is the ROBOTIS "update_crc" flavour (poly 0x8005, init 0, MSB first)
    computed over the whole packet including the header, up to - but not
    including - the CRC bytes.  Get any of that wrong and `sync_read` fails with
    a checksum error, so tests/ has host-side vectors for each packet shape.
*/

#ifndef DXL2_H
#define DXL2_H

#include <stdint.h>

#include "board.h"

/* A status packet's body is byte-stuffed, so the wire form can be up to about
   4/3 of the register read: 512 covers a full 256-byte register-space read with
   stuffing, the 7-byte prefix and the CRC. */
#define DXL_MAX_FRAME   512U
#define DXL_MAX_RESP    512U
/* largest de-stuffed body (error byte + parameters) we will handle */
#define DXL_MAX_BODY    264U
#define DXL_PEND_MAX    32U

typedef struct {
    uint8_t  buf[DXL_MAX_FRAME];
    uint16_t n;
    uint16_t need;
    uint32_t last_ms;
    uint8_t  active;
    /* staged REG_WRITE contents, applied by ACTION */
    uint8_t  pend_valid;
    uint16_t pend_addr;
    uint8_t  pend_len;
    uint8_t  pend_data[DXL_PEND_MAX];
    /* Where the last sync_read put us in its id list; see src/bus_arb.h. */
    uint8_t  sync_named;
    uint8_t  sync_index;
    /* Fast Sync Read (0x8A): one status packet is built by every addressed
       device in turn, and the CRC of a block covers everything on the wire up to
       that block.  A node that is not first therefore has to hear the blocks
       ahead of it before it can answer at all, which is what this state carries -
       see the "fast sync read" note in src/dxl2.c.  The node that IS first
       answers from the instruction frame and never touches any of it. */
    uint8_t  fast_active;       /* a block of ours is owed                    */
    uint8_t  fast_index;        /* our position in the instruction id list    */
    uint8_t  fast_rlen;         /* how many bytes it asked for                */
    uint8_t  fast_err;          /* error byte our block reports               */
    uint8_t  fast_data[REG_SPACE_SIZE];     /* payload, read on arrival       */
    uint16_t fast_total;        /* LEN of the aggregate packet                */
    uint16_t fast_seen;         /* bytes of it heard so far                   */
    uint16_t fast_need;         /* bytes to hear before our block goes out    */
    uint16_t fast_crc;          /* running CRC over what we have heard        */
    /* statistics */
    uint32_t frames;
    uint32_t answered;
    uint32_t bad_crc;
    uint32_t bad_len;
    uint32_t ignored;
    uint32_t responses;
    uint8_t  last_inst;
} dxl_slave_t;

void dxl_slave_init(dxl_slave_t *s);

/*! \brief drop a partial frame (long inter-byte gap) but keep the statistics. */
void dxl_slave_reset(dxl_slave_t *s);

/*! \brief consume one received byte.
    \retval length of the response to transmit, 0 when there is nothing to send */
uint16_t dxl_slave_feed(dxl_slave_t *s, uint8_t byte, uint32_t now_ms,
                        uint8_t *out, uint16_t out_max);

/*! \brief consume one byte of a Fast Sync Read status packet while a block of
           ours is pending, in interrupt context.
    \retval length of our block once the devices ahead of us are done, 0 while
            there is nothing to send yet (the common case) */
uint16_t dxl_slave_fast_rx(dxl_slave_t *s, uint8_t byte,
                           uint8_t *out, uint16_t out_max);

#endif /* DXL2_H */
