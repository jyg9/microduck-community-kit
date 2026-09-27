/*!
    \file    fee.c
    \brief   FeeTech SCS/HLS protocol slave - see fee.h
*/

#include "fee.h"

#include <string.h>

#include "dbg.h"
#include "dev.h"

#define FEE_FRAME_TIMEOUT_MS    20U

/* Ack status byte.  A real servo's is 0 on success, and FeeTech tooling reads a
   nonzero value as a *servo* status (over-voltage, encoder, temperature,
   current, load) - which is why this node never reports its own protocol errors
   there.  The reason a read or write was refused is published in the vendor
   window's V_LAST_ERR instead, where a host can ask for it deliberately. */
#define FEE_ST_OK       0x00U

/*! \brief ACK status level (register 8).  Vendor table: 0 = "除读指令与 PING
   指令外其它指令不返回应答包", 1 = answer everything.  Initial value is 1. */
static uint8_t ack_level(void)
{
    return dev_image(PROTO_FEE)[8];
}

static uint8_t status_from_err(uint8_t err)
{
    if (0U != err) {
        dev_note_error(PROTO_FEE, err);
    }
    return FEE_ST_OK;
}

static uint16_t build_ack(uint8_t id, uint8_t status,
                          const uint8_t *data, uint8_t n,
                          uint8_t *out, uint16_t out_max)
{
    uint16_t total = (uint16_t)(n + 6U);
    uint8_t sum = 0U;
    uint16_t i;

    if (total > out_max) {
        return 0U;
    }
    out[0] = FEE_HEADER_0;
    out[1] = FEE_HEADER_1;
    out[2] = id;
    out[3] = (uint8_t)(n + 2U);
    out[4] = status;
    if (n != 0U) {
        memcpy(&out[5], data, n);
    }
    for (i = 2U; i < (uint16_t)(5U + n); i++) {
        sum = (uint8_t)(sum + out[i]);
    }
    out[5U + n] = (uint8_t)(~sum);
    return total;
}

static uint16_t handle_frame(fee_slave_t *s, uint32_t now_ms,
                             uint8_t *out, uint16_t out_max)
{
    uint8_t *f = s->buf;
    uint8_t  len = f[3];
    uint16_t total = (uint16_t)(len + 4U);
    uint8_t  id = f[2];
    uint8_t  inst = f[4];
    uint8_t  our_id = dev_cfg()->fee_id;
    uint8_t  sum = 0U;
    uint16_t i;
    uint8_t  data[REG_SPACE_SIZE];
    uint8_t  err;
    int      quiet;

    s->frames++;
    s->last_inst = inst;

    for (i = 2U; i < (uint16_t)(total - 1U); i++) {
        sum = (uint8_t)(sum + f[i]);
    }
    {
        uint8_t expect = (uint8_t)(~sum);
        if (expect != f[total - 1U]) {
            s->bad_sum++;
            return 0U;  /* SCS has no checksum-error response */
        }
    }

    if ((id != our_id) && (id != FEE_BROADCAST_ID)) {
        s->ignored++;
        return 0U;
    }

    /* With the ACK level at 0 only reads (and PING) are answered; everything
       else is still executed, it just stays quiet. */
    quiet = (0U == ack_level())
            && !(FEE_INST_PING == inst || FEE_INST_READ == inst || FEE_INST_SYNC_READ == inst);

    switch (inst) {
    case FEE_INST_PING:
        s->answered++;
        return build_ack(our_id, FEE_ST_OK, 0, 0U, out, out_max);

    case FEE_INST_READ: {
        uint8_t addr;
        uint8_t rlen;
        if (len < 4U) {
            s->bad_len++;
            return build_ack(our_id, status_from_err(DXL_ERR_LENGTH), 0, 0U, out, out_max);
        }
        addr = f[5];
        rlen = f[6];
        if (0U == rlen) {
            return build_ack(our_id, status_from_err(DXL_ERR_LENGTH), 0, 0U, out, out_max);
        }
        err = dev_read(PROTO_FEE, addr, rlen, data);
        s->answered++;
        if (0U != err) {
            return build_ack(our_id, status_from_err(err), 0, 0U, out, out_max);
        }
        return build_ack(our_id, FEE_ST_OK, data, rlen, out, out_max);
    }

    case FEE_INST_WRITE: {
        uint8_t addr;
        uint8_t wlen;
        if (len < 3U) {
            s->bad_len++;
            return build_ack(our_id, status_from_err(DXL_ERR_LENGTH), 0, 0U, out, out_max);
        }
        addr = f[5];
        wlen = (uint8_t)(len - 3U);
        if (0U == wlen) {
            s->bad_len++;
            return build_ack(our_id, status_from_err(DXL_ERR_LENGTH), 0, 0U, out, out_max);
        }
        err = dev_write(PROTO_FEE, addr, wlen, &f[6]);
        s->answered++;
        if (id == FEE_BROADCAST_ID || quiet) {
            return 0U;
        }
        return build_ack(our_id, status_from_err(err), 0, 0U, out, out_max);
    }

    case FEE_INST_REG_WRITE: {
        uint8_t addr;
        uint8_t wlen;
        if (len < 3U || (uint8_t)(len - 3U) > FEE_PEND_MAX) {
            s->bad_len++;
            return build_ack(our_id, status_from_err(DXL_ERR_LENGTH), 0, 0U, out, out_max);
        }
        addr = f[5];
        wlen = (uint8_t)(len - 3U);
        s->pend_addr = addr;
        s->pend_len = wlen;
        memcpy(s->pend_data, &f[6], wlen);
        s->pend_valid = 1U;
        s->answered++;
        if (id == FEE_BROADCAST_ID || quiet) {
            return 0U;
        }
        return build_ack(our_id, FEE_ST_OK, 0, 0U, out, out_max);
    }

    case FEE_INST_REG_ACTION:
        if (s->pend_valid) {
            (void)dev_write(PROTO_FEE, s->pend_addr, s->pend_len, s->pend_data);
            s->pend_valid = 0U;
        }
        s->answered++;
        if (id == FEE_BROADCAST_ID || quiet) {
            return 0U;
        }
        return build_ack(our_id, FEE_ST_OK, 0, 0U, out, out_max);

    case FEE_INST_RECOVERY:
        s->answered++;
        if (id == FEE_BROADCAST_ID || quiet) {
            return 0U;
        }
        return build_ack(our_id, FEE_ST_OK, 0, 0U, out, out_max);

    case FEE_INST_REBOOT:
        /* No answer, by protocol (measured on a real HD-1910 fw 3.46: silence,
           alive again after 823 ms, EEPROM settings preserved).  A servo is told
           to switch torque off first; this node has none.  The application and
           the bootloader share dev_request_reboot(), so 0x08 works in both. */
        dev_request_reboot(now_ms);
        s->answered++;
        return 0U;

    case FEE_INST_RESET:
        dev_factory_reset();
        s->answered++;
        if (id == FEE_BROADCAST_ID || quiet) {
            return 0U;
        }
        return build_ack(our_id, FEE_ST_OK, 0, 0U, out, out_max);

    case FEE_INST_CAL:
        /* middle calibration has no meaning for an IMU node: acknowledge and
           let the host clear its own homing offset */
        {
            uint8_t zero[2] = { 0U, 0U };
            (void)dev_write(PROTO_FEE, 31U, 2U, zero);
        }
        s->answered++;
        if (id == FEE_BROADCAST_ID || quiet) {
            return 0U;
        }
        return build_ack(our_id, FEE_ST_OK, 0, 0U, out, out_max);

    case FEE_INST_SYNC_READ: {
        uint8_t addr;
        uint8_t rlen;
        uint8_t nids;
        uint8_t k;
        if (len < 5U) {
            s->bad_len++;
            return 0U;
        }
        addr = f[5];
        rlen = f[6];
        nids = (uint8_t)(len - 4U);
        if (0U == rlen) {
            return 0U;
        }
        s->sync_named = 0U;
        for (k = 0U; k < nids; k++) {
            if (f[7U + k] == our_id) {
                err = dev_read(PROTO_FEE, addr, rlen, data);
                s->answered++;
                s->sync_named = 1U;
                s->sync_index = k;      /* reply after k reply slots - see bus.c */
                if (0U != err) {
                    return build_ack(our_id, status_from_err(err), 0, 0U, out, out_max);
                }
                return build_ack(our_id, FEE_ST_OK, data, rlen, out, out_max);
            }
        }
        s->ignored++;
        return 0U;
    }

    case FEE_INST_SYNC_WRITE: {
        uint8_t addr;
        uint8_t wlen;
        uint16_t off;
        if (len < 5U) {
            s->bad_len++;
            return 0U;
        }
        addr = f[5];
        wlen = f[6];
        if (0U == wlen) {
            return 0U;
        }
        off = 7U;
        while ((uint16_t)(off + 1U + wlen) <= (uint16_t)(total - 1U)) {
            if (f[off] == our_id) {
                (void)dev_write(PROTO_FEE, addr, wlen, &f[off + 1U]);
                s->answered++;
                return 0U;      /* sync write is never acknowledged */
            }
            off = (uint16_t)(off + 1U + wlen);
        }
        s->ignored++;
        return 0U;
    }

    default:
        s->ignored++;
        return 0U;
    }
}

void fee_slave_init(fee_slave_t *s)
{
    memset(s, 0, sizeof(*s));
}

void fee_slave_reset(fee_slave_t *s)
{
    /* Called when a frame stalls for longer than BUS_GAP_US.  Only the framing
       state is dropped; the statistics and a staged REG_WRITE survive, because
       those describe the bus, not the partial frame. */
    s->n = 0U;
    s->need = 0U;
    s->active = 0U;
    s->sync_named = 0U;
}

uint16_t fee_slave_feed(fee_slave_t *s, uint8_t byte, uint32_t now_ms,
                        uint8_t *out, uint16_t out_max)
{
    uint16_t resp = 0U;

    if (s->active && (uint32_t)(now_ms - s->last_ms) > FEE_FRAME_TIMEOUT_MS) {
        s->active = 0U;
        s->n = 0U;
        s->need = 0U;
    }
    s->last_ms = now_ms;

    switch (s->n) {
    case 0U:
        if (FEE_HEADER_0 == byte) {
            s->buf[0] = byte;
            s->n = 1U;
        }
        return 0U;
    case 1U:
        if (FEE_HEADER_1 == byte) {
            s->buf[1] = byte;
            s->n = 2U;
        } else {
            s->n = (FEE_HEADER_0 == byte) ? 1U : 0U;
        }
        return 0U;
    case 2U:
        /* third byte is the ID; 0xFD never is (it would collide with the
           Dynamixel 2.0 header) and is rejected outright */
        if (0xFD == byte) {
            s->n = 0U;
            return 0U;
        }
        s->buf[2] = byte;
        s->n = 3U;
        return 0U;
    case 3U:
        s->buf[3] = byte;
        s->n = 4U;
        if (byte < 2U) {
            s->bad_len++;
            s->n = 0U;
            s->need = 0U;
            return 0U;
        }
        s->need = (uint16_t)(byte + 4U);
        s->active = 1U;
        return 0U;
    default:
        break;
    }

    s->active = 1U;
    if (s->n >= FEE_MAX_FRAME) {
        s->n = 0U;
        s->need = 0U;
        return 0U;
    }
    s->buf[s->n++] = byte;

    if (s->need != 0U && s->n >= s->need) {
        resp = handle_frame(s, now_ms, out, out_max);
        if (0U != resp) {
            s->responses++;
        }
        s->n = 0U;
        s->need = 0U;
        s->active = 0U;
    }
    return resp;
}
