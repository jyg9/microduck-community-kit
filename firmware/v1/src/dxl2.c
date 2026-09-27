/*!
    \file    dxl2.c
    \brief   Dynamixel protocol 2.0 slave - see dxl2.h
*/

#include "dxl2.h"

#include <string.h>

#include "crc16.h"
#include "stuffing.h"
#include "dbg.h"
#include "dev.h"

#define DXL_FRAME_TIMEOUT_MS    20U

static uint8_t status_return_level(void)
{
    return dev_image(PROTO_DXL)[68];
}

/* Status body (ERR + params) goes out byte-stuffed: see src/stuffing.h for why
   this is a correctness requirement rather than a nicety.  LEN counts the
   stuffed bytes, and the CRC covers them too. */
static uint16_t build_status(uint8_t id, uint8_t err,
                             const uint8_t *params, uint16_t n,
                             uint8_t *out, uint16_t out_max)
{
    uint8_t body[DXL_MAX_BODY];
    uint8_t stuffed[DXL_MAX_BODY + (DXL_MAX_BODY / 3U) + 4U];
    uint16_t body_len;
    uint16_t len;
    uint16_t total;
    uint16_t crc;

    /* A nonzero error byte is protocol-correct here (rustypot decodes it), and
       it is also worth remembering in the vendor window for a host that wants
       to ask why a write was refused. */
    if (0U != err) {
        dev_note_error(PROTO_DXL, err);
    }

    if ((uint32_t)n + 1U > sizeof(body)) {
        return 0U;
    }
    body[0] = err;
    if (n != 0U) {
        memcpy(&body[1], params, n);
    }
    body_len = (uint16_t)(n + 1U);
    body_len = stuffing_add(body, body_len, stuffed);

    /* LEN counts the 0x55 marker, the stuffed body (ERR + params) and the CRC */
    len = (uint16_t)(body_len + 3U);
    total = (uint16_t)(7U + len);
    if (total > out_max) {
        return 0U;
    }
    out[0] = DXL_HEADER_0;
    out[1] = DXL_HEADER_1;
    out[2] = DXL_HEADER_2;
    out[3] = DXL_HEADER_3;
    out[4] = id;
    out[5] = (uint8_t)(len & 0xFFU);
    out[6] = (uint8_t)(len >> 8);
    out[7] = DXL_STATUS_MARKER;
    memcpy(&out[8], stuffed, body_len);
    crc = crc16_dxl(out, (uint16_t)(total - 2U));
    out[total - 2U] = (uint8_t)(crc & 0xFFU);
    out[total - 1U] = (uint8_t)(crc >> 8);

    return total;
}

/* ── instructions ───────────────────────────────────────────────────────── */

static uint16_t handle_frame(dxl_slave_t *s, uint32_t now_ms,
                             uint8_t *out, uint16_t out_max)
{
    uint8_t *f = s->buf;
    uint16_t len = (uint16_t)((uint16_t)f[5] | ((uint16_t)f[6] << 8));
    uint16_t total = (uint16_t)(7U + len);
    uint8_t  id = f[4];
    uint8_t  inst = f[7];
    uint8_t  plain[DXL_MAX_BODY];
    const uint8_t *p;
    uint16_t nparams;
    uint8_t  our_id = dev_cfg()->dxl_id;
    uint16_t crc_read = (uint16_t)((uint16_t)f[total - 2U] | ((uint16_t)f[total - 1U] << 8));
    uint16_t crc_calc = crc16_dxl(f, (uint16_t)(total - 2U));
    uint8_t  data[REG_SPACE_SIZE];
    uint8_t  err;
    uint8_t  level;

    s->frames++;
    s->last_inst = inst;

    if (crc_read != crc_calc) {
        s->bad_crc++;
        DBG_TRACE("dxl crc err id=%u inst=0x%02X", id, inst);
        if (id == our_id) {
            return build_status(our_id, DXL_ERR_CRC, 0, 0U, out, out_max);
        }
        return 0U;
    }

    /* Everything after the instruction byte is byte-stuffed and the length
       field counts the stuffed form, so unpack it before looking at anything
       (CRC is already validated, which is what makes this safe). */
    if ((uint16_t)(len - 3U) > sizeof(plain)) {
        s->bad_len++;
        return 0U;
    }
    nparams = stuffing_remove(&f[8], (uint16_t)(len - 3U), plain);
    p = plain;

    /* A status packet from a servo on the shared bus looks exactly like an
       instruction packet to this parser; the ID filter is what keeps us from
       answering traffic that was never addressed to us. */
    if ((id != our_id) && (id != DXL_BROADCAST_ID)) {
        s->ignored++;
        return 0U;
    }

    level = status_return_level();

    switch (inst) {
    case DXL_INST_PING: {
        /* Model number and firmware version come from the register image rather
           than from compile-time constants: the bootloader links this same file
           and reports BOOT_MODEL_NUMBER / version 0 there, which is how a host
           recognises "the node is in its bootloader" with one PING. */
        const uint8_t *ident = dev_image(PROTO_DXL);
        uint8_t params[3];
        params[0] = ident[0];
        params[1] = ident[1];
        params[2] = ident[6];
        s->answered++;
        return build_status(our_id, 0U, params, 3U, out, out_max);
    }

    case DXL_INST_READ: {
        uint16_t addr;
        uint16_t rlen;
        if (nparams != 4U) {
            s->bad_len++;
            return build_status(our_id, DXL_ERR_LENGTH, 0, 0U, out, out_max);
        }
        addr = (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
        rlen = (uint16_t)((uint16_t)p[2] | ((uint16_t)p[3] << 8));
        if (rlen > REG_SPACE_SIZE) {
            return build_status(our_id, DXL_ERR_LENGTH, 0, 0U, out, out_max);
        }
        err = dev_read(PROTO_DXL, addr, rlen, data);
        s->answered++;
        if (0U != err) {
            if (0U == level) {
                return 0U;
            }
            return build_status(our_id, err, 0, 0U, out, out_max);
        }
        if (0U == level) {
            return 0U;
        }
        return build_status(our_id, 0U, data, rlen, out, out_max);
    }

    case DXL_INST_WRITE: {
        uint16_t addr;
        uint16_t wlen;
        if (nparams < 3U) {
            s->bad_len++;
            return build_status(our_id, DXL_ERR_LENGTH, 0, 0U, out, out_max);
        }
        addr = (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
        wlen = (uint16_t)(nparams - 2U);
        err = dev_write(PROTO_DXL, addr, wlen, &p[2]);
        s->answered++;
        if (id == DXL_BROADCAST_ID || level < 2U) {
            return 0U;   /* broadcast writes are never acknowledged */
        }
        return build_status(our_id, err, 0, 0U, out, out_max);
    }

    case DXL_INST_REG_WRITE: {
        uint16_t addr;
        uint16_t wlen;
        if (nparams < 3U || (nparams - 2U) > DXL_PEND_MAX) {
            s->bad_len++;
            return build_status(our_id, DXL_ERR_LENGTH, 0, 0U, out, out_max);
        }
        addr = (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
        wlen = (uint16_t)(nparams - 2U);
        s->pend_addr = addr;
        s->pend_len = (uint8_t)wlen;
        memcpy(s->pend_data, &p[2], wlen);
        s->pend_valid = 1U;
        s->answered++;
        if (id == DXL_BROADCAST_ID) {
            return 0U;
        }
        return build_status(our_id, 0U, 0, 0U, out, out_max);
    }

    case DXL_INST_ACTION: {
        if (s->pend_valid && id != DXL_BROADCAST_ID) {
            (void)dev_write(PROTO_DXL, s->pend_addr, s->pend_len, s->pend_data);
            s->pend_valid = 0U;
            s->answered++;
            return build_status(our_id, 0U, 0, 0U, out, out_max);
        }
        if (id != DXL_BROADCAST_ID) {
            s->answered++;
            return build_status(our_id, 0U, 0, 0U, out, out_max);
        }
        return 0U;
    }

    case DXL_INST_FACTORY_RESET:
        dev_factory_reset();
        s->answered++;
        if (id == DXL_BROADCAST_ID) {
            return 0U;
        }
        return build_status(our_id, 0U, 0, 0U, out, out_max);

    case DXL_INST_REBOOT:
        dev_request_reboot(now_ms);
        s->answered++;
        if (id == DXL_BROADCAST_ID) {
            return 0U;
        }
        return build_status(our_id, 0U, 0, 0U, out, out_max);

    case DXL_INST_CLEAR:
        dev_clear_hw_error();
        s->answered++;
        if (id == DXL_BROADCAST_ID || level < 2U) {
            return 0U;
        }
        return build_status(our_id, 0U, 0, 0U, out, out_max);

    case DXL_INST_SYNC_READ: {
        uint16_t addr;
        uint16_t rlen;
        uint16_t nids;
        uint16_t i;
        if (nparams < 4U) {
            s->bad_len++;
            return 0U;
        }
        addr = (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
        rlen = (uint16_t)((uint16_t)p[2] | ((uint16_t)p[3] << 8));
        nids = (uint16_t)(nparams - 4U);
        s->sync_named = 0U;
        for (i = 0U; i < nids; i++) {
            if (p[4U + i] == our_id) {
                if (rlen > REG_SPACE_SIZE || 0U == level) {
                    return 0U;
                }
                err = dev_read(PROTO_DXL, addr, rlen, data);
                s->answered++;
                s->sync_named = 1U;
                s->sync_index = (uint8_t)i;     /* reply after i slots - bus.c */
                if (0U != err) {
                    return build_status(our_id, err, 0, 0U, out, out_max);
                }
                return build_status(our_id, 0U, data, rlen, out, out_max);
            }
        }
        s->ignored++;
        return 0U;
    }

    case DXL_INST_SYNC_WRITE: {
        uint16_t addr;
        uint16_t wlen;
        uint16_t off;
        if (nparams < 4U) {
            s->bad_len++;
            return 0U;
        }
        addr = (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
        wlen = (uint16_t)((uint16_t)p[2] | ((uint16_t)p[3] << 8));
        if (0U == wlen) {
            return 0U;
        }
        off = 4U;
        while ((uint16_t)(off + 1U + wlen) <= nparams) {
            if (p[off] == our_id) {
                (void)dev_write(PROTO_DXL, addr, wlen, &p[off + 1U]);
                s->answered++;
                return 0U;      /* sync write is never acknowledged */
            }
            off = (uint16_t)(off + 1U + wlen);
        }
        s->ignored++;
        return 0U;
    }

    case DXL_INST_BULK_READ: {
        /* params: (addr, len) per id, then the id list: 6N + N bytes */
        uint16_t pairs;
        uint16_t i;
        uint16_t addr;
        uint16_t rlen;
        if (nparams < 7U || (nparams % 7U) != 0U) {
            s->bad_len++;
            return 0U;
        }
        pairs = (uint16_t)(nparams / 7U);
        for (i = 0U; i < pairs; i++) {
            if (p[6U * pairs + i] == our_id) {
                addr = (uint16_t)((uint16_t)p[6U * i] | ((uint16_t)p[6U * i + 1U] << 8));
                rlen = (uint16_t)((uint16_t)p[6U * i + 2U] | ((uint16_t)p[6U * i + 3U] << 8));
                if (rlen > REG_SPACE_SIZE || 0U == level) {
                    return 0U;
                }
                err = dev_read(PROTO_DXL, addr, rlen, data);
                s->answered++;
                if (0U != err) {
                    return build_status(our_id, err, 0, 0U, out, out_max);
                }
                return build_status(our_id, 0U, data, rlen, out, out_max);
            }
        }
        s->ignored++;
        return 0U;
    }

    default:
        s->ignored++;
        if (id == DXL_BROADCAST_ID) {
            return 0U;
        }
        return build_status(our_id, DXL_ERR_INSTRUCTION, 0, 0U, out, out_max);
    }
}

/* ── framing ────────────────────────────────────────────────────────────── */

void dxl_slave_init(dxl_slave_t *s)
{
    memset(s, 0, sizeof(*s));
}

void dxl_slave_reset(dxl_slave_t *s)
{
    /* Called when a frame stalls for longer than BUS_GAP_US: drop the framing
       state, keep the statistics and a staged REG_WRITE. */
    s->n = 0U;
    s->need = 0U;
    s->active = 0U;
    s->sync_named = 0U;
}

uint16_t dxl_slave_feed(dxl_slave_t *s, uint8_t byte, uint32_t now_ms,
                        uint8_t *out, uint16_t out_max)
{
    uint16_t resp = 0U;

    if (s->active && (uint32_t)(now_ms - s->last_ms) > DXL_FRAME_TIMEOUT_MS) {
        s->active = 0U;
        s->n = 0U;
        s->need = 0U;
    }
    s->last_ms = now_ms;

    switch (s->n) {
    case 0U:
        if (DXL_HEADER_0 == byte) {
            s->buf[0] = byte;
            s->n = 1U;
        }
        return 0U;
    case 1U:
        if (DXL_HEADER_1 == byte) {
            s->buf[1] = byte;
            s->n = 2U;
        } else {
            s->n = (DXL_HEADER_0 == byte) ? 1U : 0U;
        }
        return 0U;
    case 2U:
        if (DXL_HEADER_2 == byte) {
            s->buf[2] = byte;
            s->n = 3U;
        } else {
            s->n = (DXL_HEADER_0 == byte) ? 1U : 0U;
        }
        return 0U;
    case 3U:
        if (DXL_HEADER_3 == byte) {
            s->buf[3] = byte;
            s->n = 4U;
        } else {
            s->n = (DXL_HEADER_0 == byte) ? 1U : 0U;
        }
        return 0U;
    default:
        break;
    }

    s->active = 1U;
    if (s->n >= DXL_MAX_FRAME) {
        s->n = 0U;
        s->need = 0U;
        return 0U;
    }
    s->buf[s->n++] = byte;

    if (7U == s->n) {
        uint16_t len = (uint16_t)((uint16_t)s->buf[5] | ((uint16_t)s->buf[6] << 8));
        if (len < 3U || (uint16_t)(len + 7U) > DXL_MAX_FRAME) {
            s->bad_len++;
            s->n = 0U;
            s->need = 0U;
            s->active = 0U;
            return 0U;
        }
        s->need = (uint16_t)(len + 7U);
    }

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
