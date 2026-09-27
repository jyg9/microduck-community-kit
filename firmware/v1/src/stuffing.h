/*!
    \file    stuffing.h
    \brief   Dynamixel protocol 2.0 byte stuffing

    Status (and instruction) packet bodies are transmitted with byte stuffing:
    whenever the pattern `FF FF FD` appears in the body, an extra `0xFD` is
    inserted so the payload can never be mistaken for a packet header on the
    wire.  The `LEN` field and the CRC cover the *stuffed* form, and the
    receiver removes the stuffing only after the CRC has been validated.

    This matters here for real interop, not for tidiness: microduck consumes the
    node through `rustypot` 1.6.0, whose `StatusPacketV2::from_bytes` calls
    `remove_stuffing` on `data[8..len-2]` (error byte + parameters).  A 12-byte
    IMU block containing e.g. `gyro_x = -1` (`FF FF`) followed by a `0xFD` byte
    is entirely reachable, so a node that does not stuff would be misparsed.

    The algorithm mirrors the official DynamixelSDK `addStuffing` /
    `removeStuffing` (and rustypot's port of it): the pattern scan runs over the
    original data only, so an inserted `0xFD` never seeds a new match, and
    `FF FF FF FD` still gets a stuffed byte because the `FF FF` suffix stays
    alive across extra `FF`s.
*/

#ifndef STUFFING_H
#define STUFFING_H

#include <stdint.h>

/*! \brief stuffed length of `n` bytes, worst case (`n / 3 * 4` rounded up). */
uint16_t stuffing_max_len(uint16_t n);

/*!
    \brief  insert stuffing bytes
    \param[in]  in: body bytes
    \param[in]  n: length of `in`
    \param[out] out: destination, must hold stuffing_max_len(n) bytes
    \retval     length written
*/
uint16_t stuffing_add(const uint8_t *in, uint16_t n, uint8_t *out);

/*!
    \brief  remove stuffing bytes
    \param[in]  in: body bytes as received
    \param[in]  n: length of `in`
    \param[out] out: destination, `n` bytes is always enough
    \retval     length written
*/
uint16_t stuffing_remove(const uint8_t *in, uint16_t n, uint8_t *out);

#endif /* STUFFING_H */
