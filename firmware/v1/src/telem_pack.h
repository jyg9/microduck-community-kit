/*!
    \file    telem_pack.h
    \brief   the wire forms of the sensor block, per protocol map

    The lump of bytes the runtime reads every tick is the *first 12* of the
    20-byte block, in both maps.  What follows is where the two maps differ, and
    it is not cosmetic:

      * FeeTech 56..70 is a real servo's block - position, speed, load, voltage,
        temperature, status, moving, current - and it is 15 bytes long.  The
        runtime reads exactly that span from the IMU node and every servo in one
        `sync_read`, so the node has to put its sample counter and status bits
        *inside* those 15 bytes (bytes 12 and 13) and leave byte 14 at zero, the
        way a real servo's block is shaped.  The raw accelerometer does not fit
        and lives at the 128 alias instead (measured: the servo answers a
        20-byte read at 56 too, but only 56..70 is its documented block, and a
        20-byte shared read costs ~5 bytes x 16 devices = ~0.8 ms of bus time).

      * Dynamixel 124 is `present_pwm` onwards, where the XL330 control table has
        room up to 143, so the full 20-byte block (accelerometer, counter, flags
        at 12/18/19) stays as it is.

    Keeping the packing here - pure, no register image, no hardware - is what
    lets the host tests assert the FeeTech layout byte by byte.
*/

#ifndef TELEM_PACK_H
#define TELEM_PACK_H

#include <stdint.h>

#include "board.h"

/*! \brief FeeTech 56..70: 12 control bytes, counter, status, reserved 0. */
void telem_pack_fee15(const uint8_t *telem, uint8_t *out);

#endif /* TELEM_PACK_H */
