/*!
    \file    app_header.h
    \brief   the 64-byte `app_header_t` that sits at slot_base + 0x200

    Every slot image carries one.  The bootloader uses it to decide whether a
    slot is trustworthy before it jumps into it, the application uses it to
    learn its own version and whether it is still on trial, and the host
    packaging tool (`host/package.py`) writes it.

    Layout (docs/flash_layout.md §3), 64 bytes, 4-byte aligned:

      off  size  field
      0    4     magic           APP_HDR_MAGIC
      4    2     header_version  APP_HDR_VERSION
      6    2     image_version   (major<<12)|(minor<<8)|patch
      8    4     image_size      valid bytes from slot base
      12   4     crc_low         CRC32(slot[0x000..0x1FF])
      16   4     crc_high        CRC32(slot[0x240..image_size-1])
      20   4     build_unix      build time (UTC seconds), display only
      24   4     entry           == slot[4], cross-checked against the vector
      28   2     board_id        APP_BOARD_ID
      30   2     proto_version   APP_PROTO_VERSION
      32   2     flags           APP_HDR_FLAG_*
      34   30    reserved        zero

    The header cannot cover itself with a CRC (that would be self-referential),
    which is why the image is split into two CRC-covered segments and the header
    sits in the hole between them.
*/

#ifndef APP_HEADER_H
#define APP_HEADER_H

#include <stddef.h>
#include <stdint.h>

#include "board.h"

typedef struct {
    uint32_t magic;
    uint16_t header_version;
    uint16_t image_version;
    uint32_t image_size;
    uint32_t crc_low;
    uint32_t crc_high;
    uint32_t build_unix;
    uint32_t entry;
    uint16_t board_id;
    uint16_t proto_version;
    uint16_t flags;
    uint8_t  reserved[30];
} app_header_t;

/* compile-time proof that the struct is exactly the size the linker script and
   the packaging tool assume */
typedef char app_header_must_be_64[(sizeof(app_header_t) == APP_HDR_SIZE) ? 1 : -1];

/* validation results - anything but APP_HDR_OK makes the slot unbootable */
#define APP_HDR_OK              0
#define APP_HDR_ERR_MAGIC       1
#define APP_HDR_ERR_VERSION     2
#define APP_HDR_ERR_SIZE        3
#define APP_HDR_ERR_ALIGN       4
#define APP_HDR_ERR_ENTRY       5
#define APP_HDR_ERR_CRC_LOW     6
#define APP_HDR_ERR_CRC_HIGH    7
#define APP_HDR_ERR_BOARD       8
#define APP_HDR_ERR_PROTO       9
#define APP_HDR_ERR_NO_BOOT     10      /* valid, but deliberately disabled */

/*! \brief one-line description of an APP_HDR_ERR_* / APP_HDR_OK code. */
const char *app_hdr_strerror(int code);

/* ── CRC over the two header-free segments ──────────────────────────────── */

/*! \brief CRC32 of the vector table and padding: slot[0x000 .. 0x1FF]. */
uint32_t app_image_crc_low(const uint8_t *img, uint32_t size);
/*! \brief CRC32 of the code: slot[0x240 .. size-1]. */
uint32_t app_image_crc_high(const uint8_t *img, uint32_t size);
/*! \brief CRC32 of `low || high` concatenated, a single running CRC.

    This is what `UPG_IMGCRC` carries and what the node recomputes in
    `UPG_END`, so host and node do not have to trust each other's splitting.
    Equivalent to `zlib.crc32(high, zlib.crc32(low))`. */
uint32_t app_image_crc_combined(const uint8_t *img, uint32_t size);

/* ── writing and checking ───────────────────────────────────────────────── */

/*! \brief fill in the header of a slot image in place (packaging / tests).

    `img` must already have its 64-byte header area reserved at
    `img[APP_HDR_OFF]` (the linker script does that for slot builds), the vector
    table must be in place (`img[4]` is the entry point) and `size` must be the
    exact image length, word aligned.

    \retval APP_HDR_OK or APP_HDR_ERR_* */
int app_header_fill(uint8_t *img, uint32_t size, uint16_t image_version,
                    uint32_t build_unix, uint16_t flags);

/*! \brief validate the image in a slot.
    \param[in] img  slot base (vector table) as seen by this core
    \param[in] size bytes available in the slot (SLOT_SIZE)
    \param[out] out optional copy of the header, filled on success

    The absolute address of `entry` is *not* checked here (a host tool validates
    an image whose link addresses do not match its own buffer); the bootloader
    does that separately with `app_header_entry_in_slot()`.
    \retval APP_HDR_OK or APP_HDR_ERR_* */
int app_header_check(const uint8_t *img, uint32_t size, app_header_t *out);

/*! \brief true when `entry` would land inside the slot (Thumb bit set). */
int app_header_entry_in_slot(const app_header_t *h, uint32_t slot_base,
                             uint32_t slot_size);

/* ── the running application ────────────────────────────────────────────── */

/*! \brief the header the application is running under, or NULL for APP_SLOT=0
           (development image with no bootloader). */
const app_header_t *app_own_header(void);

/*! \brief validate the running image (APP_HDR_OK, or an error when there is no
           header at all). */
int app_own_header_check(void);

/*! \brief SLOT_A / SLOT_B / SLOT_NONE for the running image. */
uint8_t app_own_slot(void);

/*! \brief the 64-byte header placeholder the packaging tool overwrites.
           Only defined for APP_SLOT=1|2; see src/app_header.c. */
extern app_header_t g_app_header;

#endif /* APP_HEADER_H */
