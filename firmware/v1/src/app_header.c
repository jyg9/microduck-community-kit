/*!
    \file    app_header.c
    \brief   see app_header.h
*/

#include "app_header.h"

#include <string.h>

#include "boot_crc32.h"

#if (APP_SLOT == 1)
#define APP_SLOT_BASE   SLOT_A_BASE
#elif (APP_SLOT == 2)
#define APP_SLOT_BASE   SLOT_B_BASE
#endif

/* ── the header of the running application ──────────────────────────────── */

#ifdef APP_SLOT_BASE
/* The linker script pins this object to slot_base + APP_HDR_OFF (`>FLASH`,
   `.app_header`).  The packaging tool patches the 64 bytes in the produced
   .bin/.hex; the values below are deliberately not bootable (image_size = 0),
   so an unpackaged image cannot be jumped into by accident.

   It is non-const on purpose: a `const` object with a known initializer could
   be constant-folded by the compiler, and the application has to read the
   values that the *packaging tool* wrote into flash, not the ones this file
   was compiled with. */
__attribute__((section(".app_header"), used, aligned(4)))
app_header_t g_app_header = {
    APP_HDR_MAGIC,          /* magic          */
    APP_HDR_VERSION,        /* header_version */
    APP_VERSION,            /* image_version  */
    0U,                     /* image_size: 0 until the tool packages it */
    0U,                     /* crc_low        */
    0U,                     /* crc_high       */
    0U,                     /* build_unix     */
    0U,                     /* entry          */
    APP_BOARD_ID,           /* board_id       */
    APP_PROTO_VERSION,      /* proto_version  */
    0U,                     /* flags          */
    { 0U }                  /* reserved       */
};

const app_header_t *app_own_header(void)
{
    return (const app_header_t *)&g_app_header;
}

uint8_t app_own_slot(void)
{
#if (APP_SLOT == 1)
    return SLOT_A;
#else
    return SLOT_B;
#endif
}
#else
const app_header_t *app_own_header(void)
{
    /* APP_SLOT=0: a plain image at 0x08000000 with no bootloader and no header */
    return NULL;
}

uint8_t app_own_slot(void)
{
    return SLOT_NONE;
}
#endif /* APP_SLOT_BASE */

int app_own_header_check(void)
{
    const app_header_t *h = app_own_header();

    if (NULL == h) {
        return APP_HDR_ERR_MAGIC;
    }
    return app_header_check((const uint8_t *)(const void *)h - APP_HDR_OFF,
                            SLOT_SIZE, NULL);
}

/* ── error strings ──────────────────────────────────────────────────────── */

const char *app_hdr_strerror(int code)
{
    switch (code) {
    case APP_HDR_OK:            return "ok";
    case APP_HDR_ERR_MAGIC:     return "no header (slot empty or never packaged)";
    case APP_HDR_ERR_VERSION:   return "unknown header version";
    case APP_HDR_ERR_SIZE:      return "image_size out of range";
    case APP_HDR_ERR_ALIGN:     return "image_size not word aligned";
    case APP_HDR_ERR_ENTRY:     return "entry does not match the vector table";
    case APP_HDR_ERR_CRC_LOW:   return "crc_low mismatch (vector table region)";
    case APP_HDR_ERR_CRC_HIGH:  return "crc_high mismatch (code region)";
    case APP_HDR_ERR_BOARD:     return "wrong board id";
    case APP_HDR_ERR_PROTO:     return "unsupported upgrade protocol version";
    case APP_HDR_ERR_NO_BOOT:   return "slot disabled (NO_BOOT flag)";
    default:                    return "unknown error";
    }
}

/* ── CRC segments ───────────────────────────────────────────────────────── */

uint32_t app_image_crc_low(const uint8_t *img, uint32_t size)
{
    if (size < APP_HDR_OFF) {
        return boot_crc32(NULL, 0U);
    }
    return boot_crc32(img, APP_HDR_OFF);
}

uint32_t app_image_crc_high(const uint8_t *img, uint32_t size)
{
    uint32_t start = APP_HDR_OFF + APP_HDR_SIZE;

    if (size <= start) {
        return boot_crc32(NULL, 0U);
    }
    return boot_crc32(&img[start], size - start);
}

uint32_t app_image_crc_combined(const uint8_t *img, uint32_t size)
{
    uint32_t start = APP_HDR_OFF + APP_HDR_SIZE;
    uint32_t crc = app_image_crc_low(img, size);

    if (size > start) {
        crc = boot_crc32_update(crc, &img[start], size - start);
    }
    return crc;
}

/* ── writing ────────────────────────────────────────────────────────────── */

int app_header_fill(uint8_t *img, uint32_t size, uint16_t image_version,
                    uint32_t build_unix, uint16_t flags)
{
    app_header_t h;
    uint32_t entry;

    if (size < UPG_MIN_IMGLEN) {
        return APP_HDR_ERR_SIZE;
    }
    if ((size % 4U) != 0U) {
        return APP_HDR_ERR_ALIGN;
    }
    memcpy(&entry, &img[4], sizeof(entry));
    if (0U == (entry & 1U)) {
        return APP_HDR_ERR_ENTRY;   /* not a Thumb address: not a vector table */
    }

    memset(&h, 0, sizeof(h));
    h.magic = APP_HDR_MAGIC;
    h.header_version = APP_HDR_VERSION;
    h.image_version = image_version;
    h.image_size = size;
    h.crc_low = app_image_crc_low(img, size);
    h.crc_high = app_image_crc_high(img, size);
    h.build_unix = build_unix;
    h.entry = entry;
    h.board_id = APP_BOARD_ID;
    h.proto_version = APP_PROTO_VERSION;
    h.flags = flags;

    /* the header region itself is not covered by either CRC, so writing it last
       (and writing it whole) is safe */
    memcpy(&img[APP_HDR_OFF], &h, sizeof(h));
    return APP_HDR_OK;
}

/* ── checking ───────────────────────────────────────────────────────────── */

int app_header_entry_in_slot(const app_header_t *h, uint32_t slot_base,
                             uint32_t slot_size)
{
    uint32_t lo = slot_base + UPG_MIN_IMGLEN;
    uint32_t hi = slot_base + slot_size;

    if (0U == (h->entry & 1U)) {
        return 0;
    }
    if (h->entry < lo || h->entry >= hi) {
        return 0;
    }
    return 1;
}

int app_header_check(const uint8_t *img, uint32_t size, app_header_t *out)
{
    app_header_t h;
    uint32_t entry;

    if (NULL == img || size < UPG_MIN_IMGLEN) {
        return APP_HDR_ERR_SIZE;
    }
    memcpy(&h, &img[APP_HDR_OFF], sizeof(h));

    if (h.magic != APP_HDR_MAGIC) {
        return APP_HDR_ERR_MAGIC;
    }
    if (h.header_version != APP_HDR_VERSION) {
        return APP_HDR_ERR_VERSION;
    }
    if (h.image_size < UPG_MIN_IMGLEN || h.image_size > size) {
        return APP_HDR_ERR_SIZE;
    }
    if (0U != (h.image_size % 4U)) {
        return APP_HDR_ERR_ALIGN;
    }
    memcpy(&entry, &img[4], sizeof(entry));
    if (h.entry != entry || 0U == (entry & 1U)) {
        return APP_HDR_ERR_ENTRY;
    }
    if (h.crc_low != app_image_crc_low(img, h.image_size)) {
        return APP_HDR_ERR_CRC_LOW;
    }
    if (h.crc_high != app_image_crc_high(img, h.image_size)) {
        return APP_HDR_ERR_CRC_HIGH;
    }
    if (h.board_id != APP_BOARD_ID) {
        return APP_HDR_ERR_BOARD;
    }
    if (h.proto_version != APP_PROTO_VERSION) {
        return APP_HDR_ERR_PROTO;
    }
    if (0U != (h.flags & APP_HDR_FLAG_NO_BOOT)) {
        if (NULL != out) {
            memcpy(out, &h, sizeof(h));
        }
        return APP_HDR_ERR_NO_BOOT;
    }
    if (NULL != out) {
        memcpy(out, &h, sizeof(h));
    }
    return APP_HDR_OK;
}
