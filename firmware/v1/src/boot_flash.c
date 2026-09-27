/*!
    \file    boot_flash.c
    \brief   see boot_flash.h
*/

#include "boot_flash.h"

#include <string.h>

#ifndef IMU_TO_DXL_HOST_TEST
#include "gd32f30x.h"
#endif

/* ── address helpers ────────────────────────────────────────────────────── */

static int range_in_flash(uint32_t addr, uint32_t len)
{
    if (len == 0U) {
        return 0;
    }
    if (addr < FLASH_BASE_ADDR) {
        return 0;
    }
    if ((addr - FLASH_BASE_ADDR) > FLASH_TOTAL_SIZE) {
        return 0;
    }
    if (len > (FLASH_TOTAL_SIZE - (addr - FLASH_BASE_ADDR))) {
        return 0;
    }
    return 1;
}

uint32_t boot_flash_slot_base(uint32_t slot)
{
    if (SLOT_A == slot) {
        return SLOT_A_BASE;
    }
    if (SLOT_B == slot) {
        return SLOT_B_BASE;
    }
    return 0U;
}

static int slot_range_ok(uint32_t slot, uint32_t offset, uint32_t len)
{
    if (0U == boot_flash_slot_base(slot)) {
        return 0;
    }
    if (len == 0U) {
        return 0;
    }
    if (offset >= SLOT_SIZE) {
        return 0;
    }
    if (len > (SLOT_SIZE - offset)) {
        return 0;
    }
    return 1;
}

/* ── the simulated flash used by the host tests ─────────────────────────── */

#ifdef IMU_TO_DXL_HOST_TEST

static uint8_t  s_flash[FLASH_TOTAL_SIZE];
static uint8_t  s_present[FLASH_TOTAL_SIZE];   /* 0 = erased, 1 = programmed */
static uint32_t s_erase_count;
static uint32_t s_program_count;
static uint32_t s_reject_count;

void boot_flash_test_reset(void)
{
    memset(s_flash, 0xFF, sizeof(s_flash));
    memset(s_present, 0, sizeof(s_present));
    s_erase_count = 0U;
    s_program_count = 0U;
    s_reject_count = 0U;
}

void boot_flash_test_load(uint32_t addr, const void *src, uint32_t len)
{
    if (!range_in_flash(addr, len)) {
        s_reject_count++;
        return;
    }
    memcpy(&s_flash[addr - FLASH_BASE_ADDR], src, len);
    memset(&s_present[addr - FLASH_BASE_ADDR], 1, len);
}

uint8_t *boot_flash_test_base(void)
{
    return s_flash;
}

uint32_t boot_flash_test_erase_count(void)
{
    return s_erase_count;
}

uint32_t boot_flash_test_program_count(void)
{
    return s_program_count;
}

uint32_t boot_flash_test_reject_count(void)
{
    return s_reject_count;
}

const uint8_t *boot_flash_ptr(uint32_t addr)
{
    if (!range_in_flash(addr, 1U)) {
        return NULL;
    }
    return &s_flash[addr - FLASH_BASE_ADDR];
}

int boot_flash_read(uint32_t addr, void *dst, uint32_t len)
{
    if (NULL == dst || !range_in_flash(addr, len)) {
        return -1;
    }
    memcpy(dst, &s_flash[addr - FLASH_BASE_ADDR], len);
    return 0;
}

int boot_flash_erase_page(uint32_t addr)
{
    if ((addr % FLASH_PAGE_SIZE) != 0U) {
        s_reject_count++;
        return -1;
    }
    if (!range_in_flash(addr, FLASH_PAGE_SIZE)) {
        s_reject_count++;
        return -1;
    }
    s_erase_count++;
    memset(&s_flash[addr - FLASH_BASE_ADDR], 0xFF, FLASH_PAGE_SIZE);
    memset(&s_present[addr - FLASH_BASE_ADDR], 0, FLASH_PAGE_SIZE);
    return 0;
}

int boot_flash_program_word(uint32_t addr, uint32_t word)
{
    uint32_t idx;

    if ((addr % 4U) != 0U || !range_in_flash(addr, 4U)) {
        s_reject_count++;
        return -1;
    }
    idx = addr - FLASH_BASE_ADDR;
    /* real flash can only clear bits, and this is the bug class the tests are
       meant to catch (programming over a non-erased word) */
    if (0U != (~((uint32_t)s_flash[idx] | ((uint32_t)s_flash[idx + 1U] << 8)
                 | ((uint32_t)s_flash[idx + 2U] << 16)
                 | ((uint32_t)s_flash[idx + 3U] << 24)) & word)) {
        s_reject_count++;
        return -1;
    }
    s_program_count++;
    s_flash[idx] = (uint8_t)(word & 0xFFU);
    s_flash[idx + 1U] = (uint8_t)((word >> 8) & 0xFFU);
    s_flash[idx + 2U] = (uint8_t)((word >> 16) & 0xFFU);
    s_flash[idx + 3U] = (uint8_t)((word >> 24) & 0xFFU);
    memset(&s_present[idx], 1, 4U);
    return 0;
}

#else /* target build: the real FMC */

const uint8_t *boot_flash_ptr(uint32_t addr)
{
    if (!range_in_flash(addr, 1U)) {
        return NULL;
    }
    return (const uint8_t *)(uintptr_t)addr;
}

int boot_flash_read(uint32_t addr, void *dst, uint32_t len)
{
    if (NULL == dst || !range_in_flash(addr, len)) {
        return -1;
    }
    memcpy(dst, (const void *)(uintptr_t)addr, len);
    return 0;
}

int boot_flash_erase_page(uint32_t addr)
{
    fmc_state_enum st;

    if ((addr % FLASH_PAGE_SIZE) != 0U || !range_in_flash(addr, FLASH_PAGE_SIZE)) {
        return -1;
    }
    fmc_unlock();
    st = fmc_page_erase(addr);
    fmc_lock();
    return (FMC_READY == st) ? 0 : -1;
}

int boot_flash_program_word(uint32_t addr, uint32_t word)
{
    fmc_state_enum st;

    if ((addr % 4U) != 0U || !range_in_flash(addr, 4U)) {
        return -1;
    }
    fmc_unlock();
    st = fmc_word_program(addr, word);
    fmc_lock();
    return (FMC_READY == st) ? 0 : -1;
}

#endif /* IMU_TO_DXL_HOST_TEST */

/* ── the checked slot-only layer ────────────────────────────────────────── */

int boot_flash_slot_erase(uint32_t slot, uint32_t offset, uint32_t len)
{
    uint32_t base = boot_flash_slot_base(slot);
    uint32_t addr;
    uint32_t end;

    if (0U == base || !slot_range_ok(slot, offset, len)) {
        return -1;
    }
    if ((offset % FLASH_PAGE_SIZE) != 0U) {
        return -1;
    }
    end = offset + len;
    for (addr = offset; addr < end; addr += FLASH_PAGE_SIZE) {
        if (0 != boot_flash_erase_page(base + addr)) {
            return -1;
        }
    }
    return 0;
}

int boot_flash_slot_program(uint32_t slot, uint32_t offset, const void *src,
                            uint32_t len)
{
    uint32_t base = boot_flash_slot_base(slot);
    const uint8_t *p = (const uint8_t *)src;
    uint32_t i;

    if (0U == base || !slot_range_ok(slot, offset, len)) {
        return -1;
    }
    if ((offset % 4U) != 0U || (len % 4U) != 0U) {
        return -1;
    }
    for (i = 0U; i < len; i += 4U) {
        uint32_t word = (uint32_t)p[i]
                        | ((uint32_t)p[i + 1U] << 8)
                        | ((uint32_t)p[i + 2U] << 16)
                        | ((uint32_t)p[i + 3U] << 24);
        if (0 != boot_flash_program_word(base + offset + i, word)) {
            return -1;
        }
    }
    return 0;
}

const uint8_t *boot_flash_slot_ptr(uint32_t slot)
{
    uint32_t base = boot_flash_slot_base(slot);

    if (0U == base) {
        return NULL;
    }
    return boot_flash_ptr(base);
}
