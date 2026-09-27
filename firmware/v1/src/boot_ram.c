/*!
    \file    boot_ram.c
    \brief   see boot_ram.h
*/

#include "boot_ram.h"

#include <stddef.h>

/* NOINIT: the linker script places this section at BOOT_RAM_ADDR and the
   startup code's zero-fill loop only covers `.bss`, so the value written by the
   application is still there when the bootloader runs after the reset. */
__attribute__((section(".boot_ram"), used, aligned(8)))
boot_ram_t g_boot_ram;

void boot_ram_request(uint32_t arg)
{
    g_boot_ram.arg = arg;
    __asm__ volatile("" ::: "memory");
    g_boot_ram.magic = BOOT_RAM_MAGIC;
    __asm__ volatile("" ::: "memory");
}

int boot_ram_consume(uint32_t *arg)
{
    if (g_boot_ram.magic != BOOT_RAM_MAGIC) {
        return 0;
    }
    if (NULL != arg) {
        *arg = g_boot_ram.arg;
    }
    /* clear before anything else can reset again, so a stray reset cannot loop
       back into boot mode forever */
    g_boot_ram.magic = 0U;
    __asm__ volatile("" ::: "memory");
    return 1;
}
