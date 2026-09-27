/*
 * GD32F303CC (GD32F30X_HD, LQFP48) - bootloader image
 *   FLASH : 256 KB @ 0x08000000
 *   SRAM  :  48 KB @ 0x20000000
 *
 * Layout follows the GigaDevice V3.0.3 reference script for the xC density
 * (Firmware/CMSIS/GD/GD32F30x/Source/GCC/Ld/gd32f303xC_flash.ld).
 *
 * The startup file (startup_gd32f30x_hd.S) puts the initial main stack
 * pointer in the vector table as "_sp", so that symbol must exist; it is
 * provided at the top of the heap_stack reservation.
 *
 * This is the 32 KB bootloader at 0x08000000: it validates the two application
 * slots and can be entered over the motor bus (docs/flash_layout.md).  It is
 * only ever programmed with a debugger, never from the bus.
 */

ENTRY(Reset_Handler)

MEMORY
{
  FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 32K
  RAM   (xrw) : ORIGIN = 0x20000000, LENGTH = 48896
  BOOTRAM (rw): ORIGIN = 0x2000BF00, LENGTH = 256
}

SECTIONS
{
  __stack_size = DEFINED(__stack_size) ? __stack_size : 2K;
  __heap_size  = DEFINED(__heap_size)  ? __heap_size  : 1K;

  /* ISR vectors - must be the first thing in FLASH */
  .vectors :
  {
    . = ALIGN(4);
    KEEP(*(.vectors))
    . = ALIGN(4);
    __Vectors_End  = .;
    __Vectors_Size = __Vectors_End - __gVectors;
  } >FLASH

  /* program code */
  .text :
  {
    . = ALIGN(4);
    *(.text)
    *(.text*)
    *(.glue_7)
    *(.glue_7t)
    *(.eh_frame)

    KEEP (*(.init))
    KEEP (*(.fini))

    . = ALIGN(4);
    _etext = .;
  } >FLASH

  /* read-only data */
  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)
    *(.rodata*)
    . = ALIGN(4);
  } >FLASH

  /* exception unwind tables */
  .ARM.extab :
  {
    *(.ARM.extab* .gnu.linkonce.armextab.*)
  } >FLASH

  .ARM :
  {
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
  } >FLASH

  .ARM.attributes :
  {
    *(.ARM.attributes)
  } >FLASH

  .preinit_array :
  {
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array*))
    PROVIDE_HIDDEN (__preinit_array_end = .);
  } >FLASH

  .init_array :
  {
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array*))
    PROVIDE_HIDDEN (__init_array_end = .);
  } >FLASH

  .fini_array :
  {
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(.fini_array*))
    KEEP (*(SORT(.fini_array.*)))
    PROVIDE_HIDDEN (__fini_array_end = .);
  } >FLASH

  /* initialized data: lives in RAM, copied from FLASH by the startup code */
  _sidata = LOADADDR(.data);

  .data :
  {
    . = ALIGN(4);
    _sdata = .;
    *(.data)
    *(.data*)
    . = ALIGN(4);
    _edata = .;
  } >RAM AT> FLASH

  /* zero-initialized data */
  . = ALIGN(4);
  .bss :
  {
    . = ALIGN(4);
    _sbss = .;
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
    __bss_end__ = _ebss;
  } >RAM

  /* heap and stack space (also catches RAM overflow at link time) */
  .heap_stack :
  {
    . = ALIGN(8);
    PROVIDE ( end  = _ebss );
    PROVIDE ( _end = _ebss );
    . = . + __heap_size;
    PROVIDE( _heap_end = . );
    . = . + __stack_size;
    PROVIDE( _sp = . );
    . = ALIGN(8);
  } >RAM

  /* The last 256 bytes of SRAM hold the application -> bootloader handshake
     word.  It lives in its own 256-byte memory region at the top of SRAM,
     outside .bss (the startup code must not clear it) and above the stack, so a
     deep call stack can never overwrite it.  See src/boot_ram.h. */
  .boot_ram (NOLOAD) :
  {
    . = ALIGN(8);
    KEEP(*(.boot_ram))
    . = ALIGN(8);
  } >BOOTRAM

  ASSERT(_sp <= 0x2000BF00, "stack overlaps the .boot_ram handshake area")
}
