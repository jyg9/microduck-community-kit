/*
 * GD32F303CC (GD32F30X_HD, LQFP48) - slot_a image
 *   FLASH : 256 KB @ 0x08000000
 *   SRAM  :  48 KB @ 0x20000000
 *
 * Layout follows the GigaDevice V3.0.3 reference script for the xC density
 * (Firmware/CMSIS/GD/GD32F30x/Source/GCC/Ld/gd32f303xC_flash.ld).
 *
 * The startup file (startup_gd32f30x_hd.S) puts the initial main stack
 * pointer in the vector table as "_sp", so that symbol must exist; it is
 * provided at the top of the heap_stack reservation.
 */

ENTRY(Reset_Handler)

MEMORY
{
  FLASH (rx)  : ORIGIN = 0x08008000, LENGTH = 96K
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
    /* Pad to the app header with 0xFF, i.e. the value of erased flash.  The gap
       is part of the CRC-covered vector-table segment, so it must read the same
       in the .bin, in the .hex and in flash - the linker would otherwise fill it
       with 0x00 and every image would fail its own crc_low check. */
    FILL(0xFF)
    . = 0x200;
  } >FLASH

  /* Application image header (src/app_header.h).  The Cortex-M4 fetches the
     initial MSP and the reset vector from slot_base + 0 / + 4, so nothing may
     be placed in front of the vector table: the header goes to a fixed 0x200
     offset instead, which is why the payload starts at 0x240.  The packaging
     tool fills these 64 bytes in the produced .bin/.hex. */
  .app_header : ALIGN(4)
  {
    KEEP(*(.app_header))
  } >FLASH

  /* The .vectors section above pads itself to exactly 0x200 with 0xFF, so this
     section lands on slot_base + 0x200.  Enforce it at link time: the
     bootloader and the packaging tool both hardcode that offset. */
  ASSERT(ADDR(.app_header) == ORIGIN(FLASH) + 0x200,
         "the application header must be at slot_base + 0x200")
  ASSERT(SIZEOF(.app_header) == 0x40, "the application header must be 64 bytes")

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
