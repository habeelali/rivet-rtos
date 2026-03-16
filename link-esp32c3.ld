/* ESP32-C3: execute from flash (XIP). App at 0x42000020 (IROM). Data in DRAM.
 * Flash: code + rodata. DRAM: data (copied at startup), bss, stack, heap.
 */
OUTPUT_ARCH(riscv)
ENTRY(_start)

MEMORY
{
  FLASH (rx) : ORIGIN = 0x42000020, LENGTH = 0x100000   /* 1MB app in flash */
  DRAM (rwx) : ORIGIN = 0x3FC80000, LENGTH = 0x50000   /* 320KB DRAM */
}

SECTIONS
{
  .text : {
    *(.text._start)
    *(.text .text.*)
  } > FLASH

  .rodata : { *(.rodata .rodata.*) } > FLASH
  .eh_frame : { KEEP(*(.eh_frame)) } > FLASH

  .data : AT(LOADADDR(.eh_frame) + SIZEOF(.eh_frame)) {
    __data_start = .;
    *(.data .data.*)
    __data_end = .;
  } > DRAM

  __data_load = LOADADDR(.data);

  .bss : {
    __bss_start = .;
    *(.bss .bss.*)
    *(COMMON)
    __bss_end = .;
    __stack_bottom = .;
    . += 16K;
    __stack_top = .;
    __heap_start = .;
    . += 32K;
    __heap_end = .;
  } > DRAM

  _end = .;
}
