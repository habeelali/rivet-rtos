/* QEMU RISC-V virt machine: kernel loaded at 0x80000000 */
OUTPUT_ARCH(riscv)
ENTRY(_start)

MEMORY
{
  RAM (rwx) : ORIGIN = 0x80000000, LENGTH = 128K
}

SECTIONS
{
  .text : {
    *(.text._start)
    *(.text .text.*)
  } > RAM

  .rodata : { *(.rodata .rodata.*) } > RAM
  .data   : { *(.data .data.*) } > RAM
  .bss    : {
    __bss_start = .;
    *(.bss .bss.*)
    *(COMMON)
    __bss_end = .;
    __stack_bottom = .;
    . += 4K;   /* bootstrap stack */
    __stack_top = .;
    __heap_start = .;
    . += 8K;   /* task stacks will be in .bss arrays */
    __heap_end = .;
  } > RAM

  _end = .;
}
