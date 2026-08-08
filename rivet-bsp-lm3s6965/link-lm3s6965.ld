/* Linker script for LM3S6965 (Cortex-M3) on QEMU.
 * ROM at 0x0 (256 KB), RAM at 0x20000000 (64 KB).
 */

MEMORY
{
  FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 256K
  RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 64K
}

ENTRY(Reset);

SECTIONS
{
  /* Vector table at start of flash */
  .vector_table ORIGIN(FLASH) : {
    LONG(ORIGIN(RAM) + LENGTH(RAM));  /* Initial stack pointer */
    LONG(Reset);                        /* Reset handler */
    LONG(NMI);
    LONG(HardFault);
    LONG(MemManage);
    LONG(BusFault);
    LONG(UsageFault);
    LONG(0);                            /* Reserved */
    LONG(0);
    LONG(0);
    LONG(0);
    LONG(rivet_svc_handler);  /* kernel SVC (frame init from Handler mode) */
    LONG(DebugMon);
    LONG(0);
    LONG(PendSV);
    LONG(SysTick);
    /* IRQ 0-31 */
    FILL(0);
    . = ORIGIN(FLASH) + 0x100;
  } > FLASH

  .text : {
    *(.text .text.*);
    *(.rodata .rodata.*);
  } > FLASH

  .data : {
    __data_start = .;
    *(.data .data.*);
    __data_end = .;
  } > RAM AT > FLASH

  __data_load = LOADADDR(.data);

  .bss (NOLOAD) : {
    __bss_start = .;
    *(.bss .bss.*);
    *(COMMON);
    __bss_end = .;
  } > RAM

  /* Task registry section — read-only in flash */
  .rivet_tasks : {
    __rivet_tasks_start = .;
    KEEP(*(.rivet_tasks));
    __rivet_tasks_end = .;
  } > FLASH

  /* System stack */
  /* Preemptive task stacks: one contiguous, 16 KiB-aligned pool (plan.md
   * §3) — MPU region 6 denies the whole pool; region 7 enables only the
   * currently-running task's stack. Stacks are allocated from this pool
   * at spawn time (power-of-two sizes). */
  .task_stacks (NOLOAD) : ALIGN(16384) {
    __task_stacks_start = .;
    . += 16K;
    __task_stacks_end = .;
  } > RAM

  .stack (NOLOAD) : ALIGN(8) {
    __stack_bottom = .;
    . += 8K;
    __stack_top = .;
  } > RAM

  _end = .;
}

/* Dummy exception handlers (overridden by user if needed) */
PROVIDE(NMI = DefaultHandler);
PROVIDE(HardFault = DefaultHandler);
PROVIDE(MemManage = DefaultHandler);
PROVIDE(BusFault = DefaultHandler);
PROVIDE(UsageFault = DefaultHandler);
PROVIDE(SVC = DefaultHandler);
PROVIDE(DebugMon = DefaultHandler);
PROVIDE(PendSV = DefaultHandler);
PROVIDE(SysTick = DefaultHandler);
