/* Raspberry Pi Pico (RP2040), real hardware. Cortex-M0+ (ARMv6-M), no
 * FPU, no hardware-FP ABI — same reasoning as every other Cortex-M board
 * in this workspace: build for `thumbv6m-none-eabi`, never emit an
 * instruction that would need FP context.
 *
 * Flash 2 MiB (XIP-mapped) at 0x10000000, RAM 256 KiB (the four
 * contiguous 64 KiB SRAM0-3 banks; the extra two 4 KiB "striped" scratch
 * banks at 0x20040000 are left unused — not needed for anything this
 * kernel does).
 *
 * Unlike every other board here, flash does not start with a vector
 * table: RP2040's boot ROM reads the first 256 bytes as a checksummed
 * "boot2" second-stage bootloader (see rivet-bsp-rp2040's own module
 * docs for the full story) and only jumps into the real vector table
 * once *that* has brought the QSPI flash into XIP mode. `.boot2` is
 * therefore its own section, pinned to `ORIGIN(FLASH)`, exactly 256
 * bytes, ahead of everything else.
 */

MEMORY
{
  FLASH (rx)  : ORIGIN = 0x10000000, LENGTH = 2048K
  RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 256K
}

ENTRY(Reset);

SECTIONS
{
  .boot2 ORIGIN(FLASH) : {
    KEEP(*(.boot2));
    . = ORIGIN(FLASH) + 0x100;
  } > FLASH

  /* Vector table right after boot2. ARMv6-M's fixed-exception layout is
   * shorter than ARMv7-M's: no MemManage/BusFault/UsageFault/DebugMonitor
   * (slots 4-10 and 12-13 are plain Reserved, not routed to anything —
   * this architecture has exactly one fault handler, HardFault, for
   * everything those would have been on M3/M4) and this port's
   * `rivet-arch-cortex-m0` never installs an SVC handler (no MPU to work
   * around from Handler mode — see that crate's own docs — so SVCall
   * falls back to `DefaultHandler` like any other unused vector).
   * IRQ 0-25 are RP2040's real, fixed external-interrupt positions (see
   * rivet-bsp-rp2040::irq) — all routed to the one shared
   * `rivet_irq_handler` trampoline, same as every other Cortex-M board
   * in this workspace. */
  .vector_table ORIGIN(FLASH) + 0x100 : {
    __vector_table = .;
    LONG(ORIGIN(RAM) + LENGTH(RAM));  /* Initial stack pointer */
    LONG(Reset);                        /* Reset handler */
    LONG(NMI);
    LONG(HardFault);
    LONG(0); LONG(0); LONG(0); LONG(0); LONG(0); LONG(0); LONG(0); /* Reserved (4-10) */
    LONG(DefaultHandler);               /* SVCall (11) — unused on this port */
    LONG(0); LONG(0);                   /* Reserved (12-13) */
    LONG(PendSV);
    LONG(SysTick);
    /* IRQ 0-25 */
    LONG(rivet_irq_handler); LONG(rivet_irq_handler); LONG(rivet_irq_handler); LONG(rivet_irq_handler);
    LONG(rivet_irq_handler); LONG(rivet_irq_handler); LONG(rivet_irq_handler); LONG(rivet_irq_handler);
    LONG(rivet_irq_handler); LONG(rivet_irq_handler); LONG(rivet_irq_handler); LONG(rivet_irq_handler);
    LONG(rivet_irq_handler); LONG(rivet_irq_handler); LONG(rivet_irq_handler); LONG(rivet_irq_handler);
    LONG(rivet_irq_handler); LONG(rivet_irq_handler); LONG(rivet_irq_handler); LONG(rivet_irq_handler);
    LONG(rivet_irq_handler); LONG(rivet_irq_handler); LONG(rivet_irq_handler); LONG(rivet_irq_handler);
    LONG(rivet_irq_handler); LONG(rivet_irq_handler);
    FILL(0);
    . = ORIGIN(FLASH) + 0x100 + 0x200;
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

  /* Preemptive task stacks: one contiguous, 16 KiB-aligned pool — same
   * shape as every other board (no MPU on this arch to actually enforce
   * isolation with it — see rivet-arch-cortex-m0's own docs — but the
   * pool/stack-watermark machinery above the arch layer is unconditional,
   * so it still needs somewhere real to live). */
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

  /* Same orphan-section reasoning as every other real-hardware board's
   * linker script in this workspace: EABI unwind tables carry no useful
   * content here (no C++ exceptions, never unwinds) and would otherwise
   * land in RAM via the linker's default orphan placement, ballooning a
   * flashable `.bin`. */
  /DISCARD/ : {
    *(.ARM.exidx*)
    *(.ARM.extab*)
  }
}

/* Dummy exception handlers (overridden by user if needed) */
PROVIDE(NMI = DefaultHandler);
PROVIDE(HardFault = DefaultHandler);
PROVIDE(PendSV = DefaultHandler);
PROVIDE(SysTick = DefaultHandler);
PROVIDE(rivet_irq_handler = DefaultHandler);
