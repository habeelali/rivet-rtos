/* STM32F401RE (Nucleo-64), real hardware. Cortex-M4, no hardware-FP ABI
 * (context switch here never saves/restores FPU registers — this
 * workspace's Cortex-M port has never needed to; kept that way by
 * building for `thumbv7em-none-eabi`, not `-eabihf`, so the compiler
 * never emits an instruction that would need it).
 *
 * Flash 512 KiB at 0x08000000, RAM 96 KiB at 0x20000000 (single SRAM
 * block on this part — no separate CCM, unlike larger F4s).
 */

MEMORY
{
  FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 512K
  RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 96K
}

ENTRY(Reset);

SECTIONS
{
  /* Vector table at start of flash. Slots 0-14 are the fixed ARMv7-M
   * exceptions (same shape as every other Cortex-M board in this
   * workspace); slots 16+ are this chip's real external-interrupt
   * positions (up to SPI4 = 84, per the `stm32f401` PAC's own vector
   * table) — all routed to the one shared `rivet_irq_handler` trampoline
   * (it reads `SCB.ICSR.VECTACTIVE` to know which actually fired, so one
   * symbol correctly serves every slot; see rivet-arch-cortex-m/src/
   * nvic.rs). Generous headroom (96 slots) rather than trimming to
   * exactly what's used today, so a later test that reaches for another
   * peripheral's IRQ doesn't also need a linker-script edit. */
  .vector_table ORIGIN(FLASH) : {
    /* `__vector_table` — SCB.VTOR's reset value is 0x00000000, but this
     * board's vector table lives at 0x08000000 (real flash, not QEMU's
     * conveniently-zero-based load address); rivet-arch-cortex-m's init
     * sets VTOR from this symbol on every Cortex-M board so an
     * interrupt (SysTick, PendSV, an NVIC IRQ) actually vectors through
     * *this* table instead of whatever VTOR defaults to. Confirmed the
     * hard way on real hardware: without this, boot printed fine (the
     * CPU's own reset vector fetch is hardware-latched, unaffected by
     * VTOR) but SysTick never once reached `rivet_arch_cortex_m::
     * systick::handler` — no crash, no fault dump, just silence forever
     * after the first console output, because the exception hardware
     * was reading a *different*, effectively-empty vector table at
     * address 0. */
    __vector_table = .;
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
    /* IRQ 0-95 (plan.md Phase 26 follow-up): one shared trampoline per
     * slot, matching every other Cortex-M board in this workspace. */
    LONG(rivet_irq_handler); LONG(rivet_irq_handler); LONG(rivet_irq_handler); LONG(rivet_irq_handler);
    LONG(rivet_irq_handler); LONG(rivet_irq_handler); LONG(rivet_irq_handler); LONG(rivet_irq_handler);
    LONG(rivet_irq_handler); LONG(rivet_irq_handler); LONG(rivet_irq_handler); LONG(rivet_irq_handler);
    LONG(rivet_irq_handler); LONG(rivet_irq_handler); LONG(rivet_irq_handler); LONG(rivet_irq_handler);
    LONG(rivet_irq_handler); LONG(rivet_irq_handler); LONG(rivet_irq_handler); LONG(rivet_irq_handler);
    LONG(rivet_irq_handler); LONG(rivet_irq_handler); LONG(rivet_irq_handler); LONG(rivet_irq_handler);
    LONG(rivet_irq_handler); LONG(rivet_irq_handler); LONG(rivet_irq_handler); LONG(rivet_irq_handler);
    LONG(rivet_irq_handler); LONG(rivet_irq_handler); LONG(rivet_irq_handler); LONG(rivet_irq_handler);
    LONG(rivet_irq_handler); LONG(rivet_irq_handler); LONG(rivet_irq_handler); LONG(rivet_irq_handler);
    LONG(rivet_irq_handler); LONG(rivet_irq_handler); LONG(rivet_irq_handler); LONG(rivet_irq_handler);
    LONG(rivet_irq_handler); LONG(rivet_irq_handler); LONG(rivet_irq_handler); LONG(rivet_irq_handler);
    LONG(rivet_irq_handler); LONG(rivet_irq_handler); LONG(rivet_irq_handler); LONG(rivet_irq_handler);
    LONG(rivet_irq_handler); LONG(rivet_irq_handler); LONG(rivet_irq_handler); LONG(rivet_irq_handler);
    LONG(rivet_irq_handler); LONG(rivet_irq_handler); LONG(rivet_irq_handler); LONG(rivet_irq_handler);
    LONG(rivet_irq_handler); LONG(rivet_irq_handler); LONG(rivet_irq_handler); LONG(rivet_irq_handler);
    LONG(rivet_irq_handler); LONG(rivet_irq_handler); LONG(rivet_irq_handler); LONG(rivet_irq_handler);
    LONG(rivet_irq_handler); LONG(rivet_irq_handler); LONG(rivet_irq_handler); LONG(rivet_irq_handler);
    LONG(rivet_irq_handler); LONG(rivet_irq_handler); LONG(rivet_irq_handler); LONG(rivet_irq_handler);
    LONG(rivet_irq_handler); LONG(rivet_irq_handler); LONG(rivet_irq_handler); LONG(rivet_irq_handler);
    LONG(rivet_irq_handler); LONG(rivet_irq_handler); LONG(rivet_irq_handler); LONG(rivet_irq_handler);
    LONG(rivet_irq_handler); LONG(rivet_irq_handler); LONG(rivet_irq_handler); LONG(rivet_irq_handler);
    LONG(rivet_irq_handler); LONG(rivet_irq_handler); LONG(rivet_irq_handler); LONG(rivet_irq_handler);
    LONG(rivet_irq_handler); LONG(rivet_irq_handler); LONG(rivet_irq_handler); LONG(rivet_irq_handler);
    LONG(rivet_irq_handler); LONG(rivet_irq_handler); LONG(rivet_irq_handler); LONG(rivet_irq_handler);
    FILL(0);
    . = ORIGIN(FLASH) + 0x200;
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

  /* Preemptive task stacks: one contiguous, 16 KiB-aligned pool (plan.md
   * §3) — MPU region 6 denies the whole pool; region 7 enables only the
   * currently-running task's stack. */
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

  /* `.ARM.exidx`/`.ARM.extab`: the ARM EABI unwind-table sections
   * `arm-none-eabi`-flavour codegen emits automatically — this kernel
   * has no C++ exceptions and never unwinds, so they carry no useful
   * content, but left unrouted they become an *orphan section* the
   * linker places by its own default rules right after `.stack`,
   * landing at a RAM address with real (if tiny) file content. That's
   * harmless for the QEMU Cortex-M boards in this workspace (which boot
   * the ELF directly), but real hardware flashing needs a raw `.bin`
   * (objcopy -O binary), which spans from the lowest to the highest
   * *content-bearing* address in one contiguous file — a content-
   * bearing section landing in RAM, ~0x18000000 bytes past this board's
   * flash-resident code, turned a ~24 KB firmware image into a ~400 MB
   * one. Discarding them avoids the orphan placement entirely. */
  /DISCARD/ : {
    *(.ARM.exidx*)
    *(.ARM.extab*)
  }
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
PROVIDE(rivet_irq_handler = DefaultHandler);
