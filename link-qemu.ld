/* QEMU RISC-V virt machine: kernel loaded at 0x80000000.
 * Includes .rivet_tasks section for task discovery.
 */
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

  .bss : {
    __bss_start = .;
    *(.bss .bss.*)
    *(COMMON)
    __bss_end = .;
  } > RAM

  /* Task registry: executor discovers tasks by walking this section */
  .rivet_tasks : {
    __rivet_tasks_start = .;
    KEEP(*(.rivet_tasks));
    __rivet_tasks_end = .;
  } > RAM

  /* System stack */
  .stack (NOLOAD) : ALIGN(16) {
    __stack_bottom = .;
    . += 8K;
    __stack_top = .;
  } > RAM

  /* Dedicated interrupt/trap-handler stack (plan.md §2.1 / [B3]).
   * The RISC-V trap entry keeps the 128-byte frame on the interrupted
   * task's own stack but runs the Rust handler (schedule, poll_timers,
   * panic formatting, fault handling) on this stack via mscratch.
   * Sized conservatively; re-measured in Phase 2.7 via watermarking. */
  .isr_stack (NOLOAD) : ALIGN(16) {
    __isr_stack_bottom = .;
    . += 2K;
    __isr_stack_top = .;
  } > RAM

  /* Preemptive task stacks: one contiguous, 16 KiB-aligned pool (plan.md
   * §3) so the RISC-V PMP can guard each stack's low end and the CM3 MPU
   * can deny the whole pool with one region while enabling only the
   * currently-running task's stack with another. Stacks are allocated
   * from this pool at spawn time (power-of-two sizes). */
  .task_stacks (NOLOAD) : ALIGN(16384) {
    __task_stacks_start = .;
    . += 16K;
    __task_stacks_end = .;
  } > RAM

  _end = .;
}
