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
  .data   : {
    __data_start = .;
    *(.data .data.*)
    __data_end = .;
  } > RAM

  /* No separate flash load address on this board (the whole image loads
   * directly into RAM) — `rivet-rt`'s shared `_start` always copies
   * `.data` from `__data_load`, so this just makes that copy a no-op
   * self-copy rather than needing a QEMU-virt-specific boot path. */
  __data_load = __data_start;

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
   * Sized conservatively; re-measured in Phase 2.7 via watermarking.
   *
   * plan.md Phase 19: on real SMP, two harts trapping concurrently must
   * not share this stack (each would corrupt the other's saved frame).
   * Reserved here as 4 * 2K = 8K, one 2K slice per hart, statically sized
   * to this board's documented SMP ceiling (plan.md Phase 19: "qemu-virt
   * SMP build sets [`RIVET_MAX_HARTS`] to the -smp count, max 4") rather
   * than dynamically from the env var, since the linker script isn't
   * templated from it — `rivet-arch-riscv::__rivet_arch_init` picks its
   * own hart's slice via `hart_id()` at runtime, so any configured
   * `RIVET_MAX_HARTS <= 4` just leaves the unused tail slices untouched.
   * QEMU virt's RAM (128K, see MEMORY above) is the hard limit on going
   * any higher without also shrinking something else. */
  .isr_stack (NOLOAD) : ALIGN(16) {
    __isr_stack_bottom = .;
    . += 2K * 4;
    __isr_stack_top = .;
  } > RAM

  /* Secondary-hart boot stacks (plan.md Phase 19): hart 0 uses `.stack`
   * above; harts 1.. need their own boot stack before they can safely run
   * any Rust at all (Rust always needs a valid `sp`, even before the
   * kernel proper starts). One 512B slice per hart (shallow, fixed-depth
   * call chain — `_start` -> `rivet_secondary_main` -> `rivet::
   * run_secondary_hart` -> `port::arch::init`/`preempt::
   * start_secondary_hart`'s dispatch loop — before switching onto the
   * dispatched task's own pool stack and never touching this one again),
   * same 4-hart-ceiling reasoning as `.isr_stack`. `_start`'s asm indexes
   * this by `mhartid` (see `rivet-rt`). Each secondary hart's own task/
   * ISR stacks (once it's running the scheduler) come from the normal
   * `.task_stacks`/`.isr_stack` pools like hart 0's. */
  .secondary_stacks (NOLOAD) : ALIGN(16) {
    __secondary_stacks_bottom = .;
    . += 512 * 4;
    __secondary_stacks_top = .;
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
