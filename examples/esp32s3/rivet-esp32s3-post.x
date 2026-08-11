/* Processed AFTER `link.x` (see build.rs). */
SECTIONS
{
  /* `link.x`'s own `.data : { ... } > RWDATA AT > RODATA` rule (a real
   * flash-to-RAM copy, appropriate for its own XIP-oriented design)
   * leaves the LMA cursor sitting inside the RODATA (flash) region
   * afterward; `.bss` (immediately after `.data` in `link.x`, no `AT>`
   * of its own) silently inherits that as ITS load address too — a real,
   * confirmed bogus `PhysAddr` in flash space on a *zero-file-size*
   * `NOLOAD` segment (`readelf -l` showed it directly), which fed a
   * spurious extra "ROM segment" into `espflash`'s image builder and
   * produced a bootloader-side `abort()` at hand-off with no diagnostic
   * message — not anticipated in advance, found by comparing program
   * headers against a known-working image. This board's whole design is
   * RAM-resident (see `memory.x`'s docs), so `.data` needs no real
   * flash-to-RAM copy at all — redefining it here, *after* `link.x` (so
   * this explicit `AT>` is the one that wins; unlike section *content*,
   * which accumulates in first-encountered order across multiple
   * same-named SECTIONS commands, the region/`AT>` assignment itself is
   * governed by whichever processed script's clause is explicit —
   * confirmed empirically after the `.rodata`-ordering trick alone did
   * *not* also fix this), keeps the LMA cursor in RWDATA the whole time,
   * so nothing after it (`.bss` included) inherits a bogus flash
   * address either. */
  .data : ALIGN(4) {
  } > RWDATA AT> RWDATA

  /* The `.data`/`AT>` override above did NOT actually change `_sidata`'s
   * computed value in practice (confirmed via `nm`: it stayed a flash
   * address even after adding that override) — `_sidata = LOADADDR(.data)`
   * in `link.x` evidently still resolves against `link.x`'s own `.data`
   * rule (the one holding the *real* content), not this empty
   * region-only redefinition. Worse: that flash address
   * (`_data_start + .data's size`, i.e. exactly the boundary of what
   * `espflash` actually mapped as this image's DROM segment) points
   * *one byte past* the end of the mapped/cached region — reading it
   * during `Reset`'s data-copy step is a real, confirmed fault (found on
   * hardware: silent crash-loop before any Rivet code ever prints,
   * `objdump`/`nm` narrowed it down after the trampoline `l32r` fix
   * didn't resolve it). Fixed directly: `_sidata` is a plain symbol
   * assignment (not a `SECTIONS`-block attribute), and plain assignments
   * follow ordinary last-processed-wins semantics — unlike the region/
   * `AT>` case above, this one actually works. Pointing it at
   * `_data_start` makes `Reset`'s copy read `.data`'s own *already-
   * correct* RAM bytes onto themselves — a harmless no-op, matching this
   * board's real design (no genuine flash-to-RAM copy needed at all). */
  _sidata = _data_start;

  /* Testing a hypothesis (plan.md Phase 22) — see memory.x. */
  .irom_test : ALIGN(4) {
    LONG(0)
  } > IROM

  /* Cooperative-task registry: `#[rivet::task]` discovers tasks by
   * walking this section. Empty in this phase's preemptive-only
   * examples, but `rivet::executor::init()` references these symbols
   * unconditionally, so the section must exist regardless. */
  .rivet_tasks : {
    __rivet_tasks_start = .;
    KEEP(*(.rivet_tasks));
    __rivet_tasks_end = .;
  } > RWDATA

  /* Preemptive task stacks: one contiguous, 16 KiB-aligned pool, matching
   * every other board's `.task_stacks`. Sized for the Phase 25 smoke
   * suite (stress_spawn/stress_max_ptasks fill the registry to
   * MAX_PTASKS, i.e. up to 16 concurrent 1 KiB stacks) with headroom;
   * RWDATA has ~172 KiB total and everything else in this crate's builds
   * uses well under half of it, so 64 KiB here is comfortable, not tight. */
  .task_stacks (NOLOAD) : ALIGN(16384) {
    __task_stacks_start = .;
    . += 64K;
    __task_stacks_end = .;
  } > RWDATA

  /* Boot stack for hart 0 — `xtensa-lx-rt`'s `Reset` reads
   * `_stack_start_cpu0` directly by this exact symbol name. */
  .stack (NOLOAD) : ALIGN(16) {
    . = ALIGN(16);
    _stack_end_cpu0 = .;
    . += 8K;
    . = ALIGN(16);
    _stack_start_cpu0 = .;
  } > RWDATA

  /* Boot stack for APP_CPU (plan.md Phase 24) — `rivet-arch-xtensa`'s
   * naked `rivet_appcpu_entry` reads `_appcpu_stack_top` directly by
   * this exact symbol name, mirroring `_stack_start_cpu0` above (both
   * are the *top* of a descending stack). No-op-sized cost on a
   * single-core-configured build (`RIVET_MAX_HARTS == 1`): the region
   * is reserved either way, but nothing ever runs on it if APP_CPU is
   * never released (`rivet-bsp-esp32s3::release_app_cpu` is a no-op in
   * that case). */
  .appcpu_stack (NOLOAD) : ALIGN(16) {
    . = ALIGN(16);
    _appcpu_stack_end = .;
    . += 8K;
    . = ALIGN(16);
    _appcpu_stack_top = .;
  } > RWDATA
}
