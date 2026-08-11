/* ESP32-C6 real hardware (plan.md Phase 26).
 *
 * Confirmed on hardware (not assumed) through several failed attempts —
 * `espflash`'s ELF-to-app-image conversion and the ESP-IDF bootloader's
 * loader have real requirements this workspace's other boards never
 * exposed a linker script to directly, since none of them ship through
 * this exact image format:
 *
 * 1. The bootloader's image loader requires *exactly two* ROM-mapped
 *    segments and refuses to boot a single merged one ("Assert failed in
 *    unpack_load_app... (rom_index == 2)").
 * 2. It reads the app descriptor (`esp_app_desc!()`, `.flash.appdesc`)
 *    starting at the *first* segment specifically — placing it anywhere
 *    else reproduces "Failed to fetch app description header!".
 * 3. Two `PT_LOAD` segments that both land in the *same* 0x42000000+
 *    flash-cache window (relying only on the `.text`/`.rodata` X-flag
 *    difference for two ELF program headers) get *coalesced back into
 *    one* by the bootloader's own contiguous-range mapping — a second
 *    segment genuinely needs to be a *different* flash-cache window: the
 *    ESP32-S3's I-bus/D-bus SRAM alias pattern, here applied to flash.
 *    IROM (0x42000000+, instruction-bus) and DROM (0x42800000+,
 *    data-bus) are both real aliases of the same physical flash, a fixed
 *    0x800000 apart.
 * 4. `espflash` reorders segments into [IROM, every RAM-loaded segment,
 *    DROM] in the final `.bin`, regardless of this script's declaration
 *    order or the ELF's program-header order (confirmed directly:
 *    `espflash save-image` + parsing the segment table by hand) — and
 *    packs them *tightly*, ignoring any deliberate LMA gap this script
 *    tries to insert (also confirmed directly: inserting padding changed
 *    nothing about where `espflash` actually placed the bytes). So
 *    rather than picking a fixed DROM origin and trying to force the
 *    flash-side position to match it — impossible, that position is
 *    `espflash`'s to decide — `.rodata`'s VMA is computed the other way
 *    around, from wherever its LMA naturally ends up: `vma = lma +
 *    0x800000` is correct for *whatever* that position turns out to be.
 * 5. That LMA itself needs one more correction this script's own ELF-
 *    level view can't see: `espflash`'s app-image format inserts an
 *    8-byte per-segment header between every pair of segments, which
 *    doesn't exist anywhere in the ELF's own contiguous LMA numbering.
 *    `.rodata` is the third segment (index 2, after `.text` and
 *    `.data`), so two such headers (16 bytes) have already been inserted
 *    ahead of it in the real flashed file by the time it's reached —
 *    confirmed by hand from a real "Segment 2 load address ..., doesn't
 *    match data ..." error: the two values differed by exactly 0x10.
 *    `SEGMENT_HEADER_OVERHEAD` below is that correction, generalized to
 *    "one 8-byte header per already-placed segment" so it stays correct
 *    if a future change adds or removes a segment before `.rodata`.
 */
OUTPUT_ARCH(riscv)
ENTRY(_start)

/* Fixed distance between the two flash-cache alias windows. */
DROM_ALIAS_OFFSET = 0x800000;
/* Per point 5: two segments (.text, .data) precede .rodata in the real
 * flashed image, each with an 8-byte header the ELF itself never sees. */
SEGMENT_HEADER_OVERHEAD = 2 * 8;

MEMORY
{
  IROM (rx) : ORIGIN = 0x42000000 + 0x20, LENGTH = 0x400000 - 0x20
  /* Wide enough to cover any address `.rodata`'s computed VMA (IROM-
   * relative LMA + `DROM_ALIAS_OFFSET`) could land at — used only for
   * bounds-checking; the actual address is given explicitly below. */
  DROM (r)  : ORIGIN = 0x42800000, LENGTH = 0x1000000
  RAM  (rwx): ORIGIN = 0x40800000, LENGTH = 0x6E610
}

SECTIONS
{
  /* `KEEP`: nothing else references `.flash.appdesc`, so `--gc-sections`
   * would otherwise drop it. Must stay first — see point 2 above. */
  .text : {
    KEEP(*(.flash.appdesc))
    *(.text._start)
    *(.text .text.*)
  } > IROM

  .data : {
    __data_start = .;
    *(.data .data.*)
    __data_end = .;
  } > RAM AT > IROM

  /* `rivet-rt`'s shared `_start` always does a `.data` copy from
   * `__data_load`, needed on boards where the app itself is the only
   * thing that ever places `.data`'s initial content in RAM. Confirmed
   * on real hardware that this board is *not* such a board: the boot log
   * prints `segment 1: ... load` (not `map`) for the RAM-targeted
   * segment — the ESP-IDF bootloader itself already reads it via SPI
   * flash (not constrained by the CPU's own I-bus/D-bus cache windows)
   * and writes the correct bytes to `__data_start` directly, before
   * `_start` ever runs. `_start`'s own subsequent copy is therefore not
   * just redundant but actively harmful: reading `.data`'s content back
   * through *either* flash-cache window (I-bus/IROM or its D-bus/DROM
   * alias — both tried) via ordinary `lw` returned garbage rather than
   * faulting, silently overwriting the bootloader's already-correct
   * work (root-caused via `rivet::critical`'s `LOCK_OWNER: AtomicIsize
   * = AtomicIsize::new(-1)`: came up as whatever was already sitting in
   * RAM instead of -1, hanging the very first `critical::enter` call's
   * CAS loop forever). Matching `rivet-bsp-qemu-virt`'s own fix for the
   * same underlying need (a board where the loader already places
   * `.data` correctly): make the copy a harmless self-copy. */
  __data_load = __data_start;

  /* `rust-lld` may warn "address of section .rodata is not a multiple of
   * alignment (8)" on some binaries (whichever ones happen to need an
   * 8-byte-aligned static, e.g. a `u64`/`f64` constant) — the LMA above
   * is derived from wherever `.text`+`.data` happen to end, with no
   * alignment guarantee, and deliberately isn't rounded up to fix that:
   * point 4 above already found the hard way that `espflash` ignores any
   * gap this script inserts and packs segments tightly regardless, so
   * rounding up here would just make the ELF's own numbering diverge
   * from the real flashed image again — reproducing the exact "load
   * address doesn't match data" failure this LMA formula exists to
   * avoid. In practice this hasn't caused an observed fault (RV32I
   * tolerates unaligned loads/stores; verified on real hardware across
   * multiple binaries including one that trips this warning), so it's
   * left as a known, harmless warning rather than "fixed" in a way that
   * would trade a benign warning for a real, hard-to-reproduce bug. */
  .rodata (LOADADDR(.data) + SIZEOF(.data) + SEGMENT_HEADER_OVERHEAD + DROM_ALIAS_OFFSET) :
    AT(LOADADDR(.data) + SIZEOF(.data) + SEGMENT_HEADER_OVERHEAD)
  {
    *(.rodata .rodata.*)
    /* Auto-generated by the `esp32c6` PAC's own linker fragments, not
     * something this script explicitly requests — left unrouted, these
     * become orphan sections LD places by its own default rules,
     * potentially splitting into a *third* ROM segment (reproducing the
     * `rom_index == 2` failure from the other direction). */
    *(.espressif.metadata)
    KEEP(*(.eh_frame))
    /* Task registry (`rivet::task::TaskReg` — priority, two function
     * pointers, and a raw pointer to the task's `TaskCell`, itself
     * separately `.bss`-resident): every field is set at compile time
     * and never mutated, so this belongs in flash-mapped DROM next to
     * every other constant, not RAM. Confirmed the hard way: giving it
     * its own `> RAM` output section with no `AT()` (matching every
     * other board, all of which have one flat memory region so
     * VMA==LMA is automatic) inherited a bogus LMA on this board's
     * split flash/RAM memory model — the ESP-IDF bootloader rejected
     * the resulting image outright ("Segment 3 load address ... doesn't
     * match data ..."), only reproduced once a real `#[rivet::task]`
     * user (`sleep_test.rs`) gave the section actual content; every
     * earlier C6 binary only used `spawn_ptask!`, so this section was
     * always empty and the bug stayed latent. Living here instead
     * means no separate segment, no LMA arithmetic, and no runtime copy
     * needed — it was never going to be written to. */
    __rivet_tasks_start = .;
    KEEP(*(.rivet_tasks));
    __rivet_tasks_end = .;
  } > DROM

  .bss (NOLOAD) : {
    __bss_start = .;
    *(.bss .bss.*)
    *(COMMON)
    __bss_end = .;
  } > RAM

  /* Boot stack for hart 0 (single-core chip — no secondary-hart stacks
   * needed, unlike qemu-virt's -smp support). */
  .stack (NOLOAD) : ALIGN(16) {
    __stack_bottom = .;
    . += 8K;
    __stack_top = .;
  } > RAM

  /* Dedicated interrupt/trap-handler stack — same role as every other
   * RISC-V board's (see rivet-bsp-qemu-virt's own comment): the trap
   * entry keeps the 128-byte frame on the interrupted task's own stack
   * but runs the Rust handler on this one via mscratch. */
  .isr_stack (NOLOAD) : ALIGN(16) {
    __isr_stack_bottom = .;
    . += 2K;
    __isr_stack_top = .;
  } > RAM

  /* `rivet-rt`'s shared RISC-V `_start` always references
   * `__secondary_stacks_top` (indexed by `mhartid` for a secondary-hart
   * boot path) even on a single-core board where `mhartid` is always 0
   * and that path never actually runs — the symbol still has to resolve
   * for the assembler's `la` pseudo-instruction. One minimal slice. */
  .secondary_stacks (NOLOAD) : ALIGN(16) {
    __secondary_stacks_bottom = .;
    . += 16;
    __secondary_stacks_top = .;
  } > RAM

  /* Preemptive task stacks: one contiguous, 16 KiB-aligned pool (plan.md
   * §3), so the RISC-V PMP can guard each stack's low end. */
  .task_stacks (NOLOAD) : ALIGN(16384) {
    __task_stacks_start = .;
    . += 32K;
    __task_stacks_end = .;
  } > RAM

  _end = .;
}
