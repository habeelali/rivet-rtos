//! PMP guard bands (stack-overflow detection on RV32 M-mode).
//!
//! RISC-V PMP entries with `L=1` are enforced against M-mode and immutable
//! until reset — so isolation is boot-time-static: each task stack's
//! guard band is denied by a locked entry programmed when the stack is
//! allocated, and entry 15 is a locked TOR catch-all that explicitly
//! allows everything above the last guard. Lower indices win, so guards
//! take precedence over the catch-all. Overflow past a stack's low end
//! faults (mcause 5/7); the kernel's own access to stacks is unaffected
//! (only the guard band itself is denied).
//!
//! Pure ISA — no board/MMIO knowledge.
//!
//! # Guard size is probed, not a fixed 64 bytes (plan.md Phase 26)
//!
//! The guard band's minimum size is `2^(G+2)` bytes, where `G` is this
//! hardware's PMP *grain* (RISC-V privileged spec) — `G == 0` on every
//! board this module was originally written against (QEMU virt,
//! MPS2-AN385), giving the historical hardcoded 64 bytes, but the
//! ESP32-C6 turned out to have `G > 0`: writing all-1s to a PMP address
//! register and reading it back did not return all-1s, the spec's own
//! recommended probe for a nonzero grain. [`probed_guard_size`] runs the
//! probe once, at [`init_catch_all`] time (using entry 0 — guaranteed
//! unlocked, since `__rivet_arch_init` always runs before any task stack
//! is ever allocated), and caches the result; `rivet::preempt::stack_pool`
//! queries it through [`crate::min_guard_size`] (the port contract) to
//! decide how many bytes to actually reserve below each stack, so the
//! reservation and what [`register_guard`] denies always agree by
//! construction rather than by two separately-hardcoded constants
//! agreeing by luck.
//!
//! # Known SMP limitation (plan.md Phase 19)
//!
//! PMP CSRs are genuinely per-hart hardware (like `MSIP`, unlike shared
//! `mtime`) — [`init_catch_all`] runs on every hart via
//! `__rivet_arch_init` (hart 0 through `rivet::init()`, secondary harts
//! through `rivet::run_secondary_hart()`), so the catch-all is correctly
//! present everywhere. [`register_guard`] is different: it's called once,
//! lazily, from `stack_pool::alloc_stack` on whichever hart happens to be
//! running when a given pool slot is *first* used — under Phase 19's
//! global run queue (any hart can dispatch any ready task), a task's
//! stack could later run on a *different* hart, whose PMP never got that
//! guard entry. That hart would not fault on overflow into that specific
//! guard band (the software watermark check in `preempt::on_tick_locked`
//! still catches it — one tick later, not synchronously at the write).
//! Closing this fully means replicating every guard to every hart (a PMP
//! shootdown protocol, IPI-driven, analogous to a TLB shootdown) — judged
//! out of scope for this phase relative to its cost; documented here
//! rather than silently shipped. Not a regression for the common case of
//! `MAX_HARTS == 1`, where "whichever hart" is always hart 0.

const NAPOT_GUARD_CFG: u8 = 0x98; // L | NAPOT | no RWX
const TOR_ALLOW_CFG: u8 = 0x8F; // L | TOR | RWX

/// `2^6 = 64` bytes — the desired/historical guard size, expressed as
/// trailing one-bits in the NAPOT encoding (`n - 3` for a `2^n`-byte
/// region). Still exactly what gets programmed on every `G == 0` board.
const DESIRED_GUARD_ONE_BITS: u32 = 3;

/// Probed PMP grain (`G`), cached after the first call. `u32::MAX` means
/// "not yet probed".
static GRAIN: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(u32::MAX);

/// Probe this hardware's actual PMP grain using entry 0, if not already
/// cached. Only safe to call before entry 0 is ever locked — true at
/// [`init_catch_all`] time (boot, before any stack allocation) and
/// nowhere else, which is why this is `pub(crate)` and only called from
/// there, never from [`register_guard`] itself.
fn probe_grain_using_entry0() -> u32 {
    let cached = GRAIN.load(core::sync::atomic::Ordering::Relaxed);
    if cached != u32::MAX {
        return cached;
    }
    use riscv::register::pmpaddr0;
    pmpaddr0::write(0xFFFF_FFFF);
    let read = pmpaddr0::read() as u32;
    // Per the privileged spec's own recommended probe: the number of
    // trailing one-bits in the read-back value is `G - 1` (`G == 0`
    // reads back all-ones, i.e. `trailing_ones() == 32`, clamped below).
    let trailing_ones = read.trailing_ones();
    let g = if trailing_ones >= 32 { 0 } else { trailing_ones + 1 };
    // Restore entry 0 to a harmless value (0): the probe write must not
    // leave a stray, unlocked-but-nonzero address sitting in a PMP entry
    // `register_guard` will overwrite properly later anyway, but leaving
    // it at 0 makes the intermediate state obviously inert if anything
    // ever reads it before that.
    pmpaddr0::write(0);
    GRAIN.store(g, core::sync::atomic::Ordering::Relaxed);
    g
}

/// The guard band size this hardware can actually encode, in bytes —
/// `64` unless a nonzero probed grain forces it larger. See the module
/// docs for why this must run (and be cached) before any real guard is
/// registered.
pub(crate) fn probed_guard_size() -> usize {
    let g = GRAIN.load(core::sync::atomic::Ordering::Relaxed);
    debug_assert!(
        g != u32::MAX,
        "rivet-arch-riscv::pmp: probed_guard_size() called before init_catch_all()"
    );
    1usize << (DESIRED_GUARD_ONE_BITS.max(g.saturating_sub(1)) + 3)
}

/// Program the guard for stack allocation `entry` (0-14): a locked NAPOT
/// entry denying the guard band below the stack — [`probed_guard_size`]
/// bytes, not unconditionally 64: on `G == 0` hardware (every board this
/// was originally written against) the two are identical; the ESP32-C6
/// (plan.md Phase 26) is the first board where they differ.
pub fn register_guard(guard_base: usize, entry: usize) {
    let g = GRAIN.load(core::sync::atomic::Ordering::Relaxed);
    debug_assert!(
        g != u32::MAX,
        "rivet-arch-riscv::pmp: register_guard() called before init_catch_all()"
    );
    // NAPOT low-bits pattern: `max(desired, grain-forced)` one-bits below
    // the address. `guard_base` itself must already be aligned to the
    // resulting size — `rivet::preempt::stack_pool` guarantees this by
    // reserving `probed_guard_size()` bytes (via `min_guard_size`), not a
    // bare 64, before computing `guard_base`.
    let one_bits = DESIRED_GUARD_ONE_BITS.max(g.saturating_sub(1));
    let pmpaddr = (guard_base >> 2) | ((1usize << one_bits) - 1);
    // Write the ADDRESS first, then the config byte: the config write
    // (with L=1) LOCKS the entry, and QEMU rejects (and logs a guest
    // error for) any pmpaddr write to an already-locked entry.
    match entry {
        0 => riscv::register::pmpaddr0::write(pmpaddr),
        1 => riscv::register::pmpaddr1::write(pmpaddr),
        2 => riscv::register::pmpaddr2::write(pmpaddr),
        3 => riscv::register::pmpaddr3::write(pmpaddr),
        4 => riscv::register::pmpaddr4::write(pmpaddr),
        5 => riscv::register::pmpaddr5::write(pmpaddr),
        6 => riscv::register::pmpaddr6::write(pmpaddr),
        7 => riscv::register::pmpaddr7::write(pmpaddr),
        8 => riscv::register::pmpaddr8::write(pmpaddr),
        9 => riscv::register::pmpaddr9::write(pmpaddr),
        10 => riscv::register::pmpaddr10::write(pmpaddr),
        11 => riscv::register::pmpaddr11::write(pmpaddr),
        12 => riscv::register::pmpaddr12::write(pmpaddr),
        13 => riscv::register::pmpaddr13::write(pmpaddr),
        14 => riscv::register::pmpaddr14::write(pmpaddr),
        _ => return, // beyond the PMP budget — watermark fallback
    }
    // Now lock the entry (L=1 | NAPOT | no access).
    match entry {
        0 => pmpcfg_write_byte(0, NAPOT_GUARD_CFG),
        1 => pmpcfg_write_byte(1, NAPOT_GUARD_CFG),
        2 => pmpcfg_write_byte(2, NAPOT_GUARD_CFG),
        3 => pmpcfg_write_byte(3, NAPOT_GUARD_CFG),
        4 => pmpcfg_write_byte(4, NAPOT_GUARD_CFG),
        5 => pmpcfg_write_byte(5, NAPOT_GUARD_CFG),
        6 => pmpcfg_write_byte(6, NAPOT_GUARD_CFG),
        7 => pmpcfg_write_byte(7, NAPOT_GUARD_CFG),
        8 => pmpcfg_write_byte(8, NAPOT_GUARD_CFG),
        9 => pmpcfg_write_byte(9, NAPOT_GUARD_CFG),
        10 => pmpcfg_write_byte(10, NAPOT_GUARD_CFG),
        11 => pmpcfg_write_byte(11, NAPOT_GUARD_CFG),
        12 => pmpcfg_write_byte(12, NAPOT_GUARD_CFG),
        13 => pmpcfg_write_byte(13, NAPOT_GUARD_CFG),
        14 => pmpcfg_write_byte(14, NAPOT_GUARD_CFG),
        _ => {}
    }
}

/// Set the 8-bit config byte for PMP entry `i` in the right pmpcfg register.
fn pmpcfg_write_byte(i: usize, byte: u8) {
    use riscv::register::pmpcfg0;
    let shift = (i % 4) * 8;
    let mask = 0xFFusize << shift;
    let value = (byte as usize) << shift;
    match i / 4 {
        0 => pmpcfg0::write((pmpcfg0::read().bits & !mask) | value),
        1 => {
            riscv::register::pmpcfg1::write((riscv::register::pmpcfg1::read().bits & !mask) | value)
        }
        2 => {
            riscv::register::pmpcfg2::write((riscv::register::pmpcfg2::read().bits & !mask) | value)
        }
        3 => {
            riscv::register::pmpcfg3::write((riscv::register::pmpcfg3::read().bits & !mask) | value)
        }
        _ => {}
    }
}

/// Locked catch-all allow for M-mode: everything above the last guard is
/// explicitly permitted. Called once at boot, on every hart, before any
/// task stack is ever allocated — also where the PMP grain gets probed
/// (see the module docs), since entry 0 is guaranteed unlocked here.
pub(crate) fn init_catch_all() {
    probe_grain_using_entry0();
    use riscv::register::pmpaddr15;
    // Address first, then the locking config (writing pmpaddr to a locked
    // entry is rejected by hardware/QEMU).
    // pmpaddr15 = 0xFFFFFFFF makes entry 15's TOR range end at the top of
    // the address space (safe CSR write).
    pmpaddr15::write(0xFFFF_FFFF);
    // L=1 freezes the entry until reset.
    pmpcfg_write_byte(15, TOR_ALLOW_CFG);
}
