//! Stock SiFive PLIC (Platform-Level Interrupt Controller) driver
//! (plan.md Phase 13).
//!
//! Same Group C shape as [`crate::clint`]: the mechanism (priority/
//! enable/threshold/claim-complete register layout) is universal across
//! PLIC-equipped RV32 platforms, but the base address is per-board —
//! call [`configure`] once from `__rivet_board_init`, before enabling any
//! IRQ. Verified against QEMU's `virt` machine empirically (`info mtree`:
//! `riscv.sifive.plic` at `0x0c00_0000`), not assumed.
//!
//! Only the M-mode context is used (context 0 = hart 0's M-mode context —
//! QEMU's `virt` machine always allocates one PLIC context per hart per
//! privilege mode it supports, and this kernel runs entirely in M-mode,
//! never touching S-mode). Multi-hart support (context `2 * hart_id`) is
//! plan.md Phase 19's concern, not this one.

use core::sync::atomic::{AtomicUsize, Ordering};

static BASE: AtomicUsize = AtomicUsize::new(0);

/// M-mode context for hart 0. See module docs.
const CONTEXT: usize = 0;

const PRIORITY_OFFSET: usize = 0x0000; // + 4 * source
// Pending-bits register (base + 0x1000, read-only, one bit per source) is
// part of the PLIC spec but unused by this driver — claim/complete
// already tells us exactly which source needs handling.
const ENABLE_OFFSET: usize = 0x2000; // + context * 0x80 + 4 * (source / 32)
const CONTEXT_BASE_OFFSET: usize = 0x20_0000; // + context * 0x1000
const THRESHOLD_SUB_OFFSET: usize = 0x0000;
const CLAIM_COMPLETE_SUB_OFFSET: usize = 0x0004;

/// Tell the driver where this board's PLIC lives. Must be called before
/// any other function in this module, and before unmasking `mie.MEIE`.
pub fn configure(base: usize) {
    BASE.store(base, Ordering::Relaxed);
    // SAFETY: `base` is caller-supplied (the board's own memory map);
    // this only runs once at board init before any interrupt can fire —
    // set the M-mode context's priority threshold to 0 (accept every
    // configured priority >= 1) so per-source `enable` below is the only
    // gate that matters.
    unsafe {
        threshold_reg().write_volatile(0);
    }
    // Global gate: without `mie.MEIE`, no PLIC-claimed interrupt ever
    // reaches the core, regardless of per-source enable bits below.
    // SAFETY: setting MEIE is safe in M-mode at any point before
    // interrupts are globally enabled (mstatus.MIE) — which they aren't
    // yet this early in board init.
    unsafe {
        riscv::register::mie::set_mext();
    }
}

fn base() -> usize {
    let b = BASE.load(Ordering::Relaxed);
    debug_assert!(b != 0, "rivet-arch-riscv::plic::configure was never called");
    b
}

fn priority_reg(source: u32) -> *mut u32 {
    (base() + PRIORITY_OFFSET + 4 * source as usize) as *mut u32
}

fn enable_reg(source: u32) -> *mut u32 {
    (base() + ENABLE_OFFSET + CONTEXT * 0x80 + 4 * (source as usize / 32)) as *mut u32
}

fn threshold_reg() -> *mut u32 {
    (base() + CONTEXT_BASE_OFFSET + CONTEXT * 0x1000 + THRESHOLD_SUB_OFFSET) as *mut u32
}

fn claim_complete_reg() -> *mut u32 {
    (base() + CONTEXT_BASE_OFFSET + CONTEXT * 0x1000 + CLAIM_COMPLETE_SUB_OFFSET) as *mut u32
}

/// Enable PLIC source `irq_num` for the M-mode context, at priority 1
/// (any non-zero priority; this driver doesn't expose per-source
/// priority tuning beyond "on", matching [`set_priority`]'s simplified
/// contract below — source priority and context threshold interact, and
/// most bare-metal PLIC users never need more than one active level).
pub fn enable(irq_num: u32) {
    // SAFETY: `irq_num` indexes into the PLIC's own source-priority array
    // (out-of-range writes land in reserved-but-mapped PLIC address space
    // per the SiFive PLIC spec, not adjacent unrelated memory); volatile
    // MMIO writes, single owner (no concurrent access from this driver).
    unsafe {
        priority_reg(irq_num).write_volatile(1);
        let reg = enable_reg(irq_num);
        let bit = 1u32 << (irq_num % 32);
        reg.write_volatile(reg.read_volatile() | bit);
    }
}

pub fn disable(irq_num: u32) {
    // SAFETY: see `enable`.
    unsafe {
        let reg = enable_reg(irq_num);
        let bit = 1u32 << (irq_num % 32);
        reg.write_volatile(reg.read_volatile() & !bit);
    }
}

/// Set source `irq_num`'s priority (1 = lowest active, up to the PLIC's
/// implemented width — QEMU's `virt` PLIC implements 3 priority bits, so
/// values above 7 are truncated by hardware). `0` would disable it
/// (PLIC semantics: priority 0 never claims) — use [`disable`] instead if
/// that's the intent, so the two operations stay distinct in the log.
pub fn set_priority(irq_num: u32, priority: u8) {
    // SAFETY: see `enable`.
    unsafe {
        priority_reg(irq_num).write_volatile(priority as u32);
    }
}

/// Called from the trap handler on a machine-external-interrupt
/// (`mcause` code 11): claim the highest-priority pending source, dispatch
/// it through `rivet::irq`, then signal completion. The claim/complete
/// protocol is mandatory — a source that's claimed but never completed
/// never re-fires even after the physical condition recurs.
pub fn claim_dispatch_complete() {
    // SAFETY: see `enable`; claim (read) and complete (write) are the
    // PLIC's own documented protocol for this exact register.
    let source = unsafe { claim_complete_reg().read_volatile() };
    if source == 0 {
        // Spurious claim (nothing pending) — the PLIC spec explicitly
        // allows this; nothing to complete.
        return;
    }
    rivet::irq::dispatch(source);
    // SAFETY: see `enable`.
    unsafe {
        claim_complete_reg().write_volatile(source);
    }
}
