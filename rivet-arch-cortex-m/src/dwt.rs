//! DWT `CYCCNT`-based cycle counter (plan.md Phase 10).
//!
//! `CYCCNT` is architecturally optional (the `NOCYCCNT` bit in
//! `DWT_CTRL` is implementation-defined) even though every real
//! Cortex-M3/4/7 core ships it in practice. Rather than assume that,
//! [`init`] runs a genuine probe — enable, write a known value, confirm it
//! actually counts — and [`enabled`] records the result so
//! [`cycle_count`] can fall back to [`systick::now_micros`] (coarser, but
//! still monotonic, which is all `__rivet_arch_cycle_count`'s contract
//! requires) on a core where the probe fails.

use core::sync::atomic::{AtomicBool, Ordering};
use cortex_m::peripheral::{DCB, DWT};

static DWT_USABLE: AtomicBool = AtomicBool::new(false);

/// Enable the DWT cycle counter and confirm it actually advances.
/// Idempotent; safe to call once from `__rivet_arch_init`.
pub fn init() {
    // Raw PTR access (consistent with the rest of this crate, which never
    // holds a `Peripherals` singleton) rather than `Peripherals::take()`.
    // SAFETY: DCB/DWT are the statically-known ARMv7-M debug peripheral
    // addresses, present as MMIO on every Cortex-M3/4/7 whether or not a
    // debugger is attached; this module exclusively owns the
    // cycle-counter subset of their registers (other bits are untouched).
    unsafe {
        (*DCB::PTR).demcr.modify(|w| w | (1 << 24)); // TRCENA
        (*DWT::PTR).lar.write(0xC5AC_CE55); // unlock (no-op on cores without a lock register)
        (*DWT::PTR).cyccnt.write(0);
        (*DWT::PTR).ctrl.modify(|w| w | 1); // CYCCNTENA
    }
    // Probe: the counter must have advanced past zero after a handful of
    // instructions. If `NOCYCCNT` is set, `ctrl`'s CYCCNTENA bit itself
    // reads back as unwritable-to-1 on some cores; checking the counter's
    // actual movement (rather than trusting the control bit) catches both
    // cases with one test.
    for _ in 0..8 {
        cortex_m::asm::nop();
    }
    let advanced = DWT::cycle_count() != 0;
    DWT_USABLE.store(advanced, Ordering::Release);
}

pub fn cycle_count() -> u64 {
    if DWT_USABLE.load(Ordering::Acquire) {
        return DWT::cycle_count() as u64;
    }
    // Fallback: microsecond-resolution but still monotonic — callers only
    // ever take deltas (see `__rivet_arch_cycle_count`'s contract), so
    // this degrades precision, not correctness. Only available with the
    // `systick` feature; without either source, 0 is returned (still
    // "monotonic", trivially — a board with neither DWT nor SysTick has
    // no cycle-adjacent source at all to report).
    #[cfg(feature = "systick")]
    {
        super::systick::now_micros()
    }
    #[cfg(not(feature = "systick"))]
    {
        0
    }
}
