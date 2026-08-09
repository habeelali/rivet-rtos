//! Stock NVIC interrupt controller driver (plan.md Phase 13).
//!
//! NVIC is architectural — same registers at the same fixed addresses on
//! every Cortex-M — so unlike RISC-V's CLINT/PLIC (platform-defined base
//! addresses, Group C constants from the board), this needs nothing from
//! the BSP beyond which IRQ *number* corresponds to which peripheral
//! (that mapping lives in each `rivet-bsp-*` crate's `irq` module).
//!
//! Raw register access (`NVIC::PTR`), not `cortex_m::peripheral::NVIC`'s
//! generic `Nr`-trait convenience methods — this crate has no PAC-
//! generated interrupt-number enum to hand it, and never will (the whole
//! point of Group A/B is not depending on a board-specific PAC).

use cortex_m::peripheral::{scb::VectActive, NVIC, SCB};

/// Enable IRQ `n` (0-based, the "external interrupt" numbering — vector
/// table slot `16 + n`).
pub fn enable(n: u32) {
    // SAFETY: NVIC::PTR is the statically-known NVIC base, identical on
    // every Cortex-M; ISER is a volatile MMIO set-enable register (write
    // 1 to enable the corresponding bit, writing 0 bits elsewhere is a
    // no-op by hardware design, so no read-modify-write race is possible
    // even without a critical section).
    unsafe {
        (*NVIC::PTR).iser[(n / 32) as usize].write(1 << (n % 32));
    }
}

/// Disable IRQ `n`.
pub fn disable(n: u32) {
    // SAFETY: see `enable` (ICER has the same write-1-to-clear-bit shape).
    unsafe {
        (*NVIC::PTR).icer[(n / 32) as usize].write(1 << (n % 32));
    }
}

/// Set IRQ `n`'s priority (0 = highest). NVIC priority registers are
/// byte-addressable per IRQ on ARMv7-M, so this is a single non-atomic
/// byte write — no read-modify-write hazard.
pub fn set_priority(n: u32, priority: u8) {
    // SAFETY: see `enable`; `ipr` is byte-indexed per IRQ.
    unsafe {
        (*NVIC::PTR).ipr[n as usize].write(priority);
    }
}

/// Force IRQ `n` pending (software trigger, via NVIC's ISPR). Used by the
/// `irq_test` example to exercise the real hardware dispatch path
/// (vector table → NVIC → `rivet_irq_handler` → `rivet::irq::dispatch`)
/// without needing an external device to assert a physical interrupt
/// line — a standard, legitimate NVIC self-test technique (ARMv7-M ISPR
/// is specified for exactly this).
pub fn pend(n: u32) {
    // SAFETY: see `enable`; ISPR has the same write-1-to-set-bit shape.
    unsafe {
        (*NVIC::PTR).ispr[(n / 32) as usize].write(1 << (n % 32));
    }
}

/// Generic vector-table target for every external-interrupt slot: reads
/// which IRQ is actually active from `SCB.ICSR.VECTACTIVE` and dispatches
/// through `rivet::irq`. One shared symbol works for every IRQ number —
/// the vector table entries are all `LONG(rivet_irq_handler)` — because
/// `VectActive` is exactly "which vector actually fired", the same
/// information a per-IRQ-numbered handler function's identity would
/// otherwise encode.
#[no_mangle]
extern "C" fn rivet_irq_handler() {
    if let VectActive::Interrupt { irqn } = SCB::vect_active() {
        rivet::irq::dispatch(irqn as u32);
    }
}
