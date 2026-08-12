//! Stock NVIC interrupt controller driver — identical role to
//! `rivet-arch-cortex-m::nvic`'s own (see its module docs); duplicated for
//! the same separate-crate reason as `systick.rs`. NVIC is architected on
//! ARMv6-M too (same registers, same fixed addresses), just with fewer
//! IRQ lines and fewer implemented priority bits (2, not 8) — this driver
//! never assumes a specific count/width, so both differences are
//! transparent to it.

use cortex_m::peripheral::{scb::VectActive, NVIC, SCB};

/// Enable IRQ `n` (0-based, the "external interrupt" numbering — vector
/// table slot `16 + n`).
pub fn enable(n: u32) {
    // SAFETY: NVIC::PTR is the statically-known NVIC base; ISER is a
    // volatile MMIO set-enable register (write 1 to enable the
    // corresponding bit, writing 0 bits elsewhere is a no-op by hardware
    // design, so no read-modify-write race is possible even without a
    // critical section).
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

/// Set IRQ `n`'s priority (0 = highest). Unlike ARMv7-M (where `IPR` is
/// byte-addressable per IRQ), ARMv6-M's `cortex-m` crate models `IPR` as
/// word-addressed — four IRQs' priority bytes packed per 32-bit register
/// — so a single-IRQ write needs a read-modify-write at word granularity.
/// Only the top 2 bits of each byte are actually implemented in hardware
/// (4 priority levels), but writing the full byte (as every caller in
/// this workspace does — 0x00 or 0xFF) still lands on the right end of
/// the range regardless.
pub fn set_priority(n: u32, priority: u8) {
    // SAFETY: see `enable`; `ipr` is word-indexed, 4 IRQs per word.
    unsafe {
        let ipr = &(*NVIC::PTR).ipr[(n / 4) as usize];
        let shift = (n % 4) * 8;
        let mut word = ipr.read();
        word &= !(0xFFu32 << shift);
        word |= (priority as u32) << shift;
        ipr.write(word);
    }
}

/// Force IRQ `n` pending (software trigger, via NVIC's ISPR) — same
/// self-test use as `rivet-arch-cortex-m::nvic::pend`.
pub fn pend(n: u32) {
    // SAFETY: see `enable`; ISPR has the same write-1-to-set-bit shape.
    unsafe {
        (*NVIC::PTR).ispr[(n / 32) as usize].write(1 << (n % 32));
    }
}

/// Generic vector-table target for every external-interrupt slot — see
/// `rivet-arch-cortex-m::nvic`'s identical function for the full
/// rationale (`VectActive` + one shared trampoline for every IRQ number).
#[no_mangle]
extern "C" fn rivet_irq_handler() {
    if let VectActive::Interrupt { irqn } = SCB::vect_active() {
        rivet::irq::dispatch(irqn as u32);
    }
}
