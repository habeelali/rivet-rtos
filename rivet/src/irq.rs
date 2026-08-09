//! IRQ dispatch (plan.md Phase 13).
//!
//! A fixed-size table (`RIVET_MAX_IRQS` slots) of registered handlers,
//! populated at init and walked by whichever `rivet-arch-*` controller
//! driver the board enables (`rivet-arch-cortex-m/nvic`'s
//! `rivet_irq_handler`, `rivet-arch-riscv/plic`'s claim/dispatch/complete
//! loop). The split follows the same Group A/B logic as everything else
//! in this kernel: **the controller** (NVIC, PLIC) **is arch** — every
//! board on that ISA shares the same interrupt-controller hardware — but
//! **the IRQ number** (which number is UART0, which is the GPIO block) is
//! entirely board-specific, so it lives in each `rivet-bsp-*` crate's own
//! `irq` module as plain constants, never here.
//!
//! [`register`] stores a plain function pointer, not a closure — IRQ
//! handlers run on the arch's ISR stack with no allocator and (on
//! Cortex-M) in Handler mode, so a `'static fn()` is the right shape: no
//! captured state beyond what a `static` can already hold.

use crate::sync::atomic::{AtomicUsize, Ordering};

pub const MAX_IRQS: usize = crate::config::MAX_IRQS;

#[cfg(not(loom))]
static HANDLERS: [AtomicUsize; MAX_IRQS] = [const { AtomicUsize::new(0) }; MAX_IRQS];
#[cfg(loom)]
loom::lazy_static! {
    static ref HANDLERS: [AtomicUsize; MAX_IRQS] = core::array::from_fn(|_| AtomicUsize::new(0));
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IrqError {
    /// `irq_num >= RIVET_MAX_IRQS`.
    OutOfRange,
}

/// Register `handler` for `irq_num`. Overwrites any previous registration
/// for the same number (the caller is expected to do this once at init,
/// before calling [`enable`]). Does **not** enable the interrupt at the
/// controller — call [`enable`] once the handler is registered, in that
/// order, so the controller never fires into an empty slot.
pub fn register(irq_num: u32, handler: fn()) -> Result<(), IrqError> {
    let slot = HANDLERS.get(irq_num as usize).ok_or(IrqError::OutOfRange)?;
    slot.store(handler as usize, Ordering::Release);
    Ok(())
}

/// Deregister `irq_num`'s handler (a spurious/unhandled interrupt after
/// this becomes a silent no-op in [`dispatch`], not a fault — matching
/// how a not-yet-registered slot behaves before the first [`register`]).
pub fn unregister(irq_num: u32) {
    if let Some(slot) = HANDLERS.get(irq_num as usize) {
        slot.store(0, Ordering::Release);
    }
}

/// Enable `irq_num` at the arch's interrupt controller
/// (`port::arch::irq_enable`).
pub fn enable(irq_num: u32) {
    crate::port::arch::irq_enable(irq_num);
}

/// Disable `irq_num` at the arch's interrupt controller.
pub fn disable(irq_num: u32) {
    crate::port::arch::irq_disable(irq_num);
}

/// Set `irq_num`'s controller priority (0 = highest; the controller's own
/// range/granularity — e.g. NVIC's 8-bit `ipr` byte — is arch-defined).
pub fn set_priority(irq_num: u32, priority: u8) {
    crate::port::arch::irq_set_priority(irq_num, priority);
}

/// Called by the arch controller driver with the IRQ number that just
/// fired. Looks up and calls the registered handler; a no-op if nothing
/// is registered for `irq_num` (a controller can only report interrupts
/// it was told to enable, so this path is only hit by a genuine
/// registration bug or race, not routine operation — silently ignoring it
/// is safer than panicking on the ISR stack).
pub fn dispatch(irq_num: u32) {
    let Some(slot) = HANDLERS.get(irq_num as usize) else {
        return;
    };
    let ptr = slot.load(Ordering::Acquire);
    if ptr == 0 {
        return;
    }
    // `ptr as *const ()` (not a direct `usize`-to-`fn()` transmute, which
    // Miri correctly flags as producing a provenance-less/dangling
    // pointer even though the bit pattern is identical) is the `as`
    // int-to-pointer cast that looks back up the provenance the `as
    // usize` cast in `register` exposed.
    let raw_ptr = ptr as *const ();
    // SAFETY: `ptr` was stored by `register` from a `fn()` value — the
    // only thing ever stored here — so `raw_ptr` points at exactly that
    // function; transmuting a pointer-to-pointer-width `fn()` changes
    // only the type, not the bits.
    let handler: fn() = unsafe { core::mem::transmute::<*const (), fn()>(raw_ptr) };
    handler();
}

#[cfg(feature = "test-support")]
pub(crate) fn reset_for_test() {
    for slot in HANDLERS.iter() {
        slot.store(0, Ordering::Relaxed);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::sync::atomic::{AtomicBool, Ordering as StdOrdering};

    static CALLED: AtomicBool = AtomicBool::new(false);
    fn handler() {
        CALLED.store(true, StdOrdering::Release);
    }

    #[test]
    fn register_and_dispatch() {
        crate::kernel_test! {
            CALLED.store(false, StdOrdering::Release);
            register(3, handler).unwrap();
            dispatch(3);
            assert!(CALLED.load(StdOrdering::Acquire));
        }
    }

    #[test]
    fn dispatch_unregistered_is_noop() {
        crate::kernel_test! {
            // Must not panic.
            dispatch(1);
        }
    }

    #[test]
    fn register_out_of_range() {
        crate::kernel_test! {
            assert_eq!(register(MAX_IRQS as u32, handler), Err(IrqError::OutOfRange));
        }
    }

    #[test]
    fn unregister_makes_dispatch_noop() {
        crate::kernel_test! {
            CALLED.store(false, StdOrdering::Release);
            register(5, handler).unwrap();
            unregister(5);
            dispatch(5);
            assert!(!CALLED.load(StdOrdering::Acquire));
        }
    }
}
