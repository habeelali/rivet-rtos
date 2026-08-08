//! Critical section abstraction.

#[cfg(target_arch = "riscv32")]
mod imp {
    pub fn enter<R>(f: impl FnOnce() -> R) -> R {
        use riscv::register::mstatus;
        // SAFETY: this is the kernel's critical-section primitive — the
        // closure must run with interrupts (mstatus.MIE) disabled.
        // Deliberately save/restore *only* the MIE bit, never the whole
        // mstatus: restoring all of mstatus would also restore MPP/MPIE
        // as captured at entry, undoing any privilege-mode changes made
        // inside the closure (plan.md [B16]). Nested `enter` works:
        // an inner call observes MIE=0 and re-disables (no-op), then
        // restores it to 0, leaving the outer call to re-enable.
        unsafe {
            let was_enabled = mstatus::read().mie();
            mstatus::clear_mie();
            let r = f();
            if was_enabled {
                mstatus::set_mie();
            }
            r
        }
    }
}

#[cfg(target_arch = "arm")]
mod imp {
    pub fn enter<R>(f: impl FnOnce() -> R) -> R {
        cortex_m::interrupt::free(|_| f())
    }
}

#[cfg(not(any(target_arch = "riscv32", target_arch = "arm")))]
mod imp {
    pub fn enter<R>(f: impl FnOnce() -> R) -> R {
        f()
    }
}

/// Run a closure with interrupts disabled.
#[inline]
pub fn enter<F, R>(f: F) -> R
where
    F: FnOnce() -> R,
{
    imp::enter(f)
}
