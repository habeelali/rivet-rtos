//! `embedded_hal_async::digital::Wait` on PC13/EXTI15_10 (embedded-hal-
//! plan.md Phase F) — the Nucleo-64's B1 user button line, chosen
//! because it is the one edge source on this workspace's boards that
//! can be driven end-to-end on real hardware with no human involved:
//! `EXTI_SWIER` (software interrupt event register) lets software pend
//! the exact same interrupt a real electrical edge would, so this
//! proves the whole edge -> ISR -> [`Signal`] -> `Wait` path on genuine
//! silicon without anyone pressing a button. No QEMU board in this
//! workspace models an externally-driven GPIO edge source headlessly,
//! which is why this is stm32f401re-only (see embedded-hal-plan.md).
//!
//! `EXTI_SWIER` sets the same pending bit a real edge would and is
//! independent of `RTSR`/`FTSR` (the trigger-direction selects), so it
//! exercises this exact code path regardless of which trigger a given
//! `wait_for_*` call armed.

use rivet::sync::Signal;

const EXTI_BASE: usize = 0x4001_3C00;
const EXTI_IMR: usize = 0x00;
const EXTI_RTSR: usize = 0x08;
const EXTI_FTSR: usize = 0x0C;
const EXTI_SWIER: usize = 0x10;
const EXTI_PR: usize = 0x14;

const GPIOC_BASE: usize = 0x4002_0800;
const GPIOC_IDR: usize = 0x10;

const LINE13: u32 = 1 << 13;

/// PC13, EXTI line 13 — construct via [`stm32_exti13_instance!`].
pub struct ExtiPc13 {
    sig: &'static Signal,
}

impl ExtiPc13 {
    /// # Safety
    /// `sig` must be the exact [`Signal`] the ISR the
    /// [`stm32_exti13_instance!`] caller registered for `EXTI15_10`
    /// calls [`Signal::signal`] on. Assumes `SYSCFG.EXTICR4` already
    /// routes line 13 to port C and `GPIOC`'s clock is enabled (board-
    /// owned pin muxing, matching every other peripheral in this
    /// workspace — see this crate's `i2c`/`gpio` modules).
    pub const unsafe fn new(sig: &'static Signal) -> Self {
        Self { sig }
    }

    fn reg(offset: usize) -> *mut u32 {
        (EXTI_BASE + offset) as *mut u32
    }

    fn set_triggers(&self, rising: bool, falling: bool) {
        // SAFETY: fixed EXTI register block, exclusively owned per the
        // constructor's contract.
        unsafe {
            let mut rtsr = core::ptr::read_volatile(Self::reg(EXTI_RTSR));
            rtsr = if rising { rtsr | LINE13 } else { rtsr & !LINE13 };
            core::ptr::write_volatile(Self::reg(EXTI_RTSR), rtsr);

            let mut ftsr = core::ptr::read_volatile(Self::reg(EXTI_FTSR));
            ftsr = if falling { ftsr | LINE13 } else { ftsr & !LINE13 };
            core::ptr::write_volatile(Self::reg(EXTI_FTSR), ftsr);

            let imr = core::ptr::read_volatile(Self::reg(EXTI_IMR));
            core::ptr::write_volatile(Self::reg(EXTI_IMR), imr | LINE13);
        }
    }

    fn is_high(&self) -> bool {
        // SAFETY: fixed GPIOC register block.
        unsafe {
            (core::ptr::read_volatile((GPIOC_BASE + GPIOC_IDR) as *const u32) >> 13) & 1 != 0
        }
    }

    async fn wait_for_edge(&mut self, rising: bool, falling: bool) {
        self.sig.reset();
        self.set_triggers(rising, falling);
        self.sig.wait().await;
    }

    /// Software-pend EXTI line 13 exactly as a real electrical edge
    /// would — see the module docs. Test-only entry point (not part of
    /// any embedded-hal trait), used by `stm32_wait_test.rs` to prove
    /// this path without a human pressing B1.
    pub fn trigger_software_interrupt(&self) {
        // SAFETY: fixed EXTI register block.
        unsafe { core::ptr::write_volatile(Self::reg(EXTI_SWIER), LINE13) };
    }

    /// Arm rising/falling detection and unmask line 13, synchronously,
    /// without waiting. Test-only (real `embedded_hal_async::digital::
    /// Wait` callers use the trait methods below, which arm and wait as
    /// one atomic-from-the-caller's-perspective step) — lets
    /// `stm32_wait_test.rs` arm *then* trigger, matching the ordering
    /// `signal_irq_test.rs` already proved on three other boards
    /// (Phase B), rather than triggering before arming has happened at
    /// all (which — same as a real edge arriving before `RTSR`/`IMR`
    /// are configured — is legitimately missed, not a bug).
    pub fn arm(&self, rising: bool, falling: bool) {
        self.sig.reset();
        self.set_triggers(rising, falling);
    }

    /// Await a signal already armed via [`ExtiPc13::arm`]. Test-only.
    pub async fn wait_armed(&self) {
        self.sig.wait().await;
    }
}

/// Shared ISR body for `EXTI15_10` — covers lines 10-15, but this board
/// only ever arms line 13. `PR` is write-1-to-clear, checked before
/// clearing so a spurious call (another armed line in 10-15, not used
/// today but worth being honest about) doesn't `signal()` on a
/// non-event.
pub fn isr(sig: &Signal) {
    // SAFETY: fixed EXTI register block, single owner per instance.
    unsafe {
        let pr = core::ptr::read_volatile((EXTI_BASE + EXTI_PR) as *const u32);
        if pr & LINE13 != 0 {
            core::ptr::write_volatile((EXTI_BASE + EXTI_PR) as *mut u32, LINE13);
            sig.signal();
        }
    }
}

/// Declares a `static Signal` and a named `fn()` ISR bound to it, for
/// PC13/EXTI13. See [`ExtiPc13::new`] for why this can't just be a
/// generic function with an inline static.
#[macro_export]
macro_rules! stm32_exti13_instance {
    ($sig_name:ident, $isr_name:ident) => {
        static $sig_name: ::rivet::sync::Signal = ::rivet::sync::Signal::new();

        fn $isr_name() {
            $crate::wait::isr(&$sig_name);
        }
    };
}

impl embedded_hal::digital::ErrorType for ExtiPc13 {
    type Error = core::convert::Infallible;
}

impl embedded_hal_async::digital::Wait for ExtiPc13 {
    async fn wait_for_high(&mut self) -> Result<(), Self::Error> {
        while !self.is_high() {
            self.wait_for_edge(true, false).await;
        }
        Ok(())
    }

    async fn wait_for_low(&mut self) -> Result<(), Self::Error> {
        while self.is_high() {
            self.wait_for_edge(false, true).await;
        }
        Ok(())
    }

    async fn wait_for_rising_edge(&mut self) -> Result<(), Self::Error> {
        self.wait_for_edge(true, false).await;
        Ok(())
    }

    async fn wait_for_falling_edge(&mut self) -> Result<(), Self::Error> {
        self.wait_for_edge(false, true).await;
        Ok(())
    }

    async fn wait_for_any_edge(&mut self) -> Result<(), Self::Error> {
        self.wait_for_edge(true, true).await;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[allow(dead_code)]
    async fn generic_wait<W: embedded_hal_async::digital::Wait>(w: &mut W) {
        let _ = w.wait_for_rising_edge().await;
    }

    #[test]
    fn type_checks() {
        let _ = generic_wait::<ExtiPc13>;
    }
}
