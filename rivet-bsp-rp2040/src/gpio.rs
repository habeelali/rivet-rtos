//! Typestate GPIO for the RP2040 (dual Cortex-M0+), following
//! `rivet-bsp-lm3s6965::gpio`'s exact pattern.
//!
//! Three registers cooperate per pin on RP2040, unlike the single-block
//! GPIO peripherals on the other three boards: `IO_BANK0` selects the
//! pin's function (`FUNCSEL = 5` routes it to `SIO`, the plain-GPIO
//! path), `PADS_BANK0` controls the pad's electrical behavior (input
//! enable / output disable), and `SIO` itself holds the direction/value
//! registers `set_high`/`is_high`/etc. actually touch. All three base
//! addresses and register offsets here were verified directly against
//! the `rp2040-pac` crate's generated source (`RegisterBlock` field
//! offsets and each block's `PTR` constant), not the datasheet alone —
//! this workspace has no RP2040 physically connected this session, so
//! this module is `cargo check`-verified against the real target but
//! **not yet flashed and observed on real hardware**, unlike the other
//! three boards' GPIO modules in this same phase. It mirrors the exact
//! sequence `rivet_bsp_rp2040::__rivet_board_init` already uses for the
//! LED pin (`gpio_ctrl().write(|w| w.funcsel().sio())`,
//! `pads_bank0.gpio(25).modify(|_, w| w.od().clear_bit().ie().clear_bit())`,
//! `sio.gpio_oe_set()`), which *is* hardware-proven — just through the
//! PAC's typed API instead of raw offsets (a const-generic `Pin<N, MODE>`
//! needs a real memory address, not a PAC singleton type).
//!
//! Doesn't touch `RESETS`: unlike STM32's per-port `AHB1ENR` gating,
//! RP2040's `IO_BANK0`/`PADS_BANK0` are single blocks covering every
//! pin, and `rivet_bsp_rp2040`'s own board init already releases both
//! unconditionally before any application code runs — there's no
//! per-pin reset state this module needs to manage.

use core::marker::PhantomData;

/// `IO_BANK0` — pin function select. One base for all 30 GPIOs.
const IO_BANK0: usize = 0x4001_4000;
/// `PADS_BANK0` — pad electrical config. One base for all 30 GPIOs.
const PADS_BANK0: usize = 0x4001_c000;
/// `SIO` — the direction/value registers "being a plain GPIO" means.
/// One base, not per-pin.
const SIO: usize = 0xd000_0000;

const SIO_GPIO_IN: usize = 0x004;
const SIO_GPIO_OUT: usize = 0x010;
const SIO_GPIO_OUT_SET: usize = 0x014;
const SIO_GPIO_OUT_CLR: usize = 0x018;
const SIO_GPIO_OE_SET: usize = 0x024;
const SIO_GPIO_OE_CLR: usize = 0x028;

/// `IO_BANK0`'s `FUNCSEL` value that routes a pin to `SIO` (plain GPIO),
/// per the RP2040 datasheet's function-select table (confirmed against
/// `rp2040-pac`'s `FUNCSEL_A::SIO = 5`).
const FUNCSEL_SIO: u32 = 5;

/// `GPIOx_CTRL` sits at `+0x04` within each pin's 8-byte `IO_BANK0`
/// block (`GPIOx_STATUS` at `+0x00` comes first).
fn io_bank0_ctrl(n: u8) -> *mut u32 {
    (IO_BANK0 + (n as usize) * 8 + 0x04) as *mut u32
}

/// Pin 0's pad control register sits at `+0x04` (offset `0x00` is
/// `VOLTAGE_SELECT`, shared across the whole bank), then one 4-byte
/// register per pin.
fn pads_bank0_gpio(n: u8) -> *mut u32 {
    (PADS_BANK0 + 0x04 + (n as usize) * 4) as *mut u32
}

/// Typestate marker: pin configured as digital input.
pub struct Input;
/// Typestate marker: pin configured as digital output.
pub struct Output;

/// A single GPIO pin. RP2040 has one flat pin numbering (0-29, no port
/// letters), so unlike the other boards' `Pin<BASE, N, MODE>`, `N` alone
/// selects the physical pin. Kept as `Pin<N, MODE>` (no `BASE`) rather
/// than force a meaningless port parameter just to match their shape.
pub struct Pin<const N: u8, MODE> {
    _mode: PhantomData<MODE>,
}

impl<const N: u8> Pin<N, Input> {
    /// Take ownership of this pin, defaulting to input mode.
    ///
    /// # Safety
    /// The caller must ensure no other code concurrently accesses the
    /// same pin `N` — there's no runtime tracking preventing two `Pin`
    /// handles for the same physical pin from being created.
    pub const unsafe fn new() -> Self {
        Self { _mode: PhantomData }
    }

    pub fn is_high(&self) -> bool {
        // SAFETY: `N` selects a real GPIO pin (0-29); exclusive access
        // is guaranteed by `Pin::new`'s safety contract.
        unsafe { (core::ptr::read_volatile((SIO + SIO_GPIO_IN) as *const u32) >> N) & 1 != 0 }
    }

    pub fn is_low(&self) -> bool {
        !self.is_high()
    }

    /// Reconfigure as a digital output: mux the pin to `SIO`
    /// (`IO_BANK0.GPIOx_CTRL.FUNCSEL = 5`), configure the pad for
    /// output-only (`PADS_BANK0`: `OD=0`, `IE=0` — matching this board's
    /// own proven LED setup), and enable `SIO`'s output-enable bit.
    pub fn into_output(self) -> Pin<N, Output> {
        // SAFETY: `N` is a real GPIO pin, exclusively owned per
        // `Pin::new`'s contract; `IO_BANK0`/`PADS_BANK0` are already out
        // of reset by board init (see module docs).
        unsafe {
            core::ptr::write_volatile(io_bank0_ctrl(N), FUNCSEL_SIO);
            let pad = pads_bank0_gpio(N);
            let val = core::ptr::read_volatile(pad);
            core::ptr::write_volatile(pad, val & !(0b11 << 6)); // OD=0, IE=0
            core::ptr::write_volatile((SIO + SIO_GPIO_OE_SET) as *mut u32, 1u32 << N);
        }
        Pin { _mode: PhantomData }
    }
}

impl<const N: u8> Pin<N, Output> {
    /// Reconfigure as a digital input: mux to `SIO`, enable the pad's
    /// input path (`IE=1`, `OD=0`), and clear `SIO`'s output-enable bit.
    pub fn into_input(self) -> Pin<N, Input> {
        // SAFETY: as in `into_output`.
        unsafe {
            core::ptr::write_volatile(io_bank0_ctrl(N), FUNCSEL_SIO);
            let pad = pads_bank0_gpio(N);
            let val = core::ptr::read_volatile(pad);
            core::ptr::write_volatile(pad, (val & !(1 << 7)) | (1 << 6)); // OD=0, IE=1
            core::ptr::write_volatile((SIO + SIO_GPIO_OE_CLR) as *mut u32, 1u32 << N);
        }
        Pin { _mode: PhantomData }
    }

    pub fn set_high(&mut self) {
        // SAFETY: as in `into_output` — `SIO`'s SET/CLR registers are
        // write-only atomic bit operations, no read-modify-write race.
        unsafe { core::ptr::write_volatile((SIO + SIO_GPIO_OUT_SET) as *mut u32, 1u32 << N) };
    }

    pub fn set_low(&mut self) {
        // SAFETY: as in `set_high`.
        unsafe { core::ptr::write_volatile((SIO + SIO_GPIO_OUT_CLR) as *mut u32, 1u32 << N) };
    }

    pub fn is_set_high(&self) -> bool {
        // SAFETY: as in `set_high` — `GPIO_OUT` reflects what this pin
        // is being driven to (the RP2040 analogue of STM32's `ODR`),
        // distinct from `GPIO_IN` (actual pad electrical state, used by
        // `Input::is_high`).
        unsafe { (core::ptr::read_volatile((SIO + SIO_GPIO_OUT) as *const u32) >> N) & 1 != 0 }
    }

    pub fn is_set_low(&self) -> bool {
        !self.is_set_high()
    }

    pub fn toggle(&mut self) {
        if self.is_set_high() {
            self.set_low();
        } else {
            self.set_high();
        }
    }
}

// ── embedded-hal 1.0 ─────────────────────────────────────────────────
//
// Thin wrappers over the infallible inherent methods above: this GPIO
// block has no error conditions to report (no bus, no ack, just a
// memory-mapped register), so every method returns `Ok`.
impl<const N: u8, MODE> embedded_hal::digital::ErrorType for Pin<N, MODE> {
    type Error = core::convert::Infallible;
}

impl<const N: u8> embedded_hal::digital::OutputPin for Pin<N, Output> {
    fn set_low(&mut self) -> Result<(), Self::Error> {
        Pin::set_low(self);
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        Pin::set_high(self);
        Ok(())
    }
}

impl<const N: u8> embedded_hal::digital::StatefulOutputPin for Pin<N, Output> {
    fn is_set_high(&mut self) -> Result<bool, Self::Error> {
        Ok(Pin::is_set_high(self))
    }

    fn is_set_low(&mut self) -> Result<bool, Self::Error> {
        Ok(Pin::is_set_low(self))
    }
}

impl<const N: u8> embedded_hal::digital::InputPin for Pin<N, Input> {
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok(Pin::is_high(self))
    }

    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok(Pin::is_low(self))
    }
}

#[cfg(test)]
mod tests {
    // Compile-only checks: these would fail to compile if the typestate
    // boundary were wrong (e.g. if Input exposed set_high). We can't
    // exercise real register writes on host (no memory-mapped GPIO), so
    // this only verifies the type-level API shape — RP2040's registers
    // are at fixed real addresses (no fake-port trick needed/possible
    // the way the other boards' tests use one, since these functions are
    // never actually invoked here either, only referenced as `fn` items).
    use super::*;

    #[test]
    fn typestate_transitions_compile() {
        #[allow(dead_code)]
        fn never_called() {
            // SAFETY: never actually invoked (see test comment above) —
            // this function is referenced but not called.
            let pin: Pin<15, Input> = unsafe { Pin::new() };
            let mut pin = pin.into_output();
            pin.set_high();
            pin.set_low();
            pin.toggle();
            let _pin: Pin<15, Input> = pin.into_input();
        }
        let _ = never_called as fn();
    }

    #[allow(dead_code)]
    fn blink(pin: &mut impl embedded_hal::digital::OutputPin) {
        let _ = pin.set_high();
        let _ = pin.set_low();
    }

    #[allow(dead_code)]
    fn read(pin: &mut impl embedded_hal::digital::InputPin) -> bool {
        pin.is_high().unwrap_or(false)
    }

    #[test]
    fn generic_embedded_hal_functions_accept_pin() {
        fn never_called() {
            // SAFETY: never actually invoked.
            let out: Pin<15, Output> = unsafe { Pin::new() }.into_output();
            let mut out = out;
            blink(&mut out);
            // SAFETY: as above.
            let inp: Pin<14, Input> = unsafe { Pin::new() };
            let mut inp = inp;
            let _ = read(&mut inp);
        }
        let _ = never_called as fn();
    }
}
