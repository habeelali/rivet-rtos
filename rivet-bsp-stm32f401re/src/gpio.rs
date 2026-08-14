//! Typestate GPIO for the STM32F401RE (Cortex-M4), following
//! `rivet-bsp-lm3s6965::gpio`'s exact pattern (see that module's doc
//! comment for the typestate rationale).
//!
//! Two real differences from LM3S6965's GPIO: STM32's AHB1 peripheral
//! clocks are gated (`RCC.AHB1ENR`) — a port with its clock off silently
//! drops every register write, so `into_output()` enables it as part of
//! configuring the pin rather than requiring a separate step a caller
//! could forget. And STM32 has genuinely separate input/output data
//! registers (`IDR`/`ODR`), unlike LM3S6965's one address-masked
//! `GPIODATA` serving both roles — `is_high()` (Input) reads the real
//! electrical state via `IDR`; `is_set_high()` (Output) reads what's
//! being driven via `ODR`.

use core::marker::PhantomData;

/// GPIO port base addresses (STM32F401 memory map, RM0368) — A through E
/// and H only; this part has no GPIOF/G.
pub const PORT_A: usize = 0x4002_0000;
pub const PORT_B: usize = 0x4002_0400;
pub const PORT_C: usize = 0x4002_0800;
pub const PORT_D: usize = 0x4002_0C00;
pub const PORT_E: usize = 0x4002_1000;
pub const PORT_H: usize = 0x4002_1C00;

/// Register offsets, relative to a port's base address.
const MODER: usize = 0x00;
const IDR: usize = 0x10;
const ODR: usize = 0x14;
const BSRR: usize = 0x18;

/// `RCC.AHB1ENR` — one bit per AHB1 peripheral (GPIOx clock gates live
/// here); a port's registers read/write as no-ops until its bit is set.
const RCC_AHB1ENR: usize = 0x4002_3830;

/// `RCC_AHB1ENR`'s GPIOx enable bit for a given port base, derived at
/// compile time: ports are spaced 0x400 apart from `PORT_A` (bit 0)
/// through `PORT_E` (bit 4); `PORT_H` sits 3 slots further at bit 7,
/// matching RM0368's AHB1ENR bit layout (GPIOF/G's bits 5/6 don't exist
/// on this part, but the spacing still lands `PORT_H` correctly).
const fn ahb1enr_bit(base: usize) -> u32 {
    ((base - PORT_A) / 0x400) as u32
}

/// Typestate marker: pin configured as digital input.
pub struct Input;
/// Typestate marker: pin configured as digital output.
pub struct Output;

/// A single GPIO pin. `BASE` selects the port (see `PORT_*` constants),
/// `N` the pin number (0-15), `MODE` the typestate (`Input`/`Output`).
pub struct Pin<const BASE: usize, const N: u8, MODE> {
    _mode: PhantomData<MODE>,
}

impl<const BASE: usize, const N: u8> Pin<BASE, N, Input> {
    /// Take ownership of this pin, defaulting to input mode (`MODER`
    /// reset state). No hardware writes happen here — direction is
    /// whatever the peripheral reset state already has.
    ///
    /// # Safety
    /// The caller must ensure no other code concurrently accesses the
    /// same `(BASE, N)` pin — there's no runtime tracking preventing two
    /// `Pin` handles for the same physical pin from being created.
    pub const unsafe fn new() -> Self {
        Self { _mode: PhantomData }
    }

    pub fn is_high(&self) -> bool {
        // SAFETY: `BASE`/`N` select a real, memory-mapped GPIO port/pin
        // (see `PORT_*`); exclusive access is guaranteed by `Pin::new`'s
        // safety contract.
        unsafe { (core::ptr::read_volatile((BASE + IDR) as *const u32) >> N) & 1 != 0 }
    }

    pub fn is_low(&self) -> bool {
        !self.is_high()
    }

    /// Reconfigure as a digital output: enables the port's AHB1 clock
    /// (idempotent — harmless if already on, e.g. from board init) and
    /// sets `MODER`'s 2-bit field for this pin to `01` (output).
    pub fn into_output(self) -> Pin<BASE, N, Output> {
        // SAFETY: `BASE` is a compile-time constant selecting a real,
        // memory-mapped GPIO port (see `PORT_*`), `N < 16` is the type's
        // own contract, and exclusive access to the pin is guaranteed by
        // `Pin::new`'s safety contract.
        unsafe {
            let rcc_en = RCC_AHB1ENR as *mut u32;
            let bit = 1u32 << ahb1enr_bit(BASE);
            core::ptr::write_volatile(rcc_en, core::ptr::read_volatile(rcc_en) | bit);

            let moder = (BASE + MODER) as *mut u32;
            let mask = 0b11u32 << (N * 2);
            let val = (core::ptr::read_volatile(moder) & !mask) | (0b01u32 << (N * 2));
            core::ptr::write_volatile(moder, val);
        }
        Pin { _mode: PhantomData }
    }
}

impl<const BASE: usize, const N: u8> Pin<BASE, N, Output> {
    /// Reconfigure as a digital input (clears `MODER`'s 2-bit field for
    /// this pin back to `00`).
    pub fn into_input(self) -> Pin<BASE, N, Input> {
        // SAFETY: as in `into_output` — compile-time port base, exclusive
        // pin handle.
        unsafe {
            let moder = (BASE + MODER) as *mut u32;
            let mask = 0b11u32 << (N * 2);
            core::ptr::write_volatile(moder, core::ptr::read_volatile(moder) & !mask);
        }
        Pin { _mode: PhantomData }
    }

    pub fn set_high(&mut self) {
        // SAFETY: as in `into_output` — `BSRR` is write-only set/reset,
        // no read-modify-write race possible; bit `N` sets this pin.
        unsafe { core::ptr::write_volatile((BASE + BSRR) as *mut u32, 1u32 << N) };
    }

    pub fn set_low(&mut self) {
        // SAFETY: as in `set_high`; bit `N + 16` resets this pin.
        unsafe { core::ptr::write_volatile((BASE + BSRR) as *mut u32, 1u32 << (N + 16)) };
    }

    pub fn is_set_high(&self) -> bool {
        // SAFETY: as in `set_high` — volatile read of a memory-mapped
        // register is sound for an exclusive pin handle. `ODR` reflects
        // what this pin is being driven to, not the electrical state
        // (that's `IDR`, used by `Input::is_high`).
        unsafe { (core::ptr::read_volatile((BASE + ODR) as *const u32) >> N) & 1 != 0 }
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
impl<const BASE: usize, const N: u8, MODE> embedded_hal::digital::ErrorType for Pin<BASE, N, MODE> {
    type Error = core::convert::Infallible;
}

impl<const BASE: usize, const N: u8> embedded_hal::digital::OutputPin for Pin<BASE, N, Output> {
    fn set_low(&mut self) -> Result<(), Self::Error> {
        Pin::set_low(self);
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        Pin::set_high(self);
        Ok(())
    }
}

impl<const BASE: usize, const N: u8> embedded_hal::digital::StatefulOutputPin
    for Pin<BASE, N, Output>
{
    fn is_set_high(&mut self) -> Result<bool, Self::Error> {
        Ok(Pin::is_set_high(self))
    }

    fn is_set_low(&mut self) -> Result<bool, Self::Error> {
        Ok(Pin::is_set_low(self))
    }
}

impl<const BASE: usize, const N: u8> embedded_hal::digital::InputPin for Pin<BASE, N, Input> {
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
    // this only verifies the type-level API shape, using a fake base
    // address that's never actually dereferenced.
    use super::*;

    const FAKE_PORT: usize = 0x1000_0000;

    #[test]
    fn typestate_transitions_compile() {
        #[allow(dead_code)]
        fn never_called() {
            // SAFETY: fake base address, never dereferenced — compile-only.
            let pin: Pin<FAKE_PORT, 5, Input> = unsafe { Pin::new() };
            let mut pin = pin.into_output();
            pin.set_high();
            pin.set_low();
            pin.toggle();
            let _pin: Pin<FAKE_PORT, 5, Input> = pin.into_input();
        }
        let _ = core::mem::size_of::<Pin<FAKE_PORT, 5, Output>>();
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
            // SAFETY: fake base address, never dereferenced.
            let out: Pin<FAKE_PORT, 5, Output> = unsafe { Pin::new() }.into_output();
            let mut out = out;
            blink(&mut out);
            // SAFETY: as above.
            let inp: Pin<FAKE_PORT, 6, Input> = unsafe { Pin::new() };
            let mut inp = inp;
            let _ = read(&mut inp);
        }
        let _ = never_called as fn();
    }

    #[test]
    fn ahb1enr_bit_matches_rm0368() {
        assert_eq!(ahb1enr_bit(PORT_A), 0);
        assert_eq!(ahb1enr_bit(PORT_B), 1);
        assert_eq!(ahb1enr_bit(PORT_C), 2);
        assert_eq!(ahb1enr_bit(PORT_D), 3);
        assert_eq!(ahb1enr_bit(PORT_E), 4);
        assert_eq!(ahb1enr_bit(PORT_H), 7);
    }
}
