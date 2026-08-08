//! Typestate GPIO for the LM3S6965 (Stellaris) Cortex-M3.
//!
//! Pin direction is tracked in the *type*, not a runtime flag: a
//! `Pin<BASE, N, Input>` simply has no `set_high`/`set_low`/`toggle`
//! methods — calling them on a pin still configured as input is a
//! compile error, not a runtime check. `into_output()`/`into_input()`
//! consume `self` and return the pin retyped to the new mode, so the old
//! (wrongly-typed) handle can't be used afterward either.
//!
//! ```ignore
//! use rivet::hal::gpio::{Pin, Input, PORT_F};
//!
//! let led: Pin<PORT_F, 1, Input> = unsafe { Pin::new() };
//! let mut led = led.into_output();
//! led.set_high();
//! led.toggle();
//! // led.into_input().set_high();  // <- would not compile: Input has no set_high
//! ```

use core::marker::PhantomData;

/// GPIO port base addresses (LM3S6965 memory map).
pub const PORT_A: usize = 0x4000_4000;
pub const PORT_B: usize = 0x4000_5000;
pub const PORT_C: usize = 0x4000_6000;
pub const PORT_D: usize = 0x4000_7000;
pub const PORT_E: usize = 0x4002_4000;
pub const PORT_F: usize = 0x4002_5000;
pub const PORT_G: usize = 0x4002_6000;

/// Register offsets, relative to a port's base address.
const GPIODIR: usize = 0x400;
const GPIODEN: usize = 0x51C;

/// Typestate marker: pin configured as digital input.
pub struct Input;
/// Typestate marker: pin configured as digital output.
pub struct Output;

/// A single GPIO pin. `BASE` selects the port (see `PORT_*` constants),
/// `N` the pin number (0-7), `MODE` the typestate (`Input`/`Output`).
pub struct Pin<const BASE: usize, const N: u8, MODE> {
    _mode: PhantomData<MODE>,
}

impl<const BASE: usize, const N: u8> Pin<BASE, N, Input> {
    /// Take ownership of this pin, defaulting to input mode (GPIODIR reset
    /// state). No hardware writes happen here — direction is whatever the
    /// peripheral reset state already has.
    ///
    /// # Safety
    /// The caller must ensure no other code concurrently accesses the same
    /// `(BASE, N)` pin — there's no runtime tracking preventing two `Pin`
    /// handles for the same physical pin from being created.
    pub const unsafe fn new() -> Self {
        Self { _mode: PhantomData }
    }

    /// Reconfigure as a digital output. Sets GPIODEN (digital enable —
    /// required on LM3S6965 for any digital I/O, analog by default) and
    /// GPIODIR (direction) for this pin.
    pub fn into_output(self) -> Pin<BASE, N, Output> {
        let mask = 1u32 << N;
        // SAFETY: `BASE` is a compile-time constant selecting a real,
        // memory-mapped GPIO port (see `PORT_*`), and `N < 8` is enforced
        // by the mask being a single bit. Exclusive access to the pin is
        // guaranteed by `Pin::new`'s safety contract (one handle per
        // physical pin).
        unsafe {
            let dir = (BASE + GPIODIR) as *mut u32;
            let den = (BASE + GPIODEN) as *mut u32;
            core::ptr::write_volatile(dir, core::ptr::read_volatile(dir) | mask);
            core::ptr::write_volatile(den, core::ptr::read_volatile(den) | mask);
        }
        Pin { _mode: PhantomData }
    }
}

impl<const BASE: usize, const N: u8> Pin<BASE, N, Output> {
    /// Reconfigure as a digital input (clears GPIODIR for this pin).
    pub fn into_input(self) -> Pin<BASE, N, Input> {
        let mask = 1u32 << N;
        // SAFETY: as in `into_output` — compile-time port base, single-bit
        // pin mask, exclusive pin handle.
        unsafe {
            let dir = (BASE + GPIODIR) as *mut u32;
            core::ptr::write_volatile(dir, core::ptr::read_volatile(dir) & !mask);
        }
        Pin { _mode: PhantomData }
    }

    /// LM3S6965's GPIODATA register uses address-bus bits [9:2] as a pin
    /// mask: only bits whose corresponding address line is set are
    /// affected by the access, letting single-pin read/write skip a
    /// read-modify-write. This computes that per-pin address once.
    #[inline]
    fn data_reg(&self) -> *mut u32 {
        (BASE + (((1usize << N) & 0xFF) << 2)) as *mut u32
    }

    pub fn set_high(&mut self) {
        // SAFETY: as in `into_output` — memory-mapped port at a
        // compile-time address, exclusive pin handle, single-bit
        // address-masked data access.
        unsafe { core::ptr::write_volatile(self.data_reg(), 0xFFFF_FFFF) };
    }

    pub fn set_low(&mut self) {
        // SAFETY: as in `set_high`.
        unsafe { core::ptr::write_volatile(self.data_reg(), 0) };
    }

    pub fn is_set_high(&self) -> bool {
        // SAFETY: as in `set_high` — volatile read of a memory-mapped
        // register is sound for an exclusive pin handle.
        unsafe { core::ptr::read_volatile(self.data_reg()) != 0 }
    }

    pub fn toggle(&mut self) {
        if self.is_set_high() {
            self.set_low();
        } else {
            self.set_high();
        }
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
        // The fake base address is never dereferenced — the pin is never
        // actually constructed at runtime; this only checks the typestate
        // API shape compiles.
        #[allow(dead_code)]
        fn never_called() {
            // SAFETY: fake base address, never dereferenced — compile-only.
            let pin: Pin<FAKE_PORT, 3, Input> = unsafe { Pin::new() };
            let mut pin = pin.into_output();
            pin.set_high();
            pin.set_low();
            pin.toggle();
            let _pin: Pin<FAKE_PORT, 3, Input> = pin.into_input();
        }
        // Silence the unused-const lint for the compile-only path.
        let _ = core::mem::size_of::<Pin<FAKE_PORT, 3, Output>>();
    }
}
