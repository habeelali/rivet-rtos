//! Typestate GPIO for the ESP32-C6, following
//! `rivet-bsp-lm3s6965::gpio`'s exact pattern (embedded-hal-plan.md
//! Phase D).
//!
//! Two peripherals cooperate, ESP32-family style: `IO_MUX` selects each
//! pad's function (`MCU_SEL = 1` is `PIN_FUNC_GPIO` — Function 2, the
//! plain-digital-I/O routing, a fixed ESP-IDF-wide constant across every
//! ESP32 variant, not something the SVD names directly) and `FUN_IE`
//! gates whether the pad's input path is live at all; `GPIO` itself
//! holds the actual output/enable/input registers this module's
//! `set_high`/`is_high`/etc. touch. Both base addresses and every
//! register offset here were verified directly against the `esp32c6`
//! crate's generated source (`Periph<_, ADDR>` type aliases and each
//! `RegisterBlock`'s field-offset doc comments), not the datasheet
//! alone. No GPIO matrix routing is needed for plain digital I/O — the
//! matrix only matters for peripheral (UART/SPI/etc.) signal routing;
//! `MCU_SEL = PIN_FUNC_GPIO` connects the pad directly to `GPIO`'s
//! OUT/IN registers, matching ESP-IDF's own `gpio_config()`.

use core::marker::PhantomData;

/// `IO_MUX` — per-pad function/input-enable config.
const IO_MUX: usize = 0x6009_0000;
/// `GPIO` — the output/enable/input registers "being a plain GPIO"
/// means.
const GPIO: usize = 0x6009_1000;

const GPIO_OUT: usize = 0x04;
const GPIO_OUT_W1TS: usize = 0x08;
const GPIO_OUT_W1TC: usize = 0x0C;
const GPIO_ENABLE_W1TS: usize = 0x24;
const GPIO_ENABLE_W1TC: usize = 0x28;
const GPIO_IN: usize = 0x3C;

/// `PIN_FUNC_GPIO` — the `IO_MUX` `MCU_SEL` value that routes a pad
/// directly to the `GPIO` peripheral (ESP-IDF's `soc/io_mux_reg.h`
/// constant, consistent across every ESP32 variant this workspace
/// targets).
const PIN_FUNC_GPIO: u32 = 1;
const MCU_SEL_SHIFT: u32 = 12;
const FUN_IE_BIT: u32 = 1 << 9;

/// `IO_MUX`'s per-pad register: `PIN_CTRL` occupies `+0x00`, then one
/// 4-byte register per pad starting at `+0x04`.
fn io_mux_gpio(n: u8) -> *mut u32 {
    (IO_MUX + 0x04 + (n as usize) * 4) as *mut u32
}

/// Typestate marker: pin configured as digital input.
pub struct Input;
/// Typestate marker: pin configured as digital output.
pub struct Output;

/// A single GPIO pin (0-30 on this chip — `IO_MUX`'s pad array has 31
/// entries). Flat numbering, no port letters, matching
/// `rivet_bsp_rp2040::gpio::Pin<N, MODE>`'s shape for the same reason.
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
        // SAFETY: `N` selects a real GPIO pin (0-30); exclusive access
        // is guaranteed by `Pin::new`'s safety contract.
        unsafe { (core::ptr::read_volatile((GPIO + GPIO_IN) as *const u32) >> N) & 1 != 0 }
    }

    pub fn is_low(&self) -> bool {
        !self.is_high()
    }

    /// Reconfigure as a digital output: route the pad to `GPIO`
    /// (`MCU_SEL = PIN_FUNC_GPIO`, input path left as-is — a pad with
    /// `FUN_IE` still set reads back its own driven value fine, exactly
    /// like every other board's output pins here), then enable `GPIO`'s
    /// output-enable bit for this pin.
    pub fn into_output(self) -> Pin<N, Output> {
        // SAFETY: `N` is a real GPIO pin, exclusively owned per
        // `Pin::new`'s contract.
        unsafe {
            let reg = io_mux_gpio(N);
            let val = core::ptr::read_volatile(reg);
            let mask = 0b111u32 << MCU_SEL_SHIFT;
            core::ptr::write_volatile(reg, (val & !mask) | (PIN_FUNC_GPIO << MCU_SEL_SHIFT));
            core::ptr::write_volatile((GPIO + GPIO_ENABLE_W1TS) as *mut u32, 1u32 << N);
        }
        Pin { _mode: PhantomData }
    }
}

impl<const N: u8> Pin<N, Output> {
    /// Reconfigure as a digital input: route to `GPIO`, ensure `FUN_IE`
    /// (input enable) is set, and clear `GPIO`'s output-enable bit.
    pub fn into_input(self) -> Pin<N, Input> {
        // SAFETY: as in `into_output`.
        unsafe {
            let reg = io_mux_gpio(N);
            let val = core::ptr::read_volatile(reg);
            let mask = 0b111u32 << MCU_SEL_SHIFT;
            let new_val = (val & !mask) | (PIN_FUNC_GPIO << MCU_SEL_SHIFT) | FUN_IE_BIT;
            core::ptr::write_volatile(reg, new_val);
            core::ptr::write_volatile((GPIO + GPIO_ENABLE_W1TC) as *mut u32, 1u32 << N);
        }
        Pin { _mode: PhantomData }
    }

    pub fn set_high(&mut self) {
        // SAFETY: as in `into_output` — `OUT_W1TS`/`OUT_W1TC` are
        // write-only atomic bit-set/clear registers, no
        // read-modify-write race.
        unsafe { core::ptr::write_volatile((GPIO + GPIO_OUT_W1TS) as *mut u32, 1u32 << N) };
    }

    pub fn set_low(&mut self) {
        // SAFETY: as in `set_high`.
        unsafe { core::ptr::write_volatile((GPIO + GPIO_OUT_W1TC) as *mut u32, 1u32 << N) };
    }

    pub fn is_set_high(&self) -> bool {
        // SAFETY: as in `set_high` — `GPIO_OUT` reflects what this pin
        // is being driven to, distinct from `GPIO_IN` (actual pad
        // electrical state, used by `Input::is_high`).
        unsafe { (core::ptr::read_volatile((GPIO + GPIO_OUT) as *const u32) >> N) & 1 != 0 }
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

// ── embedded-hal 1.0 (embedded-hal-plan.md Phase D) ─────────────────
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
    use super::*;

    #[test]
    fn typestate_transitions_compile() {
        #[allow(dead_code)]
        fn never_called() {
            // SAFETY: never actually invoked — referenced but not called.
            let pin: Pin<8, Input> = unsafe { Pin::new() };
            let mut pin = pin.into_output();
            pin.set_high();
            pin.set_low();
            pin.toggle();
            let _pin: Pin<8, Input> = pin.into_input();
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
            let out: Pin<8, Output> = unsafe { Pin::new() }.into_output();
            let mut out = out;
            blink(&mut out);
            // SAFETY: as above.
            let inp: Pin<9, Input> = unsafe { Pin::new() };
            let mut inp = inp;
            let _ = read(&mut inp);
        }
        let _ = never_called as fn();
    }
}
