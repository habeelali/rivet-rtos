//! BCM2837 GPIO.
//!
//! Same typestate shape the other boards' pins use: a `Pin` carries its
//! number and direction in the type, and the embedded-hal traits are
//! implemented on the configured forms rather than on a raw handle.
//!
//! One detail makes this board pleasant to test: `GPLEV` reports the
//! actual electrical level of the pad, not the value last written to
//! `GPSET`/`GPCLR`. Driving an output and reading it back therefore
//! exercises the whole path down to the pin without needing a wire
//! looped between two of them.

use core::convert::Infallible;
use core::marker::PhantomData;
use core::ptr::{read_volatile, write_volatile};

use crate::mmio::GPIO_BASE;

const GPFSEL0: usize = 0x00;
const GPSET0: usize = 0x1C;
const GPCLR0: usize = 0x28;
const GPLEV0: usize = 0x34;
const GPPUD: usize = 0x94;
const GPPUDCLK0: usize = 0x98;

/// Function-select codes. The alternate functions are not in numeric
/// order, which is a recurring source of wrong pin muxing.
pub mod func {
    pub const INPUT: u32 = 0b000;
    pub const OUTPUT: u32 = 0b001;
    pub const ALT0: u32 = 0b100;
    pub const ALT1: u32 = 0b101;
    pub const ALT2: u32 = 0b110;
    pub const ALT3: u32 = 0b111;
    pub const ALT4: u32 = 0b011;
    pub const ALT5: u32 = 0b010;
}

/// Pull-up/down selection for [`set_pull`].
pub mod pull {
    pub const NONE: u32 = 0b00;
    pub const DOWN: u32 = 0b01;
    pub const UP: u32 = 0b10;
}

/// Select the function of a pin.
///
/// # Safety
/// GPFSEL registers are shared, so this is a read-modify-write against
/// state another agent could be changing. Callers must own the pin.
pub unsafe fn set_function(pin: u8, f: u32) {
    // Ten pins per register, three bits each.
    let reg = (GPIO_BASE + GPFSEL0 + (pin as usize / 10) * 4) as *mut u32;
    let shift = (pin as usize % 10) * 3;
    // SAFETY: caller owns the pin; the address is a valid GPFSEL.
    unsafe {
        let v = read_volatile(reg);
        write_volatile(reg, (v & !(0b111 << shift)) | (f << shift));
    }
}

/// Drive `pin` high without holding a `Pin` handle.
///
/// For the interrupt path, where there is nowhere to keep a handle and
/// the cost of the write is part of what is being measured. GPSET is
/// write-to-set, so this is a single store that cannot disturb any other
/// pin, including one another core is driving.
///
/// # Safety
/// The pin must already be configured as an output, and nothing else may
/// be driving it.
pub unsafe fn raise(pin: u8) {
    let reg = (GPIO_BASE + GPSET0 + (pin as usize / 32) * 4) as *mut u32;
    // SAFETY: forwarded from this function's contract.
    unsafe { write_volatile(reg, 1 << (pin % 32)) };
}

/// Drive `pin` low. See [`raise`].
///
/// # Safety
/// Same as [`raise`].
pub unsafe fn lower(pin: u8) {
    let reg = (GPIO_BASE + GPCLR0 + (pin as usize / 32) * 4) as *mut u32;
    // SAFETY: forwarded from this function's contract.
    unsafe { write_volatile(reg, 1 << (pin % 32)) };
}

/// Apply a pull-up, pull-down, or neither, to a pin.
///
/// BCM2837 uses the BCM2835 clocked handshake for this. The newer
/// register-per-pin scheme belongs to BCM2711 and does not exist here.
///
/// # Safety
/// Shared registers, as above.
pub unsafe fn set_pull(pin: u8, p: u32) {
    let gppud = (GPIO_BASE + GPPUD) as *mut u32;
    let clk = (GPIO_BASE + GPPUDCLK0 + (pin as usize / 32) * 4) as *mut u32;
    // SAFETY: caller owns the pin. The waits are required by the
    // datasheet, not defensive padding.
    unsafe {
        write_volatile(gppud, p);
        crate::delay(150);
        write_volatile(clk, 1 << (pin % 32));
        crate::delay(150);
        write_volatile(gppud, 0);
        write_volatile(clk, 0);
    }
}

/// Marker for a pin configured as an output.
pub struct Output;
/// Marker for a pin configured as an input.
pub struct Input;

/// A single GPIO pin, with its direction in the type.
pub struct Pin<MODE> {
    pin: u8,
    _mode: PhantomData<MODE>,
}

impl Pin<Input> {
    /// Take a pin as an input.
    ///
    /// # Safety
    /// Nothing else may be using this pin.
    pub unsafe fn input(pin: u8) -> Self {
        // SAFETY: forwarded from this function's contract.
        unsafe { set_function(pin, func::INPUT) };
        Pin {
            pin,
            _mode: PhantomData,
        }
    }

    pub fn is_high(&self) -> bool {
        level(self.pin)
    }

    /// Reconfigure as an output.
    pub fn into_output(self) -> Pin<Output> {
        // SAFETY: this handle owns the pin.
        unsafe { set_function(self.pin, func::OUTPUT) };
        Pin {
            pin: self.pin,
            _mode: PhantomData,
        }
    }
}

impl Pin<Output> {
    /// Take a pin as an output.
    ///
    /// # Safety
    /// Nothing else may be using this pin.
    pub unsafe fn output(pin: u8) -> Self {
        // SAFETY: forwarded from this function's contract.
        unsafe { set_function(pin, func::OUTPUT) };
        Pin {
            pin,
            _mode: PhantomData,
        }
    }

    pub fn set_high(&mut self) {
        let reg = (GPIO_BASE + GPSET0 + (self.pin as usize / 32) * 4) as *mut u32;
        // SAFETY: GPSET is write-to-set, so this cannot disturb other pins.
        unsafe { write_volatile(reg, 1 << (self.pin % 32)) };
    }

    pub fn set_low(&mut self) {
        let reg = (GPIO_BASE + GPCLR0 + (self.pin as usize / 32) * 4) as *mut u32;
        // SAFETY: GPCLR is write-to-clear, same reasoning.
        unsafe { write_volatile(reg, 1 << (self.pin % 32)) };
    }

    /// The pad's actual level, which for an output is the value it is
    /// driving. Reading this back is what makes a single pin
    /// self-testable.
    pub fn level(&self) -> bool {
        level(self.pin)
    }

    /// Reconfigure as an input.
    pub fn into_input(self) -> Pin<Input> {
        // SAFETY: this handle owns the pin.
        unsafe { set_function(self.pin, func::INPUT) };
        Pin {
            pin: self.pin,
            _mode: PhantomData,
        }
    }
}

fn level(pin: u8) -> bool {
    let reg = (GPIO_BASE + GPLEV0 + (pin as usize / 32) * 4) as *const u32;
    // SAFETY: a plain read of a level register.
    unsafe { read_volatile(reg) & (1 << (pin % 32)) != 0 }
}

impl<MODE> embedded_hal::digital::ErrorType for Pin<MODE> {
    type Error = Infallible;
}

impl embedded_hal::digital::OutputPin for Pin<Output> {
    fn set_high(&mut self) -> Result<(), Infallible> {
        Pin::set_high(self);
        Ok(())
    }
    fn set_low(&mut self) -> Result<(), Infallible> {
        Pin::set_low(self);
        Ok(())
    }
}

impl embedded_hal::digital::StatefulOutputPin for Pin<Output> {
    fn is_set_high(&mut self) -> Result<bool, Infallible> {
        Ok(self.level())
    }
    fn is_set_low(&mut self) -> Result<bool, Infallible> {
        Ok(!self.level())
    }
}

impl embedded_hal::digital::InputPin for Pin<Input> {
    fn is_high(&mut self) -> Result<bool, Infallible> {
        Ok(Pin::is_high(self))
    }
    fn is_low(&mut self) -> Result<bool, Infallible> {
        Ok(!Pin::is_high(self))
    }
}
