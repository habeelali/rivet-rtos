//! Real-hardware proof of `rivet_bsp_stm32f401re::gpio`'s typestate GPIO
//! (embedded-hal-plan.md Phase D): drives the Nucleo-64's onboard LD2 LED
//! (PA5) both through the inherent API and through a generic
//! `embedded_hal::digital::{OutputPin, StatefulOutputPin}`-bound
//! function, reading back `ODR` after each write to confirm the register
//! genuinely changed — not just that the code compiled.

#![no_std]
#![no_main]

use rivet_bsp_stm32f401re as _;
use rivet_rt as _;

use embedded_hal::digital::{OutputPin, StatefulOutputPin};
use rivet_bsp_stm32f401re::gpio::{Input, Pin, PORT_A};

fn drive_via_generic<P: OutputPin + StatefulOutputPin>(pin: &mut P) -> (bool, bool) {
    pin.set_high().unwrap();
    let high = pin.is_set_high().unwrap();
    pin.set_low().unwrap();
    let low = pin.is_set_low().unwrap();
    (high, low)
}

fn test_task(_: &'static ()) -> ! {
    // SAFETY: PA5 (LD2), exclusively owned by this task — nothing else
    // in this binary touches it.
    let led: Pin<PORT_A, 5, Input> = unsafe { Pin::new() };
    let mut led = led.into_output();

    // Inherent API first.
    led.set_high();
    let inherent_high = led.is_set_high();
    led.set_low();
    let inherent_low = led.is_set_low();

    rivet::console::write_str("INHERENT_HIGH=");
    rivet::console::write_str(if inherent_high { "1" } else { "0" });
    rivet::console::write_str(" INHERENT_LOW=");
    rivet::console::write_str(if inherent_low { "1" } else { "0" });
    rivet::console::write_str("\n");

    // Then through the generic embedded-hal trait bound — proves the
    // trait impls actually reach the same hardware, not a separate path.
    let (generic_high, generic_low) = drive_via_generic(&mut led);

    rivet::console::write_str("GENERIC_HIGH=");
    rivet::console::write_str(if generic_high { "1" } else { "0" });
    rivet::console::write_str(" GENERIC_LOW=");
    rivet::console::write_str(if generic_low { "1" } else { "0" });
    rivet::console::write_str("\n");

    if inherent_high && inherent_low && generic_high && generic_low {
        rivet::console::write_str("GPIO_TEST_OK\n");
        rivet::exit_success();
    } else {
        rivet::console::write_str("GPIO_TEST_FAIL\n");
        rivet::exit_failure(1);
    }
}

#[rivet::main]
fn main() -> ! {
    rivet::console::write_str("Rivet gpio_test\n");
    let _ = rivet::spawn_ptask!(stack = 1024, priority = 1, entry = test_task, arg = ());
    rivet::run();
}
