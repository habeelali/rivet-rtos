//! Real-hardware proof of `rivet_bsp_esp32s3::gpio`'s typestate GPIO:
//! drives GPIO2 both through the
//! inherent API and through a generic `embedded_hal::digital::
//! {OutputPin, StatefulOutputPin}`-bound function, reading back
//! `GPIO_OUT` after each write to confirm the register genuinely
//! changed. GPIO2 chosen as a plain, non-strapping pin on this chip
//! (unlike GPIO0/3/45/46, which have boot-mode-strapping roles).

#![no_std]
#![no_main]

use rivet_bsp_esp32s3 as _;
use rivet_rt as _;

use embedded_hal::digital::{OutputPin, StatefulOutputPin};
use rivet_bsp_esp32s3::gpio::{Input, Pin};

fn drive_via_generic<P: OutputPin + StatefulOutputPin>(pin: &mut P) -> (bool, bool) {
    pin.set_high().unwrap();
    let high = pin.is_set_high().unwrap();
    pin.set_low().unwrap();
    let low = pin.is_set_low().unwrap();
    (high, low)
}

fn test_task(_: &'static ()) -> ! {
    // SAFETY: GPIO2, exclusively owned by this task — nothing else in
    // this binary touches it.
    let pin: Pin<2, Input> = unsafe { Pin::new() };
    let mut pin = pin.into_output();

    // Inherent API first.
    pin.set_high();
    let inherent_high = pin.is_set_high();
    pin.set_low();
    let inherent_low = pin.is_set_low();

    rivet::console::write_str("INHERENT_HIGH=");
    rivet::console::write_str(if inherent_high { "1" } else { "0" });
    rivet::console::write_str(" INHERENT_LOW=");
    rivet::console::write_str(if inherent_low { "1" } else { "0" });
    rivet::console::write_str("\n");

    // Then through the generic embedded-hal trait bound.
    let (generic_high, generic_low) = drive_via_generic(&mut pin);

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
