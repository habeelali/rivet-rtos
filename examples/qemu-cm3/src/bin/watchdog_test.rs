//! Watchdog test (plan.md §3.5 / §3.6).
//!
//! The watchdog is armed with a short period; a task feeds it for a few
//! periods then stops. The watchdog fires: on Cortex-M the real
//! luminary-watchdog hardware resets the system (QEMU models reset-on-
//! expiry); on RISC-V the software watchdog resets via `riscv.sifive.test`
//! (0x7777). Either way "RIVET WATCHDOG TIMEOUT" is printed first — the
//! harness asserts the marker via golden-on-timeout (the reset reboots the
//! guest rather than exiting).

#![no_std]
#![no_main]

use rivet_bsp_lm3s6965 as _;
use rivet_rt as _;

static FEEDS: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);

fn feeder(_: &'static ()) -> ! {
    // Feed 3 watchdog periods, then go silent.
    loop {
        let n = FEEDS.load(core::sync::atomic::Ordering::Acquire);
        if n >= 3 {
            // Stop feeding: the watchdog must fire.
            loop {
                core::hint::spin_loop();
            }
        }
        rivet::watchdog::feed();
        FEEDS.store(n + 1, core::sync::atomic::Ordering::Release);
        // Wait ~half a watchdog period between feeds.
        for _ in 0..200_000 {
            core::hint::spin_loop();
        }
    }
}

#[rivet::main]
fn main() -> ! {
    rivet::console::write_str("Rivet CM3 watchdog_test: feeding then going silent\n");

    rivet::watchdog::init(rivet::time::Duration::from_millis(250));

    let _ = rivet::spawn_ptask!(stack = 512, priority = 2, entry = feeder, arg = ());

    rivet::run();
}
