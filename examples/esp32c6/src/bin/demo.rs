//! Rivet RTOS on real ESP32-C6 hardware (plan.md Phase 26) — the minimal
//! first proof, same shape as the ESP32-S3 board's own first test: one
//! preemptive task, no tick/switching yet (validates boot, watchdog
//! disable, console, and `start_first_task`'s fabricated first-dispatch
//! `Context` in isolation, before the interrupt-matrix/tick work that
//! real preemption needs — see `rivet-bsp-esp32c6`'s module docs).

#![no_std]
#![no_main]

use rivet_bsp_esp32c6 as _;
use rivet_rt as _;

fn worker(_: &'static ()) -> ! {
    let mut n: u32 = 0;
    loop {
        rivet::console::write_str("RIVET_C6 tick ");
        print_dec(n);
        rivet::console::write_str("\n");
        n = n.wrapping_add(1);
        if n >= 5 {
            rivet::console::write_str("RIVET_C6_OK\n");
            rivet::exit_success();
        }
        for _ in 0..3_000_000u32 {
            core::hint::spin_loop();
        }
    }
}

fn print_dec(mut n: u32) {
    if n == 0 {
        rivet::console::write_str("0");
        return;
    }
    let mut digits = [0u8; 10];
    let mut i = 0;
    while n > 0 {
        digits[i] = b'0' + (n % 10) as u8;
        n /= 10;
        i += 1;
    }
    let mut buf = [0u8; 10];
    for j in 0..i {
        buf[j] = digits[i - 1 - j];
    }
    if let Ok(s) = core::str::from_utf8(&buf[..i]) {
        rivet::console::write_str(s);
    }
}

#[rivet::main]
fn main() -> ! {
    rivet::console::write_str("Rivet esp32c6 demo\n");
    let _ = rivet::spawn_ptask!(stack = 2048, priority = 1, entry = worker, arg = ());
    rivet::run();
}
