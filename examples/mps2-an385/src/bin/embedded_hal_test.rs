//! End-to-end `embedded-hal`/`-async`/`-nb` test (plan.md Phase 15).
//!
//! Proves `RivetDelay`/`Serial` work through *generic* code written
//! against the traits, not just that they compile standalone: `delay`
//! is called through a trait-bounded generic function and actually
//! blocks the calling task (measured via wall-clock time), and `Serial`
//! is exercised through `embedded_hal_nb::serial::Write` to prove a byte
//! written that way really reaches the console.

#![no_std]
#![no_main]

use rivet_bsp_mps2_an385 as _;
use rivet_rt as _;

use embedded_hal_async::delay::DelayNs;
use embedded_hal_nb::serial::Write;
use rivet_bsp_support::delay::RivetDelay;
use rivet_bsp_support::serial::Serial;

async fn wait_via_generic<D: DelayNs>(d: &mut D, ms: u32) {
    d.delay_ms(ms).await;
}

fn write_via_generic<W: Write<u8>>(w: &mut W, bytes: &[u8]) {
    for &b in bytes {
        nb::block!(w.write(b)).ok();
    }
    nb::block!(w.flush()).ok();
}

fn test_task(_: &'static ()) -> ! {
    let mut delay = RivetDelay;
    let start = rivet::port::board::now_us();
    // `delay_ms` is async, but `RivetDelay` is a blocking impl (see its
    // docs) — `poll_once`-style manual driving isn't needed; calling it
    // from a preemptive task and just running the future to completion
    // synchronously (it never actually returns Pending) is the intended
    // usage.
    let fut = wait_via_generic(&mut delay, 20);
    let waker = noop_waker();
    let poll = core::pin::pin!(fut)
        .as_mut()
        .poll(&mut core::task::Context::from_waker(&waker));
    if poll.is_pending() {
        // `RivetDelay` is documented to never actually suspend (it blocks
        // synchronously inside the one poll) — a `Pending` here would
        // mean that documented contract broke.
        rivet::console::write_str("DELAY_FAIL: future returned Pending\n");
        rivet::exit_failure(2);
    }
    let elapsed = rivet::port::board::now_us() - start;

    rivet::console::write_str("DELAY_ELAPSED_US=");
    print_dec(elapsed as usize);
    rivet::console::write_str("\n");
    if elapsed < 15_000 {
        rivet::console::write_str("DELAY_FAIL\n");
        rivet::exit_failure(1);
    }
    rivet::console::write_str("DELAY_OK\n");

    let mut serial = Serial;
    write_via_generic(&mut serial, b"HELLO_NB\n");
    rivet::console::write_str("SERIAL_OK\n");

    rivet::console::write_str("EMBEDDED_HAL_TEST_OK\n");
    rivet::exit_success();
}

fn noop_waker() -> core::task::Waker {
    fn clone(_: *const ()) -> core::task::RawWaker {
        raw()
    }
    fn noop(_: *const ()) {}
    fn raw() -> core::task::RawWaker {
        core::task::RawWaker::new(
            core::ptr::null(),
            &core::task::RawWakerVTable::new(clone, noop, noop, noop),
        )
    }
    // SAFETY: every vtable function is a genuine no-op; this waker is
    // never actually woken (the future it drives never returns Pending).
    unsafe { core::task::Waker::from_raw(raw()) }
}

use core::future::Future;

fn print_dec(mut n: usize) {
    if n == 0 {
        rivet::console::write_str("0");
        return;
    }
    let mut digits = [0u8; 20];
    let mut i = 0;
    while n > 0 {
        digits[i] = b'0' + (n % 10) as u8;
        n /= 10;
        i += 1;
    }
    let mut buf = [0u8; 20];
    for j in 0..i {
        buf[j] = digits[i - 1 - j];
    }
    if let Ok(s) = core::str::from_utf8(&buf[..i]) {
        rivet::console::write_str(s);
    }
}

#[rivet::main]
fn main() -> ! {
    rivet::console::write_str("Rivet embedded_hal_test\n");
    let _ = rivet::spawn_ptask!(stack = 1024, priority = 1, entry = test_task, arg = ());
    rivet::run();
}
