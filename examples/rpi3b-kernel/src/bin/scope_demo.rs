#![no_std]
#![no_main]
//! Puts the doorbell latency on three pins, so an oscilloscope can measure
//! it instead of taking rivet's word for it.
//!
//! Every figure in `docs/rpi3b-benchmarks.md` is the software timing
//! itself. That is a real measurement, but it is an argument rivet makes
//! on its own behalf, using a counter it reads with instructions it also
//! schedules. This binary makes the same interval visible to an instrument
//! outside the machine, on the one path where an external observer can see
//! both ends: the doorbell, whose start event happens on the Linux side.
//!
//! Wiring, all four pins adjacent in the corner of the 40-pin header:
//!
//! ```text
//!   header 37   GPIO26   ch C   rivet, in the woken task
//!   header 38   GPIO20   ch A   Linux, just before ringing the doorbell
//!   header 39   GND             ground for all three probes
//!   header 40   GPIO21   ch B   rivet, first thing in the interrupt handler
//! ```
//!
//! Trigger on A rising. Three intervals fall out of one capture:
//!
//! ```text
//!   A -> B   Linux's MMIO write to rivet's interrupt handler running
//!   B -> C   the scheduler waking the task that was awaiting the doorbell
//!   A -> C   the whole path, and the figure rt_bench reports as
//!            "Linux doorbell to task"
//! ```
//!
//! Run it against `rivet-amp scope`, which drives channel A. Run that
//! under Linux load too: the point of the exercise is that the interval
//! barely moves.
//!
//! # What the trace does and does not include
//!
//! A GPIO write on this SoC is a store to Device memory, so each edge
//! costs a trip to the peripheral bus. That cost is inside the measured
//! interval, on both sides, and it is why the scope reads slightly longer
//! than `rt_bench`, which stamps a counter register instead. Neither is
//! wrong. The scope figure includes the cost of being observed, which is
//! the honest number if you intend to react to the doorbell by driving a
//! pin, and the software figure is the honest one if you intend to react
//! to it in software.
//!
//! Both pins rivet drives are configured here, including the one Linux
//! drives. `GPFSEL` is a read-modify-write register and the two sides do
//! not coordinate, so if both configured their own pin the two updates
//! could lose each other. Setting all three from one side removes the
//! race entirely. The level registers need no such care: `GPSET` and
//! `GPCLR` are write-to-set and write-to-clear, so two cores driving
//! different pins in the same bank cannot interfere.

use core::sync::atomic::{AtomicU32, Ordering};

use rivet_arch_aarch64 as _;
use rivet_bsp_rpi3b::{gpio, kernel};

/// Linux raises this before ringing. rivet only configures it.
const PIN_TRIGGER: u8 = 20;
/// Raised in the doorbell interrupt handler.
const PIN_ISR: u8 = 21;
/// Raised in the task the doorbell wakes.
const PIN_TASK: u8 = 26;

/// How long the pins stay high after an event. Only the rising edges
/// carry the measurement; this exists so the pulse is wide enough to see
/// at a timebase that also shows a millisecond of context.
const PULSE_US: u64 = 200;

static PULSES: AtomicU32 = AtomicU32::new(0);

fn w(s: &str) {
    rivet::console::write_str(s);
}

fn dec(mut v: u64) {
    let mut buf = [0u8; 24];
    let mut i = buf.len();
    loop {
        i -= 1;
        buf[i] = b'0' + (v % 10) as u8;
        v /= 10;
        if v == 0 {
            break;
        }
    }
    // SAFETY: every byte written is an ASCII digit.
    w(unsafe { core::str::from_utf8_unchecked(&buf[i..]) });
}

#[rivet::task(priority = 3, stack = 4096)]
async fn responder() {
    loop {
        kernel::DOORBELL.wait().await;
        // Before anything else, including the reset below: this edge is
        // the measurement.
        // SAFETY: configured as an output in main, and only this task
        // drives it.
        unsafe { gpio::raise(PIN_TASK) };
        kernel::DOORBELL.reset();
        PULSES.fetch_add(1, Ordering::Relaxed);

        rivet::time::Sleep::<PULSE_US>::new().await;
        // SAFETY: as above. The ISR raised PIN_ISR and leaves clearing it
        // here, since the handler has nowhere to wait.
        unsafe {
            gpio::lower(PIN_TASK);
            gpio::lower(PIN_ISR);
        }
    }
}

fn reporter(_: &'static ()) -> ! {
    w("\n==== rivet rpi3b scope demo ====\n");
    w("header 38 / GPIO20  ch A  Linux, before ringing\n");
    w("header 40 / GPIO21  ch B  rivet, in the interrupt handler\n");
    w("header 37 / GPIO26  ch C  rivet, in the woken task\n");
    w("header 39           GND\n");
    w("trigger on A rising; A->B is interrupt latency, A->C is doorbell-to-task\n");
    w("drive it from Linux with: sudo rivet-amp scope 200\n\n");

    let mut last = 0u32;
    loop {
        rivet::preempt::sleep_ms(2000);
        let n = PULSES.load(Ordering::Relaxed);
        w("pulses=");
        dec(n as u64);
        w(" (+");
        dec((n - last) as u64);
        w(" in 2 s)\n");
        last = n;
    }
}

#[no_mangle]
pub extern "C" fn rust_main(_dtb: u64) -> ! {
    // SAFETY: called once, from EL2, on the boot stack.
    unsafe { rivet_bsp_rpi3b::board_bringup() };
    extern "C" {
        fn rivet_main() -> !;
    }
    // SAFETY: generated by `#[rivet::main]` below.
    unsafe { rivet_main() }
}

#[rivet::main]
fn main() -> ! {
    // SAFETY: none of these three pins has a function on a Pi 3B, and
    // nothing in this image or in Linux's device tree claims them.
    // Configuring the Linux-driven pin from here too is deliberate; see
    // the note at the top of this file.
    unsafe {
        gpio::set_function(PIN_TRIGGER, gpio::func::OUTPUT);
        gpio::set_function(PIN_ISR, gpio::func::OUTPUT);
        gpio::set_function(PIN_TASK, gpio::func::OUTPUT);
        gpio::lower(PIN_TRIGGER);
        gpio::lower(PIN_ISR);
        gpio::lower(PIN_TASK);
    }
    // SAFETY: PIN_ISR was just configured as an output above.
    unsafe {
        kernel::set_doorbell_scope_pin(PIN_ISR);
        kernel::enable_doorbell();
    }
    let _ = rivet::spawn_ptask!(stack = 4096, priority = 1, entry = reporter, arg = ());
    rivet::run();
}
