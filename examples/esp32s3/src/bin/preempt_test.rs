//! Rivet RTOS on real ESP32-S3 hardware (plan.md Phase 23) — proof of real
//! preemptive context switching, not just the single-task bootstrap path
//! `demo.rs` exercises: two same-priority tasks, neither ever yielding or
//! blocking, interleaved purely by the tick-driven `CONTEXTS`
//! save/restore path (`__level_3_interrupt`'s "candidate has been
//! interrupted at least once before" branch) — the one code path Phase 22
//! never actually reached on hardware (a single task never triggers a real
//! switch, only the one-time bootstrap).

#![no_std]
#![no_main]

use rivet_bsp_esp32s3 as _;
use rivet_rt as _;

struct SpinArg {
    label: u8,
}
static ARG_A: SpinArg = SpinArg { label: b'A' };
static ARG_B: SpinArg = SpinArg { label: b'B' };

static PROGRESS_A: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
static PROGRESS_B: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);

fn spin_task(arg: &'static SpinArg) -> ! {
    let counter = if arg.label == b'A' {
        &PROGRESS_A
    } else {
        &PROGRESS_B
    };
    let mut rounds: u32 = 0;
    loop {
        let count = counter.fetch_add(1, core::sync::atomic::Ordering::Relaxed) + 1;
        if count % 200_000 == 0 {
            let s = [arg.label];
            if let Ok(s) = core::str::from_utf8(&s) {
                rivet::console::write_str(s);
            }
            rounds += 1;
            if rounds >= 12 {
                rivet::console::write_str(if arg.label == b'A' { "\nA_DONE\n" } else { "\nB_DONE\n" });
                rivet::preempt::park_forever();
            }
        }
    }
}

#[rivet::main]
fn main() -> ! {
    rivet::console::write_str("Rivet esp32s3 preempt_test: two same-priority tasks (A, B)\n");
    let _ = rivet::spawn_ptask!(stack = 1024, priority = 1, entry = spin_task, arg = ARG_A);
    let _ = rivet::spawn_ptask!(stack = 1024, priority = 1, entry = spin_task, arg = ARG_B);
    rivet::run();
}
