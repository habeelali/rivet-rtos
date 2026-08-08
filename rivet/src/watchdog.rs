//! Watchdog policy — arch/board-independent.
//!
//! The actual watchdog hardware (or lack of it) is entirely a board fact,
//! reached through [`crate::port::board`]: [`init`]/[`feed`] forward to
//! `__rivet_board_wdt_init`/`__rivet_board_wdt_feed`; boards with a real
//! hardware watchdog implement those directly, boards without one arm a
//! software deadline and check it from `__rivet_board_wdt_check` (called
//! every tick via [`on_tick`]) — see `docs/porting.md` for the full
//! contract. Note independence: a hardware watchdog keeps counting even
//! if the CPU wedges with interrupts off; a software one, checked from the
//! tick, cannot catch a hang that stops ticks.
//!
//! Task-level watchdogs are a separate, purely kernel-side mechanism: a
//! task that calls [`checkin`] in its main loop opts in; the tick handler
//! flags it if it goes silent longer than [`enable_checkins`].

use core::sync::atomic::{AtomicU32, Ordering};

/// Task checkin timeout in microseconds (0 = task checkins disabled).
static CHECKIN_TIMEOUT_US: AtomicU32 = AtomicU32::new(0);

/// Initialize the watchdog with the given period.
pub fn init(period: crate::time::Duration) {
    crate::port::board::wdt_init(period.as_micros() as u32);
}

/// Kick the watchdog. Call periodically from the main loop / a high-
/// priority task.
pub fn feed() {
    crate::port::board::wdt_feed();
}

/// Called by the tick handler every tick: gives the board a chance to
/// check a software watchdog deadline, then scans task-level checkins.
pub fn on_tick() {
    crate::port::board::wdt_check();
    check_task_checkins();
}

/// Opt a task into the task-level watchdog: record the current time as its
/// last checkin. Called periodically from the task's own main loop.
pub fn checkin() {
    if CHECKIN_TIMEOUT_US.load(Ordering::Acquire) == 0 {
        return;
    }
    if let Some(id) = crate::preempt::sched::current() {
        if let Some(t) = crate::preempt::tcb::get(id) {
            t.last_checkin
                .store(crate::port::board::now_us() as u32, Ordering::Release);
        }
    }
}

/// Enable task-level checkin monitoring with the given timeout.
pub fn enable_checkins(timeout: crate::time::Duration) {
    CHECKIN_TIMEOUT_US.store(timeout.as_micros() as u32, Ordering::Release);
}

/// Scan tasks that opted into checkins; reset if any has been silent too
/// long. (Per-task *isolation* of an unresponsive task is separate fault-
/// policy work; here the recovery is a diagnosed reset.)
fn check_task_checkins() {
    let timeout = CHECKIN_TIMEOUT_US.load(Ordering::Acquire);
    if timeout == 0 {
        return;
    }
    let now = crate::port::board::now_us() as u32;
    for (id, t) in crate::preempt::tcb::TASKS.iter().enumerate() {
        if !t.used.load(Ordering::Acquire) {
            continue;
        }
        let last = t.last_checkin.load(Ordering::Acquire);
        if last != 0 && now.wrapping_sub(last) > timeout {
            crate::console::write_str("RIVET TASK CHECKIN TIMEOUT task=");
            print_dec(id);
            crate::console::write_str("\n");
            crate::port::board::reset();
        }
    }
}

fn print_dec(mut n: usize) {
    if n == 0 {
        crate::console::write_str("0");
        return;
    }
    let mut digits = [0u8; 10];
    let mut i = 0;
    while n > 0 {
        digits[i] = b'0' + (n % 10) as u8;
        n /= 10;
        i += 1;
    }
    let mut out = [0u8; 10];
    for j in 0..i {
        out[j] = digits[i - 1 - j];
    }
    if let Ok(s) = core::str::from_utf8(&out[..i]) {
        crate::console::write_str(s);
    }
}

/// Test-only reset (host).
#[cfg(feature = "test-support")]
pub(crate) fn reset_for_test() {
    CHECKIN_TIMEOUT_US.store(0, Ordering::Release);
}
