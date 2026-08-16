#![no_std]
#![no_main]
//! Real-time characterisation suite for the AArch64 / Pi 3B port.
//!
//! Every figure is min / mean / max over many samples, in nanoseconds.
//! A mean alone says almost nothing about a real-time system, and a
//! maximum without a sample count says little more, so both are always
//! printed with `n`.
//!
//! # Timebase
//!
//! `CNTPCT_EL0`, the architected counter, fixed at 19.2 MHz on this SoC.
//! One tick is 52.083 ns and it does not move with CPU frequency, which
//! matters because the cluster clock is owned by Linux. Conversion is
//! exact: `ns = ticks * 625 / 12`.
//!
//! The counter's resolution is also the floor on what can be measured.
//! Anything reported as 52 ns is one tick, meaning "at or below the
//! resolution of the instrument", not "exactly 52 ns". The suite measures
//! a bare counter read first so that floor is visible rather than
//! implied.
//!
//! # What is and is not measured
//!
//! These are black-box measurements under a controlled load, not proven
//! WCET bounds. See `docs/realtime.md` for the methodology this number
//! set should be read against. Two figures here depend on Linux
//! participating and print as skipped otherwise: the doorbell latency and
//! the ring round trip.
//!
//! Run it under both conditions. Idle numbers describe the mechanism;
//! numbers taken while Linux saturates its three cores describe what the
//! arrangement is actually worth, since that is the case core isolation
//! exists to survive.

use core::sync::atomic::{AtomicBool, AtomicU32, AtomicU64, Ordering};

use rivet::preempt::PriorityMutex;
use rivet::sync::Semaphore;
use rivet_arch_aarch64 as _;
use rivet_bsp_rpi3b::{kernel, shmem};

// ── Timebase ──────────────────────────────────────────────────────

fn cntpct() -> u64 {
    let v: u64;
    // SAFETY: reading the counter has no side effects. The ISB stops the
    // read being hoisted above the work being timed.
    unsafe {
        core::arch::asm!("isb", "mrs {}, cntpct_el0", out(reg) v,
                         options(nomem, nostack, preserves_flags))
    };
    v
}

/// Exact for 19.2 MHz: 1e9 / 19.2e6 = 625 / 12.
fn ns(ticks: u64) -> u64 {
    ticks.saturating_mul(625) / 12
}

// ── Reporting ─────────────────────────────────────────────────────

/// One microsecond, in counter ticks, rounded up.
///
/// The bar for "this sample ran long". Fixed rather than relative to the
/// row's own minimum, so the column means the same thing in every row and
/// the counts can be compared across them.
const LONG: u64 = 20;

struct Stats {
    min: u64,
    max: u64,
    sum: u64,
    n: u64,
    /// Samples over `LONG`. On a tight distribution the integer mean
    /// collapses onto the minimum and stops carrying information, leaving
    /// the maximum as the only informative column. But a maximum says
    /// nothing about frequency, and two samples at 3 us out of 20000 is a
    /// very different system from two thousand of them.
    over: u64,
}

impl Stats {
    const fn new() -> Self {
        Stats {
            min: u64::MAX,
            max: 0,
            sum: 0,
            n: 0,
            over: 0,
        }
    }
    fn add(&mut self, v: u64) {
        if v < self.min {
            self.min = v;
        }
        if v > self.max {
            self.max = v;
        }
        self.sum += v;
        self.n += 1;
        if v > LONG {
            self.over += 1;
        }
    }
    fn mean(&self) -> u64 {
        self.sum.checked_div(self.n).unwrap_or(0)
    }
}

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

fn pad(label: &str, width: usize) {
    w(label);
    for _ in 0..width.saturating_sub(label.len()) {
        w(" ");
    }
}

/// A row without the long-sample column, for quantities where a
/// one-microsecond bar is meaningless because every sample clears it.
fn row_plain(label: &str, s: &Stats) {
    row_inner(label, s, false)
}

/// One row of the report, converting ticks to nanoseconds.
fn row(label: &str, s: &Stats) {
    row_inner(label, s, true)
}

fn row_inner(label: &str, s: &Stats, show_over: bool) {
    w("  ");
    pad(label, 30);
    if s.n == 0 {
        w("no samples\n");
        return;
    }
    w("min ");
    dec(ns(s.min));
    w("  mean ");
    dec(ns(s.mean()));
    w("  max ");
    dec(ns(s.max));
    w(" ns   n=");
    dec(s.n);
    if show_over {
        w("  >1us=");
        dec(s.over);
    }
    w("\n");
}

/// A row already in microseconds, for figures that are naturally coarse.
fn row_us(label: &str, s: &Stats) {
    w("  ");
    pad(label, 30);
    if s.n == 0 {
        w("no samples\n");
        return;
    }
    w("min ");
    dec(s.min);
    w("  mean ");
    dec(s.mean());
    w("  max ");
    dec(s.max);
    w(" us   n=");
    dec(s.n);
    w("\n");
}

fn skipped(label: &str, why: &str) {
    w("  ");
    pad(label, 30);
    w("skipped: ");
    w(why);
    w("\n");
}

// ── Shared state for the multi-task benchmarks ────────────────────

const PINGPONG: u64 = 2000;
static TURN_B: AtomicBool = AtomicBool::new(false);

static SEM: Semaphore<1> = Semaphore::new(1);
static MTX: PriorityMutex<u64> = PriorityMutex::new(0);

/// Handoff timing for the contended mutex: the holder stamps the moment
/// before it unlocks, the waiter stamps the moment it acquires.
static MTX_RELEASED_AT: AtomicU64 = AtomicU64::new(0);
static MTX_HANDOFF: AtomicU64 = AtomicU64::new(0);
static MTX_ROUNDS: AtomicU32 = AtomicU32::new(0);
static MTX_HOLDER_GO: AtomicBool = AtomicBool::new(false);

static SIG: rivet::sync::Signal = rivet::sync::Signal::new();
static SIGNAL_AT: AtomicU64 = AtomicU64::new(0);
static WOKE_AT: AtomicU64 = AtomicU64::new(0);

/// Doorbell latency, filled in by the command handler when Linux sends a
/// timestamped ping. Both sides read the same architected counter
/// (`CNTVCT_EL0` from Linux, `CNTPCT_EL0` here, and `CNTVOFF_EL2` is zero
/// on this board), so the difference is a true one-way latency rather
/// than two clocks being compared.
static DB_MIN: AtomicU64 = AtomicU64::new(u64::MAX);
static DB_MAX: AtomicU64 = AtomicU64::new(0);
static DB_SUM: AtomicU64 = AtomicU64::new(0);
static DB_N: AtomicU64 = AtomicU64::new(0);
static DB_OVER: AtomicU64 = AtomicU64::new(0);

fn record_db(v: u64) {
    DB_MAX.fetch_max(v, Ordering::Relaxed);
    let mut cur = DB_MIN.load(Ordering::Relaxed);
    while v < cur {
        match DB_MIN.compare_exchange_weak(cur, v, Ordering::Relaxed, Ordering::Relaxed) {
            Ok(_) => break,
            Err(a) => cur = a,
        }
    }
    DB_SUM.fetch_add(v, Ordering::Relaxed);
    DB_N.fetch_add(1, Ordering::Relaxed);
    if v > LONG {
        DB_OVER.fetch_add(1, Ordering::Relaxed);
    }
}

// ── Helper tasks ──────────────────────────────────────────────────

/// Runs at the suite's own priority. A lower-priority partner would never
/// be scheduled while the suite is spinning, and the measurement would hang
/// rather than complete.
fn pong(_: &'static ()) -> ! {
    let mut n = 0u64;
    while n < PINGPONG {
        if TURN_B.load(Ordering::Acquire) {
            TURN_B.store(false, Ordering::Release);
            n += 1;
        }
        rivet::port::arch::request_reschedule();
    }
    rivet::preempt::park_forever();
}

/// Low-priority mutex holder, so the high-priority waiter below has
/// something to be blocked by and therefore something to inherit from.
fn mtx_holder(_: &'static ()) -> ! {
    loop {
        if !MTX_HOLDER_GO.load(Ordering::Acquire) {
            rivet::preempt::sleep_ms(1);
            continue;
        }
        let g = MTX.lock();
        MTX_HOLDER_GO.store(false, Ordering::Release);
        // Hold briefly so the waiter genuinely blocks rather than
        // winning the lock uncontended.
        rivet::preempt::sleep_ms(2);
        // Stamp immediately before releasing: the gap from here to the
        // waiter running is the handoff, and nothing else.
        MTX_RELEASED_AT.store(cntpct(), Ordering::Release);
        drop(g);
        rivet::preempt::sleep_ms(1);
    }
}

/// High-priority waiter. Blocking on a lock held by a lower-priority task
/// is exactly the inversion priority inheritance exists to bound.
fn mtx_waiter(_: &'static ()) -> ! {
    loop {
        if MTX_RELEASED_AT.load(Ordering::Acquire) != 0 || !MTX_HOLDER_GO.load(Ordering::Acquire) {
            let g = MTX.lock();
            let got = cntpct();
            let rel = MTX_RELEASED_AT.swap(0, Ordering::AcqRel);
            if rel != 0 && got > rel {
                MTX_HANDOFF.store(got - rel, Ordering::Release);
                MTX_ROUNDS.fetch_add(1, Ordering::Release);
            }
            drop(g);
        }
        rivet::preempt::sleep_ms(1);
    }
}

#[rivet::task(priority = 1, stack = 2048)]
async fn signal_waiter() {
    loop {
        SIG.wait().await;
        WOKE_AT.store(cntpct(), Ordering::Release);
        SIG.reset();
    }
}

/// Consumes commands from Linux, which is what makes the doorbell and
/// round-trip figures possible at all.
#[rivet::task(priority = 3, stack = 4096)]
async fn commands() {
    let mut buf = [0u8; 512];
    loop {
        kernel::DOORBELL.wait().await;
        // Stamp on arrival, before any parsing, so the figure is the
        // wake path and not the work that follows it.
        let arrived = cntpct();
        kernel::DOORBELL.reset();
        loop {
            // SAFETY: the shared window is mapped and the rings are up.
            let n = unsafe { shmem::COMMAND.read_bytes(&mut buf) };
            if n == 0 {
                break;
            }
            let mut start = 0;
            for i in 0..n {
                if buf[i] == b'\n' {
                    handle(&buf[start..i], arrived);
                    start = i + 1;
                }
            }
            if start < n {
                handle(&buf[start..n], arrived);
            }
        }
    }
}

fn parse_u64(s: &[u8]) -> Option<u64> {
    let mut v: u64 = 0;
    let mut any = false;
    for &c in s {
        if c.is_ascii_digit() {
            v = v.checked_mul(10)?.checked_add((c - b'0') as u64)?;
            any = true;
        } else if any {
            break;
        }
    }
    any.then_some(v)
}

fn handle(cmd: &[u8], arrived: u64) {
    if let Some(rest) = cmd.strip_prefix(b"ts ") {
        // Linux stamped the counter just before ringing the bell.
        if let Some(sent) = parse_u64(rest) {
            if arrived > sent {
                record_db(arrived - sent);
            }
        }
        // Echo so the sender can close a round trip.
        w("[echo]\n");
    } else if let Some(rest) = cmd.strip_prefix(b"flood ") {
        // Fill the console ring so the Linux side can time its own drain.
        // rivet outruns the reader here by a wide margin, and that is the
        // point: the number this produces is the consumer's rate, and any
        // shortfall is bytes the reader could not keep up with.
        let kib = parse_u64(rest).unwrap_or(64).min(4096);
        let line = [b'.'; 64];
        for _ in 0..(kib * 16) {
            // SAFETY: every byte is ASCII, so this stays a text stream.
            w(unsafe { core::str::from_utf8_unchecked(&line) });
        }
    }
}

// ── The suite ─────────────────────────────────────────────────────

fn suite(_: &'static ()) -> ! {
    w("\n==== rivet rpi3b real-time characterisation ====\n");
    w("timebase CNTPCT_EL0 @ 19.2 MHz, 1 tick = 52.083 ns\n");
    w("figures are min / mean / max; 52 ns means at-or-below resolution\n\n");

    rivet::preempt::sleep_ms(300); // settle

    const N: u64 = 20_000;

    // -- instrument floor -------------------------------------------
    w("instrument\n");
    let mut floor = Stats::new();
    for _ in 0..N {
        let t0 = cntpct();
        floor.add(cntpct() - t0);
    }

    row("counter read (floor)", &floor);

    // -- scheduler ---------------------------------------------------
    w("\nscheduler\n");
    let mut resched = Stats::new();
    for _ in 0..N {
        let t0 = cntpct();
        rivet::port::arch::request_reschedule();
        resched.add(cntpct() - t0);
    }

    row("reschedule, no task change", &resched);

    // Two switches per exchange, out to the other task and back.
    let t0 = cntpct();
    let mut n = 0u64;
    while n < PINGPONG {
        TURN_B.store(true, Ordering::Release);
        while TURN_B.load(Ordering::Acquire) {
            rivet::port::arch::request_reschedule();
        }
        n += 1;
    }
    let ctx_total = cntpct() - t0;
    let ctx_each = ctx_total / (PINGPONG * 2);

    w("  ");
    pad("task-to-task switch", 30);
    w("mean ");
    dec(ns(ctx_each));
    w(" ns   n=");
    dec(PINGPONG * 2);
    w("\n");

    // -- synchronisation primitives ----------------------------------
    w("\nsynchronisation\n");
    let mut sem = Stats::new();
    for _ in 0..N {
        let t0 = cntpct();
        let ok = SEM.try_acquire();
        if ok {
            SEM.release();
        }
        sem.add(cntpct() - t0);
    }

    row("semaphore try/release", &sem);

    let mut mtx_un = Stats::new();
    for _ in 0..N {
        let t0 = cntpct();
        if let Some(g) = MTX.try_lock() {
            drop(g);
        }
        mtx_un.add(cntpct() - t0);
    }

    row("mutex try_lock/unlock", &mtx_un);

    // -- memory: cached against the uncached shared window -----------
    w("\nmemory and IPC\n");
    //
    // The shared window is Device-nGnRnE so that Linux and rivet agree on
    // visibility without cache maintenance. That agreement has a price,
    // and this is it: the same access pattern against Normal cacheable
    // memory versus the shared window.
    let mut cached = Stats::new();
    let mut uncached = Stats::new();
    let mut scratch = [0u64; 64];
    let shared = shmem::SHARED_BASE + 0x1F_0000; // clear of every ring
    for _ in 0..2_000 {
        let t0 = cntpct();
        for (i, slot) in scratch.iter_mut().enumerate() {
            // SAFETY: writing our own stack array.
            unsafe { core::ptr::write_volatile(slot, i as u64) };
        }
        cached.add(cntpct() - t0);

        let t0 = cntpct();
        for i in 0..64usize {
            // SAFETY: inside the mapped shared window, past every ring.
            unsafe { core::ptr::write_volatile((shared + i * 8) as *mut u64, i as u64) };
        }
        uncached.add(cntpct() - t0);
    }
    core::hint::black_box(&scratch);

    // The other way to share Normal memory across the two tiers is to keep
    // it cacheable and clean the lines by hand. This is what that costs for
    // eight 64-byte lines, and it is the number that decides whether the
    // Device mapping above is actually the worse choice.
    let mut clean = Stats::new();
    let base = scratch.as_ptr() as usize;
    for _ in 0..2_000 {
        let t0 = cntpct();
        for line in 0..8usize {
            // SAFETY: cleaning cache lines backing our own stack array.
            unsafe {
                core::arch::asm!("dc cvac, {}", in(reg) base + line * 64,
                                 options(nostack, preserves_flags))
            };
        }
        // SAFETY: ordering barrier for the maintenance above.
        unsafe { core::arch::asm!("dsb ish", options(nostack, preserves_flags)) };
        clean.add(cntpct() - t0);
    }

    row("64 writes, Normal cached", &cached);
    row("64 writes, Device shared", &uncached);
    row("dc cvac x8 + dsb ish", &clean);

    // -- ring throughput ---------------------------------------------
    let payload = [0x5Au8; 1024];
    let t0 = cntpct();
    for _ in 0..256 {
        // SAFETY: the trace ring is initialised; this is a throughput
        // probe, and any reader is expected to see it as noise.
        unsafe { shmem::TRACE.write_bytes(&payload) };
    }
    let ring_total = cntpct() - t0;
    let ring_bytes = 256u64 * 1024;
    // MiB/s, computed in integers: bytes / seconds.
    let ring_mibs = (ring_bytes * 19_200_000) / (ring_total.max(1) * 1024 * 1024);

    w("  ");
    pad("ring write throughput", 30);
    dec(ring_mibs);
    w(" MiB/s over ");
    dec(ring_bytes / 1024);
    w(" KiB\n");

    // -- cross-tier wake ---------------------------------------------
    w("\nwakes\n");
    let mut sigw = Stats::new();
    for _ in 0..200 {
        WOKE_AT.store(0, Ordering::Release);
        SIGNAL_AT.store(cntpct(), Ordering::Release);
        SIG.signal();
        rivet::preempt::sleep_ms(2);
        let woke = WOKE_AT.load(Ordering::Acquire);
        let sent = SIGNAL_AT.load(Ordering::Acquire);
        if woke > sent {
            sigw.add(woke - sent);
        }
    }

    row("Signal to async task", &sigw);

    // -- contended mutex handoff -------------------------------------
    let mut mtx_ho = Stats::new();
    for _ in 0..100 {
        MTX_RELEASED_AT.store(0, Ordering::Release);
        let before = MTX_ROUNDS.load(Ordering::Acquire);
        MTX_HOLDER_GO.store(true, Ordering::Release);
        for _ in 0..40 {
            rivet::preempt::sleep_ms(1);
            if MTX_ROUNDS.load(Ordering::Acquire) != before {
                mtx_ho.add(MTX_HANDOFF.load(Ordering::Acquire));
                break;
            }
        }
    }

    row("mutex handoff, contended", &mtx_ho);

    // -- interrupts ---------------------------------------------------
    w("\ninterrupts\n");
    kernel::reset_irq_stats();
    rivet::preempt::sleep_ms(3000);
    let k = kernel::irq_stats();
    let irq = Stats {
        min: k.lat_min,
        max: k.lat_max,
        sum: k.lat_sum,
        n: k.count,
        over: k.lat_over,
    };
    let tick = Stats {
        min: k.cost_min,
        max: k.cost_max,
        sum: k.cost_sum,
        n: k.count,
        over: k.cost_over,
    };
    let gap = Stats {
        min: k.gap_min,
        max: k.gap_max,
        sum: k.gap_sum,
        n: k.gap_count,
        over: 0,
    };

    row("hardware to handler", &irq);
    row("tick handler cost", &tick);
    row_plain("tick-to-tick interval", &gap);

    // -- deadline behaviour -------------------------------------------
    w("\ndeadlines\n");
    //
    // Lateness against the absolute deadline, not gap between wakeups.
    // Gap is the difference of two quantisation errors and swings a whole
    // tick while nothing is actually late.
    let mut late = Stats::new();
    let period_us: u64 = 10_000;
    let mut next = rivet::port::board::now_us() + period_us;
    for i in 0..520u64 {
        let deadline = next;
        rivet::preempt::sleep_until(deadline);
        next += period_us;
        if i < 20 {
            continue; // discard the grid-alignment transient
        }
        late.add(rivet::port::board::now_us().saturating_sub(deadline));
    }

    row_us("lateness vs absolute deadline", &late);

    // -- the half that needs Linux ------------------------------------
    //
    // Everything above is self-contained. The doorbell and round-trip
    // figures are not: they describe a path that starts on the other side
    // of the machine, so they need the other side to be running. Hold a
    // window open and say so, rather than reporting a mechanism as absent
    // when nobody was asked to exercise it.
    w("\nIPC with Linux\n");
    w("  waiting up to 180 s. On Linux, detach the console reader and run:\n");
    w("    sudo rivet-amp bench\n");

    let mut quiet = 0u32;
    let mut seen = 0u64;
    for _ in 0..1800u32 {
        rivet::preempt::sleep_ms(100);
        let now = DB_N.load(Ordering::Relaxed);
        if now != seen {
            seen = now;
            quiet = 0;
        } else if seen > 0 {
            // Finish once the pings stop, rather than sitting out the
            // full timeout after the work is already done.
            quiet += 1;
            if quiet > 100 {
                break;
            }
        }
    }

    let dbn = DB_N.load(Ordering::Relaxed);
    if dbn > 0 {
        let db = Stats {
            min: DB_MIN.load(Ordering::Relaxed),
            max: DB_MAX.load(Ordering::Relaxed),
            sum: DB_SUM.load(Ordering::Relaxed),
            n: dbn,
            over: DB_OVER.load(Ordering::Relaxed),
        };
        row("Linux doorbell to task", &db);
        w("  (round trip and one-way throughput are reported by rivet-amp bench)\n");
    } else {
        skipped("Linux doorbell to task", "no timestamped pings arrived");
    }

    w("\nRT_BENCH_OK\n");
    rivet::exit_success();
}

#[no_mangle]
pub extern "C" fn rust_main(_dtb: u64) -> ! {
    // SAFETY: called once, from EL2, on the boot stack.
    unsafe { rivet_bsp_rpi3b::board_bringup() };
    rivet_bsp_rpi3b::publish_identity!();
    extern "C" {
        fn rivet_main() -> !;
    }
    // SAFETY: generated by `#[rivet::main]` below.
    unsafe { rivet_main() }
}

#[rivet::main]
fn main() -> ! {
    // SAFETY: called once, on the core that services the doorbell.
    unsafe { kernel::enable_doorbell() };
    let _ = rivet::spawn_ptask!(stack = 4096, priority = 2, entry = pong, arg = ());
    let _ = rivet::spawn_ptask!(stack = 4096, priority = 1, entry = mtx_holder, arg = ());
    let _ = rivet::spawn_ptask!(stack = 4096, priority = 4, entry = mtx_waiter, arg = ());
    let _ = rivet::spawn_ptask!(stack = 8192, priority = 2, entry = suite, arg = ());
    rivet::run();
}
