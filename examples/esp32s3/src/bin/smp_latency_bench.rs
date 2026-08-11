//! Real-time characterization: SMP cross-core wakeup latency (ESP32-S3
//! only — the only board in this workspace with real dual-core hardware).
//!
//! **Forced, not opportunistic.** An earlier version of this bench let two
//! equal-priority tasks (`holder`/`waiter`) migrate freely and just
//! observed which core each wake landed on — across 2000 reps, every
//! single one stayed same-core: whichever hart ran the unlocking task
//! could service its own reschedule check faster than the cross-core IPI
//! to the other hart could be serviced, so that hart always "won" the
//! newly-woken task before the other hart got a look-in. This version
//! forces the cross-core case structurally: `holder` runs at a priority
//! *above* `waiter` and never blocks or yields after unlocking (an
//! infinite busy-spin at that priority) — so holder's own hart can never
//! locally reschedule to `waiter` (a lower-priority task can't preempt a
//! higher-priority one that's still ready), and the only hart that *can*
//! ever run `waiter` is the other one, dispatched purely via
//! `wake_other_harts`'s cross-core reschedule IPI
//! (`rivet::preempt::sched::ready_add` → `request_reschedule_on`). Every
//! sample this version records is therefore a genuine cross-core wake by
//! construction, not by luck.

#![no_std]
#![no_main]

use rivet_bsp_esp32s3 as _;
use rivet_rt as _;

use core::sync::atomic::{AtomicU32, Ordering};
use rivet::preempt::PriorityMutex;

const REPS: u32 = 1000;
const MAX_HARTS: usize = rivet::config::MAX_HARTS;
/// A gap between holder's unlock and its next lock attempt, long enough
/// for the cross-core IPI + waiter's own dispatch to genuinely complete
/// (measured in practice to need headroom over the wake latency itself —
/// too short and holder just wins the re-lock race every time, starving
/// waiter and producing zero samples, the same failure mode this bench
/// exists to avoid).
const HOLDER_GAP_CYCLES: u32 = 20_000;

static MTX: PriorityMutex<u32> = PriorityMutex::new(0);
static UNLOCK_HART: AtomicU32 = AtomicU32::new(0);
// Phase 30: this used to be a raw `cycle_count()` (Xtensa CCOUNT) value
// compared against a `cycle_count()` read on *waiter*'s hart. That is
// invalid on Xtensa: CCOUNT is a per-core register with no cross-core
// synchronization guarantee (unlike, say, an invariant TSC) — the two
// harts' counters can differ by an arbitrary, unbounded offset, so any
// subtraction across cores produces a number with no physical meaning.
// This was the root cause of the previously observed "avg < min"
// (statistically impossible) results. Root-caused by inspecting
// `rivet-bsp-esp32s3`'s own `now_us()`, which is *also* built directly on
// `xtensa_lx::timer::get_cycle_count()` (i.e. the same per-core CCOUNT) —
// so it is not a usable cross-core clock either, on this board.
//
// Fix: stop trying to correlate absolute timestamps across cores at all.
// Instead measure the wake latency entirely from *waiter*'s own,
// self-consistent clock: the delta between "waiter is about to attempt
// the lock" and "waiter's lock() call returned" on waiter's own hart.
// Since waiter is genuinely blocked (not spinning) for that whole
// interval, this delta *is* the cross-core wake latency (block + IPI +
// dispatch + scheduler overhead), and it needs no cross-core comparison
// to be valid.
static CROSS_CORE_MIN: AtomicU32 = AtomicU32::new(u32::MAX);
static CROSS_CORE_MAX: AtomicU32 = AtomicU32::new(0);
// Widened to u64: 1000 samples of ~10-20M cycles each would overflow
// AtomicU32 (max ~4.29e9), previously producing a silently wrapped,
// nonsensical average ("avg < min"). RV32/Xtensa have no native
// `AtomicU64` (see `rivet::exec_time`'s module docs for the same gap),
// so this follows the same established `static mut u64` +
// `critical::enter` discipline rather than a real atomic. Only ever
// touched from `waiter`, single-hart, so the critical section here is
// solely to make the 64-bit read/modify/write torn-write-free against
// itself if this were ever preempted mid-update — not for cross-hart
// synchronization (nothing else touches it).
static mut CROSS_CORE_SUM: u64 = 0;
static CROSS_CORE_N: AtomicU32 = AtomicU32::new(0);
static SAME_CORE_N: AtomicU32 = AtomicU32::new(0);
static STOP: AtomicU32 = AtomicU32::new(0);
static N_FILLER_RUNNING: usize = 2 * MAX_HARTS;
static FILLER_COUNTERS: [AtomicU32; 4] = [const { AtomicU32::new(0) }; 4];
static HOLDER_ITERS: AtomicU32 = AtomicU32::new(0);

struct Unit;
static UNIT: Unit = Unit;

fn print_u64(mut n: u64) {
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

fn filler(counter: &'static AtomicU32) -> ! {
    // Priority 1 — strictly below `waiter`'s priority 3, so this never
    // blocks `waiter` from being dispatched on whichever hart it's on;
    // its only job is to keep that hart looking "busy" the same way a
    // real deployment's background work would, not to interfere.
    while STOP.load(Ordering::Acquire) == 0 {
        counter.fetch_add(1, Ordering::Relaxed);
        core::hint::spin_loop();
    }
    rivet::preempt::park_forever();
}

fn holder(_: &'static Unit) -> ! {
    // Priority 5: strictly above `waiter`'s 3. Never blocks, never
    // yields — occupies its own hart at the highest priority in the
    // system for the entire test, which is exactly what forces every
    // `waiter` wake to be a genuine cross-core dispatch (see module docs).
    //
    // Runs forever (`waiter`, not this loop, decides when the test ends
    // and sets `STOP`) — an earlier version bounded this loop at `REPS`
    // iterations of its own, and since holder's own re-lock never
    // actually has to wait for anything (nothing else besides `waiter`
    // ever touches this mutex, and `waiter` only gets a turn on the rare
    // cross-core-IPI-driven wake), holder finished all `REPS` iterations
    // almost immediately — ending the test after `waiter` had gotten only
    // one or two real chances, not real cross-core sample collection.
    while STOP.load(Ordering::Acquire) == 0 {
        let g = MTX.lock();
        HOLDER_ITERS.fetch_add(1, Ordering::Relaxed);
        for _ in 0..300u32 {
            core::hint::spin_loop();
        }
        UNLOCK_HART.store(rivet::port::arch::hart_id() as u32, Ordering::SeqCst);
        drop(g);
        // The gap: gives waiter genuine time to be cross-core-dispatched
        // and win the next lock race before holder tries again.
        let t0 = rivet::port::arch::cycle_count();
        while (rivet::port::arch::cycle_count().wrapping_sub(t0) as u32) < HOLDER_GAP_CYCLES {
            core::hint::spin_loop();
        }
    }
    rivet::preempt::park_forever();
}

fn waiter(_: &'static Unit) -> ! {
    // Paced, not generation-tracked: an earlier version tried a shared
    // "unlock generation" counter so a re-acquisition of an already-free
    // mutex (within the same holder gap) could be told apart from a
    // genuine fresh wake — it reliably crashed (LoadProhibited,
    // reproducible byte-for-byte across rebuilds and stack-size changes,
    // not investigated further given time cost). This simpler approach
    // sidesteps the whole class of bug: after each sample, wait longer
    // than holder's own gap before trying to lock again, so by
    // construction there is at most one attempt per holder cycle — no
    // shared "have I seen this one" state needed at all.
    let mut n = 0u32;
    while n < REPS {
        // Self-consistent measurement (Phase 30): both reads below are on
        // *this* hart's own clock, taken immediately around the blocking
        // call. `attempt_at` is read right before `lock()` is attempted;
        // since waiter is genuinely blocked (not spinning) whenever the
        // mutex isn't free, the delta to the moment `lock()` returns is
        // the real end-to-end wake latency (cross-core IPI + dispatch +
        // scheduler overhead) — no cross-core timestamp correlation
        // needed, so it can't be invalidated by Xtensa's per-core CCOUNT.
        let attempt_at = rivet::port::arch::cycle_count() as u32;
        let g = MTX.lock();
        let acquired_at = rivet::port::arch::cycle_count() as u32;
        let wake_hart = rivet::port::arch::hart_id() as u32;
        let unlock_hart = UNLOCK_HART.load(Ordering::SeqCst);
        drop(g);
        n += 1;

        let latency = acquired_at.wrapping_sub(attempt_at);
        if wake_hart == unlock_hart {
            // Should be structurally impossible (holder's hart can never
            // locally reschedule to a lower-priority ready task) — tallied
            // rather than asserted away, so a violation shows up as data,
            // not a silent pass.
            SAME_CORE_N.fetch_add(1, Ordering::Relaxed);
        } else {
            // SAFETY: `CROSS_CORE_SUM` is only ever touched from this task
            // (single-hart, single-writer); `critical::enter` here exists
            // solely to make the 64-bit RMW torn-write-free against a
            // hypothetical local preemption mid-update, not for cross-hart
            // synchronization (see the static's own doc comment).
            rivet::critical::enter(|| unsafe {
                CROSS_CORE_SUM += latency as u64;
            });
            CROSS_CORE_N.fetch_add(1, Ordering::Relaxed);
            CROSS_CORE_MIN.fetch_min(latency, Ordering::Relaxed);
            CROSS_CORE_MAX.fetch_max(latency, Ordering::Relaxed);
        }

        // Outpace holder's own gap so the next `MTX.lock()` above lands
        // after holder has re-locked+re-held+re-unlocked at least once —
        // i.e. against a genuinely fresh unlock, not the one just consumed.
        let t0 = rivet::port::arch::cycle_count();
        while (rivet::port::arch::cycle_count().wrapping_sub(t0) as u32)
            < HOLDER_GAP_CYCLES + 2_000
        {
            core::hint::spin_loop();
        }
    }
    // Waiter drives the test's own length — holder just runs forever as
    // an obstacle (see its own comment) — so this is the only place
    // `STOP` gets set.
    STOP.store(1, Ordering::Release);

    rivet::console::write_str("=== smp_latency_bench (forced cross-core mutex wake, cycles) ===\n");
    let cc_n = CROSS_CORE_N.load(Ordering::Relaxed);
    rivet::console::write_str("cross_core: n=");
    print_u64(cc_n as u64);
    if cc_n > 0 {
        rivet::console::write_str(" min=");
        print_u64(CROSS_CORE_MIN.load(Ordering::Relaxed) as u64);
        rivet::console::write_str(" max=");
        print_u64(CROSS_CORE_MAX.load(Ordering::Relaxed) as u64);
        rivet::console::write_str(" avg=");
        // SAFETY: waiter (this task) is the sole writer of `CROSS_CORE_SUM`
        // and has already finished its last write by this point (the
        // sampling loop above has ended) — read is race-free without a
        // critical section, but wrapped anyway for consistency with the
        // read/modify/write discipline used elsewhere on this static.
        let sum = rivet::critical::enter(|| unsafe { CROSS_CORE_SUM });
        print_u64(sum / cc_n as u64);
    }
    rivet::console::write_str("\nsame_core_unexpected: n=");
    print_u64(SAME_CORE_N.load(Ordering::Relaxed) as u64);
    rivet::console::write_str("\nholder_iters=");
    print_u64(HOLDER_ITERS.load(Ordering::Relaxed) as u64);
    rivet::console::write_str("\n=== end smp_latency_bench ===\n");

    rivet::report();

    if cc_n == 0 {
        rivet::console::write_str("SMP_LATENCY_BENCH_NO_CROSS_CORE_SAMPLES\n");
        rivet::exit_failure(1);
    }
    rivet::console::write_str("SMP_LATENCY_BENCH_OK\n");
    rivet::exit_success();
}

fn watchdog(_: &'static Unit) -> ! {
    // Diagnostic safety net: if `waiter` never reaches `REPS` within a
    // generous wall-clock bound, print whatever progress was made instead
    // of just silently hanging past the capture window. Includes full
    // min/max/avg, not just counts.
    rivet::preempt::sleep_ms(5_000);
    if STOP.load(Ordering::Acquire) == 0 {
        rivet::console::write_str("SMP_LATENCY_BENCH_WATCHDOG_FIRED\n");
        rivet::console::write_str("holder_iters=");
        print_u64(HOLDER_ITERS.load(Ordering::Relaxed) as u64);
        let cc_n = CROSS_CORE_N.load(Ordering::Relaxed);
        rivet::console::write_str(" cross_core_n=");
        print_u64(cc_n as u64);
        if cc_n > 0 {
            rivet::console::write_str(" cross_core_min=");
            print_u64(CROSS_CORE_MIN.load(Ordering::Relaxed) as u64);
            rivet::console::write_str(" cross_core_max=");
            print_u64(CROSS_CORE_MAX.load(Ordering::Relaxed) as u64);
            rivet::console::write_str(" cross_core_avg=");
            // SAFETY: watchdog runs concurrently with waiter (possibly on
            // the other hart) while waiter may still be mid-update here —
            // unlike the end-of-test read above, this one genuinely needs
            // `critical::enter`'s cross-hart exclusion, not just style
            // consistency.
            let sum = rivet::critical::enter(|| unsafe { CROSS_CORE_SUM });
            print_u64(sum / cc_n as u64);
        }
        rivet::console::write_str(" same_core_unexpected_n=");
        print_u64(SAME_CORE_N.load(Ordering::Relaxed) as u64);
        rivet::console::write_str("\n");
        rivet::exit_failure(2);
    }
    rivet::preempt::park_forever();
}

#[rivet::main]
fn main() -> ! {
    rivet::console::write_str("Rivet smp_latency_bench (forced cross-core)\n");

    // 16384, not the original 4096: `rivet-arch-xtensa::timer`'s
    // `on_timer_irq` doc comment covers why — the cross-hart `CONTEXTS`
    // race fix adds real stack usage to every dispatch, and 4096 no
    // longer had enough headroom once that fix landed.
    let _ = rivet::spawn_ptask!(stack = 16384, priority = 10, entry = watchdog, arg = UNIT);
    let _ = rivet::spawn_ptask!(stack = 16384, priority = 3, entry = waiter, arg = UNIT);
    let _ = rivet::spawn_ptask!(stack = 16384, priority = 5, entry = holder, arg = UNIT);

    rivet::run();
}
