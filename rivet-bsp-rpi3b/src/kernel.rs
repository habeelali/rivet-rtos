//! Group B of the port contract: the board port for the Pi 3B.
//!
//! Clock, tick, console, reset and watchdog, plus the interrupt-controller
//! hooks `rivet-arch-aarch64` forwards to. See that crate's module docs
//! for why the controller lives here rather than there: BCM2837 has no
//! usable GIC, so there is nothing architectural to drive.
//!
//! Two independent timers are in play, which is worth keeping straight:
//!
//! - The **System Timer** at `0x3F003000` is a free-running 1 MHz counter,
//!   used for `now_us`. One microsecond per count means no scaling at all.
//! - The **architected generic timer** (`CNTP_*_EL0`, 19.2 MHz here)
//!   drives the periodic tick, routed to the core through the ARM-local
//!   block at `0x4000_0000`. It is per-core, which is what the eventual
//!   multi-core work wants, and it needs no MMIO to read.

use core::ptr::{read_volatile, write_volatile};
use core::sync::atomic::{AtomicU32, AtomicU64, Ordering};

use crate::mmio::PERIPHERAL_BASE;
use crate::Pl011;

/// Free-running 1 MHz counter.
const SYSTEM_TIMER_BASE: usize = PERIPHERAL_BASE + 0x0000_3000;
const ST_CLO: usize = 0x04;
const ST_CHI: usize = 0x08;

/// Broadcom's own interrupt controller. Not a GIC.
const IRQ_BASE: usize = PERIPHERAL_BASE + 0x0000_B200;
const ENABLE_IRQS_1: usize = 0x10;
const ENABLE_IRQS_2: usize = 0x14;
const DISABLE_IRQS_1: usize = 0x1C;
const DISABLE_IRQS_2: usize = 0x20;

/// Per-core timers and mailboxes. Outside the peripheral window.
const ARM_LOCAL_BASE: usize = 0x4000_0000;
/// Timer interrupt control and pending-source registers are banked per
/// core, four bytes apart. Reaching for the core 0 instances from
/// whichever core happens to be running is the obvious mistake here, and
/// it presents as a tick that simply never fires.
const CORE_TIMER_IRQCNTL_BASE: usize = 0x40;
const CORE_IRQ_SOURCE_BASE: usize = 0x60;

fn core_timer_irqcntl() -> usize {
    ARM_LOCAL_BASE + CORE_TIMER_IRQCNTL_BASE + crate::smp::current_core() * 4
}

fn core_irq_source() -> usize {
    ARM_LOCAL_BASE + CORE_IRQ_SOURCE_BASE + crate::smp::current_core() * 4
}
/// Non-secure physical timer, routed as IRQ rather than FIQ.
const TIMER_IRQ_CNTPNSIRQ: u32 = 1 << 1;

/// Power management block, which owns the watchdog and the reset path.
const PM_BASE: usize = PERIPHERAL_BASE + 0x0010_0000;
const PM_RSTC: usize = 0x1C;
const PM_WDOG: usize = 0x24;
const PM_PASSWORD: u32 = 0x5A00_0000;
const PM_RSTC_WRCFG_FULL_RESET: u32 = 0x20;
const PM_RSTC_WRCFG_CLR: u32 = 0xFFFF_FFCF;

/// Ticks elapsed since `tick_start`, and the period it was started with.
static TICK_PERIOD_US: AtomicU32 = AtomicU32::new(0);
static TICK_HZ: AtomicU32 = AtomicU32::new(0);

/// The tick rate the board was actually started at.
///
/// Published to Linux rather than left implicit: the rate lives in the
/// image's build configuration, so the Linux side has no way to know it
/// and every latency figure it reports depends on it.
pub fn configured_tick_hz() -> u32 {
    TICK_HZ.load(Ordering::Acquire)
}
macro_rules! read_sysreg {
    ($name:literal) => {{
        let v: u64;
        // SAFETY: reading a system register has no side effects.
        unsafe {
            core::arch::asm!(concat!("mrs {}, ", $name), out(reg) v,
                             options(nomem, nostack, preserves_flags))
        };
        v
    }};
}

macro_rules! write_sysreg {
    ($name:literal, $v:expr) => {{
        // SAFETY: each caller documents why the write is sound.
        unsafe {
            core::arch::asm!(concat!("msr ", $name, ", {}"), in(reg) $v,
                             options(nomem, nostack, preserves_flags))
        };
    }};
}

// ── Clock ─────────────────────────────────────────────────────────

#[no_mangle]
extern "Rust" fn __rivet_board_now_us() -> u64 {
    // The 64-bit counter is two 32-bit registers, so a naive read can tear
    // across a low-word wrap. Re-read the high word and retry if it moved.
    loop {
        // SAFETY: plain MMIO reads of the System Timer.
        unsafe {
            let hi0 = read_volatile((SYSTEM_TIMER_BASE + ST_CHI) as *const u32);
            let lo = read_volatile((SYSTEM_TIMER_BASE + ST_CLO) as *const u32);
            let hi1 = read_volatile((SYSTEM_TIMER_BASE + ST_CHI) as *const u32);
            if hi0 == hi1 {
                // 1 MHz, so counts are already microseconds.
                return ((hi1 as u64) << 32) | lo as u64;
            }
        }
    }
}

// ── Tick ──────────────────────────────────────────────────────────

#[no_mangle]
extern "Rust" fn __rivet_board_tick_start(hz: u32) {
    let hz = hz.max(1);
    let freq = read_sysreg!("cntfrq_el0");
    let interval = freq / hz as u64;
    TK.interval.store(interval, Ordering::Release);
    TICK_PERIOD_US.store(1_000_000 / hz, Ordering::Release);
    TICK_HZ.store(hz, Ordering::Release);
    // The header was published before the tick started, so the rate in it
    // is still zero. Fill it in now that there is one.
    #[cfg(feature = "amp")]
    // SAFETY: the shared window is mapped by the time the tick starts.
    unsafe {
        crate::sysinfo::set_tick_hz(hz)
    };

    // Route the non-secure physical timer to *this* core as an IRQ. The
    // kernel may well be running on a core the firmware never booted.
    // SAFETY: MMIO write to this core's ARM-local interrupt-control
    // register.
    unsafe {
        write_volatile(core_timer_irqcntl() as *mut u32, TIMER_IRQ_CNTPNSIRQ);
    }

    // Arm the first expiry against the absolute comparator rather than a
    // countdown, then enable the timer. CNTP_CTL_EL0: bit 0 enable,
    // bit 1 mask.
    let now = read_sysreg!("cntpct_el0");
    write_sysreg!("cntp_cval_el0", now + interval);
    write_sysreg!("cntp_ctl_el0", 1u64);
}

/// Interrupt-latency and handler-cost statistics, in architected timer
/// ticks (52.08 ns each at 19.2 MHz).
///
/// The latency measured is genuine: the comparator value that just fired
/// is the exact hardware instant of expiry, so subtracting it from the
/// counter read at the top of the handler covers exception entry, the
/// full register save and the board's dispatch, with nothing estimated.
///
/// They live in one cache-line-aligned block rather than as separate
/// statics.
///
/// As separate statics these landed wherever the linker put them, and it
/// put them badly: the three minima initialise to `u64::MAX` so they went
/// to `.data` while the rest, zero-initialised, went to `.bss`, 36 KiB
/// away. Three cache lines, touched once per tick and not otherwise,
/// which is the access pattern that lives in a shared L2 and is evicted
/// between ticks by the other cores. The footprint was decided by
/// initialiser values, which is no way to decide it.
///
/// Grouped and aligned, this is two lines and stays two lines however the
/// linker feels, and adding a counter cannot silently cost a third.
#[repr(C, align(64))]
struct TickCounters {
    lat_min: AtomicU64,
    lat_max: AtomicU64,
    lat_sum: AtomicU64,
    lat_cnt: AtomicU64,
    lat_over: AtomicU64,
    cost_min: AtomicU64,
    cost_max: AtomicU64,
    cost_sum: AtomicU64,
    cost_over: AtomicU64,
    gap_min: AtomicU64,
    gap_max: AtomicU64,
    gap_sum: AtomicU64,
    gap_cnt: AtomicU64,
    last_entry: AtomicU64,
    interval: AtomicU64,
}

static TK: TickCounters = TickCounters {
    lat_min: AtomicU64::new(u64::MAX),
    lat_max: AtomicU64::new(0),
    lat_sum: AtomicU64::new(0),
    lat_cnt: AtomicU64::new(0),
    lat_over: AtomicU64::new(0),
    cost_min: AtomicU64::new(u64::MAX),
    cost_max: AtomicU64::new(0),
    cost_sum: AtomicU64::new(0),
    cost_over: AtomicU64::new(0),
    gap_min: AtomicU64::new(u64::MAX),
    gap_max: AtomicU64::new(0),
    gap_sum: AtomicU64::new(0),
    gap_cnt: AtomicU64::new(0),
    last_entry: AtomicU64::new(0),
    interval: AtomicU64::new(0),
};

/// Per-stage timing inside the tick handler, behind `tick-phases`.
///
/// Answers where a tick's time actually goes, rather than leaving it to be
/// inferred from the total. Three stages: the handler's own bookkeeping,
/// which touches nothing but this module's statics; the watchdog; and the
/// timer wheel.
///
/// Off by default because it adds three counter reads, each with an ISB,
/// to a path that runs ten thousand times a second, and those reads land
/// inside the very interval the handler reports as its cost. A build with
/// this on measures itself measuring.
#[cfg(feature = "tick-phases")]
mod phases {
    use core::sync::atomic::{AtomicU64, Ordering};
    pub static BOOKKEEPING: [AtomicU64; 3] =
        [AtomicU64::new(0), AtomicU64::new(0), AtomicU64::new(0)];
    pub static WATCHDOG: [AtomicU64; 3] = [AtomicU64::new(0), AtomicU64::new(0), AtomicU64::new(0)];
    pub static TIMERS: [AtomicU64; 3] = [AtomicU64::new(0), AtomicU64::new(0), AtomicU64::new(0)];
    /// A control that touches only this module's own memory, never the
    /// shared window, so a stage that degrades under load says something
    /// about private memory rather than about sharing.
    pub static CONTROL: [AtomicU64; 3] = [AtomicU64::new(0), AtomicU64::new(0), AtomicU64::new(0)];
    pub static SCRATCH: [AtomicU64; 8] = [
        AtomicU64::new(0),
        AtomicU64::new(0),
        AtomicU64::new(0),
        AtomicU64::new(0),
        AtomicU64::new(0),
        AtomicU64::new(0),
        AtomicU64::new(0),
        AtomicU64::new(0),
    ];

    /// `[max, sum, over-one-microsecond]`.
    pub fn record(cell: &[AtomicU64; 3], d: u64) {
        cell[0].fetch_max(d, Ordering::Relaxed);
        cell[1].fetch_add(d, Ordering::Relaxed);
        if d > super::LONG_TICKS {
            cell[2].fetch_add(1, Ordering::Relaxed);
        }
    }

    pub fn reset() {
        for c in [&BOOKKEEPING, &WATCHDOG, &TIMERS, &CONTROL] {
            for f in c {
                f.store(0, Ordering::Relaxed);
            }
        }
    }

    pub fn snapshot(cell: &[AtomicU64; 3]) -> (u64, u64, u64) {
        (
            cell[0].load(Ordering::Relaxed),
            cell[1].load(Ordering::Relaxed),
            cell[2].load(Ordering::Relaxed),
        )
    }
}

/// `(max, sum, over_1us)` for each stage: bookkeeping, watchdog, timer
/// wheel, and the private-memory control.
#[cfg(feature = "tick-phases")]
pub fn tick_phase_stats() -> [(u64, u64, u64); 4] {
    [
        phases::snapshot(&phases::BOOKKEEPING),
        phases::snapshot(&phases::WATCHDOG),
        phases::snapshot(&phases::TIMERS),
        phases::snapshot(&phases::CONTROL),
    ]
}

/// One microsecond in counter ticks, rounded up, at the 19.2 MHz
/// architected counter. The bar for counting a sample as having run long.
const LONG_TICKS: u64 = 20;

/// Everything the tick handler measures about itself, in timer ticks.
///
/// A tuple got unreadable once there was more than one thing being
/// counted, and every field here is a different quantity that happens to
/// share a unit.
#[derive(Clone, Copy)]
pub struct IrqStats {
    /// Comparator match to the first instruction of the handler.
    pub lat_min: u64,
    pub lat_max: u64,
    pub lat_sum: u64,
    /// Ticks observed, which is the sample count for latency and cost.
    pub count: u64,
    /// Latency samples over one microsecond. A maximum on its own says
    /// nothing about how often the worst case happens, and on a tight
    /// distribution the integer mean collapses onto the minimum and stops
    /// saying anything at all.
    pub lat_over: u64,
    /// Time spent inside the handler itself.
    pub cost_min: u64,
    pub cost_max: u64,
    pub cost_sum: u64,
    /// Handler-cost samples over one microsecond.
    pub cost_over: u64,
    /// Interval between consecutive handler entries. The comparator is
    /// advanced by a fixed step so the deadline grid cannot drift; what
    /// varies here is only how promptly each deadline was serviced, which
    /// is what tick jitter means on this design.
    pub gap_min: u64,
    pub gap_max: u64,
    pub gap_sum: u64,
    pub gap_count: u64,
}

/// Snapshot of the tick handler's self-measurements.
pub fn irq_stats() -> IrqStats {
    IrqStats {
        lat_min: TK.lat_min.load(Ordering::Relaxed),
        lat_max: TK.lat_max.load(Ordering::Relaxed),
        lat_sum: TK.lat_sum.load(Ordering::Relaxed),
        count: TK.lat_cnt.load(Ordering::Relaxed),
        lat_over: TK.lat_over.load(Ordering::Relaxed),
        cost_min: TK.cost_min.load(Ordering::Relaxed),
        cost_max: TK.cost_max.load(Ordering::Relaxed),
        cost_sum: TK.cost_sum.load(Ordering::Relaxed),
        cost_over: TK.cost_over.load(Ordering::Relaxed),
        gap_min: TK.gap_min.load(Ordering::Relaxed),
        gap_max: TK.gap_max.load(Ordering::Relaxed),
        gap_sum: TK.gap_sum.load(Ordering::Relaxed),
        gap_count: TK.gap_cnt.load(Ordering::Relaxed),
    }
}

/// Discard everything gathered so far, so a measurement window excludes
/// start-up.
pub fn reset_irq_stats() {
    TK.lat_min.store(u64::MAX, Ordering::Relaxed);
    TK.lat_max.store(0, Ordering::Relaxed);
    TK.lat_sum.store(0, Ordering::Relaxed);
    TK.lat_cnt.store(0, Ordering::Relaxed);
    TK.cost_min.store(u64::MAX, Ordering::Relaxed);
    TK.cost_max.store(0, Ordering::Relaxed);
    TK.cost_sum.store(0, Ordering::Relaxed);
    TK.gap_min.store(u64::MAX, Ordering::Relaxed);
    TK.gap_max.store(0, Ordering::Relaxed);
    TK.gap_sum.store(0, Ordering::Relaxed);
    TK.gap_cnt.store(0, Ordering::Relaxed);
    TK.last_entry.store(0, Ordering::Relaxed);
    TK.lat_over.store(0, Ordering::Relaxed);
    TK.cost_over.store(0, Ordering::Relaxed);
    #[cfg(feature = "tick-phases")]
    phases::reset();
}

fn record_min(cell: &AtomicU64, v: u64) {
    let mut cur = cell.load(Ordering::Relaxed);
    while v < cur {
        match cell.compare_exchange_weak(cur, v, Ordering::Relaxed, Ordering::Relaxed) {
            Ok(_) => break,
            Err(actual) => cur = actual,
        }
    }
}

/// Called from the interrupt path when the generic timer has expired.
fn on_timer_tick() {
    crate::scope::tick_begin();
    // Advance the absolute comparator rather than reloading a countdown.
    //
    // Writing CNTP_TVAL_EL0 sets the deadline relative to *now*, meaning
    // the moment the handler gets around to it, so every tick quietly
    // absorbs the exception entry, the register save and the MMIO read
    // above it. That is a small constant, but it is a constant added
    // every single tick, and it makes the tick grid walk away from real
    // time forever. Advancing CVAL by a fixed interval cannot drift: a
    // late handler produces one late tick, not a permanently skewed
    // grid.
    let interval = TK.interval.load(Ordering::Acquire);
    let cval = read_sysreg!("cntp_cval_el0");
    let entry = read_sysreg!("cntpct_el0");
    write_sysreg!("cntp_cval_el0", cval + interval);

    // `cval` is the instant the comparator matched, so this is the real
    // hardware-to-handler latency rather than an estimate.
    let lat = entry.wrapping_sub(cval);
    TK.lat_max.fetch_max(lat, Ordering::Relaxed);
    record_min(&TK.lat_min, lat);
    TK.lat_sum.fetch_add(lat, Ordering::Relaxed);
    let ticks = TK.lat_cnt.fetch_add(1, Ordering::Relaxed) + 1;

    // Liveness for the Linux side, published from a counter this handler
    // already maintains. Adding one of its own put another cold line on
    // this path and cost more than the feature is worth; see
    // sysinfo::heartbeat.
    #[cfg(feature = "amp")]
    if ticks & (crate::sysinfo::TICKS_PER_BEAT as u64 - 1) == 0 {
        // SAFETY: the shared window is mapped for the life of an AMP image.
        unsafe { crate::sysinfo::heartbeat(entry) };
    }

    if lat > LONG_TICKS {
        TK.lat_over.fetch_add(1, Ordering::Relaxed);
    }

    // Skip the first tick after a reset: there is no previous entry to
    // measure an interval against, and treating zero as one would report
    // a gap the size of the uptime.
    let prev = TK.last_entry.swap(entry, Ordering::Relaxed);
    if prev != 0 {
        let gap = entry.wrapping_sub(prev);
        TK.gap_max.fetch_max(gap, Ordering::Relaxed);
        record_min(&TK.gap_min, gap);
        TK.gap_sum.fetch_add(gap, Ordering::Relaxed);
        TK.gap_cnt.fetch_add(1, Ordering::Relaxed);
    }

    #[cfg(feature = "tick-phases")]
    let t_book = read_sysreg!("cntpct_el0");
    #[cfg(feature = "tick-phases")]
    phases::record(&phases::BOOKKEEPING, t_book.wrapping_sub(entry));

    rivet::watchdog::on_tick();
    #[cfg(feature = "tick-phases")]
    let t_wdog = read_sysreg!("cntpct_el0");
    #[cfg(feature = "tick-phases")]
    phases::record(&phases::WATCHDOG, t_wdog.wrapping_sub(t_book));

    rivet::timer::poll_timers(__rivet_board_now_us());
    #[cfg(feature = "tick-phases")]
    let t_timers = read_sysreg!("cntpct_el0");
    #[cfg(feature = "tick-phases")]
    phases::record(&phases::TIMERS, t_timers.wrapping_sub(t_wdog));

    // The control: a fixed number of read-modify-writes against this
    // module's own statics, which live in the reserved region Linux
    // cannot even map. No shared memory, no coherency traffic with the
    // other cluster, nothing but private cacheable RAM. Whatever this
    // does under load is the floor for every other stage.
    #[cfg(feature = "tick-phases")]
    {
        for slot in phases::SCRATCH.iter() {
            slot.fetch_add(1, Ordering::Relaxed);
        }
        let t_ctl = read_sysreg!("cntpct_el0");
        phases::record(&phases::CONTROL, t_ctl.wrapping_sub(t_timers));
    }

    let cost = read_sysreg!("cntpct_el0").wrapping_sub(entry);
    TK.cost_max.fetch_max(cost, Ordering::Relaxed);
    record_min(&TK.cost_min, cost);
    TK.cost_sum.fetch_add(cost, Ordering::Relaxed);
    if cost > LONG_TICKS {
        TK.cost_over.fetch_add(1, Ordering::Relaxed);
    }

    crate::scope::tick_end();
}

// ── Interrupts ────────────────────────────────────────────────────

/// Fired when Linux rings this core's mailbox doorbell.
///
/// A `Signal` rather than a callback, so a task can `.await` it and the
/// core sits in `WFI` between commands instead of polling the shared
/// window.
pub static DOORBELL: rivet::sync::Signal = rivet::sync::Signal::new();

/// Count of doorbells taken, for tests that want to prove one arrived.
static DOORBELL_COUNT: AtomicU32 = AtomicU32::new(0);

pub fn doorbell_count() -> u32 {
    DOORBELL_COUNT.load(Ordering::Relaxed)
}

/// Let Linux interrupt this core through mailbox 0.
///
/// # Safety
/// Call once, from the core that will service the doorbell.
pub unsafe fn enable_doorbell() {
    // SAFETY: forwarded from this function's contract.
    unsafe { crate::mailbox::enable_on_this_core(crate::mailbox::DOORBELL) };
}

/// Acknowledge and service whatever raised the current interrupt.
///
/// Scheduling is not done here: `rivet-arch-aarch64` runs the scheduler on
/// the way out of the interrupt regardless of what fired.
#[no_mangle]
extern "Rust" fn __rivet_board_on_irq() {
    // SAFETY: MMIO read of this core's pending-source register.
    let source = unsafe { read_volatile(core_irq_source() as *const u32) };

    if source & TIMER_IRQ_CNTPNSIRQ != 0 {
        on_timer_tick();
    }

    if source & crate::mailbox::IRQ_SOURCE_MBOX0 != 0 {
        // First thing in the branch, so the rising edge marks the handler
        // being reached and not the work that follows it. The task the
        // doorbell wakes drops it again, making the width the wake path.
        crate::scope::doorbell_begin();
        // Clearing the mailbox is what drops the interrupt line; skip it
        // and this handler re-enters forever.
        // SAFETY: servicing this core's own mailbox.
        unsafe { crate::mailbox::take(crate::mailbox::DOORBELL) };
        DOORBELL_COUNT.fetch_add(1, Ordering::Relaxed);
        DOORBELL.signal();
    }

    // Peripheral interrupts arrive through the legacy controller, which
    // this core sees as one aggregated source. Anything registered with
    // `rivet::irq` gets dispatched by number.
    if source & (1 << 8) != 0 {
        dispatch_peripheral_irqs();
    }
}

/// Walk the legacy controller's two pending words and dispatch each set
/// line. There is no claim/complete handshake to perform: the peripheral
/// itself owns the acknowledgement, which its handler does.
fn dispatch_peripheral_irqs() {
    // SAFETY: MMIO reads of the pending registers.
    let (p1, p2) = unsafe {
        (
            read_volatile((IRQ_BASE + 0x04) as *const u32),
            read_volatile((IRQ_BASE + 0x08) as *const u32),
        )
    };
    for bit in 0..32 {
        if p1 & (1 << bit) != 0 {
            rivet::irq::dispatch(bit);
        }
    }
    for bit in 0..32 {
        if p2 & (1 << bit) != 0 {
            rivet::irq::dispatch(32 + bit);
        }
    }
}

#[no_mangle]
extern "Rust" fn __rivet_board_irq_enable(irq_num: u32) {
    let (reg, bit) = if irq_num < 32 {
        (ENABLE_IRQS_1, irq_num)
    } else {
        (ENABLE_IRQS_2, irq_num - 32)
    };
    // These registers are write-to-set: writing a zero bit does nothing,
    // so there is no read-modify-write and no race with an interrupt
    // enabling a different line.
    // SAFETY: MMIO write to the legacy interrupt controller.
    unsafe { write_volatile((IRQ_BASE + reg) as *mut u32, 1 << bit) };
}

#[no_mangle]
extern "Rust" fn __rivet_board_irq_disable(irq_num: u32) {
    let (reg, bit) = if irq_num < 32 {
        (DISABLE_IRQS_1, irq_num)
    } else {
        (DISABLE_IRQS_2, irq_num - 32)
    };
    // SAFETY: as above, write-to-clear.
    unsafe { write_volatile((IRQ_BASE + reg) as *mut u32, 1 << bit) };
}

/// No-op: this controller has no priority levels at all. Every enabled
/// line is equal, and the dispatch order above is simply bit order.
#[no_mangle]
extern "Rust" fn __rivet_board_irq_set_priority(_irq_num: u32, _priority: u8) {}

// ── Console ───────────────────────────────────────────────────────

/// `init_uart_clock` and the baud from config.txt.
const UART_CLK_HZ: u32 = 48_000_000;
const BAUD: u32 = 115_200;

#[no_mangle]
extern "Rust" fn __rivet_board_init() {
    // Leave the UART alone when another OS owns it. `Pl011::init`
    // re-muxes GPIO14/15, which would pull the serial console out from
    // under Linux mid-sentence.
    if CONSOLE_TO_SHMEM.load(Ordering::Acquire) != 0 {
        return;
    }
    // SAFETY: called once, before any task runs, and this core is the
    // only thing touching the UART or GPIO14/15.
    unsafe { Pl011.init(UART_CLK_HZ, BAUD) };
}

/// Where kernel console output goes.
///
/// Defaults to the UART. Running alongside another OS that owns the
/// serial line, [`use_shared_console`] redirects it into the ring in
/// shared memory instead, since there is only one UART on the header.
static CONSOLE_TO_SHMEM: AtomicU32 = AtomicU32::new(0);

/// Send kernel console output to the shared-memory ring rather than the
/// UART, for when another OS owns the serial line.
///
/// # Safety
/// The shared window must be mapped, and [`crate::shmem::init`] must
/// have run.
pub unsafe fn use_shared_console() {
    CONSOLE_TO_SHMEM.store(1, Ordering::Release);
}

#[no_mangle]
extern "Rust" fn __rivet_board_console_write(ptr: *const u8, len: usize) {
    // SAFETY: the kernel passes a valid pointer/length pair from a live
    // byte slice.
    let bytes = unsafe { core::slice::from_raw_parts(ptr, len) };
    if CONSOLE_TO_SHMEM.load(Ordering::Acquire) != 0 {
        // SAFETY: the redirect is only armed once the window is mapped
        // and the ring initialised.
        unsafe { crate::shmem::write_bytes(bytes) };
        return;
    }
    for &b in bytes {
        // SAFETY: blocking write to a UART brought up in `__rivet_board_init`.
        unsafe { Pl011.put_byte(b) };
    }
}

/// No-op: the console is a blocking polled writer, so there is no TX
/// interrupt that could have disarmed itself and need kicking.
#[no_mangle]
extern "Rust" fn __rivet_board_console_kick_tx() {}

// ── Trace transport ───────────────────────────────────────────────

/// PulseTrace frames go to their own ring, never the console.
///
/// The wire format is framed and checksummed binary, so a log line
/// landing in the middle of it corrupts a frame. Sharing one transport
/// between the two would make both unreadable, which is why the shared
/// window carries a second ring rather than one.
///
/// The UART is not an option here even standalone: on this board it is
/// either the console or, alongside Linux, not ours at all.
#[cfg(feature = "trace")]
#[no_mangle]
extern "Rust" fn __rivet_board_trace_write(ptr: *const u8, len: usize) {
    // SAFETY: the kernel passes a valid pointer/length pair from a live
    // byte slice.
    let bytes = unsafe { core::slice::from_raw_parts(ptr, len) };
    // SAFETY: the trace ring is brought up by `board_bringup` before any
    // task can run and therefore before anything can emit a frame.
    unsafe { crate::shmem::TRACE.write_bytes(bytes) };
}

// ── Reset, exit, watchdog ─────────────────────────────────────────

#[no_mangle]
extern "Rust" fn __rivet_board_reset() -> ! {
    // SAFETY: arming the PM watchdog for the shortest possible period and
    // requesting a full reset is the documented reset path on this SoC;
    // every write needs the password in the top byte.
    unsafe {
        write_volatile((PM_BASE + PM_WDOG) as *mut u32, PM_PASSWORD | 1);
        let rstc = read_volatile((PM_BASE + PM_RSTC) as *const u32);
        write_volatile(
            (PM_BASE + PM_RSTC) as *mut u32,
            PM_PASSWORD | (rstc & PM_RSTC_WRCFG_CLR) | PM_RSTC_WRCFG_FULL_RESET,
        );
    }
    loop {
        core::hint::spin_loop();
    }
}

/// Park rather than reset.
///
/// On the QEMU boards `exit` reaches a semihosting exit device and gives
/// the test harness a status. There is no such device here, and resetting
/// would drop straight back into the same image, so a board watched over
/// a serial line is better served by halting with the exit code visible
/// than by looping forever through its own boot.
#[no_mangle]
extern "Rust" fn __rivet_board_exit(code: u32) -> ! {
    // An orderly end. Without this the heartbeat simply stops and Linux
    // reports a hang, which would be wrong and would cry wolf every time
    // a benchmark finished.
    #[cfg(feature = "amp")]
    // SAFETY: the shared window is mapped for the life of an AMP image.
    unsafe {
        crate::sysinfo::set_state(if code == 0 {
            crate::sysinfo::state::EXITED
        } else {
            crate::sysinfo::state::FAULTED
        })
    };

    rivet::console::write_str(if code == 0 {
        "\n[rivet] exit(0)\n"
    } else {
        "\n[rivet] exit(nonzero)\n"
    });
    rivet::console::flush_sync();

    // Mask interrupts before halting. Otherwise the timer keeps firing
    // into a kernel that has stopped, the heartbeat keeps incrementing,
    // and Linux sees a healthy pulse from something that is no longer
    // running. A stopped kernel should look stopped.
    // SAFETY: nothing further runs on this core.
    unsafe { core::arch::asm!("msr daifset, #3", options(nomem, nostack)) };

    loop {
        // SAFETY: WFE is side-effect free.
        unsafe { core::arch::asm!("wfe", options(nomem, nostack)) };
    }
}

/// The PM watchdog counts at 65536 Hz, so one tick is about 15.26 us.
/// Treating microseconds as ticks, which an earlier version did, asks for
/// a timeout 65536 times shorter than intended.
const WDOG_TICKS_PER_SEC: u64 = 65536;
/// The counter is 20 bits, so the longest period it can express is a
/// little over 16 seconds.
const WDOG_MAX_TICKS: u64 = 0x000F_FFFF;

#[no_mangle]
extern "Rust" fn __rivet_board_wdt_init(period_us: u32) {
    if period_us == 0 {
        return;
    }
    arm_watchdog(period_us);
}

#[no_mangle]
extern "Rust" fn __rivet_board_wdt_feed() {
    let period = WDT_PERIOD_US.load(Ordering::Acquire);
    if period != 0 {
        arm_watchdog(period);
    }
}

/// No-op: this is a real hardware watchdog that counts down on its own,
/// so there is no software deadline to check against the clock.
#[no_mangle]
extern "Rust" fn __rivet_board_wdt_check() {}

static WDT_PERIOD_US: AtomicU32 = AtomicU32::new(0);

fn arm_watchdog(period_us: u32) {
    WDT_PERIOD_US.store(period_us, Ordering::Release);
    // Saturate rather than wrap: wrapping a too-long period turns a
    // generous timeout into a very short one, which resets the board
    // instead of protecting it.
    let ticks = ((period_us as u64 * WDOG_TICKS_PER_SEC) / 1_000_000).min(WDOG_MAX_TICKS) as u32;
    // SAFETY: password-protected PM register writes, as in `reset`.
    unsafe {
        write_volatile((PM_BASE + PM_WDOG) as *mut u32, PM_PASSWORD | ticks);
        let rstc = read_volatile((PM_BASE + PM_RSTC) as *const u32);
        write_volatile(
            (PM_BASE + PM_RSTC) as *mut u32,
            PM_PASSWORD | (rstc & PM_RSTC_WRCFG_CLR) | PM_RSTC_WRCFG_FULL_RESET,
        );
    }
}
