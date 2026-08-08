//! Deferred-formatting logging: [`log!`](crate::log!) is safe to call from
//! ISR context (it does no formatting, no allocation, and never blocks —
//! it just pushes a `{level, task_id, timestamp, message}` frame into a
//! ring buffer), and a drain task formats and writes frames to the
//! console at its own pace, off the hot path.
//!
//! # Scope note (plan.md Phase 8)
//!
//! The old plan (§6.5) called for interning format strings into a
//! `.rivet_log_fmt` linker section, storing only a small integer index per
//! frame, and decoding on the host from the ELF's debug info (a
//! `rivet-decode` crate). This module takes a simpler route that still
//! delivers the properties that actually matter (ISR-safe, O(1) in the
//! hot path, deferred formatting, lock-free ring buffer, dropped-frame
//! accounting): a frame stores the message as a plain `&'static str`
//! pointer + length instead of an interned index. This means **no
//! interpolated arguments** — `log!` takes a level and a string literal,
//! not a `format_args!`-style template. Extending this to support
//! arguments (and the on-target interning + host decoder) is a real next
//! step, not attempted here; the frame format below is deliberately
//! small enough that adding an argument payload later is a additive
//! change, not a rewrite.
//!
//! The ring buffer is Rivet's own SPSC [`crate::sync::Channel`] — but
//! logging is inherently **multi**-producer (any task or ISR might log),
//! so every producer path goes through [`crate::critical::enter`] to
//! serialize pushes into a single logical producer. That's sound because
//! critical sections are Rivet's only synchronization primitive across
//! interrupt context, and Rivet's supported multi-core story is AMP (one
//! independent kernel instance per hart, no shared state) — see plan.md
//! §9.1/§9.2.

use crate::sync::{Channel, Once, Receiver, Sender};
use core::sync::atomic::{AtomicU32, Ordering};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Level {
    Trace,
    Debug,
    Info,
    Warn,
    Error,
}

impl Level {
    fn as_str(self) -> &'static str {
        match self {
            Level::Trace => "TRACE",
            Level::Debug => "DEBUG",
            Level::Info => "INFO",
            Level::Warn => "WARN",
            Level::Error => "ERROR",
        }
    }
}

#[derive(Clone, Copy)]
pub struct LogFrame {
    pub level: Level,
    /// Preemptive task id, or `None` if logged from a context with no
    /// current task (e.g. before the scheduler starts).
    pub task_id: Option<u16>,
    pub timestamp_us: u32,
    pub msg: &'static str,
}

/// Ring capacity. Deliberately not wired into `RIVET_*` build-time config
/// (plan.md §4.1) yet — a fixed default is the honest starting point for a
/// feature this new; making it configurable is a small follow-up once
/// there's a real workload to size it against.
const CAPACITY: usize = 16;

static CHANNEL: Channel<LogFrame, CAPACITY> = Channel::new();
static SENDER: Once<Sender<'static, LogFrame, CAPACITY>> = Once::new();
static RECEIVER: Once<Receiver<'static, LogFrame, CAPACITY>> = Once::new();
static DROPPED: AtomicU32 = AtomicU32::new(0);

/// Called once from [`crate::init`]. Splitting the channel here (rather
/// than lazily on first use) means the first `log!` call anywhere is
/// never the one paying for initialization, and keeps `push`'s hot path
/// to just a critical-section-guarded `try_send`.
pub(crate) fn init() {
    if let Some((tx, rx)) = CHANNEL.split() {
        let _ = SENDER.set(tx);
        let _ = RECEIVER.set(rx);
    }
}

/// Push a frame. ISR-safe: no allocation, no unbounded loops, never
/// blocks — a full ring drops the frame and counts it (see
/// [`dropped_frames`]) rather than backing up whatever called this.
#[doc(hidden)]
pub fn push(level: Level, msg: &'static str) {
    let task_id = crate::preempt::sched::current().map(|id| id as u16);
    let timestamp_us = crate::port::board::now_us() as u32;
    let frame = LogFrame {
        level,
        task_id,
        timestamp_us,
        msg,
    };
    let sent = match SENDER.get() {
        // SAFETY-relevant, not memory-safety: `try_send` requires a
        // single logical producer; the critical section serializes
        // however many concurrent callers (tasks and/or ISRs) there are
        // into one.
        Some(tx) => crate::critical::enter(|| tx.try_send(frame)).is_ok(),
        None => false,
    };
    if !sent {
        DROPPED.fetch_add(1, Ordering::Relaxed);
    }
}

/// Number of frames dropped so far because the ring was full (or logging
/// hadn't been initialized yet — i.e. called before [`crate::init`]).
pub fn dropped_frames() -> usize {
    DROPPED.load(Ordering::Relaxed) as usize
}

fn write_frame(frame: &LogFrame) {
    crate::console::write_str("[");
    crate::console::write_str(frame.level.as_str());
    crate::console::write_str("] t=");
    write_dec(frame.timestamp_us as usize);
    if let Some(id) = frame.task_id {
        crate::console::write_str(" task=");
        write_dec(id as usize);
    }
    crate::console::write_str(" ");
    crate::console::write_str(frame.msg);
    crate::console::write_str("\n");
}

fn write_dec(mut n: usize) {
    if n == 0 {
        crate::console::write_str("0");
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
        crate::console::write_str(s);
    }
}

/// Drain and format one pending frame. Returns `false` if the ring was
/// empty. Call this in a loop from a low-priority task to flush the log
/// (see [`drain_forever`] for a ready-made one).
pub fn drain_one() -> bool {
    match RECEIVER.get().and_then(|rx| rx.try_recv()) {
        Some(frame) => {
            write_frame(&frame);
            true
        }
        None => false,
    }
}

/// A ready-made drain loop: `.await`s new frames and writes them to the
/// console as they arrive. Spawn this as a low-priority `#[rivet::task]`
/// if you want logging without writing your own drain loop.
///
/// ```ignore
/// #[rivet::task(priority = 0)]
/// async fn log_drain() {
///     rivet::log::drain_forever().await;
/// }
/// ```
pub async fn drain_forever() -> ! {
    let rx = loop {
        if let Some(rx) = RECEIVER.get() {
            break rx;
        }
        // Logging hasn't been initialized yet (called before rivet::init)
        // — extremely unlikely given normal boot order, but don't spin
        // hot if it happens.
        crate::time::Sleep::<1000>::new().await;
    };
    loop {
        let frame = rx.recv().await;
        write_frame(&frame);
    }
}

/// Log a message at the given level. ISR-safe. Takes a plain string
/// literal or `&'static str` — see the module docs for why there's no
/// `format_args!`-style interpolation (yet).
#[macro_export]
macro_rules! log {
    ($level:expr, $msg:expr) => {
        $crate::log::push($level, $msg)
    };
}
