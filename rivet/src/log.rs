//! Deferred-formatting logging: [`log!`](crate::log!) is safe to call from
//! ISR context (it does no formatting, no allocation, and never blocks —
//! it just pushes a `{level, task_id, timestamp, message}` frame into a
//! ring buffer), and a drain task formats and writes frames to the
//! console at its own pace, off the hot path.
//!
//! # Scope note (plan.md Phase 8, extended by Phase 16)
//!
//! The old plan (§6.5) called for interning format strings into a
//! `.rivet_log_fmt` linker section, storing only a small integer index per
//! frame, and decoding on the host from the ELF's debug info (a
//! `rivet-decode` crate). This module takes a simpler route that still
//! delivers the properties that actually matter (ISR-safe, O(1) in the
//! hot path, deferred formatting, lock-free ring buffer, dropped-frame
//! accounting): a frame stores the message as a plain `&'static str`
//! pointer + length, plus (Phase 16) up to two [`LogArg`] values —
//! **not** a full `format_args!`-style template (that needs the
//! interned-format-string + host-decoder machinery the old plan
//! described, still not attempted here). `log!("x={}", x)` covers the
//! large majority of real call sites, which log one or two values
//! alongside a fixed message; `write_frame` substitutes each `{}` in
//! `msg` with the corresponding argument, formatted at drain time (off
//! the hot path, same as everything else here).
//!
//! The ring buffer is Rivet's own SPSC [`crate::sync::Channel`] — but
//! logging is inherently **multi**-producer (any task or ISR might log,
//! on any hart), so every producer path goes through
//! [`crate::critical::enter`] to serialize pushes into a single logical
//! producer. Since plan.md Phase 19, `critical::enter` is a genuine
//! cross-hart lock (not just a local interrupt mask), so this holds under
//! real SMP too, not only the single-hart case.

use crate::sync::{Channel, Once, Receiver, Sender};
use crate::sync::atomic::{AtomicU32, Ordering};

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

/// A single interpolated argument (plan.md Phase 16): a small closed set
/// covering the large majority of real log call sites, not a general
/// `Display`/`Debug` payload (which would need real formatting work done
/// eagerly, defeating the point of deferring it to drain time).
#[derive(Clone, Copy)]
pub enum LogArg {
    /// No argument in this slot (fewer than 2 given to `log!`).
    None,
    U32(u32),
    I32(i32),
    F32(f32),
    Str(&'static str),
}

impl From<u32> for LogArg {
    fn from(v: u32) -> Self {
        LogArg::U32(v)
    }
}
impl From<i32> for LogArg {
    fn from(v: i32) -> Self {
        LogArg::I32(v)
    }
}
impl From<f32> for LogArg {
    fn from(v: f32) -> Self {
        LogArg::F32(v)
    }
}
impl From<&'static str> for LogArg {
    fn from(v: &'static str) -> Self {
        LogArg::Str(v)
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
    /// Up to two interpolated arguments, substituted in order for each
    /// `{}` in `msg` at drain time. `LogArg::None` in a slot the message
    /// doesn't reference is simply unused.
    pub args: [LogArg; 2],
}

/// Ring capacity. Deliberately not wired into `RIVET_*` build-time config
/// (plan.md §4.1) yet — a fixed default is the honest starting point for a
/// feature this new; making it configurable is a small follow-up once
/// there's a real workload to size it against.
const CAPACITY: usize = 16;

#[cfg(not(loom))]
static CHANNEL: Channel<LogFrame, CAPACITY> = Channel::new();
#[cfg(loom)]
loom::lazy_static! {
    static ref CHANNEL: Channel<LogFrame, CAPACITY> = Channel::new();
}

#[cfg(not(loom))]
static SENDER: Once<Sender<'static, LogFrame, CAPACITY>> = Once::new();
#[cfg(loom)]
loom::lazy_static! {
    static ref SENDER: Once<Sender<'static, LogFrame, CAPACITY>> = Once::new();
}

#[cfg(not(loom))]
static RECEIVER: Once<Receiver<'static, LogFrame, CAPACITY>> = Once::new();
#[cfg(loom)]
loom::lazy_static! {
    static ref RECEIVER: Once<Receiver<'static, LogFrame, CAPACITY>> = Once::new();
}
#[cfg(not(loom))]
static DROPPED: AtomicU32 = AtomicU32::new(0);
#[cfg(loom)]
loom::lazy_static! {
    static ref DROPPED: AtomicU32 = AtomicU32::new(0);
}

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
pub fn push(level: Level, msg: &'static str, arg0: LogArg, arg1: LogArg) {
    let task_id = crate::preempt::sched::current().map(|id| id as u16);
    let timestamp_us = crate::port::board::now_us() as u32;
    let frame = LogFrame {
        level,
        task_id,
        timestamp_us,
        msg,
        args: [arg0, arg1],
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
    write_interpolated(frame.msg, &frame.args);
    crate::console::write_str("\n");
}

/// Write `msg`, substituting each `{}` (in order) with the corresponding
/// entry of `args`. Extra `{}` beyond the two argument slots are written
/// through literally, since there's nothing to fill them with — no
/// silent truncation of the message.
fn write_interpolated(msg: &str, args: &[LogArg; 2]) {
    let mut rest = msg;
    let mut arg_idx = 0usize;
    while let Some(pos) = rest.find("{}") {
        crate::console::write_str(&rest[..pos]);
        match args.get(arg_idx) {
            Some(LogArg::None) | None => crate::console::write_str("{}"),
            Some(arg) => write_arg(arg),
        }
        arg_idx += 1;
        rest = &rest[pos + 2..];
    }
    crate::console::write_str(rest);
}

fn write_arg(arg: &LogArg) {
    match *arg {
        LogArg::None => {}
        LogArg::U32(v) => crate::console::_print(core::format_args!("{v}")),
        LogArg::I32(v) => crate::console::_print(core::format_args!("{v}")),
        LogArg::F32(v) => crate::console::_print(core::format_args!("{v}")),
        LogArg::Str(s) => crate::console::write_str(s),
    }
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

/// Log a message at the given level. ISR-safe: pushing a frame does no
/// formatting and never blocks (see the module docs for why arguments are
/// a small closed set — `u32`/`i32`/`f32`/`&'static str` — rather than a
/// full `format_args!`-style template). Up to two `{}` placeholders in
/// `$msg` are substituted, in order, at drain time:
///
/// ```ignore
/// rivet::log!(Level::Info, "task {} spawned", id);
/// rivet::log!(Level::Warn, "retry {}/{}", attempt, max);
/// ```
#[macro_export]
macro_rules! log {
    ($level:expr, $msg:expr) => {
        $crate::log::push(
            $level,
            $msg,
            $crate::log::LogArg::None,
            $crate::log::LogArg::None,
        )
    };
    ($level:expr, $msg:expr, $a:expr) => {
        $crate::log::push(
            $level,
            $msg,
            $crate::log::LogArg::from($a),
            $crate::log::LogArg::None,
        )
    };
    ($level:expr, $msg:expr, $a:expr, $b:expr) => {
        $crate::log::push(
            $level,
            $msg,
            $crate::log::LogArg::from($a),
            $crate::log::LogArg::from($b),
        )
    };
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn log_arg_from_conversions() {
        assert!(matches!(LogArg::from(5u32), LogArg::U32(5)));
        assert!(matches!(LogArg::from(-3i32), LogArg::I32(-3)));
        assert!(matches!(LogArg::from("hi"), LogArg::Str("hi")));
        match LogArg::from(1.5f32) {
            LogArg::F32(v) => assert!((v - 1.5).abs() < f32::EPSILON),
            _ => panic!("expected F32"),
        }
    }

    /// Pure substitution logic — doesn't touch the ring or console (the
    /// host `port::host` console write is a no-op, so a real end-to-end
    /// check of what actually got written needs the QEMU suite;
    /// `examples/*/src/bin/report_test.rs`'s `hello from A, i={0..4}`
    /// golden lines are that check). This test isolates `write_interpolated`'s
    /// placeholder-counting/fallback behavior instead, by checking it
    /// against a fake sink is impossible without one — so it only
    /// checks it doesn't panic across the boundary cases (0, 1, 2, and
    /// more `{}` than args).
    #[test]
    fn write_interpolated_boundary_cases_do_not_panic() {
        write_interpolated("no placeholders", &[LogArg::None, LogArg::None]);
        write_interpolated("one {}", &[LogArg::U32(1), LogArg::None]);
        write_interpolated("two {} and {}", &[LogArg::U32(1), LogArg::Str("x")]);
        write_interpolated("three {} {} {}", &[LogArg::U32(1), LogArg::U32(2)]);
    }
}
