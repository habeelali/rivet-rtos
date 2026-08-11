//! Debug console — the board's UART/semihosting/whatever, reached through
//! [`crate::port::board`]. Replaces the old `rivet::arch::debug_print`;
//! application code should use this module (or [`crate::print!`] /
//! [`crate::println!`]) instead of talking to the port directly.
//!
//! # Interrupt-driven mode (plan.md Phase 14)
//!
//! By default every write is a blocking spin on the board's polling
//! write, exactly as before — always correct, including from the fault
//! path (see below for why that matters). A board can opt in to
//! interrupt-driven TX by registering its own TX-empty IRQ handler
//! (through [`crate::irq`]) that calls [`tx_irq_next_byte`] and calling
//! [`enable_irq_tx`] once that's wired up; from then on, [`write_str`]/
//! [`write_bytes`] push into a ring buffer instead of blocking on
//! hardware directly, and the registered ISR drains it.
//!
//! **Deliberately drop-on-full, not block-on-full** — the same policy
//! [`crate::log`] uses, and for the same reason, sharpened by a real
//! constraint here: [`crate::fault::on_fault`] calls `console::write_str`
//! from *inside* the trap/exception handler on a single-hart kernel,
//! where no interrupt (including the one TX handler that would ever
//! drain the ring) can preempt the trap handler that's currently running.
//! Blocking there would deadlock permanently, not just stall — dropping
//! and counting is the only safe choice.
//!
//! RX is push-only from the board's side ([`on_rx_byte`], called from a
//! registered RX IRQ handler) and pull-only from the application side
//! ([`try_read_byte`]) — genuinely additive, doesn't touch the existing
//! write path at all.

use core::fmt::{self, Write};
use core::sync::atomic::{AtomicBool, Ordering};

use crate::sync::{Channel, Once, Receiver, Sender};

const RX_CAPACITY: usize = 64;
const TX_CAPACITY: usize = 256;

#[cfg(not(loom))]
static RX_CHANNEL: Channel<u8, RX_CAPACITY> = Channel::new();
#[cfg(loom)]
loom::lazy_static! {
    static ref RX_CHANNEL: Channel<u8, RX_CAPACITY> = Channel::new();
}

#[cfg(not(loom))]
static RX_SENDER: Once<Sender<'static, u8, RX_CAPACITY>> = Once::new();
#[cfg(loom)]
loom::lazy_static! {
    static ref RX_SENDER: Once<Sender<'static, u8, RX_CAPACITY>> = Once::new();
}

#[cfg(not(loom))]
static RX_RECEIVER: Once<Receiver<'static, u8, RX_CAPACITY>> = Once::new();
#[cfg(loom)]
loom::lazy_static! {
    static ref RX_RECEIVER: Once<Receiver<'static, u8, RX_CAPACITY>> = Once::new();
}

#[cfg(not(loom))]
static TX_CHANNEL: Channel<u8, TX_CAPACITY> = Channel::new();
#[cfg(loom)]
loom::lazy_static! {
    static ref TX_CHANNEL: Channel<u8, TX_CAPACITY> = Channel::new();
}

#[cfg(not(loom))]
static TX_SENDER: Once<Sender<'static, u8, TX_CAPACITY>> = Once::new();
#[cfg(loom)]
loom::lazy_static! {
    static ref TX_SENDER: Once<Sender<'static, u8, TX_CAPACITY>> = Once::new();
}

#[cfg(not(loom))]
static TX_RECEIVER: Once<Receiver<'static, u8, TX_CAPACITY>> = Once::new();
#[cfg(loom)]
loom::lazy_static! {
    static ref TX_RECEIVER: Once<Receiver<'static, u8, TX_CAPACITY>> = Once::new();
}

static IRQ_TX_ACTIVE: AtomicBool = AtomicBool::new(false);

/// Called once from [`crate::init`], splitting both rings up front so the
/// first write/read anywhere never pays for it.
pub(crate) fn init() {
    if let Some((tx, rx)) = RX_CHANNEL.split() {
        let _ = RX_SENDER.set(tx);
        let _ = RX_RECEIVER.set(rx);
    }
    if let Some((tx, rx)) = TX_CHANNEL.split() {
        let _ = TX_SENDER.set(tx);
        let _ = TX_RECEIVER.set(rx);
    }
}

/// Switch [`write_str`]/[`write_bytes`] to interrupt-driven mode. Call
/// this once the board's TX-empty IRQ handler is registered and enabled
/// (it must already be able to call [`tx_irq_next_byte`] and re-arm/
/// disable the hardware interrupt itself — this module has no MMIO
/// access of its own).
pub fn enable_irq_tx() {
    IRQ_TX_ACTIVE.store(true, Ordering::Release);
}

/// Called from the board's TX-empty ISR: pull the next queued byte, if
/// any, for the ISR to write to hardware. `None` means the ring is
/// empty — the ISR should disable the TX interrupt at that point (it
/// will be re-armed by the next dropped-into-empty-ring write, via
/// [`crate::port::arch::request_reschedule`]-style "kick" the board's own
/// IRQ handler is responsible for, matching how it originally armed it).
pub fn tx_irq_next_byte() -> Option<u8> {
    TX_RECEIVER.get().and_then(|rx| rx.try_recv())
}

/// Called from the board's RX ISR with one received byte.
pub fn on_rx_byte(b: u8) {
    if let Some(tx) = RX_SENDER.get() {
        // Drop-on-full: a byte arriving faster than any consumer reads
        // means there's nobody waiting for it right now anyway.
        let _ = tx.try_send(b);
    }
}

/// Non-blocking read of one received byte (task context). `None` if
/// nothing is buffered, or interrupt-driven RX was never wired up.
pub fn try_read_byte() -> Option<u8> {
    RX_RECEIVER.get().and_then(|rx| rx.try_recv())
}

fn write_bytes_irq(bytes: &[u8]) -> bool {
    let Some(tx) = TX_SENDER.get() else {
        return false;
    };
    // The whole call — every byte's push, the prime, and the kick — runs
    // under one `critical::enter`, not per-byte. Two things depend on
    // this: (1) multiple concurrent producers (any task, or the fault
    // path from trap context) pushing into an SPSC channel need
    // serializing into one logical producer, same as `crate::log`; a
    // *per-byte* critical section still lets one task's message be
    // preempted mid-string by another task's, interleaving their text
    // byte-by-byte on the wire — observed directly, not hypothetical.
    // (2) the "prime" write below must never race the hardware ISR
    // pulling from the same SPSC receiver.
    crate::critical::enter(|| {
        for &b in bytes {
            // Order-preserving backpressure, not drop-on-full: since the
            // whole call runs with the ISR masked, the ring can never
            // drain *during* this push on its own — so on a full ring,
            // pull the oldest queued byte out and write it directly
            // (polling, always completes, can't deadlock) to make room,
            // then retry. This never loses a byte and never reorders one
            // relative to the others; it only ever costs a few polling
            // writes on a message that overruns the ring's capacity.
            while tx.try_send(b).is_err() {
                if let Some(old) = tx_irq_next_byte() {
                    crate::port::board::console_write(&[old]);
                } else {
                    break; // ring reported full but is now empty: retry
                }
            }
        }
        // "Prime the pump": both the NS16550 and PL011 TX-empty condition
        // are edge-triggered on the *transition* to empty, not
        // level-sensed — merely re-enabling the interrupt mask in
        // `console_kick_tx` doesn't recreate that edge if no new byte is
        // ever written, so a ring that goes idle and is then written to
        // again would sit queued forever. Writing one byte here directly
        // guarantees a real transmit-complete event soon, which *does*
        // re-assert the interrupt for whatever's left.
        if let Some(b) = tx_irq_next_byte() {
            crate::port::board::console_write(&[b]);
        }
        // Enable the hardware TX interrupt so the primed byte's
        // completion (and everything queued behind it) keeps draining
        // without further help from here.
        crate::port::board::console_kick_tx();
    });
    true
}

pub fn write_str(s: &str) {
    write_bytes(s.as_bytes());
}

pub fn write_bytes(bytes: &[u8]) {
    if IRQ_TX_ACTIVE.load(Ordering::Acquire) && write_bytes_irq(bytes) {
        return;
    }
    // plan.md Phase 29/30, found on real ESP32-S3 dual-core hardware: the
    // polling fallback below is a direct, unsynchronized hardware
    // register write on every board that uses it (confirmed for S3:
    // `rivet-bsp-esp32s3::__rivet_board_console_write` polls
    // `UART0.status().txfifo_cnt()` and writes `UART0.fifo()` with no
    // lock at all) — this module's own docs already say the *design*
    // assumes "on a single-hart kernel" for the fault-path write, and
    // that assumption silently stopped holding the moment a real second
    // hart existed: two harts calling this concurrently interleave their
    // byte writes on the shared UART FIFO, confirmed to produce genuinely
    // corrupted binary garbage on the wire, not just interleaved-but-
    // readable text — including fault diagnostics a human needs to
    // actually read.
    //
    // A `critical::enter`-wrapped (unconditionally blocking) version was
    // tried and reverted: it introduces exactly the failure mode this
    // module's own docs warn about for the fault path — a lock that
    // *blocks* until the other hart releases it turns "one hart crashed"
    // into "both harts silently hang forever" the moment the other hart
    // is genuinely wedged while holding it. Fault-path output must never
    // be able to block on another hart's cooperation, full stop.
    //
    // The bounded-retry version below was *also* provisionally reverted
    // once, on the belief it hung `mutex_test`'s QEMU stress phase on
    // both Cortex-M targets — that belief was wrong. Phase 30 found the
    // actual cause: `mutex_test`'s 2,000,000-iteration contended-mutex
    // phase genuinely takes well over the 15-120s capture windows used
    // to test it (150+ real seconds on STM32 hardware at 16MHz), on
    // *pristine, unmodified* code too — confirmed by reverting every
    // session change, including this file, back to the original
    // unsynchronized write, and reproducing the identical "no output"
    // symptom with a short capture window. This was never a regression
    // from the lock below: a bounded-retry try-lock cannot hang
    // indefinitely by construction — it gives up and writes
    // unsynchronized after `LOCK_SPIN_LIMIT` iterations, a fixed, small
    // cost per call, entirely unrelated to how long a *caller's own*
    // workload takes to reach its next print. Re-verified against the
    // full `riscv`/`cm3`/`mps2` QEMU suites and real STM32/S3/C6
    // hardware, with adequate timeouts this time, before being kept.
    let mut spins: u32 = 0;
    while CONSOLE_WRITE_LOCK
        .compare_exchange_weak(false, true, Ordering::Acquire, Ordering::Relaxed)
        .is_err()
    {
        spins += 1;
        if spins >= LOCK_SPIN_LIMIT {
            crate::port::board::console_write(bytes);
            return;
        }
        core::hint::spin_loop();
    }
    crate::port::board::console_write(bytes);
    CONSOLE_WRITE_LOCK.store(false, Ordering::Release);
}

/// Bounded-retry lock for [`write_bytes`]'s polling path — see its own
/// comment for why this is deliberately not `critical::enter` (which
/// would block unboundedly). Plain `AtomicBool`, not the crate's usual
/// nesting-aware `critical::enter`: this lock is only ever held for the
/// duration of one `port::board::console_write` call, never nested.
static CONSOLE_WRITE_LOCK: AtomicBool = AtomicBool::new(false);
/// How many spin iterations to wait for [`CONSOLE_WRITE_LOCK`] before
/// giving up and writing unsynchronized. Not calibrated against any
/// particular board's clock — large enough that a healthy other hart's
/// brief, normal-length write (a handful of bytes, one polling loop each)
/// reliably finishes within it, small enough that a genuinely wedged
/// other hart doesn't stall this one's own diagnostic output for long.
const LOCK_SPIN_LIMIT: u32 = 100_000;

/// Synchronously drain any bytes still queued in the TX ring, via the
/// blocking polling write. No-op if interrupt-driven TX was never
/// enabled (nothing can be queued there).
///
/// Call this before anything that terminates or resets the guest right
/// after printing diagnostics — [`crate::fault::on_fault`]'s `Panic`
/// policy, the default panic handler, a watchdog timeout — since all of
/// them print a final message and then call [`crate::port::board::reset`]
/// or exit essentially immediately. Without a synchronous flush there,
/// that message would very likely be lost: it's sitting in the TX ring
/// waiting for the interrupt-driven ISR to drain it, but the guest halts
/// before that interrupt ever gets a chance to fire. Diagnostic output a
/// human needs to actually see must not depend on an interrupt that may
/// never come.
pub fn flush_sync() {
    // The TX ring's receiver end is SPSC — normally consumed only by the
    // board's hardware TX-empty ISR. Draining it here too, without
    // excluding that ISR, would be a second concurrent consumer racing
    // on the same `head` index (observed directly: this caused real
    // output truncation on Cortex-M, where interrupts stay enabled
    // through this call unless something masks them). `critical::enter`
    // makes this genuinely the only consumer for its duration.
    crate::critical::enter(|| {
        while let Some(b) = tx_irq_next_byte() {
            crate::port::board::console_write(&[b]);
        }
    });
}

struct Console;

impl Write for Console {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        write_str(s);
        Ok(())
    }
}

#[doc(hidden)]
pub fn _print(args: fmt::Arguments) {
    // A formatting error here would mean a `fmt::Write` impl returned
    // `Err` for a plain UART byte write, which never fails.
    let _ = Console.write_fmt(args);
}

/// Write formatted text to the debug console. See [`println!`] for a
/// version that appends a newline.
#[macro_export]
macro_rules! print {
    ($($arg:tt)*) => {{
        $crate::console::_print(core::format_args!($($arg)*));
    }};
}

/// Write formatted text to the debug console, followed by a newline.
#[macro_export]
macro_rules! println {
    () => { $crate::print!("\n") };
    ($($arg:tt)*) => {{
        $crate::console::_print(core::format_args!($($arg)*));
        $crate::print!("\n");
    }};
}
