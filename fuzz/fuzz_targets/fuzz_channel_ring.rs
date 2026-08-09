//! Fuzz target: SPSC channel ring invariants (plan.md §1.5).
//!
//! Input: byte stream of try_send/try_recv ops. Invariants: every value
//! sent is received exactly once and in FIFO order; nothing received that
//! was not sent; the ring never returns a wrong value. The channel is a
//! single static reused across iterations (no per-iteration allocation, so
//! LeakSanitizer stays quiet); each iteration must fully drain it.

#![no_main]

use std::sync::OnceLock;

use libfuzzer_sys::fuzz_target;

use rivet::sync::{Channel, Receiver, Sender};

fn channel() -> &'static Channel<u16, 8> {
    static CHAN: OnceLock<Channel<u16, 8>> = OnceLock::new();
    CHAN.get_or_init(Channel::new)
}

// `Channel::split` succeeds exactly once (it's a one-shot `Once`-guarded
// handoff, like the SPSC rings `rivet::console`/`rivet::log` use) — split
// once here and reuse the same `Sender`/`Receiver` (both `&self`-based,
// no exclusive access needed) across every fuzz iteration, matching this
// target's own doc comment ("single static reused across iterations").
fn endpoints() -> &'static (Sender<'static, u16, 8>, Receiver<'static, u16, 8>) {
    static ENDPOINTS: OnceLock<(Sender<'static, u16, 8>, Receiver<'static, u16, 8>)> =
        OnceLock::new();
    ENDPOINTS.get_or_init(|| channel().split().expect("split channel once"))
}

fuzz_target!(|data: &[u8]| {
    let (tx, rx) = endpoints();

    // Expected FIFO of sent-but-not-received values (empty at iteration
    // start: the previous iteration drained fully).
    let mut sent: std::collections::VecDeque<u16> = std::collections::VecDeque::new();
    let mut next_value = 0u16;

    for (i, &b) in data.iter().enumerate() {
        if b % 2 == 0 {
            let v = next_value;
            next_value = next_value.wrapping_add(1);
            match tx.try_send(v) {
                Ok(()) => sent.push_back(v),
                Err(v) => {
                    // Ring full: must match the SPSC capacity bound.
                    assert!(sent.len() >= 7, "try_send failed with fewer than 7 in flight");
                    assert_eq!(v, next_value.wrapping_sub(1));
                }
            }
        } else {
            let got = rx.try_recv();
            match got {
                Some(v) => {
                    let expected = sent.pop_front().expect("received but nothing sent");
                    assert_eq!(v, expected, "ring returned a wrong value");
                }
                None => assert!(sent.is_empty(), "try_recv None with values in flight"),
            }
        }
        let _ = i;
    }

    // Full drain: everything sent must be received exactly once, in order.
    while let Some(v) = rx.try_recv() {
        let expected = sent.pop_front().expect("received but nothing sent");
        assert_eq!(v, expected, "ring returned a wrong value during final drain");
    }
    assert!(sent.is_empty(), "values left in flight at iteration end");
});
