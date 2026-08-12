//! Atomic shim: `loom::sync::atomic` under permutation testing, otherwise
//! `core::sync::atomic` — or, on a target with no native RMW atomics (see
//! the `atomics-polyfill` feature), `portable_atomic` instead (plan.md
//! §1.3).
//!
//! The lock-free core (`waker`, `sync/semaphore`, `sync/channel`,
//! `preempt/tcb`) is written against this module. Under normal builds
//! (`--cfg loom` absent, `atomics-polyfill` off) this is a zero-cost
//! alias of the core atomics; with `RUSTFLAGS='--cfg loom'` the same code
//! compiles against loom's atomics, letting the models in `tests/loom.rs`
//! explore every interleaving of the Acquire/Release orderings that are
//! otherwise justified only by prose comments. With `atomics-polyfill` on
//! (ARMv6-M boards — no LDREX/STREX, so `core`'s `AtomicU32` etc. don't
//! even have `compare_exchange`/`fetch_or`/`swap` on that target),
//! `portable_atomic` provides the identical API via a critical-section-
//! guarded fallback — every call site elsewhere in this crate is
//! unaffected either way.

#[cfg(loom)]
pub use loom::sync::atomic::*;

#[cfg(all(not(loom), feature = "atomics-polyfill"))]
pub use portable_atomic::*;

#[cfg(all(not(loom), not(feature = "atomics-polyfill")))]
pub use core::sync::atomic::*;
