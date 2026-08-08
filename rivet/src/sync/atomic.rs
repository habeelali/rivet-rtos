//! Atomic shim: `loom::sync::atomic` under permutation testing, otherwise
//! `core::sync::atomic` (plan.md §1.3).
//!
//! The lock-free core (`waker`, `sync/semaphore`, `sync/channel`,
//! `preempt/tcb`) is written against this module. Under normal builds
//! (`--cfg loom` absent) this is a zero-cost alias of the core atomics;
//! with `RUSTFLAGS='--cfg loom'` the same code compiles against loom's
//! atomics, letting the models in `tests/loom.rs` explore every
//! interleaving of the Acquire/Release orderings that are otherwise
//! justified only by prose comments.

#[cfg(loom)]
pub use loom::sync::atomic::*;

#[cfg(not(loom))]
pub use core::sync::atomic::*;
