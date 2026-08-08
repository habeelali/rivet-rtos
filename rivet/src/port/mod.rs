//! The RTOS/board port contract.
//!
//! `rivet` is a pure kernel: scheduler, TCB, executor, timers, sync
//! primitives, fault policy. It contains no MMIO and no `#[cfg(target_arch
//! = ...)]`. Everything that touches real hardware is declared here as an
//! `extern "Rust"` symbol and provided by two kinds of downstream crates:
//!
//! - **[`arch`]** ("Group A") — the CPU port: context switch, trap entry,
//!   per-arch memory-protection programming. Provided by a `rivet-arch-*`
//!   crate (e.g. `rivet-arch-riscv`, `rivet-arch-cortex-m`).
//! - **[`board`]** ("Group B") — the board port: clocks, console, tick
//!   source, exit/reset, watchdog. Provided by a `rivet-bsp-*` crate.
//!
//! Binding is by symbol name, resolved at final link time — a board that
//! forgets to implement a symbol gets a link error naming it, not a type
//! error. Bringing Rivet up on a new board means writing a new
//! `rivet-bsp-*` crate (and usually reusing an existing `rivet-arch-*`);
//! it never means touching this crate. See `docs/porting.md` for the full
//! contract and a worked example.
//!
//! On host targets (tests, `cargo check`), [`host`] provides both halves
//! with no-ops/fakes so the kernel's own test suite doesn't need a real
//! board at all.

pub mod arch;
pub mod board;

#[cfg(feature = "host-port")]
pub mod host;
