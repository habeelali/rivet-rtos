//! Shared helpers for `rivet-bsp-*` crates.
//!
//! Small, board-shape-agnostic pieces that would otherwise be
//! reimplemented per board: a software watchdog fallback for boards
//! without real watchdog hardware, and an NS16550-compatible UART driver
//! (the most common "some UART is mapped somewhere" case on RV32
//! platforms). Nothing here is wired to the port contract directly —
//! each BSP's own `#[no_mangle]` functions call into these.

#![no_std]

pub mod delay;
pub mod ns16550;
pub mod serial;
pub mod sw_watchdog;
