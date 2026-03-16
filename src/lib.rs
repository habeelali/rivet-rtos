//! Rivet RTOS — minimal RTOS for RISC-V in Rust.
//!
//! Target: ESP32-C3.

#![no_std]

pub mod arch;
pub mod board;
pub mod kernel;

/// Crate version for tests and tooling.
pub const VERSION: &str = env!("CARGO_PKG_VERSION");
