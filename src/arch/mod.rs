//! Architecture-specific code (context switch, critical sections).

pub mod riscv;
pub use riscv::{context_switch, critical_section};
