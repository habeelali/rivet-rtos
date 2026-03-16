//! Architecture-specific code (context switch, critical sections).

pub mod riscv;
pub use riscv::{context_switch, critical_section, switch_to_first};

#[cfg(target_arch = "riscv32")]
pub use riscv::{qemu_exit_success, qemu_print_str};
