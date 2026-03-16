//! Rivet RTOS — minimal RTOS for RISC-V in Rust.
//!
//! Target: ESP32-C3.

#![no_std]

pub mod arch;
pub mod board;
pub mod kernel;

/// Crate version for tests and tooling.
pub const VERSION: &str = env!("CARGO_PKG_VERSION");

/// Panic handler for RISC-V (bare metal). On host (tests) std provides it.
#[cfg(target_arch = "riscv32")]
#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}

#[cfg(test)]
mod tests {
    use crate::arch;
    use crate::kernel::{self, BinarySemaphore, TaskState};

    #[test]
    fn semaphore_block_and_signal() {
        kernel::scheduler_init();
        kernel::set_context_switch(arch::context_switch);
        assert!(kernel::register_task(0, 0x1000, 1));
        assert!(kernel::register_task(1, 0x2000, 1));
        kernel::set_current(0);
        let mut sem = BinarySemaphore::new_taken();
        sem.wait();
        // On RISC-V we actually switch to task 1; on host the stub returns and we stay in task 0.
        #[cfg(target_arch = "riscv32")]
        assert_eq!(kernel::get_current(), Some(1));
        #[cfg(target_arch = "riscv32")]
        assert_eq!(kernel::schedule(), Some(1));
        sem.signal();
        assert_eq!(kernel::get_task_state(0), Some(TaskState::Ready));
    }
}
