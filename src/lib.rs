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
#[cfg(all(target_arch = "riscv32", not(feature = "esp32c3")))]
#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}

/// Panic handler when building for ESP32-C3: log to UART then hang.
#[cfg(all(target_arch = "riscv32", feature = "esp32c3"))]
#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    crate::board::esp32c3::uart_print("panic: ");
    if let Some(loc) = info.location() {
        let mut buf = [0u8; 24];
        let line = loc.line();
        let mut i = 0usize;
        let mut n = line;
        if n == 0 {
            buf[i] = b'0';
            i += 1;
        } else {
            let mut digits = [0u8; 6];
            let mut d = 0;
            while n > 0 {
                digits[d] = b'0' + (n % 10) as u8;
                n /= 10;
                d += 1;
            }
            while d > 0 {
                d -= 1;
                buf[i] = digits[d];
                i += 1;
            }
        }
        crate::board::esp32c3::uart_print(core::str::from_utf8(&buf[..i]).unwrap_or("?"));
        crate::board::esp32c3::uart_print("\n");
    } else {
        crate::board::esp32c3::uart_print("(no location)\n");
    }
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
