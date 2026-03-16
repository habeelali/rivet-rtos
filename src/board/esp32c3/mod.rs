//! ESP32-C3 board support: UART logs, tick timer, init.
//!
//! UART0 at 0x60000000 (AHB). FIFO data at +0x1C. 115200 8N1 after [`uart_init`].

use crate::arch;
use crate::kernel;

/// UART0 base (AHB). FIFO data register at base + 0x1C.
const UART0_BASE: u32 = 0x6000_0000;
const UART0_FIFO: *mut u32 = (UART0_BASE + 0x1C) as *mut u32;
const UART0_CONF0: *mut u32 = (UART0_BASE + 0x20) as *mut u32;
const UART0_CLKDIV: *mut u32 = (UART0_BASE + 0x14) as *mut u32;

/// Call once at startup to enable UART0 @ 115200 (e.g. from rust_main).
pub fn uart_init() {
    unsafe {
        *UART0_CLKDIV = 43;
        *UART0_CONF0 = 1;
    }
}

/// Write one byte to UART0 (blocking).
pub fn uart_write_byte(b: u8) {
    unsafe {
        core::ptr::write_volatile(UART0_FIFO, b as u32);
    }
}

/// Print a string to UART0 (for logs and panic).
pub fn uart_print(s: &str) {
    for &b in s.as_bytes() {
        uart_write_byte(b);
    }
}

/// Install the port: set kernel context switch to RISC-V implementation.
/// Call once at startup before starting the scheduler.
pub fn install_port() {
    kernel::set_context_switch(arch::context_switch);
}

/// Tick: call from timer ISR or main loop. Runs scheduler in critical section;
/// if a higher-priority (or round-robin next) task is ready, preempts and switches.
pub fn tick() {
    arch::critical_section(|| {
        kernel::tick();
    });
}
