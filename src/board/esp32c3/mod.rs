//! ESP32-C3 board support: tick timer, init.
//!
//! Stub for now. On real hardware: configure a timer to call [`kernel::tick`]
//! at the desired rate (e.g. 1 ms) from interrupt, and call [`arch::critical_section`]
//! around scheduler logic.

use crate::arch;
use crate::kernel;

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
