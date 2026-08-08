//! Dummy arch implementation for host-side testing.
//! All operations are no-ops or return fixed values.

use core::sync::atomic::{AtomicU64, Ordering};

/// Host backend: no real frame; keep the same contract as the embedded
/// ports for consistent spawn checks.
pub const MIN_TASK_STACK: usize = 256;

static TICKS: AtomicU64 = AtomicU64::new(0);

pub fn sleep() {
    // On host, just yield to the OS scheduler.
    core::hint::spin_loop();
}

pub fn pend_executor() {
    // No-op on host.
}

pub fn early_init() {
    // No-op on host.
}

pub fn now_micros() -> u64 {
    // Return a monotonically increasing fake time.
    TICKS.fetch_add(1, Ordering::Relaxed)
}

/// Test-only: reset the fake clock to zero. Part of the global reset done
/// by [`crate::kernel_test!`].
#[cfg(feature = "test-support")]
pub fn reset_test_clock() {
    TICKS.store(0, Ordering::Relaxed);
}

pub fn debug_print_hex32(_n: u32) {}

pub fn debug_print(_s: &str) {
    // No-op on host.
}

pub fn exit_success() -> ! {
    #[cfg(test)]
    {
        // Return from the executor for testing.
        // The test harness doesn't support true diverging functions.
        // Use a sentinel to break out of the executor loop in tests.
        extern "Rust" {
            fn __rivet_test_exit() -> !;
        }
        // If the linker doesn't provide this, just loop.
        loop {
            core::hint::spin_loop();
        }
    }
    #[cfg(not(test))]
    loop {
        core::hint::spin_loop();
    }
}

pub fn exit_failure(_code: u32) -> ! {
    // Host backend: never actually called by tests; spin to satisfy `-> !`.
    loop {
        core::hint::spin_loop();
    }
}

pub fn system_reset() -> ! {
    // Host backend: no reset mechanism; spin to satisfy `-> !`.
    loop {
        core::hint::spin_loop();
    }
}

pub fn on_switch_to(_stack_base: usize, _stack_size: usize) {
    // No memory-protection unit on the host backend.
}
pub fn mpu_allow_scratch(_base: usize, _size: usize) {}
pub fn mpu_clear_scratch() {}
pub fn pmp_register_guard(_guard_base: usize, _entry: usize) {}

// ── Preemptive tier stubs (host: no real stack switching) ────────────

pub unsafe fn init_task_stack(stack: &mut [u8], _entry_fn: usize, _arg: usize) -> usize {
    // No real bootstrap frame on host; just hand back a pointer into the
    // stack so `Tcb::sp` has a non-zero value. Never actually resumed via
    // a real context switch on this backend.
    stack.as_mut_ptr() as usize
}

pub unsafe fn start_first_task(_sp: usize) -> ! {
    loop {
        core::hint::spin_loop();
    }
}

pub fn yield_now() {
    // No-op on host: preemptive-tier tests exercise scheduling logic
    // directly rather than through a real interrupt-driven switch.
}
