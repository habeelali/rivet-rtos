//! Host port: implements both Group A and Group B of the contract with
//! no-ops / fakes, so `cargo test -p rivet` (and anything else built for
//! a non-embedded target) links and runs without a real arch/BSP crate.
//!
//! Enabled by the `host-port` feature, which `test-support` pulls in
//! automatically — never enabled for embedded builds.

use core::sync::atomic::{AtomicU64, Ordering};

static TICKS: AtomicU64 = AtomicU64::new(0);

/// Test-only: reset the fake clock to zero. Part of the global reset done
/// by [`crate::kernel_test!`].
#[cfg(feature = "test-support")]
pub fn reset_test_clock() {
    TICKS.store(0, Ordering::Relaxed);
    CYCLES.store(0, Ordering::Relaxed);
}

// ── Group A (arch) ───────────────────────────────────────────────────

#[no_mangle]
extern "Rust" fn __rivet_arch_init() {}

#[no_mangle]
extern "Rust" fn __rivet_arch_idle() {
    core::hint::spin_loop();
}

#[no_mangle]
extern "Rust" fn __rivet_arch_request_reschedule() {
    // No-op on host: preemptive-tier tests exercise scheduling logic
    // directly rather than through a real interrupt-driven switch.
}

#[no_mangle]
extern "Rust" fn __rivet_arch_irq_save() -> usize {
    0
}

#[no_mangle]
extern "Rust" fn __rivet_arch_irq_restore(_token: usize) {}

#[no_mangle]
unsafe extern "Rust" fn __rivet_arch_init_task_stack(
    stack_ptr: *mut u8,
    _stack_len: usize,
    _entry_fn: usize,
    _arg: usize,
) -> usize {
    // No real bootstrap frame on host; just hand back a pointer into the
    // stack so `Tcb::sp` has a non-zero value. Never actually resumed via
    // a real context switch on this backend.
    stack_ptr as usize
}

#[no_mangle]
unsafe extern "Rust" fn __rivet_arch_start_first_task(_sp: usize) -> ! {
    loop {
        core::hint::spin_loop();
    }
}

#[no_mangle]
extern "Rust" fn __rivet_arch_on_switch_to(_stack_base: usize, _stack_size: usize) {}

#[no_mangle]
extern "Rust" fn __rivet_arch_guard_register(_guard_base: usize, _slot: usize) {}

#[no_mangle]
extern "Rust" fn __rivet_arch_scratch_open(_base: usize, _size: usize) {}

#[no_mangle]
extern "Rust" fn __rivet_arch_scratch_close() {}

#[no_mangle]
extern "Rust" fn __rivet_arch_min_task_stack() -> usize {
    // Host backend: no real frame; keep the same contract shape as the
    // embedded ports for consistent spawn checks.
    256
}

static CYCLES: AtomicU64 = AtomicU64::new(0);

#[no_mangle]
extern "Rust" fn __rivet_arch_cycle_count() -> u64 {
    // Fake but monotonic: advances once per call, which is all the
    // exec-time/latency accounting above this symbol ever assumes.
    CYCLES.fetch_add(1, Ordering::Relaxed)
}

#[no_mangle]
extern "Rust" fn __rivet_arch_irq_enable(_irq_num: u32) {}

#[no_mangle]
extern "Rust" fn __rivet_arch_irq_disable(_irq_num: u32) {}

#[no_mangle]
extern "Rust" fn __rivet_arch_irq_set_priority(_irq_num: u32, _priority: u8) {}

// ── Group B (board) ──────────────────────────────────────────────────

#[no_mangle]
extern "Rust" fn __rivet_board_init() {}

#[no_mangle]
extern "Rust" fn __rivet_board_now_us() -> u64 {
    // A monotonically increasing fake time.
    TICKS.fetch_add(1, Ordering::Relaxed)
}

#[no_mangle]
extern "Rust" fn __rivet_board_tick_start(_hz: u32) {}

#[no_mangle]
unsafe extern "Rust" fn __rivet_board_console_write(_ptr: *const u8, _len: usize) {
    // No-op on host.
}

#[no_mangle]
extern "Rust" fn __rivet_board_reset() -> ! {
    loop {
        core::hint::spin_loop();
    }
}

#[no_mangle]
extern "Rust" fn __rivet_board_exit(_code: u32) -> ! {
    loop {
        core::hint::spin_loop();
    }
}

#[no_mangle]
extern "Rust" fn __rivet_board_wdt_init(_period_us: u32) {}

#[no_mangle]
extern "Rust" fn __rivet_board_wdt_feed() {}

#[no_mangle]
extern "Rust" fn __rivet_board_wdt_check() {}
