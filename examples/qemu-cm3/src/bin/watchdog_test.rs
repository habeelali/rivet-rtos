//! Watchdog test (plan.md §3.5 / §3.6).
//!
//! The watchdog is armed with a short period; a task feeds it for a few
//! periods then stops. The watchdog fires: on Cortex-M the real
//! luminary-watchdog hardware resets the system (QEMU models reset-on-
//! expiry); on RISC-V the software watchdog resets via `riscv.sifive.test`
//! (0x7777). Either way "RIVET WATCHDOG TIMEOUT" is printed first — the
//! harness asserts the marker via golden-on-timeout (the reset reboots the
//! guest rather than exiting).

#![no_std]
#![no_main]

use core::panic::PanicInfo;

static FEEDS: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);

fn feeder(_: &'static ()) -> ! {
    // Feed 3 watchdog periods, then go silent.
    loop {
        let n = FEEDS.load(core::sync::atomic::Ordering::Acquire);
        if n >= 3 {
            // Stop feeding: the watchdog must fire.
            loop {
                core::hint::spin_loop();
            }
        }
        rivet::watchdog::feed();
        FEEDS.store(n + 1, core::sync::atomic::Ordering::Release);
        // Wait ~half a watchdog period between feeds.
        for _ in 0..200_000 {
            core::hint::spin_loop();
        }
    }
}

// ── Startup (Cortex-M3 boilerplate) ────────────────────────────────

extern "C" {
    static __data_load: u8;
    static __data_start: u8;
    static __data_end: u8;
    static __bss_start: u8;
    static __bss_end: u8;
}

/// # Safety
/// Runs at power-on reset as the vector-table Reset entry; performs the
/// .data copy and .bss zeroing, then starts the kernel.
#[no_mangle]
pub unsafe extern "C" fn Reset() -> ! {
    let data_load = core::ptr::addr_of!(__data_load);
    let data_start = core::ptr::addr_of!(__data_start);
    let data_end = core::ptr::addr_of!(__data_end);
    let count = data_end as usize - data_start as usize;
    for i in 0..count {
        core::ptr::write(
            (data_start as *mut u8).add(i),
            core::ptr::read(data_load.add(i)),
        );
    }

    let bss_start = core::ptr::addr_of!(__bss_start);
    let bss_end = core::ptr::addr_of!(__bss_end);
    let bss_count = bss_end as usize - bss_start as usize;
    for i in 0..bss_count {
        core::ptr::write((bss_start as *mut u8).add(i), 0);
    }

    rivet::init();
    rivet::arch::debug_print("Rivet CM3 watchdog_test: feeding then going silent\n");

    rivet::watchdog::init(rivet::time::Duration::from_millis(250));

    let _ = rivet::spawn_ptask!(stack = 512, priority = 2, entry = feeder, arg = ());

    rivet::run();
}

/// # Safety
/// Exception entry point installed in the vector table; never called
/// directly.
#[no_mangle]
pub unsafe extern "C" fn SysTick() {
    rivet::arch::cortex_m::systick_handler();
}

/// # Safety
/// Exception entry point installed in the vector table (via
/// `PROVIDE(... = DefaultHandler)` in the linker script); never called
/// directly.
#[no_mangle]
pub unsafe extern "C" fn DefaultHandler() {
    rivet::arch::debug_print("DEFAULT_HANDLER/FAULT\n");
    loop {
        core::hint::spin_loop();
    }
}

/// # Safety
/// Exception entry point installed in the vector table; never called
/// directly.
#[no_mangle]
pub unsafe extern "C" fn HardFault() {
    rivet::arch::debug_print("HARD_FAULT\n");
    loop {
        core::hint::spin_loop();
    }
}

/// # Safety
/// Exception entry point installed in the vector table; never called
/// directly.
#[no_mangle]
pub unsafe extern "C" fn UsageFault() {
    rivet::arch::debug_print("USAGE_FAULT\n");
    loop {
        core::hint::spin_loop();
    }
}

/// # Safety
/// Exception entry point installed in the vector table; never called
/// directly.
#[no_mangle]
pub unsafe extern "C" fn BusFault() {
    rivet::arch::debug_print("BUS_FAULT\n");
    loop {
        core::hint::spin_loop();
    }
}

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    rivet::arch::debug_print("PANIC: ");
    if let Some(loc) = info.location() {
        rivet::arch::debug_print(loc.file());
        rivet::arch::debug_print(":");
        let mut n = loc.line();
        let mut digits = [0u8; 10];
        let mut i = 0;
        while n > 0 {
            digits[i] = b'0' + (n % 10) as u8;
            n /= 10;
            i += 1;
        }
        let mut buf = [0u8; 10];
        for j in 0..i {
            buf[j] = digits[i - 1 - j];
        }
        if let Ok(s) = core::str::from_utf8(&buf[..i]) {
            rivet::arch::debug_print(s);
        }
    }
    rivet::arch::debug_print("\n");
    loop {
        core::hint::spin_loop();
    }
}
