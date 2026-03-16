//! Rivet RTOS on ESP32-C3 with UART logs.
//!
//! Build (ready to flash, no flash performed):
//!   cargo build --example esp32c3_uart --target riscv32imc-unknown-none-elf --release --features esp32c3
//!
//! Flash and monitor (when ready):
//!   espflash flash --monitor target/riscv32imc-unknown-none-elf/release/examples/esp32c3_uart --port /dev/ttyACM0
//!
//! Note: ESP32-S3 is Xtensa; this target is ESP32-C3 (RISC-V).

#![no_std]
#![no_main]

#[cfg(not(target_arch = "riscv32"))]
compile_error!("Build with: --target riscv32imc-unknown-none-elf");

#[cfg(not(feature = "esp32c3"))]
compile_error!("Build with: --features esp32c3");

use core::arch::global_asm;

extern "C" {
    static __data_load: u8;
    static __data_start: u8;
    static __data_end: u8;
    static __bss_start: u8;
    static __bss_end: u8;
    static __stack_top: u8;
}

global_asm!(
    ".section .text._start",
    ".global _start",
    "_start:",
    "  la    t0, __data_load",
    "  la    t1, __data_start",
    "  la    t2, __data_end",
    "1:",
    "  bgeu  t1, t2, 2f",
    "  lw    t3, 0(t0)",
    "  sw    t3, 0(t1)",
    "  addi  t0, t0, 4",
    "  addi  t1, t1, 4",
    "  j     1b",
    "2:",
    "  la    t0, __bss_start",
    "  la    t1, __bss_end",
    "3:",
    "  bgeu  t0, t1, 4f",
    "  sw    zero, 0(t0)",
    "  addi  t0, t0, 4",
    "  j     3b",
    "4:",
    "  la    sp, __stack_top",
    "  call  rust_main",
    "  ebreak",
);

const STACK_SIZE: usize = 2048;
const FRAME_SIZE: usize = 64;
const STACK_ALIGN: usize = 16;

#[repr(align(16))]
struct AlignedStack([u8; STACK_SIZE]);
static mut TASK0_STACK: AlignedStack = AlignedStack([0; STACK_SIZE]);
static mut TASK1_STACK: AlignedStack = AlignedStack([0; STACK_SIZE]);

#[no_mangle]
fn rust_main() -> ! {
    rivet_rtos::board::esp32c3::uart_init();
    rivet_rtos::board::esp32c3::uart_print("Rivet RTOS ESP32-C3\n");

    rivet_rtos::kernel::scheduler_init();
    rivet_rtos::board::esp32c3::install_port();

    let sp0 = unsafe {
        let base = core::ptr::addr_of!(TASK0_STACK.0) as usize;
        let frame_start = (base + STACK_SIZE - FRAME_SIZE) & !(STACK_ALIGN - 1);
        let ra_slot = frame_start as *mut usize;
        *ra_slot = task0_entry as *const () as usize;
        frame_start
    };
    let sp1 = unsafe {
        let base = core::ptr::addr_of!(TASK1_STACK.0) as usize;
        let frame_start = (base + STACK_SIZE - FRAME_SIZE) & !(STACK_ALIGN - 1);
        let ra_slot = frame_start as *mut usize;
        *ra_slot = task1_entry as *const () as usize;
        frame_start
    };

    assert!(rivet_rtos::kernel::register_task(0, sp0, 0));
    assert!(rivet_rtos::kernel::register_task(1, sp1, 1));
    rivet_rtos::kernel::set_current(0);
    rivet_rtos::board::esp32c3::uart_print("Starting task 0\n");

    unsafe {
        rivet_rtos::arch::switch_to_first(sp0);
    }
    rivet_rtos::board::esp32c3::uart_print("idle\n");
    loop {
        rivet_rtos::arch::critical_section(|| rivet_rtos::kernel::yield_task());
    }
}

fn task0_entry() -> ! {
    rivet_rtos::board::esp32c3::uart_print("Task 0 running\n");
    for _ in 0..5 {
        rivet_rtos::board::esp32c3::uart_print("0");
        rivet_rtos::arch::critical_section(|| rivet_rtos::kernel::yield_task());
        for _ in 0..100_000 {
            core::hint::spin_loop();
        }
    }
    rivet_rtos::board::esp32c3::uart_print("\nTask 0 done\n");
    loop {
        rivet_rtos::arch::critical_section(|| rivet_rtos::kernel::yield_task());
    }
}

fn task1_entry() -> ! {
    rivet_rtos::board::esp32c3::uart_print("Task 1 running\n");
    for _ in 0..5 {
        rivet_rtos::board::esp32c3::uart_print("1");
        rivet_rtos::arch::critical_section(|| rivet_rtos::kernel::yield_task());
        for _ in 0..100_000 {
            core::hint::spin_loop();
        }
    }
    rivet_rtos::board::esp32c3::uart_print("\nTask 1 done\n");
    loop {
        rivet_rtos::arch::critical_section(|| rivet_rtos::kernel::yield_task());
    }
}
