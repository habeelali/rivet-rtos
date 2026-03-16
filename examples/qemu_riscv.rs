//! Run Rivet on QEMU RISC-V (virt machine). Two tasks: one waits on a semaphore,
//! the other signals and exits via semihosting. Build and run:
//!
//!   cargo build --example qemu_riscv --target riscv32imc-unknown-none-elf --release
//!   ./scripts/run-qemu.sh
//!
//! Or: qemu-system-riscv32 -machine virt -cpu rv32 -bios none -kernel <elf> -nographic -semihosting

#![no_std]
#![no_main]

#[cfg(not(target_arch = "riscv32"))]
compile_error!("Build with: cargo build --example qemu_riscv --target riscv32imc-unknown-none-elf");

use core::arch::global_asm;
use rivet_rtos::kernel::BinarySemaphore;

extern "C" {
    static __stack_top: u8;
    static __bss_start: u8;
    static __bss_end: u8;
}

global_asm!(
    ".section .text._start",
    ".global _start",
    "_start:",
    "  la    sp, __stack_top",
    "  la    t0, __bss_start",
    "  la    t1, __bss_end",
    "1:",
    "  bgeu  t0, t1, 2f",
    "  sw    zero, 0(t0)",
    "  addi  t0, t0, 4",
    "  j     1b",
    "2:",
    "  call  rust_main",
    "  ebreak",
);

// QEMU virt machine: UART0 at 0x10000000 (NS16550 data register).
const UART0_DATA: *mut u8 = 0x1000_0000 as *mut u8;

fn uart_print(s: &str) {
    for &b in s.as_bytes() {
        unsafe { core::ptr::write_volatile(UART0_DATA, b) };
    }
}

static mut SEM: BinarySemaphore = BinarySemaphore::new_taken();
// Task 1 blocks here after signaling SEM so we switch back to Task 0.
static mut SEM2: BinarySemaphore = BinarySemaphore::new_taken();
const STACK_SIZE: usize = 1024;
const FRAME_SIZE: usize = 64;
/// RISC-V ABI requires 16-byte aligned stack pointer at call boundaries.
const STACK_ALIGN: usize = 16;

#[repr(align(16))]
struct AlignedStack([u8; STACK_SIZE]);
static mut TASK0_STACK: AlignedStack = AlignedStack([0; STACK_SIZE]);
static mut TASK1_STACK: AlignedStack = AlignedStack([0; STACK_SIZE]);

#[no_mangle]
fn rust_main() -> ! {
    uart_print("Rivet RTOS QEMU test\n");
    rivet_rtos::kernel::scheduler_init();
    rivet_rtos::kernel::set_context_switch(rivet_rtos::arch::context_switch);

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

    assert!(rivet_rtos::kernel::register_task(0, sp0, 1));
    assert!(rivet_rtos::kernel::register_task(1, sp1, 1));
    rivet_rtos::kernel::set_current(0);
    uart_print("Starting task 0 (will block on semaphore)\n");

    unsafe {
        rivet_rtos::arch::switch_to_first(sp0);
    }
    loop {}
}

fn task0_entry() -> ! {
    uart_print("  Task 0: waiting on semaphore...\n");
    rivet_rtos::arch::critical_section(|| {
        unsafe { (*core::ptr::addr_of_mut!(SEM)).wait() };
    });
    uart_print("  Task 0: got semaphore (resumed)\n");
    uart_print("Success! Exiting.\n");
    // Brief delay so UART output flushes before QEMU exits
    for _ in 0..1_000_000 {
        core::hint::spin_loop();
    }
    rivet_rtos::arch::qemu_exit_success();
}

fn task1_entry() -> ! {
    uart_print("  Task 1: signaling semaphore (Task 0 will wake)\n");
    rivet_rtos::arch::critical_section(|| {
        unsafe { (*core::ptr::addr_of_mut!(SEM)).signal() };
    });
    uart_print("  Task 1: blocking on SEM2 (switch back to Task 0)\n");
    rivet_rtos::arch::critical_section(|| {
        unsafe { (*core::ptr::addr_of_mut!(SEM2)).wait() };
    });
    uart_print("  Task 1: unexpected wake\n");
    loop {}
}
