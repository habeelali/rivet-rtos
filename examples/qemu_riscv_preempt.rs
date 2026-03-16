//! Preemptive scheduler demo on QEMU RISC-V: two tasks alternate via tick().
//!
//! Run: ./scripts/run-qemu.sh preempt
//! Or: cargo build --example qemu_riscv_preempt --target riscv32imc-unknown-none-elf --release
//!     qemu-system-riscv32 -machine virt -kernel target/.../qemu_riscv_preempt -nographic -serial mon:stdio -semihosting

#![no_std]
#![no_main]

#[cfg(not(target_arch = "riscv32"))]
compile_error!("Build with: cargo build --example qemu_riscv_preempt --target riscv32imc-unknown-none-elf");

use core::arch::global_asm;

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

const UART0_DATA: *mut u8 = 0x1000_0000 as *mut u8;

fn uart_print(s: &str) {
    for &b in s.as_bytes() {
        unsafe { core::ptr::write_volatile(UART0_DATA, b) };
    }
}

const STACK_SIZE: usize = 1024;
const FRAME_SIZE: usize = 64;
const STACK_ALIGN: usize = 16;

#[repr(align(16))]
struct AlignedStack([u8; STACK_SIZE]);
static mut TASK0_STACK: AlignedStack = AlignedStack([0; STACK_SIZE]);
static mut TASK1_STACK: AlignedStack = AlignedStack([0; STACK_SIZE]);

#[no_mangle]
fn rust_main() -> ! {
    uart_print("Rivet RTOS preemptive tick test\n");
    uart_print("Two tasks alternate via kernel::tick(); expect 0 and 1 interleaved.\n");

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

    unsafe {
        rivet_rtos::arch::switch_to_first(sp0);
    }
    loop {}
}

fn task0_entry() -> ! {
    for _ in 0..10 {
        uart_print("0");
        rivet_rtos::arch::critical_section(|| rivet_rtos::kernel::tick());
        for _ in 0..50_000 {
            core::hint::spin_loop();
        }
    }
    uart_print("\nTask 0 done. Exiting.\n");
    for _ in 0..1_000_000 {
        core::hint::spin_loop();
    }
    rivet_rtos::arch::qemu_exit_success();
}

fn task1_entry() -> ! {
    for _ in 0..10 {
        uart_print("1");
        rivet_rtos::arch::critical_section(|| rivet_rtos::kernel::tick());
        for _ in 0..50_000 {
            core::hint::spin_loop();
        }
    }
    uart_print("\nTask 1 done.\n");
    loop {}
}
