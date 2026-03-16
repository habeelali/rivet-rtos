//! Full-depth QEMU test: priority, round-robin, semaphores, preemptive tick, 4 tasks.
//!
//! Run: ./scripts/run-qemu.sh full   (or ./scripts/run-qemu.sh with no args)
//! Build: cargo build --example qemu_riscv_full --target riscv32imc-unknown-none-elf --release
//!
//! Phases (all in one run):
//!   1. Priority + round-robin: tasks 0,1,2 (prio 0,1,0) alternate via tick(); higher prio runs first.
//!   2. Semaphore: task 0 blocks on SEM, task 1 signals then blocks on SEM2, task 0 resumes and signals SEM2.
//!   3. Preemption: three tasks print and tick() in a loop; output interleaved.
//!   4. Exit with success.

#![no_std]
#![no_main]

#[cfg(not(target_arch = "riscv32"))]
compile_error!("Build with: cargo build --example qemu_riscv_full --target riscv32imc-unknown-none-elf");

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

const UART0_DATA: *mut u8 = 0x1000_0000 as *mut u8;

fn uart_print(s: &str) {
    for &b in s.as_bytes() {
        unsafe { core::ptr::write_volatile(UART0_DATA, b) };
    }
}

static mut SEM: BinarySemaphore = BinarySemaphore::new_taken();
static mut SEM2: BinarySemaphore = BinarySemaphore::new_taken();

const STACK_SIZE: usize = 2048;
const FRAME_SIZE: usize = 64;
const STACK_ALIGN: usize = 16;

#[repr(align(16))]
struct AlignedStack([u8; STACK_SIZE]);
static mut TASK0_STACK: AlignedStack = AlignedStack([0; STACK_SIZE]);
static mut TASK1_STACK: AlignedStack = AlignedStack([0; STACK_SIZE]);
static mut TASK2_STACK: AlignedStack = AlignedStack([0; STACK_SIZE]);
static mut TASK3_STACK: AlignedStack = AlignedStack([0; STACK_SIZE]);

fn spin(n: u32) {
    for _ in 0..n {
        core::hint::spin_loop();
    }
}

#[no_mangle]
fn rust_main() -> ! {
    uart_print("Rivet RTOS full-depth QEMU test\n");
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
    let sp2 = unsafe {
        let base = core::ptr::addr_of!(TASK2_STACK.0) as usize;
        let frame_start = (base + STACK_SIZE - FRAME_SIZE) & !(STACK_ALIGN - 1);
        let ra_slot = frame_start as *mut usize;
        *ra_slot = task2_entry as *const () as usize;
        frame_start
    };
    let sp3 = unsafe {
        let base = core::ptr::addr_of!(TASK3_STACK.0) as usize;
        let frame_start = (base + STACK_SIZE - FRAME_SIZE) & !(STACK_ALIGN - 1);
        let ra_slot = frame_start as *mut usize;
        *ra_slot = task3_entry as *const () as usize;
        frame_start
    };

    assert!(rivet_rtos::kernel::register_task(0, sp0, 0));
    assert!(rivet_rtos::kernel::register_task(1, sp1, 1));
    assert!(rivet_rtos::kernel::register_task(2, sp2, 0));
    assert!(rivet_rtos::kernel::register_task(3, sp3, 0));

    rivet_rtos::kernel::set_current(0);
    uart_print("Phase 1: priority (1) + round-robin (0,2) via tick()\n");

    unsafe {
        rivet_rtos::arch::switch_to_first(sp0);
    }
    loop {}
}

// Task 0: prio 0. Phase 1: print "0" + tick. Phase 2: wait(SEM), then "0 resumed", signal(SEM2). Phase 3: print "0" + yield a few times. Then exit.
fn task0_entry() -> ! {
    rivet_rtos::arch::critical_section(|| {
        for _ in 0..4 {
            uart_print("0");
            rivet_rtos::kernel::tick();
            spin(30_000);
        }
    });
    uart_print("\nTask 0: blocking on SEM...\n");
    rivet_rtos::arch::critical_section(|| {
        unsafe { (*core::ptr::addr_of_mut!(SEM)).wait() };
    });
    uart_print("Task 0: resumed; signaling SEM2\n");
    rivet_rtos::arch::critical_section(|| {
        unsafe { (*core::ptr::addr_of_mut!(SEM2)).signal() };
    });
    rivet_rtos::arch::critical_section(|| {
        for _ in 0..4 {
            uart_print("0");
            rivet_rtos::kernel::yield_task();
            spin(30_000);
        }
    });
    uart_print("\nTask 0 done.\n");
    uart_print("SUCCESS: full-depth test complete. Exiting.\n");
    spin(100_000);
    rivet_rtos::arch::qemu_exit_success();
}

// Task 1: prio 1. Phase 1: print "1" + tick. Phase 2: signal(SEM), wait(SEM2). Phase 3: print "1" + yield.
fn task1_entry() -> ! {
    rivet_rtos::arch::critical_section(|| {
        for _ in 0..4 {
            uart_print("1");
            rivet_rtos::kernel::tick();
            spin(30_000);
        }
    });
    uart_print("\nTask 1: signaling SEM, then blocking on SEM2...\n");
    rivet_rtos::arch::critical_section(|| {
        unsafe { (*core::ptr::addr_of_mut!(SEM)).signal() };
    });
    rivet_rtos::arch::critical_section(|| {
        unsafe { (*core::ptr::addr_of_mut!(SEM2)).wait() };
    });
    uart_print("Task 1: resumed.\n");
    rivet_rtos::arch::critical_section(|| {
        for _ in 0..4 {
            uart_print("1");
            rivet_rtos::kernel::yield_task();
            spin(30_000);
        }
    });
    uart_print("\nTask 1 done.\n");
    loop {
        rivet_rtos::arch::critical_section(|| rivet_rtos::kernel::yield_task());
    }
}

// Task 2: prio 0. Phase 1 and 2: print "2" + tick/yield.
fn task2_entry() -> ! {
    rivet_rtos::arch::critical_section(|| {
        for _ in 0..4 {
            uart_print("2");
            rivet_rtos::kernel::tick();
            spin(30_000);
        }
    });
    uart_print("\nTask 2: phase 1 done.\n");
    rivet_rtos::arch::critical_section(|| {
        for _ in 0..4 {
            uart_print("2");
            rivet_rtos::kernel::yield_task();
            spin(30_000);
        }
    });
    uart_print("\nTask 2 done.\n");
    loop {
        rivet_rtos::arch::critical_section(|| rivet_rtos::kernel::yield_task());
    }
}

// Task 3: prio 0. Phase 1 and 2: print "3" + tick/yield.
fn task3_entry() -> ! {
    rivet_rtos::arch::critical_section(|| {
        for _ in 0..4 {
            uart_print("3");
            rivet_rtos::kernel::tick();
            spin(30_000);
        }
    });
    uart_print("\nTask 3: phase 1 done.\n");
    rivet_rtos::arch::critical_section(|| {
        for _ in 0..4 {
            uart_print("3");
            rivet_rtos::kernel::yield_task();
            spin(30_000);
        }
    });
    uart_print("\nTask 3 done.\n");
    loop {
        rivet_rtos::arch::critical_section(|| rivet_rtos::kernel::yield_task());
    }
}
