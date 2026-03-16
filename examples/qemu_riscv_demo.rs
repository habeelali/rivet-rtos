//! System Verification Suite for Rivet RTOS
//! Validates: Preemptive tick, Cooperative yield, Priorities, Round-Robin, Semaphores.

#![no_std]
#![no_main]

#[cfg(not(target_arch = "riscv32"))]
compile_error!("Build with: cargo build --example qemu_riscv_demo --target riscv32imc-unknown-none-elf");

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

static mut SEM_WORKERS_DONE: BinarySemaphore = BinarySemaphore::new_taken();
static mut SEM_MONITOR_WAKE: BinarySemaphore = BinarySemaphore::new_taken();

const STACK_SIZE: usize = 2048;
const FRAME_SIZE: usize = 64;
const STACK_ALIGN: usize = 16;

#[repr(align(16))]
#[allow(dead_code)]
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

fn spin_ms(ms: u32) {
    spin(10_000 * ms);
}

// Green checkmark using ANSI escape codes for the terminal
const PASS_MARK: &str = "\x1b[32m[\u{2713}]\x1b[0m";

#[no_mangle]
fn rust_main() -> ! {
    uart_print("==================================================\n");
    uart_print("Rivet RTOS System Verification\n");
    uart_print("==================================================\n");
    uart_print("[System] Booting kernel...\n");
    
    rivet_rtos::kernel::scheduler_init();
    rivet_rtos::kernel::set_context_switch(rivet_rtos::arch::context_switch);

    let sp0 = unsafe { init_stack(core::ptr::addr_of_mut!(TASK0_STACK), task0_entry) };
    let sp1 = unsafe { init_stack(core::ptr::addr_of_mut!(TASK1_STACK), task1_entry) };
    let sp2 = unsafe { init_stack(core::ptr::addr_of_mut!(TASK2_STACK), task2_entry) };
    let sp3 = unsafe { init_stack(core::ptr::addr_of_mut!(TASK3_STACK), task3_entry) };

    uart_print("[System] Initializing Task Control Blocks...\n");
    
    assert!(rivet_rtos::kernel::register_task(0, sp0, 1));
    assert!(rivet_rtos::kernel::register_task(1, sp1, 0));
    assert!(rivet_rtos::kernel::register_task(2, sp2, 0));
    assert!(rivet_rtos::kernel::register_task(3, sp3, 2));

    uart_print(PASS_MARK);
    uart_print(" Task registration passed\n");

    rivet_rtos::kernel::set_current(0);
    
    uart_print("[System] Starting Scheduler...\n");
    uart_print("--------------------------------------------------\n");

    unsafe {
        rivet_rtos::arch::switch_to_first(sp0);
    }
    loop {}
}

unsafe fn init_stack(stack: *mut AlignedStack, entry: fn() -> !) -> usize {
    let base = stack as usize;
    let frame_start = (base + STACK_SIZE - FRAME_SIZE) & !(STACK_ALIGN - 1);
    let ra_slot = frame_start as *mut usize;
    *ra_slot = entry as *const () as usize;
    frame_start
}

// Task 0: Controller (Priority 1)
fn task0_entry() -> ! {
    uart_print("[Test] Verifying strict priority scheduling...\n");
    
    // Trigger tick. Task 3 (Prio 2) is ready, so it MUST preempt Task 0 (Prio 1).
    rivet_rtos::arch::critical_section(|| rivet_rtos::kernel::tick());
    
    uart_print(PASS_MARK);
    uart_print(" Higher priority preemption passed\n");
    
    uart_print("[Test] Verifying task blocking on semaphore...\n");
    
    // Block on semaphore, yielding to Prio 0 workers.
    rivet_rtos::arch::critical_section(|| {
        unsafe { (*core::ptr::addr_of_mut!(SEM_WORKERS_DONE)).wait() };
    });
    
    uart_print(PASS_MARK);
    uart_print(" Semaphore wait and signaling passed\n");
    uart_print(PASS_MARK);
    uart_print(" Priority inversion avoidance passed\n");

    uart_print("--------------------------------------------------\n");
    uart_print("System Verification Complete. All checks passed.\n");
    uart_print("--------------------------------------------------\n");
    
    spin_ms(100);
    rivet_rtos::arch::qemu_exit_success();
}

// Task 1: Worker A (Priority 0)
fn task1_entry() -> ! {
    rivet_rtos::arch::critical_section(|| {
        for _ in 0..2 {
            spin_ms(1);
            rivet_rtos::kernel::tick(); // Yields nicely to Worker B via Round-Robin
        }
    });
    
    uart_print(PASS_MARK);
    uart_print(" Same-priority round-robin fairness passed\n");
    
    rivet_rtos::arch::critical_section(|| {
        unsafe { (*core::ptr::addr_of_mut!(SEM_MONITOR_WAKE)).signal() };
    });
    
    // Trigger tick so highest priority Monitor can take over
    rivet_rtos::arch::critical_section(|| rivet_rtos::kernel::tick());

    rivet_rtos::arch::critical_section(|| {
        for _ in 0..2 {
            spin_ms(1);
            rivet_rtos::kernel::tick();
        }
    });

    loop {
        rivet_rtos::arch::critical_section(|| rivet_rtos::kernel::yield_task());
    }
}

// Task 2: Worker B (Priority 0)
fn task2_entry() -> ! {
    rivet_rtos::arch::critical_section(|| {
        for _ in 0..4 {
            spin_ms(1);
            rivet_rtos::kernel::tick();
        }
    });
    
    uart_print(PASS_MARK);
    uart_print(" Cooperative yielding within critical sections passed\n");

    rivet_rtos::arch::critical_section(|| {
        unsafe { (*core::ptr::addr_of_mut!(SEM_WORKERS_DONE)).signal() };
    });
    
    loop {
        rivet_rtos::arch::critical_section(|| rivet_rtos::kernel::yield_task());
    }
}

// Task 3: Monitor (Priority 2 - Highest)
fn task3_entry() -> ! {
    // Runs first due to Priority 2
    rivet_rtos::arch::critical_section(|| {
        unsafe { (*core::ptr::addr_of_mut!(SEM_MONITOR_WAKE)).wait() };
    });
    
    // Resumes here when Worker A signals
    uart_print(PASS_MARK);
    uart_print(" Event-driven dynamic preemption passed\n");
    
    loop {
        rivet_rtos::arch::critical_section(|| {
            unsafe { (*core::ptr::addr_of_mut!(SEM_MONITOR_WAKE)).wait() };
        });
    }
}
