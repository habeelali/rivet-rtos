//! Rivet RTOS boot glue: `_start`/`Reset`, bss/data initialization, default
//! exception handlers, and a default panic handler — everything that was
//! previously copy-pasted into every example binary's `main.rs`.
//!
//! Link this crate (`use rivet_rt as _;`) alongside one `rivet-arch-*` and
//! one `rivet-bsp-*` crate, declare your entry point with
//! [`rivet::main`](../rivet_macros/attr.main.html), and that's a complete
//! binary:
//!
//! ```ignore
//! #![no_std]
//! #![no_main]
//!
//! use rivet_bsp_qemu_virt as _;
//! use rivet_rt as _;
//!
//! #[rivet::main]
//! fn main() -> ! {
//!     rivet::println!("hello");
//!     rivet::run()
//! }
//! ```
//!
//! Unlike `rivet`/`rivet-arch-*`/`rivet-bsp-*`, this crate legitimately
//! knows the target architecture (`#[cfg(target_arch = ...)]`) — it is
//! boot glue for a *known* target, the same role `cortex-m-rt`/`riscv-rt`
//! play in the wider embedded-Rust ecosystem, not kernel or board logic.
//! It reaches no MMIO of its own beyond what every binary needs to boot at
//! all (stack pointer, bss/data, and — on Cortex-M — the small set of
//! exception vectors every board needs *something* installed at).

#![no_std]

extern "C" {
    // Referenced by symbol name from the RISC-V global_asm! `_start` (not
    // a Rust-level call there, so rustc's dead-code analysis doesn't see
    // it as used on that target) and by direct call from Cortex-M's
    // `Reset` below.
    #[allow(dead_code)]
    fn rivet_main() -> !;
}

#[cfg(target_arch = "riscv32")]
mod riscv {
    /// The compile-time-configured hart ceiling, exposed as a real linked
    /// symbol so `_start`'s `global_asm!` (which can only reference
    /// linker symbols, not Rust consts) can compare `mhartid` against it
    /// (plan.md Phase 19). `1` on every board that hasn't opted into
    /// `RIVET_MAX_HARTS > 1` — harts `1..` then immediately park, exactly
    /// the pre-Phase-19 behavior.
    #[no_mangle]
    static __rivet_max_harts: u32 = rivet::config::MAX_HARTS as u32;

    // `mhartid` guard: harts other than 0 must never touch shared kernel
    // statics (bss zeroing, `rivet_main`'s `rivet::init()`) — those are
    // hart-0-only, one-time facts. Before plan.md Phase 19 the guard
    // simply parked every other hart forever (`-smp N > 1` ran N kernel
    // copies otherwise, over one set of kernel statics — Rivet's
    // multi-core story was AMP-or-nothing). Phase 19 gives harts
    // `1..RIVET_MAX_HARTS` a real bring-up path instead: each gets its
    // own boot stack (`__hart_n_stack_top`, sized per hart in the linker
    // script) and spins on `rivet::kernel_ready()` before calling
    // `rivet_secondary_main`, which does per-hart arch init and enters
    // the scheduler. Harts `>= RIVET_MAX_HARTS` (a build might run under
    // `-smp` higher than it was configured for) still park forever —
    // there is no kernel state sized for them.
    //
    // No `.data` copy needed: this is a single-RAM-region target (no
    // separate flash load address), so `.data`'s initial contents are
    // already at their run-time VMA in the loaded image.
    core::arch::global_asm!(
        ".section .text._start",
        ".global _start",
        "_start:",
        "  csrr t0, mhartid",
        "  bnez t0, secondary_entry",
        "  la   sp, __stack_top",
        "  la   t0, __bss_start",
        "  la   t1, __bss_end",
        "1:",
        "  bgeu t0, t1, 2f",
        "  sw   zero, 0(t0)",
        "  addi t0, t0, 4",
        "  j    1b",
        "2:",
        "  call rivet_main",
        // rivet_main is `-> !`; this is unreachable in practice, kept only
        // so a hypothetical return doesn't fall off the end of .text.
        "  j    park_hart",
        // A secondary hart (mhartid != 0): if it's within the configured
        // hart ceiling, give it its own boot stack (one 1K slice per hart,
        // `.secondary_stacks`, indexed by mhartid — never shared with hart
        // 0's `__stack_top` or with each other) and hand off to Rust,
        // which spins on `rivet::kernel_ready()` before touching any
        // kernel state. Out-of-range harts park immediately, same as
        // before Phase 19.
        "secondary_entry:",
        "  la   t1, __rivet_max_harts",
        "  lw   t1, 0(t1)",
        "  bgeu t0, t1, park_hart",
        "  la   t2, __secondary_stacks_top",
        "  slli t3, t0, 9", // t3 <- mhartid * 512 (power-of-two shift,
                            // matches link-qemu-virt.ld's per-hart
                            // .secondary_stacks slice size; avoids
                            // needing the M extension)
        "  sub  sp, t2, t3",
        "  call rivet_secondary_main",
        "  j    park_hart",
        "park_hart:",
        "  wfi",
        "  j    park_hart",
    );

    /// Hart bring-up on a secondary hart, called from `_start`'s asm once
    /// it has its own boot stack: spin for `rivet::kernel_ready()`
    /// (hart 0's signal that `rivet::init()` and every boot-time
    /// `spawn_ptask!` have completed), then hand off to
    /// `rivet::run_secondary_hart()` (per-hart arch bring-up — trap
    /// vector, ISR stack slice, PMP — followed by the scheduler). Never
    /// returns.
    ///
    /// # Safety
    /// Must only be reached from `_start`'s asm, on a hart whose id is
    /// `< RIVET_MAX_HARTS` (the asm already checked this), with that
    /// hart's own boot stack already installed as `sp`.
    #[no_mangle]
    unsafe extern "C" fn rivet_secondary_main() -> ! {
        while !rivet::kernel_ready() {
            core::hint::spin_loop();
        }
        rivet::run_secondary_hart();
    }
}

#[cfg(target_arch = "arm")]
mod cortex_m {
    extern "C" {
        static __data_load: u8;
        static __data_start: u8;
        static __data_end: u8;
        static __bss_start: u8;
        static __bss_end: u8;
    }

    /// # Safety
    /// Runs at power-on reset as the vector-table Reset entry; performs
    /// the `.data` copy and `.bss` zeroing, then starts the kernel.
    #[no_mangle]
    pub unsafe extern "C" fn Reset() -> ! {
        // SAFETY: `__data_*`/`__bss_*` are the board linker script's
        // (via rivet-rt's common linker fragment) data/bss bounds; this
        // runs once, before any other code, with nothing else touching
        // that memory yet.
        unsafe {
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

            super::rivet_main()
        }
    }

    /// Shared fallback for exception vectors no board/test overrides:
    /// prints a marker and halts. A binary that wants to observe a
    /// specific fault (e.g. the fault-isolation test suite) defines its
    /// own `#[no_mangle] extern "C" fn HardFault()` etc., which — being a
    /// strong symbol — takes priority over this crate's; the linker
    /// script only falls back to `DefaultHandler` via `PROVIDE` for
    /// vectors nothing else defines.
    #[no_mangle]
    pub extern "C" fn DefaultHandler() {
        rivet::console::write_str("HARD_FAULT\n");
        // This handler runs at HardFault's fixed, always-highest
        // exception priority and then spins forever without returning —
        // no lower-priority interrupt (including a board's
        // interrupt-driven console TX ISR, plan.md Phase 14) can ever
        // preempt it, so a message queued into that ring here would
        // never drain on its own.
        rivet::console::flush_sync();
        loop {
            core::hint::spin_loop();
        }
    }
}

/// Default panic handler: prints the location and message via
/// [`rivet::console`], then exits with a distinguishable failure code.
/// Disable with `default-features = false` to supply your own.
#[cfg(feature = "panic-handler")]
mod panic {
    use core::fmt::Write;
    use core::panic::PanicInfo;

    struct ConsoleWriter;
    impl Write for ConsoleWriter {
        fn write_str(&mut self, s: &str) -> core::fmt::Result {
            rivet::console::write_str(s);
            Ok(())
        }
    }

    #[panic_handler]
    fn panic(info: &PanicInfo) -> ! {
        rivet::console::write_str("PANIC: ");
        if let Some(loc) = info.location() {
            let _ = write!(ConsoleWriter, "{}:{}", loc.file(), loc.line());
        } else {
            rivet::console::write_str("(no location)");
        }
        rivet::console::write_str(": ");
        let _ = write!(ConsoleWriter, "{}", info.message());
        rivet::console::write_str("\n");
        rivet::exit_failure(0xFF);
    }
}
