//! Rivet RTOS — zero-allocation, dual-tier RTOS for microcontrollers.
//!
//! # Architecture
//!
//! Two tiers of concurrency, unified under one priority scheduler:
//!
//! - **Preemptive tier** (`#[rivet::ptask]` / [`spawn_ptask!`]): each task
//!   gets its own stack. The timer tick can suspend a running task at *any*
//!   point (not just at a yield/await) and resume a higher-priority one
//!   instead — real priority preemption, with priority inheritance on
//!   [`preempt::PriorityMutex`] to avoid priority inversion.
//! - **Cooperative tier** (`#[rivet::task]`): `async fn` tasks compiled to
//!   `Future` state machines, polled on a single shared stack — zero
//!   per-task stack cost, ideal for I/O-bound/event-driven logic. This
//!   tier runs as an ordinary preemptive task at the lowest priority, so
//!   any real preemptive task immediately preempts it; it fills otherwise
//!   idle CPU time and only calls `WFI` when nothing anywhere is ready.
//!
//! # Targets
//!
//! - **ARM Cortex-M** (M0+/M3/M4/M7/M33) — via the `arch-cortex-m` feature
//! - **RISC-V** (RV32) — via the `arch-riscv` feature
//!
//! # Example
//!
//! ```ignore
//! use rivet::sync::Semaphore;
//!
//! static SEM: Semaphore<1> = Semaphore::new(0);
//!
//! // Cooperative: fine for I/O-bound logic.
//! #[rivet::task(priority = 0)]
//! async fn background() {
//!     loop {
//!         SEM.acquire().await;
//!     }
//! }
//!
//! // Preemptive: genuinely can't be starved by a lower/equal priority
//! // task that never yields.
//! static CFG: u32 = 42;
//! fn critical_task(cfg: &'static u32) -> ! {
//!     loop {
//!         // real work, no .await required anywhere
//!     }
//! }
//!
//! fn main() -> ! {
//!     rivet::init();
//!     rivet::spawn_ptask!(stack = 2048, priority = 5, entry = critical_task, arg = CFG);
//!     rivet::run();
//! }
//! ```

#![no_std]
#![forbid(clippy::undocumented_unsafe_blocks)]

// Test-support (feature = "test-support") uses `std::sync::Mutex` to
// serialize host tests that share kernel globals. The feature is only ever
// enabled from this crate's own `[dev-dependencies]`, so `std` is only
// linked into test builds, never embedded builds.
#[cfg(feature = "test-support")]
extern crate std;

pub mod config;
pub mod console;
pub mod critical;
pub mod deadlines;
pub mod exec_time;
pub mod executor;
pub mod fault;
pub mod irq;
pub mod latency;
pub mod log;
pub mod port;
pub mod preempt;
pub mod report;
/// Dump kernel state to the console. See the [`report`] module docs for
/// exactly what's included and what's deliberately left out.
pub use report::report;
pub mod sync;
pub mod task;
pub mod time;
pub mod timer;
pub mod trace;
pub mod waker;
pub mod watchdog;

#[cfg(feature = "test-support")]
pub mod test_support;

/// Declare a static async (cooperative-tier) task. See the [`task`] module
/// docs and the crate-level example. Lives in the macro namespace, so it
/// coexists with the `task` module (`rivet::task::TaskCell` etc.) at the
/// same path.
pub use rivet_macros::task;

/// Declare the application entry point. See [`rivet_macros::main`] for the
/// full docs and an example.
pub use rivet_macros::main;

/// Serialize + reset a host test. Every test that touches kernel globals
/// (task registry, waker bitmaps, timer slots, scheduler state) must be
/// wrapped in this macro: `cargo test` runs test fns on parallel threads
/// and the shared statics otherwise race (observed flake: 1/8 runs of
/// `cargo test -p rivet`).
///
/// ```ignore
/// #[test]
/// fn my_test() {
///     rivet::kernel_test! {
///         // ... test body ...
///     }
/// }
/// ```
#[cfg(feature = "test-support")]
#[macro_export]
macro_rules! kernel_test {
    ($($body:tt)*) => {{
        let __rivet_test_guard = $crate::test_support::acquire();
        $crate::test_support::reset_all();
        $($body)*
    }};
}

/// Crate version.
pub const VERSION: &str = env!("CARGO_PKG_VERSION");

/// Stack reserved for the cooperative-tier task (the async executor,
/// spawned automatically at priority 0 by [`init`]).
///
/// Aligned to its own size (unlike [`preempt::Stack`]'s general 16-byte
/// alignment, which is enough for a context-switch frame but not for
/// this): this is the one task stack in the kernel that bypasses the
/// pool's own size-aligned carving (`stack_pool::alloc_stack`) — spawned
/// directly from a fixed `'static` buffer in [`init`] — yet still gets
/// handed to `port::arch::on_switch_to`, which on Cortex-M reprograms an
/// MPU region sized to it. An MPU region's base must be aligned to its
/// own size; a plain `.bss` array has no such guarantee (found via a
/// board with a different `.bss` layout than the one this was first
/// written against — same class of bug the pool's alignment math exists
/// to prevent, just missed for this one non-pool stack).
const ASYNC_IDLE_STACK_SIZE: usize = 4096;
#[repr(align(4096))]
struct AlignedIdleStack([u8; ASYNC_IDLE_STACK_SIZE]);
static mut ASYNC_IDLE_STACK: AlignedIdleStack = AlignedIdleStack([0; ASYNC_IDLE_STACK_SIZE]);
static ASYNC_IDLE_ARG: () = ();

fn async_idle_entry(_arg: &'static ()) -> ! {
    // Safety: EXECUTOR.init() was called in `init()`, before this task can
    // possibly run (the preemptive scheduler doesn't start until `run()`).
    unsafe {
        core::ptr::addr_of!(executor::EXECUTOR)
            .as_ref()
            .unwrap()
            .run();
    }
}

/// Initialize the kernel: set up the arch layer, discover `#[rivet::task]`
/// (cooperative) tasks, and spawn the async executor as the lowest-priority
/// preemptive task. Call [`spawn_ptask!`] for any additional preemptive
/// tasks after this, then call [`run`].
pub fn init() {
    port::arch::init();
    // Before `port::board::init()`: a board's init may call
    // `console::enable_irq_tx()` (plan.md Phase 14), which needs the TX/
    // RX rings already split. Calling it earlier than that would be
    // silently harmless today (`write_bytes_irq` falls back to polling
    // if `TX_SENDER` isn't set yet) but fragile to rely on.
    console::init();
    port::board::init();
    port::board::tick_start(config::TICK_HZ);
    log::init();
    // Safety: EXECUTOR is only accessed here at boot, before run().
    unsafe {
        core::ptr::addr_of_mut!(executor::EXECUTOR)
            .as_mut()
            .unwrap()
            .init();
    }
    // Safety: ASYNC_IDLE_STACK and ASYNC_IDLE_ARG are static, `'static`
    // data never aliased anywhere else; the executor task is spawned
    // exactly once here, before `run()` starts the scheduler, so no other
    // task can observe the half-initialized registration.
    unsafe {
        #[allow(static_mut_refs)]
        let _ = preempt::spawn(
            &mut ASYNC_IDLE_STACK.0,
            0, // lowest priority: any real preemptive task preempts this
            async_idle_entry,
            &ASYNC_IDLE_ARG,
        );
    }
}

/// Start the preemptive scheduler. Never returns.
/// Must be called after [`init`] (and any [`spawn_ptask!`] calls).
/// Set by [`run`], just before entering the scheduler (plan.md Phase 19):
/// the signal secondary harts spin on before bringing up their own arch
/// state and calling [`run_secondary_hart`]. `init()` plus every
/// boot-time [`spawn_ptask!`] call is guaranteed complete by the time this
/// is set, since both happen (by construction, in every binary this
/// workspace builds) before the app calls `run()`.
static KERNEL_READY: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(false);

/// Whether hart 0 has finished boot and is running the scheduler (plan.md
/// Phase 19). `rivet-rt`'s secondary-hart bring-up spins on this before
/// calling [`run_secondary_hart`]. Always `false` (and irrelevant) on a
/// single-hart board — nothing there ever calls the secondary-hart path.
pub fn kernel_ready() -> bool {
    KERNEL_READY.load(core::sync::atomic::Ordering::Acquire)
}

pub fn run() -> ! {
    KERNEL_READY.store(true, core::sync::atomic::Ordering::Release);
    preempt::start();
}

/// Bring up the preemptive scheduler on a **secondary** hart (plan.md
/// Phase 19, RISC-V `virt` under `-smp > 1` only): per-hart arch bring-up
/// (trap vector, ISR stack slice, PMP catch-all — everything
/// [`port::arch::init`] does, all genuinely per-hart CSR/register state)
/// followed by [`preempt::start_secondary_hart`]. Deliberately does
/// **not** repeat [`init`]'s other steps (console/board/log setup, the
/// async-idle-task spawn) — those are global, one-time facts owned by
/// hart 0. Call only after [`kernel_ready`] is true, from a hart other
/// than the one that called [`run`]. Never returns.
pub fn run_secondary_hart() -> ! {
    port::arch::init();
    preempt::start_secondary_hart();
}

/// Voluntarily give up the CPU: request an immediate reschedule
/// opportunity, same as a mutex unlock waking a higher-priority waiter.
/// Safe to call from task or ISR context.
pub fn yield_now() {
    port::arch::request_reschedule();
}

/// Terminate successfully. Never returns. Under QEMU this reduces to the
/// board's exit device / semihosting path (the `xtask` test harness
/// asserts on the resulting exit code); on real hardware, boards typically
/// map this to a reset or halt.
pub fn exit_success() -> ! {
    port::board::exit(0)
}

/// Terminate with a distinguishable non-zero failure code. Never returns.
pub fn exit_failure(code: u32) -> ! {
    port::board::exit(code)
}

/// Trigger a system reset (watchdog / fault-policy recovery). Never
/// returns.
pub fn system_reset() -> ! {
    port::board::reset()
}
