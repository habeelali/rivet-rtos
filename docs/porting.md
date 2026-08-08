# Porting Rivet to a new board

Rivet is split into four layers (see `plan.md` for the full rationale):

```
  app / examples          uses rivet + one rivet-bsp-* ; no MMIO, no boot code
        |
  rivet-rt                _start / Reset, vectors, bss+data, hart park, panic,
        |                 #[rivet::main] ; links the BSP-supplied linker script
  rivet-bsp-<board>       memory map, clocks, console, tick source, reset/exit,
        |                 HW watchdog, GPIO/SPI/I2C drivers, link-<board>.ld
  rivet-arch-<isa>        context switch, trap entry, critical section, MPU/PMP,
        |                 optional default tick+console drivers (feature-gated)
  rivet                   kernel. Scheduler, TCB, executor, timers, sync, fault
                          policy. Zero MMIO. Zero #[cfg(target_arch)].
```

Bringing up a new board means writing a `rivet-bsp-*` crate. You will not touch
`rivet`, `rivet-arch-*`, or `rivet-rt` unless you hit a genuine kernel bug (see
"What actually happened when we did this" at the end — it happens, and that's
fine, it's a kernel fix then, not a board workaround).

This guide was written *from* adding `rivet-bsp-mps2-an385` as the project's
third board — every step here is something that was actually done, not
theorized.

## Step 0: check your target has what the kernel needs

Before writing anything, confirm your target actually has hardware atomic
read-modify-write instructions (`AtomicUsize::fetch_add`, `compare_exchange`,
etc.). The kernel's scheduler, waker bitmaps, and timer queue are lock-free and
use these throughout. Concretely:

```bash
cargo check -p rivet --target <your-target-triple>
```

If this fails with `no method named 'fetch_or' found for struct 'Atomic<T>'`
(or similar), your target's ISA doesn't have the atomic extension the
`core::sync::atomic` RMW operations need (e.g. `riscv32imc-unknown-none-elf` —
RV32IMC, no `A` extension — fails this exact way; `riscv32imac-unknown-none-elf`,
with the `A` extension, passes). **This is not fixable from a BSP crate** — it
would require the kernel to switch to `portable-atomic`-style software-emulated
atomics (a critical-section-based RMW fallback), which is a real, cross-cutting
kernel change, not a porting exercise. Confirmed by trying exactly this for an
ESP32-C3 BSP during this project's Phase 7: `rivet` fails to compile for
`riscv32imc-unknown-none-elf` with 36 missing-atomic-method errors. Tracked as
a known limitation, not attempted further here.

## Step 1: pick (or write) an arch crate

If your board's CPU is already covered by `rivet-arch-riscv` (any RV32 core
with the `A` extension) or `rivet-arch-cortex-m` (any Cortex-M3/4/7/33 with an
MPU), you don't write a new arch crate — skip to Step 2.

Writing a new arch crate is out of scope for this guide (it means hand-writing
a context-switch trap/exception entry in assembly, and is a much larger task);
see `rivet-arch-riscv/src/lib.rs` / `rivet-arch-cortex-m/src/lib.rs` for what
one looks like.

## Step 2: probe your board's memory map in QEMU (or read the datasheet)

Don't guess addresses. If you're targeting QEMU, use its monitor:

```bash
qemu-system-arm -M <your-machine> -monitor stdio -serial none -display none -S
(qemu) info mtree
```

This dumps every memory region and peripheral QEMU actually modeled, with
addresses — exactly what `rivet-bsp-mps2-an385/link-mps2-an385.ld`'s header
comment records for MPS2 AN385. For real hardware, use the datasheet's memory
map table instead. Either way, **write down the source** (the `info mtree`
dump, or the datasheet section) in your linker script / BSP crate's doc
comments — the next person porting a board needs to trust these numbers
without re-deriving them.

You need, at minimum: a boot/flash region, a RAM region, your console UART's
base address and register layout, and (if you want a real watchdog) the
watchdog's base and register layout. Verify peripheral register layouts the
same way if you're not sure — QEMU's device model source is authoritative;
when in doubt, write the simplest possible driver and check for `guest_errors`
in `-d guest_errors -D qemu.log` output, or just try sending a byte and see if
anything comes out on the serial console.

## Step 3: the linker script

Copy `rivet-bsp-lm3s6965/link-lm3s6965.ld` or
`rivet-bsp-mps2-an385/link-mps2-an385.ld` and adjust the `MEMORY` block and
vector-table entries to your board. The linker contract every board's script
must satisfy (these exact symbol/section names, referenced by the kernel and
arch crate via `extern "C"`):

| Symbol / section | Required by | Constraint |
|---|---|---|
| `.rivet_tasks`, `__rivet_tasks_{start,end}` | `rivet/src/task.rs` | `KEEP`; may be flash |
| `.task_stacks`, `__task_stacks_{start,end}` | `rivet/src/preempt/stack_pool.rs` | RAM, `NOLOAD`, **power-of-two size and alignment** (`ALIGN(16384)` in the reference boards) |
| `.isr_stack`, `__isr_stack_{bottom,top}` | `rivet-arch-riscv` only | RV32 only, 16-byte aligned |
| `.stack`, `__stack_{bottom,top}` | `rivet-rt` (RISC-V `_start`) | boot stack |
| `__bss_{start,end}`, `__data_{start,end}`, `__data_load` | `rivet-rt` (`Reset`, Cortex-M) | `.data`'s `AT >` load address must be set for XIP-flash boards |
| `.vector_table` (Cortex-M only) | `rivet-rt` | at `ORIGIN(FLASH)`; `LONG(Reset)`, `LONG(NMI)`, ..., `LONG(PendSV)`, `LONG(SysTick)` — `PendSV`/`MemManage`/`rivet_svc_handler` are provided by `rivet-arch-cortex-m` (strong symbols, resolved automatically once it's linked in); the rest fall back to `rivet-rt`'s `DefaultHandler` via `PROVIDE(...)` unless your board overrides one |
| `ENTRY(_start)` (RISC-V) / `ENTRY(Reset)` (Cortex-M) | linker | |

## Step 4: the BSP crate

```toml
[package]
name = "rivet-bsp-yourboard"
# ...
# Required: lets your build.rs (Step 5) publish the linker script path.
links = "rivet_bsp_yourboard"

[dependencies]
rivet = { path = "../rivet" }
rivet-arch-cortex-m = { path = "../rivet-arch-cortex-m", features = ["systick"] }
# or: rivet-arch-riscv = { path = "../rivet-arch-riscv", features = ["clint"] }
#     (only if your RISC-V board actually has a CLINT — see Step 4a)
```

Implement every symbol in `rivet::port::board` (Group B) as a
`#[no_mangle] extern "Rust" fn`:

```rust
#[no_mangle]
extern "Rust" fn __rivet_board_init() { /* clock/PLL bring-up, console init */ }
#[no_mangle]
extern "Rust" fn __rivet_board_now_us() -> u64 { /* monotonic, tear-free */ }
#[no_mangle]
extern "Rust" fn __rivet_board_tick_start(hz: u32) { /* arm the periodic tick */ }
#[no_mangle]
unsafe extern "Rust" fn __rivet_board_console_write(ptr: *const u8, len: usize) { /* UART */ }
#[no_mangle]
extern "Rust" fn __rivet_board_reset() -> ! { /* never returns */ }
#[no_mangle]
extern "Rust" fn __rivet_board_exit(code: u32) -> ! { /* 0 = success; never returns */ }
#[no_mangle]
extern "Rust" fn __rivet_board_wdt_init(period_us: u32) { /* 0 = disabled */ }
#[no_mangle]
extern "Rust" fn __rivet_board_wdt_feed() { /* re-arm */ }
#[no_mangle]
extern "Rust" fn __rivet_board_wdt_check() { /* no-op if you have real HW WDT */ }
```

The exact signatures (and why each exists) are documented in
`rivet/src/port/board.rs` — read it before implementing; it's the actual
contract, this table is a summary.

For a Cortex-M board, also provide the `SysTick` exception vector (referenced
directly by your linker script, not part of the port contract — it's an
exception vector, not a Group A/B symbol):

```rust
#[no_mangle]
pub unsafe extern "C" fn SysTick() {
    rivet_arch_cortex_m::systick::handler();
}
```

### Step 4a: the "Group C" case — does your RISC-V board have a CLINT?

If yes (most RV32 "virt"-like platforms do, at a near-universal `0x0200_0000`
base, though the clock rate varies): enable `rivet-arch-riscv`'s `clint`
feature, call `rivet_arch_riscv::clint::configure(base, mtime_hz)` from your
`__rivet_board_init`, and implement `__rivet_board_now_us`/`__rivet_board_tick_start`
as one-line passthroughs to `rivet_arch_riscv::clint::{now_micros, tick_start}`
(see `rivet-bsp-qemu-virt/src/lib.rs`).

If no (e.g. an ESP32-C3's SYSTIMER): don't enable `clint`; implement
`__rivet_board_tick_start`/`__rivet_board_now_us` against your own timer
peripheral, and additionally supply your own
`#[no_mangle] extern "Rust" fn __rivet_arch_request_reschedule()` — the
symbol-contract mechanism doesn't care which crate defines a Group A symbol,
only that exactly one does, so your BSP is allowed to override this one when
the arch crate's stock backend doesn't fit your platform.

### Step 4b: watchdog, if you have no real hardware one

Use `rivet-bsp-support::sw_watchdog` rather than reimplementing a software
watchdog (see `rivet-bsp-qemu-virt/src/lib.rs` for the ~10-line usage). Be
honest in your BSP's doc comment that this is not independent of the CPU: a
tick-driven check cannot catch a hang that stops ticks (interrupts disabled in
a spin loop). Real hardware watchdog independence is only real if you have
actual watchdog hardware — implement `__rivet_board_wdt_check` as a no-op in
that case (the hardware resets on its own).

## Step 5: the linker-script build.rs

Cargo's `cargo:rustc-link-arg` is only honored when emitted by the **final
binary's own** build script — it does not propagate up from a library
dependency's build script. So your BSP crate's `build.rs` publishes the script
path via `links` + `cargo:KEY=VALUE`:

```rust
// rivet-bsp-yourboard/build.rs
fn main() {
    let manifest_dir = std::path::PathBuf::from(std::env::var("CARGO_MANIFEST_DIR").unwrap());
    let script = manifest_dir.join("link-yourboard.ld");
    println!("cargo:linker-script={}", script.display());
    println!("cargo:rerun-if-changed={}", script.display());
}
```

...and every binary crate that uses your board needs this ~5-line snippet
(the one piece of per-binary wiring this mechanism requires):

```rust
// your-app/build.rs — change only the env var name to match your BSP's `links` key
fn main() {
    let script = std::env::var("DEP_RIVET_BSP_YOURBOARD_LINKER_SCRIPT")
        .expect("rivet-bsp-yourboard must be a dependency (missing `links` metadata)");
    println!("cargo:rustc-link-arg-bins=-T{script}");
    println!("cargo:rerun-if-changed={script}");
}
```

(The `DEP_<NAME>_<KEY>` variable name is derived from your BSP's `links`
value, uppercased with non-alphanumerics turned into `_`.)

## Step 6: write your binary

```rust
#![no_std]
#![no_main]

use rivet_bsp_yourboard as _; // links the BSP + arch + linker script
use rivet_rt as _;            // links boot glue + default panic handler

#[rivet::main]
fn main() -> ! {
    rivet::println!("hello from your board");
    rivet::spawn_ptask!(stack = 512, priority = 1, entry = my_task, arg = ());
    rivet::run()
}
```

Add your board to `xtask`'s registry (`xtask/src/main.rs`'s `BOARDS` const)
and a `smoke_tests` match arm, and you can run the full QEMU test harness
against it: `cargo xtask test --target yourboard --suite smoke`.

## What actually happened when we did this (MPS2 AN385, Phase 7)

Two real things came out of adding a third board that are worth knowing before
you add a fourth:

1. **A genuine, pre-existing kernel bug surfaced.** `rivet::init()`'s
   cooperative-executor stack (`ASYNC_IDLE_STACK`) was spawned directly from a
   fixed `'static` buffer rather than through the stack pool's own
   size-aligned carving, and had no alignment guarantee beyond 16 bytes — but
   it's still handed to `port::arch::on_switch_to`, which on Cortex-M
   reprograms an MPU region sized to it, and an MPU region's base must be
   aligned to its own size. `lm3s6965evb`'s specific `.bss` layout happened to
   place it on a 4096-byte boundary by luck; MPS2's different set of statics
   didn't, and QEMU logged `DRBAR[7]: ... misaligned to DRSR region size`.
   **This was a one-line fix in `rivet/src/lib.rs`** (`#[repr(align(4096))]`
   on the static) — a kernel fix, correctly made in the kernel, not a
   board-specific workaround. It's expected that testing against a genuinely
   different board finds bugs like this; that's the point of doing it.
2. **A second, smaller oddity was found and not fully root-caused**: an
   intermittent "M profile return from interrupt with misaligned PC" QEMU
   warning, plus an occasional related `DRBAR` line, that doesn't affect
   correctness (every test still passes) but wasn't tracked down to an exact
   cause before time ran out on this pass. See `tests/golden/KNOWN_FAILURES.md`
   for the full account — recorded honestly rather than silently added to an
   ignore-list without explanation.

Net result: the git diff for adding this board touched `rivet-bsp-mps2-an385/`
(new), `examples/mps2-an385/` (new), `xtask/src/main.rs` (new registry entry +
test cases), and **15 lines of `rivet/src/lib.rs`** for the bug fix above.
`rivet-arch-cortex-m/` and `rivet-rt/` were untouched, confirming the
arch/board split itself is in the right place — the one kernel change needed
was a real bug fix that happened to be found this way, not evidence the
boundary is wrong.
