## Update (Phase 2 of the layering refactor): all three now pass

After the RTOS/board separation (Phase 2, "extract the port contract"),
all three tests below started passing reliably (verified 3/3 consecutive
runs each: `fault_overflow`, `fault_isolate`, `stress_max_ptasks`, plus the
full `cm3` smoke suite: 10/10). This was **not a deliberate fix** — no code
in this pass targeted fault handling or MPU/stack-pool logic — and the
root cause was not chased down further (would require diffing generated
code/addresses between the old and new binaries, disproportionate to the
layering task). The leading hypothesis: a genuine interrupt-enable/disable
bug was found and fixed in `rivet-arch-cortex-m`'s new
`__rivet_arch_irq_save`/`__rivet_arch_irq_restore` (a `Primask::is_active()`
polarity inversion that caused critical sections to *toggle* global
interrupts rather than correctly restore them, parity-dependent on how
many critical sections had run) — critical-section timing plausibly
affected these tests' exact fault/stack timing enough to dodge whatever
condition previously caused the double-fault escalation described below.
Left as a pleasant surprise, not a claimed fix; the analysis below is kept
for the record in case the failure mode resurfaces on a different board or
kernel change.

# Known pre-existing failures (baseline, before layering refactor)

Captured 2026-08-08, before any layering changes. These are pre-existing bugs
unrelated to the RTOS/BSP layering work and are explicitly out of scope for
that refactor. They are recorded here so later phases can confirm they fail
**identically** (not worse, not differently) rather than being silently
papered over.

## cm3/fault_overflow

`examples/qemu-cm3/src/bin/fault_overflow.rs` overflows a 512-byte task stack
with a 2048-byte local array. The write drives PSP ~1536 bytes into the MPU
deny region (region 6, the whole `.task_stacks` pool minus the current
task's region 7). On Cortex-M, exception entry auto-stacks {r0-r3, r12, lr,
pc, xPSR} onto whatever stack was active pre-exception (PSP, since the
kernel runs tasks in Thread mode on PSP) — and that auto-stack push lands at
the same (already-denied) SP, so the MemManage fault's own entry stacking is
*also* denied. Per ARMv7-M, a fault that recurs during a fault handler's own
exception-entry sequence escalates to HardFault rather than delivering a
clean, diagnosable MemManage.

Observed: `Rivet CM3 fault_overflow\nabout to overflow a stack\nHARD_FAULT\n`
then the CM3 `HardFault` handler spins forever -> 30s xtask timeout.
Expected (never delivered): `RIVET FAULT ... memmanage ... RIVET_FAILURE code=250`.

This is a real gap in the two-region MPU design (old plan.md §3.1): it can
cleanly catch a *shallow* overflow (a few bytes past the guard) but not a
*deep* one that also swallows the fault entry's own stacking. Fixing it
would need either a dedicated Handler-mode-only stacking path (impossible on
armv7-M for the initial hardware frame) or a much larger guard band plus
stack-limit pre-checking. Left unfixed here; tracked for a future
fault-isolation hardening pass, not the layering overhaul.

Status after each phase of the layering refactor: **must still fail exactly
this way** (same three lines, same HardFault, same timeout). If it starts
passing or fails differently, that is a real regression signal worth
investigating (could indicate the refactor accidentally changed MPU
programming or stack layout).

## cm3/fault_isolate

Same root cause as `cm3/fault_overflow` above (deep stack overflow -> PSP
lands in the MPU deny region -> the MemManage handler's own hardware
auto-stack-push also lands there -> ARMv7-M escalates to HardFault instead
of delivering MemManage). Observed:
`Rivet CM3 fault_isolate\nfaulting task will be isolated\nHARD_FAULT\n`,
then hangs to the 30s timeout. Not fixed here; same disposition as
`fault_overflow`.

## cm3/stress_max_ptasks

Fails with `HARD_FAULT` shortly after boot (`Rivet CM3 stress_max_ptasks\n`
then `HARD_FAULT\n`, hangs to the 40s timeout). Root cause not
investigated — distinct symptom from the overflow tests above (this test
does not deliberately overflow anything; it fills the task registry to
`MAX_PTASKS` with plain 512-byte worker stacks, well within the 16 KiB
pool by a naive size sum). Left as a pre-existing, unexplained failure;
out of scope for the layering refactor. Flagged for a future investigation
pass (possibly stack-pool alignment padding, or an MPU region-7
reprogramming race under rapid spawn).

## Fixed during Phase 1 (harness bugs, not kernel bugs)

Two `xtask` test-table typos were corrected as part of establishing this
baseline (metadata-only, zero effect on kernel/example code):

- `cm3/join_test` was missing `extra_qemu_args: &["-machine", "lm3s6965evb"]`
  entirely, so QEMU exited immediately with "No machine specified" (exit
  code 1, zero bytes of guest output) — not a kernel issue, a missing test
  declaration.
- `cm3/join_test` and `cm3/respawn_test` had `ignore_log_lines: &["Timer
  with period zero, disabled"]` (past tense) but QEMU's stellaris-watchdog
  model actually emits "Timer with period zero, **disabling**" (present
  participle) — the mismatched string meant the benign line was never
  filtered, failing the qemu.log-must-be-clean assertion.

Both now pass. These are recorded here (not silently fixed) because they
change which tests are green in the "before" baseline used for regression
diffing in later phases.

## mps2-an385/demo: benign "misaligned PC" warning on SVC return (Phase 7)

Adding the `mps2-an385` board (Phase 7, proving the arch/board boundary)
surfaced a real, valuable bug — fixed, see below — and left one smaller,
unresolved oddity.

**Fixed:** `rivet::init()`'s `ASYNC_IDLE_STACK` (the cooperative-tier
executor's stack, spawned directly from a fixed `'static` buffer rather
than through the pool's own size-aligned carving) had no alignment
guarantee beyond `preempt::Stack`'s general 16 bytes. Since it's still
handed to `port::arch::on_switch_to` — which on Cortex-M reprograms an
MPU region sized to it, and an MPU region's base must be aligned to its
own size — a `.bss` layout that didn't happen to place it on a 4096-byte
boundary produced a genuinely misprogrammed MPU region (QEMU logged
`DRBAR[7]: ... misaligned to DRSR region size`). `lm3s6965evb`'s specific
`.bss` layout happened to avoid this by luck; `mps2-an385`'s different
statics (no GPIO heartbeat task, different `Once` cells) didn't. Fixed by
giving the static its own `#[repr(align(4096))]` wrapper in
`rivet/src/lib.rs` — verified the fix also silently applied to
`lm3s6965evb` (the bug existed there too, just unobserved).

**Not resolved:** after that fix, `mps2-an385`'s demo still logs "M
profile return from interrupt with misaligned PC is UNPREDICTABLE on
v7M" — consistently 5 times per run (matches the demo's 5
`spawn_ptask!` calls, suggesting a connection to the `rivet_svc_handler`
exception-return path, though not confirmed by tracing), and does not
occur on `lm3s6965evb` running the equivalent demo. The demo completes
correctly regardless (`SUCCESS`, exit 0) — this is QEMU flagging a
genuine ARMv7-M architectural violation somewhere in the guest that
doesn't visibly corrupt anything in this specific scenario, not a
functional failure. Not chased down further (would need GDB-level
tracing of the exact PC value at each SVC return, and GDB tooling in
this environment has its own unrelated issues — see the ctx_switch.py
notes elsewhere in this session). Tracked as a real, open question for
whoever next touches `rivet-bsp-mps2-an385` or `rivet-arch-cortex-m`'s
SVC handling. `mps2-an385`'s xtask test cases mark this specific line
as an accepted (not silently hidden) known-benign log line.

## Phase 10 — mps2-an385's QEMU NVIC model doesn't implement DEMCR

`rivet-arch-cortex-m::dwt::init` (execution-time accounting, plan.md
Phase 10) sets `DEMCR.TRCENA` before enabling the DWT cycle counter —
architectural on every real ARMv7-M core. On `mps2-an385`, QEMU logs
`NVIC: Bad read offset 0xdfc` / `NVIC: Bad write offset 0xdfc` for this
access: its NVIC/SCS device model doesn't implement the DEMCR register at
all. `lm3s6965evb`'s model does (no such warning there) — a genuine
per-machine-model gap in QEMU, not a kernel bug. Functionally harmless:
the DWT probe in `dwt::init` checks whether the cycle counter *actually
advanced* rather than trusting the write, so on `mps2-an385` it correctly
falls back to the SysTick-derived (coarser, still monotonic) cycle
source. Added to `mps2`'s `ignore_log_lines` in `xtask/src/main.rs`
alongside the other documented mps2-model quirks above.

## Phase 14 — Cortex-M interrupt-driven TX: ack-after-write self-limited the drain

Found and fixed during Phase 14's interrupt-driven console work, not a currently-open
issue — recorded because the debugging path is instructive.

Both Cortex-M UART ISRs (`rivet-bsp-lm3s6965`'s PL011, `rivet-bsp-mps2-an385`'s CMSDK
UART) originally acknowledged the TX-empty interrupt *after* writing the next queued
byte to the data register. On real silicon this ordering merely races (a fresh
trigger-level crossing between the two stores could be missed); under QEMU's PL011/CMSDK
UART models — no TX FIFO, no transmit timing, the data-register write is what
*synchronously* raises the TX-empty condition — it fails deterministically: acking
immediately after the write erases the very interrupt the write just generated, which
self-limits the ISR to draining exactly one byte per invocation regardless of how much
is queued. Combined with the "prime the pump" mechanism (needed because both UARTs'
TX-empty condition is edge-triggered, not level-sensed, so merely re-enabling the
interrupt mask doesn't recreate a missed edge), the ring backed up under any real message
length and the original drop-on-full policy silently truncated output mid-message —
producing symptoms (a message cut off partway, followed by fragments of *other*
messages) that looked exactly like byte-level corruption from concurrent producers, not
a one-line ack-ordering bug. Root-caused by consulting an advisor after independently
ruling out NVIC priority ordering, PRIMASK/critical-section correctness, and PL011
bit-position mistakes. Fixed by acking first, then writing; also replaced drop-on-full
with order-preserving backpressure (pull-oldest-and-write-directly-then-retry) in
`rivet::console::write_bytes_irq`, removing the ring-capacity-dependent truncation risk
entirely. A related, real (if latent) bug caught in the same review: the original
one-line `push {lr}` in `rivet-arch-cortex-m`'s PendSV handler left MSP 4-mod-8 aligned
across the `bl` into Rust — an AAPCS violation that's harmless on Cortex-M3 (no
alignment-sensitive load/store instructions used there) but would bite an M4F/M7 port;
fixed with an explicit `sub sp, #4`/`add sp, #4` pair rather than padding the push/pop
register list (r4 specifically is *not* a safe padding choice — it's overwritten with
the new task's real value by the `ldmia` a few instructions later, and popping over it
afterward would have silently restored the *old* task's stale r4).

## Phase 17 — pre-existing race: `joiner` leak on respawn causes intermittent AlreadyJoined

Found by scaling `soak_smoke`'s iteration count past the 200-iteration CI baseline (a
real, pre-Phase-10 kernel bug — not introduced by this session's Phase 10-16 work, just
never exercised hard enough to hit before). Fixed; recorded because the mechanism and
the debugging path are both instructive.

**Symptom:** intermittent `JOIN_MISMATCH ... got=AlreadyJoined` from a tight spawn/join/
despawn loop, non-monotonic in iteration count (N=200/300/500/8200/20000 all passed;
N=250 failed at iteration 216, N=1000 failed at iteration 56) — a timing-dependent race,
not a scale-dependent one.

**Root cause** (confirmed by an advisor after independent investigation had correctly
localized but not fully explained it): a worker task spawned at *higher* priority than
its joiner can run to completion — including `rivet_task_exit_core`'s
`joiner.swap(NO_TASK)` — before the joiner ever reaches its own registration CAS. The
swap drains a `joiner` field that's still `NO_TASK` (a no-op, correctly so at that
moment), the joiner's subsequent CAS then succeeds and sets `joiner` to itself, and
*nothing is ever left to clear it again* — `despawn`/`register_full` never touched
`joiner`, on the (false, for this ordering) assumption that the previous occupant's exit
path had already handled it. The next task spawned into the same recycled slot inherits
a permanently-stuck `joiner`, and its join CAS fails with `AlreadyJoined`.

A related, not-yet-observed-but-real second bug in the same area, found in the same
investigation: `join_task`'s `while !exited.load() { block_current(); ... }` loop's
check-then-block wasn't atomic, so a tick landing between the load and `block_current()`
could mark the joiner Ready-then-immediately-self-Blocked with nothing left to ever wake
it — a permanent hang, not just a wrong error. And a third: `spawn()` published a new
task as schedulable (`register_full`'s `ready_add`) *before* writing its result-size
metadata, so a fast-exiting higher-priority task could have its return value silently
never written.

**Fix:** `tcb::register_full` now unconditionally resets `joiner`/`exited`/
`stop_requested`/`result_size`/`held_count` inside its RESERVED publish window (self-
sufficient slot recycling, no cross-task ordering assumption); `join_task` releases its
own `joiner` registration via CAS on every return path; the wait loop's check-then-block
is atomic under `critical::enter` (the same `[B1]` pattern `PriorityMutex::lock_timeout`
already uses); `spawn()` publishes registration and result metadata together in one
critical section. Verified against the exact failing N values (250, 1000), plus every
value up to 20,000 tested during the fix — all pass.

## Phase 19 — `clint::msip()` targeted hart 0's register regardless of caller

**Symptom:** `examples/qemu-riscv/src/bin/smp_test.rs` (built with `RIVET_MAX_HARTS=2`)
hung 100% reproducibly under `-smp 2` (3/3 runs, deterministic — QEMU's TCG round-robin
vCPU stepping is itself deterministic per config), while the identical binary passed
reliably at `-smp 1` and, misleadingly, at `-smp 4` (3/3 runs there too).

**Root cause:** `clint::msip()` — the backing function for both
`__rivet_arch_request_reschedule`'s self-IPI and `ack_soft_irq`'s pending-bit clear —
was `(base() + MSIP_OFFSET) as *mut u32`: always hart 0's `MSIP` register, regardless of
which hart called it. Before Phase 19 this was correct by accident (hart 0 was the only
hart that ever ran kernel code, so "self" and "hart 0" were the same fact). Once
secondary harts run real tasks that call `request_reschedule()` on themselves (blocking
in `sleep_until`, `park_forever`, `yield_now`), a secondary hart's self-request pends
*hart 0's* software interrupt instead of its own — the calling hart never traps on its
own request and silently falls through past what was meant to be a synchronous
context-switch point, while its task state is left `Blocked` with nothing to ever
actually preempt it into blocking. `-smp 4` happened to pass anyway: `sched::ready_add`'s
`wake_other_harts()` broadcast (which correctly targets each hart's own `MSIP` via
`request_reschedule_on`/`msip_for`) fires far more often with more harts contending for
the same ready pool, and apparently ended up masking the self-target bug's effect often
enough in this specific binary's timing to pass 3/3 — a reminder that "passes at the
higher hart count" is not evidence of correctness by itself; `-smp 2`'s lower IPI
traffic just made the bug reliably visible instead of accidentally papered over.

**Fix:** `msip()` now resolves via `msip_for(riscv::register::mhartid::read())` — a live
CSR read on every call, not a cached value — so it always targets the calling hart's own
register. Verified: `-smp 2` now passes 3/3 (previously 0/3); `-smp 1` and `-smp 4` still
pass; the full riscv smoke suite (single-hart, `RIVET_MAX_HARTS=1` default) is
unaffected, confirming the fix is behavior-preserving there.

## Phase 19 — pre-existing Miri UB in `irq::dispatch`'s function-pointer round-trip

Found while re-running the full verification block (`cargo +nightly miri test -p rivet
--lib`) as part of Phase 19's acceptance criteria — a real, pre-existing bug from Phase
13, not introduced by Phase 19's own changes (confirmed via `git stash`: reproduces
identically on the pre-Phase-19 commit).

**Symptom:** `error: Undefined Behavior: pointer not dereferenceable ... dangling
pointer (it has no provenance)` at `irq.rs:91`, inside `irq::dispatch`.

**Root cause:** `register` stores a handler via `slot.store(handler as usize, ...)` (a
blessed "exposing" pointer-to-integer cast); `dispatch` retrieved it via
`core::mem::transmute::<usize, fn()>(ptr)` — a direct bit-level reinterpret, not an `as`
int-to-pointer cast. Miri's provenance tracking recognizes `as usize` → `as *const T` as
the pair that exposes-then-looks-up an allocation's provenance; `transmute` between
`usize` and a pointer-shaped type bypasses that machinery entirely, so Miri (correctly,
per the strict-provenance model) treats the resulting "pointer" as having no provenance
at all — dangling, even though the bit pattern is the original function's real address.

**Fix:** insert an explicit `ptr as *const ()` cast (the operation Miri's model actually
recognizes) before transmuting the *pointer* (not the raw integer) to `fn()`. Same bits,
same runtime behavior on real hardware (which has no notion of "provenance" at all —
this only matters for Miri's abstract machine), but now something Miri accepts. Loud
warning remains (`integer-to-pointer cast ... Miri might miss pointer bugs`), which is
expected and not itself a failure — it flags that this specific operation opts out of
Miri's strongest (zero-int-to-ptr-casts) provenance mode, which is unavoidable for a
function-pointer table stored as `AtomicUsize` slots.

## Phase 19 — pre-existing loom compile failures in four unrelated statics

Found the same way as the Miri bug above: re-running the full verification block
(`RUSTFLAGS='--cfg loom' cargo test -p rivet --features loom --test loom --release`)
surfaced 12 compile errors across `console.rs` (RX/TX `Channel`/`Sender`/`Receiver`),
`log.rs` (`CHANNEL`/`SENDER`/`RECEIVER`), `deadlines.rs` (`PERIOD_US`/`BUDGET_US`), and
`latency.rs` (`HISTOGRAMS`) — all pre-existing (Phases 11/12/14/16), reproduces
identically via `git stash` on the pre-Phase-19 commit. None of these statics had ever
actually been loom-compiled before; the crate's own precedent for this exact situation
(`tests/loom.rs`'s comment: "loom's atomics are not const-constructible, so `Channel::
new` can't initialize a plain `static`") had simply never been applied to the newer
files that needed it.

**Fix:** wrapped each in the established `#[cfg(not(loom))] static X = ...` /
`#[cfg(loom)] loom::lazy_static! { static ref X = ...; }` pattern already used
throughout the older files (`critical.rs`, `waker.rs`, `irq.rs`, `executor.rs`). Verified
the full loom suite now compiles and passes (4/4 tests) under `--cfg loom`.

## Phase 19 — `global_asm!` + `lto = true` can't rely on the target triple's own ISA extensions

Found while writing the per-hart ISR-stack/boot-stack index math in
`rivet-arch-riscv`/`rivet-rt`'s `global_asm!` blocks: a plain `mul` instruction (RV32
`M` extension — `riscv32imac-unknown-none-elf`'s own target name literally spells out
`m`) compiled fine under `cargo build --target riscv32imac-unknown-none-elf` (default
`dev` profile) but failed as `error: instruction requires the following: 'Zmmul'` when
the *same crate* was pulled into the full `qemu-riscv` binary under `--release`
(`lto = true, codegen-units = 1` in the workspace's `[profile.release]`) — reproducible,
not flaky. Root cause not fully chased down (plausibly an LLVM/rustc interaction where
fat-LTO's single merged codegen unit assembles `global_asm!` blocks against a narrower
subtarget-feature string than the crate's own compile flags would otherwise select), but
not worth chasing further: both use sites were multiplying by a compile-time
power-of-two constant (2048, then 512), so `slli` (base RV32I, needs no extension at
all) is both a strictly better instruction choice and sidesteps the whole question.
**Lesson for future `global_asm!` work in this crate: avoid `M`-extension instructions
in `global_asm!` blocks entirely if a base-ISA equivalent exists — plain `cargo build
--target ... ` without `--release` is not sufficient to catch this, since the failure
only manifests under the LTO'd release profile the QEMU harness actually uses.**

Separately, but found via the same build attempts: the `qemu-riscv` binaries' `.bss`/
`.rivet_tasks`/stack-section RAM budget (QEMU `virt`'s 128K) is *already* tight enough at
`opt-level = "z"` that a naive `cargo build` (`dev` profile, no size optimization) always
overflows it — not a regression, just confirms these binaries were never meant to be
built outside the release profile the harness always uses. Also: initially reserved
`.isr_stack`/`.secondary_stacks` for an 8-hart ceiling (matching `RIVET_MAX_HARTS`'s
crate-wide absolute max), which overflowed the 128K budget; right-sized to a 4-hart
ceiling instead (matching this board's own documented SMP scope — plan.md Phase 19:
"qemu-virt SMP build sets `RIVET_MAX_HARTS` to the `-smp` count, max 4").
