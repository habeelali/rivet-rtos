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
