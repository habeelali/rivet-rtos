//! Rivet QEMU test harness (plan.md §1.6, §1.7).
//!
//! Replaces `scripts/run-*.sh` with a typed, in-tree harness:
//!
//! - builds the example for its target triple (release or `release-checked`);
//! - runs it under QEMU with a wall-clock timeout, capturing guest output;
//! - asserts the **exit code** and an **ordered golden-output regex
//!   sequence** (not just "contains");
//! - always runs with `-d guest_errors -D qemu.log` and **fails the test if
//!   the log is non-empty** unless the test declares expected traps;
//! - supports `-icount shift=N` for instruction-count-deterministic runs
//!   (§1.7), used by the timing/regression suites;
//! - RISC-V exits via `riscv.sifive.test` (0x5555 = pass,
//!   `0x3333 | code << 16` = fail with a distinguishable code) — the
//!   semihosting workaround is a secondary path only.
//!
//! Usage:
//! ```text
//! cargo xtask test --target riscv|cm3 [--suite smoke|stress|gdb] [--profile release|release-checked] [--icount N]
//! cargo xtask soak --target riscv|cm3 --sim-hours N
//! cargo xtask list --target riscv|cm3
//! ```

use std::env;
use std::path::PathBuf;
use std::process::{Command, Stdio};
use std::thread;
use std::time::{Duration, Instant};

use regex::Regex;

// ── Target definitions ──────────────────────────────────────────────

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Target {
    Riscv,
    Cm3,
}

impl Target {
    fn from_str(s: &str) -> Result<Self, String> {
        match s {
            "riscv" => Ok(Target::Riscv),
            "cm3" => Ok(Target::Cm3),
            other => Err(format!(
                "unknown target `{other}` (expected `riscv` or `cm3`)"
            )),
        }
    }

    fn triple(self) -> &'static str {
        match self {
            Target::Riscv => "riscv32imac-unknown-none-elf",
            Target::Cm3 => "thumbv7m-none-eabi",
        }
    }

    fn qemu_bin(self) -> &'static str {
        match self {
            Target::Riscv => "qemu-system-riscv32",
            Target::Cm3 => "qemu-system-arm",
        }
    }
}

// ── Test cases ─────────────────────────────────────────────────────

#[derive(Clone)]
struct TestCase {
    name: &'static str,
    pkg: &'static str,
    /// Binary target name (the ELF name under target/<triple>/<profile>/).
    bin: &'static str,
    /// Ordered golden regexes. Each must match, in order, somewhere after
    /// the previous match in the captured guest output.
    golden: &'static [&'static str],
    /// Expected QEMU exit status.
    exit_code: i32,
    /// Wall-clock timeout before the test is killed and fails.
    timeout: Duration,
    /// `-icount shift=N` (deterministic timing, §1.7). `None` = no icount.
    icount: Option<u32>,
    /// Pass `-d int` too (fault/interrupt suites). The log must still be
    /// empty unless `allow_traps`.
    log_int: bool,
    /// Declare expected traps: qemu.log is allowed to be non-empty.
    allow_traps: bool,
    /// Assert the golden output even when the guest never exits (fault
    /// tests under Panic policy halt or reset instead of exiting cleanly).
    assert_golden_on_timeout: bool,
    /// Known-benign QEMU machine-model lines allowed in qemu.log (e.g. the
    /// lm3s6965evb stellaris watchdog emits "Timer with period zero,
    /// disabling" at device reset, before any guest code runs). Every
    /// *other* line still fails the test.
    ignore_log_lines: &'static [&'static str],
    /// Extra QEMU args appended verbatim.
    extra_qemu_args: &'static [&'static str],
}

fn demo_golden() -> &'static [&'static str] {
    &[
        r"Rivet RTOS v0\.1\.0",
        r"Phase 0: priority inheritance",
        r"\[pi_low: acquiring mutex\]",
        r"\[pi_low: holds mutex, spawning medium\+high\]",
        r"\[pi_high: trying to acquire mutex\]",
        r"\[pi_low: critical section done, releasing\]",
        r"\[pi_high: got mutex — priority inheritance worked\]",
        // Interleaved A/B preemption proof: A, then B, then A again.
        r"A",
        r"B",
        r"A",
        r"consumer: sum=15",
        r"SUCCESS",
    ]
}

fn smoke_tests(target: Target) -> Vec<TestCase> {
    match target {
        Target::Riscv => vec![
            TestCase {
                name: "demo",
                pkg: "qemu-riscv",
                bin: "qemu-riscv",
                golden: demo_golden(),
                exit_code: 0,
                timeout: Duration::from_secs(90),
                icount: None,
                log_int: false,
                allow_traps: false,
                assert_golden_on_timeout: false,
                // QEMU logs a line whenever a pmpcfg write re-touches an
                // already-locked entry's byte (the guards in the same
                // register are configured at different spawn times; QEMU
                // still applies the unlocked bytes). Benign by design.
                ignore_log_lines: &[
                    "ignoring pmpcfg write - locked",
                    "ignoring pmpaddr write - locked",
                    "ignoring pmpaddr write - pmpcfg + 1 locked",
                ],
                extra_qemu_args: &["-machine", "virt", "-cpu", "rv32", "-bios", "none"],
            },
            // plan.md §2.3 acceptance: nested inheritance trace ([B11]),
            // lock_timeout/try_lock, and 1M-cycle contention stress ([B1]).
            TestCase {
                name: "mutex_test",
                pkg: "qemu-riscv",
                bin: "mutex_test",
                golden: &[
                    r"TIMEOUT_OK",
                    r"TRYLOCK_OK",
                    r"HOLDS_AB",
                    r"EFF_WHILE_HOLDING=8",
                    r"EFF_AFTER_UNLOCK_B=8",
                    r"WA_GOT_A",
                    r"WB_GOT_B",
                    r"EFF_AFTER_UNLOCK_A=2",
                    r"MUTEX_OK",
                ],
                exit_code: 0,
                timeout: Duration::from_secs(120),
                icount: None,
                log_int: false,
                allow_traps: false,
                assert_golden_on_timeout: false,
                ignore_log_lines: &[
                    "ignoring pmpcfg write - locked",
                    "ignoring pmpaddr write - locked",
                    "ignoring pmpaddr write - pmpcfg + 1 locked",
                ],
                extra_qemu_args: &["-machine", "virt", "-cpu", "rv32", "-bios", "none"],
            },
            // plan.md §2.4 [B2] acceptance: spawn workers from a running
            // task under -icount (deterministic ticks mid-registration);
            // one spawn past capacity must return None; no trap allowed.
            TestCase {
                name: "stress_spawn",
                pkg: "qemu-riscv",
                bin: "stress_spawn",
                golden: &[r"SPAWNER_FULL_OK", r"SPAWN_STRESS_OK"],
                exit_code: 0,
                timeout: Duration::from_secs(60),
                icount: Some(10),
                log_int: false,
                allow_traps: false,
                assert_golden_on_timeout: false,
                ignore_log_lines: &[
                    "ignoring pmpcfg write - locked",
                    "ignoring pmpaddr write - locked",
                    "ignoring pmpaddr write - pmpcfg + 1 locked",
                ],
                extra_qemu_args: &["-machine", "virt", "-cpu", "rv32", "-bios", "none"],
            },
            // ── Phase 3 fault suite ────────────────────────────────────
            // plan.md §3.6: stack overflow → Panic policy dump + exit 0xFA.
            TestCase {
                name: "fault_overflow",
                pkg: "qemu-riscv",
                bin: "fault_overflow",
                golden: &[r"RIVET FAULT", r"store-access", r"task="],
                exit_code: 250,
                timeout: Duration::from_secs(30),
                icount: None,
                log_int: false,
                allow_traps: true, // the fault itself logs a trap
                assert_golden_on_timeout: false,
                ignore_log_lines: &[
                    "ignoring pmpcfg write - locked",
                    "ignoring pmpaddr write - locked",
                    "ignoring pmpaddr write - pmpcfg + 1 locked",
                ],
                extra_qemu_args: &["-machine", "virt", "-cpu", "rv32", "-bios", "none"],
            },
            // plan.md §3.4: IsolateTask policy — the system survives a
            // faulting task; its mutex is poisoned.
            TestCase {
                name: "fault_isolate",
                pkg: "qemu-riscv",
                bin: "fault_isolate",
                golden: &[
                    r"RIVET FAULT",
                    r"HOOK_SAW_TASK=1",
                    r"POISONED_OK",
                    r"ISOLATION_OK",
                ],
                exit_code: 0,
                timeout: Duration::from_secs(30),
                icount: None,
                log_int: false,
                allow_traps: true,
                assert_golden_on_timeout: false,
                ignore_log_lines: &[
                    "ignoring pmpcfg write - locked",
                    "ignoring pmpaddr write - locked",
                    "ignoring pmpaddr write - pmpcfg + 1 locked",
                ],
                extra_qemu_args: &["-machine", "virt", "-cpu", "rv32", "-bios", "none"],
            },
            // plan.md §5.2/§5.3: task entry returns → kernel exit trampoline
            // stores the result and wakes the joiner.
            TestCase {
                name: "join_test",
                pkg: "qemu-riscv",
                bin: "join_test",
                golden: &[r"JOIN_OK v=42", r"JOIN_TEST_OK"],
                exit_code: 0,
                timeout: Duration::from_secs(20),
                icount: Some(0),
                log_int: false,
                allow_traps: false,
                assert_golden_on_timeout: true,
                extra_qemu_args: &["-machine", "virt", "-cpu", "rv32", "-bios", "none"],
                ignore_log_lines: &[
                    "ignoring pmpcfg write - locked",
                    "ignoring pmpaddr write - locked",
                ],
            },
            // plan.md §5.4/§5.5: despawn → slot+stack recycle → respawn with
            // stale-handle detection, plus pause/resume.
            TestCase {
                name: "respawn_test",
                pkg: "qemu-riscv",
                bin: "respawn_test",
                golden: &[r"RESPAWN_TEST_OK"],
                exit_code: 0,
                timeout: Duration::from_secs(20),
                icount: Some(0),
                log_int: false,
                allow_traps: false,
                assert_golden_on_timeout: true,
                extra_qemu_args: &["-machine", "virt", "-cpu", "rv32", "-bios", "none"],
                ignore_log_lines: &[
                    "ignoring pmpcfg write - locked",
                    "ignoring pmpaddr write - locked",
                ],
            },
            // plan.md §3.5: software watchdog fires and resets (0x7777 →
            // QEMU reboots; the marker proves the timeout was diagnosed).
            TestCase {
                name: "watchdog_test",
                pkg: "qemu-riscv",
                bin: "watchdog_test",
                golden: &[r"RIVET WATCHDOG TIMEOUT"],
                exit_code: 0,
                timeout: Duration::from_secs(20),
                icount: None,
                log_int: false,
                allow_traps: false,
                assert_golden_on_timeout: true,
                ignore_log_lines: &[
                    "ignoring pmpcfg write - locked",
                    "ignoring pmpaddr write - locked",
                    "ignoring pmpaddr write - pmpcfg + 1 locked",
                ],
                extra_qemu_args: &["-machine", "virt", "-cpu", "rv32", "-bios", "none"],
            },
            // plan.md §4.4: fill the registry, typed error on overflow.
            TestCase {
                name: "stress_max_ptasks",
                pkg: "qemu-riscv",
                bin: "stress_max_ptasks",
                golden: &[r"STRESS_MAX_OK ran=14"],
                exit_code: 0,
                timeout: Duration::from_secs(40),
                icount: None,
                log_int: false,
                allow_traps: false,
                assert_golden_on_timeout: false,
                ignore_log_lines: &[
                    "ignoring pmpcfg write - locked",
                    "ignoring pmpaddr write - locked",
                    "ignoring pmpaddr write - pmpcfg + 1 locked",
                ],
                extra_qemu_args: &["-machine", "virt", "-cpu", "rv32", "-bios", "none"],
            },
            // plan.md §2.2 [B6] acceptance: 10 x 100ms sleeps must elapse
            // exactly 1s ± 30ms under -icount (tick re-armed from previous
            // mtimecmp; per-tick latency is ~µs under icount, so 10k ticks
            // of drift would fail).
            TestCase {
                name: "drift_test",
                pkg: "qemu-riscv",
                bin: "drift_test",
                golden: &[r"DRIFT_OK"],
                exit_code: 0,
                timeout: Duration::from_secs(120),
                icount: Some(10),
                log_int: false,
                allow_traps: false,
                assert_golden_on_timeout: false,
                ignore_log_lines: &[
                    "ignoring pmpcfg write - locked",
                    "ignoring pmpaddr write - locked",
                    "ignoring pmpaddr write - pmpcfg + 1 locked",
                ],
                extra_qemu_args: &["-machine", "virt", "-cpu", "rv32", "-bios", "none"],
            },
        ],
        Target::Cm3 => vec![
            TestCase {
                name: "demo",
                pkg: "qemu-cm3",
                bin: "qemu-cm3",
                golden: demo_golden(),
                exit_code: 0,
                timeout: Duration::from_secs(90),
                icount: None,
                log_int: false,
                allow_traps: false,
                assert_golden_on_timeout: false,
                // lm3s6965evb's stellaris watchdog/gptm models emit this at
                // device reset (period zero), before any guest instruction
                // runs.
                ignore_log_lines: &["Timer with period zero, disabling"],
                extra_qemu_args: &["-machine", "lm3s6965evb"],
            },
            // plan.md §2.3 acceptance: nested inheritance trace ([B11]),
            // lock_timeout/try_lock, and 1M-cycle contention stress ([B1]).
            TestCase {
                name: "mutex_test",
                pkg: "qemu-cm3",
                bin: "mutex_test",
                golden: &[
                    r"TIMEOUT_OK",
                    r"TRYLOCK_OK",
                    r"HOLDS_AB",
                    r"EFF_WHILE_HOLDING=8",
                    r"EFF_AFTER_UNLOCK_B=8",
                    r"WA_GOT_A",
                    r"WB_GOT_B",
                    r"EFF_AFTER_UNLOCK_A=2",
                    r"MUTEX_OK",
                ],
                exit_code: 0,
                timeout: Duration::from_secs(120),
                icount: None,
                log_int: false,
                allow_traps: false,
                assert_golden_on_timeout: false,
                ignore_log_lines: &["Timer with period zero, disabling"],
                extra_qemu_args: &["-machine", "lm3s6965evb"],
            },
            // plan.md §2.4 [B2] acceptance: spawn workers from a running
            // task under -icount; one spawn past capacity must return None.
            // shift=6 (not 10): the Cortex-M SysTick is a hardware counter
            // that cannot coalesce missed ticks the way the RISC-V CLINT
            // does, so at shift=10 the ISR duration can exceed the tick
            // period and storm.
            TestCase {
                name: "stress_spawn",
                pkg: "qemu-cm3",
                bin: "stress_spawn",
                golden: &[r"SPAWNER_FULL_OK", r"SPAWN_STRESS_OK"],
                exit_code: 0,
                timeout: Duration::from_secs(60),
                icount: Some(6),
                log_int: false,
                allow_traps: false,
                assert_golden_on_timeout: false,
                ignore_log_lines: &["Timer with period zero, disabling"],
                extra_qemu_args: &["-machine", "lm3s6965evb"],
            },
            // ── Phase 3 fault suite ────────────────────────────────────
            // plan.md §3.6: stack overflow → MemManage → Panic dump + halt.
            TestCase {
                name: "fault_overflow",
                pkg: "qemu-cm3",
                bin: "fault_overflow",
                golden: &[r"RIVET FAULT", r"memmanage", r"RIVET_FAILURE code=250"],
                exit_code: 0,
                timeout: Duration::from_secs(30),
                icount: None,
                log_int: false,
                allow_traps: true, // the fault itself logs a trap
                assert_golden_on_timeout: true,
                ignore_log_lines: &["Timer with period zero, disabling"],
                extra_qemu_args: &["-machine", "lm3s6965evb"],
            },
            // plan.md §3.4: IsolateTask policy via the asm MemManage entry.
            TestCase {
                name: "fault_isolate",
                pkg: "qemu-cm3",
                bin: "fault_isolate",
                golden: &[
                    r"RIVET FAULT",
                    r"HOOK_SAW_TASK=1",
                    r"POISONED_OK",
                    r"ISOLATION_OK",
                ],
                exit_code: 0,
                timeout: Duration::from_secs(30),
                icount: None,
                log_int: false,
                allow_traps: true,
                assert_golden_on_timeout: false,
                ignore_log_lines: &["Timer with period zero, disabling"],
                extra_qemu_args: &["-machine", "lm3s6965evb"],
            },
            // plan.md §5.2/§5.3: task exit + join on the Cortex-M3 port.
            TestCase {
                name: "join_test",
                pkg: "qemu-cm3",
                bin: "join_test",
                golden: &[r"JOIN_OK v=42", r"JOIN_TEST_OK"],
                exit_code: 0,
                timeout: Duration::from_secs(20),
                icount: Some(0),
                log_int: false,
                allow_traps: false,
                assert_golden_on_timeout: true,
                extra_qemu_args: &["-machine", "lm3s6965evb"],
                ignore_log_lines: &["Timer with period zero, disabling"],
            },
            // plan.md §5.4/§5.5: respawn + pause/resume on the Cortex-M3 port.
            TestCase {
                name: "respawn_test",
                pkg: "qemu-cm3",
                bin: "respawn_test",
                golden: &[r"RESPAWN_TEST_OK"],
                exit_code: 0,
                timeout: Duration::from_secs(20),
                icount: Some(0),
                log_int: false,
                allow_traps: false,
                assert_golden_on_timeout: true,
                extra_qemu_args: &["-machine", "lm3s6965evb"],
                ignore_log_lines: &["Timer with period zero, disabling"],
            },
            // plan.md §3.5: real hardware WDT reset — the banner appearing
            // twice (ordered golden) proves the guest rebooted.
            TestCase {
                name: "watchdog_test",
                pkg: "qemu-cm3",
                bin: "watchdog_test",
                golden: &[r"watchdog_test: feeding", r"watchdog_test: feeding"],
                exit_code: 0,
                timeout: Duration::from_secs(20),
                icount: None,
                log_int: false,
                allow_traps: false,
                assert_golden_on_timeout: true,
                ignore_log_lines: &["Timer with period zero, disabling"],
                extra_qemu_args: &["-machine", "lm3s6965evb"],
            },
            // plan.md §4.4: fill the registry, typed error on overflow.
            TestCase {
                name: "stress_max_ptasks",
                pkg: "qemu-cm3",
                bin: "stress_max_ptasks",
                golden: &[r"STRESS_MAX_OK ran=14"],
                exit_code: 0,
                timeout: Duration::from_secs(40),
                icount: None,
                log_int: false,
                allow_traps: false,
                assert_golden_on_timeout: false,
                ignore_log_lines: &["Timer with period zero, disabling"],
                extra_qemu_args: &["-machine", "lm3s6965evb"],
            },
            // plan.md §2.2 [B5] acceptance: run past 2^32 µs of kernel
            // time (tick accelerated to 10 µs in the test binary);
            // Sleep::<100_000> must still fire (old u32 µs counter wrapped
            // at 71.6 min and hung).
            TestCase {
                name: "soak_time_wrap",
                pkg: "qemu-cm3",
                bin: "soak_time_wrap",
                golden: &[r"AFTER_WRAP"],
                exit_code: 0,
                timeout: Duration::from_secs(180),
                icount: None,
                log_int: false,
                allow_traps: false,
                assert_golden_on_timeout: false,
                ignore_log_lines: &["Timer with period zero, disabling"],
                extra_qemu_args: &["-machine", "lm3s6965evb"],
            },
        ],
    }
}

// ── Runner ─────────────────────────────────────────────────────────

fn workspace_root() -> PathBuf {
    // xtask is always run from the workspace root (`cargo xtask ...`), but
    // resolve robustly: parent of the xtask dir.
    let mut dir = env::current_exe().expect("current exe");
    dir.pop(); // xtask
    dir.pop(); // target/<profile>/ -> target
    dir.pop(); // -> workspace root
    dir
}

struct QemuResult {
    stdout: String,
    exit_code: Option<i32>,
    timed_out: bool,
}

fn run_qemu(qemu: &str, args: &[String], timeout: Duration, log_file: &PathBuf) -> QemuResult {
    let mut cmd = Command::new(qemu);
    cmd.args(args)
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .stdin(Stdio::null());

    let mut child = cmd.spawn().unwrap_or_else(|e| {
        panic!("failed to spawn {qemu}: {e} (is qemu-system-misc / qemu-system-arm installed?)")
    });
    let mut stdout = child.stdout.take().unwrap();
    let mut stderr = child.stderr.take().unwrap();

    // Drain stdout+stderr on a thread so the child never blocks on a full
    // pipe while we wait.
    let (tx, rx) = std::sync::mpsc::channel();
    let _reader = thread::spawn(move || {
        use std::io::Read;
        let mut out = String::new();
        let mut err = String::new();
        let _ = stdout.read_to_string(&mut out);
        let _ = stderr.read_to_string(&mut err);
        let _ = tx.send((out, err));
    });

    let start = Instant::now();
    let status = loop {
        match child.try_wait() {
            Ok(Some(status)) => break Some(status),
            Ok(None) => {
                if start.elapsed() > timeout {
                    let _ = child.kill();
                    let _ = child.wait();
                    let (out, err) = rx.recv().unwrap_or_default();
                    let _ = std::fs::write(log_file, err);
                    return QemuResult {
                        stdout: out,
                        exit_code: None,
                        timed_out: true,
                    };
                }
                thread::sleep(Duration::from_millis(50));
            }
            Err(e) => panic!("error waiting on qemu: {e}"),
        }
    };
    let (out, err) = rx.recv().unwrap_or_default();
    // Keep stderr (QEMU's own diagnostics) in the log file too — it is
    // distinct from the guest-error log written via -D.
    if !err.trim().is_empty() {
        let _ = std::fs::write(log_file, err);
    }
    QemuResult {
        stdout: out,
        exit_code: status.and_then(|s| s.code()),
        timed_out: false,
    }
}

fn assert_ordered_golden(stdout: &str, golden: &[&str], name: &str) {
    let mut search_from = 0;
    for (i, pat) in golden.iter().enumerate() {
        let re = Regex::new(pat).unwrap_or_else(|e| panic!("bad golden regex {pat:?}: {e}"));
        let rest = &stdout[search_from..];
        let m = re.find(rest).unwrap_or_else(|| {
            panic!(
                "test `{name}`: golden pattern #{i} `{pat}` not found after position \
                 {search_from} in guest output:\n---\n{stdout}\n---"
            )
        });
        search_from += m.end();
    }
}

fn build_example(pkg: &str, bin: &str, target: Target, profile: &str) -> PathBuf {
    let mut cmd = Command::new(env::var("CARGO").unwrap_or_else(|_| "cargo".into()));
    cmd.args([
        "build",
        "--package",
        pkg,
        "--bin",
        bin,
        "--target",
        target.triple(),
    ]);
    if profile != "release" {
        cmd.arg("--profile").arg(profile);
    } else {
        cmd.arg("--release");
    }
    let status = cmd
        .status()
        .unwrap_or_else(|e| panic!("failed to run cargo build: {e}"));
    assert!(status.success(), "cargo build of {pkg} failed");

    workspace_root()
        .join("target")
        .join(target.triple())
        .join(profile)
        .join(bin)
}

/// Golden-baseline capture (layering-refactor plan.md Phase 1): runs a test
/// case exactly like `run_test_case` but never asserts — just records raw
/// guest stdout + exit status to `tests/golden/<target>-<name>.txt`. Used to
/// snapshot current behavior before a refactor so later phases can diff
/// against it byte-for-byte instead of re-deriving expectations.
fn capture_test_case(target: Target, tc: &TestCase, profile: &str, out_dir: &PathBuf) {
    let el = build_example(tc.pkg, tc.bin, target, profile);
    let logs_dir = workspace_root().join("target").join("qemu-logs");
    std::fs::create_dir_all(&logs_dir).expect("create qemu-logs dir");
    let log_file = logs_dir.join(format!("{}-{}.log", target_name(target), tc.name));
    let _ = std::fs::remove_file(&log_file);

    let mut args: Vec<String> = vec![];
    args.extend(tc.extra_qemu_args.iter().map(|s| s.to_string()));
    args.push("-kernel".into());
    args.push(el.to_str().unwrap().into());
    args.push("-nographic".into());
    args.push("-monitor".into());
    args.push("none".into());
    args.push("-serial".into());
    args.push("stdio".into());
    if let Some(shift) = tc.icount {
        args.push("-icount".to_string());
        args.push(format!("shift={shift}"));
    }
    if tc.log_int {
        args.push("-d".into());
        args.push("int,guest_errors".into());
    } else {
        args.push("-d".into());
        args.push("guest_errors".into());
    }
    args.push("-D".into());
    args.push(log_file.to_str().unwrap().into());
    if target == Target::Cm3 {
        args.push("-semihosting".into());
    }

    eprintln!("[xtask] capturing {}/{}", target_name(target), tc.name);
    let result = run_qemu(target.qemu_bin(), &args, tc.timeout, &log_file);

    std::fs::create_dir_all(out_dir).expect("create golden out dir");
    let out_file = out_dir.join(format!("{}-{}.txt", target_name(target), tc.name));
    let header = format!(
        "# exit_code={:?} timed_out={}\n",
        result.exit_code, result.timed_out
    );
    std::fs::write(&out_file, format!("{header}{}", result.stdout)).expect("write golden file");
    eprintln!(
        "[xtask]   -> {} ({} bytes, exit={:?}, timed_out={})",
        out_file.display(),
        result.stdout.len(),
        result.exit_code,
        result.timed_out
    );
}

fn run_test_case(target: Target, tc: &TestCase, profile: &str) {
    let el = build_example(tc.pkg, tc.bin, target, profile);
    let logs_dir = workspace_root().join("target").join("qemu-logs");
    std::fs::create_dir_all(&logs_dir).expect("create qemu-logs dir");
    let log_file = logs_dir.join(format!("{}-{}.log", target_name(target), tc.name));
    let _ = std::fs::remove_file(&log_file);

    let mut args: Vec<String> = vec![];
    args.extend(tc.extra_qemu_args.iter().map(|s| s.to_string()));
    args.push("-kernel".into());
    args.push(el.to_str().unwrap().into());
    args.push("-nographic".into());
    args.push("-monitor".into());
    args.push("none".into());
    args.push("-serial".into());
    args.push("stdio".into());
    if let Some(shift) = tc.icount {
        args.push("-icount".to_string());
        args.push(format!("shift={shift}"));
    }
    // `-d guest_errors` always; add `int` only when the test declares it
    // (fault/interrupt suites). The log written via -D is asserted empty
    // unless `allow_traps`.
    if tc.log_int {
        args.push("-d".into());
        args.push("int,guest_errors".into());
    } else {
        args.push("-d".into());
        args.push("guest_errors".into());
    }
    args.push("-D".into());
    args.push(log_file.to_str().unwrap().into());
    // Cortex-M needs semihosting for its exit path; RISC-V exits via
    // riscv.sifive.test.
    if target == Target::Cm3 {
        args.push("-semihosting".into());
    }

    eprintln!(
        "[xtask] running {}/{}: {} (timeout {}s)",
        target_name(target),
        tc.name,
        tc.name,
        tc.timeout.as_secs()
    );
    let result = run_qemu(target.qemu_bin(), &args, tc.timeout, &log_file);

    if result.timed_out {
        if tc.assert_golden_on_timeout {
            // The guest intentionally never exits (fault test halted or
            // reset); still verify the ordered golden output.
            assert_ordered_golden(&result.stdout, tc.golden, tc.name);
            eprintln!(
                "[xtask] PASS {}/{} (halted as expected)",
                target_name(target),
                tc.name
            );
            return;
        }
        panic!(
            "test `{}` TIMED OUT after {}s; last output:\n---\n{}\n---",
            tc.name,
            tc.timeout.as_secs(),
            result.stdout
        );
    }
    let actual = result.exit_code.expect("qemu exited without a status code");
    assert_eq!(
        actual, tc.exit_code,
        "test `{}`: exit code {actual}, expected {}; output:\n---\n{}\n---",
        tc.name, tc.exit_code, result.stdout
    );

    assert_ordered_golden(&result.stdout, tc.golden, tc.name);

    // qemu.log must be empty unless the test declares expected traps or
    // the lines are in the known-benign allow-list.
    if !tc.allow_traps {
        let log = std::fs::read_to_string(&log_file).unwrap_or_default();
        let unexpected: Vec<&str> = log
            .lines()
            .filter(|line| {
                let trimmed = line.trim();
                !trimmed.is_empty()
                    && !tc
                        .ignore_log_lines
                        .iter()
                        .any(|pat| trimmed.starts_with(pat))
            })
            .collect();
        assert!(
            unexpected.is_empty(),
            "test `{}`: qemu.log contains {} undeclared line(s) (trap / guest error):\n---\n{}\n---",
            tc.name,
            unexpected.len(),
            unexpected.join("\n")
        );
    }

    eprintln!("[xtask] PASS {}/{}", target_name(target), tc.name);
}

/// Run the GDB-scripted context-switch verification (plan.md §1.8):
/// start QEMU halted with the gdb stub on :1234, run the gdb script against
/// it, then kill QEMU. Requires `gdb-multiarch` (or `$RIVET_GDB`).
fn run_gdb_test(target: Target, profile: &str) {
    let pkg = match target {
        Target::Riscv => "qemu-riscv",
        Target::Cm3 => "qemu-cm3",
    };
    let el = build_example(pkg, pkg, target, profile);
    let gdb = env::var("RIVET_GDB").unwrap_or_else(|_| "gdb-multiarch".into());

    let mut args: Vec<String> = vec![];
    args.extend(match target {
        Target::Riscv => vec![
            "-machine".into(),
            "virt".into(),
            "-cpu".into(),
            "rv32".into(),
            "-bios".into(),
            "none".into(),
        ],
        Target::Cm3 => vec!["-machine".into(), "lm3s6965evb".into()],
    });
    args.push("-kernel".into());
    args.push(el.to_str().unwrap().into());
    args.push("-nographic".into());
    args.push("-serial".into());
    args.push("mon:stdio".into());
    args.push("-s".into()); // gdbstub on :1234
    args.push("-S".into()); // halt at reset until gdb continues

    eprintln!(
        "[xtask] starting QEMU with gdbstub for {}/ctx_switch",
        target_name(target)
    );
    let mut qemu = Command::new(target.qemu_bin())
        .args(&args)
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .spawn()
        .expect("failed to spawn qemu for gdb test");
    thread::sleep(Duration::from_secs(1));

    let script = workspace_root().join("tests/gdb/ctx_switch.py");
    let status = Command::new(&gdb)
        .args(["-batch", "-x"])
        .arg(&script)
        .arg(&el)
        .status()
        .unwrap_or_else(|e| {
            panic!("failed to run {gdb}: {e} (install gdb-multiarch or set $RIVET_GDB)")
        });

    let _ = qemu.kill();
    let _ = qemu.wait();

    assert!(
        status.success(),
        "{}/ctx_switch: GDB verification FAILED (exit {})",
        target_name(target),
        status.code().unwrap_or(-1)
    );
    eprintln!("[xtask] PASS {}/gdb: ctx_switch", target_name(target));
}

fn target_name(t: Target) -> &'static str {
    match t {
        Target::Riscv => "riscv",
        Target::Cm3 => "cm3",
    }
}

// ── CLI ────────────────────────────────────────────────────────────

fn usage() -> ! {
    eprintln!(
        "usage: cargo xtask <test|soak|list> --target <riscv|cm3> [--suite <smoke|stress|gdb>] \
         [--profile <release|release-checked>] [--icount N]"
    );
    std::process::exit(2);
}

fn main() {
    let args: Vec<String> = env::args().skip(1).collect();
    if args.is_empty() {
        usage();
    }
    let cmd = args[0].as_str();
    let mut target: Option<Target> = None;
    let mut suite = "smoke".to_string();
    let mut profile = "release".to_string();
    let mut icount: Option<u32> = None;
    let mut sim_hours: Option<u64> = None;
    let mut only: Option<String> = None;

    let mut i = 1;
    while i < args.len() {
        match args[i].as_str() {
            "--target" => {
                i += 1;
                target = Some(Target::from_str(&args[i]).unwrap_or_else(|e| {
                    eprintln!("{e}");
                    std::process::exit(2);
                }));
            }
            "--suite" => {
                i += 1;
                suite = args[i].clone();
            }
            "--only" => {
                i += 1;
                only = Some(args[i].clone());
            }
            "--profile" => {
                i += 1;
                profile = args[i].clone();
            }
            "--icount" => {
                i += 1;
                icount = Some(args[i].parse().expect("--icount must be an integer shift"));
            }
            "--sim-hours" => {
                i += 1;
                sim_hours = Some(args[i].parse().expect("--sim-hours must be an integer"));
            }
            other => {
                eprintln!("unknown argument `{other}`");
                usage();
            }
        }
        i += 1;
    }

    let target = target.unwrap_or_else(|| {
        eprintln!("--target <riscv|cm3> is required");
        usage();
    });

    match cmd {
        "list" => {
            for tc in smoke_tests(target) {
                println!("{}", tc.name);
            }
        }
        "capture" => {
            let out_dir = workspace_root().join("tests").join("golden");
            for tc in smoke_tests(target) {
                capture_test_case(target, &tc, &profile, &out_dir);
            }
        }
        "test" => {
            if suite == "gdb" {
                run_gdb_test(target, &profile);
                eprintln!("[xtask] {}/gdb: 1 test(s) passed", target_name(target));
                return;
            }

            let mut cases: Vec<TestCase> = match suite.as_str() {
                "smoke" => smoke_tests(target),
                // stress suite is populated by Phase 4's stress binaries.
                other => {
                    eprintln!(
                        "[xtask] suite `{other}` has no tests defined for {} yet",
                        target_name(target)
                    );
                    Vec::new()
                }
            };
            if let Some(filter) = &only {
                cases.retain(|tc| tc.name.contains(filter.as_str()));
            }
            for mut tc in cases.clone() {
                if let Some(shift) = icount {
                    tc.icount = Some(shift);
                }
                run_test_case(target, &tc, &profile);
            }
            eprintln!(
                "[xtask] {}/{}: {} test(s) passed",
                target_name(target),
                suite,
                cases.len()
            );
        }
        "soak" => {
            let hours = sim_hours.unwrap_or_else(|| {
                eprintln!("--sim-hours N is required for soak");
                usage();
            });
            eprintln!(
                "[xtask] soak for {} not implemented until Phase 8 (requested {}h sim)",
                target_name(target),
                hours
            );
        }
        _ => usage(),
    }
}
