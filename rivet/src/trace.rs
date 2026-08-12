//! Rivet Debugger wire protocol encoder — emits the binary trace frames
//! `rivet-debugger-app` (a separate, sibling project — see its own SRS/
//! PLAN at `../rivet-debugger`) decodes live over a UART.
//!
//! Mirrors `rivet-trace-protocol`'s frame layout (SYNC0/SYNC1, CRC-16/
//! CCITT-FALSE, `EventKind`/`Payload` encoding) by hand, not by sharing
//! a crate: that project lives in a separate repository, and depending
//! on it directly from this kernel crate would mean `rivet` no longer
//! builds standalone from a fresh clone — the same reasoning `rivet`
//! already applies to every board-specific crate (nothing outside this
//! repo, nothing MMIO-shaped, in the kernel itself). If the two drift,
//! `rivet-trace-protocol`'s own decoder is the ground truth; re-derive
//! this module's byte layout from `rivet-trace-protocol/src/frame.rs`
//! and `event.rs`, not from memory.
//!
//! Gated behind the `trace` feature (off by default, same discipline as
//! [`crate::latency`]): every call in this module is a no-op unless a
//! board crate both enables the feature and implements
//! [`crate::port::board::trace_write`]'s extern symbol.

#[cfg(feature = "trace")]
mod imp {
    pub const SYNC0: u8 = 0xA5;
    pub const SYNC1: u8 = 0x5A;
    pub const PROTOCOL_VERSION: u8 = 1;
    pub const NO_TASK: u32 = u32::MAX;
    pub const MAX_PAYLOAD: usize = 32;
    pub const FRAME_OVERHEAD: usize = 2 + 1 + 2 + 2 + 1 + 4 + 8 + 1 + 2;
    pub const MAX_FRAME: usize = FRAME_OVERHEAD + MAX_PAYLOAD;

    pub fn crc16(data: &[u8]) -> u16 {
        let mut crc: u16 = 0xFFFF;
        for &byte in data {
            crc ^= (byte as u16) << 8;
            for _ in 0..8 {
                if crc & 0x8000 != 0 {
                    crc = (crc << 1) ^ 0x1021;
                } else {
                    crc <<= 1;
                }
            }
        }
        crc
    }

    use core::sync::atomic::{AtomicU16, Ordering};
    static SEQ: AtomicU16 = AtomicU16::new(0);

    /// Writes one frame into `buf`, returns the number of bytes written.
    /// `payload.len()` must be `<= MAX_PAYLOAD`.
    pub fn encode(
        buf: &mut [u8; MAX_FRAME],
        kind: u16,
        core_id: u8,
        task_id: u32,
        timestamp: u64,
        payload: &[u8],
    ) -> usize {
        debug_assert!(payload.len() <= MAX_PAYLOAD);
        let seq = SEQ.fetch_add(1, Ordering::Relaxed);
        buf[0] = SYNC0;
        buf[1] = SYNC1;
        buf[2] = PROTOCOL_VERSION;
        buf[3..5].copy_from_slice(&seq.to_le_bytes());
        buf[5..7].copy_from_slice(&kind.to_le_bytes());
        buf[7] = core_id;
        buf[8..12].copy_from_slice(&task_id.to_le_bytes());
        buf[12..20].copy_from_slice(&timestamp.to_le_bytes());
        buf[20] = payload.len() as u8;
        buf[21..21 + payload.len()].copy_from_slice(payload);
        let crc_end = 21 + payload.len();
        let crc = crc16(&buf[0..crc_end]);
        buf[crc_end..crc_end + 2].copy_from_slice(&crc.to_le_bytes());
        crc_end + 2
    }

    pub fn now_ts() -> u64 {
        crate::port::board::now_us()
    }

    pub fn core_id() -> u8 {
        crate::port::arch::hart_id() as u8
    }
}

#[cfg(feature = "trace")]
use imp::*;

// EventKind discriminants actually emitted from this module — a subset
// of `rivet-trace-protocol::EventKind`'s full ~48, matching what this
// kernel currently has real hook points for (`docs/DOCUMENTATION.md`
// §18 lists broadening this as a follow-up, not a gap hidden here).
#[cfg(feature = "trace")]
mod kind {
    pub const STREAM_HEADER: u16 = 0x0701;
    pub const TASK_CREATED: u16 = 0x0001;
    pub const CONTEXT_SWITCH: u16 = 0x0009;
    pub const IRQ_ENTER: u16 = 0x0301;
    pub const IRQ_EXIT: u16 = 0x0302;
    pub const MUTEX_LOCK_ACQUIRED: u16 = 0x0102;
    pub const MUTEX_UNLOCK: u16 = 0x0104;
    pub const PRIORITY_INHERIT: u16 = 0x0105;
    pub const HARD_FAULT: u16 = 0x0601;
    pub const STACK_OVERFLOW: u16 = 0x0605;
}

/// Why a context switch happened, matching
/// `rivet-trace-protocol::SwitchReason`'s wire encoding.
#[cfg(feature = "trace")]
#[derive(Clone, Copy)]
pub enum SwitchReason {
    Preempted = 0,
    TimerWake = 6,
}

// Cached so a late-joining client can be caught up (see
// `reannounce_stream_header`) — the header is otherwise only ever sent
// once, at boot, same gap `reannounce_all_tasks` closes for task info.
#[cfg(feature = "trace")]
static CACHED_CPU_HZ: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
#[cfg(feature = "trace")]
static CACHED_MAX_HARTS: core::sync::atomic::AtomicU8 = core::sync::atomic::AtomicU8::new(0);

/// Sent once, first frame of a session: SRS §19's "target information —
/// Rivet version, CPU frequency." Call as early as
/// [`crate::port::board::trace_write`] is usable.
#[cfg(feature = "trace")]
pub fn stream_header(cpu_hz: u32, max_harts: u8) {
    use core::sync::atomic::Ordering;
    CACHED_CPU_HZ.store(cpu_hz, Ordering::Relaxed);
    CACHED_MAX_HARTS.store(max_harts, Ordering::Relaxed);
    let mut payload = [0u8; 8];
    payload[0] = 0; // rivet_version.major — this workspace ships 0.x
    payload[1] = 1;
    payload[2] = 0;
    payload[3..7].copy_from_slice(&cpu_hz.to_le_bytes());
    payload[7] = max_harts;
    let mut buf = [0u8; MAX_FRAME];
    let n = encode(&mut buf, kind::STREAM_HEADER, core_id(), NO_TASK, now_ts(), &payload);
    crate::port::board::trace_write(&buf[..n]);
}

/// Re-sends `StreamHeader` from the values cached by the last real
/// [`stream_header`] call. A client that connects even slightly after
/// boot (unavoidable given real server/browser startup latency) misses
/// the one-shot boot frame and shows "unknown clock" forever — same gap,
/// same fix shape as [`reannounce_all_tasks`]. No-op if `stream_header`
/// was never called (cached cpu_hz still 0).
#[cfg(feature = "trace")]
pub fn reannounce_stream_header() {
    use core::sync::atomic::Ordering;
    let cpu_hz = CACHED_CPU_HZ.load(Ordering::Relaxed);
    if cpu_hz == 0 {
        return;
    }
    stream_header(cpu_hz, CACHED_MAX_HARTS.load(Ordering::Relaxed));
}

/// A task was just registered with the scheduler — sent once, right as
/// [`crate::preempt::spawn`] commits the new task's slot, so the host UI
/// has the task's real priority before its first `ContextSwitch` ever
/// arrives.
#[cfg(feature = "trace")]
pub fn task_created(task_id: u16, priority: u8, stack_size: u32) {
    let mut payload = [0u8; 5];
    payload[0] = priority;
    payload[1..5].copy_from_slice(&stack_size.to_le_bytes());
    let mut buf = [0u8; MAX_FRAME];
    let n = encode(
        &mut buf,
        kind::TASK_CREATED,
        core_id(),
        task_id as u32,
        now_ts(),
        &payload,
    );
    crate::port::board::trace_write(&buf[..n]);
}

/// A real scheduler dispatch: `prev_task` left the CPU (for `reason`),
/// `next_task` is now running. Called from the scheduler's actual
/// dispatch commit point, not synthesized from polling.
#[cfg(feature = "trace")]
pub fn context_switch(prev_task: u16, next_task: u16, reason: SwitchReason) {
    let mut payload = [0u8; 9];
    payload[0..4].copy_from_slice(&(prev_task as u32).to_le_bytes());
    payload[4..8].copy_from_slice(&(next_task as u32).to_le_bytes());
    payload[8] = reason as u8;
    let mut buf = [0u8; MAX_FRAME];
    let n = encode(
        &mut buf,
        kind::CONTEXT_SWITCH,
        core_id(),
        next_task as u32,
        now_ts(),
        &payload,
    );
    crate::port::board::trace_write(&buf[..n]);
}

/// An interrupt was entered (`entry = true`) or exited.
#[cfg(feature = "trace")]
pub fn isr(irq: u32, entry: bool) {
    let mut payload = [0u8; 4];
    payload.copy_from_slice(&irq.to_le_bytes());
    let mut buf = [0u8; MAX_FRAME];
    let k = if entry { kind::IRQ_ENTER } else { kind::IRQ_EXIT };
    let n = encode(&mut buf, k, core_id(), NO_TASK, now_ts(), &payload);
    crate::port::board::trace_write(&buf[..n]);
}

/// A `PriorityMutex` was acquired by `task_id`.
#[cfg(feature = "trace")]
pub fn mutex_lock_acquired(task_id: u16, mutex_id: u32) {
    let mut payload = [0u8; 4];
    payload.copy_from_slice(&mutex_id.to_le_bytes());
    let mut buf = [0u8; MAX_FRAME];
    let n = encode(
        &mut buf,
        kind::MUTEX_LOCK_ACQUIRED,
        core_id(),
        task_id as u32,
        now_ts(),
        &payload,
    );
    crate::port::board::trace_write(&buf[..n]);
}

/// A `PriorityMutex` was released by `task_id`.
#[cfg(feature = "trace")]
pub fn mutex_unlock(task_id: u16, mutex_id: u32) {
    let mut payload = [0u8; 4];
    payload.copy_from_slice(&mutex_id.to_le_bytes());
    let mut buf = [0u8; MAX_FRAME];
    let n = encode(&mut buf, kind::MUTEX_UNLOCK, core_id(), task_id as u32, now_ts(), &payload);
    crate::port::board::trace_write(&buf[..n]);
}

/// `task_id`'s effective priority was boosted by priority inheritance
/// while a higher-priority task waited on a mutex it holds.
#[cfg(feature = "trace")]
pub fn priority_inherit(task_id: u16, mutex_id: u32) {
    let mut payload = [0u8; 4];
    payload.copy_from_slice(&mutex_id.to_le_bytes());
    let mut buf = [0u8; MAX_FRAME];
    let n = encode(
        &mut buf,
        kind::PRIORITY_INHERIT,
        core_id(),
        task_id as u32,
        now_ts(),
        &payload,
    );
    crate::port::board::trace_write(&buf[..n]);
}

/// A task faulted. `reason` mirrors `crate::fault::FaultKind`'s
/// discriminant (0=InstructionAccess, 1=LoadAccess, 2=StoreAccess,
/// 3=MemManage, 4=StackOverflow, 5=BudgetExceeded).
#[cfg(feature = "trace")]
pub fn fault(task_id: u16, reason: u8, pc: u32) {
    let mut payload = [0u8; 8];
    payload[0..4].copy_from_slice(&(reason as u32).to_le_bytes());
    payload[4..8].copy_from_slice(&pc.to_le_bytes());
    let mut buf = [0u8; MAX_FRAME];
    let k = if reason == 4 { kind::STACK_OVERFLOW } else { kind::HARD_FAULT };
    let n = encode(&mut buf, k, core_id(), task_id as u32, now_ts(), &payload);
    crate::port::board::trace_write(&buf[..n]);
}

/// Re-emits `TaskCreated` for every currently-registered task.
/// [`task_created`] fires once, at spawn time — a debugger app that
/// connects even a moment later (or drops and reconnects) never sees
/// it, and every task it knows about would show priority 0 forever
/// (a real gap found by actually reconnecting to a live board, not
/// theorized). Called periodically from the tick path
/// ([`crate::preempt::on_tick`], counter-gated so it costs one scan of
/// [`crate::preempt::tcb::MAX_PTASKS`] slots every couple of seconds,
/// not every tick) so a late-joining client has the true picture within
/// one interval, not just at boot.
#[cfg(feature = "trace")]
pub fn reannounce_all_tasks() {
    use crate::preempt::tcb;
    use core::sync::atomic::Ordering;
    for id in 0..tcb::MAX_PTASKS {
        let Some(t) = tcb::get(id) else { continue };
        if !t.used.load(Ordering::Acquire) {
            continue;
        }
        let priority = t.base_priority.load(Ordering::Acquire);
        let stack_size = t.stack_size.load(Ordering::Acquire) as u32;
        task_created(id as u16, priority, stack_size);
    }
}
