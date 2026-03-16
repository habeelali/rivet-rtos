//! Task control block and task state.

/// Maximum number of tasks (static allocation).
pub const MAX_TASKS: usize = 4;

/// Task execution state.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TaskState {
    /// Task is ready to run.
    Ready,
    /// Task is blocked (e.g. waiting on a semaphore).
    Blocked,
    /// Task is currently running (only one at a time).
    Running,
}

/// Task control block: minimal per-task state.
#[derive(Clone, Copy, Debug)]
pub struct Tcb {
    pub state: TaskState,
    /// Stack pointer (saved/restored on context switch).
    pub sp: usize,
    /// Priority (higher = higher priority); 0 = lowest.
    pub priority: u8,
}

impl Tcb {
    pub const fn new(sp: usize, priority: u8) -> Self {
        Self {
            state: TaskState::Ready,
            sp,
            priority,
        }
    }
}
