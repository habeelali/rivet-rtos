//! Core kernel: scheduler, tasks, synchronization.

pub mod scheduler;
pub mod semaphore;
pub mod task;

pub use scheduler::{
    block_current, block_current_and_switch, get_current, get_current_sp, get_task_sp,
    get_task_state, init as scheduler_init, register_task, schedule, set_context_switch,
    set_current, set_current_sp, unblock, ContextSwitchFn,
};
pub use semaphore::BinarySemaphore;
pub use task::{TaskState, Tcb, MAX_TASKS};
