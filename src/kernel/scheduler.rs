//! Scheduler: ready queue, current task, block/unblock, schedule.

use crate::kernel::task::{Tcb, TaskState, MAX_TASKS};

/// Context switch function type: (prev_sp_ptr, next_sp).
/// Set by the port at init; called when blocking and switching to another task.
pub type ContextSwitchFn = unsafe fn(prev_sp: *mut usize, next_sp: usize);

static mut TASKS: [Option<Tcb>; MAX_TASKS] = [None; MAX_TASKS];
static mut CURRENT: Option<usize> = None;
static mut CONTEXT_SWITCH: Option<ContextSwitchFn> = None;

/// Register a task. Call with the task's initial stack pointer (top of stack,
/// with saved context already set up so that restoring returns to task entry).
/// Returns true if the slot was free and the task was registered.
pub fn register_task(id: usize, sp: usize, priority: u8) -> bool {
    if id >= MAX_TASKS {
        return false;
    }
    unsafe {
        if TASKS[id].is_some() {
            return false;
        }
        TASKS[id] = Some(Tcb::new(sp, priority));
        true
    }
}

/// Set the context switch function (called by the port at init).
/// For host tests, set to a no-op or stub.
pub fn set_context_switch(f: ContextSwitchFn) {
    unsafe {
        CONTEXT_SWITCH = Some(f);
    }
}

/// Initialize scheduler state. Clears all tasks and current.
pub fn init() {
    unsafe {
        CURRENT = None;
        for i in 0..MAX_TASKS {
            TASKS[i] = None;
        }
    }
}

/// Current task id, if any.
pub fn get_current() -> Option<usize> {
    unsafe { CURRENT }
}

/// Set current task (used after a context switch).
pub fn set_current(id: usize) {
    unsafe {
        if id < MAX_TASKS {
            if let Some(ref mut t) = TASKS[id] {
                t.state = TaskState::Running;
            }
            CURRENT = Some(id);
        }
    }
}

/// Stack pointer of the current task.
pub fn get_current_sp() -> Option<usize> {
    unsafe {
        let id = CURRENT?;
        TASKS[id].as_ref().map(|t| t.sp)
    }
}

/// Stack pointer of a task by id.
pub fn get_task_sp(id: usize) -> Option<usize> {
    unsafe {
        if id >= MAX_TASKS {
            return None;
        }
        TASKS[id].as_ref().map(|t| t.sp)
    }
}

/// Update current task's saved sp (after it was saved by context switch).
pub fn set_current_sp(sp: usize) {
    unsafe {
        if let Some(id) = CURRENT {
            if id < MAX_TASKS {
                if let Some(ref mut t) = TASKS[id] {
                    t.sp = sp;
                }
            }
        }
    }
}

/// Mark the current task blocked. Does not switch; caller must call schedule and context_switch.
pub fn block_current() {
    unsafe {
        if let Some(id) = CURRENT {
            if id < MAX_TASKS {
                if let Some(ref mut t) = TASKS[id] {
                    t.state = TaskState::Blocked;
                }
            }
        }
    }
}

/// Mark task `id` ready.
pub fn unblock(id: usize) {
    unsafe {
        if id < MAX_TASKS {
            if let Some(ref mut t) = TASKS[id] {
                t.state = TaskState::Ready;
            }
        }
    }
}

/// Task state (for tests).
pub fn get_task_state(id: usize) -> Option<TaskState> {
    unsafe {
        if id >= MAX_TASKS {
            return None;
        }
        TASKS[id].as_ref().map(|t| t.state)
    }
}

/// Select next task to run (round-robin over Ready tasks).
/// Starts from current + 1 (or 0 if no current). Does not change current.
pub fn schedule() -> Option<usize> {
    unsafe {
        let start = CURRENT.map(|c| (c + 1) % MAX_TASKS).unwrap_or(0);
        for i in 0..MAX_TASKS {
            let id = (start + i) % MAX_TASKS;
            if let Some(ref t) = TASKS[id] {
                if t.state == TaskState::Ready {
                    return Some(id);
                }
            }
        }
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn register_then_schedule() {
        init();
        assert!(register_task(0, 0x1000, 1));
        assert!(register_task(1, 0x2000, 1));
        assert_eq!(get_task_state(0), Some(TaskState::Ready));
        assert_eq!(get_task_state(1), Some(TaskState::Ready));
        assert_eq!(schedule(), Some(0));
    }

    #[test]
    fn init_clears_state() {
        init();
        assert_eq!(get_current(), None);
        assert_eq!(get_task_state(0), None);
    }

    #[test]
    fn block_and_unblock() {
        init();
        assert!(register_task(0, 0x1000, 1));
        assert!(register_task(1, 0x2000, 1));
        set_current(0);
        block_current();
        assert_eq!(get_task_state(0), Some(TaskState::Blocked));
        assert_eq!(schedule(), Some(1));
        unblock(0);
        assert_eq!(get_task_state(0), Some(TaskState::Ready));
        set_current(1);
        assert_eq!(schedule(), Some(0));
    }
}

/// Block current task and switch to the next ready task.
/// Call only when at least one other task is ready (e.g. idle).
/// Updates current task's sp via context_switch, then sets current to next.
pub fn block_current_and_switch() {
    let cur = get_current();
    let next_id = schedule();

    let (cur_id, next_id) = match (cur, next_id) {
        (Some(c), Some(n)) if c != n => (c, n),
        _ => return,
    };

    block_current();

    let next_sp = match get_task_sp(next_id) {
        Some(s) => s,
        None => return,
    };

    // Set current to next task BEFORE context_switch. context_switch suspends
    // this task, so anything after it only runs when we're resumed later.
    // If we set_current after, CURRENT stays wrong for the entire duration the
    // next task is running, breaking any scheduling the next task performs.
    unsafe {
        let slot = &mut TASKS[cur_id];
        if let Some(ref mut tcb) = slot {
            let prev_sp_ptr = &mut tcb.sp as *mut usize;
            set_current(next_id);
            if let Some(sw) = CONTEXT_SWITCH {
                sw(prev_sp_ptr, next_sp);
            }
            // Execution resumes here when someone switches back to us.
            // Restore our own id as current.
            set_current(cur_id);
        }
    }
}
