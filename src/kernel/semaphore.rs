//! Binary semaphore: blocking wait and signal.

use crate::kernel::scheduler;

/// Binary semaphore (0 or 1). Call [`BinarySemaphore::wait`] / [`BinarySemaphore::signal`]
/// from within a critical section (or single-threaded test).
pub struct BinarySemaphore {
    /// 0 = taken, 1 = available.
    value: u8,
    /// Task id blocked on this semaphore, if any.
    blocked_task: Option<usize>,
}

impl BinarySemaphore {
    /// Create a semaphore that is initially taken (0).
    pub const fn new_taken() -> Self {
        Self {
            value: 0,
            blocked_task: None,
        }
    }

    /// Create a semaphore that is initially available (1).
    pub const fn new_available() -> Self {
        Self {
            value: 1,
            blocked_task: None,
        }
    }

    /// Wait (take). If value is 1, set to 0 and return. Otherwise block current task
    /// and switch away; when woken, take the semaphore (value becomes 0) and return.
    /// Caller must ensure critical section or single-threaded context.
    pub fn wait(&mut self) {
        if self.value == 1 {
            self.value = 0;
            return;
        }
        if let Some(cur) = scheduler::get_current() {
            self.blocked_task = Some(cur);
            scheduler::block_current_and_switch();
            // When we return we were woken by signal(); signal() already took blocked_task and unblocked us.
            // We now hold the token (value stays 0).
        }
    }

    /// Signal (give). If someone is blocked, unblock them. Otherwise set value to 1.
    /// Caller must ensure critical section or single-threaded context.
    pub fn signal(&mut self) {
        if let Some(id) = self.blocked_task.take() {
            scheduler::unblock(id);
        } else {
            self.value = 1;
        }
    }

    /// Value for tests (0 or 1).
    pub fn value(&self) -> u8 {
        self.value
    }
}
