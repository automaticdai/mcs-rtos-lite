use mcs_arch::Ticks;

use crate::types::{CritLevel, Priority, TaskId, TaskSpec, TaskState};

/// Runtime control block for a traditional stateful thread.
#[derive(Clone, Debug)]
pub struct TaskControlBlock<const L: usize, P> {
    /// Immutable mixed-criticality specification.
    pub spec: TaskSpec<L>,
    /// Scheduler-specific parameters (e.g., `AmcParams<L>` or `EdfVdParams<L>`).
    pub sched_params: P,
    /// Current lifecycle state of the task.
    pub state: TaskState,
    /// Saved Process Stack Pointer (`PSP` on Cortex-M) for context switching.
    pub sp: usize,
    /// Initial stack pointer used when resetting an aborted job's stack frame.
    pub initial_sp: usize,
    /// Accumulated execution time of the currently active job in hardware ticks.
    pub exec_time: Ticks,
    /// Timestamp when the task was most recently switched into `Running` state.
    pub last_dispatched_at: Ticks,
    /// Release timestamp of the current job.
    pub current_job_release: Ticks,
    /// Timestamp when the next periodic job is due to arrive.
    pub next_release: Ticks,
    /// Dynamic priority ceiling boost when holding an IPCP resource lock.
    pub priority_boost: Option<Priority>,
    /// Total jobs completed within budget since boot.
    pub completed_jobs: u64,
    /// Number of times this task has exhausted its active mode budget.
    pub overrun_count: u32,
}

impl<const L: usize, P: Copy> TaskControlBlock<L, P> {
    /// Creates a new TCB in the `Dormant` state, ready for initial release at tick `0`.
    pub const fn new(spec: TaskSpec<L>, sched_params: P, initial_sp: usize) -> Self {
        assert!(spec.validate(), "Invalid mixed-criticality TaskSpec");
        Self {
            spec,
            sched_params,
            state: TaskState::Dormant,
            sp: initial_sp,
            initial_sp,
            exec_time: 0,
            last_dispatched_at: 0,
            current_job_release: 0,
            next_release: 0,
            priority_boost: None,
            completed_jobs: 0,
            overrun_count: 0,
        }
    }

    #[inline]
    pub const fn id(&self) -> TaskId {
        self.spec.id
    }

    #[inline]
    pub const fn criticality(&self) -> CritLevel {
        self.spec.criticality
    }

    /// Returns the remaining execution budget for the active job in `system_mode`.
    #[inline]
    pub const fn remaining_budget(&self, system_mode: CritLevel) -> Ticks {
        let budget = self.spec.budget_for_mode(system_mode);
        budget.saturating_sub(self.exec_time)
    }

    /// Returns true if the task has an unfinished job in progress (`Ready`, `Running`, or `Blocked`).
    #[inline]
    pub const fn is_job_active(&self) -> bool {
        matches!(
            self.state,
            TaskState::Ready | TaskState::Running | TaskState::Blocked
        )
    }
}
