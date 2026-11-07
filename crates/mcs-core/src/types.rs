use mcs_arch::{StackBounds, Ticks};

/// Criticality level index in `0..L` (`0` = lowest criticality, `L - 1` = highest criticality).
#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct CritLevel(pub usize);

impl CritLevel {
    pub const LOWEST: Self = Self(0);

    #[inline]
    pub const fn as_usize(self) -> usize {
        self.0
    }
}

/// Identifier of a statically allocated task in `0..N`.
#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct TaskId(pub usize);

impl TaskId {
    #[inline]
    pub const fn as_usize(self) -> usize {
        self.0
    }
}

/// Fixed priority level in `0..64` (`63` = highest urgency, `0` = lowest urgency).
/// Mapped directly to a 64-bit hardware CLZ bitmap in `AmcScheduler`.
#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Priority(pub u8);

impl Priority {
    pub const MIN: Self = Self(0);
    pub const MAX: Self = Self(63);

    #[inline]
    pub const fn new(val: u8) -> Self {
        assert!(val < 64, "Priority must be in 0..64 for 64-bit CLZ bitmap");
        Self(val)
    }
}

/// Policy applied to a task when the system criticality mode `M` exceeds the task's criticality `L_i`.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum DegradePolicy {
    /// Classic Vestal policy: immediately abort any active job and suppress job arrivals while `M > L_i`.
    Abort,
    /// Preserve the thread's register/stack state across the higher-criticality excursion and resume
    /// once the system recovers to `M <= L_i`.
    Suspend,
    /// Demote the task to a low background slack priority band so it can still progress if no
    /// higher-criticality tasks are ready.
    BackgroundSlack,
}

/// Execution state of a stateful thread.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum TaskState {
    /// Waiting for its next periodic/sporadic release.
    Dormant,
    /// Eligible to be scheduled in the current criticality mode.
    Ready,
    /// Currently executing on the processor.
    Running,
    /// Blocked on a same-criticality synchronization primitive.
    Blocked,
    /// Frozen because `system_mode > task.criticality` under `DegradePolicy::Suspend`.
    SuspendedByMode,
    /// Aborted for the remainder of the current job period due to budget overrun or mode shedding.
    Aborted,
}

/// Static specification of a mixed-criticality task across `L` criticality levels.
#[derive(Copy, Clone, Debug)]
pub struct TaskSpec<const L: usize> {
    pub id: TaskId,
    /// Assigned criticality partition (`0..L`).
    pub criticality: CritLevel,
    /// Minimum inter-arrival time / period in hardware ticks.
    pub period: Ticks,
    /// Relative deadline in hardware ticks (`<= period` for constrained-deadline tasks).
    pub deadline: Ticks,
    /// Non-decreasing WCET budget vector across system criticality modes `0..L`.
    /// For modes `m <= criticality.0`, `budgets[m]` is the enforced execution budget.
    /// For modes `m > criticality.0`, `budgets[m]` is only used if `DegradePolicy::BackgroundSlack` is chosen.
    pub budgets: [Ticks; L],
    /// Behavior when `system_mode > criticality`.
    pub degrade_policy: DegradePolicy,
    /// Memory bounds of the thread's dedicated stack.
    pub stack_bounds: StackBounds,
}

impl<const L: usize> TaskSpec<L> {
    /// Validates that the task specification obeys Vestal's mixed-criticality invariants.
    pub const fn validate(&self) -> bool {
        if L == 0 || self.criticality.0 >= L {
            return false;
        }
        if self.period == 0 || self.deadline == 0 {
            return false;
        }
        // Budgets up to the task's own criticality level must be positive and monotonically non-decreasing:
        // C_i(0) <= C_i(1) <= ... <= C_i(L_i)
        let mut m = 0;
        while m <= self.criticality.0 {
            if self.budgets[m] == 0 || self.budgets[m] > self.deadline {
                return false;
            }
            if m > 0 && self.budgets[m] < self.budgets[m - 1] {
                return false;
            }
            m += 1;
        }
        true
    }

    /// Returns the active WCET budget for this task in `system_mode`.
    #[inline]
    pub const fn budget_for_mode(&self, system_mode: CritLevel) -> Ticks {
        let idx = if system_mode.0 < L {
            system_mode.0
        } else {
            L - 1
        };
        self.budgets[idx]
    }
}
