use mcs_arch::Ticks;

use crate::scheduler::Scheduler;
use crate::task::TaskControlBlock;
use crate::types::{CritLevel, DegradePolicy, TaskId, TaskState};

/// Per-task parameters for Earliest Deadline First with Virtual Deadlines (`EDF-VD`)
/// across `L` criticality levels.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct EdfVdParams<const L: usize> {
    /// Relative virtual deadline `D'_i(m)` for each system criticality mode `m in 0..L`.
    /// For modes `m < task.criticality`, `virtual_deadlines[m]` is scaled down (`D'_i(m) = x_m * D_i`)
    /// to reserve slack for mode transitions. At `m == task.criticality`, `virtual_deadlines[m] == D_i`.
    pub virtual_deadlines: [Ticks; L],
}

impl<const L: usize> EdfVdParams<L> {
    pub const fn new(virtual_deadlines: [Ticks; L]) -> Self {
        Self { virtual_deadlines }
    }

    /// Helper to construct `EdfVdParams` by scaling the nominal relative deadline `deadline`
    /// by `num / den` (`0 < num <= den`) for all modes strictly below `task_crit`.
    pub const fn from_scale(
        task_crit: CritLevel,
        deadline: Ticks,
        scale_num: u64,
        scale_den: u64,
    ) -> Self {
        assert!(scale_num > 0 && scale_num <= scale_den);
        let scaled = (deadline * scale_num) / scale_den;
        let scaled = if scaled == 0 { 1 } else { scaled };
        let mut vds = [deadline; L];
        let mut m = 0;
        while m < task_crit.0 && m < L {
            vds[m] = scaled;
            m += 1;
        }
        Self {
            virtual_deadlines: vds,
        }
    }
}

/// Multi-level Earliest Deadline First with Virtual Deadlines (`EDF-VD`) Scheduler.
///
/// In any criticality mode `M`:
/// 1. Eligible tasks with `criticality >= M` compete by earliest absolute virtual deadline
///    `d_i(M) = current_job_release + virtual_deadlines[M]`.
/// 2. When `M` escalates to `M'`, active jobs of criticality `>= M'` automatically have their
///    effective absolute deadlines relaxed from `d_i(M)` to `d_i(M')`.
/// 3. Degraded tasks with `criticality < M` and `DegradePolicy::BackgroundSlack` only execute
///    when no task with `criticality >= M` is ready.
pub struct EdfVdScheduler<const L: usize, const N: usize> {
    /// Bitmask of tasks (`0..N`) currently in the ready queue.
    ready_mask: u64,
    /// Cached absolute virtual deadline for each task in the current mode.
    abs_deadlines: [Ticks; N],
}

impl<const L: usize, const N: usize> Default for EdfVdScheduler<L, N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const L: usize, const N: usize> EdfVdScheduler<L, N> {
    pub const fn new() -> Self {
        assert!(N <= 64, "EdfVdScheduler supports up to 64 static tasks");
        Self {
            ready_mask: 0,
            abs_deadlines: [u64::MAX; N],
        }
    }

    #[inline]
    fn compute_abs_deadline(tcb: &TaskControlBlock<L, EdfVdParams<L>>, mode: CritLevel) -> Ticks {
        let m = if mode.0 < L { mode.0 } else { L - 1 };
        tcb.current_job_release
            .saturating_add(tcb.sched_params.virtual_deadlines[m])
    }

    #[inline]
    fn is_task_eligible(tcb: &TaskControlBlock<L, EdfVdParams<L>>, mode: CritLevel) -> bool {
        if !matches!(tcb.state, TaskState::Ready | TaskState::Running) {
            return false;
        }
        if tcb.spec.criticality.0 >= mode.0 {
            true
        } else {
            matches!(tcb.spec.degrade_policy, DegradePolicy::BackgroundSlack)
        }
    }
}

impl<const L: usize, const N: usize> Scheduler<L, N> for EdfVdScheduler<L, N> {
    type TaskSchedParams = EdfVdParams<L>;

    fn init(&mut self, tasks: &[TaskControlBlock<L, Self::TaskSchedParams>; N], mode: CritLevel) {
        self.ready_mask = 0;
        for (i, tcb) in tasks.iter().enumerate() {
            self.abs_deadlines[i] = Self::compute_abs_deadline(tcb, mode);
            if Self::is_task_eligible(tcb, mode) {
                self.ready_mask |= 1u64 << i;
            }
        }
    }

    #[inline]
    fn on_task_ready(
        &mut self,
        task_id: TaskId,
        tasks: &[TaskControlBlock<L, Self::TaskSchedParams>; N],
        mode: CritLevel,
        _now: Ticks,
    ) {
        let idx = task_id.0;
        let tcb = &tasks[idx];
        self.abs_deadlines[idx] = Self::compute_abs_deadline(tcb, mode);
        if Self::is_task_eligible(tcb, mode) {
            self.ready_mask |= 1u64 << idx;
        }
    }

    #[inline]
    fn on_task_unready(
        &mut self,
        task_id: TaskId,
        _tasks: &[TaskControlBlock<L, Self::TaskSchedParams>; N],
        _mode: CritLevel,
    ) {
        self.ready_mask &= !(1u64 << task_id.0);
    }

    fn on_mode_change(
        &mut self,
        tasks: &[TaskControlBlock<L, Self::TaskSchedParams>; N],
        _old_mode: CritLevel,
        new_mode: CritLevel,
        _now: Ticks,
    ) {
        self.ready_mask = 0;
        let mut i = 0;
        while i < N {
            let tcb = &tasks[i];
            self.abs_deadlines[i] = Self::compute_abs_deadline(tcb, new_mode);
            if Self::is_task_eligible(tcb, new_mode) {
                self.ready_mask |= 1u64 << i;
            }
            i += 1;
        }
    }

    fn pick_next(
        &self,
        tasks: &[TaskControlBlock<L, Self::TaskSchedParams>; N],
        mode: CritLevel,
    ) -> Option<TaskId> {
        if self.ready_mask == 0 {
            return None;
        }

        // Primary pass: tasks with criticality >= system_mode ordered by earliest virtual deadline
        // (Tie-breaker: higher criticality first, then lower TaskId)
        let mut best_primary: Option<(TaskId, Ticks, CritLevel)> = None;
        // Secondary pass: background slack tasks (criticality < system_mode with BackgroundSlack policy)
        let mut best_slack: Option<(TaskId, Ticks, CritLevel)> = None;

        let mut mask = self.ready_mask;
        while mask != 0 {
            let idx = mask.trailing_zeros() as usize;
            mask &= mask - 1;

            let tcb = &tasks[idx];
            if !matches!(tcb.state, TaskState::Ready | TaskState::Running) {
                continue;
            }
            let dl = self.abs_deadlines[idx];
            let crit = tcb.spec.criticality;

            if crit.0 >= mode.0 {
                match best_primary {
                    None => best_primary = Some((tcb.spec.id, dl, crit)),
                    Some((_, best_dl, best_crit)) => {
                        if dl < best_dl || (dl == best_dl && crit > best_crit) {
                            best_primary = Some((tcb.spec.id, dl, crit));
                        }
                    }
                }
            } else if matches!(tcb.spec.degrade_policy, DegradePolicy::BackgroundSlack) {
                match best_slack {
                    None => best_slack = Some((tcb.spec.id, dl, crit)),
                    Some((_, best_dl, best_crit)) => {
                        if dl < best_dl || (dl == best_dl && crit > best_crit) {
                            best_slack = Some((tcb.spec.id, dl, crit));
                        }
                    }
                }
            }
        }

        best_primary
            .map(|(id, _, _)| id)
            .or_else(|| best_slack.map(|(id, _, _)| id))
    }
}
