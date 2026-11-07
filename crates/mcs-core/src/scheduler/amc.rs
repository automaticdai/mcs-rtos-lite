use mcs_arch::Ticks;

use crate::scheduler::Scheduler;
use crate::task::TaskControlBlock;
use crate::types::{CritLevel, DegradePolicy, Priority, TaskId, TaskSpec, TaskState};

/// Per-task parameters for Adaptive Mixed-Criticality (AMC) fixed-priority scheduling.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct AmcParams<const L: usize> {
    /// Static priority assigned to the task in each criticality mode `0..L`.
    /// Commonly identical across modes, or customized per mode via Audsley's OPA.
    pub priorities: [Priority; L],
    /// Optional low background priority used when `system_mode > task.criticality`
    /// and the task uses `DegradePolicy::BackgroundSlack`.
    pub slack_priority: Option<Priority>,
}

impl<const L: usize> AmcParams<L> {
    /// Creates `AmcParams` with a uniform fixed priority across all nominal modes `0..L`.
    pub const fn uniform(priority: Priority) -> Self {
        Self {
            priorities: [priority; L],
            slack_priority: None,
        }
    }

    /// Creates `AmcParams` with a uniform fixed priority and a background slack priority for degraded modes.
    pub const fn uniform_with_slack(priority: Priority, slack_priority: Priority) -> Self {
        Self {
            priorities: [priority; L],
            slack_priority: Some(slack_priority),
        }
    }

    /// Creates `AmcParams` with mode-specific priorities (e.g. from per-mode Audsley OPA).
    pub const fn per_mode(priorities: [Priority; L], slack_priority: Option<Priority>) -> Self {
        Self {
            priorities,
            slack_priority,
        }
    }

    /// Returns the effective base priority of a task in `system_mode`, or `None` if shed.
    #[inline]
    pub const fn effective_priority(
        &self,
        task_crit: CritLevel,
        degrade_policy: DegradePolicy,
        system_mode: CritLevel,
    ) -> Option<Priority> {
        if system_mode.0 <= task_crit.0 {
            Some(self.priorities[system_mode.0])
        } else {
            match degrade_policy {
                DegradePolicy::BackgroundSlack => self.slack_priority,
                DegradePolicy::Abort | DegradePolicy::Suspend => None,
            }
        }
    }
}

/// $O(1)$ Bitmap Adaptive Mixed-Criticality (AMC) Scheduler.
///
/// Uses a 64-bit priority bitmap (`ready_bitmap`) where bit `p` is set whenever a task with
/// effective priority `p` is ready. On ARM Cortex-M and RISC-V with B-extension,
/// `leading_zeros()` compiles to a single `CLZ` instruction.
pub struct AmcScheduler<const L: usize, const N: usize> {
    /// Bit `p` is `1` if `prio_to_task[p]` holds a ready/running task.
    ready_bitmap: u64,
    /// Direct lookup table mapping priority `p` (`0..64`) to its ready `TaskId`.
    prio_to_task: [Option<TaskId>; 64],
    /// Bitmask of tasks (`0..N`) currently marked ready in the scheduler.
    ready_tasks_mask: u64,
}

impl<const L: usize, const N: usize> Default for AmcScheduler<L, N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const L: usize, const N: usize> AmcScheduler<L, N> {
    pub const fn new() -> Self {
        assert!(N <= 64, "AmcScheduler supports up to 64 static tasks");
        Self {
            ready_bitmap: 0,
            prio_to_task: [None; 64],
            ready_tasks_mask: 0,
        }
    }

    #[inline]
    fn task_effective_prio(
        tcb: &TaskControlBlock<L, AmcParams<L>>,
        mode: CritLevel,
    ) -> Option<Priority> {
        let base = tcb.sched_params.effective_priority(
            tcb.spec.criticality,
            tcb.spec.degrade_policy,
            mode,
        )?;
        match tcb.priority_boost {
            Some(boost) if boost.0 > base.0 => Some(boost),
            _ => Some(base),
        }
    }

    fn rebuild_bitmap(&mut self, tasks: &[TaskControlBlock<L, AmcParams<L>>; N], mode: CritLevel) {
        self.ready_bitmap = 0;
        self.prio_to_task = [None; 64];

        let mut i = 0;
        while i < N {
            if (self.ready_tasks_mask & (1u64 << i)) != 0 {
                let tcb = &tasks[i];
                if matches!(tcb.state, TaskState::Ready | TaskState::Running) {
                    if let Some(prio) = Self::task_effective_prio(tcb, mode) {
                        let p = prio.0 as usize;
                        self.ready_bitmap |= 1u64 << p;
                        self.prio_to_task[p] = Some(tcb.spec.id);
                    }
                }
            }
            i += 1;
        }
    }
}

impl<const L: usize, const N: usize> Scheduler<L, N> for AmcScheduler<L, N> {
    type TaskSchedParams = AmcParams<L>;

    fn init(&mut self, tasks: &[TaskControlBlock<L, Self::TaskSchedParams>; N], mode: CritLevel) {
        self.ready_tasks_mask = 0;
        for (i, tcb) in tasks.iter().enumerate() {
            if matches!(tcb.state, TaskState::Ready | TaskState::Running) {
                self.ready_tasks_mask |= 1u64 << i;
            }
        }
        self.rebuild_bitmap(tasks, mode);
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
        self.ready_tasks_mask |= 1u64 << idx;
        if let Some(prio) = Self::task_effective_prio(&tasks[idx], mode) {
            let p = prio.0 as usize;
            self.ready_bitmap |= 1u64 << p;
            self.prio_to_task[p] = Some(task_id);
        }
    }

    #[inline]
    fn on_task_unready(
        &mut self,
        task_id: TaskId,
        tasks: &[TaskControlBlock<L, Self::TaskSchedParams>; N],
        mode: CritLevel,
    ) {
        let idx = task_id.0;
        self.ready_tasks_mask &= !(1u64 << idx);
        if let Some(prio) = Self::task_effective_prio(&tasks[idx], mode) {
            let p = prio.0 as usize;
            if self.prio_to_task[p] == Some(task_id) {
                self.ready_bitmap &= !(1u64 << p);
                self.prio_to_task[p] = None;
            } else {
                self.rebuild_bitmap(tasks, mode);
            }
        } else {
            self.rebuild_bitmap(tasks, mode);
        }
    }

    fn on_mode_change(
        &mut self,
        tasks: &[TaskControlBlock<L, Self::TaskSchedParams>; N],
        _old_mode: CritLevel,
        new_mode: CritLevel,
        _now: Ticks,
    ) {
        self.rebuild_bitmap(tasks, new_mode);
    }

    #[inline]
    fn pick_next(
        &self,
        _tasks: &[TaskControlBlock<L, Self::TaskSchedParams>; N],
        _mode: CritLevel,
    ) -> Option<TaskId> {
        if self.ready_bitmap == 0 {
            return None;
        }
        // O(1) priority resolution using hardware CLZ instruction:
        let highest_prio = 63 - (self.ready_bitmap.leading_zeros() as usize);
        self.prio_to_task[highest_prio]
    }
}

/// Compile-time / offline **AMC-rtb** (Adaptive Mixed-Criticality Response-Time Bound) verifier
/// generalized across `L` criticality levels.
///
/// Verifies both:
/// 1. **Steady-state schedulability** in every criticality mode `m in 0..L`.
/// 2. **Mode-transition schedulability** from mode `m_lo` to `m_hi` (`m_lo < m_hi`), accounting for
///    interference from lower-criticality tasks (`crit <= m_lo`) executing up to their `C_j(m_lo)`
///    budget before the transition occurs, and higher-criticality tasks (`crit >= m_hi`) executing
///    up to their `C_k(m_hi)` budget.
pub const fn verify_amc_rtb<const L: usize, const N: usize>(
    specs: &[TaskSpec<L>; N],
    params: &[AmcParams<L>; N],
) -> bool {
    let mut i = 0;
    while i < N {
        if !specs[i].validate() {
            return false;
        }
        let crit_i = specs[i].criticality.0;

        // 1. Check steady-state response time R_i(m) <= D_i for each mode m <= crit_i
        let mut m = 0;
        while m <= crit_i {
            let prio_i = params[i].priorities[m].0;
            let mut r = specs[i].budgets[m];
            loop {
                let mut r_next = specs[i].budgets[m];
                let mut j = 0;
                while j < N {
                    if j != i
                        && specs[j].criticality.0 >= m
                        && params[j].priorities[m].0 >= prio_i
                    {
                        let jobs = r.div_ceil(specs[j].period);
                        r_next = r_next.saturating_add(jobs.saturating_mul(specs[j].budgets[m]));
                    }
                    j += 1;
                }
                if r_next > specs[i].deadline {
                    return false;
                }
                if r_next == r {
                    break;
                }
                r = r_next;
            }
            // Save steady-state response time at mode m as r_steady_m for transition checks
            let r_steady_m = r;

            // 2. Check mode-transition response time R_i*(m -> m_hi) for all m_hi in (m+1)..=crit_i
            let mut m_hi = m + 1;
            while m_hi <= crit_i {
                let prio_i_hi = params[i].priorities[m_hi].0;
                let mut r_trans = specs[i].budgets[m_hi];
                loop {
                    let mut r_next = specs[i].budgets[m_hi];
                    let mut j = 0;
                    while j < N {
                        if j != i {
                            let crit_j = specs[j].criticality.0;
                            if crit_j < m_hi && crit_j >= m && params[j].priorities[m].0 >= prio_i {
                                // Lower-criticality task j can only interfere during the [0, r_steady_m) window
                                let jobs = r_steady_m.div_ceil(specs[j].period);
                                r_next =
                                    r_next.saturating_add(jobs.saturating_mul(specs[j].budgets[m]));
                            } else if crit_j >= m_hi && params[j].priorities[m_hi].0 >= prio_i_hi {
                                // Higher-criticality task j interferes across the full transition window
                                let jobs = r_trans.div_ceil(specs[j].period);
                                r_next = r_next
                                    .saturating_add(jobs.saturating_mul(specs[j].budgets[m_hi]));
                            }
                        }
                        j += 1;
                    }
                    if r_next > specs[i].deadline {
                        return false;
                    }
                    if r_next == r_trans {
                        break;
                    }
                    r_trans = r_next;
                }
                m_hi += 1;
            }

            m += 1;
        }

        i += 1;
    }
    true
}
