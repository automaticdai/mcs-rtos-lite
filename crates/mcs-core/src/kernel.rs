use mcs_arch::{Arch, Ticks};

use crate::ipc::IpcpResourceSpec;
use crate::scheduler::Scheduler;
use crate::task::TaskControlBlock;
use crate::types::{CritLevel, DegradePolicy, Priority, TaskId, TaskState};

/// Telemetry and diagnostic counters maintained by the kernel.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub struct KernelStats {
    /// Total context switches between stateful threads.
    pub total_context_switches: u64,
    /// Context switches between tasks in the **same** criticality partition (MPU/PMP untouched).
    pub fast_same_crit_switches: u64,
    /// Context switches across **different** criticality partitions (MPU/PMP reprogrammed).
    pub cross_crit_mpu_switches: u64,
    /// Number of upward criticality mode escalations (`M -> M + 1`).
    pub mode_escalations: u64,
    /// Number of downward criticality recoveries to nominal mode (`M -> 0`) at an idle instant.
    pub idle_recoveries: u64,
    /// Total jobs aborted because they exceeded their maximum guaranteed budget `C_i(L_i)`.
    pub max_budget_aborts: u64,
}

/// High-level kernel event returned after handling a timer interrupt or syscall.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum KernelEvent {
    /// Normal scheduling step completed; holds the currently running task (or `None` if idle).
    Dispatched(Option<TaskId>),
    /// System criticality mode escalated from `old_mode` to `new_mode` due to `overrunning_task`.
    ModeEscalated {
        overrunning_task: TaskId,
        old_mode: CritLevel,
        new_mode: CritLevel,
    },
    /// System recovered to nominal mode (`CritLevel(0)`) at an idle instant.
    ModeRecovered { old_mode: CritLevel },
    /// A task overrun its maximum criticality budget `C_i(L_i)` and had its job aborted.
    JobAbortedOverrun { task: TaskId, mode: CritLevel },
}

/// Error returned by Immediate Priority Ceiling Protocol (IPCP) operations.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum IpcpError {
    NoRunningTask,
    CrossCriticalityViolation {
        task_crit: CritLevel,
        resource_crit: CritLevel,
    },
}

/// Core mixed-criticality RTOS kernel parameterized over:
/// - `L`: Number of criticality levels (`0..L`).
/// - `N`: Number of statically allocated tasks (`0..N`).
/// - `S`: Pluggable scheduling policy (`AmcScheduler<L, N>`, `EdfVdScheduler<L, N>`, etc.).
/// - `A`: Target hardware architecture backend (`Arch<L>`).
pub struct Kernel<const L: usize, const N: usize, S: Scheduler<L, N>, A: Arch<L>> {
    tasks: [TaskControlBlock<L, S::TaskSchedParams>; N],
    partitions: [A::PartitionConfig; L],
    scheduler: S,
    arch: A,
    system_mode: CritLevel,
    current_task: Option<TaskId>,
    /// Tracks which criticality partition's MPU/PMP configuration is currently loaded in hardware.
    active_partition_crit: Option<CritLevel>,
    stats: KernelStats,
}

impl<const L: usize, const N: usize, S: Scheduler<L, N>, A: Arch<L>> Kernel<L, N, S, A> {
    /// Constructs a new static `Kernel` instance in nominal mode (`CritLevel(0)`).
    pub const fn new(
        tasks: [TaskControlBlock<L, S::TaskSchedParams>; N],
        partitions: [A::PartitionConfig; L],
        scheduler: S,
        arch: A,
    ) -> Self {
        assert!(L > 0, "At least 1 criticality level is required");
        assert!(N > 0, "At least 1 task is required");
        Self {
            tasks,
            partitions,
            scheduler,
            arch,
            system_mode: CritLevel::LOWEST,
            current_task: None,
            active_partition_crit: None,
            stats: KernelStats {
                total_context_switches: 0,
                fast_same_crit_switches: 0,
                cross_crit_mpu_switches: 0,
                mode_escalations: 0,
                idle_recoveries: 0,
                max_budget_aborts: 0,
            },
        }
    }

    #[inline]
    pub const fn system_mode(&self) -> CritLevel {
        self.system_mode
    }

    #[inline]
    pub const fn current_task(&self) -> Option<TaskId> {
        self.current_task
    }

    #[inline]
    pub const fn active_partition_crit(&self) -> Option<CritLevel> {
        self.active_partition_crit
    }

    #[inline]
    pub const fn stats(&self) -> &KernelStats {
        &self.stats
    }

    #[inline]
    pub const fn tasks(&self) -> &[TaskControlBlock<L, S::TaskSchedParams>; N] {
        &self.tasks
    }

    #[inline]
    pub fn arch(&self) -> &A {
        &self.arch
    }

    #[inline]
    pub fn arch_mut(&mut self) -> &mut A {
        &mut self.arch
    }

    /// Initializes hardware protection tables, releases initial periodic jobs at `now`,
    /// and dispatches the highest-eligibility task.
    pub fn boot(&mut self) -> Option<TaskId> {
        self.arch.init_hardware(&self.partitions);
        let now = self.arch.now();
        self.scheduler.init(&self.tasks, self.system_mode);
        self.release_due_jobs(now);
        self.reschedule(now)
    }

    /// Primary interrupt entry point for the one-shot budget/release timer or system tick.
    ///
    /// 1. Charges elapsed execution time to `current_task`.
    /// 2. Checks whether `current_task` exhausted its active mode budget `C_i(M)`, escalating
    ///    `M -> M + 1` (if `L_i > M`) or aborting the job (if `M >= L_i`).
    /// 3. Releases any periodic jobs whose `next_release <= now`.
    /// 4. Checks for idle-instant recovery if `system_mode > 0`.
    /// 5. Selects and dispatches the next task, reprogramming the MPU/PMP **only** if criticality changes.
    pub fn on_timer_interrupt(&mut self) -> KernelEvent {
        let now = self.arch.now();
        self.account_running_task(now);

        let mut event = None;

        if let Some(curr_id) = self.current_task {
            let budget = self.tasks[curr_id.0].spec.budget_for_mode(self.system_mode);
            if self.tasks[curr_id.0].exec_time >= budget {
                event = Some(self.handle_budget_exhaustion(curr_id, now));
            }
        }

        self.release_due_jobs(now);

        if self.system_mode.0 > 0 {
            if let Some(recovered) = self.try_idle_instant_recovery(now) {
                if event.is_none() {
                    event = Some(recovered);
                }
            }
        }

        let next = self.reschedule(now);
        event.unwrap_or(KernelEvent::Dispatched(next))
    }

    /// Marks the currently running task's job as completed (`SVC` / `sys_job_complete`).
    ///
    /// Resets its execution time accumulator, transitions it to `Dormant` until `next_release`,
    /// checks for idle-instant recovery to nominal mode, and dispatches the next ready task.
    pub fn complete_current_job(&mut self) -> KernelEvent {
        let now = self.arch.now();
        self.account_running_task(now);

        if let Some(curr_id) = self.current_task {
            self.scheduler
                .on_task_unready(curr_id, &self.tasks, self.system_mode);
            let tcb = &mut self.tasks[curr_id.0];
            tcb.state = TaskState::Dormant;
            tcb.exec_time = 0;
            tcb.priority_boost = None;
            tcb.sp = tcb.initial_sp;
            tcb.completed_jobs += 1;
            self.current_task = None;
        }

        self.release_due_jobs(now);

        let mut recovery_event = None;
        if self.system_mode.0 > 0 {
            recovery_event = self.try_idle_instant_recovery(now);
        }

        let next = self.reschedule(now);
        recovery_event.unwrap_or(KernelEvent::Dispatched(next))
    }

    /// Acquires a same-criticality resource under the Immediate Priority Ceiling Protocol (IPCP).
    ///
    /// Immediately raises the calling task's effective priority to `resource.ceiling_priority`
    /// and updates the scheduler ready queue.
    pub fn acquire_ipcp_resource(
        &mut self,
        resource: &IpcpResourceSpec,
    ) -> Result<Option<Priority>, IpcpError> {
        let curr_id = self.current_task.ok_or(IpcpError::NoRunningTask)?;
        let task_crit = self.tasks[curr_id.0].spec.criticality;
        if task_crit != resource.partition_crit {
            return Err(IpcpError::CrossCriticalityViolation {
                task_crit,
                resource_crit: resource.partition_crit,
            });
        }

        let prev_boost = self.tasks[curr_id.0].priority_boost;
        let new_boost = match prev_boost {
            Some(existing) if existing.0 >= resource.ceiling_priority.0 => existing,
            _ => resource.ceiling_priority,
        };
        self.tasks[curr_id.0].priority_boost = Some(new_boost);
        let now = self.arch.now();
        self.scheduler
            .on_task_ready(curr_id, &self.tasks, self.system_mode, now);
        Ok(prev_boost)
    }

    /// Releases an IPCP resource, restoring the task's previous priority boost and rescheduling
    /// if a higher-priority task was preempted.
    pub fn release_ipcp_resource(
        &mut self,
        previous_boost: Option<Priority>,
    ) -> Result<Option<TaskId>, IpcpError> {
        let curr_id = self.current_task.ok_or(IpcpError::NoRunningTask)?;
        let now = self.arch.now();
        self.account_running_task(now);
        self.tasks[curr_id.0].priority_boost = previous_boost;
        self.scheduler
            .on_mode_change(&self.tasks, self.system_mode, self.system_mode, now);
        Ok(self.reschedule(now))
    }

    /// Charges elapsed cycles since `last_dispatched_at` to the currently running task.
    fn account_running_task(&mut self, now: Ticks) {
        if let Some(curr_id) = self.current_task {
            let tcb = &mut self.tasks[curr_id.0];
            if tcb.state == TaskState::Running && now > tcb.last_dispatched_at {
                let delta = now - tcb.last_dispatched_at;
                tcb.exec_time = tcb.exec_time.saturating_add(delta);
                tcb.last_dispatched_at = now;
            }
        }
    }

    /// Handles WCET budget exhaustion for `task_id` at timestamp `now`.
    fn handle_budget_exhaustion(&mut self, task_id: TaskId, now: Ticks) -> KernelEvent {
        self.tasks[task_id.0].overrun_count += 1;
        let task_crit = self.tasks[task_id.0].spec.criticality;

        // Case 1: The overrunning task has higher criticality than the current system mode.
        // Escalate system criticality mode step-by-step (M -> M + 1) until the task has remaining budget
        // or reaches its own criticality ceiling.
        if self.system_mode.0 < task_crit.0 {
            let old_mode = self.system_mode;
            let mut target_mode = CritLevel(self.system_mode.0 + 1);
            while target_mode.0 < task_crit.0
                && self.tasks[task_id.0].exec_time
                    >= self.tasks[task_id.0].spec.budget_for_mode(target_mode)
            {
                target_mode = CritLevel(target_mode.0 + 1);
            }

            self.transition_mode_upward(old_mode, target_mode, now);

            // Check if even at target_mode the task has already exhausted its budget
            if self.tasks[task_id.0].exec_time
                >= self.tasks[task_id.0].spec.budget_for_mode(self.system_mode)
            {
                self.abort_task_job(task_id);
            }

            KernelEvent::ModeEscalated {
                overrunning_task: task_id,
                old_mode,
                new_mode: self.system_mode,
            }
        } else {
            // Case 2: The task exhausted its maximum guaranteed budget at its own criticality level
            // (or its slack budget in a degraded mode). Isolate the fault by aborting the job.
            self.abort_task_job(task_id);
            self.stats.max_budget_aborts += 1;
            KernelEvent::JobAbortedOverrun {
                task: task_id,
                mode: self.system_mode,
            }
        }
    }

    /// Escalates `system_mode` from `old_mode` to `new_mode` (`new_mode > old_mode`),
    /// applying each lower-criticality task's `DegradePolicy`.
    fn transition_mode_upward(&mut self, old_mode: CritLevel, new_mode: CritLevel, now: Ticks) {
        self.system_mode = new_mode;
        self.stats.mode_escalations += 1;

        for i in 0..N {
            let task_crit = self.tasks[i].spec.criticality;
            if task_crit.0 < new_mode.0 {
                let id = self.tasks[i].spec.id;
                match self.tasks[i].spec.degrade_policy {
                    DegradePolicy::Abort => {
                        if self.tasks[i].is_job_active() {
                            self.scheduler.on_task_unready(id, &self.tasks, old_mode);
                            self.tasks[i].state = TaskState::Aborted;
                            self.tasks[i].exec_time = 0;
                            self.tasks[i].priority_boost = None;
                            self.tasks[i].sp = self.tasks[i].initial_sp;
                            if self.current_task == Some(id) {
                                self.current_task = None;
                            }
                        }
                    }
                    DegradePolicy::Suspend => {
                        if matches!(self.tasks[i].state, TaskState::Ready | TaskState::Running) {
                            self.scheduler.on_task_unready(id, &self.tasks, old_mode);
                            self.tasks[i].state = TaskState::SuspendedByMode;
                            if self.current_task == Some(id) {
                                self.current_task = None;
                            }
                        }
                    }
                    DegradePolicy::BackgroundSlack => {
                        // Task stays Ready/Running; scheduler.on_mode_change() demotes it to slack band.
                    }
                }
            }
        }

        self.scheduler
            .on_mode_change(&self.tasks, old_mode, new_mode, now);
    }

    /// Aborts a single task's current job due to budget overrun.
    fn abort_task_job(&mut self, task_id: TaskId) {
        self.scheduler
            .on_task_unready(task_id, &self.tasks, self.system_mode);
        let tcb = &mut self.tasks[task_id.0];
        tcb.state = TaskState::Aborted;
        tcb.exec_time = 0;
        tcb.priority_boost = None;
        tcb.sp = tcb.initial_sp;
        if self.current_task == Some(task_id) {
            self.current_task = None;
        }
    }

    /// Releases periodic jobs whose `next_release <= now`.
    fn release_due_jobs(&mut self, now: Ticks) {
        for i in 0..N {
            while now >= self.tasks[i].next_release {
                let period = self.tasks[i].spec.period;
                let release_tick = self.tasks[i].next_release;
                self.tasks[i].next_release = release_tick.saturating_add(period);

                let task_crit = self.tasks[i].spec.criticality;
                let policy = self.tasks[i].spec.degrade_policy;

                // Determine whether this task is permitted to release a new job in `system_mode`
                let can_release = task_crit.0 >= self.system_mode.0
                    || matches!(policy, DegradePolicy::BackgroundSlack);

                if can_release {
                    if matches!(self.tasks[i].state, TaskState::Dormant | TaskState::Aborted) {
                        self.tasks[i].state = TaskState::Ready;
                        self.tasks[i].exec_time = 0;
                        self.tasks[i].priority_boost = None;
                        self.tasks[i].sp = self.tasks[i].initial_sp;
                        self.tasks[i].current_job_release = release_tick;
                        let id = self.tasks[i].spec.id;
                        self.scheduler
                            .on_task_ready(id, &self.tasks, self.system_mode, now);
                    }
                }
            }
        }
    }

    /// Checks whether the system has reached an **Idle Instant** for higher-criticality tasks
    /// (no task with `criticality > 0` has an active job), and if so, recovers to `CritLevel(0)`.
    fn try_idle_instant_recovery(&mut self, now: Ticks) -> Option<KernelEvent> {
        for i in 0..N {
            if self.tasks[i].spec.criticality.0 > 0 && self.tasks[i].is_job_active() {
                return None;
            }
        }

        let old_mode = self.system_mode;
        self.system_mode = CritLevel::LOWEST;
        self.stats.idle_recoveries += 1;

        // Resume any lower-criticality tasks that were frozen via `DegradePolicy::Suspend`
        for i in 0..N {
            if self.tasks[i].state == TaskState::SuspendedByMode {
                self.tasks[i].state = TaskState::Ready;
                let id = self.tasks[i].spec.id;
                self.scheduler
                    .on_task_ready(id, &self.tasks, self.system_mode, now);
            }
        }

        self.scheduler
            .on_mode_change(&self.tasks, old_mode, self.system_mode, now);

        Some(KernelEvent::ModeRecovered { old_mode })
    }

    /// Selects the highest-priority / earliest-deadline ready task, executes a context switch
    /// if needed (using the fast path if criticality matches), and arms the one-shot hardware timer.
    fn reschedule(&mut self, now: Ticks) -> Option<TaskId> {
        let next_id = self.scheduler.pick_next(&self.tasks, self.system_mode);

        if next_id != self.current_task {
            // Transition outgoing task from Running -> Ready (if still active)
            let prev_sp_slot = if let Some(prev_id) = self.current_task {
                if self.tasks[prev_id.0].state == TaskState::Running {
                    self.tasks[prev_id.0].state = TaskState::Ready;
                }
                Some(&raw mut self.tasks[prev_id.0].sp)
            } else {
                None
            };

            if let Some(next) = next_id {
                let next_crit = self.tasks[next.0].spec.criticality;
                let next_bounds = self.tasks[next.0].spec.stack_bounds;
                let next_sp = self.tasks[next.0].sp;

                self.tasks[next.0].state = TaskState::Running;
                self.tasks[next.0].last_dispatched_at = now;

                // Per-criticality-partition MPU/PMP optimization:
                // Only reprogram the hardware memory protection unit when switching across
                // DIFFERENT criticality levels!
                if self.active_partition_crit == Some(next_crit) {
                    self.stats.fast_same_crit_switches += 1;
                } else {
                    self.arch
                        .switch_partition(next_crit.0, &self.partitions[next_crit.0]);
                    self.active_partition_crit = Some(next_crit);
                    self.stats.cross_crit_mpu_switches += 1;
                }

                // Always update the per-thread stack guard (1 register on ARMv8-M / 1 slot on ARMv7-M)
                self.arch.update_stack_guard(next_bounds);
                self.arch.trigger_context_switch(prev_sp_slot, next_sp);
                self.stats.total_context_switches += 1;
            }

            self.current_task = next_id;
        } else if let Some(curr) = self.current_task {
            self.tasks[curr.0].last_dispatched_at = now;
        }

        self.arm_next_timer_event(now);
        self.current_task
    }

    /// Computes the earliest upcoming kernel event (`min(running_task_budget_expiry, next_job_release)`)
    /// and programs the hardware one-shot timer.
    fn arm_next_timer_event(&mut self, now: Ticks) {
        let mut earliest: Option<Ticks> = None;

        // 1. Remaining execution budget of the currently running task
        if let Some(curr_id) = self.current_task {
            let rem = self.tasks[curr_id.0].remaining_budget(self.system_mode);
            let expiry = now.saturating_add(rem);
            earliest = Some(expiry);
        }

        // 2. Earliest upcoming periodic job arrival
        for i in 0..N {
            let task_crit = self.tasks[i].spec.criticality;
            let eligible_in_mode = task_crit.0 >= self.system_mode.0
                || matches!(
                    self.tasks[i].spec.degrade_policy,
                    DegradePolicy::BackgroundSlack
                );
            if eligible_in_mode {
                let rel = self.tasks[i].next_release;
                earliest = Some(match earliest {
                    Some(e) if e <= rel => e,
                    _ => rel,
                });
            }
        }

        self.arch.arm_oneshot_timer(earliest);
    }
}
