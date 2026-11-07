pub mod amc;
pub mod edf_vd;

use mcs_arch::Ticks;

use crate::task::TaskControlBlock;
use crate::types::{CritLevel, TaskId};

pub use amc::{AmcParams, AmcScheduler, verify_amc_rtb};
pub use edf_vd::{EdfVdParams, EdfVdScheduler};

/// Pluggable mixed-criticality scheduling policy parameterized over `L` criticality levels
/// and `N` statically allocated tasks.
pub trait Scheduler<const L: usize, const N: usize> {
    /// Per-task scheduling parameters stored inside each `TaskControlBlock`.
    type TaskSchedParams: Copy + Clone;

    /// Initializes the scheduler with the static task table and initial system criticality mode.
    fn init(&mut self, tasks: &[TaskControlBlock<L, Self::TaskSchedParams>; N], mode: CritLevel);

    /// Inserts `task_id` into the ready queue for the current `mode`.
    fn on_task_ready(
        &mut self,
        task_id: TaskId,
        tasks: &[TaskControlBlock<L, Self::TaskSchedParams>; N],
        mode: CritLevel,
        now: Ticks,
    );

    /// Removes `task_id` from the ready queue (on job completion, blocking, suspension, or abort).
    fn on_task_unready(
        &mut self,
        task_id: TaskId,
        tasks: &[TaskControlBlock<L, Self::TaskSchedParams>; N],
        mode: CritLevel,
    );

    /// Rebuilds or updates ready-queue ordering when the system criticality mode changes from
    /// `old_mode` to `new_mode` (either upward escalation or downward idle-instant recovery).
    fn on_mode_change(
        &mut self,
        tasks: &[TaskControlBlock<L, Self::TaskSchedParams>; N],
        old_mode: CritLevel,
        new_mode: CritLevel,
        now: Ticks,
    );

    /// Selects the highest-priority / earliest-deadline ready task in the current `mode`.
    fn pick_next(
        &self,
        tasks: &[TaskControlBlock<L, Self::TaskSchedParams>; N],
        mode: CritLevel,
    ) -> Option<TaskId>;
}
