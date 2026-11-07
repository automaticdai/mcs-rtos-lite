#![no_std]
#![forbid(unsafe_op_in_unsafe_fn)]

/// Monotonic time / execution budget unit (in hardware cycles or timer ticks).
pub type Ticks = u64;

/// Bounds of a thread's dedicated stack in memory.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct StackBounds {
    /// Lowest valid address of the stack (where overflow occurs on downward-growing stacks).
    pub bottom: usize,
    /// Highest address of the stack (initial stack pointer location).
    pub top: usize,
}

impl StackBounds {
    /// Creates stack bounds from a base address and byte size.
    #[inline]
    pub const fn new(bottom: usize, size_bytes: usize) -> Self {
        Self {
            bottom,
            top: bottom + size_bytes,
        }
    }

    /// Returns the total stack size in bytes.
    #[inline]
    pub const fn size(&self) -> usize {
        self.top.saturating_sub(self.bottom)
    }

    /// Checks whether a given stack pointer lies strictly within `[bottom, top]`.
    #[inline]
    pub const fn contains_sp(&self, sp: usize) -> bool {
        sp >= self.bottom && sp <= self.top
    }
}

/// Hardware abstraction interface parameterized over `L` criticality levels.
///
/// Implementations provide:
/// - Preemptive stateful thread context switching (`PendSV` on Cortex-M, trap handler on RISC-V).
/// - Per-criticality-partition memory protection (`MPU` on Cortex-M, `PMP` on RISC-V), reprogrammed
///   **only** on cross-criticality switches (`prev_crit != next_crit`).
/// - Per-thread intra-partition stack guard updates (`PSPLIM` on ARMv8-M or dynamic stack guard region on ARMv7-M).
/// - Cycle-accurate monotonic clock and one-shot budget timer for Vestal WCET enforcement.
pub trait Arch<const L: usize> {
    /// Hardware-specific memory protection table for a single criticality partition
    /// (e.g., precomputed ARM MPU `RBAR`/`RASR` pairs or RISC-V `pmpcfg`/`pmpaddr` CSRs).
    type PartitionConfig: Copy + Clone;

    /// Initializes architecture-specific hardware (exception priorities, MPU/PMP enable, timer).
    fn init_hardware(&mut self, partitions: &[Self::PartitionConfig; L]);

    /// Initializes a stateful thread's stack frame so it can be restored by the context switcher.
    ///
    /// Returns the initial stack pointer (`PSP` on Cortex-M) to store in the task's TCB.
    ///
    /// # Safety
    /// `stack` must be properly aligned for the target architecture (8-byte aligned on ARM AAPCS).
    unsafe fn init_task_stack(
        stack: &mut [u32],
        entry: extern "C" fn(usize) -> !,
        arg: usize,
    ) -> usize;

    /// Returns the current monotonic hardware tick/cycle count.
    fn now(&self) -> Ticks;

    /// Programs the hardware one-shot timer to fire at `target_tick` (absolute monotonic tick).
    /// Used both for exact WCET budget enforcement and next periodic job arrival.
    fn arm_oneshot_timer(&mut self, target_tick: Option<Ticks>);

    /// Reprograms the hardware MPU/PMP for `next_crit` using its static `PartitionConfig`.
    ///
    /// Called by the kernel **only** on a cross-criticality context switch (`prev_crit != next_crit`),
    /// keeping same-criticality context switches on the fast path.
    fn switch_partition(&mut self, next_crit: usize, config: &Self::PartitionConfig);

    /// Updates the hardware stack-overflow guard for the incoming stateful thread.
    ///
    /// Executed on every context switch (e.g., 1 write to `PSPLIM` on Cortex-M33 or 1 MPU slot on Cortex-M4).
    fn update_stack_guard(&mut self, bounds: StackBounds);

    /// Requests an asynchronous context switch (e.g., sets `SCB->ICSR.PENDSVSET` on Cortex-M)
    /// and records the outgoing and incoming stack pointer slots.
    fn trigger_context_switch(&mut self, prev_sp_slot: Option<*mut usize>, next_sp: usize);
}
