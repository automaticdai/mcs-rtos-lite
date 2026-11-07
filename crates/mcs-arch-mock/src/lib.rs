#![no_std]
#![forbid(unsafe_op_in_unsafe_fn)]

use mcs_arch::{Arch, StackBounds, Ticks};

/// Mock partition configuration for deterministic host simulation.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct MockPartitionConfig {
    pub partition_id: usize,
    pub base_addr: usize,
    pub size_bytes: usize,
}

impl MockPartitionConfig {
    pub const fn new(partition_id: usize, base_addr: usize, size_bytes: usize) -> Self {
        Self {
            partition_id,
            base_addr,
            size_bytes,
        }
    }
}

/// Deterministic discrete-event hardware simulator implementing `Arch<L>`.
pub struct MockArch<const L: usize> {
    pub clock: Ticks,
    pub armed_timer: Option<Ticks>,
    pub active_partition: Option<MockPartitionConfig>,
    pub active_stack_guard: Option<StackBounds>,
    pub active_sp: usize,
    pub partition_switch_count: u64,
    pub stack_guard_update_count: u64,
    pub context_switch_count: u64,
    pub hardware_initialized: bool,
}

impl<const L: usize> Default for MockArch<L> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const L: usize> MockArch<L> {
    pub const fn new() -> Self {
        Self {
            clock: 0,
            armed_timer: None,
            active_partition: None,
            active_stack_guard: None,
            active_sp: 0,
            partition_switch_count: 0,
            stack_guard_update_count: 0,
            context_switch_count: 0,
            hardware_initialized: false,
        }
    }

    /// Advances the simulated monotonic clock by `delta` ticks.
    #[inline]
    pub fn advance(&mut self, delta: Ticks) {
        self.clock = self.clock.saturating_add(delta);
    }

    /// Advances the simulated clock directly to `target_tick`.
    #[inline]
    pub fn jump_to(&mut self, target_tick: Ticks) {
        assert!(
            target_tick >= self.clock,
            "Monotonic clock cannot move backwards"
        );
        self.clock = target_tick;
    }
}

impl<const L: usize> Arch<L> for MockArch<L> {
    type PartitionConfig = MockPartitionConfig;

    fn init_hardware(&mut self, _partitions: &[Self::PartitionConfig; L]) {
        self.hardware_initialized = true;
    }

    unsafe fn init_task_stack(
        stack: &mut [u32],
        entry: extern "C" fn(usize) -> !,
        arg: usize,
    ) -> usize {
        assert!(stack.len() >= 4);
        let len = stack.len();
        stack[len - 1] = entry as usize as u32;
        stack[len - 2] = arg as u32;
        unsafe { stack.as_mut_ptr().add(len - 2) as usize }
    }

    #[inline]
    fn now(&self) -> Ticks {
        self.clock
    }

    fn arm_oneshot_timer(&mut self, target_tick: Option<Ticks>) {
        self.armed_timer = target_tick;
    }

    fn switch_partition(&mut self, _next_crit: usize, config: &Self::PartitionConfig) {
        self.active_partition = Some(*config);
        self.partition_switch_count += 1;
    }

    fn update_stack_guard(&mut self, bounds: StackBounds) {
        self.active_stack_guard = Some(bounds);
        self.stack_guard_update_count += 1;
    }

    fn trigger_context_switch(&mut self, prev_sp_slot: Option<*mut usize>, next_sp: usize) {
        if let Some(slot) = prev_sp_slot {
            unsafe {
                slot.write(self.active_sp);
            }
        }
        self.active_sp = next_sp;
        self.context_switch_count += 1;
    }
}
