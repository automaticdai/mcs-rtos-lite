#![no_std]
#![forbid(unsafe_op_in_unsafe_fn)]

pub mod context;
pub mod mpu;

pub use context::{
    CortexMHardwareFrame, CortexMInitialFrame, CortexMSoftwareFrame, EXC_RETURN_THREAD_PSP,
    INITIAL_XPSR_THUMB, init_cortex_m_stack,
};
use mcs_arch::{Arch, StackBounds, Ticks};
pub use mpu::{Armv7mMpuRegion, CortexMPartitionConfig, MpuAccessPerm};

// ============================================================================
// ARM Cortex-M System Control Space (SCS) Architectural MMIO Addresses
// ============================================================================

/// System Control Block Interrupt Control and State Register (`SCB->ICSR`).
pub const SCB_ICSR_ADDR: usize = 0xE000_ED04;
/// Bit 28 in `SCB->ICSR` triggers a `PendSV` exception.
pub const SCB_ICSR_PENDSVSET: u32 = 1u32 << 28;

/// System Handler Priority Register 3 (`SCB->SHPR3`: configures `PendSV` and `SysTick` priorities).
pub const SCB_SHPR3_ADDR: usize = 0xE000_ED20;

/// MPU Control Register (`MPU->CTRL`).
pub const MPU_CTRL_ADDR: usize = 0xE000_ED94;
/// Enables MPU with default memory map enabled for privileged kernel mode (`PRIVDEFENA | ENABLE`).
pub const MPU_CTRL_ENABLE_PRIVDEF: u32 = (1u32 << 2) | 1u32;

/// MPU Region Base Address Register (`MPU->RBAR`).
pub const MPU_RBAR_ADDR: usize = 0xE000_ED9C;
/// MPU Region Attribute and Size Register (`MPU->RASR`).
pub const MPU_RASR_ADDR: usize = 0xE000_EDA0;

/// SysTick Control and Status Register (`SYST->CSR`).
pub const SYST_CSR_ADDR: usize = 0xE000_E010;
/// SysTick Reload Value Register (`SYST->RVR`).
pub const SYST_RVR_ADDR: usize = 0xE000_E014;
/// SysTick Current Value Register (`SYST->CVR`).
pub const SYST_CVR_ADDR: usize = 0xE000_E018;

/// ARM Cortex-M hardware backend for `mcs-rtos-lite`.
///
/// Supports:
/// - **Per-criticality-partition MPU reprogramming**: Writes `MPU_RBAR`/`MPU_RASR` for Regions `0..7`
///   only when switching between tasks of different criticalities.
/// - **Per-thread dynamic stack guard**: Updates MPU Region `7` (32-byte `NoAccess` red zone at
///   `stack.bottom`) on every context switch.
/// - **`PendSV` deferred context switching**: Records outgoing/incoming `PSP` pointers and sets
///   `SCB->ICSR.PENDSVSET`.
pub struct CortexMArch<const L: usize> {
    /// Monotonic cycle/tick counter maintained by the timer/DWT driver.
    monotonic_ticks: Ticks,
    /// Currently armed one-shot timer target tick.
    armed_timer_target: Option<Ticks>,
    /// Mirror of active hardware MPU regions (`0..8`, where `0..7` are partition regions and `7` is stack guard).
    active_mpu_regions: [Armv7mMpuRegion; 8],
    /// Whether `MPU->CTRL` is enabled.
    mpu_enabled: bool,
    /// Whether `SCB->ICSR.PENDSVSET` has been asserted for a pending context switch.
    pendsv_pending: bool,
    /// Outgoing task's `PSP` save slot for the `PendSV` handler.
    pending_prev_sp_slot: Option<*mut usize>,
    /// Incoming task's `PSP` value to restore in the `PendSV` handler.
    pending_next_sp: usize,
}

impl<const L: usize> Default for CortexMArch<L> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const L: usize> CortexMArch<L> {
    pub const fn new() -> Self {
        Self {
            monotonic_ticks: 0,
            armed_timer_target: None,
            active_mpu_regions: [
                Armv7mMpuRegion::disabled(0),
                Armv7mMpuRegion::disabled(1),
                Armv7mMpuRegion::disabled(2),
                Armv7mMpuRegion::disabled(3),
                Armv7mMpuRegion::disabled(4),
                Armv7mMpuRegion::disabled(5),
                Armv7mMpuRegion::disabled(6),
                Armv7mMpuRegion::disabled(7),
            ],
            mpu_enabled: false,
            pendsv_pending: false,
            pending_prev_sp_slot: None,
            pending_next_sp: 0,
        }
    }

    /// Advances the hardware monotonic clock (called by the timer ISR or QEMU test harness).
    #[inline]
    pub fn advance_ticks(&mut self, delta: Ticks) {
        self.monotonic_ticks = self.monotonic_ticks.saturating_add(delta);
    }

    /// Sets the hardware monotonic clock to an exact tick value.
    #[inline]
    pub fn set_ticks(&mut self, ticks: Ticks) {
        self.monotonic_ticks = ticks;
    }

    /// Returns the currently armed one-shot timer target tick.
    #[inline]
    pub const fn armed_timer_target(&self) -> Option<Ticks> {
        self.armed_timer_target
    }

    /// Returns the currently active 8 hardware MPU regions (`0..7` partition + `7` stack guard).
    #[inline]
    pub const fn active_mpu_regions(&self) -> &[Armv7mMpuRegion; 8] {
        &self.active_mpu_regions
    }

    /// Returns true if `PendSV` has been pended since the last context switch completion.
    #[inline]
    pub const fn is_pendsv_pending(&self) -> bool {
        self.pendsv_pending
    }

    /// Returns the incoming task's `PSP` queued for `PendSV`.
    #[inline]
    pub const fn pending_next_sp(&self) -> usize {
        self.pending_next_sp
    }

    /// Clears the `PendSV` pending state after the assembly handler completes the switch.
    #[inline]
    pub fn complete_pendsv_switch(&mut self, saved_prev_sp: usize) {
        if let Some(slot) = self.pending_prev_sp_slot.take() {
            unsafe {
                slot.write(saved_prev_sp);
            }
        }
        self.pendsv_pending = false;
    }

    #[inline]
    fn write_mpu_region_hw(&mut self, idx: usize, region: Armv7mMpuRegion) {
        self.active_mpu_regions[idx] = region;
        #[cfg(target_arch = "arm")]
        unsafe {
            (MPU_RBAR_ADDR as *mut u32).write_volatile(region.rbar);
            (MPU_RASR_ADDR as *mut u32).write_volatile(region.rasr);
        }
    }
}

impl<const L: usize> Arch<L> for CortexMArch<L> {
    type PartitionConfig = CortexMPartitionConfig;

    fn init_hardware(&mut self, _partitions: &[Self::PartitionConfig; L]) {
        self.mpu_enabled = true;
        #[cfg(target_arch = "arm")]
        unsafe {
            // Set PendSV to lowest exception priority (0xFF) in SCB->SHPR3 (bits 23:16)
            let shpr3 = (SCB_SHPR3_ADDR as *const u32).read_volatile();
            (SCB_SHPR3_ADDR as *mut u32).write_volatile(shpr3 | (0xFFu32 << 16));
            // Enable MPU with PRIVDEFENA so the privileged kernel can access all memory
            (MPU_CTRL_ADDR as *mut u32).write_volatile(MPU_CTRL_ENABLE_PRIVDEF);
            core::arch::asm!("dsb", "isb", options(nomem, nostack));
        }
    }

    unsafe fn init_task_stack(
        stack: &mut [u32],
        entry: extern "C" fn(usize) -> !,
        arg: usize,
    ) -> usize {
        unsafe { init_cortex_m_stack(stack, entry, arg) }
    }

    #[inline]
    fn now(&self) -> Ticks {
        self.monotonic_ticks
    }

    fn arm_oneshot_timer(&mut self, target_tick: Option<Ticks>) {
        self.armed_timer_target = target_tick;
        #[cfg(target_arch = "arm")]
        if let Some(target) = target_tick {
            let delta = target.saturating_sub(self.monotonic_ticks).max(1);
            let reload = delta.min(0x00FF_FFFF) as u32;
            unsafe {
                (SYST_CSR_ADDR as *mut u32).write_volatile(0);
                (SYST_RVR_ADDR as *mut u32).write_volatile(reload);
                (SYST_CVR_ADDR as *mut u32).write_volatile(0);
                // Enable SysTick with processor clock and interrupt (CLKSOURCE | TICKINT | ENABLE = 0b111)
                (SYST_CSR_ADDR as *mut u32).write_volatile(0b111);
            }
        }
    }

    fn switch_partition(&mut self, _next_crit: usize, config: &Self::PartitionConfig) {
        // Reprogram Regions 0..7 for the incoming Criticality Partition
        let mut i = 0;
        while i < 7 {
            self.write_mpu_region_hw(i, config.regions[i]);
            i += 1;
        }
        #[cfg(target_arch = "arm")]
        unsafe {
            core::arch::asm!("dsb", "isb", options(nomem, nostack));
        }
    }

    fn update_stack_guard(&mut self, bounds: StackBounds) {
        // Region 7 is dedicated to the active thread's 32-byte stack overflow guard
        let guard = Armv7mMpuRegion::encode_stack_guard(bounds);
        self.write_mpu_region_hw(7, guard);
    }

    fn trigger_context_switch(&mut self, prev_sp_slot: Option<*mut usize>, next_sp: usize) {
        self.pending_prev_sp_slot = prev_sp_slot;
        self.pending_next_sp = next_sp;
        self.pendsv_pending = true;
        #[cfg(target_arch = "arm")]
        unsafe {
            context::MCS_PENDSV_MAILBOX.prev_sp_slot =
                prev_sp_slot.unwrap_or(core::ptr::null_mut());
            context::MCS_PENDSV_MAILBOX.next_sp = next_sp;
            (SCB_ICSR_ADDR as *mut u32).write_volatile(SCB_ICSR_PENDSVSET);
            core::arch::asm!("dsb", "isb", options(nomem, nostack));
        }
    }
}
