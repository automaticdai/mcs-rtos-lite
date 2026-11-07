use mcs_arch::{Arch, StackBounds};
use mcs_arch_cortex_m::{
    CortexMArch, CortexMInitialFrame, CortexMPartitionConfig, EXC_RETURN_THREAD_PSP,
    INITIAL_XPSR_THUMB, MpuAccessPerm,
};
use mcs_arch_mock::{MockArch, MockPartitionConfig};
use mcs_core::{
    AmcParams, AmcScheduler, CritLevel, DegradePolicy, EdfVdParams, EdfVdScheduler, IpcpError,
    IpcpResourceSpec, Kernel, KernelEvent, Priority, SpscChannel, TaskControlBlock, TaskId,
    TaskSpec, TaskState, TripleBuffer, verify_amc_rtb,
};

extern "C" fn dummy_task_entry(_arg: usize) -> ! {
    loop {
        core::hint::spin_loop();
    }
}

/// 1. Tests a 3-criticality-level (`L = 3`: `LO = 0`, `MID = 1`, `HI = 2`) system under `AmcScheduler`:
///    - Compile-time `verify_amc_rtb` verification
///    - Stepwise upward mode escalations (`0 -> 1 -> 2`) on WCET budget exhaustion
///    - `Suspend` and `Abort` degradation policies
///    - Downward recovery (`2 -> 0`) at an idle instant
#[test]
fn test_three_level_amc_mode_escalations_and_idle_recovery() {
    const L: usize = 3;
    const N: usize = 3;

    const SPECS: [TaskSpec<L>; N] = [
        // Task 0: LO criticality (level 0), suspended when mode > 0
        TaskSpec {
            id: TaskId(0),
            criticality: CritLevel(0),
            period: 100,
            deadline: 100,
            budgets: [15, 15, 15],
            degrade_policy: DegradePolicy::Suspend,
            stack_bounds: StackBounds::new(0x2000_0000, 1024),
        },
        // Task 1: MID criticality (level 1), aborted when mode > 1
        TaskSpec {
            id: TaskId(1),
            criticality: CritLevel(1),
            period: 50,
            deadline: 50,
            budgets: [10, 20, 20],
            degrade_policy: DegradePolicy::Abort,
            stack_bounds: StackBounds::new(0x2000_0400, 1024),
        },
        // Task 2: HI criticality (level 2), budgets [10, 18, 30]
        TaskSpec {
            id: TaskId(2),
            criticality: CritLevel(2),
            period: 40,
            deadline: 40,
            budgets: [10, 18, 30],
            degrade_policy: DegradePolicy::Abort,
            stack_bounds: StackBounds::new(0x2000_0800, 1024),
        },
    ];

    const PARAMS: [AmcParams<L>; N] = [
        AmcParams::uniform(Priority::new(10)),
        AmcParams::uniform(Priority::new(20)),
        AmcParams::uniform(Priority::new(30)),
    ];

    // Compile-time / const AMC-rtb schedulability proof across all 3 criticality levels!
    const SCHEDULABLE: bool = verify_amc_rtb(&SPECS, &PARAMS);
    assert!(SCHEDULABLE, "Task set must pass 3-level AMC-rtb analysis");

    let tasks = [
        TaskControlBlock::new(SPECS[0], PARAMS[0], 0x2000_0400),
        TaskControlBlock::new(SPECS[1], PARAMS[1], 0x2000_0800),
        TaskControlBlock::new(SPECS[2], PARAMS[2], 0x2000_0C00),
    ];

    let partitions = [
        MockPartitionConfig::new(0, 0x2000_0000, 0x400),
        MockPartitionConfig::new(1, 0x2000_0400, 0x400),
        MockPartitionConfig::new(2, 0x2000_0800, 0x400),
    ];

    let mut kernel = Kernel::new(tasks, partitions, AmcScheduler::new(), MockArch::new());

    // Boot at t = 0: all 3 tasks are released; Task 2 (HI, prio 30) is dispatched first.
    let first = kernel.boot();
    assert_eq!(first, Some(TaskId(2)));
    assert_eq!(kernel.system_mode(), CritLevel(0));
    // One-shot timer should be armed to Task 2's nominal mode-0 budget (t = 10)
    assert_eq!(kernel.arch().armed_timer, Some(10));

    // At t = 10, Task 2 exhausts its mode-0 budget C_2(0) = 10 without completing!
    // System must escalate from mode 0 -> mode 1:
    // - Task 0 (LO, DegradePolicy::Suspend) transitions from Ready -> SuspendedByMode
    // - Task 1 (MID, crit 1 >= mode 1) remains Ready
    // - Task 2 (HI) gets its budget extended to C_2(1) = 18 (8 ticks remaining)
    kernel.arch_mut().jump_to(10);
    let ev1 = kernel.on_timer_interrupt();
    assert_eq!(
        ev1,
        KernelEvent::ModeEscalated {
            overrunning_task: TaskId(2),
            old_mode: CritLevel(0),
            new_mode: CritLevel(1),
        }
    );
    assert_eq!(kernel.system_mode(), CritLevel(1));
    assert_eq!(kernel.tasks()[0].state, TaskState::SuspendedByMode);
    assert_eq!(kernel.tasks()[1].state, TaskState::Ready);
    assert_eq!(kernel.current_task(), Some(TaskId(2)));
    assert_eq!(kernel.arch().armed_timer, Some(18));

    // At t = 18, Task 2 exhausts its mode-1 budget C_2(1) = 18 without completing!
    // System must escalate from mode 1 -> mode 2:
    // - Task 1 (MID, DegradePolicy::Abort) is aborted (state -> Aborted)
    // - Task 2 (HI) gets its budget extended to C_2(2) = 30 (12 ticks remaining, timer at t = 30)
    kernel.arch_mut().jump_to(18);
    let ev2 = kernel.on_timer_interrupt();
    assert_eq!(
        ev2,
        KernelEvent::ModeEscalated {
            overrunning_task: TaskId(2),
            old_mode: CritLevel(1),
            new_mode: CritLevel(2),
        }
    );
    assert_eq!(kernel.system_mode(), CritLevel(2));
    assert_eq!(kernel.tasks()[1].state, TaskState::Aborted);
    assert_eq!(kernel.current_task(), Some(TaskId(2)));
    assert_eq!(kernel.arch().armed_timer, Some(30));

    // At t = 25 (within its HI budget of 30), Task 2 completes its job!
    // Since Task 1 (MID) was aborted and no tasks of criticality > 0 are active,
    // t = 25 is an Idle Instant!
    // The kernel must recover to nominal mode 0 and resume Task 0 (which was SuspendedByMode).
    kernel.arch_mut().jump_to(25);
    let ev3 = kernel.complete_current_job();
    assert_eq!(
        ev3,
        KernelEvent::ModeRecovered {
            old_mode: CritLevel(2),
        }
    );
    assert_eq!(kernel.system_mode(), CritLevel(0));
    assert_eq!(kernel.current_task(), Some(TaskId(0)));
    assert_eq!(kernel.tasks()[0].state, TaskState::Running);
    assert_eq!(kernel.stats().mode_escalations, 2);
    assert_eq!(kernel.stats().idle_recoveries, 1);
}

/// 2. Verifies the Per-Criticality-Partition MPU optimization:
///    Switching between two tasks in the SAME criticality partition uses the fast path
///    (does NOT reprogram the MPU partition table, only updates the stack guard),
///    whereas switching across DIFFERENT criticality partitions reprograms the MPU.
#[test]
fn test_per_criticality_partition_fast_path_context_switch() {
    const L: usize = 2;
    const N: usize = 3;

    // Tasks 1 and 2 both belong to Criticality Level 1 (HI), while Task 0 belongs to Level 0 (LO).
    let specs: [TaskSpec<L>; N] = [
        TaskSpec {
            id: TaskId(0),
            criticality: CritLevel(0),
            period: 100,
            deadline: 100,
            budgets: [20, 20],
            degrade_policy: DegradePolicy::Suspend,
            stack_bounds: StackBounds::new(0x2000_0000, 512),
        },
        TaskSpec {
            id: TaskId(1),
            criticality: CritLevel(1),
            period: 100,
            deadline: 100,
            budgets: [15, 25],
            degrade_policy: DegradePolicy::Abort,
            stack_bounds: StackBounds::new(0x2000_1000, 512),
        },
        TaskSpec {
            id: TaskId(2),
            criticality: CritLevel(1),
            period: 100,
            deadline: 100,
            budgets: [15, 25],
            degrade_policy: DegradePolicy::Abort,
            stack_bounds: StackBounds::new(0x2000_1200, 512),
        },
    ];

    let tasks = [
        TaskControlBlock::new(specs[0], AmcParams::uniform(Priority::new(5)), 0x2000_0200),
        TaskControlBlock::new(specs[1], AmcParams::uniform(Priority::new(15)), 0x2000_1200),
        TaskControlBlock::new(specs[2], AmcParams::uniform(Priority::new(25)), 0x2000_1400),
    ];

    let partitions = [
        MockPartitionConfig::new(0, 0x2000_0000, 0x1000),
        MockPartitionConfig::new(1, 0x2000_1000, 0x1000),
    ];

    let mut kernel = Kernel::new(tasks, partitions, AmcScheduler::new(), MockArch::new());

    // 1. Boot dispatches Task 2 (Crit 1): Initial MPU load (cross_crit_mpu_switches = 1)
    assert_eq!(kernel.boot(), Some(TaskId(2)));
    assert_eq!(kernel.stats().cross_crit_mpu_switches, 1);
    assert_eq!(kernel.stats().fast_same_crit_switches, 0);
    assert_eq!(kernel.arch().stack_guard_update_count, 1);

    // 2. Task 2 completes at t = 5 -> switches to Task 1 (also Crit 1!).
    // Because prev_crit (1) == next_crit (1), MPU partition table is NOT reprogrammed!
    kernel.arch_mut().jump_to(5);
    kernel.complete_current_job();
    assert_eq!(kernel.current_task(), Some(TaskId(1)));
    assert_eq!(kernel.stats().cross_crit_mpu_switches, 1);
    assert_eq!(kernel.stats().fast_same_crit_switches, 1);
    // Stack guard WAS updated for Task 1's stack:
    assert_eq!(kernel.arch().stack_guard_update_count, 2);
    assert_eq!(
        kernel.arch().active_stack_guard,
        Some(StackBounds::new(0x2000_1000, 512))
    );

    // 3. Task 1 completes at t = 12 -> switches to Task 0 (Crit 0).
    // Now criticality changes (1 -> 0), so MPU partition table IS reprogrammed!
    kernel.arch_mut().jump_to(12);
    kernel.complete_current_job();
    assert_eq!(kernel.current_task(), Some(TaskId(0)));
    assert_eq!(kernel.stats().cross_crit_mpu_switches, 2);
    assert_eq!(kernel.stats().fast_same_crit_switches, 1);
    assert_eq!(kernel.arch().stack_guard_update_count, 3);
}

/// 3. Tests the pluggable `EdfVdScheduler` (Earliest Deadline First with Virtual Deadlines):
///    Verifies that a HI-criticality task with a longer nominal deadline is prioritized early in
///    LO mode via its shortened virtual deadline, and has its deadline extended upon mode switch.
#[test]
fn test_pluggable_edf_vd_scheduler_virtual_deadline_scaling() {
    const L: usize = 2;
    const N: usize = 2;

    // Task 0 (LO): period = deadline = 25, budget = 10
    // Task 1 (HI): period = deadline = 30, budgets = [10, 20], virtual deadline in LO mode = 20 (scaled by 2/3)
    // Under standard EDF at t=0, Task 0 (D=25) would beat Task 1 (D=30).
    // Under EDF-VD in mode 0, Task 1 has virtual deadline D'_1(0) = 20 < 25, so Task 1 runs first!
    let spec_lo = TaskSpec {
        id: TaskId(0),
        criticality: CritLevel(0),
        period: 25,
        deadline: 25,
        budgets: [10, 10],
        degrade_policy: DegradePolicy::BackgroundSlack,
        stack_bounds: StackBounds::new(0x2000_0000, 512),
    };
    let spec_hi = TaskSpec {
        id: TaskId(1),
        criticality: CritLevel(1),
        period: 30,
        deadline: 30,
        budgets: [10, 20],
        degrade_policy: DegradePolicy::Abort,
        stack_bounds: StackBounds::new(0x2000_0200, 512),
    };

    let vd_lo = EdfVdParams::from_scale(CritLevel(0), 25, 2, 3); // Crit 0 -> [25, 25]
    let vd_hi = EdfVdParams::from_scale(CritLevel(1), 30, 2, 3); // Crit 1 -> [20, 30]
    assert_eq!(vd_hi.virtual_deadlines, [20, 30]);

    let tasks: [TaskControlBlock<L, EdfVdParams<L>>; N] = [
        TaskControlBlock::new(spec_lo, vd_lo, 0x2000_0200),
        TaskControlBlock::new(spec_hi, vd_hi, 0x2000_0400),
    ];
    let partitions = [
        MockPartitionConfig::new(0, 0x2000_0000, 512),
        MockPartitionConfig::new(1, 0x2000_0200, 512),
    ];

    let mut kernel = Kernel::new(tasks, partitions, EdfVdScheduler::new(), MockArch::new());

    // At t = 0, EDF-VD picks Task 1 (HI) because its virtual deadline (20) < Task 0's deadline (25)
    assert_eq!(kernel.boot(), Some(TaskId(1)));

    // At t = 10, Task 1 overruns C_1(0) = 10 -> escalates to mode 1.
    // Task 0 has DegradePolicy::BackgroundSlack, so it stays Ready in the background slack band!
    kernel.arch_mut().jump_to(10);
    let ev = kernel.on_timer_interrupt();
    assert!(matches!(ev, KernelEvent::ModeEscalated { .. }));
    assert_eq!(kernel.current_task(), Some(TaskId(1)));

    // At t = 18, Task 1 completes its job -> idle instant recovers to mode 0, Task 0 runs!
    kernel.arch_mut().jump_to(18);
    kernel.complete_current_job();
    assert_eq!(kernel.system_mode(), CritLevel(0));
    assert_eq!(kernel.current_task(), Some(TaskId(0)));
}

/// 4. Tests Wait-Free Cross-Criticality IPC (`TripleBuffer` and `SpscChannel`) and
///    Same-Criticality Immediate Priority Ceiling Protocol (`IpcpResourceSpec`).
#[test]
fn test_wait_free_cross_criticality_ipc_and_ipcp_ceiling() {
    // 4a. TripleBuffer wait-free latest-value exchange
    static SENSOR_STATE: TripleBuffer<u32> = TripleBuffer::new(0);
    unsafe {
        assert_eq!(SENSOR_STATE.read_latest(), (0, false));
        SENSOR_STATE.publish(42);
        SENSOR_STATE.publish(99); // Overwrites intermediate unread value without blocking
        assert_eq!(SENSOR_STATE.read_latest(), (99, true));
        assert_eq!(SENSOR_STATE.read_latest(), (99, false));
    }

    // 4b. SpscChannel bounded lockless event stream
    static EVENT_CHAN: SpscChannel<u16, 4> = SpscChannel::new();
    unsafe {
        assert!(EVENT_CHAN.is_empty());
        assert_eq!(EVENT_CHAN.try_send(10), Ok(()));
        assert_eq!(EVENT_CHAN.try_send(20), Ok(()));
        assert_eq!(EVENT_CHAN.try_send(30), Ok(()));
        // Ring buffer of capacity 4 holds 3 usable slots
        assert_eq!(EVENT_CHAN.try_send(40), Err(40));
        assert_eq!(EVENT_CHAN.try_recv(), Some(10));
        assert_eq!(EVENT_CHAN.try_send(40), Ok(()));
        assert_eq!(EVENT_CHAN.try_recv(), Some(20));
        assert_eq!(EVENT_CHAN.try_recv(), Some(30));
        assert_eq!(EVENT_CHAN.try_recv(), Some(40));
        assert_eq!(EVENT_CHAN.try_recv(), None);
    }

    // 4c. IPCP priority ceiling boost & cross-criticality violation prevention
    const L: usize = 2;
    const N: usize = 2;
    let specs: [TaskSpec<L>; N] = [
        TaskSpec {
            id: TaskId(0),
            criticality: CritLevel(0),
            period: 50,
            deadline: 50,
            budgets: [20, 20],
            degrade_policy: DegradePolicy::Abort,
            stack_bounds: StackBounds::new(0x2000_0000, 512),
        },
        TaskSpec {
            id: TaskId(1),
            criticality: CritLevel(0),
            period: 20,
            deadline: 20,
            budgets: [5, 5],
            degrade_policy: DegradePolicy::Abort,
            stack_bounds: StackBounds::new(0x2000_0200, 512),
        },
    ];
    let mut t0 = TaskControlBlock::new(specs[0], AmcParams::uniform(Priority::new(10)), 0x2000_0200);
    let mut t1 = TaskControlBlock::new(specs[1], AmcParams::uniform(Priority::new(20)), 0x2000_0400);
    // Delay Task 1's first arrival until t = 5 so Task 0 starts running at t = 0
    t0.next_release = 0;
    t1.next_release = 5;

    let partitions = [
        MockPartitionConfig::new(0, 0x2000_0000, 1024),
        MockPartitionConfig::new(1, 0x2000_0400, 1024),
    ];
    let mut kernel = Kernel::new([t0, t1], partitions, AmcScheduler::new(), MockArch::new());
    assert_eq!(kernel.boot(), Some(TaskId(0)));

    // Cross-criticality resource lock attempt by Task 0 (Crit 0) on a Crit 1 resource must fail:
    let hi_res = IpcpResourceSpec::new(CritLevel(1), Priority::new(30));
    assert_eq!(
        kernel.acquire_ipcp_resource(&hi_res),
        Err(IpcpError::CrossCriticalityViolation {
            task_crit: CritLevel(0),
            resource_crit: CritLevel(1),
        })
    );

    // Same-criticality IPCP resource with ceiling priority 25 (higher than Task 1's priority 20):
    let lo_shared_res = IpcpResourceSpec::new(CritLevel(0), Priority::new(25));
    let prev_boost = kernel.acquire_ipcp_resource(&lo_shared_res).unwrap();
    assert_eq!(prev_boost, None);

    // At t = 5, Task 1 (prio 20) arrives, but Task 0 holds `lo_shared_res` with ceiling 25!
    // Task 0 must NOT be preempted until it releases the resource.
    kernel.arch_mut().jump_to(5);
    kernel.on_timer_interrupt();
    assert_eq!(kernel.current_task(), Some(TaskId(0)));

    // At t = 7, Task 0 releases `lo_shared_res` -> Task 1 (prio 20) immediately preempts Task 0!
    kernel.arch_mut().jump_to(7);
    let next_after_release = kernel.release_ipcp_resource(prev_boost).unwrap();
    assert_eq!(next_after_release, Some(TaskId(1)));
}

/// 5. Tests the ARM Cortex-M backend (`CortexMArch`) end-to-end with `Kernel`:
///    - Compile-time ARMv7-M MPU region encoding (`RBAR` / `RASR`)
///    - 16-word thread stack frame initialization (`EXC_RETURN_THREAD_PSP`, `INITIAL_XPSR_THUMB`)
///    - Dynamic Region 7 stack guard updates and `PendSV` context switching
#[test]
fn test_cortex_m_backend_mpu_and_pendsv_lifecycle() {
    const L: usize = 2;
    const N: usize = 2;

    let mut stack0 = [0u32; 64];
    let mut stack1 = [0u32; 64];

    let sp0 = unsafe { CortexMArch::<L>::init_task_stack(&mut stack0, dummy_task_entry, 0xAA) };
    let sp1 = unsafe { CortexMArch::<L>::init_task_stack(&mut stack1, dummy_task_entry, 0xBB) };

    // Inspect the synthetic 16-word Cortex-M initial stack frame
    let frame0 = unsafe { *(sp0 as *const CortexMInitialFrame) };
    assert_eq!(frame0.hw.r0, 0xAA);
    assert_eq!(frame0.hw.lr, EXC_RETURN_THREAD_PSP);
    assert_eq!(frame0.hw.xpsr, INITIAL_XPSR_THUMB);
    assert_eq!(frame0.hw.pc & 1, 1, "Thumb bit must be set in initial PC");

    // Configure compile-time validated ARMv7-M MPU partition tables
    const PARTITIONS: [CortexMPartitionConfig; L] = [
        CortexMPartitionConfig::empty()
            .with_region(0, 0x0800_0000, 64 * 1024, MpuAccessPerm::ReadOnly, false)
            .with_region(1, 0x2000_0000, 16 * 1024, MpuAccessPerm::FullRw, true),
        CortexMPartitionConfig::empty()
            .with_region(0, 0x0800_0000, 64 * 1024, MpuAccessPerm::ReadOnly, false)
            .with_region(1, 0x2000_4000, 16 * 1024, MpuAccessPerm::FullRw, true)
            .with_region(2, 0x2000_0000, 16 * 1024, MpuAccessPerm::UnprivilegedRo, true),
    ];

    let specs: [TaskSpec<L>; N] = [
        TaskSpec {
            id: TaskId(0),
            criticality: CritLevel(0),
            period: 50,
            deadline: 50,
            budgets: [10, 10],
            degrade_policy: DegradePolicy::Suspend,
            stack_bounds: StackBounds::new(0x2000_0000, 256),
        },
        TaskSpec {
            id: TaskId(1),
            criticality: CritLevel(1),
            period: 50,
            deadline: 50,
            budgets: [10, 20],
            degrade_policy: DegradePolicy::Abort,
            stack_bounds: StackBounds::new(0x2000_4000, 256),
        },
    ];

    let tasks = [
        TaskControlBlock::new(specs[0], AmcParams::uniform(Priority::new(10)), sp0),
        TaskControlBlock::new(specs[1], AmcParams::uniform(Priority::new(20)), sp1),
    ];

    let mut kernel = Kernel::new(tasks, PARTITIONS, AmcScheduler::new(), CortexMArch::new());

    // Boot selects Task 1 (Crit 1): loads PARTITIONS[1] into MPU Regions 0..6 and sets Region 7 stack guard
    assert_eq!(kernel.boot(), Some(TaskId(1)));
    assert!(kernel.arch().is_pendsv_pending());
    assert_eq!(kernel.arch().pending_next_sp(), sp1);
    assert_eq!(
        kernel.arch().active_mpu_regions()[1],
        PARTITIONS[1].regions[1]
    );
    // Region 7 must be configured as a 32-byte stack guard at 0x2000_4000
    assert_eq!(
        kernel.arch().active_mpu_regions()[7].rbar & !0x1F,
        0x2000_4000
    );

    // Simulate PendSV completion and Task 1 job completion at t = 6 -> switches to Task 0 (Crit 0)
    kernel.arch_mut().complete_pendsv_switch(sp1);
    kernel.arch_mut().set_ticks(6);
    kernel.complete_current_job();

    assert_eq!(kernel.current_task(), Some(TaskId(0)));
    assert!(kernel.arch().is_pendsv_pending());
    assert_eq!(kernel.arch().pending_next_sp(), sp0);
    assert_eq!(
        kernel.arch().active_mpu_regions()[1],
        PARTITIONS[0].regions[1]
    );
    assert_eq!(
        kernel.arch().active_mpu_regions()[7].rbar & !0x1F,
        0x2000_0000
    );
}
