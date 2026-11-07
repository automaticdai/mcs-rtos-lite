/// Initial `xPSR` value with the Thumb bit (`bit 24`) set, required by all ARM Cortex-M cores.
pub const INITIAL_XPSR_THUMB: u32 = 0x0100_0000;

/// `EXC_RETURN` value instructing the Cortex-M processor to return to **Thread Mode** using
/// the **Process Stack Pointer (`PSP`)** with standard 8-word integer frame stacking.
pub const EXC_RETURN_THREAD_PSP: u32 = 0xFFFF_FFFD;

/// Hardware-stacked exception frame pushed automatically onto `PSP` upon exception entry
/// (8 words = 32 bytes, 8-byte aligned per ARM AAPCS).
#[repr(C)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct CortexMHardwareFrame {
    pub r0: u32,
    pub r1: u32,
    pub r2: u32,
    pub r3: u32,
    pub r12: u32,
    pub lr: u32,
    pub pc: u32,
    pub xpsr: u32,
}

/// Software-saved callee-register frame pushed onto `PSP` by the `PendSV` handler
/// immediately below `CortexMHardwareFrame` (8 words = 32 bytes).
#[repr(C)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct CortexMSoftwareFrame {
    pub r4: u32,
    pub r5: u32,
    pub r6: u32,
    pub r7: u32,
    pub r8: u32,
    pub r9: u32,
    pub r10: u32,
    pub r11: u32,
}

/// Complete 16-word (64-byte) synthetic context frame placed at the top of a newly initialized
/// stateful thread's stack so `PendSV` can restore it identically to a preempted thread.
#[repr(C)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct CortexMInitialFrame {
    pub sw: CortexMSoftwareFrame,
    pub hw: CortexMHardwareFrame,
}

/// Initializes a Cortex-M thread stack with a synthetic 16-word context frame and returns
/// the initial Process Stack Pointer (`PSP`) address.
///
/// # Safety
/// `stack` must have at least 16 `u32` words (64 bytes) and be aligned to an 8-byte boundary.
pub unsafe fn init_cortex_m_stack(
    stack: &mut [u32],
    entry: extern "C" fn(usize) -> !,
    arg: usize,
) -> usize {
    assert!(
        stack.len() >= 16,
        "Cortex-M thread stack must be at least 16 words (64 bytes)"
    );

    // Ensure 8-byte alignment at the top of the stack per ARM AAPCS
    let top_ptr = unsafe { stack.as_mut_ptr().add(stack.len()) } as usize;
    let aligned_top = top_ptr & !0x7usize;
    let frame_ptr = (aligned_top - core::mem::size_of::<CortexMInitialFrame>()) as *mut CortexMInitialFrame;

    let frame = CortexMInitialFrame {
        sw: CortexMSoftwareFrame {
            r4: 0x0404_0404,
            r5: 0x0505_0505,
            r6: 0x0606_0606,
            r7: 0x0707_0707,
            r8: 0x0808_0808,
            r9: 0x0909_0909,
            r10: 0x1010_1010,
            r11: 0x1111_1111,
        },
        hw: CortexMHardwareFrame {
            r0: arg as u32,
            r1: 0,
            r2: 0,
            r3: 0,
            r12: 0,
            lr: EXC_RETURN_THREAD_PSP,
            pc: (entry as usize as u32) | 1, // Ensure Thumb LSB bit is set
            xpsr: INITIAL_XPSR_THUMB,
        },
    };

    unsafe {
        frame_ptr.write_volatile(frame);
    }

    frame_ptr as usize
}

/// Global mailbox used by the `PendSV` exception handler on ARM Cortex-M targets
/// to save the outgoing task's `PSP` and load the incoming task's `PSP`.
#[repr(C)]
pub struct PendSvMailbox {
    /// Pointer to `tcb.sp` of the outgoing task (or null on initial boot dispatch).
    pub prev_sp_slot: *mut usize,
    /// Incoming task's `PSP` value.
    pub next_sp: usize,
}

unsafe impl Sync for PendSvMailbox {}

#[cfg(target_arch = "arm")]
#[unsafe(no_mangle)]
pub static mut MCS_PENDSV_MAILBOX: PendSvMailbox = PendSvMailbox {
    prev_sp_slot: core::ptr::null_mut(),
    next_sp: 0,
};

/// Naked `PendSV` context-switch exception handler for ARMv7-M / ARMv8-M.
///
/// 1. Reads current Process Stack Pointer (`PSP`).
/// 2. If `MCS_PENDSV_MAILBOX.prev_sp_slot` is non-null, pushes callee-saved registers `r4-r11`
///    onto the outgoing thread's stack (`stmdb r0!, {r4-r11}`) and stores the updated `PSP`
///    into `*prev_sp_slot`.
/// 3. Loads `MCS_PENDSV_MAILBOX.next_sp`, pops `r4-r11` (`ldmia r0!, {r4-r11}`), updates `PSP`,
///    switches Thread Mode to unprivileged (`CONTROL = 0b11`), and returns via `bx lr`.
#[cfg(target_arch = "arm")]
#[unsafe(naked)]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn PendSV() {
    core::arch::naked_asm!(
        "cpsid i",
        "ldr r2, =MCS_PENDSV_MAILBOX",
        "ldr r1, [r2, #0]",       // r1 = prev_sp_slot
        "cbz r1, 1f",             // Skip saving outgoing context on initial boot dispatch
        "mrs r0, psp",
        "stmdb r0!, {{r4-r11}}",  // Push software callee-saved frame
        "str r0, [r1, #0]",       // *prev_sp_slot = updated PSP
        "movs r3, #0",
        "str r3, [r2, #0]",       // Clear prev_sp_slot
        "1:",
        "ldr r0, [r2, #4]",       // r0 = next_sp
        "ldmia r0!, {{r4-r11}}",  // Pop incoming task's r4-r11
        "msr psp, r0",            // Update hardware PSP
        "movs r3, #3",            // CONTROL: SPSEL=1 (PSP) | nPRIV=1 (Unprivileged Thread Mode)
        "msr control, r3",
        "isb",
        "cpsie i",
        "bx lr",
    );
}

