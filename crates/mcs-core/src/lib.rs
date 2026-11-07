#![no_std]
#![forbid(unsafe_op_in_unsafe_fn)]

pub mod ipc;
pub mod kernel;
pub mod scheduler;
pub mod task;
pub mod types;

pub use ipc::{IpcpResourceSpec, SpscChannel, TripleBuffer};
pub use kernel::{IpcpError, Kernel, KernelEvent, KernelStats};
pub use mcs_arch::{Arch, StackBounds, Ticks};
pub use scheduler::{
    AmcParams, AmcScheduler, EdfVdParams, EdfVdScheduler, Scheduler, verify_amc_rtb,
};
pub use task::TaskControlBlock;
pub use types::{CritLevel, DegradePolicy, Priority, TaskId, TaskSpec, TaskState};
