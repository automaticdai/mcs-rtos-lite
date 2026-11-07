use core::cell::UnsafeCell;
use core::mem::MaybeUninit;
use core::sync::atomic::{AtomicU8, AtomicUsize, Ordering};

use crate::types::{CritLevel, Priority};

// ============================================================================
// 1. Wait-Free Triple Buffer (Latest-Value State Exchange across Criticalities)
// ============================================================================

/// Packed state bits in `TripleBuffer::state`:
/// - Bits `0..2`: index of the current shared buffer (`0`, `1`, or `2`).
/// - Bit `2` (`0x04`): `NEW_DATA_BIT` indicating the writer has published a fresh value.
const SHARED_IDX_MASK: u8 = 0b0011;
const NEW_DATA_BIT: u8 = 0b0100;

/// Wait-free Triple Buffer for passing latest-value sensor/actuation state across criticality
/// partitions (e.g., `LO -> HI` or `HI -> LO`) with **zero blocking** and **zero criticality inversion**.
///
/// Both `publish()` and `read_latest()` complete in a constant number of instructions ($O(1)$ wait-free).
pub struct TripleBuffer<T: Copy> {
    buffers: [UnsafeCell<T>; 3],
    /// Atomic state tracking the shared slot index and dirty flag.
    state: AtomicU8,
    /// Slot index currently owned by the single writer (`0..3`).
    writer_idx: UnsafeCell<u8>,
    /// Slot index currently owned by the single reader (`0..3`).
    reader_idx: UnsafeCell<u8>,
}

// SAFETY: Single producer and single consumer never access the same buffer index simultaneously
// because `writer_idx`, `reader_idx`, and `state & SHARED_IDX_MASK` are always a permutation of `{0, 1, 2}`.
unsafe impl<T: Copy + Send> Sync for TripleBuffer<T> {}
unsafe impl<T: Copy + Send> Send for TripleBuffer<T> {}

impl<T: Copy> TripleBuffer<T> {
    /// Creates a new `TripleBuffer` initialized with `initial`.
    pub const fn new(initial: T) -> Self {
        Self {
            buffers: [
                UnsafeCell::new(initial),
                UnsafeCell::new(initial),
                UnsafeCell::new(initial),
            ],
            // Initially: writer owns 0, shared is 1 (clean), reader owns 2
            state: AtomicU8::new(1),
            writer_idx: UnsafeCell::new(0),
            reader_idx: UnsafeCell::new(2),
        }
    }

    /// Publishes a new value into the triple buffer without blocking ($O(1)$ wait-free).
    ///
    /// # Safety
    /// Must only be called by the designated single producer task/ISR.
    #[inline]
    pub unsafe fn publish(&self, value: T) {
        let w_idx = unsafe { *self.writer_idx.get() };
        unsafe {
            *self.buffers[w_idx as usize].get() = value;
        }
        // Atomically swap our filled `w_idx` with the shared slot index and set `NEW_DATA_BIT`
        let prev_state = self
            .state
            .swap((w_idx & SHARED_IDX_MASK) | NEW_DATA_BIT, Ordering::AcqRel);
        unsafe {
            *self.writer_idx.get() = prev_state & SHARED_IDX_MASK;
        }
    }

    /// Reads the most recently published value without blocking ($O(1)$ wait-free).
    /// Returns `(value, is_fresh)` where `is_fresh` is `true` if a new value was published since
    /// the previous read.
    ///
    /// # Safety
    /// Must only be called by the designated single consumer task.
    #[inline]
    pub unsafe fn read_latest(&self) -> (T, bool) {
        let current = self.state.load(Ordering::Acquire);
        let is_fresh = (current & NEW_DATA_BIT) != 0;
        let r_idx = if is_fresh {
            let old_r = unsafe { *self.reader_idx.get() };
            // Swap our reader slot with the shared slot and clear `NEW_DATA_BIT`
            let prev_state = self
                .state
                .swap(old_r & SHARED_IDX_MASK, Ordering::AcqRel);
            let new_r = prev_state & SHARED_IDX_MASK;
            unsafe {
                *self.reader_idx.get() = new_r;
            }
            new_r
        } else {
            unsafe { *self.reader_idx.get() }
        };

        let val = unsafe { *self.buffers[r_idx as usize].get() };
        (val, is_fresh)
    }
}

// ============================================================================
// 2. Wait-Free SPSC Ring Buffer (Cross-Criticality Event Queue)
// ============================================================================

/// Wait-free Single-Producer Single-Consumer (SPSC) bounded ring buffer for cross-criticality
/// message passing. Never blocks a higher-criticality task.
pub struct SpscChannel<T: Copy, const CAP: usize> {
    slots: [UnsafeCell<MaybeUninit<T>>; CAP],
    head: AtomicUsize,
    tail: AtomicUsize,
}

unsafe impl<T: Copy + Send, const CAP: usize> Sync for SpscChannel<T, CAP> {}
unsafe impl<T: Copy + Send, const CAP: usize> Send for SpscChannel<T, CAP> {}

impl<T: Copy, const CAP: usize> SpscChannel<T, CAP> {
    pub const fn new() -> Self {
        assert!(CAP >= 2, "SpscChannel capacity must be at least 2");
        Self {
            slots: [const { UnsafeCell::new(MaybeUninit::uninit()) }; CAP],
            head: AtomicUsize::new(0),
            tail: AtomicUsize::new(0),
        }
    }

    /// Attempts to enqueue `item` without blocking. Returns `Err(item)` if full.
    ///
    /// # Safety
    /// Must only be called by the single producer.
    #[inline]
    pub unsafe fn try_send(&self, item: T) -> Result<(), T> {
        let head = self.head.load(Ordering::Relaxed);
        let next_head = (head + 1) % CAP;
        if next_head == self.tail.load(Ordering::Acquire) {
            return Err(item);
        }
        unsafe {
            (*self.slots[head].get()).write(item);
        }
        self.head.store(next_head, Ordering::Release);
        Ok(())
    }

    /// Attempts to dequeue an item without blocking. Returns `None` if empty.
    ///
    /// # Safety
    /// Must only be called by the single consumer.
    #[inline]
    pub unsafe fn try_recv(&self) -> Option<T> {
        let tail = self.tail.load(Ordering::Relaxed);
        if tail == self.head.load(Ordering::Acquire) {
            return None;
        }
        let item = unsafe { (*self.slots[tail].get()).assume_init_read() };
        let next_tail = (tail + 1) % CAP;
        self.tail.store(next_tail, Ordering::Release);
        Some(item)
    }

    /// Returns the number of queued elements.
    #[inline]
    pub fn len(&self) -> usize {
        let head = self.head.load(Ordering::Acquire);
        let tail = self.tail.load(Ordering::Acquire);
        if head >= tail {
            head - tail
        } else {
            CAP - tail + head
        }
    }

    #[inline]
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

// ============================================================================
// 3. Same-Criticality Immediate Priority Ceiling Protocol (IPCP / SRP)
// ============================================================================

/// Descriptor for a shared resource protected by the Immediate Priority Ceiling Protocol (IPCP).
///
/// Only tasks belonging to `partition_crit` (or a permitted criticality band) may acquire the resource,
/// immediately boosting the acquiring task's effective priority to `ceiling_priority` to prevent
/// deadlock and bound blocking time to at most a single critical section.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct IpcpResourceSpec {
    /// Criticality partition owning this resource.
    pub partition_crit: CritLevel,
    /// Highest static priority among all tasks that may access this resource.
    pub ceiling_priority: Priority,
}

impl IpcpResourceSpec {
    pub const fn new(partition_crit: CritLevel, ceiling_priority: Priority) -> Self {
        Self {
            partition_crit,
            ceiling_priority,
        }
    }
}
