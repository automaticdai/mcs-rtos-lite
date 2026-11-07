# `mcs-rtos-lite`

A `#![no_std]`, zero-heap-allocation mixed-criticality real-time operating system (RTOS) written in Rust for highest performance and spatial/temporal safety.

## Key Features

- **Arbitrary $N$-Level Vestal Mixed-Criticality Model (`const L: usize`)**:
  - Parameterized over `L` criticality levels (`0..L`) at compile time with zero dynamic allocation.
  - Multi-step upward criticality mode escalations (`M -> M + 1`) on WCET budget exhaustion and deterministic **Idle-Instant Recovery** (`M -> 0`).
  - Configurable per-task degradation policies (`Abort`, `Suspend`, `BackgroundSlack`).
- **Pluggable `Scheduler<L, N>` Architecture**:
  - **`AmcScheduler<L, N>`**: $O(1)$ fixed-priority Adaptive Mixed-Criticality scheduler backed by a 64-bit hardware `CLZ` bitmap, plus a `const fn` offline response-time analysis verifier (`verify_amc_rtb`).
  - **`EdfVdScheduler<L, N>`**: Multi-level Earliest Deadline First with Virtual Deadlines (`EDF-VD`).
- **Per-Criticality-Partition MPU Isolation + Fast-Path Context Switching**:
  - Hardware MPU regions are reprogrammed **only** on cross-criticality switches (`prev_task.crit != next_task.crit`), keeping intra-partition switches on the fast path while always updating the per-thread stack overflow guard.
- **Cross-Criticality Wait-Free IPC & Same-Criticality IPCP**:
  - Wait-free `TripleBuffer<T>` (latest-value exchange) and `SpscChannel<T, CAP>` (lockless event ring buffer) prevent criticality and priority inversion across partitions.
  - `IpcpResourceSpec` enforces the Immediate Priority Ceiling Protocol (IPCP / SRP) within a criticality partition.

## Workspace Structure

- `crates/mcs-arch`: `#![no_std]` hardware abstraction trait (`Arch<const L: usize>`, `StackBounds`, `Ticks`).
- `crates/mcs-core`: `#![no_std]` mixed-criticality kernel, `AmcScheduler`, `EdfVdScheduler`, and wait-free IPC primitives.
- `crates/mcs-arch-cortex-m`: `#![no_std]` ARM Cortex-M (ARMv7-M / ARMv8-M) backend with compile-time MPU region encoding, Region 7 dynamic stack guard, and `PendSV` stateful thread context switching.
- `crates/mcs-arch-mock`: Deterministic discrete-event hardware simulator and integration test suite.

## Running Tests

```bash
cargo test --workspace
```
