use mcs_arch::StackBounds;

/// Access permissions for an ARM Cortex-M MPU region (`AP[2:0]` field in `MPU_RASR`).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum MpuAccessPerm {
    /// No access at any privilege level (ideal for a red-zone stack guard region).
    NoAccess = 0b000,
    /// Privileged read/write only; unprivileged access faults (`AP = 0b001`).
    PrivilegedRw = 0b001,
    /// Privileged read/write, unprivileged read-only (`AP = 0b010` — ideal for cross-criticality IPC inspection).
    UnprivilegedRo = 0b010,
    /// Full read/write for both privileged and unprivileged Thread Mode (`AP = 0b011`).
    FullRw = 0b011,
    /// Privileged read-only, unprivileged read-only (`AP = 0b110` — ideal for shared `.rodata` / code).
    ReadOnly = 0b110,
}

/// Precomputed ARMv7-M MPU region register pair (`MPU_RBAR` and `MPU_RASR`).
///
/// Storing precomputed register words allows `switch_partition()` to reprogram up to 4–7 regions
/// with a single `STMIA` / unrolled register write sequence during a cross-criticality switch.
#[repr(C)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct Armv7mMpuRegion {
    /// Region Base Address Register value (`ADDR[31:5] | VALID (bit 4) | REGION[3:0]`).
    pub rbar: u32,
    /// Region Attribute and Size Register value (`XN | AP | TEX/S/C/B | SIZE | ENABLE`).
    pub rasr: u32,
}

impl Armv7mMpuRegion {
    /// Returns a disabled MPU region slot for `region_num` (`0..8`).
    pub const fn disabled(region_num: u8) -> Self {
        assert!(region_num < 8, "ARMv7-M standard MPU has 8 regions (0..8)");
        Self {
            rbar: (1u32 << 4) | ((region_num as u32) & 0xF),
            rasr: 0,
        }
    }

    /// Encodes a validated ARMv7-M MPU region at compile time (`const fn`).
    ///
    /// Invariants verified at compile time:
    /// - `region_num < 7` (Region 7 is reserved for the per-thread dynamic stack guard).
    /// - `size_bytes >= 32` and `size_bytes.is_power_of_two()`.
    /// - `base_addr` is naturally aligned to `size_bytes`.
    pub const fn encode(
        region_num: u8,
        base_addr: u32,
        size_bytes: u32,
        perm: MpuAccessPerm,
        execute_never: bool,
    ) -> Self {
        assert!(
            region_num < 7,
            "Regions 0..7 are for criticality partitions; Region 7 is reserved for stack guard"
        );
        assert!(
            size_bytes >= 32 && size_bytes.is_power_of_two(),
            "ARMv7-M MPU region size must be a power of two >= 32 bytes"
        );
        assert!(
            (base_addr & (size_bytes - 1)) == 0,
            "ARMv7-M MPU base_addr must be naturally aligned to size_bytes"
        );

        // SIZE field in RASR is (log2(size_bytes) - 1)
        let size_field = size_bytes.trailing_zeros() - 1;
        let xn_bit = if execute_never { 1u32 << 28 } else { 0 };
        let ap_bits = (perm as u32) << 24;
        // Normal memory, Shareable, Write-Back Write-Allocate (C=1, B=1, S=1 -> bits 18..16 = 0b111)
        let mem_attr = 0b111u32 << 16;
        let enable_bit = 1u32;

        let rbar = (base_addr & !0x1Fu32) | (1u32 << 4) | ((region_num as u32) & 0xF);
        let rasr = xn_bit | ap_bits | mem_attr | (size_field << 1) | enable_bit;

        Self { rbar, rasr }
    }

    /// Encodes Region 7 as a 32-byte `NoAccess` stack-overflow red-zone guard at `bounds.bottom`.
    #[inline]
    pub const fn encode_stack_guard(bounds: StackBounds) -> Self {
        let base_addr = (bounds.bottom as u32) & !0x1Fu32;
        // 32-byte region -> log2(32) - 1 = 4
        let size_field = 4u32;
        let xn_bit = 1u32 << 28;
        let ap_bits = (MpuAccessPerm::NoAccess as u32) << 24;
        let enable_bit = 1u32;

        Self {
            rbar: base_addr | (1u32 << 4) | 7u32,
            rasr: xn_bit | ap_bits | (size_field << 1) | enable_bit,
        }
    }
}

/// Hardware MPU configuration for a single Criticality Partition on ARM Cortex-M.
///
/// Holds up to 7 static MPU regions (Regions `0..7`) covering the partition's `.rodata`,
/// `.data`/`.bss`, shared cross-criticality IPC buffers, and MMIO peripherals.
/// Region `7` is dynamically updated per thread for stack overflow protection.
#[repr(C)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct CortexMPartitionConfig {
    pub regions: [Armv7mMpuRegion; 7],
}

impl CortexMPartitionConfig {
    /// Creates an empty partition config with all 7 partition regions disabled.
    pub const fn empty() -> Self {
        Self {
            regions: [
                Armv7mMpuRegion::disabled(0),
                Armv7mMpuRegion::disabled(1),
                Armv7mMpuRegion::disabled(2),
                Armv7mMpuRegion::disabled(3),
                Armv7mMpuRegion::disabled(4),
                Armv7mMpuRegion::disabled(5),
                Armv7mMpuRegion::disabled(6),
            ],
        }
    }

    /// Builder helper to set region `idx` (`0..7`) in `const` contexts.
    pub const fn with_region(
        mut self,
        idx: u8,
        base_addr: u32,
        size_bytes: u32,
        perm: MpuAccessPerm,
        execute_never: bool,
    ) -> Self {
        let slot = idx as usize;
        self.regions[slot] =
            Armv7mMpuRegion::encode(idx, base_addr, size_bytes, perm, execute_never);
        self
    }
}
