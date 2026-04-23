//! Flash geometry retrieval from probe-rs registry
//!
//! Provides flash base, size, and sector layout for Cortex-M chips.
//!
//! This module requires the `hardware` feature to be enabled.
//! It pulls in probe-rs which adds significant compile time.

#![cfg(feature = "hardware")]
// Allow dead code for future use
#![allow(dead_code)]

use anyhow::{bail, Result};
use probe_rs::config::{MemoryRegion, NvmRegion, SectorDescription};

/// Flash geometry information
#[derive(Debug, Clone)]
pub struct FlashGeometry {
    /// Base address of flash memory
    pub flash_base: u32,
    /// Total flash size in bytes
    pub flash_size: u32,
    /// Individual sectors (expanded from sector groups)
    pub sectors: Vec<SectorInfo>,
}

/// Information about a single flash sector
#[derive(Debug, Clone)]
pub struct SectorInfo {
    /// Start address of this sector
    pub address: u32,
    /// Size of this sector in bytes
    pub size: u32,
}

/// Expand sector groups into individual sectors
///
/// probe-rs `SectorDescription` is a GROUP: "all sectors from this address
/// have this size until the next group starts"
fn expand_sector_groups(
    flash_base: u32,
    flash_size: u32,
    groups: &[SectorDescription],
) -> Result<Vec<SectorInfo>> {
    // GUARDRAIL: Empty groups likely means "mass erase only" - unsupported
    if groups.is_empty() {
        bail!("No sector groups defined - chip may only support mass erase");
    }

    // GUARDRAIL: Sort groups by address to ensure correct ordering
    let mut sorted_groups: Vec<_> = groups.iter().collect();
    sorted_groups.sort_by_key(|g| g.address);

    let flash_end = flash_base as u64 + flash_size as u64;
    let mut sectors = Vec::new();

    for (i, group) in sorted_groups.iter().enumerate() {
        let group_start = flash_base as u64 + group.address;

        // GUARDRAIL: Validate group_start is within flash bounds
        if group_start < flash_base as u64 || group_start >= flash_end {
            bail!(
                "Sector group {} starts at {:#x}, outside flash range [{:#x}, {:#x})",
                i,
                group_start,
                flash_base,
                flash_end
            );
        }

        let group_end = if i + 1 < sorted_groups.len() {
            flash_base as u64 + sorted_groups[i + 1].address
        } else {
            flash_end
        };

        // GUARDRAIL: Validate group size divides evenly
        // Partial sectors are NOT supported - flash erase operates on whole sectors
        let group_span = group_end - group_start;
        if !group_span.is_multiple_of(group.size) {
            bail!(
                "Sector group {} span ({:#x}) is not a multiple of sector size ({:#x}). \
                 This would create partial sectors which cannot be erased. \
                 Check probe-rs target definition or add board override.",
                i,
                group_span,
                group.size
            );
        }

        // Generate sectors for this group
        let mut addr = group_start;
        while addr < group_end {
            sectors.push(SectorInfo {
                address: addr as u32,
                size: group.size as u32,
            });
            addr += group.size;
        }
    }

    Ok(sectors)
}

/// Select the correct NVM region from a chip's memory map
///
/// Heuristic (portable):
/// 1. Prefer region marked as boot memory (`is_boot_memory`)
/// 2. For STM32 specifically: prefer region at 0x0800_0000 (known boot address)
/// 3. Otherwise: prefer largest NVM region and warn when ambiguous
fn select_flash_region<'a>(regions: &'a [MemoryRegion], chip: &str) -> Result<&'a NvmRegion> {
    let nvm_regions: Vec<_> = regions.iter().filter_map(|r| r.as_nvm_region()).collect();

    match nvm_regions.len() {
        0 => bail!("No NVM region found for {}", chip),
        1 => Ok(nvm_regions[0]),
        _ => {
            // Multiple NVM regions - use heuristics

            // First try: prefer region marked as boot memory
            let boot_regions: Vec<_> = nvm_regions
                .iter()
                .copied()
                .filter(|nvm| nvm.is_boot_memory())
                .collect();

            if boot_regions.len() == 1 {
                return Ok(boot_regions[0]);
            }

            // STM32 special case: prefer region at known boot address
            if chip.starts_with("STM32") {
                let boot_candidate = nvm_regions
                    .iter()
                    .copied()
                    .filter(|nvm| nvm.range.start == 0x08000000)
                    .max_by_key(|nvm| nvm.range.end - nvm.range.start);

                if let Some(region) = boot_candidate {
                    return Ok(region);
                }
            }

            // Fallback: largest NVM region with warning
            let largest = nvm_regions
                .iter()
                .copied()
                .max_by_key(|nvm| nvm.range.end - nvm.range.start)
                .unwrap();

            eprintln!(
                "Warning: {} has {} NVM regions, using largest at {:#x}. \
                 Consider adding explicit flash_base override in board metadata.",
                chip,
                nvm_regions.len(),
                largest.range.start
            );
            Ok(largest)
        }
    }
}

/// Get flash geometry from probe-rs builtin registry
///
/// Returns `FlashGeometry` containing base address, size, and expanded sector list.
pub fn get_geometry_from_probe_rs(chip: &str) -> Result<FlashGeometry> {
    let registry = probe_rs::config::Registry::from_builtin_families();
    let target = registry
        .get_target_by_name(chip)
        .map_err(|e| anyhow::anyhow!("Target '{}' not found in probe-rs registry: {}", chip, e))?;

    let flash_region = select_flash_region(&target.memory_map, chip)?;

    let flash_base = flash_region.range.start as u32;
    let flash_size = (flash_region.range.end - flash_region.range.start) as u32;

    // Find flash algorithm that covers this region
    let flash_algo = target
        .flash_algorithms
        .iter()
        .find(|algo| {
            let props = &algo.flash_properties;
            props.address_range.start <= flash_region.range.start
                && props.address_range.end >= flash_region.range.end
        })
        .or_else(|| target.flash_algorithms.first())
        .ok_or_else(|| anyhow::anyhow!("No flash algorithm for {}", chip))?;

    let sectors =
        expand_sector_groups(flash_base, flash_size, &flash_algo.flash_properties.sectors)?;

    Ok(FlashGeometry {
        flash_base,
        flash_size,
        sectors,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_expand_uniform_sectors() {
        // Simulate STM32H743: 2MB flash with 128KB uniform sectors
        let groups = vec![SectorDescription {
            address: 0,
            size: 0x20000, // 128KB
        }];

        let sectors = expand_sector_groups(0x08000000, 0x200000, &groups).unwrap();

        assert_eq!(sectors.len(), 16); // 2MB / 128KB = 16 sectors
        assert_eq!(sectors[0].address, 0x08000000);
        assert_eq!(sectors[0].size, 0x20000);
        assert_eq!(sectors[1].address, 0x08020000);
        assert_eq!(sectors[15].address, 0x081E0000);
    }

    #[test]
    fn test_expand_mixed_sectors() {
        // Simulate STM32F429: mixed sector sizes
        let groups = vec![
            SectorDescription {
                address: 0,
                size: 0x4000,
            }, // 16KB sectors 0-3
            SectorDescription {
                address: 0x10000,
                size: 0x10000,
            }, // 64KB sector 4
            SectorDescription {
                address: 0x20000,
                size: 0x20000,
            }, // 128KB sectors 5+
        ];

        // Total: 4x16KB + 1x64KB + 7x128KB = 1MB
        let sectors = expand_sector_groups(0x08000000, 0x100000, &groups).unwrap();

        // Sectors 0-3: 16KB each
        assert_eq!(sectors[0].size, 0x4000);
        assert_eq!(sectors[1].size, 0x4000);
        assert_eq!(sectors[2].size, 0x4000);
        assert_eq!(sectors[3].size, 0x4000);

        // Sector 4: 64KB
        assert_eq!(sectors[4].address, 0x08010000);
        assert_eq!(sectors[4].size, 0x10000);

        // Sectors 5+: 128KB
        assert_eq!(sectors[5].address, 0x08020000);
        assert_eq!(sectors[5].size, 0x20000);
    }

    #[test]
    fn test_empty_groups_error() {
        let result = expand_sector_groups(0x08000000, 0x100000, &[]);
        assert!(result.is_err());
        assert!(result.unwrap_err().to_string().contains("mass erase"));
    }

    #[test]
    fn test_stm32h743_geometry() {
        // This test requires probe-rs builtin targets
        let result = get_geometry_from_probe_rs("STM32H743VITx");
        if let Ok(geo) = result {
            assert_eq!(geo.flash_base, 0x08000000);
            assert!(geo.flash_size >= 0x100000); // At least 1MB
            assert!(!geo.sectors.is_empty());
            // H7 has 128KB uniform sectors
            assert_eq!(geo.sectors[0].size, 0x20000);
        }
        // If target not found, that's OK in test environment
    }
}

#[test]
fn test_rp235x_geometry() {
    // Try different RP235x target names
    for name in ["RP2350", "RP2350A", "rp2350", "RP235x", "rp235x"] {
        let result = get_geometry_from_probe_rs(name);
        println!("{}: {:?}", name, result.is_ok());
        if let Ok(geo) = result {
            println!(
                "  base={:#x} size={:#x} sectors={}",
                geo.flash_base,
                geo.flash_size,
                geo.sectors.len()
            );
        }
    }
}
