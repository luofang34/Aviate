//! Flash layout policy and lock file management
//!
//! Determines where bootloader and app are placed in flash.

// Allow dead code for future platform infrastructure (ESP32, RP2040)
#![allow(dead_code)]

use crate::geometry::FlashGeometry;
use anyhow::{bail, Context, Result};
use serde::{Deserialize, Serialize};
use std::fs;
use std::path::Path;

/// Computed flash layout
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FlashLayout {
    /// Start address of bootloader (usually flash_base)
    pub bootloader_start: u32,
    /// Reserved space for bootloader (aligned to sector boundary)
    pub bootloader_reserve: u32,
    /// Start address of application
    pub app_start: u32,
}

/// Layout lock file format (stored as JSON)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LayoutLock {
    /// Schema version for forward compatibility
    pub layout_version: u32,
    /// Flash base address (hex string for readability)
    #[serde(with = "hex_u32")]
    pub flash_base: u32,
    /// Bootloader reserve size (hex string)
    #[serde(with = "hex_u32")]
    pub bootloader_reserve: u32,
    /// Application start address (hex string)
    #[serde(with = "hex_u32")]
    pub app_start: u32,
    /// Bootloader size this lock was computed from
    pub computed_from_bootloader_size: u32,
}

/// Reserve mode for layout computation
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ReserveMode {
    /// Compute reserve from bootloader size, grow lock if needed
    #[default]
    Auto,
    /// Use fixed reserve from lock, error if bootloader outgrows it
    Fixed,
}

/// Policy for computing flash layout
pub trait LayoutPolicy {
    /// Compute layout given flash geometry and bootloader size
    fn compute_layout(&self, geometry: &FlashGeometry, bootloader_size: u32)
        -> Result<FlashLayout>;
}

/// Default for Cortex-M internal flash: reserve minimum erasable span
pub struct BootloaderInFirstSectors;

impl LayoutPolicy for BootloaderInFirstSectors {
    fn compute_layout(&self, geo: &FlashGeometry, bootloader_size: u32) -> Result<FlashLayout> {
        let mut reserve = 0u32;

        for sector in &geo.sectors {
            reserve += sector.size;
            if reserve >= bootloader_size {
                break;
            }
        }

        // Safety: if bootloader won't fit, error
        if reserve < bootloader_size {
            bail!(
                "Bootloader ({} bytes) won't fit in flash ({} bytes)",
                bootloader_size,
                geo.flash_size
            );
        }

        Ok(FlashLayout {
            bootloader_start: geo.flash_base,
            bootloader_reserve: reserve,
            app_start: geo.flash_base + reserve,
        })
    }
}

/// ESP32: partition-table driven layout (fixed offsets)
pub struct Esp32PartitionTable {
    pub bootloader_offset: u32,      // Usually 0x1000
    pub partition_table_offset: u32, // Usually 0x8000
    pub app_offset: u32,             // Usually 0x10000
}

impl LayoutPolicy for Esp32PartitionTable {
    fn compute_layout(&self, _geo: &FlashGeometry, _bootloader_size: u32) -> Result<FlashLayout> {
        Ok(FlashLayout {
            bootloader_start: self.bootloader_offset,
            bootloader_reserve: self.app_offset - self.bootloader_offset,
            app_start: self.app_offset,
        })
    }
}

/// RP2040: UF2 conventions (external QSPI flash)
/// Note: "your bootloader" vs "ROM boot + boot2" are distinct concepts
/// Boot2 (256 bytes) is a stage2 loader that lives at flash base
pub struct Rp2040Uf2Convention {
    pub app_offset: u32, // Typically 0x100 after stage2 boot2
}

impl LayoutPolicy for Rp2040Uf2Convention {
    fn compute_layout(&self, _geo: &FlashGeometry, _bootloader_size: u32) -> Result<FlashLayout> {
        // RP2040 doesn't have a separate bootloader in our sense
        // The app starts after the 256-byte boot2 stage
        Ok(FlashLayout {
            bootloader_start: 0,
            bootloader_reserve: self.app_offset,
            app_start: self.app_offset,
        })
    }
}

/// Validate that bootloader fits within reserved space
pub fn validate_layout(layout: &FlashLayout, bootloader_size: u32) -> Result<()> {
    if bootloader_size > layout.bootloader_reserve {
        bail!(
            "Bootloader ({:#x} bytes) exceeds reserved space ({:#x} bytes)!\n\
             Either shrink bootloader or adjust layout.",
            bootloader_size,
            layout.bootloader_reserve
        );
    }
    Ok(())
}

/// Load layout lock from a board directory
pub fn load_layout_lock(board_dir: &Path) -> Result<Option<LayoutLock>> {
    let lock_path = board_dir.join("layout.lock.json");

    if !lock_path.exists() {
        return Ok(None);
    }

    let content = fs::read_to_string(&lock_path)
        .with_context(|| format!("Failed to read {}", lock_path.display()))?;

    let lock: LayoutLock = serde_json::from_str(&content)
        .with_context(|| format!("Failed to parse {}", lock_path.display()))?;

    Ok(Some(lock))
}

/// Save layout lock to a board directory
pub fn save_layout_lock(board_dir: &Path, lock: &LayoutLock) -> Result<()> {
    let lock_path = board_dir.join("layout.lock.json");

    let content = serde_json::to_string_pretty(lock)?;
    fs::write(&lock_path, content)
        .with_context(|| format!("Failed to write {}", lock_path.display()))?;

    Ok(())
}

/// Resolve layout with lock file management
///
/// Rules:
/// - If lock exists and reserve is sufficient: use lock (stable addresses)
/// - If lock exists but reserve is insufficient:
///   - Auto mode: grow lock and warn
///   - Fixed mode: error
/// - If no lock: compute and create lock
pub fn resolve_layout_with_lock(
    geometry: &FlashGeometry,
    bootloader_size: u32,
    board_dir: &Path,
    reserve_mode: ReserveMode,
) -> Result<FlashLayout> {
    let policy = BootloaderInFirstSectors;
    let needed_layout = policy.compute_layout(geometry, bootloader_size)?;

    if let Some(existing_lock) = load_layout_lock(board_dir)? {
        // Lock exists - check if it's still valid
        if needed_layout.bootloader_reserve <= existing_lock.bootloader_reserve {
            // Lock is sufficient, use it for stable addresses
            return Ok(FlashLayout {
                bootloader_start: existing_lock.flash_base,
                bootloader_reserve: existing_lock.bootloader_reserve,
                app_start: existing_lock.app_start,
            });
        }

        // Bootloader has grown beyond lock
        match reserve_mode {
            ReserveMode::Auto => {
                eprintln!(
                    "Warning: Bootloader ({} bytes) outgrew locked reserve ({:#x}). \
                     Growing lock to {:#x}.",
                    bootloader_size,
                    existing_lock.bootloader_reserve,
                    needed_layout.bootloader_reserve
                );

                let new_lock = LayoutLock {
                    layout_version: 1,
                    flash_base: geometry.flash_base,
                    bootloader_reserve: needed_layout.bootloader_reserve,
                    app_start: needed_layout.app_start,
                    computed_from_bootloader_size: bootloader_size,
                };
                save_layout_lock(board_dir, &new_lock)?;

                Ok(needed_layout)
            }
            ReserveMode::Fixed => {
                bail!(
                    "Bootloader ({} bytes) exceeds fixed reserved space ({:#x} bytes)!\n\
                     Either shrink bootloader or increase bootloader_reserve_bytes in board metadata.",
                    bootloader_size,
                    existing_lock.bootloader_reserve
                );
            }
        }
    } else {
        // No lock exists, create one
        let new_lock = LayoutLock {
            layout_version: 1,
            flash_base: geometry.flash_base,
            bootloader_reserve: needed_layout.bootloader_reserve,
            app_start: needed_layout.app_start,
            computed_from_bootloader_size: bootloader_size,
        };
        save_layout_lock(board_dir, &new_lock)?;

        eprintln!(
            "Created layout.lock.json: app_start = {:#x}",
            needed_layout.app_start
        );

        Ok(needed_layout)
    }
}

/// Get bootloader size from a built binary
pub fn get_bootloader_size(bin_path: &Path) -> Result<u32> {
    let metadata = fs::metadata(bin_path)
        .with_context(|| format!("Failed to read bootloader binary: {}", bin_path.display()))?;

    Ok(metadata.len() as u32)
}

/// Custom serde module for hex u32 serialization
mod hex_u32 {
    use serde::{self, Deserialize, Deserializer, Serializer};

    pub fn serialize<S>(value: &u32, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        serializer.serialize_str(&format!("{:#x}", value))
    }

    pub fn deserialize<'de, D>(deserializer: D) -> Result<u32, D::Error>
    where
        D: Deserializer<'de>,
    {
        let s = String::deserialize(deserializer)?;
        let s = s.trim_start_matches("0x").trim_start_matches("0X");
        u32::from_str_radix(s, 16).map_err(serde::de::Error::custom)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::SectorInfo;

    fn make_uniform_geometry(sector_size: u32, count: u32) -> FlashGeometry {
        let sectors: Vec<_> = (0..count)
            .map(|i| SectorInfo {
                address: 0x08000000 + i * sector_size,
                size: sector_size,
            })
            .collect();

        FlashGeometry {
            flash_base: 0x08000000,
            flash_size: sector_size * count,
            sectors,
        }
    }

    #[test]
    fn test_bootloader_in_first_sectors_uniform() {
        // STM32H7: 128KB sectors
        let geo = make_uniform_geometry(0x20000, 16);
        let policy = BootloaderInFirstSectors;

        // 30KB bootloader fits in one 128KB sector
        let layout = policy.compute_layout(&geo, 30 * 1024).unwrap();
        assert_eq!(layout.bootloader_reserve, 0x20000);
        assert_eq!(layout.app_start, 0x08020000);

        // 200KB bootloader needs two sectors
        let layout = policy.compute_layout(&geo, 200 * 1024).unwrap();
        assert_eq!(layout.bootloader_reserve, 0x40000);
        assert_eq!(layout.app_start, 0x08040000);
    }

    #[test]
    fn test_bootloader_in_first_sectors_mixed() {
        // STM32F4: mixed sectors (16KB x4, 64KB x1, 128KB x7)
        let sectors = vec![
            SectorInfo {
                address: 0x08000000,
                size: 0x4000,
            },
            SectorInfo {
                address: 0x08004000,
                size: 0x4000,
            },
            SectorInfo {
                address: 0x08008000,
                size: 0x4000,
            },
            SectorInfo {
                address: 0x0800C000,
                size: 0x4000,
            },
            SectorInfo {
                address: 0x08010000,
                size: 0x10000,
            },
            SectorInfo {
                address: 0x08020000,
                size: 0x20000,
            },
        ];

        let geo = FlashGeometry {
            flash_base: 0x08000000,
            flash_size: 0x100000,
            sectors,
        };

        let policy = BootloaderInFirstSectors;

        // 30KB bootloader needs 16+16 = 32KB (sectors 0-1) - wait, that's only 32KB
        // Actually: 16+16+16 = 48KB to cover 30KB
        let layout = policy.compute_layout(&geo, 30 * 1024).unwrap();
        // 30KB = 30720 bytes, needs sectors until cumulative >= 30720
        // Sector 0: 16KB (16384), cumulative = 16384 < 30720
        // Sector 1: 16KB, cumulative = 32768 > 30720
        assert_eq!(layout.bootloader_reserve, 0x8000); // 32KB
        assert_eq!(layout.app_start, 0x08008000);
    }

    #[test]
    fn test_validate_layout() {
        let layout = FlashLayout {
            bootloader_start: 0x08000000,
            bootloader_reserve: 0x20000,
            app_start: 0x08020000,
        };

        assert!(validate_layout(&layout, 30 * 1024).is_ok());
        assert!(validate_layout(&layout, 0x20000).is_ok());
        assert!(validate_layout(&layout, 0x20001).is_err());
    }

    #[test]
    fn test_layout_lock_serialization() {
        let lock = LayoutLock {
            layout_version: 1,
            flash_base: 0x08000000,
            bootloader_reserve: 0x20000,
            app_start: 0x08020000,
            computed_from_bootloader_size: 30552,
        };

        let json = serde_json::to_string_pretty(&lock).unwrap();
        assert!(json.contains("\"flash_base\": \"0x8000000\""));
        assert!(json.contains("\"app_start\": \"0x8020000\""));

        let parsed: LayoutLock = serde_json::from_str(&json).unwrap();
        assert_eq!(parsed.flash_base, 0x08000000);
        assert_eq!(parsed.app_start, 0x08020000);
    }
}
