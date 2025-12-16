//! Flash plan abstraction for multi-segment flashing
//!
//! Supports single-segment (app-only) and multi-segment (bootloader + app) flashing.

// Allow dead code for future platform infrastructure (ESP32, RP2040)
#![allow(dead_code)]

use crate::layout::FlashLayout;
use std::path::PathBuf;

/// Kind of flash segment
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SegmentKind {
    /// Bootloader binary
    Bootloader,
    /// Application binary
    App,
    /// ESP32 partition table
    PartitionTable,
    /// ESP32 OTA data
    OtaData,
    /// RP2040 stage2 boot loader (256 bytes) - distinct from "your bootloader"
    Boot2,
}

impl std::fmt::Display for SegmentKind {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            SegmentKind::Bootloader => write!(f, "Bootloader"),
            SegmentKind::App => write!(f, "App"),
            SegmentKind::PartitionTable => write!(f, "PartitionTable"),
            SegmentKind::OtaData => write!(f, "OtaData"),
            SegmentKind::Boot2 => write!(f, "Boot2"),
        }
    }
}

/// A segment to flash
#[derive(Debug, Clone)]
pub struct Segment {
    /// Path to binary file
    pub path: PathBuf,
    /// Flash address
    pub address: u32,
    /// Kind of segment
    pub kind: SegmentKind,
}

/// Plan for flashing one or more segments
#[derive(Debug, Clone)]
pub struct FlashPlan {
    /// Segments to flash, in order
    pub segments: Vec<Segment>,
}

impl FlashPlan {
    /// Create an empty flash plan
    pub fn new() -> Self {
        Self {
            segments: Vec::new(),
        }
    }

    /// Add a segment to the plan
    pub fn add_segment(&mut self, path: PathBuf, address: u32, kind: SegmentKind) {
        self.segments.push(Segment {
            path,
            address,
            kind,
        });
    }

    /// STM32: 2 segments (bootloader + app)
    pub fn stm32_full(bootloader: PathBuf, app: PathBuf, layout: &FlashLayout) -> Self {
        Self {
            segments: vec![
                Segment {
                    path: bootloader,
                    address: layout.bootloader_start,
                    kind: SegmentKind::Bootloader,
                },
                Segment {
                    path: app,
                    address: layout.app_start,
                    kind: SegmentKind::App,
                },
            ],
        }
    }

    /// App-only flash (when bootloader already installed)
    pub fn app_only(app: PathBuf, app_address: u32) -> Self {
        Self {
            segments: vec![Segment {
                path: app,
                address: app_address,
                kind: SegmentKind::App,
            }],
        }
    }

    /// RP2040: app only (boot2 is embedded in UF2/binary)
    pub fn rp2040(app: PathBuf, app_address: u32) -> Self {
        // For RP2040, the app binary typically includes boot2
        // We just flash it at the base address
        Self::app_only(app, app_address)
    }

    /// ESP32: 3+ segments (bootloader + partition table + app)
    pub fn esp32(
        bootloader: PathBuf,
        partition_table: PathBuf,
        app: PathBuf,
        bootloader_offset: u32,
        partition_table_offset: u32,
        app_offset: u32,
    ) -> Self {
        Self {
            segments: vec![
                Segment {
                    path: bootloader,
                    address: bootloader_offset,
                    kind: SegmentKind::Bootloader,
                },
                Segment {
                    path: partition_table,
                    address: partition_table_offset,
                    kind: SegmentKind::PartitionTable,
                },
                Segment {
                    path: app,
                    address: app_offset,
                    kind: SegmentKind::App,
                },
            ],
        }
    }

    /// Check if plan contains a bootloader segment
    pub fn has_bootloader(&self) -> bool {
        self.segments
            .iter()
            .any(|s| s.kind == SegmentKind::Bootloader)
    }

    /// Get the app segment if present
    pub fn app_segment(&self) -> Option<&Segment> {
        self.segments.iter().find(|s| s.kind == SegmentKind::App)
    }

    /// Validate that all segment paths exist
    pub fn validate_paths(&self) -> anyhow::Result<()> {
        for seg in &self.segments {
            if !seg.path.exists() {
                anyhow::bail!("{} binary not found: {}", seg.kind, seg.path.display());
            }
        }
        Ok(())
    }
}

impl Default for FlashPlan {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_stm32_full_plan() {
        let layout = crate::layout::FlashLayout {
            bootloader_start: 0x08000000,
            bootloader_reserve: 0x20000,
            app_start: 0x08020000,
        };

        let plan = FlashPlan::stm32_full(
            PathBuf::from("/tmp/bootloader.bin"),
            PathBuf::from("/tmp/app.bin"),
            &layout,
        );

        assert_eq!(plan.segments.len(), 2);
        assert!(plan.has_bootloader());

        assert_eq!(plan.segments[0].kind, SegmentKind::Bootloader);
        assert_eq!(plan.segments[0].address, 0x08000000);

        assert_eq!(plan.segments[1].kind, SegmentKind::App);
        assert_eq!(plan.segments[1].address, 0x08020000);
    }

    #[test]
    fn test_app_only_plan() {
        let plan = FlashPlan::app_only(PathBuf::from("/tmp/app.bin"), 0x08020000);

        assert_eq!(plan.segments.len(), 1);
        assert!(!plan.has_bootloader());

        let app = plan.app_segment().unwrap();
        assert_eq!(app.address, 0x08020000);
    }

    #[test]
    fn test_esp32_plan() {
        let plan = FlashPlan::esp32(
            PathBuf::from("/tmp/bootloader.bin"),
            PathBuf::from("/tmp/partitions.bin"),
            PathBuf::from("/tmp/app.bin"),
            0x1000,
            0x8000,
            0x10000,
        );

        assert_eq!(plan.segments.len(), 3);
        assert!(plan.has_bootloader());

        assert_eq!(plan.segments[0].kind, SegmentKind::Bootloader);
        assert_eq!(plan.segments[0].address, 0x1000);

        assert_eq!(plan.segments[1].kind, SegmentKind::PartitionTable);
        assert_eq!(plan.segments[1].address, 0x8000);

        assert_eq!(plan.segments[2].kind, SegmentKind::App);
        assert_eq!(plan.segments[2].address, 0x10000);
    }
}
