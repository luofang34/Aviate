//! Board metadata resolution from Cargo.toml
//!
//! Reads [package.metadata.aviate-board] from board crates.

// Allow dead code for future platform infrastructure
#![allow(dead_code)]

use crate::layout::ReserveMode;
use crate::programmer::Programmer;
use anyhow::{bail, Context, Result};
use cargo_metadata::{MetadataCommand, Package};
use serde::Deserialize;
use std::path::{Path, PathBuf};

/// Board metadata from Cargo.toml
#[derive(Debug, Clone)]
pub struct BoardMetadata {
    /// Board package name (e.g., "aviate-board-micoair-h743-v2")
    pub package_name: String,
    /// Path to board crate directory
    pub board_dir: PathBuf,
    /// Target chip name (probe-rs registry name for Cortex-M)
    pub chip: String,
    /// Programming method
    pub programmer: Programmer,
    /// Schema version for compatibility
    pub layout_version: u32,
    /// Reserve mode (auto or fixed)
    pub reserve_mode: ReserveMode,
    /// Fixed bootloader reserve (if reserve_mode = fixed)
    pub bootloader_reserve_bytes: Option<u32>,
    /// USB VID override
    pub vid: Option<u16>,
    /// USB PID override
    pub pid: Option<u16>,
}

/// Raw metadata as read from Cargo.toml
#[derive(Debug, Deserialize)]
struct RawBoardMetadata {
    chip: String,
    programmer: String,
    #[serde(default = "default_layout_version")]
    layout_version: u32,
    #[serde(default)]
    reserve_mode: Option<String>,
    bootloader_reserve_bytes: Option<u32>,
    bootloader_reserve_sectors: Option<u32>,
    vid: Option<String>,
    pid: Option<String>,
}

fn default_layout_version() -> u32 {
    1
}

impl BoardMetadata {
    /// Check if this is a SITL/XIL board (no flashing)
    pub fn is_simulator(&self) -> bool {
        self.chip == "xil" || self.chip == "sitl" || self.programmer == Programmer::None
    }

    /// Check if this is an STM32 board
    pub fn is_stm32(&self) -> bool {
        self.chip.starts_with("STM32")
    }
}

/// Resolve board metadata by board name
///
/// Board name can be:
/// - Full package name: "aviate-board-micoair-h743-v2"
/// - Short name: "micoair-h743-v2"
pub fn resolve_board(board_name: &str) -> Result<BoardMetadata> {
    // Normalize board name
    let package_name = if board_name.starts_with("aviate-board-") {
        board_name.to_string()
    } else {
        format!("aviate-board-{}", board_name)
    };

    // Run cargo metadata to find the package
    let metadata = MetadataCommand::new()
        .manifest_path("Cargo.toml")
        .exec()
        .context("Failed to run cargo metadata")?;

    // Find the board package
    let board_package = metadata
        .packages
        .iter()
        .find(|p| p.name == package_name)
        .ok_or_else(|| {
            // List available boards for helpful error
            let boards: Vec<_> = metadata
                .packages
                .iter()
                .filter(|p| p.name.starts_with("aviate-board-"))
                .map(|p| p.name.trim_start_matches("aviate-board-"))
                .collect();

            anyhow::anyhow!(
                "Board '{}' not found. Available boards: {}",
                board_name,
                boards.join(", ")
            )
        })?;

    parse_board_metadata(board_package)
}

/// Resolve board metadata from a path to the board's Cargo.toml
pub fn resolve_board_from_path(board_dir: &Path) -> Result<BoardMetadata> {
    let manifest_path = board_dir.join("Cargo.toml");

    let metadata = MetadataCommand::new()
        .manifest_path(&manifest_path)
        .exec()
        .with_context(|| format!("Failed to read {}", manifest_path.display()))?;

    // The first workspace member should be the board
    let board_package = metadata
        .packages
        .first()
        .ok_or_else(|| anyhow::anyhow!("No package found in {}", manifest_path.display()))?;

    parse_board_metadata(board_package)
}

/// Parse board metadata from a cargo_metadata Package
fn parse_board_metadata(package: &Package) -> Result<BoardMetadata> {
    // Get the aviate-board metadata section
    let raw: RawBoardMetadata = package
        .metadata
        .get("aviate-board")
        .ok_or_else(|| {
            anyhow::anyhow!(
                "Package {} missing [package.metadata.aviate-board] section",
                package.name
            )
        })
        .and_then(|v| {
            serde_json::from_value(v.clone()).map_err(|e| {
                anyhow::anyhow!("Invalid aviate-board metadata in {}: {}", package.name, e)
            })
        })?;

    // Parse programmer
    let programmer = Programmer::from_str(&raw.programmer)
        .with_context(|| format!("Invalid programmer in {}", package.name))?;

    // Parse reserve mode
    let reserve_mode = match raw.reserve_mode.as_deref() {
        Some("fixed") => ReserveMode::Fixed,
        Some("auto") | None => ReserveMode::Auto,
        Some(other) => bail!("Invalid reserve_mode in {}: {}", package.name, other),
    };

    // Parse VID/PID
    let vid = raw.vid.as_ref().map(|s| parse_hex_u16(s)).transpose()?;
    let pid = raw.pid.as_ref().map(|s| parse_hex_u16(s)).transpose()?;

    // Get board directory from manifest path
    let board_dir = package
        .manifest_path
        .parent()
        .ok_or_else(|| anyhow::anyhow!("Invalid manifest path"))?
        .to_path_buf()
        .into_std_path_buf();

    Ok(BoardMetadata {
        package_name: package.name.clone(),
        board_dir,
        chip: raw.chip,
        programmer,
        layout_version: raw.layout_version,
        reserve_mode,
        bootloader_reserve_bytes: raw.bootloader_reserve_bytes,
        vid,
        pid,
    })
}

/// Parse a hex string like "0x0483" or "0483" to u16
fn parse_hex_u16(s: &str) -> Result<u16> {
    let s = s.trim_start_matches("0x").trim_start_matches("0X");
    u16::from_str_radix(s, 16).map_err(|e| anyhow::anyhow!("Invalid hex value: {}", e))
}

/// List all available board names
pub fn list_boards() -> Result<Vec<String>> {
    let metadata = MetadataCommand::new()
        .manifest_path("Cargo.toml")
        .exec()
        .context("Failed to run cargo metadata")?;

    let boards: Vec<_> = metadata
        .packages
        .iter()
        .filter(|p| p.name.starts_with("aviate-board-"))
        .map(|p| p.name.trim_start_matches("aviate-board-").to_string())
        .collect();

    Ok(boards)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_hex_u16() {
        assert_eq!(parse_hex_u16("0x0483").unwrap(), 0x0483);
        assert_eq!(parse_hex_u16("0483").unwrap(), 0x0483);
        assert_eq!(parse_hex_u16("0Xdf11").unwrap(), 0xdf11);
    }
}
