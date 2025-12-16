//! Build script for aviate-bootloader
//!
//! Dynamically generates memory.x linker script from probe-rs chip registry.
//! This ensures memory layout is always correct without maintaining separate files.

use std::env;
use std::fs;
use std::io::Write;
use std::path::PathBuf;

use anyhow::{bail, Result};
use probe_rs::config::{MemoryRegion, Registry};

/// Chip family for platform-specific linker script sections
#[derive(Clone, Copy, PartialEq)]
enum ChipFamily {
    Stm32,
    Rp2350,
}

/// Chip configuration mapping feature flags to probe-rs target names
struct ChipConfig {
    /// probe-rs target name
    target_name: &'static str,
    /// Expected flash base address (for validation)
    expected_flash_base: u32,
    /// RAM region selector (some chips have multiple RAM regions)
    ram_selector: RamSelector,
    /// Bootloader flash size limit (we don't need full flash)
    bootloader_flash_limit: u32,
    /// Chip family for platform-specific linker sections
    family: ChipFamily,
}

/// How to select RAM region when chip has multiple
enum RamSelector {
    /// Use largest RAM region
    Largest,
    /// Use RAM region at specific address
    AtAddress(u64),
}

fn get_chip_config() -> Result<ChipConfig> {
    // Check which chip feature is enabled via env vars
    // Note: Use env vars, not cfg!(), because cfg! is evaluated at build script compile time
    let chip_stm32h743 = env::var("CARGO_FEATURE_CHIP_STM32H743").is_ok();
    let chip_rp2350 = env::var("CARGO_FEATURE_CHIP_RP2350").is_ok();

    if chip_stm32h743 {
        Ok(ChipConfig {
            target_name: "STM32H743VITx",
            expected_flash_base: 0x0800_0000,
            // STM32H7 has multiple RAM regions, prefer D1 AXI SRAM at 0x24000000
            ram_selector: RamSelector::AtAddress(0x2400_0000),
            bootloader_flash_limit: 128 * 1024, // 128KB for bootloader
            family: ChipFamily::Stm32,
        })
    } else if chip_rp2350 {
        Ok(ChipConfig {
            target_name: "RP2350",
            expected_flash_base: 0x1000_0000,
            ram_selector: RamSelector::Largest,
            bootloader_flash_limit: 256 * 1024, // 256KB for bootloader
            family: ChipFamily::Rp2350,
        })
    } else {
        bail!(
            "No chip selected! Enable exactly one chip-* feature.\n\
             Available: chip-stm32h743, chip-rp2350"
        );
    }
}

/// Extract flash (NVM) region from probe-rs target
fn get_flash_region(
    memory_map: &[MemoryRegion],
    chip: &str,
    expected_base: u32,
) -> Result<(u64, u64)> {
    let nvm_regions: Vec<_> = memory_map
        .iter()
        .filter_map(|r| r.as_nvm_region())
        .collect();

    if nvm_regions.is_empty() {
        bail!("No NVM (flash) region found for {}", chip);
    }

    // Prefer boot memory region if available
    let boot_region = nvm_regions.iter().find(|r| r.is_boot_memory());

    // Or region at expected address
    let expected_region = nvm_regions
        .iter()
        .find(|r| r.range.start == expected_base as u64);

    // Or largest
    let largest_region = nvm_regions
        .iter()
        .max_by_key(|r| r.range.end - r.range.start);

    let region = boot_region
        .or(expected_region)
        .or(largest_region)
        .ok_or_else(|| anyhow::anyhow!("No suitable flash region for {}", chip))?;

    let base = region.range.start;
    let size = region.range.end - region.range.start;

    // Validate against expected base
    if base != expected_base as u64 {
        eprintln!(
            "Warning: {} flash base {:#x} differs from expected {:#x}",
            chip, base, expected_base
        );
    }

    Ok((base, size))
}

/// Extract RAM region from probe-rs target
fn get_ram_region(
    memory_map: &[MemoryRegion],
    chip: &str,
    selector: &RamSelector,
) -> Result<(u64, u64)> {
    let ram_regions: Vec<_> = memory_map
        .iter()
        .filter_map(|r| r.as_ram_region())
        .collect();

    if ram_regions.is_empty() {
        bail!("No RAM region found for {}", chip);
    }

    let region = match selector {
        RamSelector::Largest => ram_regions
            .iter()
            .max_by_key(|r| r.range.end - r.range.start),
        RamSelector::AtAddress(addr) => {
            // First try exact match, then closest
            ram_regions
                .iter()
                .find(|r| r.range.start == *addr)
                .or_else(|| {
                    ram_regions
                        .iter()
                        .max_by_key(|r| r.range.end - r.range.start)
                })
        }
    };

    let region = region.ok_or_else(|| anyhow::anyhow!("No suitable RAM region for {}", chip))?;

    Ok((region.range.start, region.range.end - region.range.start))
}

/// Generate memory.x linker script content
fn generate_memory_x(
    flash_base: u64,
    flash_size: u64,
    ram_base: u64,
    ram_size: u64,
    bootloader_flash_limit: u32,
    family: ChipFamily,
) -> String {
    // Limit flash size for bootloader (we don't need full flash)
    let flash_size = flash_size.min(bootloader_flash_limit as u64);

    // RP2350 needs special handling - BOOT region for start_block, then FLASH for code
    if family == ChipFamily::Rp2350 {
        // Reserve 256 bytes for start_block at beginning of flash
        let boot_size: u64 = 256;
        let flash_start = flash_base + boot_size;
        let flash_len = flash_size.saturating_sub(boot_size);

        format!(
            r#"/* Auto-generated by build.rs from probe-rs registry - DO NOT EDIT */
/* RP2350 Boot ROM requires Image Definition (start_block) at flash base */
MEMORY
{{
    BOOT  : ORIGIN = {:#010x}, LENGTH = {}
    FLASH : ORIGIN = {:#010x}, LENGTH = {}K
    RAM   : ORIGIN = {:#010x}, LENGTH = {}K
}}

/* Place start_block in BOOT region (before FLASH) */
SECTIONS {{
    .start_block : {{
        __start_block_addr = .;
        KEEP(*(.start_block));
        . = ALIGN(4);
    }} > BOOT
}} INSERT BEFORE .vector_table;
"#,
            flash_base,
            boot_size,
            flash_start,
            flash_len / 1024,
            ram_base,
            ram_size / 1024,
        )
    } else {
        // Standard layout for STM32 and other chips
        format!(
            r#"/* Auto-generated by build.rs from probe-rs registry - DO NOT EDIT */
MEMORY
{{
    FLASH : ORIGIN = {:#010x}, LENGTH = {}K
    RAM   : ORIGIN = {:#010x}, LENGTH = {}K
}}
"#,
            flash_base,
            flash_size / 1024,
            ram_base,
            ram_size / 1024,
        )
    }
}

fn main() -> Result<()> {
    let config = get_chip_config()?;

    // Get target from probe-rs registry
    let registry = Registry::from_builtin_families();
    let target = registry.get_target_by_name(config.target_name).map_err(|e| {
        anyhow::anyhow!(
            "Target '{}' not found in probe-rs registry: {}\n\
             Make sure probe-rs version supports this chip.",
            config.target_name,
            e
        )
    })?;

    // Extract memory regions
    let (flash_base, flash_size) =
        get_flash_region(&target.memory_map, config.target_name, config.expected_flash_base)?;

    let (ram_base, ram_size) =
        get_ram_region(&target.memory_map, config.target_name, &config.ram_selector)?;

    // Generate memory.x
    let memory_x = generate_memory_x(
        flash_base,
        flash_size,
        ram_base,
        ram_size,
        config.bootloader_flash_limit,
        config.family,
    );

    // Write to OUT_DIR
    let out_dir = PathBuf::from(env::var_os("OUT_DIR").unwrap());
    let memory_x_path = out_dir.join("memory.x");

    let mut file = fs::File::create(&memory_x_path)?;
    file.write_all(memory_x.as_bytes())?;

    // Tell cargo to use this directory for linker scripts
    println!("cargo:rustc-link-search={}", out_dir.display());

    // Report what we generated
    println!(
        "cargo:warning=Generated memory.x for {}: FLASH={:#x}+{}K, RAM={:#x}+{}K",
        config.target_name,
        flash_base,
        flash_size.min(config.bootloader_flash_limit as u64) / 1024,
        ram_base,
        ram_size / 1024,
    );

    // Rebuild if these change
    println!("cargo:rerun-if-env-changed=CARGO_FEATURE_CHIP_STM32H743");
    println!("cargo:rerun-if-env-changed=CARGO_FEATURE_CHIP_RP2350");

    Ok(())
}
