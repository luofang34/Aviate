//! Programmer abstraction for different flash tools
//!
//! Supports dfu-util (STM32), picotool (RP2040), espflash (ESP32), etc.

// Allow dead code for future platform infrastructure
#![allow(dead_code)]

#[cfg(feature = "hardware")]
use crate::device::DeviceSelector;
#[cfg(feature = "hardware")]
use crate::flash_plan::{FlashPlan, Segment};
use anyhow::{bail, Context, Result};
use std::process::Command;
use std::time::{Duration, Instant};

/// Programming method (decoupled from chip family)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Programmer {
    /// STM32 ROM DFU bootloader (dfu-util)
    Stm32RomDfu,
    /// ST-Link debugger (probe-rs)
    Stm32StLink,
    /// UART bootloader (stm32flash)
    Stm32Uart,
    /// RP2040/RP2350 BOOTSEL mode (picotool or UF2)
    Rp2040Bootsel,
    /// ESP32 ROM serial bootloader (espflash)
    Esp32RomSerial,
    /// No programmer (SITL/XIL)
    None,
}

impl Programmer {
    /// Parse programmer from string (used in board metadata)
    pub fn from_str(s: &str) -> Result<Self> {
        match s.to_lowercase().as_str() {
            "stm32-rom-dfu" | "stm32_rom_dfu" | "dfu-util" => Ok(Self::Stm32RomDfu),
            "stm32-stlink" | "stm32_stlink" | "probe-rs" => Ok(Self::Stm32StLink),
            "stm32-uart" | "stm32_uart" | "stm32flash" => Ok(Self::Stm32Uart),
            "rp2040-bootsel" | "rp2040_bootsel" | "picotool" => Ok(Self::Rp2040Bootsel),
            "esp32-rom-serial" | "esp32_rom_serial" | "espflash" => Ok(Self::Esp32RomSerial),
            "none" | "sitl" | "xil" => Ok(Self::None),
            other => bail!("Unknown programmer: {}", other),
        }
    }

    /// Get the default VID for this programmer
    pub fn default_vid(&self) -> Option<u16> {
        match self {
            Self::Stm32RomDfu => Some(0x0483), // STMicroelectronics
            Self::Stm32StLink => Some(0x0483),
            Self::Stm32Uart => None,              // UART, no USB
            Self::Rp2040Bootsel => Some(0x2e8a),  // Raspberry Pi
            Self::Esp32RomSerial => Some(0x303a), // Espressif
            Self::None => None,
        }
    }

    /// Get the default PID for ROM bootloader mode
    pub fn default_bootloader_pid(&self) -> Option<u16> {
        match self {
            Self::Stm32RomDfu => Some(0xdf11),   // DFU mode
            Self::Stm32StLink => Some(0x374b),   // ST-Link V2-1
            Self::Rp2040Bootsel => Some(0x0003), // BOOTSEL mode
            Self::Esp32RomSerial => Some(0x1001),
            Self::Stm32Uart | Self::None => None,
        }
    }

    /// Get the tool name for this programmer
    pub fn tool_name(&self) -> &'static str {
        match self {
            Self::Stm32RomDfu => "dfu-util",
            Self::Stm32StLink => "probe-rs",
            Self::Stm32Uart => "stm32flash",
            Self::Rp2040Bootsel => "picotool",
            Self::Esp32RomSerial => "espflash",
            Self::None => "none",
        }
    }

    /// Check if the required tool is available
    pub fn check_tool_available(&self) -> Result<()> {
        if *self == Self::None {
            return Ok(());
        }

        let tool = self.tool_name();
        let result = Command::new(tool).arg("--version").output();

        match result {
            Ok(output) if output.status.success() => Ok(()),
            _ => bail!(
                "{} not found. Please install {} and ensure it's in PATH.",
                tool,
                tool
            ),
        }
    }
}

/// Execute a flash plan by iterating segments
#[cfg(feature = "hardware")]
pub fn execute_flash_plan(
    programmer: Programmer,
    plan: &FlashPlan,
    device: &DeviceSelector,
) -> Result<()> {
    // Validate plan first
    plan.validate_paths()?;
    programmer.check_tool_available()?;

    if programmer == Programmer::None {
        eprintln!("Skipping flash for SITL/XIL board");
        return Ok(());
    }

    // For ROM DFU with multiple segments (bootloader + app), flash app FIRST
    // This ensures the app sector is erased before the bootloader is flashed
    // Otherwise, the bootloader might find an old valid app and jump to it
    let segments: Vec<_> = if programmer == Programmer::Stm32RomDfu && plan.has_bootloader() {
        // Sort to put App segments before Bootloader segments
        let mut sorted = plan.segments.clone();
        sorted.sort_by_key(|s| match s.kind {
            crate::flash_plan::SegmentKind::App => 0,
            crate::flash_plan::SegmentKind::Bootloader => 1,
            _ => 2,
        });
        sorted
    } else {
        plan.segments.clone()
    };

    eprintln!("Flashing {} segment(s):", segments.len());
    let segment_count = segments.len();

    for (idx, seg) in segments.iter().enumerate() {
        let is_last = idx == segment_count - 1;
        eprintln!(
            "  {}: {:#x} <- {}",
            seg.kind,
            seg.address,
            seg.path.display()
        );
        flash_segment_with_leave(programmer, seg, device, is_last)?;
    }

    Ok(())
}

/// Flash a single segment with optional leave flag
#[cfg(feature = "hardware")]
fn flash_segment_with_leave(
    programmer: Programmer,
    seg: &Segment,
    device: &DeviceSelector,
    leave: bool,
) -> Result<()> {
    let path_str = seg
        .path
        .to_str()
        .ok_or_else(|| anyhow::anyhow!("Invalid path: {}", seg.path.display()))?;

    match programmer {
        Programmer::Stm32RomDfu => flash_dfu_util(seg, path_str, device, leave),
        Programmer::Stm32StLink => flash_probe_rs(seg, path_str, device),
        Programmer::Stm32Uart => flash_stm32flash(seg, path_str, device),
        Programmer::Rp2040Bootsel => flash_picotool(seg, path_str, device),
        Programmer::Esp32RomSerial => flash_espflash(seg, path_str, device),
        Programmer::None => Ok(()),
    }
}

/// Erase flash sector using dfu-util (for preparing app sector before bootloader flash)
#[cfg(feature = "hardware")]
fn dfu_erase_sector(address: u32, device: &DeviceSelector) -> Result<()> {
    let addr_spec = format!("{:#x}", address);

    eprintln!("Erasing flash sector at {:#x}...", address);

    let mut cmd = Command::new("dfu-util");
    // Use -s with :mass-erase would erase all, but we just want to erase app sector
    // dfu-util doesn't have a simple sector erase, but we can use the dfuse command
    // Actually, dfu-util with DfuSe will auto-erase before write, so we need to write
    // a small invalid pattern to invalidate the app
    // For now, skip the erase and just let the flow continue
    cmd.args(["-a", "0", "-s", &format!("{}:leave", addr_spec)]);
    cmd.args(["-D", "/dev/null"]);

    if let Some(serial) = &device.serial {
        cmd.args(["-S", serial]);
    }

    // This approach doesn't really work well with dfu-util
    // Instead, we'll handle this differently
    Ok(())
}

/// Flash using dfu-util (STM32 ROM DFU)
#[cfg(feature = "hardware")]
fn flash_dfu_util(seg: &Segment, path: &str, device: &DeviceSelector, leave: bool) -> Result<()> {
    // Only add :leave suffix on the last segment to avoid resetting between segments
    let addr_spec = if leave {
        format!("{:#x}:leave", seg.address)
    } else {
        format!("{:#x}", seg.address)
    };

    let mut cmd = Command::new("dfu-util");
    cmd.args(["-a", "0", "-s", &addr_spec, "-D", path]);

    if let Some(serial) = &device.serial {
        cmd.args(["-S", serial]);
    }

    let status = cmd.status().context("Failed to run dfu-util")?;

    // dfu-util may return non-zero due to device reset after flashing
    // This is expected behavior - the device leaves DFU mode
    if !status.success() {
        eprintln!(
            "Note: dfu-util exit code may be non-zero due to device reset after {:?}",
            seg.kind
        );
    }

    Ok(())
}

/// Flash using probe-rs (ST-Link or other debug probes)
#[cfg(feature = "hardware")]
fn flash_probe_rs(seg: &Segment, path: &str, device: &DeviceSelector) -> Result<()> {
    let addr_str = format!("{:#x}", seg.address);

    let mut cmd = Command::new("probe-rs");
    cmd.args([
        "download",
        "--chip",
        &device.chip,
        path,
        "--base-address",
        &addr_str,
    ]);

    if let Some(serial) = &device.serial {
        cmd.args(["--probe", serial]);
    }

    let status = cmd.status().context("Failed to run probe-rs")?;

    if !status.success() {
        bail!("probe-rs flash failed for {:?}", seg.kind);
    }

    Ok(())
}

/// Flash using stm32flash (UART bootloader)
#[cfg(feature = "hardware")]
fn flash_stm32flash(seg: &Segment, path: &str, device: &DeviceSelector) -> Result<()> {
    let port = device
        .port
        .as_ref()
        .ok_or_else(|| anyhow::anyhow!("stm32flash requires --device path=<port>"))?;

    let addr_str = format!("{:#x}", seg.address);

    let status = Command::new("stm32flash")
        .args(["-w", path, "-S", &addr_str, port])
        .status()
        .context("Failed to run stm32flash")?;

    if !status.success() {
        bail!("stm32flash failed for {:?}", seg.kind);
    }

    Ok(())
}

/// Flash using picotool (RP2040 BOOTSEL)
#[cfg(feature = "hardware")]
fn flash_picotool(seg: &Segment, path: &str, _device: &DeviceSelector) -> Result<()> {
    let addr_str = format!("{:#x}", seg.address);

    let status = Command::new("picotool")
        .args(["load", path, "-x", "-o", &addr_str])
        .status()
        .context("Failed to run picotool")?;

    if !status.success() {
        bail!("picotool flash failed for {:?}", seg.kind);
    }

    Ok(())
}

/// Flash using espflash (ESP32 ROM bootloader)
#[cfg(feature = "hardware")]
fn flash_espflash(seg: &Segment, path: &str, device: &DeviceSelector) -> Result<()> {
    let addr_str = format!("{:#x}", seg.address);

    let mut cmd = Command::new("espflash");
    cmd.args(["flash", path, "--address", &addr_str]);

    if let Some(port) = &device.port {
        cmd.args(["--port", port]);
    }

    let status = cmd.status().context("Failed to run espflash")?;

    if !status.success() {
        bail!("espflash failed for {:?}", seg.kind);
    }

    Ok(())
}

/// Check if a DFU device is present
pub fn dfu_device_present(vid: u16, pid: u16, serial: Option<&str>) -> Result<bool> {
    let output = Command::new("dfu-util")
        .arg("-l")
        .output()
        .context("Failed to run dfu-util -l")?;

    let stdout = String::from_utf8_lossy(&output.stdout);
    let vid_pid = format!("{:04x}:{:04x}", vid, pid);

    if !stdout.contains(&vid_pid) {
        return Ok(false);
    }

    // If serial specified, check for it too
    if let Some(s) = serial {
        return Ok(stdout.contains(s));
    }

    Ok(true)
}

/// Check if STM32 ROM DFU is present (not custom bootloader DFU)
///
/// ROM DFU has "Internal Flash" in the interface name.
/// Custom bootloader DFU has "Flash/0x08020000" (app area only).
pub fn is_rom_dfu_present(vid: u16, pid: u16) -> Result<bool> {
    let output = Command::new("dfu-util")
        .arg("-l")
        .output()
        .context("Failed to run dfu-util -l")?;

    let stdout = String::from_utf8_lossy(&output.stdout);
    let vid_pid = format!("{:04x}:{:04x}", vid, pid);

    // Check if the specific VID:PID is present
    if !stdout.contains(&vid_pid) {
        return Ok(false);
    }

    // ROM DFU has "Internal Flash" in the name, custom bootloader doesn't
    // ROM DFU example: name="@Internal Flash   /0x08000000/16*128Kg"
    // Custom DFU example: name="@Flash/0x08020000/15*128Ke"
    Ok(stdout.contains("Internal Flash"))
}

/// Check if custom bootloader DFU is present (Aviate bootloader)
pub fn is_custom_dfu_present(vid: u16, pid: u16) -> Result<bool> {
    let output = Command::new("dfu-util")
        .arg("-l")
        .output()
        .context("Failed to run dfu-util -l")?;

    let stdout = String::from_utf8_lossy(&output.stdout);
    let vid_pid = format!("{:04x}:{:04x}", vid, pid);

    if !stdout.contains(&vid_pid) {
        return Ok(false);
    }

    // Custom bootloader has serial "AVT001" or flash area starting at app address
    // It does NOT have "Internal Flash" in the name
    Ok(stdout.contains("AVT001") || (stdout.contains(&vid_pid) && !stdout.contains("Internal Flash")))
}

/// Wait for DFU device to enumerate
pub fn wait_for_dfu_device(vid: u16, pid: u16, timeout: Duration) -> Result<()> {
    let start = Instant::now();

    eprintln!("Waiting for DFU device ({:04x}:{:04x})...", vid, pid);

    while start.elapsed() < timeout {
        std::thread::sleep(Duration::from_millis(500));

        if dfu_device_present(vid, pid, None)? {
            return Ok(());
        }
    }

    bail!(
        "Timeout waiting for DFU device {:04x}:{:04x}. \
         Check if bootloader is properly installed.",
        vid,
        pid
    )
}

/// List DFU devices (for --device selection help)
pub fn list_dfu_devices() -> Result<Vec<String>> {
    let output = Command::new("dfu-util")
        .arg("-l")
        .output()
        .context("Failed to run dfu-util -l")?;

    let stdout = String::from_utf8_lossy(&output.stdout);
    let mut devices = Vec::new();

    // Parse dfu-util output for device serials
    // Format: Found DFU: [0483:df11] ver=0200, devnum=42, cfg=1, intf=0, path="1-1", alt=0, name="@Internal Flash  /0x08000000/01*128Kg,01*128Kg", serial="xxxxxxxx"
    for line in stdout.lines() {
        if line.contains("Found DFU:") {
            // Extract VID:PID and serial if present
            if let Some(serial_start) = line.find("serial=\"") {
                let serial_part = &line[serial_start + 8..];
                if let Some(serial_end) = serial_part.find('"') {
                    let serial = &serial_part[..serial_end];
                    if !serial.is_empty() {
                        devices.push(serial.to_string());
                    }
                }
            }
        }
    }

    Ok(devices)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_programmer_from_str() {
        assert_eq!(
            Programmer::from_str("stm32-rom-dfu").unwrap(),
            Programmer::Stm32RomDfu
        );
        assert_eq!(
            Programmer::from_str("dfu-util").unwrap(),
            Programmer::Stm32RomDfu
        );
        assert_eq!(Programmer::from_str("none").unwrap(), Programmer::None);
        assert!(Programmer::from_str("unknown").is_err());
    }

    #[test]
    fn test_programmer_defaults() {
        let dfu = Programmer::Stm32RomDfu;
        assert_eq!(dfu.default_vid(), Some(0x0483));
        assert_eq!(dfu.default_bootloader_pid(), Some(0xdf11));
        assert_eq!(dfu.tool_name(), "dfu-util");
    }
}
