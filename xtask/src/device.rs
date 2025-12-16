//! Device selection and detection
//!
//! Handles multi-device scenarios on dev benches.

// Allow dead code for future platform infrastructure
#![allow(dead_code)]

use anyhow::{bail, Result};

/// Device selector for targeting a specific device
#[derive(Debug, Clone, Default)]
pub struct DeviceSelector {
    /// Target chip name (from board metadata)
    pub chip: String,
    /// Serial number (for dfu-util -S, probe-rs --probe)
    pub serial: Option<String>,
    /// Port path (for espflash --port, stm32flash)
    pub port: Option<String>,
}

impl DeviceSelector {
    /// Create selector from chip name only (auto-detect device)
    pub fn from_chip(chip: impl Into<String>) -> Self {
        Self {
            chip: chip.into(),
            serial: None,
            port: None,
        }
    }

    /// Create selector from chip and --device argument
    ///
    /// Supported formats:
    /// - `serial=XXXXXXXX` - select by USB serial number
    /// - `path=/dev/ttyUSB0` - select by port path
    /// - `/dev/ttyUSB0` or `COM3` - inferred as port
    /// - `XXXXXXXX` - inferred as serial
    pub fn from_args(chip: impl Into<String>, device_arg: Option<&str>) -> Result<Self> {
        let chip = chip.into();

        let (serial, port) = match device_arg {
            Some(d) if d.starts_with("serial=") => (Some(d[7..].to_string()), None),
            Some(d) if d.starts_with("path=") => (None, Some(d[5..].to_string())),
            Some(d) => {
                // Guess based on format
                if d.contains('/') || d.starts_with("COM") || d.starts_with("com") {
                    // Looks like a port path
                    (None, Some(d.to_string()))
                } else {
                    // Assume serial number
                    (Some(d.to_string()), None)
                }
            }
            None => (None, None),
        };

        Ok(Self { chip, serial, port })
    }

    /// Check if this selector specifies a specific device
    pub fn is_specific(&self) -> bool {
        self.serial.is_some() || self.port.is_some()
    }
}

/// Device state
#[derive(Debug, Clone)]
pub enum DeviceState {
    /// Device is in ROM bootloader mode (STM32 ROM DFU, RP2040 BOOTSEL, etc.)
    /// This allows flashing bootloader + app
    RomBootloader,
    /// Device is in custom bootloader DFU mode (Aviate bootloader)
    /// This only allows flashing app (bootloader protects itself)
    CustomBootloaderDfu,
    /// Device is running application (serial port available)
    Running(String), // Port name
    /// Device is not connected or not detected
    NotFound,
}

/// Detect the current state of a device based on board metadata
///
/// Priority:
/// 1. ROM DFU (for first-time flash of bootloader + app)
/// 2. Custom bootloader DFU (for app-only flash)
/// 3. Running app via serial (for software DFU entry)
/// 4. Not found
pub fn detect_device_state(
    vid: Option<u16>,
    bootloader_pid: Option<u16>,
) -> Result<DeviceState> {
    if let (Some(vid), Some(pid)) = (vid, bootloader_pid) {
        // Check ROM DFU first (allows full flash including bootloader)
        if crate::programmer::is_rom_dfu_present(vid, pid)? {
            return Ok(DeviceState::RomBootloader);
        }

        // Check custom bootloader DFU (app-only flash)
        if crate::programmer::is_custom_dfu_present(vid, pid)? {
            return Ok(DeviceState::CustomBootloaderDfu);
        }
    }

    // Check for running app via serial port
    if let Some(vid) = vid {
        if let Some(port) = find_serial_port_by_vid(vid)? {
            return Ok(DeviceState::Running(port));
        }
    }

    Ok(DeviceState::NotFound)
}

/// Find serial port by VID (vendor ID)
pub fn find_serial_port_by_vid(vid: u16) -> Result<Option<String>> {
    let ports = serialport::available_ports()
        .map_err(|e| anyhow::anyhow!("Failed to list ports: {}", e))?;

    for port in &ports {
        if let serialport::SerialPortType::UsbPort(info) = &port.port_type {
            if info.vid == vid {
                return Ok(Some(port.port_name.clone()));
            }
        }
    }

    Ok(None)
}

/// Find serial port by VID and optional port filter
pub fn find_serial_port_with_vid(vid: u16, port_filter: Option<&str>) -> Result<Option<String>> {
    let ports = serialport::available_ports()
        .map_err(|e| anyhow::anyhow!("Failed to list ports: {}", e))?;

    for port in &ports {
        // If filter specified, check it first
        if let Some(filter) = port_filter {
            if port.port_name != filter {
                continue;
            }
        }

        if let serialport::SerialPortType::UsbPort(info) = &port.port_type {
            if info.vid == vid {
                return Ok(Some(port.port_name.clone()));
            }
        }
    }

    Ok(None)
}

/// Find any available serial port (for auto-detect)
pub fn find_any_serial_port() -> Result<String> {
    let ports = serialport::available_ports()
        .map_err(|e| anyhow::anyhow!("Failed to list ports: {}", e))?;

    for port in &ports {
        // Prefer USB CDC ACM devices
        if let serialport::SerialPortType::UsbPort(info) = &port.port_type {
            // ST Microelectronics VID
            if info.vid == 0x0483 {
                return Ok(port.port_name.clone());
            }
        }

        // On Linux, ttyACM* are typically CDC ACM devices
        #[cfg(target_os = "linux")]
        if port.port_name.contains("ttyACM") {
            return Ok(port.port_name.clone());
        }

        // On macOS, usbmodem* are typically CDC ACM devices
        #[cfg(target_os = "macos")]
        if port.port_name.contains("usbmodem") {
            return Ok(port.port_name.clone());
        }
    }

    // List what we found for debugging
    eprintln!("Available ports:");
    for port in &ports {
        eprintln!("  {} ({:?})", port.port_name, port.port_type);
    }

    bail!("No USB CDC ACM device found. Is the device connected and running?")
}

/// List all serial ports with USB info
pub fn list_serial_ports() -> Result<Vec<SerialPortInfo>> {
    let ports = serialport::available_ports()
        .map_err(|e| anyhow::anyhow!("Failed to list ports: {}", e))?;

    let mut result = Vec::new();
    for port in ports {
        let usb_info = if let serialport::SerialPortType::UsbPort(info) = &port.port_type {
            Some((info.vid, info.pid, info.serial_number.clone()))
        } else {
            None
        };

        result.push(SerialPortInfo {
            name: port.port_name,
            usb_info,
        });
    }

    Ok(result)
}

/// Serial port information
#[derive(Debug, Clone)]
pub struct SerialPortInfo {
    /// Port name (e.g., "/dev/ttyACM0", "COM3")
    pub name: String,
    /// USB info if available: (VID, PID, Serial)
    pub usb_info: Option<(u16, u16, Option<String>)>,
}

impl SerialPortInfo {
    /// Format port info for display
    pub fn display(&self) -> String {
        if let Some((vid, pid, serial)) = &self.usb_info {
            let serial_str = serial.as_deref().unwrap_or("(no serial)");
            format!("{} [{:04x}:{:04x}] {}", self.name, vid, pid, serial_str)
        } else {
            self.name.clone()
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_device_selector_from_args() {
        // Serial format
        let sel = DeviceSelector::from_args("STM32H743VITx", Some("serial=12345678")).unwrap();
        assert_eq!(sel.serial.as_deref(), Some("12345678"));
        assert!(sel.port.is_none());

        // Path format
        let sel = DeviceSelector::from_args("STM32H743VITx", Some("path=/dev/ttyUSB0")).unwrap();
        assert!(sel.serial.is_none());
        assert_eq!(sel.port.as_deref(), Some("/dev/ttyUSB0"));

        // Inferred port (Linux)
        let sel = DeviceSelector::from_args("STM32H743VITx", Some("/dev/ttyACM0")).unwrap();
        assert!(sel.serial.is_none());
        assert_eq!(sel.port.as_deref(), Some("/dev/ttyACM0"));

        // Inferred port (Windows)
        let sel = DeviceSelector::from_args("STM32H743VITx", Some("COM3")).unwrap();
        assert!(sel.serial.is_none());
        assert_eq!(sel.port.as_deref(), Some("COM3"));

        // Inferred serial
        let sel = DeviceSelector::from_args("STM32H743VITx", Some("ABCD1234")).unwrap();
        assert_eq!(sel.serial.as_deref(), Some("ABCD1234"));
        assert!(sel.port.is_none());

        // No device arg
        let sel = DeviceSelector::from_args("STM32H743VITx", None).unwrap();
        assert!(sel.serial.is_none());
        assert!(sel.port.is_none());
    }

    #[test]
    fn test_device_selector_is_specific() {
        let generic = DeviceSelector::from_chip("STM32H743VITx");
        assert!(!generic.is_specific());

        let specific = DeviceSelector::from_args("STM32H743VITx", Some("serial=123")).unwrap();
        assert!(specific.is_specific());
    }
}
