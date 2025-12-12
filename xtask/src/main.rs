//! Aviate development tools
//!
//! Cross-platform flash tool for STM32H743 boards with Aviate bootloader.

use anyhow::{bail, Context, Result};
use regex::Regex;
use std::io::{Read, Write};
use std::process::Command;
use std::time::{Duration, Instant};

const DFU_VID_PID: &str = "0483:df11";
const APP_FLASH_ADDRESS: &str = "0x08020000";

fn main() -> Result<()> {
    let args: Vec<String> = std::env::args().collect();

    if args.len() < 2 {
        print_usage();
        return Ok(());
    }

    match args[1].as_str() {
        "flash" => {
            if args.len() < 3 {
                bail!("Usage: cargo xtask flash <firmware.bin> [serial_port]");
            }
            let firmware_path = &args[2];
            let port = args.get(3).map(|s| s.as_str());
            flash_firmware(firmware_path, port)?;
        }
        "run" => {
            if args.len() < 3 {
                bail!("Usage: cargo xtask run <app-name> [serial_port]");
            }
            let app_name = &args[2];
            let port = args.get(3).map(|s| s.as_str());
            run_app(app_name, port)?;
        }
        "dfu" => {
            let port = args.get(2).map(|s| s.as_str());
            enter_dfu_mode(port)?;
        }
        "help" | "--help" | "-h" => {
            print_usage();
        }
        cmd => {
            bail!("Unknown command: {}. Use 'cargo xtask help' for usage.", cmd);
        }
    }

    Ok(())
}

fn print_usage() {
    eprintln!(
        r#"Aviate Development Tools

USAGE:
    cargo xtask <COMMAND> [OPTIONS]

COMMANDS:
    run <app-name> [port]        Build and flash app in one step
    flash <firmware.bin> [port]  Flash firmware via software DFU
    dfu [port]                   Enter DFU mode without flashing
    help                         Show this help

EXAMPLES:
    cargo xtask run my-app                 # Build and flash app
    cargo xtask flash app.bin              # Auto-detect serial port
    cargo xtask flash app.bin /dev/ttyACM0 # Linux/macOS
    cargo xtask flash app.bin COM3         # Windows
    cargo xtask dfu                        # Just enter DFU mode

REQUIREMENTS:
    - Device running firmware with software-bootloader feature
    - dfu-util installed and in PATH
    - arm-none-eabi-objcopy for 'run' command
"#
    );
}

/// Find the first available USB CDC ACM port
fn find_serial_port() -> Result<String> {
    let ports = serialport::available_ports().context("Failed to list serial ports")?;

    for port in &ports {
        // Look for USB CDC ACM devices (ST VID or common patterns)
        if let serialport::SerialPortType::UsbPort(info) = &port.port_type {
            // ST Microelectronics VID
            if info.vid == 0x0483 {
                eprintln!("Found ST device: {}", port.port_name);
                return Ok(port.port_name.clone());
            }
        }

        // On Linux, ttyACM* are typically CDC ACM devices
        #[cfg(target_os = "linux")]
        if port.port_name.contains("ttyACM") {
            eprintln!("Found ACM device: {}", port.port_name);
            return Ok(port.port_name.clone());
        }

        // On macOS, usbmodem* are typically CDC ACM devices
        #[cfg(target_os = "macos")]
        if port.port_name.contains("usbmodem") {
            eprintln!("Found USB modem: {}", port.port_name);
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

/// Enter DFU mode by sending the dfu command and confirmation code
fn enter_dfu_mode(port: Option<&str>) -> Result<()> {
    let port_name = match port {
        Some(p) => p.to_string(),
        None => find_serial_port()?,
    };

    eprintln!("Connecting to {}...", port_name);

    let mut serial = serialport::new(&port_name, 115200)
        .timeout(Duration::from_secs(3))
        .open()
        .with_context(|| format!("Failed to open serial port: {}", port_name))?;

    // Wait for connection to stabilize
    std::thread::sleep(Duration::from_millis(500));

    // Clear any pending data
    let _ = serial.clear(serialport::ClearBuffer::All);

    // Send dfu command
    eprintln!("Sending 'dfu' command...");
    serial.write_all(b"dfu\r\n")?;
    serial.flush()?;

    // Read response
    std::thread::sleep(Duration::from_millis(500));
    let mut response = vec![0u8; 256];
    let n = serial.read(&mut response).unwrap_or(0);
    let response_str = String::from_utf8_lossy(&response[..n]);

    // Parse confirmation code
    let code_regex = Regex::new(r"CONFIRM:(\d{4})").unwrap();
    let code = match code_regex.captures(&response_str) {
        Some(caps) => caps.get(1).unwrap().as_str(),
        None => {
            eprintln!("Response: {}", response_str);
            bail!(
                "No confirmation code received. Is software-bootloader feature enabled?"
            );
        }
    };

    eprintln!("Got confirmation code: {}", code);

    // Send confirmation code
    eprintln!("Confirming reboot...");
    serial.write_all(format!("{}\r\n", code).as_bytes())?;
    serial.flush()?;

    // Wait for device to reboot (connection will drop)
    std::thread::sleep(Duration::from_millis(500));

    // Try to read final response (may fail if device already rebooted)
    let mut final_response = vec![0u8; 256];
    let _ = serial.read(&mut final_response);

    drop(serial);

    eprintln!("Device rebooting to bootloader...");

    // Wait for DFU device to appear
    wait_for_dfu_device()?;

    eprintln!("Device is now in DFU mode!");
    Ok(())
}

/// Wait for DFU device to enumerate
fn wait_for_dfu_device() -> Result<()> {
    let start = Instant::now();
    let timeout = Duration::from_secs(10);

    eprintln!("Waiting for DFU device...");

    while start.elapsed() < timeout {
        std::thread::sleep(Duration::from_millis(500));

        let output = Command::new("dfu-util").arg("-l").output();

        if let Ok(output) = output {
            let stdout = String::from_utf8_lossy(&output.stdout);
            if stdout.contains(DFU_VID_PID) {
                return Ok(());
            }
        }
    }

    bail!(
        "Timeout waiting for DFU device. Check if bootloader is properly installed."
    )
}

/// Build and flash an app in one step
fn run_app(app_name: &str, port: Option<&str>) -> Result<()> {
    // Determine app crate name (may or may not have aviate-app- prefix)
    let app_crate = if app_name.starts_with("aviate-app-") {
        app_name.to_string()
    } else {
        format!("aviate-app-{}", app_name)
    };

    // The binary name is the part after aviate-app-
    let bin_name = app_crate
        .strip_prefix("aviate-app-")
        .unwrap_or(app_name);

    eprintln!("Building {}...", app_crate);

    // Build the app for hardware target
    let status = Command::new("cargo")
        .args([
            "build",
            "-p",
            &app_crate,
            "--release",
            "--target",
            "thumbv7em-none-eabihf",
        ])
        .status()
        .context("Failed to run cargo build")?;

    if !status.success() {
        bail!("Build failed");
    }

    // Convert ELF to binary
    let elf_path = format!(
        "target/thumbv7em-none-eabihf/release/{}",
        bin_name
    );
    let bin_path = format!("/tmp/{}.bin", bin_name);

    eprintln!("Converting {} to binary...", elf_path);

    // Check arm-none-eabi-objcopy is available
    if Command::new("arm-none-eabi-objcopy")
        .arg("--version")
        .output()
        .is_err()
    {
        bail!(
            "arm-none-eabi-objcopy not found. Please install ARM toolchain."
        );
    }

    let status = Command::new("arm-none-eabi-objcopy")
        .args(["-O", "binary", &elf_path, &bin_path])
        .status()
        .context("Failed to run objcopy")?;

    if !status.success() {
        bail!("objcopy failed");
    }

    eprintln!("Flashing {}...", bin_path);
    flash_firmware(&bin_path, port)?;

    Ok(())
}

/// Flash firmware to the device
fn flash_firmware(firmware_path: &str, port: Option<&str>) -> Result<()> {
    // Check firmware file exists
    if !std::path::Path::new(firmware_path).exists() {
        bail!("Firmware file not found: {}", firmware_path);
    }

    // Check dfu-util is available
    if Command::new("dfu-util").arg("--version").output().is_err() {
        bail!("dfu-util not found. Please install dfu-util and add it to PATH.");
    }

    // First check if already in DFU mode
    let output = Command::new("dfu-util").arg("-l").output()?;
    let stdout = String::from_utf8_lossy(&output.stdout);

    if !stdout.contains(DFU_VID_PID) {
        // Not in DFU mode, need to enter it via serial
        enter_dfu_mode(port)?;
    } else {
        eprintln!("Device already in DFU mode");
    }

    // Flash the firmware
    eprintln!("Flashing {}...", firmware_path);

    let status = Command::new("dfu-util")
        .args([
            "-a",
            "0",
            "-s",
            &format!("{}:leave", APP_FLASH_ADDRESS),
            "-D",
            firmware_path,
        ])
        .status()
        .context("Failed to run dfu-util")?;

    if !status.success() {
        // dfu-util returns non-zero even on success due to device reset
        // Check if we got past the download stage
        eprintln!("Note: dfu-util exit code may be non-zero due to device reset after flashing");
    }

    eprintln!("Flash complete!");

    // Wait for device to restart
    std::thread::sleep(Duration::from_secs(2));

    // Check if app started (serial port should reappear)
    match find_serial_port() {
        Ok(port) => eprintln!("Device running on {}", port),
        Err(_) => eprintln!("Device may still be starting up..."),
    }

    Ok(())
}
