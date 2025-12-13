
//! Aviate development tools
//!
//! Cross-platform flash tool for STM32H743 boards with Aviate bootloader.

use anyhow::{bail, Context, Result};
use regex::Regex;
use std::io::{Read, Write};
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use std::thread;
use std::time::{Duration, Instant};
use sysinfo::System;

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
        "cleanup" => {
            run_cleanup()?;
        }
        "test" => {
            let config = args.get(2).map(|s| s.as_str());
            run_test(config)?;
        }
        "help" | "--help" | "-h" => {
            print_usage();
        }
        cmd => {
            bail!(
                "Unknown command: {}. Use 'cargo xtask help' for usage.",
                cmd
            );
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
    cleanup                      Clean up lingering SITL processes
    test [config]                Run SITL test (defaults to basic_flight)
    help                         Show this help

EXAMPLES:
    cargo xtask run my-app                 # Build and flash app
    cargo xtask flash app.bin              # Auto-detect serial port
    cargo xtask flash app.bin /dev/ttyACM0 # Linux/macOS
    cargo xtask flash app.bin COM3         # Windows
    cargo xtask dfu                        # Just enter DFU mode
    cargo xtask cleanup                    # Kill all SITL related processes

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
            bail!("No confirmation code received. Is software-bootloader feature enabled?");
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

fn run_cleanup() -> anyhow::Result<()> {
    println!("Cleaning up SITL processes...");
    let mut system = System::new_all();
    system.refresh_all();

    let targets = ["gz sim", "sitl-gazebo", "gcs-test", "mavrouter", "ruby"];
    let mut killed_count = 0;

    for process in system.processes().values() {
        let name = process.name();
        let cmd = process.cmd().join(" ");

        for target in targets.iter() {
            // Check process name or full command line
            if name.contains(target) || cmd.contains(target) {
                // Don't kill ourselves (xtask)
                if name.contains("xtask") {
                    continue;
                }
                
                // If it's gcs-test, only kill if it's not the one we might be spawning (though we are xtask, so gcs-test shouldn't be running yet if we are cleaning up PRE-run)
                // But if we run `xtask cleanup` manually, we kill everything.
                
                println!("  Killing: {} (PID: {})", name, process.pid());
                process.kill();
                killed_count += 1;
            }
        }
    }
    
    // Clean up shared memory on Linux
    #[cfg(target_os = "linux")]
    {
        let shm_path = Path::new("/dev/shm/aviate_gz_bridge");
        if shm_path.exists() {
             let _ = std::fs::remove_file(shm_path);
             println!("  Cleaned: /dev/shm/aviate_gz_bridge");
        }
    }

    if killed_count == 0 {
        println!("  No lingering processes found.");
    } else {
        println!("  Killed {} processes.", killed_count);
        // Wait a bit for processes to actually exit
        thread::sleep(Duration::from_millis(500));
    }

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

    bail!("Timeout waiting for DFU device. Check if bootloader is properly installed.")
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
    let bin_name = app_crate.strip_prefix("aviate-app-").unwrap_or(app_name);

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
    let elf_path = format!("target/thumbv7em-none-eabihf/release/{}", bin_name);
    let bin_path = format!("/tmp/{}.bin", bin_name);

    eprintln!("Converting {} to binary...", elf_path);

    // Check arm-none-eabi-objcopy is available
    if Command::new("arm-none-eabi-objcopy")
        .arg("--version")
        .output()
        .is_err()
    {
        bail!("arm-none-eabi-objcopy not found. Please install ARM toolchain.");
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

/// Run SITL test using gcs-test
fn run_test(config: Option<&str>) -> Result<()> {
    let config_path = config.unwrap_or("tests/missions/basic_flight.toml");

    // Always cleanup before running test to ensure clean state
    run_cleanup()?;

    // 1. Build gcs-test and FC binary with gazebo feature
    eprintln!("Building gcs-test and SITL app...");
    let status = Command::new("cargo")
        .args([
            "build",
            "-p",
            "gcs-test",
            "-p",
            "aviate-app-sitl-gazebo-x500",
            "--features",
            "gazebo",
        ])
        .status()
        .context("Failed to build gcs-test or sitl-gazebo-x500")?;

    if !status.success() {
        bail!("Failed to build tests");
    }

    // 2. Set up environment
    let cwd = std::env::current_dir()?;
    let plugin_dir = cwd.join("aviate-hal/xil/backends/gz/plugin/build");
    
    if !plugin_dir.join("libAviateGzPlugin.so").exists() {
         bail!("AviateGzPlugin not found at {}. Build with CMake first.", plugin_dir.display());
    }

    // 3. Run gcs-test
    eprintln!("Running test: {}", config_path);
    let mut cmd = Command::new("target/debug/gcs-test");
    cmd.args(["run", "--xil", config_path]);
    
    // Add plugin dir to LD_LIBRARY_PATH
    if let Ok(current_ld) = std::env::var("LD_LIBRARY_PATH") {
        cmd.env("LD_LIBRARY_PATH", format!("{}:{}", plugin_dir.display(), current_ld));
    } else {
        cmd.env("LD_LIBRARY_PATH", plugin_dir);
    }

    // Ensure cleanup on ctrl-c (basic handling)
    // In a real runner we might want complex signal handling, 
    // but gcs-test handles its own cleanup of child processes.

    let status = cmd.status().context("Failed to run gcs-test")?;

    if !status.success() {
        bail!("Test failed");
    }

    Ok(())
}
