//! Aviate development tools
//!
//! Cross-platform flash tool for embedded boards with Aviate.

mod boards;
mod device;
mod flash_plan;
mod geometry;
mod layout;
mod programmer;

use anyhow::{bail, Context, Result};
use boards::BoardMetadata;
use device::{DeviceSelector, DeviceState};
use flash_plan::FlashPlan;
use layout::LayoutPolicy;
use programmer::Programmer;
use regex::Regex;
use std::io::{Read, Write};
use std::path::{Path, PathBuf};
use std::process::Command;
use std::thread;
use std::time::Duration;
use sysinfo::System;

fn main() -> Result<()> {
    let args: Vec<String> = std::env::args().collect();

    if args.len() < 2 {
        print_usage();
        return Ok(());
    }

    match args[1].as_str() {
        "flash" => {
            if args.len() < 3 {
                bail!("Usage: cargo xtask flash <firmware.bin> [--board <board>] [--device <selector>]");
            }
            let firmware_path = &args[2];
            let mut board_name = None;
            let mut device_arg = None;
            let mut idx = 3;
            while idx < args.len() {
                match args[idx].as_str() {
                    "--board" => {
                        idx += 1;
                        if idx < args.len() {
                            board_name = Some(args[idx].clone());
                        }
                    }
                    "--device" => {
                        idx += 1;
                        if idx < args.len() {
                            device_arg = Some(args[idx].clone());
                        }
                    }
                    _ => {
                        // Legacy: positional port argument
                        if !args[idx].starts_with("--") && device_arg.is_none() {
                            device_arg = Some(args[idx].clone());
                        }
                    }
                }
                idx += 1;
            }
            flash_firmware_cmd(firmware_path, board_name.as_deref(), device_arg.as_deref())?;
        }
        "run" => {
            // Check for help
            if args.contains(&"--help".to_string()) || args.contains(&"-h".to_string()) {
                print_usage();
                return Ok(());
            }

            // New run command parsing
            let mut airframe = "x500".to_string();
            let mut board = "sitl-gazebo".to_string();
            let mut mission_config = None;
            let mut gcs = false;
            let mut headless = false;
            let mut device_arg = None;
            let mut idx = 2;

            while idx < args.len() {
                match args[idx].as_str() {
                    "--airframe" => {
                        idx += 1;
                        if idx < args.len() {
                            airframe = args[idx].clone();
                        }
                    }
                    "--board" => {
                        idx += 1;
                        if idx < args.len() {
                            board = args[idx].clone();
                        }
                    }
                    "--mission" => {
                        idx += 1;
                        if idx < args.len() {
                            mission_config = Some(args[idx].clone());
                        }
                    }
                    "--device" => {
                        idx += 1;
                        if idx < args.len() {
                            device_arg = Some(args[idx].clone());
                        }
                    }
                    "--gcs" => {
                        gcs = true;
                    }
                    "--headless" => {
                        headless = true;
                    }
                    _ => {
                        // Assume it's the airframe if it's the first positional arg
                        if !args[idx].starts_with("--") && idx == 2 {
                            airframe = args[idx].clone();
                        }
                    }
                }
                idx += 1;
            }

            if board.starts_with("sitl") {
                run_sitl(&airframe, &board, mission_config.as_deref(), gcs, headless)?;
            } else {
                run_hardware(&airframe, &board, device_arg.as_deref())?;
            }
        }
        "layout" => {
            // Layout management subcommand
            if args.len() < 3 {
                bail!("Usage: cargo xtask layout <show|reset> --board <board>");
            }
            let subcmd = &args[2];
            let mut board_name = None;
            let mut idx = 3;
            while idx < args.len() {
                if args[idx] == "--board" {
                    idx += 1;
                    if idx < args.len() {
                        board_name = Some(args[idx].clone());
                    }
                }
                idx += 1;
            }
            let board_name = board_name.ok_or_else(|| anyhow::anyhow!("--board is required"))?;
            run_layout_cmd(subcmd, &board_name)?;
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
        "test-mavlink" => {
            run_python_test()?;
        }
        "create" => {
            if args.len() < 3 {
                bail!("Usage: cargo xtask create <app-name> [--board <board>]");
            }
            let app_name = &args[2];
            let mut board = "sitl-gazebo";
            let mut idx = 3;
            while idx < args.len() {
                if args[idx] == "--board" {
                    idx += 1;
                    if idx < args.len() {
                        board = &args[idx];
                    }
                }
                idx += 1;
            }
            run_create_app(app_name, board)?;
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
    run [options]                      Run app (SITL) or build+flash (Hardware)
                                       Options:
                                         --airframe <name> (default: x500)
                                         --board <name>    (default: sitl-gazebo)
                                         --mission <path>  Run specific mission (SITL only)
                                         --device <sel>    Device selector (hardware only)
                                         --gcs             Run with GCS connected (SITL only)
                                         --headless        Run headless (SITL only)
    flash <firmware.bin> [options]     Flash firmware
                                       Options:
                                         --board <name>    Board for layout lookup
                                         --device <sel>    Device selector
    layout <show|reset> --board <name> Manage flash layout locks
    dfu [port]                         Enter DFU mode without flashing
    cleanup                            Clean up lingering SITL processes
    test-mavlink                       Run MAVLink heterogeneous tests (Python)
    test [config]                      Run SITL test (defaults to basic_flight)
    create <name> [--board B]          Create a new app from template
    help                               Show this help

DEVICE SELECTORS:
    serial=XXXXXXXX     Select by USB serial number
    path=/dev/ttyUSB0   Select by port path
    /dev/ttyACM0        Auto-infer as port path
    ABCD1234            Auto-infer as serial number

EXAMPLES:
    cargo xtask run                                              # Run x500 on SITL
    cargo xtask run --airframe quad-x --board micoair-h743-v2    # Build and flash
    cargo xtask run --board micoair-h743-v2 --device serial=123  # Flash specific device
    cargo xtask flash app.bin --board micoair-h743-v2            # Flash with layout
    cargo xtask layout show --board micoair-h743-v2              # Show layout
    cargo xtask layout reset --board micoair-h743-v2             # Reset layout lock
    cargo xtask dfu                                              # Enter DFU mode
    cargo xtask cleanup                                          # Kill SITL processes

REQUIREMENTS:
    - dfu-util installed and in PATH (for STM32 ROM DFU)
    - arm-none-eabi-objcopy (for hardware build)
    - cargo-generate (for app generation)
"#
    );
}

// ... existing helper functions ...

// Note: I will insert run_python_test at the end

/// Run Python heterogeneous tests
fn run_python_test() -> Result<()> {
    let config_path = "tests/missions/basic_flight.toml";
    let script_path = "tests/python/test_connection.py";

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
        bail!(
            "AviateGzPlugin not found at {}. Build with CMake first.",
            plugin_dir.display()
        );
    }

    // 3. Run gcs-test in RunScript mode
    eprintln!("Running python test: {}", script_path);
    let mut cmd = Command::new("target/debug/gcs-test");
    cmd.args(["run-script", config_path, script_path]);

    // Add plugin dir to LD_LIBRARY_PATH
    if let Ok(current_ld) = std::env::var("LD_LIBRARY_PATH") {
        cmd.env(
            "LD_LIBRARY_PATH",
            format!("{}:{}", plugin_dir.display(), current_ld),
        );
    } else {
        cmd.env("LD_LIBRARY_PATH", plugin_dir);
    }

    let status = cmd.status().context("Failed to run gcs-test")?;

    if !status.success() {
        bail!("Python test failed");
    }

    Ok(())
}

/// Create a new app from template
fn run_create_app(app_name: &str, board: &str) -> Result<()> {
    // Check if cargo-generate is installed
    if Command::new("cargo")
        .arg("generate")
        .arg("--version")
        .output()
        .is_err()
    {
        bail!("cargo-generate not found. Please install it with: cargo install cargo-generate");
    }

    let template_path = "aviate-app-template";
    if !std::path::Path::new(template_path).exists() {
        bail!("Template directory '{}' not found", template_path);
    }

    // Determine target directory (aviate-apps/aviate-app-<name>)
    // We want the folder name to be consistent with conventions
    let project_name = if app_name.starts_with("aviate-app-") {
        app_name.strip_prefix("aviate-app-").unwrap()
    } else {
        app_name
    };

    let full_app_name = format!("aviate-app-{}", project_name);
    let target_dir = Path::new("aviate-apps").join(&full_app_name);

    if target_dir.exists() {
        bail!("Target directory {} already exists", target_dir.display());
    }

    eprintln!(
        "Creating new app '{}' for board '{}'...",
        full_app_name, board
    );
    eprintln!("Destination: aviate-apps/");

    let status = Command::new("cargo")
        .arg("generate")
        .arg("--path")
        .arg(template_path)
        .arg("--name")
        .arg(&full_app_name)
        .arg("--destination")
        .arg("aviate-apps")
        .arg("--define")
        .arg(format!("board={}", board))
        .arg("--define")
        .arg(format!("project-name={}", project_name))
        .arg("--silent")
        .status()
        .context("Failed to run cargo generate")?;

    if !status.success() {
        bail!("cargo generate failed");
    }

    eprintln!("Created {} in aviate-apps/", full_app_name);
    eprintln!("To build: cargo xtask run {}", project_name);

    Ok(())
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

/// Wait for DFU device to enumerate (STM32 default VID:PID)
fn wait_for_dfu_device() -> Result<()> {
    // Default STM32 DFU VID:PID
    programmer::wait_for_dfu_device(0x0483, 0xdf11, Duration::from_secs(10))
}

/// Generate, build, and run on SITL
fn run_sitl(
    airframe: &str,
    board: &str,
    mission_config: Option<&str>,
    _gcs: bool,
    headless: bool,
) -> Result<()> {
    // 1. Generate and Build App
    let (_app_name, binary_path) = generate_and_build_app(airframe, board)?;

    // 2. Prepare Environment
    let cwd = std::env::current_dir()?;
    let plugin_dir = cwd.join("aviate-hal/xil/backends/gz/plugin/build");

    // Validating plugin existence only if using gazebo board
    if board == "sitl-gazebo" && !plugin_dir.join("libAviateGzPlugin.so").exists() {
        bail!(
            "AviateGzPlugin not found at {}. Build with CMake first.",
            plugin_dir.display()
        );
    }

    // 3. Run
    // If mission_config is provided, use gcs-test in Run mode
    if let Some(config_path) = mission_config {
        eprintln!("Running mission: {}", config_path);
        run_cleanup()?; // Ensure clean state

        // Build gcs-test first
        let status = Command::new("cargo")
            .args(["build", "-p", "gcs-test", "--features", "gazebo"])
            .status()
            .context("Failed to build gcs-test")?;
        if !status.success() {
            bail!("Failed to build gcs-test");
        }

        let mut cmd = Command::new("target/debug/gcs-test");
        cmd.arg("run");
        cmd.arg("--xil");
        cmd.arg(config_path);

        // Pass the custom binary path to gcs-test
        cmd.arg("--fc-binary");
        cmd.arg(&binary_path);

        // Headless handling:
        // gcs-test defaults to headless=true.
        // User wants default GUI for interactive `run` command, but "tests should always be headless".
        // If mission is executing, it is a test. So minimal/headless is better.
        // If user explicitly requests headless (headless=true here), we pass --headless flag.
        // If user explicitly wants GUI in test (how? we didn't add --gui flag), they are stuck with headless unless they don't pass headless flag via xtask?
        // Wait, gcs-test defaults to headless=true. So if we DON'T pass --headless, it is headless.
        // If we want GUI, we need to pass something to disable it, but gcs-test arg is flag for true.
        // If I want to force GUI I might not be able to if default is true in clap.
        // Ah, clap `conflicts_with` or `action = SetTrue`.
        // Let's assume for MISSION run, headless is fine.
        // If user wants headless, we pass --headless (redundant but safe).
        if headless {
            cmd.arg("--headless");
        }

        if let Ok(current_ld) = std::env::var("LD_LIBRARY_PATH") {
            cmd.env(
                "LD_LIBRARY_PATH",
                format!("{}:{}", plugin_dir.display(), current_ld),
            );
        } else {
            cmd.env("LD_LIBRARY_PATH", plugin_dir);
        }

        let status = cmd.status().context("Failed to run gcs-test")?;
        if !status.success() {
            bail!("Mission failed");
        }
    } else {
        // Interactive mode: Run the binary directly
        // If gcs=true, we might want to also launch gcs-test in GCS-only mode?
        // But for now, just launch the app.
        eprintln!("Running interactive SITL: {}", binary_path.display());
        run_cleanup()?;

        // Launch Gazebo environment directly?
        // SITL binary usually launches Gazebo if it's the main one?
        // Or do we need spawner?
        // SitlGazeboBoard (in test-gazebo-app) launches Gazebo unless --connect is passed.
        // Interactive mode `run_interactive` launches Gazebo.
        // So just running the binary is enough.

        let mut cmd = Command::new(&binary_path);
        if headless {
            cmd.arg("--headless");
        }
        // If not headless, it will launch GUI (default for app).

        // Environment variables
        if let Ok(current_ld) = std::env::var("LD_LIBRARY_PATH") {
            cmd.env(
                "LD_LIBRARY_PATH",
                format!("{}:{}", plugin_dir.display(), current_ld),
            );
        } else {
            cmd.env("LD_LIBRARY_PATH", plugin_dir);
        }

        let status = cmd.status().context("Failed to run app")?;
        if !status.success() {
            bail!("App exited with error");
        }
    }

    Ok(())
}

/// Generate and build app from template
fn generate_and_build_app(airframe: &str, board: &str) -> Result<(String, std::path::PathBuf)> {
    // We generate into aviate-apps/<board>-<airframe>
    let gen_name = format!("{}-{}", board, airframe).replace("_", "-");
    let gen_target_dir = Path::new("aviate-apps").join(&gen_name);

    if gen_target_dir.exists() {
        eprintln!(
            "Directory {} already exists. Overwriting as per HEADLESS/CI mode...",
            gen_target_dir.display()
        );
        std::fs::remove_dir_all(&gen_target_dir).context("Failed to clean generated app dir")?;
    }

    let template_path = "aviate-app-template";

    eprintln!("Generating app {}...", gen_name);
    eprintln!("  Board: {}", board);
    eprintln!("  Airframe: {}", airframe);

    // Determine environment based on board type (simulator boards have sitl-/hitl-/xil- prefix)
    let is_simulator = board.starts_with("sitl-") || board.starts_with("hitl-") || board.starts_with("xil-");
    let env = if is_simulator { "sitl" } else { "flight" };

    let status = Command::new("cargo")
        .arg("generate")
        .arg("--path")
        .arg(template_path)
        .arg("--name")
        .arg(&gen_name)
        .arg("--destination")
        .arg("aviate-apps")
        .arg("--define")
        .arg(format!("board={}", board))
        .arg("--define")
        .arg(format!("airframe={}", airframe))
        .arg("--define")
        .arg(format!("env={}", env))
        .arg("--define")
        .arg(format!("project-name={}", gen_name))
        .arg("--silent")
        .status()
        .context("Failed to run cargo generate")?;

    if !status.success() {
        bail!("cargo generate failed");
    }

    // For hardware apps, add to workspace exclude to avoid glob conflict
    if !is_simulator {
        add_to_workspace_exclude(&format!("aviate-apps/{}", gen_name))?;
    }

    eprintln!("Building {}...", gen_name);

    // Build
    let mut cmd = Command::new("cargo");
    cmd.arg("build");

    if !is_simulator {
        // Hardware apps are standalone workspaces, build from app directory
        // so .cargo/config.toml with target-specific rustflags is used
        cmd.current_dir(&gen_target_dir);
        cmd.arg("--release");
    } else {
        // SITL apps are part of main workspace
        cmd.arg("-p");
        cmd.arg(format!("aviate-app-{}", gen_name));
    }

    let status = cmd.status().context("Failed to build generated app")?;
    if !status.success() {
        bail!("Build failed");
    }

    // Locate binary
    let bin_path_buf = if !is_simulator {
        // Hardware builds go to app-local target directory (target specified in .cargo/config.toml)
        gen_target_dir
            .join("target/thumbv7em-none-eabihf/release")
            .join(&gen_name)
    } else {
        Path::new("target/debug").join(&gen_name)
    };

    Ok((gen_name, bin_path_buf))
}

/// Run on hardware - build app and flash to device
///
/// Automatically detects device state:
/// - ROM DFU: Build production bootloader (no software-dfu) + app, flash both
/// - Running app: Software DFU entry, flash app only
/// - Not found: Error with ROM DFU instructions
fn run_hardware(airframe: &str, board: &str, device_arg: Option<&str>) -> Result<()> {
    // Resolve board metadata
    let board_meta = boards::resolve_board(board)?;

    if board_meta.is_simulator() {
        bail!("Board '{}' is a simulator, use SITL mode instead", board);
    }

    // Detect device state
    let vid = board_meta.vid.or(board_meta.programmer.default_vid());
    let bootloader_pid = board_meta.pid.or(board_meta.programmer.default_bootloader_pid());
    let state = device::detect_device_state(vid, bootloader_pid)?;

    // Create device selector
    let device = DeviceSelector::from_args(&board_meta.chip, device_arg)?;

    match state {
        DeviceState::RomBootloader => {
            // Full flash: production bootloader + app
            eprintln!("ROM DFU detected - flashing production bootloader + app");

            // Build production bootloader (no software-dfu for security)
            let bootloader_bin = build_bootloader(board, true)?;

            // Build app
            let (app_name, elf_path) = generate_and_build_app(airframe, board)?;
            let app_bin = convert_to_binary(&elf_path, &app_name)?;

            // Compute layout from bootloader size
            let bootloader_size = layout::get_bootloader_size(&bootloader_bin)?;
            let geo = geometry::get_geometry_from_probe_rs(&board_meta.chip)?;
            let layout_result = layout::resolve_layout_with_lock(
                &geo,
                bootloader_size,
                &board_meta.board_dir,
                board_meta.reserve_mode,
            )?;

            // Flash both segments
            let plan = FlashPlan::stm32_full(bootloader_bin, app_bin, &layout_result);
            programmer::execute_flash_plan(board_meta.programmer, &plan, &device)?;
        }

        DeviceState::CustomBootloaderDfu => {
            // Custom bootloader DFU - flash app only (bootloader already installed)
            eprintln!("Custom bootloader DFU detected - flashing app only");

            // Build app only
            let (app_name, elf_path) = generate_and_build_app(airframe, board)?;
            let app_bin = convert_to_binary(&elf_path, &app_name)?;
            let app_address = get_app_address_for_board(&board_meta)?;

            let plan = FlashPlan::app_only(app_bin, app_address);
            programmer::execute_flash_plan(board_meta.programmer, &plan, &device)?;
        }

        DeviceState::Running(port) => {
            // App-only flash via software DFU
            eprintln!("Device running on {} - using software DFU", port);

            // Enter DFU mode via serial
            enter_dfu_mode(Some(&port))?;

            // Build app only
            let (app_name, elf_path) = generate_and_build_app(airframe, board)?;
            let app_bin = convert_to_binary(&elf_path, &app_name)?;
            let app_address = get_app_address_for_board(&board_meta)?;

            let plan = FlashPlan::app_only(app_bin, app_address);
            programmer::execute_flash_plan(board_meta.programmer, &plan, &device)?;
        }

        DeviceState::NotFound => {
            bail!(
                "No device found!\n\n\
                 To enter ROM DFU mode:\n\
                 1. Hold BOOT button\n\
                 2. Press RESET (or power cycle)\n\
                 3. Release BOOT\n\n\
                 Then run this command again."
            );
        }
    }

    eprintln!("Flash complete!");

    // Wait for device to restart
    std::thread::sleep(Duration::from_secs(2));

    // Check if app started
    if let Some(vid) = board_meta.programmer.default_vid() {
        match device::find_serial_port_by_vid(vid)? {
            Some(port) => eprintln!("Device running on {}", port),
            None => eprintln!("Device may still be starting up..."),
        }
    }

    Ok(())
}

/// Get app start address for a board
fn get_app_address_for_board(board_meta: &BoardMetadata) -> Result<u32> {
    // Try to load existing layout lock
    if let Some(lock) = layout::load_layout_lock(&board_meta.board_dir)? {
        return Ok(lock.app_start);
    }

    // No lock - compute from geometry
    if board_meta.is_stm32() {
        let geo = geometry::get_geometry_from_probe_rs(&board_meta.chip)?;
        // Use default 30KB bootloader size estimate for initial layout
        let bootloader_size = 32 * 1024;
        let computed = layout::BootloaderInFirstSectors.compute_layout(&geo, bootloader_size)?;

        eprintln!(
            "No layout.lock.json found for {}. Using computed app_start: {:#x}",
            board_meta.package_name, computed.app_start
        );
        eprintln!(
            "Run 'cargo xtask layout reset --board {}' to create a lock file.",
            board_meta.package_name.trim_start_matches("aviate-board-")
        );

        Ok(computed.app_start)
    } else {
        // Non-STM32 boards need explicit layout
        bail!(
            "Board '{}' requires layout.lock.json. Run 'cargo xtask layout reset --board {}'",
            board_meta.package_name,
            board_meta.package_name.trim_start_matches("aviate-board-")
        );
    }
}

/// Flash firmware command (standalone flash)
fn flash_firmware_cmd(
    firmware_path: &str,
    board_name: Option<&str>,
    device_arg: Option<&str>,
) -> Result<()> {
    // Check firmware file exists
    if !std::path::Path::new(firmware_path).exists() {
        bail!("Firmware file not found: {}", firmware_path);
    }

    // Get board metadata if provided
    let (programmer, app_address, chip) = if let Some(board) = board_name {
        let board_meta = boards::resolve_board(board)?;
        let addr = get_app_address_for_board(&board_meta)?;
        (board_meta.programmer, addr, board_meta.chip)
    } else {
        // Legacy mode: default STM32 DFU
        eprintln!("Warning: No --board specified, using legacy defaults (0x08020000)");
        (
            Programmer::Stm32RomDfu,
            0x08020000,
            "STM32H743VITx".to_string(),
        )
    };

    // Check tool available
    programmer.check_tool_available()?;

    // Create device selector
    let device = DeviceSelector::from_args(&chip, device_arg)?;

    // Check if already in DFU mode
    if let (Some(vid), Some(pid)) = (
        programmer.default_vid(),
        programmer.default_bootloader_pid(),
    ) {
        if !programmer::dfu_device_present(vid, pid, device.serial.as_deref())? {
            // Try to enter DFU mode
            enter_dfu_mode(device.port.as_deref())?;
        } else {
            eprintln!("Device already in DFU mode");
        }
    }

    // Create and execute flash plan
    let plan = FlashPlan::app_only(std::path::PathBuf::from(firmware_path), app_address);
    programmer::execute_flash_plan(programmer, &plan, &device)?;

    eprintln!("Flash complete!");

    // Wait for device to restart
    std::thread::sleep(Duration::from_secs(2));

    // Check if app started
    if let Some(vid) = programmer.default_vid() {
        match device::find_serial_port_by_vid(vid)? {
            Some(port) => eprintln!("Device running on {}", port),
            None => eprintln!("Device may still be starting up..."),
        }
    }

    Ok(())
}

/// Layout management command
fn run_layout_cmd(subcmd: &str, board_name: &str) -> Result<()> {
    let board_meta = boards::resolve_board(board_name)?;

    if board_meta.is_simulator() {
        bail!("Layout management not applicable to simulator boards");
    }

    match subcmd {
        "show" => {
            eprintln!("Board: {}", board_meta.package_name);
            eprintln!("Chip: {}", board_meta.chip);
            eprintln!("Programmer: {:?}", board_meta.programmer);
            eprintln!("Board directory: {}", board_meta.board_dir.display());
            eprintln!();

            // Show geometry
            if board_meta.is_stm32() {
                match geometry::get_geometry_from_probe_rs(&board_meta.chip) {
                    Ok(geo) => {
                        eprintln!("Flash geometry (from probe-rs):");
                        eprintln!("  Base: {:#x}", geo.flash_base);
                        eprintln!(
                            "  Size: {:#x} ({} KB)",
                            geo.flash_size,
                            geo.flash_size / 1024
                        );
                        eprintln!("  Sectors: {}", geo.sectors.len());
                        if let Some(first) = geo.sectors.first() {
                            eprintln!("  First sector size: {:#x}", first.size);
                        }
                    }
                    Err(e) => eprintln!("  Failed to get geometry: {}", e),
                }
            }

            eprintln!();

            // Show lock file
            match layout::load_layout_lock(&board_meta.board_dir)? {
                Some(lock) => {
                    eprintln!("Layout lock (from layout.lock.json):");
                    eprintln!("  Flash base: {:#x}", lock.flash_base);
                    eprintln!("  Bootloader reserve: {:#x}", lock.bootloader_reserve);
                    eprintln!("  App start: {:#x}", lock.app_start);
                    eprintln!(
                        "  Computed from bootloader size: {} bytes",
                        lock.computed_from_bootloader_size
                    );
                }
                None => {
                    eprintln!("No layout.lock.json found.");
                    eprintln!(
                        "Run 'cargo xtask layout reset --board {}' to create one.",
                        board_name
                    );
                }
            }
        }
        "reset" => {
            if !board_meta.is_stm32() {
                bail!("Layout reset only supported for STM32 boards currently");
            }

            let geo = geometry::get_geometry_from_probe_rs(&board_meta.chip)?;

            // Use a reasonable default bootloader size
            // In production, this would be read from a built bootloader binary
            let bootloader_size = 32 * 1024; // 32KB default estimate

            eprintln!(
                "Computing layout for {} with {} byte bootloader estimate...",
                board_meta.chip, bootloader_size
            );

            let layout_result = layout::resolve_layout_with_lock(
                &geo,
                bootloader_size,
                &board_meta.board_dir,
                board_meta.reserve_mode,
            )?;

            eprintln!("Layout created:");
            eprintln!("  Bootloader: {:#x}", layout_result.bootloader_start);
            eprintln!(
                "  Reserve: {:#x} ({} KB)",
                layout_result.bootloader_reserve,
                layout_result.bootloader_reserve / 1024
            );
            eprintln!("  App start: {:#x}", layout_result.app_start);
            eprintln!();
            eprintln!(
                "Lock file saved to: {}",
                board_meta.board_dir.join("layout.lock.json").display()
            );
        }
        other => bail!(
            "Unknown layout subcommand: {}. Use 'show' or 'reset'.",
            other
        ),
    }

    Ok(())
}

// Keep the old find_serial_port for backward compatibility
fn find_serial_port() -> Result<String> {
    device::find_any_serial_port()
}

/// Build bootloader for a board
///
/// production=true: no software-dfu (for ROM DFU first-time flash)
/// production=false: with software-dfu (for dev bootloader)
fn build_bootloader(board: &str, production: bool) -> Result<PathBuf> {
    let features = if production {
        board.to_string()
    } else {
        format!("{},software-dfu", board)
    };

    eprintln!(
        "Building {} bootloader with features: {}",
        if production { "production" } else { "dev" },
        features
    );

    let status = Command::new("cargo")
        .current_dir("aviate-bootloader")
        .args(["build", "--release", "--features", &features])
        .status()
        .context("Failed to run cargo build for bootloader")?;

    if !status.success() {
        bail!("Bootloader build failed");
    }

    // Determine the ELF path based on target architecture
    // STM32H7 uses thumbv7em-none-eabihf, RP2350 uses thumbv8m.main-none-eabihf
    let elf_path = PathBuf::from("aviate-bootloader/target/thumbv7em-none-eabihf/release/aviate-bootloader");

    if !elf_path.exists() {
        // Try thumbv8m target for RP2350
        let alt_path = PathBuf::from("aviate-bootloader/target/thumbv8m.main-none-eabihf/release/aviate-bootloader");
        if alt_path.exists() {
            return convert_to_binary(&alt_path, "aviate-bootloader");
        }
        bail!(
            "Bootloader ELF not found at {} or thumbv8m variant",
            elf_path.display()
        );
    }

    convert_to_binary(&elf_path, "aviate-bootloader")
}

/// Convert ELF to binary using arm-none-eabi-objcopy
fn convert_to_binary(elf_path: &Path, name: &str) -> Result<PathBuf> {
    let bin_path = PathBuf::from(format!("/tmp/{}.bin", name));

    let status = Command::new("arm-none-eabi-objcopy")
        .args(["-O", "binary"])
        .arg(elf_path)
        .arg(&bin_path)
        .status()
        .context("Failed to run arm-none-eabi-objcopy")?;

    if !status.success() {
        bail!("objcopy failed for {}", elf_path.display());
    }

    eprintln!("Created binary: {} ({} bytes)", bin_path.display(), std::fs::metadata(&bin_path)?.len());

    Ok(bin_path)
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
        bail!(
            "AviateGzPlugin not found at {}. Build with CMake first.",
            plugin_dir.display()
        );
    }

    // 3. Run gcs-test
    eprintln!("Running test: {}", config_path);
    let mut cmd = Command::new("target/debug/gcs-test");
    cmd.args(["run", "--xil", config_path]);

    // Add plugin dir to LD_LIBRARY_PATH
    if let Ok(current_ld) = std::env::var("LD_LIBRARY_PATH") {
        cmd.env(
            "LD_LIBRARY_PATH",
            format!("{}:{}", plugin_dir.display(), current_ld),
        );
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

/// Add a path to the workspace exclude list in root Cargo.toml
///
/// This is needed because:
/// 1. We use `aviate-apps/*` glob for SITL apps (simplicity, no hardcoded names)
/// 2. Hardware apps have their own `[workspace]` (different target)
/// 3. Cargo doesn't auto-exclude packages with `[workspace]` from parent globs
///
/// So we dynamically add hardware apps to exclude when generating them.
fn add_to_workspace_exclude(path: &str) -> Result<()> {
    let cargo_toml_path = Path::new("Cargo.toml");
    let content = std::fs::read_to_string(cargo_toml_path)
        .context("Failed to read root Cargo.toml")?;

    // Check if already excluded
    if content.contains(&format!("\"{}\"", path)) {
        return Ok(()); // Already in exclude list
    }

    // Find the exclude array and add the path
    // Look for pattern: exclude = [\n    ...\n]
    let exclude_pattern = Regex::new(r#"exclude\s*=\s*\["#).unwrap();

    if let Some(m) = exclude_pattern.find(&content) {
        // Find the closing bracket of the exclude array
        let start = m.end();
        let mut depth = 1;
        let mut end = start;
        for (i, c) in content[start..].char_indices() {
            match c {
                '[' => depth += 1,
                ']' => {
                    depth -= 1;
                    if depth == 0 {
                        end = start + i;
                        break;
                    }
                }
                _ => {}
            }
        }

        // Insert before the closing bracket
        let new_content = format!(
            "{}    \"{}\",\n{}",
            &content[..end],
            path,
            &content[end..]
        );

        std::fs::write(cargo_toml_path, new_content)
            .context("Failed to write root Cargo.toml")?;

        eprintln!("Added '{}' to workspace exclude list", path);
    }

    Ok(())
}
