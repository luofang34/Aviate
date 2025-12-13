#![forbid(unsafe_code)]

//! Aviate Gazebo SITL - X500 Quadcopter
//!
//! Runs the aviate-core flight controller in software-in-the-loop mode,
//! with direct integration to Gazebo via shared memory FFI.
//!
//! ## Usage
//!
//! ```bash
//! # Interactive mode (Gazebo with GUI, wait for GCS):
//! cargo run -p aviate-app-sitl-gazebo-x500
//!
//! # Headless mode (no GUI, wait for GCS):
//! cargo run -p aviate-app-sitl-gazebo-x500 -- --headless
//!
//! # Mock mode (no Gazebo, for unit testing):
//! cargo run -p aviate-app-sitl-gazebo-x500 -- --mock
//!
//! # Multi-vehicle instance:
//! cargo run -p aviate-app-sitl-gazebo-x500 -- --instance 1
//! ```
//!
//! For SITL testing with missions, use the gcs-test tool:
//! ```bash
//! cargo run -p gcs-test --features gazebo -- run --xil tests/xil-missions/basic_flight.toml
//! ```
//!
//! ## Environment Variables
//!
//! - `AVIATE_INSTANCE`: Instance ID for multi-vehicle (default: 0)
//! - `HEADLESS`: Set to "1" for headless mode

use std::env;
use std::path::Path;
use std::process::ExitCode;

use aviate_board_sitl_gazebo::GazeboSitlBoard;

/// Simple logging macros
macro_rules! info {
    ($($arg:tt)*) => {
        eprintln!("[INFO] {}", format_args!($($arg)*));
    };
}

macro_rules! warn {
    ($($arg:tt)*) => {
        eprintln!("[WARN] {}", format_args!($($arg)*));
    };
}

/// Command line options
struct Options {
    mock: bool,
    headless: bool,
    instance: u8,
    /// Connect to existing Gazebo (don't launch new one)
    connect: bool,
}

impl Options {
    fn parse() -> Self {
        let args: Vec<String> = env::args().collect();

        // Check for --mock
        let mock = args.iter().any(|a| a == "--mock");

        // Check for --headless or HEADLESS env
        let headless = args.iter().any(|a| a == "--headless")
            || env::var("HEADLESS").map(|v| v == "1").unwrap_or(false);

        // Check for --connect (skip Gazebo launch, assume it's already running)
        let connect = args.iter().any(|a| a == "--connect");

        // Check for --instance <N> or AVIATE_INSTANCE env
        let instance = args
            .iter()
            .position(|a| a == "--instance")
            .and_then(|i| args.get(i + 1))
            .and_then(|v| v.parse().ok())
            .or_else(|| env::var("AVIATE_INSTANCE").ok()?.parse().ok())
            .unwrap_or(0);

        Self {
            mock,
            headless,
            instance,
            connect,
        }
    }
}

fn main() -> ExitCode {
    let opts = Options::parse();

    // Load application configuration (LOW-DAL init phase)
    const APP_CONFIG_TOML: &str = include_str!("../AviateApp.toml");
    let app_config = match aviate_config::from_toml_str(APP_CONFIG_TOML) {
        Ok(config) => config,
        Err(_) => {
            eprintln!("[ERROR] Failed to parse AviateApp.toml");
            return ExitCode::FAILURE;
        }
    };

    // Validate configuration
    if aviate_config::validate(&app_config).is_err() {
        eprintln!("[ERROR] Invalid configuration in AviateApp.toml");
        return ExitCode::FAILURE;
    }

    // Extract simulator config (CLI args override TOML defaults)
    let headless = opts.headless
        || app_config
            .simulator
            .as_ref()
            .map(|s| s.headless)
            .unwrap_or(false);

    // Print banner
    println!("===========================================");
    println!("  Aviate Gazebo SITL - X500 Quadcopter");
    println!("===========================================");
    println!();
    println!("App ID:   {}", app_config.app.id);
    println!("Board:    {}", app_config.app.board);
    println!("Airframe: {}", app_config.app.airframe);
    println!("Env:      {}", app_config.app.env);
    println!("Instance: {}", opts.instance);
    println!();

    if opts.mock {
        info!("Mode: Mock (no Gazebo)");
        run_mock();
        ExitCode::SUCCESS
    } else if opts.connect {
        info!("Mode: Connect (external Gazebo)");
        run_connect(opts.instance)
    } else {
        info!(
            "Mode: Interactive (headless={})",
            if headless { "yes" } else { "no" }
        );
        run_interactive(headless, opts.instance)
    }
}

/// Mock mode: No Gazebo, just run FC with mock sensors
fn run_mock() {
    use aviate_core::hal::SystemHal;
    use aviate_hal_xil::SitlHal;

    info!("Starting mock mode (no simulator)");
    let mut hal = SitlHal::new();

    loop {
        hal.kick_watchdog();
        std::thread::sleep(std::time::Duration::from_millis(10));
    }
}

/// Connect mode: Connect to existing Gazebo (doesn't launch new one)
/// Used when gcs-test spawns this FC process and manages Gazebo separately
#[cfg(feature = "gz-plugin")]
fn run_connect(instance: u8) -> ExitCode {
    use aviate_backend_gz::GzPluginBridge;
    use aviate_hal_xil::SitlConfig;
    use std::time::Duration;

    let config = SitlConfig::for_instance(instance);
    info!("Sensor port: {}", config.sensor_port());

    // Wait for shared memory to be ready (Gazebo launched by gcs-test)
    info!("Waiting for Gazebo plugin (launched externally)...");
    if !wait_for_shm(Duration::from_secs(30)) {
        warn!("Timeout waiting for Gazebo plugin");
        return ExitCode::FAILURE;
    }
    info!("Gazebo ready");

    // Load app config for telemetry
    const APP_CONFIG_TOML: &str = include_str!("../AviateApp.toml");
    let app_config = aviate_config::from_toml_str(APP_CONFIG_TOML).ok();

    // Create the board
    let mut board = match GazeboSitlBoard::with_config_retry(config, 5, 1000) {
        Ok(b) => b,
        Err(e) => {
            warn!("Failed to initialize board: {}", e);
            return ExitCode::FAILURE;
        }
    };

    // Initialize telemetry (sends HEARTBEAT, ATTITUDE, POSITION to GCS)
    if let Some(ref cfg) = app_config {
        board.init_telemetry(cfg, 1000); // 1kHz control loop
    }

    // Connect to Gazebo via shared memory
    let plugin = match GzPluginBridge::connect_instance_with_retry(instance, 20, 500) {
        Ok(p) => {
            info!("Connected to Gazebo plugin");
            p
        }
        Err(e) => {
            warn!("Failed to connect to Gazebo: {}", e);
            return ExitCode::FAILURE;
        }
    };

    info!("Flight controller ready - waiting for GCS commands");
    info!("Connect with QGroundControl or send MAVLink commands");
    println!();

    // Main control loop
    run_control_loop(&mut board, &plugin);

    ExitCode::SUCCESS
}

#[cfg(not(feature = "gz-plugin"))]
fn run_connect(_instance: u8) -> ExitCode {
    warn!("gz-plugin feature not enabled");
    warn!("Rebuild with: cargo build -p aviate-app-sitl-gazebo-x500 --features gz-plugin");
    ExitCode::FAILURE
}

/// Interactive mode: Launch Gazebo, run FC, wait for GCS commands
#[cfg(feature = "gz-plugin")]
fn run_interactive(headless: bool, instance: u8) -> ExitCode {
    use aviate_app_sitl_gazebo_x500::{generate_world, parse_test_config_str, WorldParams};
    use aviate_backend_gz::GzPluginBridge;
    use aviate_hal_xil::SitlConfig;
    use std::time::Duration;

    let config = SitlConfig::for_instance(instance);
    info!("Sensor port: {}", config.sensor_port());

    // Generate a default world with single X500
    let test_config = parse_test_config_str(DEFAULT_WORLD_CONFIG).expect("valid default config");
    let world_params = WorldParams::default();

    // Write world to temp file
    let world_sdf = generate_world(&test_config, &world_params);
    let world_path = std::env::temp_dir().join("aviate_interactive.sdf");
    std::fs::write(&world_path, &world_sdf).expect("write world file");
    info!("World file: {}", world_path.display());

    // Clean up any existing Gazebo processes
    cleanup_gazebo();

    // Launch Gazebo
    let gz_child = match launch_gazebo(&world_path, headless) {
        Ok(child) => {
            info!(
                "Gazebo started (PID: {}, headless={})",
                child.id(),
                headless
            );
            child
        }
        Err(e) => {
            warn!("Failed to launch Gazebo: {}", e);
            return ExitCode::FAILURE;
        }
    };

    // Wait for shared memory to be ready
    info!("Waiting for Gazebo plugin...");
    if !wait_for_shm(Duration::from_secs(30)) {
        warn!("Timeout waiting for Gazebo plugin");
        drop(gz_child);
        cleanup_gazebo();
        return ExitCode::FAILURE;
    }
    info!("Gazebo ready");

    // Load app config for telemetry
    const APP_CONFIG_TOML: &str = include_str!("../AviateApp.toml");
    let app_config = aviate_config::from_toml_str(APP_CONFIG_TOML).ok();

    // Create the board
    let mut board = match GazeboSitlBoard::with_config_retry(config, 5, 1000) {
        Ok(b) => b,
        Err(e) => {
            warn!("Failed to initialize board: {}", e);
            drop(gz_child);
            cleanup_gazebo();
            return ExitCode::FAILURE;
        }
    };

    // Initialize telemetry (sends HEARTBEAT, ATTITUDE, POSITION to GCS)
    if let Some(ref cfg) = app_config {
        board.init_telemetry(cfg, 1000); // 1kHz control loop
    }

    // Connect to Gazebo via shared memory
    let plugin = match GzPluginBridge::connect_instance_with_retry(instance, 20, 500) {
        Ok(p) => {
            info!("Connected to Gazebo plugin");
            p
        }
        Err(e) => {
            warn!("Failed to connect to Gazebo: {}", e);
            drop(gz_child);
            cleanup_gazebo();
            return ExitCode::FAILURE;
        }
    };

    info!("Flight controller ready - waiting for GCS commands");
    info!("Connect with QGroundControl or send MAVLink commands");
    println!();

    // Main control loop
    run_control_loop(&mut board, &plugin);

    // Cleanup
    drop(gz_child);
    cleanup_gazebo();
    ExitCode::SUCCESS
}

#[cfg(not(feature = "gz-plugin"))]
fn run_interactive(_headless: bool, _instance: u8) -> ExitCode {
    warn!("gz-plugin feature not enabled");
    warn!("Rebuild with: cargo build -p aviate-app-sitl-gazebo-x500 --features gz-plugin");
    ExitCode::FAILURE
}

// ============================================================================
// Gazebo Management
// ============================================================================

#[cfg(feature = "gz-plugin")]
fn launch_gazebo(world_path: &Path, headless: bool) -> Result<std::process::Child, std::io::Error> {
    use std::process::{Command, Stdio};

    // Set up environment paths
    let aviate_dir = env::current_dir().unwrap_or_default();

    // Model path
    let local_models = aviate_dir.join("models");
    let px4_models = aviate_dir.join("external/PX4-gazebo-models/models");
    let mut model_paths = Vec::new();
    if local_models.exists() {
        model_paths.push(local_models.to_string_lossy().to_string());
    }
    if px4_models.exists() {
        model_paths.push(px4_models.to_string_lossy().to_string());
    }
    let gz_resource_path = model_paths.join(":");

    // Plugin path
    let plugin_dir = aviate_dir.join("aviate-hal/xil/backends/gz/plugin/build");
    let gz_plugin_path = plugin_dir.to_string_lossy().to_string();

    let mut cmd = Command::new("gz");
    cmd.arg("sim");

    if headless {
        cmd.arg("-s"); // Server only
        cmd.arg("-r"); // Run immediately
        cmd.arg("--headless-rendering");
        cmd.env_remove("DISPLAY");
    } else {
        cmd.arg("-r"); // Run immediately
    }

    cmd.arg(world_path);

    // Set environment
    if !gz_resource_path.is_empty() {
        let existing = env::var("GZ_SIM_RESOURCE_PATH").unwrap_or_default();
        let combined = if existing.is_empty() {
            gz_resource_path
        } else {
            format!("{}:{}", gz_resource_path, existing)
        };
        cmd.env("GZ_SIM_RESOURCE_PATH", combined);
    }

    if !gz_plugin_path.is_empty() {
        let existing = env::var("GZ_SIM_SYSTEM_PLUGIN_PATH").unwrap_or_default();
        let combined = if existing.is_empty() {
            gz_plugin_path.clone()
        } else {
            format!("{}:{}", gz_plugin_path, existing)
        };
        cmd.env("GZ_SIM_SYSTEM_PLUGIN_PATH", combined);
        cmd.env("LD_LIBRARY_PATH", gz_plugin_path);
    }

    cmd.stdout(Stdio::null());
    cmd.stderr(Stdio::null());

    cmd.spawn()
}

#[cfg(feature = "gz-plugin")]
fn cleanup_gazebo() {
    use std::process::Command;

    let _ = Command::new("pkill").args(["-9", "-f", "gz sim"]).status();
    let _ = std::fs::remove_file("/dev/shm/aviate_gz_bridge");
    std::thread::sleep(std::time::Duration::from_millis(500));
}

#[cfg(feature = "gz-plugin")]
fn wait_for_shm(timeout: std::time::Duration) -> bool {
    use std::time::Instant;

    let shm_path = Path::new("/dev/shm/aviate_gz_bridge");
    let start = Instant::now();

    while start.elapsed() < timeout {
        if shm_path.exists() {
            return true;
        }
        std::thread::sleep(std::time::Duration::from_millis(250));
    }

    false
}

#[cfg(feature = "gz-plugin")]
fn run_control_loop(board: &mut GazeboSitlBoard, plugin: &aviate_backend_gz::GzPluginBridge) {
    use aviate_backend_gz::{enu_to_ned_f32, enu_vel_to_ned_f32};
    use aviate_hal_xil::{
        SimBaroData, SimGnssData, SimGnssFix, SimImuData, SimMagData, SimSensorPacket,
    };

    let loop_period_us = 1000u64; // 1kHz
    let mut last_tick = board.now_us();
    let mut stats_tick = last_tick;
    let mut sensor_count = 0u64;
    let mut motor_count = 0u64;

    loop {
        let now = board.now_us();
        let elapsed = now.saturating_sub(last_tick);

        if elapsed >= loop_period_us {
            last_tick = now;

            // 1. Read physics state from Gazebo
            if let Some(state) = plugin.get_model_state() {
                let ned_pos = enu_to_ned_f32(state.pos);
                let ned_vel = enu_vel_to_ned_f32(state.vel);

                let imu = SimImuData {
                    accel: [0.0, 0.0, -9.81],
                    gyro: [
                        state.ang_vel[0] as f32,
                        -state.ang_vel[1] as f32,
                        -state.ang_vel[2] as f32,
                    ],
                    temperature: Some(25.0),
                };

                let baro = SimBaroData {
                    pressure_pa: 101325.0 + ned_pos[2] * 12.0,
                    temperature_c: 25.0,
                };

                let mag = SimMagData {
                    field_ut: [20.0, 0.0, 40.0],
                };

                let gnss = SimGnssData {
                    lat_deg: ned_pos[0] as f64 / 111000.0,
                    lon_deg: ned_pos[1] as f64 / 111000.0,
                    alt_m: -ned_pos[2],
                    vel_ned: ned_vel,
                    fix: SimGnssFix::ThreeD,
                    h_acc: 1.0,
                    v_acc: 1.5,
                    satellites: 10,
                };

                let packet = SimSensorPacket {
                    timestamp_us: state.time_us,
                    imu: Some(imu),
                    baro: Some(baro),
                    mag: Some(mag),
                    gnss: Some(gnss),
                };

                board.transport_mut().feed_sensor_packet(&packet);
                sensor_count += 1;
            }

            // 2. Run control loop
            board.step();

            // 3. Send actuator commands to Gazebo
            if let Some(cmd) = board.transport_mut().take_actuator_cmd() {
                const MAX_MOTOR_RADS: f64 = 1000.0;
                let velocities: Vec<f64> = cmd
                    .outputs
                    .iter()
                    .take(cmd.count as usize)
                    .map(|&v| (v as f64) * MAX_MOTOR_RADS)
                    .collect();

                let _ = plugin.set_motor_speeds(&velocities);
                motor_count += 1;
            }

            // 4. Print stats every 5 seconds
            if now.saturating_sub(stats_tick) >= 5_000_000 {
                stats_tick = now;
                info!(
                    "sensors={}, motors={}, armed={}",
                    sensor_count,
                    motor_count,
                    board.is_armed()
                );
            }
        } else {
            let remaining_us = loop_period_us - elapsed;
            if remaining_us > 100 {
                std::thread::sleep(std::time::Duration::from_micros(remaining_us - 100));
            }
        }
    }
}

// Default world config for interactive mode
const DEFAULT_WORLD_CONFIG: &str = r#"
name = "interactive"
description = "Interactive flight mode"
lockstep = false

[[vehicles]]
id = "x500"
model = "x500"
instance = 0
spawn_position = [0.0, 0.0, 0.1]
spawn_heading = 0.0

[vehicles.mission]
name = "manual"
phases = []
"#;
