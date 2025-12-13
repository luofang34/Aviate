//! GCS Test Tool
//!
//! CLI tool for running SITL mission tests. Thin wrapper over aviate-hal-xil's
//! MissionRunner. Uses feature flags for optional backends and XIL modes.
//!
//! ## Modes
//!
//! - **GCS-only** (default): Connect to running FC via MAVLink, no simulator control
//! - **XIL mode** (`--xil`): Full SITL/HITL testing with simulator spawning and ground truth
//!
//! ## Usage
//!
//! ```bash
//! # GCS-only mode (FC must already be running)
//! cargo run -p gcs-test -- run tests/xil-missions/basic_flight.toml
//!
//! # XIL mode with Gazebo (spawns simulator, FC, mavrouter)
//! cargo run -p gcs-test --features gazebo -- run --xil tests/xil-missions/basic_flight.toml
//!
//! # List available mission configs
//! cargo run -p gcs-test -- list
//! ```

mod router_gen;
mod spawner;

use std::path::PathBuf;
use std::process::ExitCode;
#[cfg(feature = "gazebo")]
use std::time::Duration;

use aviate_hal_xil::{parse_test_config, run_test_config, SimulatorBackend, SimulatorError};
use clap::{Parser, Subcommand};
use log::{info, warn, error};

#[cfg(feature = "gazebo")]
use router_gen::RouterParams;
#[cfg(feature = "gazebo")]
use spawner::Spawner;

#[derive(Parser, Debug)]
#[command(name = "gcs-test", author, version, about = "GCS test tool for SITL mission testing", long_about = None)]
struct Cli {
    #[command(subcommand)]
    command: Commands,
}

#[derive(Subcommand, Debug)]
enum Commands {
    /// Run a mission test from a TOML config file
    Run {
        /// Path to the mission config file
        config: PathBuf,

        /// XIL mode: spawn simulator, FC, and mavrouter
        /// Without this flag, assumes FC is already running
        #[arg(long)]
        xil: bool,

        /// Run in headless mode (no GUI)
        #[arg(long, default_value = "true")]
        headless: bool,

        /// Use MAVLink-only mode (no ground truth verification)
        #[arg(long)]
        mavlink_only: bool,
    },
    /// Run an external script (e.g. Python) against the SITL environment
    RunScript {
        /// Path to the mission config file (for environment setup)
        config: PathBuf,
        
        /// Path to the script to execute
        script: PathBuf,

        /// Run in headless mode (no GUI)
        #[arg(long, default_value = "true")]
        headless: bool,
    },
    /// List available mission configs
    List,
}

fn main() -> ExitCode {
    // Initialize logger
    env_logger::init();

    let cli = Cli::parse();

    match cli.command {
        Commands::Run {
            config,
            xil,
            headless,
            mavlink_only,
        } => {
            // Parse config
            let test_config = match parse_test_config(&config) {
                Ok(c) => c,
                Err(e) => {
                    error!("Failed to parse config: {}", e);
                    return ExitCode::FAILURE;
                }
            };

            info!("=== GCS Test: {} ===", test_config.name);
            info!("Config: {}", config.display());
            info!("Vehicles: {}", test_config.vehicles.len());
            info!("");

            if xil {
                // XIL mode: spawn everything and run test
                run_xil_test(&test_config, headless, mavlink_only)
            } else {
                // GCS-only mode: FC is already running
                run_gcs_test(&test_config, mavlink_only)
            }
        }
        Commands::RunScript {
            config,
            script,
            headless,
        } => {
            let test_config = match parse_test_config(&config) {
                Ok(c) => c,
                Err(e) => {
                    error!("Failed to parse config: {}", e);
                    return ExitCode::FAILURE;
                }
            };
            run_script_test(&test_config, &script, headless)
        }
        Commands::List => {
            // List mission configs in tests/xil-missions/
            let mission_dir = PathBuf::from("tests/missions");
            if mission_dir.exists() {
                info!("Available mission configs:");
                if let Ok(entries) = std::fs::read_dir(&mission_dir) {
                    for entry in entries.flatten() {
                        if entry
                            .path()
                            .extension()
                            .map(|e| e == "toml")
                            .unwrap_or(false)
                        {
                            info!("  {}", entry.path().display());
                        }
                    }
                }
            } else {
                warn!("Mission directory not found: {}", mission_dir.display());
            }
            ExitCode::SUCCESS
        }
    }
}

/// Run test in GCS-only mode (FC already running)
fn run_gcs_test(test_config: &aviate_hal_xil::TestConfig, mavlink_only: bool) -> ExitCode {
    info!("Mode: GCS-only (FC must be running)");

    let result = if mavlink_only {
        info!("Backend: MAVLink-only (no ground truth)");
        run_test_config(test_config, |instance| {
            Ok(MavlinkOnlyBackend::new(instance))
        })
    } else {
        #[cfg(feature = "gazebo")]
        {
            info!("Backend: Gazebo (with ground truth)");
            run_test_config(test_config, |instance| {
                aviate_backend_gz::GazeboSimBackend::connect_new(instance, 5000)
            })
        }
        #[cfg(not(feature = "gazebo"))]
        {
            info!("Backend: MAVLink-only (no backend compiled)");
            warn!("Note: Build with --features gazebo for ground truth");
            run_test_config(test_config, |instance| {
                Ok(MavlinkOnlyBackend::new(instance))
            })
        }
    };

    report_results(&result)
}

/// Run test in XIL mode (spawn simulator, FC, mavrouter)
#[allow(unused_variables)]
fn run_xil_test(
    test_config: &aviate_hal_xil::TestConfig,
    headless: bool,
    mavlink_only: bool,
) -> ExitCode {
    #[cfg(feature = "gazebo")]
    {
        use aviate_app_sitl_gazebo_x500::{generate_temp_world, WorldParams};
        use spawner::FcConfig;
        use std::path::PathBuf;

        info!("Mode: XIL (Gazebo SITL)");
        info!("Headless: {}", if headless { "yes" } else { "no (GUI)" });

        let mut spawner = Spawner::new();

        // Generate world file
        let world_params = WorldParams {
            lockstep: test_config.lockstep,
            ..WorldParams::default()
        };

        let world_path = match generate_temp_world(test_config, &world_params) {
            Ok(p) => p,
            Err(e) => {
                error!("Failed to generate world: {}", e);
                return ExitCode::FAILURE;
            }
        };
        info!("[GCS] World file: {}", world_path.display());

        // Launch Gazebo
        if let Err(e) = spawner.launch_gazebo(&world_path, headless) {
            error!("Failed to launch Gazebo: {}", e);
            return ExitCode::FAILURE;
        }

        // Wait for Gazebo shared memory
        info!("[GCS] Waiting for Gazebo plugin...");
        if !spawner.wait_for_gazebo(Duration::from_secs(30)) {
            error!("Timeout waiting for Gazebo plugin");
            return ExitCode::FAILURE;
        }
        info!("[GCS] Gazebo ready");

        // Spawn FC process for each vehicle
        // Use --connect mode so FC doesn't launch its own Gazebo
        for vehicle in &test_config.vehicles {
            let fc_config = FcConfig {
                binary_path: PathBuf::from("./target/debug/sitl-gazebo-x500"),
                args: vec!["--connect".to_string()],
                instance: vehicle.instance,
                headless,
            };

            if let Err(e) = spawner.spawn_fc(&fc_config) {
                error!("Failed to spawn FC for {}: {}", vehicle.id, e);
                return ExitCode::FAILURE;
            }
            info!(
                "[GCS] Spawned FC for {} (instance {})",
                vehicle.id, vehicle.instance
            );
        }

        // Wait for FC processes to initialize
        info!("[GCS] Waiting for FC(s) to initialize...");
        if !spawner.wait_for_fc_ready(Duration::from_secs(15)) {
            warn!("Warning: FC initialization timeout (continuing anyway)");
        }

        // Generate and spawn mavrouter (for multi-vehicle)
        if test_config.vehicles.len() > 1 {
            let router_params = RouterParams::default();
            match router_gen::generate_temp_router_config(test_config, &router_params) {
                Ok(router_path) => {
                    info!("[GCS] Router config: {}", router_path.display());
                    if let Err(e) = spawner.spawn_router(&router_path) {
                        warn!("Warning: Failed to spawn mavrouter: {}", e);
                    }
                }
                Err(e) => {
                    warn!("Warning: Failed to generate router config: {}", e);
                }
            }
        }

        // Run test with Gazebo backend
        info!("[GCS] Running {} vehicle(s)...", test_config.vehicles.len());

        let result = if mavlink_only {
            run_test_config(test_config, |instance| {
                Ok(MavlinkOnlyBackend::new(instance))
            })
        } else {
            run_test_config(test_config, |instance| {
                aviate_backend_gz::GazeboSimBackend::connect_new(instance, 10000)
            })
        };

        // Cleanup happens automatically when spawner drops
        report_results(&result)
    }

    #[cfg(not(feature = "gazebo"))]
    {
        eprintln!("XIL mode requires --features gazebo");
        eprintln!("Rebuild with: cargo build -p gcs-test --features gazebo");
        ExitCode::FAILURE
    }
}

/// Report test results and return exit code
fn report_results(result: &aviate_hal_xil::TestResult) -> ExitCode {
    println!();
    println!("========================================");
    println!("Test: {}", result.name);
    println!("Duration: {:.2}s", result.duration.as_secs_f32());
    println!("Result: {}", if result.passed { "PASS" } else { "FAIL" });

    for vr in &result.vehicle_results {
        println!();
        println!("  Vehicle: {}", vr.mission_name);
        println!("    Max altitude: {:.2}m", vr.max_altitude);
        println!(
            "    Phases: {}/{} passed",
            vr.phases.iter().filter(|p| p.passed).count(),
            vr.phases.len()
        );
    }
    println!("========================================");

    if result.passed {
        ExitCode::SUCCESS
    } else {
        ExitCode::FAILURE
    }
}

/// MAVLink-only backend (no ground truth)
///
/// Used when no simulator backend is available. The MissionRunner will still
/// work, but get_vehicle_state() returns None so ground truth comparison
/// is not possible.
struct MavlinkOnlyBackend {
    instance: u8,
}

impl MavlinkOnlyBackend {
    fn new(instance: u8) -> Self {
        Self { instance }
    }
}

impl SimulatorBackend for MavlinkOnlyBackend {
    fn name(&self) -> &str {
        "mavlink-only"
    }

    fn connect(&mut self, _instance: u8, _timeout_ms: u64) -> Result<(), SimulatorError> {
        Ok(())
    }

    fn is_connected(&self) -> bool {
        true
    }

    fn get_vehicle_state(&self) -> Option<aviate_hal_xil::VehicleState> {
        None
    }

    fn set_motor_speeds(&mut self, _speeds: &[f64]) -> Result<(), SimulatorError> {
        Ok(())
    }

    fn set_lockstep(&mut self, _enabled: bool) {}

    fn sim_step(&self) -> u64 {
        0
    }

    fn ack_step(&mut self, _step: u64) {}

    fn instance(&self) -> u8 {
        self.instance
    }
}

/// Run external script test in XIL mode
#[cfg(feature = "gazebo")]
fn run_script_test(
    test_config: &aviate_hal_xil::TestConfig,
    script: &std::path::Path,
    headless: bool,
) -> ExitCode {
    use aviate_app_sitl_gazebo_x500::{generate_temp_world, WorldParams};
    use spawner::{FcConfig, Spawner};
    use std::time::Duration;

    info!("Mode: Script (Gazebo SITL)");
    info!("Script: {}", script.display());

    let mut spawner = Spawner::new();

    // Generate world file
    let world_params = WorldParams {
        lockstep: test_config.lockstep,
        ..WorldParams::default()
    };

    let world_path = match generate_temp_world(test_config, &world_params) {
        Ok(p) => p,
        Err(e) => {
            error!("Failed to generate world: {}", e);
            return ExitCode::FAILURE;
        }
    };
    info!("[GCS] World file: {}", world_path.display());

    // Launch Gazebo
    if let Err(e) = spawner.launch_gazebo(&world_path, headless) {
        error!("Failed to launch Gazebo: {}", e);
        return ExitCode::FAILURE;
    }

    // Wait for Gazebo shared memory
    info!("[GCS] Waiting for Gazebo plugin...");
    if !spawner.wait_for_gazebo(Duration::from_secs(30)) {
        error!("Timeout waiting for Gazebo plugin");
        return ExitCode::FAILURE;
    }
    info!("[GCS] Gazebo ready");

    // Spawn FC process for each vehicle
    for vehicle in &test_config.vehicles {
        let fc_config = FcConfig {
            binary_path: std::path::PathBuf::from("./target/debug/sitl-gazebo-x500"),
            args: vec!["--connect".to_string()],
            instance: vehicle.instance,
            headless,
        };

        if let Err(e) = spawner.spawn_fc(&fc_config) {
            error!("Failed to spawn FC for {}: {}", vehicle.id, e);
            return ExitCode::FAILURE;
        }
        info!(
            "[GCS] Spawned FC for {} (instance {})",
            vehicle.id, vehicle.instance
        );
    }

    // Wait for FC processes to initialize
    info!("[GCS] Waiting for FC(s) to initialize...");
    if !spawner.wait_for_fc_ready(Duration::from_secs(15)) {
        warn!("Warning: FC initialization timeout (continuing anyway)");
    }

    // Generate and spawn mavrouter (for multi-vehicle)
    if test_config.vehicles.len() > 1 {
        let router_params = router_gen::RouterParams::default();
        match router_gen::generate_temp_router_config(test_config, &router_params) {
            Ok(router_path) => {
                info!("[GCS] Router config: {}", router_path.display());
                if let Err(e) = spawner.spawn_router(&router_path) {
                    warn!("Warning: Failed to spawn mavrouter: {}", e);
                }
            }
            Err(e) => {
                warn!("Warning: Failed to generate router config: {}", e);
            }
        }
    }

    // Run Script
    info!("[GCS] Running script...");
    let status = std::process::Command::new("python3")
        .arg(script)
        .status();

    match status {
        Ok(s) => {
             if s.success() {
                 info!("Script PASSED");
                 ExitCode::SUCCESS
             } else {
                 error!("Script FAILED with exit code: {:?}", s.code());
                 ExitCode::FAILURE
             }
        }
        Err(e) => {
            error!("Failed to execute script: {}", e);
            ExitCode::FAILURE
        }
    }
}

#[cfg(not(feature = "gazebo"))]
fn run_script_test(
    _test_config: &aviate_hal_xil::TestConfig,
    _script: &std::path::Path,
    _headless: bool,
) -> ExitCode {
    eprintln!("XIL mode requires --features gazebo");
    ExitCode::FAILURE
}
