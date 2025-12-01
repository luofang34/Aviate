//! Config-based SITL Test Runner
//!
//! Runs SITL tests defined by TOML configuration files.
//! Generates world files dynamically and executes missions.
//!
//! Usage:
//!   config-test tests/basic_flight.toml
//!   config-test tests/two_vehicle_formation.toml
//!
//! Environment variables:
//!   HEADLESS=1  - Run without GUI (uses EGL rendering)

use std::env;
use std::path::Path;
use std::process::ExitCode;

use aviate_app_quadcopter_sitl::{generate_world, parse_test_config, WorldParams};

/// Environment variable to run in headless mode
#[cfg(feature = "gz-plugin")]
const HEADLESS_ENV: &str = "HEADLESS";

fn main() -> ExitCode {
    let args: Vec<String> = env::args().collect();

    if args.len() < 2 {
        eprintln!("Usage: {} <config.toml>", args[0]);
        eprintln!();
        eprintln!("Examples:");
        eprintln!("  {} tests/basic_flight.toml", args[0]);
        eprintln!("  {} tests/two_vehicle_formation.toml", args[0]);
        return ExitCode::from(1);
    }

    let config_path = Path::new(&args[1]);

    println!("=== Aviate Config-Based SITL Test ===");
    println!("Config: {}", config_path.display());
    println!();

    // Parse configuration
    let config = match parse_test_config(config_path) {
        Ok(c) => c,
        Err(e) => {
            eprintln!("Failed to parse config: {}", e);
            return ExitCode::from(1);
        }
    };

    println!("Test: {}", config.name);
    println!("Description: {}", config.description);
    println!("Lockstep: {}", config.lockstep);
    println!("Vehicles: {}", config.vehicles.len());
    println!();

    // Print vehicle details
    for vehicle in &config.vehicles {
        println!(
            "  Vehicle: {} (model: {}, instance: {})",
            vehicle.id, vehicle.model, vehicle.instance
        );
        println!(
            "    Spawn: [{:.1}, {:.1}, {:.1}] heading={:.1}",
            vehicle.spawn_position[0],
            vehicle.spawn_position[1],
            vehicle.spawn_position[2],
            vehicle.spawn_heading
        );
        println!(
            "    Mission: {} ({} phases)",
            vehicle.mission.name,
            vehicle.mission.phases.len()
        );
    }
    println!();

    // Generate world parameters
    let params = WorldParams {
        lockstep: config.lockstep,
        ..WorldParams::default()
    };

    // Generate world SDF
    println!("=== Generated World SDF ===");
    let sdf = generate_world(&config, &params);

    // Print summary of generated world
    let line_count = sdf.lines().count();
    println!("Generated {} lines of SDF", line_count);

    // Print first 30 lines for preview
    println!();
    println!("--- Preview (first 30 lines) ---");
    for (i, line) in sdf.lines().take(30).enumerate() {
        println!("{:3}: {}", i + 1, line);
    }
    println!("...");
    println!();

    // Check for required features
    #[cfg(feature = "gz-plugin")]
    {
        run_test_with_gazebo(&config, &params)
    }

    #[cfg(not(feature = "gz-plugin"))]
    {
        println!("=== Dry Run (gz-plugin feature not enabled) ===");
        println!("To run actual tests, build with: cargo build --features gz-plugin");
        println!();
        println!("Test configuration validated successfully.");
        ExitCode::from(0)
    }
}

#[cfg(feature = "gz-plugin")]
fn run_test_with_gazebo(
    config: &aviate_app_quadcopter_sitl::TestConfig,
    params: &WorldParams,
) -> ExitCode {
    use aviate_app_quadcopter_sitl::{generate_temp_world, MissionRunner};
    use std::thread;
    use std::time::Duration;

    println!("=== Running Test with Gazebo ===");

    // Generate world file to temp location
    let world_path = match generate_temp_world(config, params) {
        Ok(p) => p,
        Err(e) => {
            eprintln!("Failed to generate world file: {}", e);
            return ExitCode::from(1);
        }
    };

    println!("World file: {}", world_path.display());
    println!();

    // Clean up any existing Gazebo processes
    cleanup_gazebo();

    // Set up Gazebo environment
    let (gz_resource_path, gz_plugin_path) = setup_gz_environment();

    // Launch Gazebo
    let headless = env::var(HEADLESS_ENV).map(|v| v == "1").unwrap_or(false);
    let gz_child = match launch_gazebo(&world_path, headless, &gz_resource_path, &gz_plugin_path) {
        Ok(child) => child,
        Err(e) => {
            eprintln!("Failed to launch Gazebo: {}", e);
            return ExitCode::from(1);
        }
    };

    // Wait for shared memory to be ready
    if !wait_for_shm(Duration::from_secs(15)) {
        eprintln!("Timeout waiting for Gazebo plugin to initialize shared memory");
        drop(gz_child);
        cleanup_gazebo();
        return ExitCode::from(1);
    }

    println!("Gazebo ready.");

    // Launch gz-bridge (Gazebo-MAVLink bridge) for each vehicle
    let bridge_children = launch_gz_bridges(config, &gz_plugin_path);

    // Give bridge time to connect to shared memory
    thread::sleep(Duration::from_millis(500));

    // Launch FC (Flight Controller) for each vehicle
    let fc_children = launch_flight_controllers(config, &gz_plugin_path);

    // Give FC time to initialize and connect to bridge
    thread::sleep(Duration::from_secs(2));

    println!("Connecting to vehicles...");
    println!();

    let mut all_passed = true;
    let mut handles = Vec::new();

    // Spawn a thread for each vehicle
    for vehicle in &config.vehicles {
        let vehicle_id = vehicle.id.clone();
        let instance = vehicle.instance;
        let mission = vehicle.mission.clone();

        let handle =
            thread::spawn(
                move || match MissionRunner::for_instance(instance, &vehicle_id) {
                    Ok(mut runner) => {
                        let result = runner.run(&mission);
                        (vehicle_id, result.passed, Some(result))
                    }
                    Err(e) => {
                        eprintln!("[{}:{}] Failed to connect: {}", vehicle_id, instance, e);
                        (vehicle_id, false, None)
                    }
                },
            );

        handles.push(handle);
    }

    // Wait for all vehicles to complete
    println!("Waiting for vehicles to complete...");
    println!();

    for handle in handles {
        let (vehicle_id, passed, result) = handle.join().unwrap();

        if !passed {
            all_passed = false;
        }

        if let Some(result) = result {
            println!("=== Vehicle {} Results ===", vehicle_id);
            println!("  Mission: {}", result.mission_name);
            println!("  Passed: {}", result.passed);
            println!("  Duration: {:.2}s", result.total_duration.as_secs_f32());
            println!("  Max Altitude: {:.2}m", result.max_altitude);
            println!();
        }
    }

    // Check global verification
    if let Some(ref verification) = config.global_verification {
        println!("=== Global Verification ===");
        if let Some(min_sep) = verification.min_separation {
            println!("  Min separation required: {}m", min_sep);
            // Note: Actual verification would need to track positions during flight
            println!("  (Verification not yet implemented for multi-vehicle)");
        }
        println!();
    }

    // Cleanup FC processes
    for mut fc in fc_children {
        let _ = fc.kill();
    }

    // Cleanup bridge processes
    for mut bridge in bridge_children {
        let _ = bridge.kill();
    }

    // Cleanup Gazebo
    drop(gz_child);
    cleanup_gazebo();

    println!("=== Test Result ===");
    if all_passed {
        println!("PASSED: {}", config.name);
        ExitCode::from(0)
    } else {
        println!("FAILED: {}", config.name);
        ExitCode::from(1)
    }
}

/// Launch gz-bridge processes for each vehicle (Gazebo-MAVLink bridge)
#[cfg(feature = "gz-plugin")]
fn launch_gz_bridges(
    config: &aviate_app_quadcopter_sitl::TestConfig,
    plugin_path: &str,
) -> Vec<std::process::Child> {
    use std::process::{Command, Stdio};

    let mut children = Vec::new();
    let aviate_dir = env::current_dir().unwrap_or_default();
    let bridge_binary = aviate_dir.join("target/debug/gz-bridge");

    // Check if bridge binary exists
    if !bridge_binary.exists() {
        println!(
            "gz-bridge binary not found at {:?}, skipping bridge launch",
            bridge_binary
        );
        println!("Build it with: cargo build -p aviate-backend-gz --features gz-plugin");
        return children;
    }

    for vehicle in &config.vehicles {
        println!(
            "Launching gz-bridge for vehicle {} (instance {})...",
            vehicle.id, vehicle.instance
        );

        let child = Command::new(&bridge_binary)
            .env("LD_LIBRARY_PATH", plugin_path)
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .spawn();

        match child {
            Ok(c) => {
                println!("  gz-bridge started (PID: {})", c.id());
                children.push(c);
            }
            Err(e) => {
                eprintln!("  Failed to launch gz-bridge: {}", e);
            }
        }
    }

    children
}

/// Launch flight controller processes for each vehicle
#[cfg(feature = "gz-plugin")]
fn launch_flight_controllers(
    config: &aviate_app_quadcopter_sitl::TestConfig,
    plugin_path: &str,
) -> Vec<std::process::Child> {
    use std::process::{Command, Stdio};

    let mut children = Vec::new();
    let aviate_dir = env::current_dir().unwrap_or_default();
    let fc_binary = aviate_dir.join("target/debug/aviate-app-quadcopter-sitl");

    // Check if FC binary exists
    if !fc_binary.exists() {
        println!("FC binary not found at {:?}, skipping FC launch", fc_binary);
        println!("Tests will run in direct mode (bypassing FC)");
        return children;
    }

    for vehicle in &config.vehicles {
        println!(
            "Launching FC for vehicle {} (instance {})...",
            vehicle.id, vehicle.instance
        );

        // Each instance needs different ports
        // Instance 0: sensor_port=14560, actuator_port=14561
        // Instance N: sensor_port=14560+N*10, actuator_port=14561+N*10
        let _sensor_port = 14560 + vehicle.instance as u16 * 10;

        let child = Command::new(&fc_binary)
            .env("LD_LIBRARY_PATH", plugin_path)
            // Future: pass instance-specific config via env or args
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .spawn();

        match child {
            Ok(c) => {
                println!("  FC started (PID: {})", c.id());
                children.push(c);
            }
            Err(e) => {
                eprintln!("  Failed to launch FC: {}", e);
            }
        }
    }

    children
}

/// Clean up any existing Gazebo, bridge, and FC processes and shared memory
#[cfg(feature = "gz-plugin")]
fn cleanup_gazebo() {
    use std::fs;
    use std::process::Command;

    // Kill Gazebo processes
    let _ = Command::new("pkill").args(["-9", "-f", "gz sim"]).status();

    // Kill gz-bridge processes
    let _ = Command::new("pkill")
        .args(["-9", "-f", "gz-bridge"])
        .status();

    // Kill FC processes
    let _ = Command::new("pkill")
        .args(["-9", "-f", "aviate-app-quadcopter-sitl"])
        .status();

    // Remove shared memory
    let _ = fs::remove_file("/dev/shm/aviate_gz_bridge");

    // Brief pause for cleanup
    std::thread::sleep(std::time::Duration::from_millis(500));
}

/// Set up Gazebo environment variables
#[cfg(feature = "gz-plugin")]
fn setup_gz_environment() -> (String, String) {
    // Find the Aviate directory (assuming we're running from repo root)
    let aviate_dir = env::current_dir().unwrap_or_default();

    // Set up model path - local models first (override), then PX4-gazebo-models
    let local_models_dir = aviate_dir.join("models");
    let px4_models_dir = aviate_dir.join("external/PX4-gazebo-models/models");

    let mut paths = Vec::new();
    if local_models_dir.exists() {
        paths.push(local_models_dir.to_string_lossy().to_string());
    }
    if px4_models_dir.exists() {
        paths.push(px4_models_dir.to_string_lossy().to_string());
    }
    let gz_resource_path = paths.join(":");

    // Set up plugin path for AviateGzPlugin
    let new_plugin_dir = aviate_dir.join("aviate-hal/xil/backends/gz/plugin/build");
    let legacy_plugin_dir = aviate_dir.join("aviate-platform/aviate_gz_plugin/build");

    let plugin_dir = if new_plugin_dir.join("libAviateGzPlugin.so").exists() {
        new_plugin_dir
    } else {
        legacy_plugin_dir
    };

    let gz_plugin_path = plugin_dir.to_string_lossy().to_string();

    (gz_resource_path, gz_plugin_path)
}

/// Launch Gazebo with the given world file
#[cfg(feature = "gz-plugin")]
fn launch_gazebo(
    world_path: &std::path::Path,
    headless: bool,
    gz_resource_path: &str,
    gz_plugin_path: &str,
) -> Result<std::process::Child, std::io::Error> {
    use std::process::{Command, Stdio};

    let mut cmd = Command::new("gz");
    cmd.arg("sim");

    if headless {
        // Server-only mode with headless rendering for sensors
        cmd.arg("-s");
        cmd.arg("-r");
        cmd.arg("--headless-rendering");
        // Clear DISPLAY to force EGL backend
        cmd.env_remove("DISPLAY");
    } else {
        cmd.arg("-r");
    }

    cmd.arg(world_path);

    // Set environment paths
    if !gz_resource_path.is_empty() {
        let existing = env::var("GZ_SIM_RESOURCE_PATH").unwrap_or_default();
        let combined = if existing.is_empty() {
            gz_resource_path.to_string()
        } else {
            format!("{}:{}", gz_resource_path, existing)
        };
        cmd.env("GZ_SIM_RESOURCE_PATH", combined);
    }

    if !gz_plugin_path.is_empty() {
        let existing = env::var("GZ_SIM_SYSTEM_PLUGIN_PATH").unwrap_or_default();
        let combined = if existing.is_empty() {
            gz_plugin_path.to_string()
        } else {
            format!("{}:{}", gz_plugin_path, existing)
        };
        cmd.env("GZ_SIM_SYSTEM_PLUGIN_PATH", combined);
    }

    // Suppress Gazebo output (use Stdio::inherit() for debugging)
    cmd.stdout(Stdio::null());
    cmd.stderr(Stdio::null());

    println!("Launching Gazebo (headless={})...", headless);
    cmd.spawn()
}

/// Wait for shared memory to be created by the plugin
#[cfg(feature = "gz-plugin")]
fn wait_for_shm(timeout: std::time::Duration) -> bool {
    use std::path::Path;
    use std::time::Instant;

    let shm_path = Path::new("/dev/shm/aviate_gz_bridge");
    let start = Instant::now();

    println!("Waiting for Gazebo plugin to initialize...");

    while start.elapsed() < timeout {
        if shm_path.exists() {
            println!("Shared memory ready.");
            return true;
        }
        std::thread::sleep(std::time::Duration::from_millis(250));
    }

    false
}
