//! Config-based SITL Test Runner
//!
//! Runs SITL tests defined by TOML configuration files.
//! Generates world files dynamically and executes missions.
//!
//! Usage:
//!   config-test tests/basic_flight.toml
//!   config-test tests/two_vehicle_formation.toml

use std::env;
use std::path::Path;
use std::process::ExitCode;

use aviate_app_quadcopter_sitl::{
    parse_test_config, generate_world, WorldParams,
};

fn main() -> ExitCode {
    let args: Vec<String> = env::args().collect();

    if args.len() < 2 {
        eprintln!("Usage: {} <config.toml>", args[0]);
        eprintln!("");
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
        println!("  Vehicle: {} (model: {}, instance: {})",
            vehicle.id, vehicle.model, vehicle.instance);
        println!("    Spawn: [{:.1}, {:.1}, {:.1}] heading={:.1}",
            vehicle.spawn_position[0],
            vehicle.spawn_position[1],
            vehicle.spawn_position[2],
            vehicle.spawn_heading);
        println!("    Mission: {} ({} phases)",
            vehicle.mission.name,
            vehicle.mission.phases.len());
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
    use std::thread;
    use aviate_app_quadcopter_sitl::{
        generate_temp_world, MissionRunner,
    };

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

    // Note: In a full implementation, we would:
    // 1. Launch Gazebo with the generated world
    // 2. Wait for shared memory to be ready
    // 3. Run missions for each vehicle (in parallel threads)
    // 4. Collect and verify results
    // 5. Check global verification criteria

    // For now, we just demonstrate running missions if Gazebo is already running
    println!("Attempting to connect to vehicles...");
    println!();

    let mut all_passed = true;
    let mut handles = Vec::new();

    // Spawn a thread for each vehicle
    for vehicle in &config.vehicles {
        let vehicle_id = vehicle.id.clone();
        let instance = vehicle.instance;
        let mission = vehicle.mission.clone();

        let handle = thread::spawn(move || {
            match MissionRunner::for_instance(instance, &vehicle_id) {
                Ok(mut runner) => {
                    let result = runner.run(&mission);
                    (vehicle_id, result.passed, Some(result))
                }
                Err(e) => {
                    eprintln!("[{}:{}] Failed to connect: {}", vehicle_id, instance, e);
                    (vehicle_id, false, None)
                }
            }
        });

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

    println!("=== Test Result ===");
    if all_passed {
        println!("PASSED: {}", config.name);
        ExitCode::from(0)
    } else {
        println!("FAILED: {}", config.name);
        ExitCode::from(1)
    }
}
