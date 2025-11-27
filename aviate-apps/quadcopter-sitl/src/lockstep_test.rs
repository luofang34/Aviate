//! Lockstep Synchronization Test
//!
//! This test verifies that lockstep mode works correctly:
//! 1. Connects to Gazebo via shared memory
//! 2. Enables lockstep mode
//! 3. Reads physics state, sends motor commands, acknowledges steps
//! 4. Verifies simulation advances in sync with our acknowledgments
//!
//! Usage:
//!   # Start Gazebo with lockstep world:
//!   HEADLESS=1 gz sim -s -r aviate-apps/quadcopter-sitl/worlds/x500_quadcopter_lockstep.sdf
//!
//!   # Run this test:
//!   ./target/debug/lockstep-test


#[cfg(feature = "gz-plugin")]
use aviate_platform_sitl::gz_plugin::{GzPluginBridge, enu_to_ned_f32};

fn main() {
    println!("Lockstep Synchronization Test");
    println!("==============================");
    println!();

    #[cfg(not(feature = "gz-plugin"))]
    {
        eprintln!("Error: gz-plugin feature not enabled");
        eprintln!("Build with: cargo build --features gz-plugin -p aviate-app-quadcopter-sitl");
        std::process::exit(1);
    }

    #[cfg(feature = "gz-plugin")]
    {
        run_lockstep_test();
    }
}

#[cfg(feature = "gz-plugin")]
fn run_lockstep_test() {
    // Connect to Gazebo plugin
    println!("[1] Connecting to AviateGzPlugin...");
    let bridge = match GzPluginBridge::connect_with_retry(20, 500) {
        Ok(b) => {
            println!("    Connected via shared memory");
            b
        }
        Err(e) => {
            eprintln!("    Failed to connect: {:?}", e);
            eprintln!();
            eprintln!("Make sure Gazebo is running with the lockstep world:");
            eprintln!("  HEADLESS=1 gz sim -s -r aviate-apps/quadcopter-sitl/worlds/x500_quadcopter_lockstep.sdf");
            std::process::exit(1);
        }
    };

    // Enable lockstep mode from Rust side
    println!("[2] Enabling lockstep mode...");
    bridge.set_lockstep(true);
    println!("    Lockstep enabled");

    // Wait a moment for initial state
    std::thread::sleep(Duration::from_millis(100));

    // Get initial step count
    let initial_step = bridge.sim_step();
    println!("    Initial sim_step: {}", initial_step);

    // Test 1: Verify simulation is blocked without acknowledgment
    println!();
    println!("[3] Testing lockstep blocking (no ack for 2s)...");
    let step_before_wait = bridge.sim_step();
    std::thread::sleep(Duration::from_secs(2));
    let step_after_wait = bridge.sim_step();

    // In lockstep mode with no ack, sim should advance slowly (timeout releases)
    let steps_during_block = step_after_wait - step_before_wait;
    println!("    Steps advanced during 2s block: {}", steps_during_block);

    if steps_during_block < 100 {
        println!("    OK: Simulation was blocked/throttled (expected with lockstep)");
    } else {
        println!("    WARNING: Simulation advanced {} steps - lockstep may not be active", steps_during_block);
    }

    // Test 2: Run with proper acknowledgment
    println!();
    println!("[4] Running with proper step acknowledgment...");
    let mut last_step = bridge.sim_step();
    let mut acked_steps = 0u64;
    let mut max_altitude = 0.0f32;
    let start = Instant::now();
    let test_duration = Duration::from_secs(3);

    // Set motor commands for takeoff
    let motor_speeds = [800.0, 800.0, 800.0, 800.0];

    while start.elapsed() < test_duration {
        // Check for new step
        let current_step = bridge.sim_step();
        if current_step > last_step {
            // Read state
            if let Some(state) = bridge.get_model_state() {
                let ned_pos = enu_to_ned_f32(state.pos);
                let altitude = -ned_pos[2]; // NED z is negative up
                if altitude > max_altitude {
                    max_altitude = altitude;
                }
            }

            // Send motor command
            let _ = bridge.set_motor_speeds(&motor_speeds);

            // Acknowledge the step
            bridge.ack_step(current_step);
            acked_steps += 1;
            last_step = current_step;
        }

        // Small sleep to avoid busy-waiting
        std::thread::sleep(Duration::from_micros(100));
    }

    let final_step = bridge.sim_step();
    let total_steps = final_step - initial_step;

    println!("    Acknowledged steps: {}", acked_steps);
    println!("    Total sim steps: {}", total_steps);
    println!("    Max altitude: {:.2} m", max_altitude);

    // Verify lockstep worked: acked_steps should be close to total_steps
    let sync_ratio = if total_steps > 0 {
        acked_steps as f64 / total_steps as f64
    } else {
        0.0
    };

    println!();
    println!("[5] Results:");
    println!("    Sync ratio: {:.1}% (acked/total)", sync_ratio * 100.0);

    // Disable lockstep
    bridge.set_lockstep(false);
    println!("    Lockstep disabled");

    // Evaluate test results
    println!();
    if sync_ratio > 0.9 && max_altitude > 0.5 {
        println!("PASSED: Lockstep synchronization working correctly");
        println!("  - Simulation synchronized with flight controller");
        println!("  - Aircraft achieved altitude: {:.2} m", max_altitude);
        std::process::exit(0);
    } else if max_altitude < 0.5 {
        println!("FAILED: Aircraft did not take off (altitude: {:.2} m)", max_altitude);
        println!("  - Check motor commands are being applied");
        std::process::exit(1);
    } else {
        println!("WARNING: Low sync ratio ({:.1}%)", sync_ratio * 100.0);
        println!("  - Lockstep may not be fully effective");
        std::process::exit(1);
    }
}
