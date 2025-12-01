//! Gazebo-MAVLink Bridge Binary
//!
//! This binary bridges Gazebo physics data to MAVLink HIL protocol.
//! Run this alongside Gazebo and the Aviate SITL application.
//!
//! Requires the `gz-plugin` feature and AviateGzPlugin loaded in Gazebo.
//!
//! Usage:
//!   gz-bridge [--instance <N>]
//!
//! Environment:
//!   AVIATE_INSTANCE - Instance ID for multi-vehicle (default: 0)

use aviate_backend_gz::{GzBridge, GzBridgeConfig};

/// Get instance ID from AVIATE_INSTANCE env var or --instance arg
fn get_instance() -> u8 {
    // Check environment variable first
    if let Ok(val) = std::env::var("AVIATE_INSTANCE") {
        if let Ok(instance) = val.parse::<u8>() {
            return instance;
        }
    }

    // Check command line args
    let args: Vec<String> = std::env::args().collect();
    for i in 0..args.len() {
        if args[i] == "--instance" && i + 1 < args.len() {
            if let Ok(instance) = args[i + 1].parse::<u8>() {
                return instance;
            }
        }
    }

    0 // Default to instance 0
}

fn main() {
    println!("Gazebo-MAVLink Bridge for Aviate SITL");
    println!("=====================================");

    // Get instance from environment or args
    let instance = get_instance();
    let config = GzBridgeConfig::for_instance(instance);
    println!("Configuration:");
    println!("  Model name:  {}", config.model_name);
    println!("  Motor topic: {}", config.motor_topic);
    println!("  Aviate port: {}", config.aviate_port);
    println!("  Test port:   {}", config.test_port);

    let mut bridge = match GzBridge::new(config) {
        Ok(b) => b,
        Err(e) => {
            eprintln!("Failed to create bridge: {}", e);
            std::process::exit(1);
        }
    };

    // Connect to the AviateGzPlugin via shared memory
    println!("Connecting to AviateGzPlugin (10s timeout)...");
    if let Err(e) = bridge.connect(10_000) {
        eprintln!("Failed to connect to plugin: {}", e);
        eprintln!("Make sure:");
        eprintln!("  1. Gazebo is running with AviateGzPlugin loaded");
        eprintln!("  2. The plugin was built: cd aviate_gz_plugin/build && cmake .. && make");
        eprintln!("  3. GZ_SIM_SYSTEM_PLUGIN_PATH includes the plugin directory");
        std::process::exit(1);
    }

    println!("Bridge running at 250 Hz. Press Ctrl+C to stop.");

    // Main loop at 250 Hz
    let period = std::time::Duration::from_micros(4000); // 4ms = 250Hz

    loop {
        let start = std::time::Instant::now();

        bridge.step();

        let elapsed = start.elapsed();
        if elapsed < period {
            std::thread::sleep(period - elapsed);
        }
    }
}
