//! Gazebo-MAVLink Bridge Binary
//!
//! This binary bridges Gazebo sensor topics to MAVLink HIL protocol.
//! Run this alongside Gazebo and the Aviate SITL application.
//!
//! Usage:
//!   gz-bridge [--imu-topic /X3/imu] [--odom-topic /model/X3/odometry]

use aviate_platform_sitl::{GzBridge, GzBridgeConfig};

fn main() {
    println!("Gazebo-MAVLink Bridge for Aviate SITL");
    println!("=====================================");

    let config = GzBridgeConfig::default();
    println!("Configuration:");
    println!("  IMU topic:   {}", config.imu_topic);
    println!("  Odom topic:  {}", config.odom_topic);
    println!("  Motor topic: {}", config.motor_topic);
    println!("  Aviate port: {}", config.aviate_port);

    let mut bridge = match GzBridge::new(config) {
        Ok(b) => b,
        Err(e) => {
            eprintln!("Failed to create bridge: {}", e);
            std::process::exit(1);
        }
    };

    println!("Subscribing to Gazebo topics...");
    if let Err(e) = bridge.subscribe() {
        eprintln!("Failed to subscribe: {}", e);
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
