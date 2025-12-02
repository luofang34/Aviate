#![forbid(unsafe_code)]

//! Aviate SITL Quadcopter Application
//!
//! Runs the aviate-core flight controller in software-in-the-loop mode,
//! with direct integration to Gazebo via shared memory FFI.
//!
//! ## Architecture
//!
//! ```text
//! Gazebo (gz-sim)
//!    ↓ shared memory (AviateGzPlugin)
//! GzPluginBridge (Rust)
//!    ↓ SimSensorPacket
//! SitlIO (simulator-neutral middleware)
//!    ↓
//! Aviate Kernel (control loop)
//!    ↓ SimActuatorCmd
//! GzPluginBridge → Gazebo motors
//! ```
//!
//! Usage:
//!   aviate-app-quadcopter-sitl [--mock] [--instance <N>]
//!
//! Options:
//!   --mock           Run in mock mode (no Gazebo, for testing)
//!   --instance <N>   Instance ID for multi-vehicle (default: 0)
//!
//! Environment:
//!   AVIATE_INSTANCE  Instance ID for multi-vehicle (default: 0)

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

use aviate_board_sitl_x500::X500SitlBoard;
use aviate_core::hal::SystemHal;
use aviate_hal_xil::{SitlConfig, SitlHal};

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
    let args: Vec<String> = std::env::args().collect();
    let mock_mode = args.iter().any(|a| a == "--mock");
    let instance = get_instance();
    let config = SitlConfig::for_instance(instance);

    info!(
        "Board: {} (airframe: {}) instance: {}",
        X500SitlBoard::board_id(),
        X500SitlBoard::airframe_id(),
        instance
    );

    if mock_mode {
        info!("Starting Aviate SITL Quadcopter (mock mode)");
        run_mock();
    } else {
        info!("Starting Aviate SITL Quadcopter (Gazebo mode)");
        info!("Sensor port: {}", config.sensor_port());
        run_gazebo(config, instance);
    }
}

fn run_mock() {
    // Mock mode uses the basic SitlHal without Gazebo
    // This is useful for unit testing without simulator
    let mut hal = SitlHal::new();

    loop {
        hal.kick_watchdog();
        std::thread::sleep(std::time::Duration::from_millis(10));
    }
}

#[cfg(feature = "gz-plugin")]
fn run_gazebo(config: SitlConfig, instance: u8) {
    use aviate_backend_gz::{enu_to_ned_f32, enu_vel_to_ned_f32, GzPluginBridge};
    use aviate_hal_xil::{
        SimBaroData, SimGnssData, SimGnssFix, SimImuData, SimMagData, SimSensorPacket,
    };

    // Create the board
    let mut board = match X500SitlBoard::with_config_retry(config, 5, 1000) {
        Ok(b) => b,
        Err(e) => {
            warn!("Failed to initialize board: {}", e);
            std::process::exit(1);
        }
    };

    // Connect to Gazebo via shared memory
    info!("Connecting to Gazebo (AviateGzPlugin)...");
    let plugin = match GzPluginBridge::connect_instance_with_retry(instance, 20, 500) {
        Ok(p) => {
            info!("Connected to Gazebo");
            p
        }
        Err(e) => {
            warn!("Failed to connect to Gazebo: {}", e);
            warn!("Make sure:");
            warn!("  1. Gazebo is running with AviateGzPlugin loaded");
            warn!("  2. The plugin was built: cd plugin/build && cmake .. && make");
            warn!("  3. GZ_SIM_SYSTEM_PLUGIN_PATH includes the plugin directory");
            std::process::exit(1);
        }
    };

    info!("Board initialized, entering main loop (1kHz)");

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
                // Convert ENU to NED
                let ned_pos = enu_to_ned_f32(state.pos);
                let ned_vel = enu_vel_to_ned_f32(state.vel);

                // Build sensor packet
                // IMU: Use angular velocity as gyro, simulated accelerometer
                let imu = SimImuData {
                    accel: [0.0, 0.0, -9.81], // Gravity in NED body frame (hovering)
                    gyro: [
                        state.ang_vel[0] as f32,  // Roll rate
                        -state.ang_vel[1] as f32, // Pitch rate (ENU to NED)
                        -state.ang_vel[2] as f32, // Yaw rate (ENU to NED)
                    ],
                    temperature: Some(25.0),
                };

                // Barometer: Derive from altitude
                let baro = SimBaroData {
                    pressure_pa: 101325.0 + ned_pos[2] * 12.0, // ~12 Pa per meter
                    temperature_c: 25.0,
                };

                // Magnetometer: Simple earth field model
                let mag = SimMagData {
                    field_ut: [20.0, 0.0, 40.0], // Approximate NED field
                };

                // GNSS: Convert position to lat/lon
                let gnss = SimGnssData {
                    lat_deg: ned_pos[0] as f64 / 111000.0, // ~111km per degree
                    lon_deg: ned_pos[1] as f64 / 111000.0,
                    alt_m: -ned_pos[2], // NED z is down
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

                // Feed sensor data to transport
                board.transport_mut().feed_sensor_packet(&packet);
                sensor_count += 1;
            }

            // 2. Run control loop
            board.step();

            // 3. Get actuator commands and send to Gazebo
            if let Some(cmd) = board.transport_mut().take_actuator_cmd() {
                // Convert normalized [0,1] to motor velocity (rad/s)
                const MAX_MOTOR_RADS: f64 = 1000.0;
                let velocities: Vec<f64> = cmd
                    .outputs
                    .iter()
                    .take(cmd.count as usize)
                    .map(|&v| (v as f64) * MAX_MOTOR_RADS)
                    .collect();

                if let Err(_e) = plugin.set_motor_speeds(&velocities) {
                    // Motor command failed, plugin may have disconnected
                }
                motor_count += 1;
            }

            // 4. Print stats every second
            if now.saturating_sub(stats_tick) >= 1_000_000 {
                stats_tick = now;
                info!(
                    "sensors={}, motors={}, armed={}",
                    sensor_count,
                    motor_count,
                    board.is_armed()
                );
            }
        } else {
            // Sleep to not busy-wait
            let remaining_us = loop_period_us - elapsed;
            if remaining_us > 100 {
                std::thread::sleep(std::time::Duration::from_micros(remaining_us - 100));
            }
        }
    }
}

#[cfg(not(feature = "gz-plugin"))]
fn run_gazebo(config: SitlConfig, _instance: u8) {
    // Fallback: Run without Gazebo integration
    // This just runs the board with MAVLink command reception
    warn!("Running without Gazebo integration (gz-plugin feature not enabled)");
    warn!("Sensor data must come from an external source");

    let mut board = match X500SitlBoard::with_config_retry(config, 5, 1000) {
        Ok(b) => b,
        Err(e) => {
            warn!("Failed to initialize board: {}", e);
            std::process::exit(1);
        }
    };

    info!("Board initialized, entering main loop");
    board.run();
}
