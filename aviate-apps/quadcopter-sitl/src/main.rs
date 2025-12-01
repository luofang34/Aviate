#![forbid(unsafe_code)]

//! Aviate SITL Quadcopter Application
//!
//! Runs the aviate-core flight controller in software-in-the-loop mode,
//! communicating with external simulators (jMAVSim, Gazebo, AirSim) via UDP MAVLink.
//!
//! This application uses the X500 SITL board configuration from aviate-boards.
//!
//! Usage:
//!   aviate-app-quadcopter-sitl [--mock] [--instance <N>]
//!
//! Options:
//!   --mock           Run in mock mode (no UDP, for testing)
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
        info!("Starting Aviate SITL Quadcopter (UDP MAVLink mode)");
        info!(
            "Listening for HIL_SENSOR/HIL_GPS on port {}",
            config.sensor_port()
        );
        info!("Sending HIL_ACTUATOR_CONTROLS to {}", config.simulator_addr());
        run_udp(config);
    }
}

fn run_mock() {
    // Mock mode uses the basic SitlHal without UDP
    // This is useful for unit testing without network
    let mut hal = SitlHal::new();

    // For mock mode, we just run a simple loop
    // The board encapsulates the full control loop
    loop {
        // In mock mode, no actual sensors, just tick the HAL
        hal.kick_watchdog();
        std::thread::sleep(std::time::Duration::from_millis(10));
    }
}

fn run_udp(config: SitlConfig) {
    // Create the X500 SITL board with retry logic and instance config
    let mut board = match X500SitlBoard::with_config_retry(config, 5, 1000) {
        Ok(b) => b,
        Err(e) => {
            warn!("Failed to initialize board: {}", e);
            warn!("Is another autopilot running on this port?");
            std::process::exit(1);
        }
    };

    info!("Board initialized, entering main loop");

    // Run the board's main control loop
    board.run();
}
