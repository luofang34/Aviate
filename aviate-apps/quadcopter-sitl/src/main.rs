#![forbid(unsafe_code)]

//! Aviate SITL Quadcopter Application
//!
//! Runs the aviate-core flight controller in software-in-the-loop mode,
//! communicating with external simulators (jMAVSim, Gazebo, AirSim) via UDP MAVLink.
//!
//! This application uses the X500 SITL board configuration from aviate-boards.
//!
//! Usage:
//!   aviate-app-quadcopter-sitl [--mock]
//!
//! Options:
//!   --mock    Run in mock mode (no UDP, for testing)

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
use aviate_hal_xil::SitlHal;

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let mock_mode = args.iter().any(|a| a == "--mock");

    info!(
        "Board: {} (airframe: {})",
        X500SitlBoard::board_id(),
        X500SitlBoard::airframe_id()
    );

    if mock_mode {
        info!("Starting Aviate SITL Quadcopter (mock mode)");
        run_mock();
    } else {
        info!("Starting Aviate SITL Quadcopter (UDP MAVLink mode)");
        info!("Listening for HIL_SENSOR/HIL_GPS on port 14560");
        info!("Sending HIL_ACTUATOR_CONTROLS to 127.0.0.1:14561");
        run_udp();
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

fn run_udp() {
    // Create the X500 SITL board with retry logic
    let mut board = match X500SitlBoard::new_with_retry(5, 1000) {
        Ok(b) => b,
        Err(e) => {
            warn!("Failed to initialize board: {}", e);
            warn!("Is another autopilot running?");
            std::process::exit(1);
        }
    };

    info!("Board initialized, entering main loop");

    // Run the board's main control loop
    board.run();
}
