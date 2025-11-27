#![forbid(unsafe_code)]

//! Aviate SITL Quadcopter Application
//!
//! Runs the aviate-core flight controller in software-in-the-loop mode,
//! communicating with external simulators (jMAVSim, Gazebo, AirSim) via UDP MAVLink.
//!
//! Usage:
//!   aviate-app-quadcopter-sitl [--mock]
//!
//! Options:
//!   --mock    Run in mock mode (no UDP, for testing)

use aviate_core::AviateKernel;
use aviate_core::control::mc::McController;
use aviate_core::control::{Command, Setpoint, CommandSource, ControlMode, ConfigMode};
use aviate_core::mixer::{QuadXMixer, ModeConfig};
use aviate_core::time::{Timestamp, TimeSource};
use aviate_core::hal::{SensorHal, ActuatorHal, SystemHal};
use aviate_core::types::Normalized;

use aviate_platform_sitl::{SitlConfig, SitlHal, UdpMavlinkHal};

fn sitl_timestamp() -> Timestamp {
    Timestamp { ticks: 0, source: TimeSource::Internal }
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let mock_mode = args.iter().any(|a| a == "--mock");

    if mock_mode {
        println!("Starting Aviate SITL Quadcopter (mock mode)...");
        run_mock();
    } else {
        println!("Starting Aviate SITL Quadcopter (UDP MAVLink mode)...");
        println!("Listening for HIL_SENSOR/HIL_GPS on port 14560");
        println!("Sending HIL_ACTUATOR_CONTROLS to 127.0.0.1:14561");
        run_udp();
    }
}

fn run_mock() {
    let mut hal = SitlHal::new();
    let mut kernel = create_kernel();

    loop {
        run_loop_iteration(&mut hal, &mut kernel);
        std::thread::sleep(std::time::Duration::from_millis(10));
    }
}

fn run_udp() {
    let config = SitlConfig::default();
    let mut hal = match UdpMavlinkHal::new(config) {
        Ok(hal) => hal,
        Err(e) => {
            eprintln!("Failed to create UDP HAL: {}", e);
            std::process::exit(1);
        }
    };

    let mut kernel = create_kernel();

    // Main loop at ~1kHz
    let loop_period_us = 1000; // 1ms = 1kHz
    let mut last_tick = hal.now_us();

    loop {
        let now = hal.now_us();
        let elapsed = now.saturating_sub(last_tick);

        if elapsed >= loop_period_us {
            last_tick = now;
            run_loop_iteration(&mut hal, &mut kernel);
        } else {
            // Sleep for remaining time (rough, not real-time precise)
            let remaining_us = loop_period_us - elapsed;
            if remaining_us > 100 {
                std::thread::sleep(std::time::Duration::from_micros(remaining_us - 100));
            }
        }
    }
}

fn create_kernel() -> AviateKernel<McController, QuadXMixer> {
    let controller = McController::default();
    let mixer = QuadXMixer { timestamp_source: sitl_timestamp };
    let mode_config = ModeConfig {
        mode: ConfigMode::Hover,
        groups: &[],
    };

    AviateKernel::new(controller, mixer, mode_config)
}

fn run_loop_iteration<H: SensorHal + ActuatorHal + SystemHal>(
    hal: &mut H,
    kernel: &mut AviateKernel<McController, QuadXMixer>,
) {
    // 1. Read sensors and update EKF
    if let Some(imu) = hal.read_imu() {
        // Predict at IMU rate (~1kHz)
        kernel.ekf.predict(&imu.value, 0.001);
    }

    if let Some(gnss) = hal.read_gnss() {
        kernel.ekf.update_gnss(&gnss);
    }

    if let Some(baro) = hal.read_baro() {
        kernel.ekf.update_baro(&baro);
    }

    if let Some(mag) = hal.read_mag() {
        kernel.ekf.update_mag(&mag);
    }

    // 2. Get command (stub - would come from RC/GCS)
    let cmd = Command {
        mode: ControlMode::Attitude,
        setpoint: Setpoint {
            collective_thrust: Normalized(0.5),
            ..Default::default()
        },
        config_mode_request: None,
        sensor_overrides: None,
        sequence: 0,
        source: CommandSource::Pilot,
    };

    // 3. Run init state machine
    if !kernel.is_ready() {
        // Keep running init until ready
        // In real usage, sensors need to be feeding valid data
    } else if kernel.init_state != aviate_core::InitState::Armed {
        // Auto-arm for testing (real system requires explicit arm command)
        let _ = kernel.arm();
    }

    // 4. Step kernel
    let actuator_cmd = kernel.step(&cmd);

    // 5. Write outputs
    hal.write(&actuator_cmd);

    // 6. Watchdog
    hal.kick_watchdog();
}
