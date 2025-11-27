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
use aviate_core::hal::{SensorHal, ActuatorHal, SystemHal, CommandHal, SystemCommand};
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
    let mut last_cmd = default_command();
    let mut last_imu_time = None;

    loop {
        run_loop_iteration(&mut hal, &mut kernel, &mut last_cmd, &mut last_imu_time);
        std::thread::sleep(std::time::Duration::from_millis(10));
    }
}

fn run_udp() {
    // Attempt to launch Gazebo if script exists
    if std::path::Path::new("./scripts/launch_gazebo.sh").exists() {
        println!("Launching Gazebo simulator via script...");
        match std::process::Command::new("./scripts/launch_gazebo.sh").status() {
            Ok(status) => {
                if !status.success() {
                    eprintln!("Warning: Simulator launch script returned error: {}", status);
                }
            }
            Err(e) => eprintln!("Failed to execute launch script: {}", e),
        }
    } else {
        println!("Launch script not found, assuming simulator is running manually.");
    }

    let config = SitlConfig::default();
    // Retry binding a few times if port is busy (race condition with pkill)
    let mut hal = None;
    for _ in 0..5 {
        match UdpMavlinkHal::new(config.clone()) {
            Ok(h) => {
                hal = Some(h);
                break;
            }
            Err(e) => {
                eprintln!("Failed to bind UDP port: {}. Retrying in 1s...", e);
                std::thread::sleep(std::time::Duration::from_secs(1));
            }
        }
    }
    
    let mut hal = hal.expect("Failed to initialize UDP HAL after retries. Is another autopilot running?");

    let mut kernel = create_kernel();
    let mut last_cmd = default_command();
    let mut last_imu_time = None;

    // Main loop at ~1kHz
    let loop_period_us = 1000; // 1ms = 1kHz
    let mut last_tick = hal.now_us();

    loop {
        let now = hal.now_us();
        let elapsed = now.saturating_sub(last_tick);

        if elapsed >= loop_period_us {
            last_tick = now;
            run_loop_iteration(&mut hal, &mut kernel, &mut last_cmd, &mut last_imu_time);
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

fn default_command() -> Command {
    Command {
        mode: ControlMode::Attitude,
        setpoint: Setpoint {
            collective_thrust: Normalized(0.0), // Idle
            ..Default::default()
        },
        config_mode_request: None,
        sensor_overrides: None,
        sequence: 0,
        source: CommandSource::Failsafe,
    }
}

fn run_loop_iteration<H: SensorHal + ActuatorHal + SystemHal + CommandHal>(
    hal: &mut H,
    kernel: &mut AviateKernel<McController, QuadXMixer>,
    last_cmd: &mut Command,
    last_imu_time: &mut Option<u64>,
) {
    // 1. Read sensors and update EKF
    if let Some(imu) = hal.read_imu() {
        let current_time = imu.timestamp.ticks;
        let dt = if let Some(last) = *last_imu_time {
            let delta_us = current_time.saturating_sub(last);
            (delta_us as f32) * 1e-6
        } else {
            0.001 // Default 1ms for first sample
        };
        *last_imu_time = Some(current_time);
        
        // Sanity check dt
        let dt = dt.clamp(0.0001, 0.1);

        kernel.ekf.predict(&imu.value, dt);
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

    // 2. Receive Commands (GCS/RC)
    if let Some(sys_cmd) = hal.recv_command() {
        match sys_cmd {
            SystemCommand::FlightControl(cmd) => {
                *last_cmd = cmd;
            }
            SystemCommand::Arm => {
                println!("Command: Arming");
                if let Err(e) = kernel.arm() {
                    println!("Arming failed: {:?}", e);
                }
                hal.arm(); // Update HAL state too
            }
            SystemCommand::Disarm => {
                println!("Command: Disarming");
                kernel.disarm();
                hal.disarm();
            }
        }
    }

    // 3. Run init state machine
    if !kernel.is_ready() {
        // Keep running init until ready
        // In real usage, sensors need to be feeding valid data
        // Mock sensors might be needed if not connected to simulator
    } 
    // Removed auto-arm logic to allow GCS control

    // 4. Step kernel
    let actuator_cmd = kernel.step(last_cmd);

    // 5. Write outputs
    hal.write(&actuator_cmd);

    // 6. Watchdog
    hal.kick_watchdog();
}
