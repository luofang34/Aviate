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

/// Simple logging macros
macro_rules! info {
    ($($arg:tt)*) => {
        eprintln!("[INFO] {}", format_args!($($arg)*));
    };
}

macro_rules! debug {
    ($($arg:tt)*) => {
        if std::env::var("RUST_LOG").map(|v| v.contains("debug")).unwrap_or(false) {
            eprintln!("[DEBUG] {}", format_args!($($arg)*));
        }
    };
}

macro_rules! warn {
    ($($arg:tt)*) => {
        eprintln!("[WARN] {}", format_args!($($arg)*));
    };
}

use aviate_core::control::mc::McController;
use aviate_core::control::{Command, CommandSource, ConfigMode, ControlMode, Setpoint};
use aviate_core::hal::{ActuatorHal, CommandHal, SensorHal, SystemCommand, SystemHal};
use aviate_core::mixer::{ModeConfig, QuadXMixer};
use aviate_core::sensor::{BaroData, GnssData, ImuData, MagData, SensorReading, SensorSet};
use aviate_core::time::{TimeDelta, TimeSource, Timestamp};
use aviate_core::types::Normalized;
use aviate_core::AviateKernel;

use aviate_platform_xil::{SitlConfig, SitlHal, UdpMavlinkHal};

fn sitl_timestamp() -> Timestamp {
    Timestamp {
        ticks: 0,
        source: TimeSource::Internal,
    }
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let mock_mode = args.iter().any(|a| a == "--mock");

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
    let mut hal = SitlHal::new();
    let mut kernel = create_kernel();
    let mut last_cmd = default_command();
    let mut last_imu_time = None;
    let mut sensor_cache = SensorCache::new();

    loop {
        run_loop_iteration(
            &mut hal,
            &mut kernel,
            &mut last_cmd,
            &mut last_imu_time,
            &mut sensor_cache,
        );
        std::thread::sleep(std::time::Duration::from_millis(10));
    }
}

fn run_udp() {
    // Gazebo should be launched externally via scripts/run_sitl.sh
    // This application expects HIL_SENSOR/HIL_GPS messages on port 14560
    debug!("Expecting Gazebo to be running with HIL bridge");

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
                warn!("Failed to bind UDP port: {}. Retrying in 1s...", e);
                std::thread::sleep(std::time::Duration::from_secs(1));
            }
        }
    }

    let mut hal =
        hal.expect("Failed to initialize UDP HAL after retries. Is another autopilot running?");

    let mut kernel = create_kernel();
    let mut last_cmd = default_command();
    let mut last_imu_time = None;
    let mut sensor_cache = SensorCache::new();

    // Main loop at ~1kHz
    let loop_period_us = 1000; // 1ms = 1kHz
    let mut last_tick = hal.now_us();

    loop {
        let now = hal.now_us();
        let elapsed = now.saturating_sub(last_tick);

        if elapsed >= loop_period_us {
            last_tick = now;
            run_loop_iteration(
                &mut hal,
                &mut kernel,
                &mut last_cmd,
                &mut last_imu_time,
                &mut sensor_cache,
            );
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
    let mixer = QuadXMixer {
        timestamp_source: sitl_timestamp,
    };
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

/// Cached sensor readings for init_step
struct SensorCache {
    imu: Option<SensorReading<ImuData>>,
    gnss: Option<SensorReading<GnssData>>,
    baro: Option<SensorReading<BaroData>>,
    mag: Option<SensorReading<MagData>>,
}

impl SensorCache {
    fn new() -> Self {
        Self {
            imu: None,
            gnss: None,
            baro: None,
            mag: None,
        }
    }

    fn to_sensor_set(&self) -> SensorSet {
        SensorSet {
            imus: [
                self.imu.unwrap_or_default(),
                SensorReading::default(),
                SensorReading::default(),
            ],
            gnss: [self.gnss.unwrap_or_default(), SensorReading::default()],
            mags: [self.mag.unwrap_or_default(), SensorReading::default()],
            baros: [self.baro.unwrap_or_default(), SensorReading::default()],
            airspeeds: [SensorReading::default(), SensorReading::default()],
            geometry: None,
        }
    }
}

fn run_loop_iteration<H: SensorHal + ActuatorHal + SystemHal + CommandHal>(
    hal: &mut H,
    kernel: &mut AviateKernel<McController, QuadXMixer>,
    last_cmd: &mut Command,
    last_imu_time: &mut Option<u64>,
    sensor_cache: &mut SensorCache,
) {
    // 1. Read sensors and update EKF
    let mut current_dt = 0.001; // Default to 1ms if no IMU yet
    let mut current_delta_us = 1000;

    if let Some(imu) = hal.read_imu() {
        let current_time = imu.timestamp.ticks;
        let delta_us_val = if let Some(last) = *last_imu_time {
            current_time.saturating_sub(last)
        } else {
            1000 // Default 1ms for first sample
        };
        current_dt = (delta_us_val as f32) * 1e-6;
        current_delta_us = delta_us_val;
        *last_imu_time = Some(current_time);

        // Sanity check dt
        current_dt = current_dt.clamp(0.0001, 0.1);

        // EKF predict is now handled inside kernel.step
        sensor_cache.imu = Some(imu);
    }

    if let Some(gnss) = hal.read_gnss() {
        // EKF update is now handled inside kernel.step
        sensor_cache.gnss = Some(gnss);
    }

    if let Some(baro) = hal.read_baro() {
        // EKF update is now handled inside kernel.step
        sensor_cache.baro = Some(baro);
    }

    if let Some(mag) = hal.read_mag() {
        // EKF update is now handled inside kernel.step
        sensor_cache.mag = Some(mag);
    }

    // Construct TimeDelta
    let time_delta = TimeDelta {
        dt_sec: aviate_core::types::Seconds(current_dt),
        tick_delta: current_delta_us,
    };

    // 2. Receive Commands (GCS/RC)
    if let Some(sys_cmd) = hal.recv_command() {
        match sys_cmd {
            SystemCommand::FlightControl(cmd) => {
                debug!(
                    "FlightControl: thrust={:.2}",
                    cmd.setpoint.collective_thrust.0
                );
                // Update throttle check for pre-arm
                kernel
                    .checks
                    .pre_arm
                    .update_throttle(cmd.setpoint.collective_thrust.0 < 0.1);
                *last_cmd = cmd;
            }
            SystemCommand::Arm => {
                info!("Arm command received (state={:?})", kernel.init_state);
                if let Err(e) = kernel.arm() {
                    warn!("Arming failed: {:?}", e);
                } else {
                    info!("Armed successfully");
                }
                hal.arm(); // Update HAL state too
            }
            SystemCommand::Disarm => {
                info!("Disarm command received");
                kernel.disarm();
                hal.disarm();
            }
        }
    }

    // 3. Run init state machine with actual sensor data
    let sensors = sensor_cache.to_sensor_set();
    if !kernel.is_ready() {
        let ts = hal.now();
        kernel.init_step(&sensors, ts);
    }

    // 4. Step kernel (command_age_ms=0 means command is fresh)
    let actuator_cmd = kernel.step(time_delta, last_cmd, &sensors, 0);

    // Debug output - print actuator commands when thrust command is non-zero
    if last_cmd.setpoint.collective_thrust.0 > 0.1 {
        let sum: f32 = actuator_cmd.outputs.iter().map(|o| o.0).sum();
        debug!(
            "ActuatorCmd: sum={:.3}, thrust_in={:.2}, state={:?}",
            sum, last_cmd.setpoint.collective_thrust.0, kernel.init_state
        );
    }

    // 5. Write outputs
    hal.write(&actuator_cmd);

    // 6. Watchdog
    hal.kick_watchdog();
}
