#![forbid(unsafe_code)]

use aviate_core::AviateKernel;
use aviate_core::control::mc::McController;
use aviate_core::control::{Command, Setpoint, CommandSource, ControlMode, ConfigMode};
use aviate_core::mixer::{QuadXMixer, ModeConfig};
use aviate_core::time::{Timestamp, TimeSource};
use aviate_core::hal::{SensorHal, ActuatorHal, SystemHal};
use aviate_core::types::Normalized;
use aviate_platform_sitl::SitlHal;

fn sitl_timestamp() -> Timestamp {
    Timestamp { ticks: 0, source: TimeSource::Internal }
}

fn main() {
    println!("Starting Aviate SITL Quadcopter...");

    // 1. Initialize HAL
    let mut hal = SitlHal::new();

    // 2. Initialize Core
    let controller = McController::default();
    let mixer = QuadXMixer { timestamp_source: sitl_timestamp };
    let mode_config = ModeConfig {
        mode: ConfigMode::Hover,
        groups: &[], // Empty groups for now
    };

    let mut kernel = AviateKernel::new(controller, mixer, mode_config);

    // 3. Main Loop
    let mut last_tick = hal.now_us();

    loop {
        let now = hal.now_us();
        let _dt_us = now.saturating_sub(last_tick);
        last_tick = now;

        // 3.1 Read Sensors (Poll HAL)
        if let Some(imu) = hal.read_imu() {
            // Core EKF prediction
            // Note: predict needs dt in seconds (f32)
            // For SITL mock, we might not have valid dt if sensors are sparse.
            // Assuming 1kHz IMU (~0.001s)
            kernel.ekf.predict(&imu.value, 0.001);
        }

        if let Some(gnss) = hal.read_gnss() {
            kernel.ekf.update_gnss(&gnss);
        }

        // 3.2 Get Command (Stub)
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

        // 3.3 Step Kernel
        // First, ensure kernel is ready/armed for this test loop
        if !kernel.is_ready() {
             // In real loop, we feed sensors until ready
             // Here we might force init_step if we had sensor data
             // kernel.init_step(..., ...);
        } else {
             // Try to arm if ready
             let _ = kernel.arm();
        }

        let actuator_cmd = kernel.step(&cmd);

        // 3.4 Write Outputs
        hal.write(&actuator_cmd);

        // 3.5 Watchdog
        hal.kick_watchdog();

        // Sleep to simulate real-time (simple spin or sleep)
        std::thread::sleep(std::time::Duration::from_millis(10));
    }
}
