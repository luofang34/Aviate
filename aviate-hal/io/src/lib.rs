//! I/O Device Traits for Aviate
//!
//! This crate provides I/O device traits for both:
//! - **Sensors (input)**: IMU, barometer, magnetometer, GNSS
//! - **Actuators (bidirectional)**: Motors, servos, and other outputs with optional telemetry
//!
//! Used by both real hardware (via embedded-hal) and SITL simulation (fake devices).
//!
//! ## Architecture
//!
//! ```text
//! ┌─────────────────────────────────────────────────────────────────┐
//! │  aviate-core (SensorHal + ActuatorHal traits)                  │
//! └─────────────────────────────────────────────────────────────────┘
//!                           ↑
//!              implements SensorHal + ActuatorHal
//!                           ↑
//! ┌─────────────────────────────────────────────────────────────────┐
//! │  BoardHal<I, B, M, G, T, A>                                     │
//! │  - Composes I/O drivers (sensors + actuators)                  │
//! │  - Converts raw readings to aviate-core types                  │
//! │  - Handles timestamps and health                               │
//! └─────────────────────────────────────────────────────────────────┘
//!                           ↑
//!     ImuDriver, BaroDriver, MagDriver, GnssDriver, ActuatorDriver
//!                           ↑
//! ┌─────────────────────────────┬───────────────────────────────────┐
//! │  Real Hardware              │  SITL / Fake Devices              │
//! │  - Icm426xx<I2C>            │  - FakeImu                        │
//! │  - Bmp390<SPI>              │  - FakeBaro                       │
//! │  - Qmc5883l<I2C>            │  - FakeMag                        │
//! │  - UbloxGnss<UART>          │  - FakeGnss                       │
//! │  - PwmMotors<TIM>           │  - FakeActuator                   │
//! │  - DshotEscs<DMA>           │    (supports telemetry)           │
//! │  - CanEscs<FDCAN>           │                                   │
//! └─────────────────────────────┴───────────────────────────────────┘
//! ```
//!
//! ## Actuator Types
//!
//! The `ActuatorDriver` trait supports various actuator types with optional telemetry:
//!
//! | Type | Commands | Telemetry | Example |
//! |------|----------|-----------|---------|
//! | PWM Motors | write() | None | Basic ESCs |
//! | DShot ESCs | write() | RPM, errors | BLHeli32 |
//! | CAN ESCs | write() | Full telemetry | DroneCAN |
//! | Servos | write() | Position | Digital servos |
//! | Other | write() | Varies | Airbrakes, parachutes |
//!
//! ## Transport Independence
//!
//! This crate is **transport-agnostic**. The fake devices buffer data that is
//! exchanged by a transport layer (in `aviate-hal-xil`). The transport could be:
//! - MAVLink over UDP (current SITL implementation)
//! - Shared memory (future Gazebo plugin optimization)
//! - Custom protocol (other simulators)
//!
//! ## Usage - SITL
//!
//! ```ignore
//! use aviate_hal_io::{BoardHal, FakeImu, FakeBaro, FakeMag, FakeGnss, FakeActuator};
//!
//! // Create fake devices for SITL
//! let imu = FakeImu::new();
//! let baro = FakeBaro::new();
//! let mag = FakeMag::new();
//! let gnss = FakeGnss::new();
//! let actuator = FakeActuator::new();
//!
//! // Create BoardHal implementing SensorHal + ActuatorHal
//! let hal = BoardHal::new(imu, baro, mag, gnss, time_source, actuator);
//!
//! // In control loop:
//! // 1. Transport feeds sensors
//! hal.imu_mut().feed(sensor_data.imu);
//!
//! // 2. Read sensors (same interface as real hardware)
//! if let Some(imu) = hal.read_imu() {
//!     // Process IMU data
//! }
//!
//! // 3. Write actuators
//! hal.write(&actuator_cmd);
//!
//! // 4. Transport takes actuator command to send to simulator
//! if let Some(cmd) = hal.actuator_mut().take_cmd() {
//!     transport.send_actuator(&cmd);
//! }
//!
//! // 5. (Optional) Read actuator telemetry if simulator provides it
//! if let Some(status) = hal.actuator_mut().read_status() {
//!     // Process ESC telemetry: RPM, current, temperature
//! }
//! ```
//!
//! ## Usage - Real Hardware
//!
//! ```ignore
//! use aviate_hal_io::BoardHal;
//! use icm426xx::Icm426xx;  // Real IMU driver
//! use bmp3xx::Bmp3xx;      // Real baro driver
//!
//! // Create real sensor and actuator drivers
//! let imu = Icm426xx::new(i2c);
//! let baro = Bmp3xx::new(spi);
//! let mag = Qmc5883l::new(i2c);
//! let gnss = UbloxGnss::new(uart);
//! let motors = DshotEscGroup::new(tim, dma);  // ESC with telemetry support
//!
//! // Same BoardHal, same interface - kernel code unchanged!
//! let hal = BoardHal::new(imu, baro, mag, gnss, time_source, motors);
//!
//! // Read ESC telemetry (if supported by hardware)
//! if hal.actuator().status_ready() {
//!     if let Some(status) = hal.actuator_mut().read_status() {
//!         for (i, ch) in status.channels[..4].iter().enumerate() {
//!             if let Some(rpm) = ch.speed_or_position {
//!                 log::info!("Motor {} RPM: {}", i, rpm);
//!             }
//!         }
//!     }
//! }
//! ```

#![no_std]
#![forbid(unsafe_code)]

pub mod board_hal;
pub mod error;
pub mod fake;
pub mod traits;

// Main exports
pub use board_hal::BoardHal;
pub use error::{ActuatorError, ActuatorResult, SensorError, SensorResult};
pub use fake::{FakeActuator, FakeBaro, FakeGnss, FakeImu, FakeMag, FakeSensorSet};
pub use traits::{
    ActuatorDriver, ActuatorErrorFlags, ActuatorStatus, ActuatorTelemetry, BaroDriver, GnssDriver,
    GnssFix, ImuCalibration, ImuDriver, MagCalibration, MagDriver, RawActuatorCmd, RawBaroReading,
    RawGnssReading, RawImuReading, RawMagReading, TimeSource, MAX_ACTUATOR_OUTPUTS,
};

// Re-export core types for convenience
pub use aviate_core::sensor::{BaroData, GnssData, ImuData, MagData, SensorHealth, SensorReading};
pub use aviate_core::types::{
    Celsius, Meters, MetersPerSecond, MetersPerSecondSquared, Microtesla, Pascals, RadiansPerSecond,
};
