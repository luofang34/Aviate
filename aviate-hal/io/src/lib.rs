//! I/O Device Traits for Aviate
//!
//! This crate provides I/O device traits for both:
//! - **Sensors (input)**: IMU, barometer, magnetometer, GNSS
//! - **Actuators (output)**: Motors, servos (future)
//!
//! Used by both real hardware (via embedded-hal) and SITL simulation (fake devices).
//!
//! ## Architecture
//!
//! ```text
//! ┌─────────────────────────────────────────────────────────────┐
//! │  aviate-core (SensorHal trait)                              │
//! └─────────────────────────────────────────────────────────────┘
//!                           ↑
//!              implements SensorHal
//!                           ↑
//! ┌─────────────────────────────────────────────────────────────┐
//! │  BoardHal<I, B, M, G, T>                                    │
//! │  - Composes I/O drivers                                     │
//! │  - Converts raw readings to aviate-core types               │
//! │  - Handles timestamps and health                            │
//! └─────────────────────────────────────────────────────────────┘
//!                           ↑
//!              ImuDriver, BaroDriver, MagDriver, GnssDriver
//!                           ↑
//! ┌───────────────────────────┬─────────────────────────────────┐
//! │  Real Hardware            │  SITL / Fake Sensors            │
//! │  - Icm426xx<I2C>          │  - FakeImu (from HIL_SENSOR)    │
//! │  - Bmp390<SPI>            │  - FakeBaro (from HIL_SENSOR)   │
//! │  - Qmc5883l<I2C>          │  - FakeMag (from HIL_SENSOR)    │
//! │  - UbloxGnss<UART>        │  - FakeGnss (from HIL_GPS)      │
//! └───────────────────────────┴─────────────────────────────────┘
//! ```
//!
//! ## Usage - SITL
//!
//! ```ignore
//! use aviate_hal_io::{BoardHal, FakeSensorSet};
//!
//! // Create fake sensors for SITL
//! let mut sensors = FakeSensorSet::new();
//!
//! // In MAVLink handler, feed HIL data:
//! sensors.feed_hil_sensor_imu(msg.xacc, msg.yacc, msg.zacc,
//!                             msg.xgyro, msg.ygyro, msg.zgyro,
//!                             msg.temperature);
//! sensors.feed_hil_sensor_baro(msg.abs_pressure, msg.temperature);
//! sensors.feed_hil_sensor_mag(msg.xmag, msg.ymag, msg.zmag);
//!
//! // Create BoardHal implementing SensorHal
//! let hal = BoardHal::new(
//!     sensors.imu, sensors.baro, sensors.mag, sensors.gnss, time_source
//! );
//!
//! // Use with flight controller (same interface as real hardware)
//! if let Some(imu) = hal.read_imu() {
//!     // Process IMU data
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
//! // Create real sensor drivers
//! let imu = Icm426xx::new(i2c);
//! let baro = Bmp3xx::new(spi);
//! let mag = Qmc5883l::new(i2c);
//! let gnss = UbloxGnss::new(uart);
//!
//! // Same BoardHal, same interface
//! let hal = BoardHal::new(imu, baro, mag, gnss, time_source);
//! ```

#![no_std]
#![forbid(unsafe_code)]

pub mod board_hal;
pub mod error;
pub mod fake;
pub mod traits;

// Main exports
pub use board_hal::BoardHal;
pub use error::{SensorError, SensorResult};
pub use fake::{FakeBaro, FakeGnss, FakeImu, FakeMag, FakeSensorSet};
pub use traits::{
    BaroDriver, GnssDriver, GnssFix, ImuCalibration, ImuDriver, MagCalibration, MagDriver,
    RawBaroReading, RawGnssReading, RawImuReading, RawMagReading, TimeSource,
};

// Re-export core types for convenience
pub use aviate_core::sensor::{BaroData, GnssData, ImuData, MagData, SensorHealth, SensorReading};
pub use aviate_core::types::{
    Celsius, Meters, MetersPerSecond, MetersPerSecondSquared, Microtesla, Pascals, RadiansPerSecond,
};
