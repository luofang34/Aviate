#![deny(missing_docs)]
//! Aviate Sensor and Actuator Drivers
//!
//! This crate provides hardware-independent sensor drivers that implement
//! the `aviate-hal-io` traits. Each driver is generic over `embedded-hal` 1.0
//! traits, allowing them to work with any microcontroller that provides
//! compatible SPI/I2C implementations.
//!
//! ## Supported Sensors
//!
//! | Sensor | Type | Interface | Trait |
//! |--------|------|-----------|-------|
//! | BMI088 | 6-axis IMU | SPI | `ImuDriver` |
//! | BMI270 | 6-axis IMU | SPI | `ImuDriver` |
//! | SPL06 | Barometer | I2C | `BaroDriver` |
//! | QMC5883L | Magnetometer | I2C | `MagDriver` |
//!
//! ## Example
//!
//! ```ignore
//! use aviate_drivers::bmi088::Bmi088;
//! use aviate_hal_io::ImuDriver;
//!
//! // Create BMI088 driver with SPI device
//! let mut imu = Bmi088::new(spi_accel, spi_gyro, Rotation::Rotate180)?;
//!
//! // Read IMU data
//! let reading = imu.read()?;
//! ```

#![no_std]
#![forbid(unsafe_code)]
#![deny(clippy::panic)]
#![deny(clippy::unwrap_used)]
#![deny(clippy::expect_used)]

pub mod bmi088;
pub mod bmi270;
pub mod qmc5883l;
pub mod spl06;

/// Sensor rotation for mounting orientation compensation
///
/// Represents standard rotations used in PX4/ArduPilot for sensor mounting.
/// The rotation is applied to the raw sensor data to align it with the
/// vehicle body frame (NED: North-East-Down).
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
#[repr(u8)]
pub enum Rotation {
    /// No rotation (0°)
    #[default]
    None = 0,
    /// Yaw 45°
    Yaw45 = 1,
    /// Yaw 90°
    Yaw90 = 2,
    /// Yaw 135°
    Yaw135 = 3,
    /// Yaw 180°
    Yaw180 = 4,
    /// Yaw 225°
    Yaw225 = 5,
    /// Yaw 270°
    Yaw270 = 6,
    /// Yaw 315°
    Yaw315 = 7,
    /// Roll 180°
    Roll180 = 8,
}

impl Rotation {
    /// Apply rotation to a 3D vector [x, y, z]
    ///
    /// Returns the rotated vector in the body frame
    pub fn apply(&self, v: [f32; 3]) -> [f32; 3] {
        match self {
            Rotation::None => v,
            Rotation::Yaw45 => {
                let c = 0.707_106_77; // cos(45°)
                let s = 0.707_106_77; // sin(45°)
                [v[0] * c - v[1] * s, v[0] * s + v[1] * c, v[2]]
            }
            Rotation::Yaw90 => [-v[1], v[0], v[2]],
            Rotation::Yaw135 => {
                let c = -0.707_106_77; // cos(135°)
                let s = 0.707_106_77; // sin(135°)
                [v[0] * c - v[1] * s, v[0] * s + v[1] * c, v[2]]
            }
            Rotation::Yaw180 => [-v[0], -v[1], v[2]],
            Rotation::Yaw225 => {
                let c = -0.707_106_77; // cos(225°)
                let s = -0.707_106_77; // sin(225°)
                [v[0] * c - v[1] * s, v[0] * s + v[1] * c, v[2]]
            }
            Rotation::Yaw270 => [v[1], -v[0], v[2]],
            Rotation::Yaw315 => {
                let c = 0.707_106_77; // cos(315°)
                let s = -0.707_106_77; // sin(315°)
                [v[0] * c - v[1] * s, v[0] * s + v[1] * c, v[2]]
            }
            Rotation::Roll180 => [v[0], -v[1], -v[2]],
        }
    }

    /// Get the rotation index (for PX4 compatibility)
    pub fn as_px4_index(&self) -> u8 {
        *self as u8
    }

    /// Create rotation from PX4 index
    pub fn from_px4_index(index: u8) -> Option<Self> {
        match index {
            0 => Some(Rotation::None),
            1 => Some(Rotation::Yaw45),
            2 => Some(Rotation::Yaw90),
            3 => Some(Rotation::Yaw135),
            4 => Some(Rotation::Yaw180),
            5 => Some(Rotation::Yaw225),
            6 => Some(Rotation::Yaw270),
            7 => Some(Rotation::Yaw315),
            8 => Some(Rotation::Roll180),
            _ => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_rotation_none() {
        let v = [1.0, 2.0, 3.0];
        let rotated = Rotation::None.apply(v);
        assert_eq!(rotated, v);
    }

    #[test]
    fn test_rotation_yaw90() {
        let v = [1.0, 0.0, 0.0];
        let rotated = Rotation::Yaw90.apply(v);
        // After 90° yaw: x becomes -y, y becomes x
        assert!((rotated[0] - 0.0).abs() < 1e-6);
        assert!((rotated[1] - 1.0).abs() < 1e-6);
        assert!((rotated[2] - 0.0).abs() < 1e-6);
    }

    #[test]
    fn test_rotation_yaw180() {
        let v = [1.0, 2.0, 3.0];
        let rotated = Rotation::Yaw180.apply(v);
        assert!((rotated[0] - (-1.0)).abs() < 1e-6);
        assert!((rotated[1] - (-2.0)).abs() < 1e-6);
        assert!((rotated[2] - 3.0).abs() < 1e-6);
    }

    #[test]
    fn test_rotation_roll180() {
        let v = [1.0, 2.0, 3.0];
        let rotated = Rotation::Roll180.apply(v);
        assert!((rotated[0] - 1.0).abs() < 1e-6);
        assert!((rotated[1] - (-2.0)).abs() < 1e-6);
        assert!((rotated[2] - (-3.0)).abs() < 1e-6);
    }

    #[test]
    fn test_px4_index_roundtrip() {
        for i in 0..=8 {
            if let Some(rot) = Rotation::from_px4_index(i) {
                assert_eq!(rot.as_px4_index(), i);
            }
        }
    }
}
