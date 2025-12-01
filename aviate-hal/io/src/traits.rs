//! Sensor and Actuator driver traits
//!
//! These traits define the interface for all I/O drivers, whether real hardware
//! (using embedded-hal) or simulated (SITL/fake devices).
//!
//! ## Design
//!
//! Each sensor type has a corresponding driver trait that abstracts the underlying
//! transport (I2C, SPI, UART, or simulated). Real hardware drivers use embedded-hal
//! traits while SITL drivers receive data from the simulator.
//!
//! ```text
//! ┌─────────────────────────────────────────────────────────────┐
//! │  SensorBridge (implements SensorHal)                        │
//! │  - Polls sensor drivers                                     │
//! │  - Converts raw readings to aviate-core types               │
//! │  - Handles timestamps and health                            │
//! └─────────────────────────────────────────────────────────────┘
//!                           ↑
//!              ImuDriver, BaroDriver, MagDriver, GnssDriver
//!                           ↑
//! ┌─────────────────────────────────────────────────────────────┐
//! │  Real Hardware              │  SITL / Fake Sensors          │
//! │  - Icm426xx<I2C>            │  - FakeImu (from HIL_SENSOR)  │
//! │  - Bmp390<SPI>              │  - FakeBaro (from HIL_SENSOR) │
//! │  - Qmc5883l<I2C>            │  - FakeMag (from HIL_SENSOR)  │
//! │  - UbloxGnss<UART>          │  - FakeGnss (from HIL_GPS)    │
//! └─────────────────────────────────────────────────────────────┘
//! ```

use crate::error::{ActuatorResult, SensorResult};

/// Time source for timestamps
pub trait TimeSource {
    /// Get current time in microseconds
    fn now_us(&self) -> u64;
}

// ============================================================================
// IMU Driver
// ============================================================================

/// Raw IMU reading
#[derive(Debug, Clone, Copy, Default)]
pub struct RawImuReading {
    /// Accelerometer X, Y, Z in m/s²
    pub accel: [f32; 3],
    /// Gyroscope X, Y, Z in rad/s
    pub gyro: [f32; 3],
    /// Optional temperature in Celsius
    pub temperature: Option<f32>,
}

/// IMU driver trait
///
/// Implement this for any IMU driver:
/// - Real hardware: ICM426xx, BMI088, MPU6050 (using embedded-hal I2C/SPI)
/// - SITL: FakeImu receiving data from HIL_SENSOR MAVLink message
pub trait ImuDriver {
    /// Read accelerometer and gyroscope data
    ///
    /// Returns calibrated data in SI units (m/s², rad/s)
    fn read(&mut self) -> SensorResult<RawImuReading>;

    /// Check if new data is available (for interrupt-driven operation)
    fn data_ready(&mut self) -> SensorResult<bool> {
        Ok(true) // Default: always ready (polled mode)
    }

    /// Get sensor source ID (for multi-sensor setups)
    fn source_id(&self) -> u8 {
        0
    }
}

// ============================================================================
// Barometer Driver
// ============================================================================

/// Raw barometer reading
#[derive(Debug, Clone, Copy, Default)]
pub struct RawBaroReading {
    /// Static pressure in Pascals
    pub pressure_pa: f32,
    /// Temperature in Celsius
    pub temperature_c: f32,
}

impl RawBaroReading {
    /// Calculate pressure altitude using standard atmosphere
    ///
    /// h = 44330.77 * (1 - (P/P0)^0.190284)
    /// where P0 = 101325 Pa (sea level standard pressure)
    pub fn altitude_m(&self) -> f32 {
        const P0: f32 = 101325.0;
        let ratio = self.pressure_pa / P0;
        44330.77 * (1.0 - libm::powf(ratio, 0.190284))
    }
}

/// Barometer driver trait
///
/// Implement this for any barometer driver:
/// - Real hardware: BMP390, MS5611, LPS22HB (using embedded-hal I2C/SPI)
/// - SITL: FakeBaro receiving data from HIL_SENSOR MAVLink message
pub trait BaroDriver {
    /// Read pressure and temperature
    fn read(&mut self) -> SensorResult<RawBaroReading>;

    /// Check if new data is available
    fn data_ready(&mut self) -> SensorResult<bool> {
        Ok(true)
    }

    /// Get sensor source ID
    fn source_id(&self) -> u8 {
        0
    }
}

// ============================================================================
// Magnetometer Driver
// ============================================================================

/// Raw magnetometer reading
#[derive(Debug, Clone, Copy, Default)]
pub struct RawMagReading {
    /// Magnetic field X, Y, Z in microtesla
    pub field_ut: [f32; 3],
}

/// Magnetometer driver trait
///
/// Implement this for any magnetometer driver:
/// - Real hardware: QMC5883L, HMC5883L, LIS3MDL (using embedded-hal I2C)
/// - SITL: FakeMag receiving data from HIL_SENSOR MAVLink message
pub trait MagDriver {
    /// Read magnetic field
    fn read(&mut self) -> SensorResult<RawMagReading>;

    /// Check if new data is available
    fn data_ready(&mut self) -> SensorResult<bool> {
        Ok(true)
    }

    /// Get sensor source ID
    fn source_id(&self) -> u8 {
        0
    }
}

// ============================================================================
// GNSS Driver
// ============================================================================

/// GNSS fix type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum GnssFix {
    #[default]
    None,
    TwoD,
    ThreeD,
    RtkFloat,
    RtkFixed,
}

/// Raw GNSS reading
#[derive(Debug, Clone, Copy, Default)]
pub struct RawGnssReading {
    /// Latitude in degrees
    pub lat_deg: f64,
    /// Longitude in degrees
    pub lon_deg: f64,
    /// Altitude above MSL in meters
    pub alt_m: f32,
    /// Velocity NED in m/s
    pub vel_ned: [f32; 3],
    /// Fix type
    pub fix: GnssFix,
    /// Horizontal accuracy estimate in meters
    pub h_acc: f32,
    /// Vertical accuracy estimate in meters
    pub v_acc: f32,
    /// Number of satellites
    pub satellites: u8,
}

/// GNSS driver trait
///
/// Implement this for any GNSS receiver:
/// - Real hardware: u-blox (using embedded-hal UART)
/// - SITL: FakeGnss receiving data from HIL_GPS MAVLink message
pub trait GnssDriver {
    /// Read GNSS position and velocity
    fn read(&mut self) -> SensorResult<RawGnssReading>;

    /// Check if new data is available
    fn data_ready(&mut self) -> SensorResult<bool> {
        Ok(true)
    }

    /// Get sensor source ID
    fn source_id(&self) -> u8 {
        0
    }
}

// ============================================================================
// Calibration Data
// ============================================================================

/// Calibration data for IMU
#[derive(Debug, Clone, Copy)]
pub struct ImuCalibration {
    /// Accelerometer bias (offset to subtract) in m/s²
    pub accel_bias: [f32; 3],
    /// Accelerometer scale factors (multiply after bias removal)
    pub accel_scale: [f32; 3],
    /// Gyroscope bias in rad/s
    pub gyro_bias: [f32; 3],
    /// Gyroscope scale factors
    pub gyro_scale: [f32; 3],
}

impl Default for ImuCalibration {
    fn default() -> Self {
        Self {
            accel_bias: [0.0; 3],
            accel_scale: [1.0; 3],
            gyro_bias: [0.0; 3],
            gyro_scale: [1.0; 3],
        }
    }
}

impl ImuCalibration {
    /// Apply calibration to raw reading
    pub fn apply(&self, raw: &RawImuReading) -> RawImuReading {
        RawImuReading {
            accel: [
                (raw.accel[0] - self.accel_bias[0]) * self.accel_scale[0],
                (raw.accel[1] - self.accel_bias[1]) * self.accel_scale[1],
                (raw.accel[2] - self.accel_bias[2]) * self.accel_scale[2],
            ],
            gyro: [
                (raw.gyro[0] - self.gyro_bias[0]) * self.gyro_scale[0],
                (raw.gyro[1] - self.gyro_bias[1]) * self.gyro_scale[1],
                (raw.gyro[2] - self.gyro_bias[2]) * self.gyro_scale[2],
            ],
            temperature: raw.temperature,
        }
    }
}

/// Calibration data for magnetometer
#[derive(Debug, Clone, Copy)]
pub struct MagCalibration {
    /// Hard iron offset (bias) in µT
    pub hard_iron: [f32; 3],
    /// Soft iron correction matrix (3x3, row-major)
    pub soft_iron: [[f32; 3]; 3],
}

impl Default for MagCalibration {
    fn default() -> Self {
        Self {
            hard_iron: [0.0; 3],
            soft_iron: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        }
    }
}

impl MagCalibration {
    /// Apply calibration to raw reading
    pub fn apply(&self, raw: &RawMagReading) -> RawMagReading {
        // Remove hard iron offset
        let corrected = [
            raw.field_ut[0] - self.hard_iron[0],
            raw.field_ut[1] - self.hard_iron[1],
            raw.field_ut[2] - self.hard_iron[2],
        ];

        // Apply soft iron correction matrix
        RawMagReading {
            field_ut: [
                self.soft_iron[0][0] * corrected[0]
                    + self.soft_iron[0][1] * corrected[1]
                    + self.soft_iron[0][2] * corrected[2],
                self.soft_iron[1][0] * corrected[0]
                    + self.soft_iron[1][1] * corrected[1]
                    + self.soft_iron[1][2] * corrected[2],
                self.soft_iron[2][0] * corrected[0]
                    + self.soft_iron[2][1] * corrected[1]
                    + self.soft_iron[2][2] * corrected[2],
            ],
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_imu_calibration_default_is_identity() {
        let cal = ImuCalibration::default();
        let raw = RawImuReading {
            accel: [1.0, 2.0, 3.0],
            gyro: [0.1, 0.2, 0.3],
            temperature: Some(25.0),
        };

        let calibrated = cal.apply(&raw);

        assert_eq!(calibrated.accel, raw.accel);
        assert_eq!(calibrated.gyro, raw.gyro);
    }

    #[test]
    fn test_imu_calibration_applies_bias() {
        let cal = ImuCalibration {
            accel_bias: [0.1, 0.2, 0.3],
            gyro_bias: [0.01, 0.02, 0.03],
            ..Default::default()
        };
        let raw = RawImuReading {
            accel: [1.0, 2.0, 3.0],
            gyro: [0.1, 0.2, 0.3],
            temperature: None,
        };

        let calibrated = cal.apply(&raw);

        assert!((calibrated.accel[0] - 0.9).abs() < 1e-6);
        assert!((calibrated.accel[1] - 1.8).abs() < 1e-6);
        assert!((calibrated.accel[2] - 2.7).abs() < 1e-6);
    }

    #[test]
    fn test_baro_altitude_sea_level() {
        let reading = RawBaroReading {
            pressure_pa: 101325.0,
            temperature_c: 15.0,
        };

        let alt = reading.altitude_m();
        assert!(alt.abs() < 1.0); // Should be ~0m at sea level
    }

    #[test]
    fn test_baro_altitude_1000m() {
        // At 1000m, pressure is approximately 89875 Pa
        let reading = RawBaroReading {
            pressure_pa: 89875.0,
            temperature_c: 15.0,
        };

        let alt = reading.altitude_m();
        assert!((alt - 1000.0).abs() < 50.0); // Within 50m of 1000m
    }

    #[test]
    fn test_gnss_fix_default() {
        let fix = GnssFix::default();
        assert_eq!(fix, GnssFix::None);
    }
}
