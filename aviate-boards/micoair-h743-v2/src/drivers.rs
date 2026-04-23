//! Real sensor drivers for MicoAir H743-V2
//!
//! This module provides sensor wrappers that implement Aviate traits using
//! embedded-hal 1.0 drivers from crates.io.
//!
//! ## Sensors on this board
//!
//! | Sensor | Model | Interface | Driver Crate |
//! |--------|-------|-----------|--------------|
//! | IMU 1 | BMI088 | SPI2 | `bmi088` |
//! | IMU 2 | BMI270 | SPI3 | (future) |
//! | Baro | SPL06 | I2C2 | `spl06-007` |
//! | Mag | QMC5883L | I2C1 | `qmc5883l` |
//!
//! ## embedded-hal compatibility
//!
//! The STM32H7 HAL implements embedded-hal 0.2.x, but our sensor drivers use
//! embedded-hal 1.0. We use `embedded-hal-compat` to bridge the gap:
//!
//! ```ignore
//! use embedded_hal_compat::ForwardCompat;
//!
//! let spi_eh1 = spi_eh02.forward();  // 0.2 -> 1.0
//! let sensor = SensorDriver::new(spi_eh1);
//! ```

use aviate_hal_io::error::{SensorError, SensorResult};
use aviate_hal_io::traits::{
    BaroDriver, ImuDriver, MagDriver, RawBaroReading, RawImuReading, RawMagReading,
};

use crate::Rotation;

// ============================================================================
// Rotation helper
// ============================================================================

impl Rotation {
    /// Apply rotation to a 3D vector [x, y, z]
    pub fn apply(&self, v: [f32; 3]) -> [f32; 3] {
        match self {
            Rotation::None => v,
            Rotation::Yaw90 => [-v[1], v[0], v[2]],
            Rotation::Yaw180 => [-v[0], -v[1], v[2]],
            Rotation::Yaw270 => [v[1], -v[0], v[2]],
            Rotation::Roll180 => [v[0], -v[1], -v[2]],
            Rotation::Pitch180 => [-v[0], v[1], -v[2]],
            // For other rotations, compute using sin/cos
            _ => {
                // Default to no rotation for unsupported rotations
                v
            }
        }
    }
}

// ============================================================================
// BMI088 IMU Wrapper
// ============================================================================

/// BMI088 IMU wrapper implementing ImuDriver
///
/// The BMI088 has separate accelerometer and gyroscope with individual SPI interfaces.
/// Both share the same SPI bus but have different chip select pins.
pub struct Bmi088Imu<SpiAccel, SpiGyro> {
    accel: bmi088::Accelerometer<bmi088::SpiInterface<SpiAccel>>,
    gyro: bmi088::Gyroscope<bmi088::SpiInterface<SpiGyro>>,
    accel_scale: f32,
    gyro_scale: f32,
    rotation: Rotation,
    source_id: u8,
}

impl<SpiAccel, SpiGyro> Bmi088Imu<SpiAccel, SpiGyro>
where
    SpiAccel: embedded_hal::spi::SpiDevice,
    SpiGyro: embedded_hal::spi::SpiDevice,
{
    /// Create a new BMI088 IMU wrapper
    ///
    /// # Arguments
    /// * `spi_accel` - SPI device for accelerometer (with CS management)
    /// * `spi_gyro` - SPI device for gyroscope (with CS management)
    /// * `delay` - Delay provider for initialization
    /// * `rotation` - Sensor mounting rotation
    pub fn new<D: embedded_hal::delay::DelayNs>(
        spi_accel: SpiAccel,
        spi_gyro: SpiGyro,
        delay: &mut D,
        rotation: Rotation,
    ) -> SensorResult<Self> {
        // Create sensor instances
        let mut accel = bmi088::Builder::new_accel_spi(spi_accel);
        let mut gyro = bmi088::Builder::new_gyro_spi(spi_gyro);

        // Initialize accelerometer
        accel.setup(delay).map_err(|_| SensorError::InitFailed)?;

        // Initialize gyroscope
        gyro.setup(delay).map_err(|_| SensorError::InitFailed)?;

        // Set gyro range to 2000 dps
        gyro.set_range(2000)
            .map_err(|_| SensorError::ConfigFailed)?;

        // Scale factors for default ranges
        // Accel: ±24g range (BMI088 default)
        const ACCEL_SCALE: f32 = (2.0 * 24.0 * 9.806_65) / 65536.0;
        // Gyro: ±2000°/s range
        const GYRO_SCALE: f32 = (2.0 * 2000.0 * core::f32::consts::PI / 180.0) / 65536.0;

        Ok(Self {
            accel,
            gyro,
            accel_scale: ACCEL_SCALE,
            gyro_scale: GYRO_SCALE,
            rotation,
            source_id: 0,
        })
    }

    /// Set sensor source ID
    pub fn with_source_id(mut self, id: u8) -> Self {
        self.source_id = id;
        self
    }
}

impl<SpiAccel, SpiGyro> ImuDriver for Bmi088Imu<SpiAccel, SpiGyro>
where
    SpiAccel: embedded_hal::spi::SpiDevice,
    SpiGyro: embedded_hal::spi::SpiDevice,
{
    fn read(&mut self) -> SensorResult<RawImuReading> {
        // Read accelerometer
        let accel_raw = self.accel.get_accel().map_err(|_| SensorError::BusError)?;

        // Read gyroscope
        let gyro_raw = self.gyro.get_gyro().map_err(|_| SensorError::BusError)?;

        // Convert to SI units
        let accel = [
            accel_raw[0] as f32 * self.accel_scale,
            accel_raw[1] as f32 * self.accel_scale,
            accel_raw[2] as f32 * self.accel_scale,
        ];

        let gyro = [
            gyro_raw[0] as f32 * self.gyro_scale,
            gyro_raw[1] as f32 * self.gyro_scale,
            gyro_raw[2] as f32 * self.gyro_scale,
        ];

        // Apply rotation
        Ok(RawImuReading {
            accel: self.rotation.apply(accel),
            gyro: self.rotation.apply(gyro),
            temperature: None,
        })
    }

    fn source_id(&self) -> u8 {
        self.source_id
    }
}

// ============================================================================
// SPL06 Barometer Wrapper (disabled - embedded-hal version mismatch)
// ============================================================================
// TODO: Update spl06-007 to embedded-hal 1.0

// ============================================================================
// QMC5883L Magnetometer Wrapper (disabled - embedded-hal version mismatch)
// ============================================================================
// TODO: Update qmc5883l to embedded-hal 1.0

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_rotation_apply() {
        let v = [1.0, 2.0, 3.0];

        // No rotation
        let result = Rotation::None.apply(v);
        assert_eq!(result, v);

        // Yaw 180
        let result = Rotation::Yaw180.apply(v);
        assert!((result[0] - (-1.0)).abs() < 1e-6);
        assert!((result[1] - (-2.0)).abs() < 1e-6);
        assert!((result[2] - 3.0).abs() < 1e-6);
    }
}
