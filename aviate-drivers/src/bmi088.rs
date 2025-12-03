//! BMI088 6-axis IMU driver wrapper
//!
//! This module wraps the `bmi088` crate from crates.io and implements
//! the `ImuDriver` trait for Aviate HAL compatibility.
//!
//! The BMI088 is a high-performance inertial measurement unit with a
//! 16-bit digital, triaxial accelerometer and a 16-bit digital, triaxial
//! gyroscope. Both sensors use separate SPI interfaces.
//!
//! ## Hardware Notes
//!
//! - Accelerometer: SPI up to 10MHz, ±3/6/12/24g range
//! - Gyroscope: SPI up to 10MHz, ±125/250/500/1000/2000°/s range
//! - Each sensor has its own chip select pin
//! - DRDY pins available for interrupt-driven operation
//!
//! ## Usage
//!
//! ```ignore
//! use aviate_drivers::bmi088::Bmi088;
//!
//! let mut imu = Bmi088::new(spi_accel, spi_gyro, delay, rotation)?;
//! let reading = imu.read()?;
//! ```

use aviate_hal_io::error::{SensorError, SensorResult};
use aviate_hal_io::traits::{ImuDriver, RawImuReading};
use bmi088::{Accelerometer, Builder, Gyroscope, SpiInterface};
use embedded_hal::delay::DelayNs;
use embedded_hal::spi::SpiDevice;

use crate::Rotation;

// =============================================================================
// Configuration Types (re-export compatible types)
// =============================================================================

/// Accelerometer range setting
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub enum AccelRange {
    /// ±3g
    G3,
    /// ±6g
    #[default]
    G6,
    /// ±12g
    G12,
    /// ±24g
    G24,
}

impl AccelRange {
    /// Get the scale factor in m/s² per LSB
    fn scale_factor(&self) -> f32 {
        // 16-bit signed, range is ±range_g
        // LSB = (2 * range_g * 9.80665) / 65536
        const G: f32 = 9.806_65;
        match self {
            AccelRange::G3 => (2.0 * 3.0 * G) / 65536.0,
            AccelRange::G6 => (2.0 * 6.0 * G) / 65536.0,
            AccelRange::G12 => (2.0 * 12.0 * G) / 65536.0,
            AccelRange::G24 => (2.0 * 24.0 * G) / 65536.0,
        }
    }
}

/// Gyroscope range setting
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub enum GyroRange {
    /// ±125°/s
    Dps125,
    /// ±250°/s
    Dps250,
    /// ±500°/s
    Dps500,
    /// ±1000°/s
    Dps1000,
    /// ±2000°/s
    #[default]
    Dps2000,
}

impl GyroRange {
    /// Get the scale factor in rad/s per LSB
    fn scale_factor(&self) -> f32 {
        // 16-bit signed, range is ±range_dps
        // LSB = (2 * range_dps * π/180) / 65536
        const DEG_TO_RAD: f32 = core::f32::consts::PI / 180.0;
        match self {
            GyroRange::Dps125 => (2.0 * 125.0 * DEG_TO_RAD) / 65536.0,
            GyroRange::Dps250 => (2.0 * 250.0 * DEG_TO_RAD) / 65536.0,
            GyroRange::Dps500 => (2.0 * 500.0 * DEG_TO_RAD) / 65536.0,
            GyroRange::Dps1000 => (2.0 * 1000.0 * DEG_TO_RAD) / 65536.0,
            GyroRange::Dps2000 => (2.0 * 2000.0 * DEG_TO_RAD) / 65536.0,
        }
    }

    /// Convert to bmi088 crate's dps value
    fn to_dps(&self) -> u32 {
        match self {
            GyroRange::Dps125 => 125,
            GyroRange::Dps250 => 250,
            GyroRange::Dps500 => 500,
            GyroRange::Dps1000 => 1000,
            GyroRange::Dps2000 => 2000,
        }
    }
}

// =============================================================================
// Driver Wrapper
// =============================================================================

/// BMI088 IMU driver wrapper
///
/// Wraps the `bmi088` crate's Accelerometer and Gyroscope types to provide
/// a unified interface implementing the `ImuDriver` trait.
pub struct Bmi088<SpiAccel, SpiGyro, D>
where
    SpiAccel: SpiDevice,
    SpiGyro: SpiDevice,
{
    accel: Accelerometer<SpiInterface<SpiAccel>>,
    gyro: Gyroscope<SpiInterface<SpiGyro>>,
    delay: D,
    accel_scale: f32,
    gyro_scale: f32,
    rotation: Rotation,
    source_id: u8,
}

impl<SpiAccel, SpiGyro, D> Bmi088<SpiAccel, SpiGyro, D>
where
    SpiAccel: SpiDevice,
    SpiGyro: SpiDevice,
    <SpiAccel as embedded_hal::spi::ErrorType>::Error: core::fmt::Debug,
    <SpiGyro as embedded_hal::spi::ErrorType>::Error: core::fmt::Debug,
    D: DelayNs,
{
    /// Create a new BMI088 driver
    ///
    /// # Arguments
    /// * `spi_accel` - SPI device for accelerometer
    /// * `spi_gyro` - SPI device for gyroscope
    /// * `delay` - Delay provider for initialization
    /// * `rotation` - Sensor mounting rotation
    ///
    /// # Returns
    /// Initialized BMI088 driver or error
    pub fn new(
        spi_accel: SpiAccel,
        spi_gyro: SpiGyro,
        mut delay: D,
        rotation: Rotation,
    ) -> SensorResult<Self> {
        // Create sensor instances using Builder associated functions
        let mut accel = Builder::new_accel_spi(spi_accel);
        let mut gyro = Builder::new_gyro_spi(spi_gyro);

        // Initialize accelerometer
        accel.setup(&mut delay).map_err(|_| SensorError::BusError)?;

        // Initialize gyroscope
        gyro.setup(&mut delay).map_err(|_| SensorError::BusError)?;

        // Set default gyro configuration
        gyro.set_range(GyroRange::default().to_dps())
            .map_err(|_| SensorError::BusError)?;
        gyro.set_bandwidth(2000)
            .map_err(|_| SensorError::BusError)?;

        Ok(Self {
            accel,
            gyro,
            delay,
            accel_scale: AccelRange::default().scale_factor(),
            gyro_scale: GyroRange::default().scale_factor(),
            rotation,
            source_id: 0,
        })
    }

    /// Create a new BMI088 driver with custom source ID
    pub fn new_with_source_id(
        spi_accel: SpiAccel,
        spi_gyro: SpiGyro,
        delay: D,
        rotation: Rotation,
        source_id: u8,
    ) -> SensorResult<Self> {
        let mut driver = Self::new(spi_accel, spi_gyro, delay, rotation)?;
        driver.source_id = source_id;
        Ok(driver)
    }

    /// Set gyroscope range
    pub fn set_gyro_range(&mut self, range: GyroRange) -> SensorResult<()> {
        self.gyro
            .set_range(range.to_dps())
            .map_err(|_| SensorError::BusError)?;
        self.gyro_scale = range.scale_factor();
        Ok(())
    }

    /// Set gyroscope bandwidth/ODR
    pub fn set_gyro_bandwidth(&mut self, odr_hz: u32) -> SensorResult<()> {
        self.gyro
            .set_bandwidth(odr_hz)
            .map_err(|_| SensorError::BusError)?;
        Ok(())
    }

    /// Perform soft reset on both sensors
    pub fn soft_reset(&mut self) -> SensorResult<()> {
        self.accel
            .soft_reset(&mut self.delay)
            .map_err(|_| SensorError::BusError)?;
        self.gyro
            .soft_reset(&mut self.delay)
            .map_err(|_| SensorError::BusError)?;
        Ok(())
    }
}

impl<SpiAccel, SpiGyro, D> ImuDriver for Bmi088<SpiAccel, SpiGyro, D>
where
    SpiAccel: SpiDevice,
    SpiGyro: SpiDevice,
    <SpiAccel as embedded_hal::spi::ErrorType>::Error: core::fmt::Debug,
    <SpiGyro as embedded_hal::spi::ErrorType>::Error: core::fmt::Debug,
    D: DelayNs,
{
    fn read(&mut self) -> SensorResult<RawImuReading> {
        // Read accelerometer
        let accel_raw = self
            .accel
            .get_accel()
            .map_err(|_| SensorError::BusError)?;

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
        let accel_rotated = self.rotation.apply(accel);
        let gyro_rotated = self.rotation.apply(gyro);

        Ok(RawImuReading {
            accel: accel_rotated,
            gyro: gyro_rotated,
            temperature: None, // External crate doesn't expose temperature easily
        })
    }

    fn data_ready(&mut self) -> SensorResult<bool> {
        // The external crate doesn't expose data ready status directly
        // Return true as data is always available in continuous mode
        Ok(true)
    }

    fn source_id(&self) -> u8 {
        self.source_id
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_accel_range_scale() {
        // ±6g range: LSB = (2 * 6 * 9.80665) / 65536 ≈ 0.00179
        let scale = AccelRange::G6.scale_factor();
        assert!((scale - 0.001_794_5).abs() < 0.0001);
    }

    #[test]
    fn test_gyro_range_scale() {
        // ±2000°/s range: LSB = (2 * 2000 * π/180) / 65536 ≈ 0.00106
        let scale = GyroRange::Dps2000.scale_factor();
        assert!((scale - 0.001_065_3).abs() < 0.0001);
    }

    #[test]
    fn test_gyro_range_to_dps() {
        assert_eq!(GyroRange::Dps125.to_dps(), 125);
        assert_eq!(GyroRange::Dps2000.to_dps(), 2000);
    }
}
