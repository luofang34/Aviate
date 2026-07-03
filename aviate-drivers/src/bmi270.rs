//! BMI270 6-axis IMU driver wrapper
//!
//! This module wraps the `bmi2` crate from crates.io and implements
//! the `ImuDriver` trait for Aviate HAL compatibility.
//!
//! The BMI270 is a high-performance inertial measurement unit with a
//! 16-bit digital, triaxial accelerometer and a 16-bit digital, triaxial
//! gyroscope. Unlike BMI088, both sensors share a single SPI interface.
//!
//! ## Hardware Notes
//!
//! - SPI up to 10MHz
//! - Accelerometer: ±2/4/8/16g range
//! - Gyroscope: ±125/250/500/1000/2000°/s range
//! - Single chip select for both accel and gyro
//! - Requires config file upload during initialization
//!
//! ## Usage
//!
//! ```ignore
//! use aviate_drivers::bmi270::Bmi270;
//!
//! let mut imu = Bmi270::new(spi, delay, config_file, rotation)?;
//! let reading = imu.read()?;
//! ```

use aviate_hal_io::error::{SensorError, SensorResult};
use aviate_hal_io::traits::{ImuDriver, RawImuReading};
use bmi2::interface::SpiInterface;
use bmi2::types::{
    AccRange as Bmi2AccRange, Burst, GyrRange as Bmi2GyrRange, GyrRangeVal, OisRange, PwrCtrl,
};
use bmi2::Bmi2;
use embedded_hal::delay::DelayNs;
use embedded_hal::spi::SpiDevice;

/// Stack buffer length for config-file burst writes. Must be at least the
/// configured burst size: `Bmi2::init` rejects `max_burst > N` with
/// `BufferTooSmall`. `Burst::default()` bursts 512 bytes.
const BURST_BUF_LEN: usize = 512;

use crate::Rotation;

// =============================================================================
// Configuration Types
// =============================================================================

/// Accelerometer range setting
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub enum AccelRange {
    /// ±2g
    G2,
    /// ±4g
    G4,
    /// ±8g
    #[default]
    G8,
    /// ±16g
    G16,
}

impl AccelRange {
    /// Get the scale factor in m/s² per LSB
    fn scale_factor(&self) -> f32 {
        const G: f32 = 9.806_65;
        match self {
            AccelRange::G2 => (2.0 * 2.0 * G) / 65536.0,
            AccelRange::G4 => (2.0 * 4.0 * G) / 65536.0,
            AccelRange::G8 => (2.0 * 8.0 * G) / 65536.0,
            AccelRange::G16 => (2.0 * 16.0 * G) / 65536.0,
        }
    }

    /// Convert to bmi2 crate's AccRange
    fn to_bmi2(self) -> Bmi2AccRange {
        match self {
            AccelRange::G2 => Bmi2AccRange::Range2g,
            AccelRange::G4 => Bmi2AccRange::Range4g,
            AccelRange::G8 => Bmi2AccRange::Range8g,
            AccelRange::G16 => Bmi2AccRange::Range16g,
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
        const DEG_TO_RAD: f32 = core::f32::consts::PI / 180.0;
        match self {
            GyroRange::Dps125 => (2.0 * 125.0 * DEG_TO_RAD) / 65536.0,
            GyroRange::Dps250 => (2.0 * 250.0 * DEG_TO_RAD) / 65536.0,
            GyroRange::Dps500 => (2.0 * 500.0 * DEG_TO_RAD) / 65536.0,
            GyroRange::Dps1000 => (2.0 * 1000.0 * DEG_TO_RAD) / 65536.0,
            GyroRange::Dps2000 => (2.0 * 2000.0 * DEG_TO_RAD) / 65536.0,
        }
    }

    /// Convert to bmi2 crate's GyrRange
    fn to_bmi2(self) -> Bmi2GyrRange {
        Bmi2GyrRange {
            range: match self {
                GyroRange::Dps125 => GyrRangeVal::Range125,
                GyroRange::Dps250 => GyrRangeVal::Range250,
                GyroRange::Dps500 => GyrRangeVal::Range500,
                GyroRange::Dps1000 => GyrRangeVal::Range1000,
                GyroRange::Dps2000 => GyrRangeVal::Range2000,
            },
            ois_range: OisRange::Range2000,
        }
    }
}

// =============================================================================
// Driver Wrapper
// =============================================================================

/// BMI270 IMU driver wrapper
///
/// Wraps the `bmi2` crate to provide a unified interface implementing
/// the `ImuDriver` trait.
pub struct Bmi270<Spi, D>
where
    Spi: SpiDevice,
{
    bmi2: Bmi2<SpiInterface<Spi>, D, BURST_BUF_LEN>,
    accel_scale: f32,
    gyro_scale: f32,
    rotation: Rotation,
    source_id: u8,
}

impl<Spi, D> Bmi270<Spi, D>
where
    Spi: SpiDevice,
    <Spi as embedded_hal::spi::ErrorType>::Error: core::fmt::Debug,
    D: DelayNs,
{
    /// Create a new BMI270 driver
    ///
    /// # Arguments
    /// * `spi` - SPI device
    /// * `delay` - Delay provider for the chip's datasheet-mandated init
    ///   delays (soft-reset settle, power-save toggles)
    /// * `config_file` - BMI270 config file (from Bosch SDK)
    /// * `rotation` - Sensor mounting rotation
    ///
    /// # Returns
    /// Initialized BMI270 driver or error
    pub fn new(spi: Spi, delay: D, config_file: &[u8], rotation: Rotation) -> SensorResult<Self> {
        // Create BMI2 instance with SPI interface.
        // Burst::default() = 512-byte bursts.
        let mut bmi2 = Bmi2::new_spi(spi, delay, Burst::default());

        // Initialize with config file
        bmi2.init(config_file)
            .map_err(|_| SensorError::DeviceNotFound)?;

        // Enable accelerometer and gyroscope
        let pwr_ctrl = PwrCtrl {
            aux_en: false,
            gyr_en: true,
            acc_en: true,
            temp_en: true,
        };
        bmi2.set_pwr_ctrl(pwr_ctrl)
            .map_err(|_| SensorError::BusError)?;

        // Set default ranges
        bmi2.set_acc_range(AccelRange::default().to_bmi2())
            .map_err(|_| SensorError::BusError)?;
        bmi2.set_gyr_range(GyroRange::default().to_bmi2())
            .map_err(|_| SensorError::BusError)?;

        Ok(Self {
            bmi2,
            accel_scale: AccelRange::default().scale_factor(),
            gyro_scale: GyroRange::default().scale_factor(),
            rotation,
            source_id: 1, // Default to source 1 (secondary IMU)
        })
    }

    /// Create a new BMI270 driver with custom source ID
    pub fn new_with_source_id(
        spi: Spi,
        delay: D,
        config_file: &[u8],
        rotation: Rotation,
        source_id: u8,
    ) -> SensorResult<Self> {
        let mut driver = Self::new(spi, delay, config_file, rotation)?;
        driver.source_id = source_id;
        Ok(driver)
    }

    /// Set accelerometer range
    pub fn set_accel_range(&mut self, range: AccelRange) -> SensorResult<()> {
        self.bmi2
            .set_acc_range(range.to_bmi2())
            .map_err(|_| SensorError::BusError)?;
        self.accel_scale = range.scale_factor();
        Ok(())
    }

    /// Set gyroscope range
    pub fn set_gyro_range(&mut self, range: GyroRange) -> SensorResult<()> {
        self.bmi2
            .set_gyr_range(range.to_bmi2())
            .map_err(|_| SensorError::BusError)?;
        self.gyro_scale = range.scale_factor();
        Ok(())
    }

    /// Read temperature
    pub fn read_temperature(&mut self) -> SensorResult<Option<f32>> {
        self.bmi2
            .get_temperature()
            .map_err(|_| SensorError::BusError)
    }
}

impl<Spi, D> ImuDriver for Bmi270<Spi, D>
where
    Spi: SpiDevice,
    <Spi as embedded_hal::spi::ErrorType>::Error: core::fmt::Debug,
    D: DelayNs,
{
    fn read(&mut self) -> SensorResult<RawImuReading> {
        // Read combined data
        let data = self.bmi2.get_data().map_err(|_| SensorError::BusError)?;

        // Convert to SI units
        let accel = [
            data.acc.x as f32 * self.accel_scale,
            data.acc.y as f32 * self.accel_scale,
            data.acc.z as f32 * self.accel_scale,
        ];

        let gyro = [
            data.gyr.x as f32 * self.gyro_scale,
            data.gyr.y as f32 * self.gyro_scale,
            data.gyr.z as f32 * self.gyro_scale,
        ];

        // Apply rotation
        let accel_rotated = self.rotation.apply(accel);
        let gyro_rotated = self.rotation.apply(gyro);

        // Get temperature if available
        let temperature = self.read_temperature().ok().flatten();

        Ok(RawImuReading {
            accel: accel_rotated,
            gyro: gyro_rotated,
            temperature,
        })
    }

    fn data_ready(&mut self) -> SensorResult<bool> {
        // Check status register for data ready
        let status = self.bmi2.get_status().map_err(|_| SensorError::BusError)?;
        Ok(status.acc_data_ready && status.gyr_data_ready)
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
        // ±8g range
        let scale = AccelRange::G8.scale_factor();
        assert!((scale - 0.002_393).abs() < 0.0001);
    }

    #[test]
    fn test_gyro_range_scale() {
        // ±2000°/s range
        let scale = GyroRange::Dps2000.scale_factor();
        assert!((scale - 0.001_065).abs() < 0.0001);
    }
}
