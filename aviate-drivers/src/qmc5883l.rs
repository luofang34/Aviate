//! QMC5883L 3-axis Magnetometer Driver
//!
//! The QMC5883L is a 3-axis magnetic sensor from QST Corporation.
//! It provides high-resolution magnetic field measurements suitable
//! for compass heading determination.
//!
//! ## Hardware Notes
//!
//! - I2C interface (up to 400 kHz)
//! - I2C address: 0x0D (fixed)
//! - Magnetic field range: ±2G or ±8G
//! - Resolution: 16-bit
//! - Data rate: 10, 50, 100, or 200 Hz
//!
//! ## Usage
//!
//! ```ignore
//! use aviate_drivers::qmc5883l::{Qmc5883l, Range, DataRate};
//!
//! let mut mag = Qmc5883l::new(i2c)?;
//! mag.set_range(Range::G8)?;
//! mag.set_data_rate(DataRate::Hz200)?;
//!
//! let reading = mag.read()?;
//! ```

use aviate_hal_io::error::{SensorError, SensorResult};
use aviate_hal_io::traits::{MagDriver, RawMagReading};
use embedded_hal::i2c::I2c;

use crate::Rotation;

// =============================================================================
// Register Addresses
// =============================================================================

mod reg {
    pub const DATA_X_LSB: u8 = 0x00;
    pub const DATA_X_MSB: u8 = 0x01;
    pub const DATA_Y_LSB: u8 = 0x02;
    pub const DATA_Y_MSB: u8 = 0x03;
    pub const DATA_Z_LSB: u8 = 0x04;
    pub const DATA_Z_MSB: u8 = 0x05;
    pub const STATUS: u8 = 0x06;
    pub const TEMP_LSB: u8 = 0x07;
    pub const TEMP_MSB: u8 = 0x08;
    pub const CONTROL_1: u8 = 0x09;
    pub const CONTROL_2: u8 = 0x0A;
    pub const SET_RESET_PERIOD: u8 = 0x0B;
    pub const CHIP_ID: u8 = 0x0D;

    pub const CHIP_ID_VALUE: u8 = 0xFF; // QMC5883L chip ID
}

// =============================================================================
// Configuration Types
// =============================================================================

/// Magnetic field measurement range
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
#[repr(u8)]
pub enum Range {
    /// ±2 Gauss (high sensitivity)
    #[default]
    G2 = 0x00,
    /// ±8 Gauss (wide range)
    G8 = 0x10,
}

impl Range {
    /// Get the scale factor in µT per LSB
    fn scale_factor(&self) -> f32 {
        // 1 Gauss = 100 µT
        match self {
            // ±2G = 4G range, 16-bit = 65536 counts
            // LSB = 4G * 100µT/G / 65536 = 0.00610 µT
            Range::G2 => 4.0 * 100.0 / 65536.0,
            // ±8G = 16G range
            // LSB = 16G * 100µT/G / 65536 = 0.0244 µT
            Range::G8 => 16.0 * 100.0 / 65536.0,
        }
    }
}

/// Data output rate
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
#[repr(u8)]
pub enum DataRate {
    /// 10 Hz
    Hz10 = 0x00,
    /// 50 Hz
    Hz50 = 0x04,
    /// 100 Hz
    Hz100 = 0x08,
    /// 200 Hz
    #[default]
    Hz200 = 0x0C,
}

/// Oversampling ratio (samples averaged per reading)
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
#[repr(u8)]
pub enum Oversampling {
    /// 512 samples
    #[default]
    X512 = 0x00,
    /// 256 samples
    X256 = 0x40,
    /// 128 samples
    X128 = 0x80,
    /// 64 samples
    X64 = 0xC0,
}

/// Operating mode
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
#[repr(u8)]
pub enum Mode {
    /// Standby mode
    Standby = 0x00,
    /// Continuous measurement mode
    #[default]
    Continuous = 0x01,
}

// =============================================================================
// Driver
// =============================================================================

/// QMC5883L magnetometer driver
///
/// Generic over an I2C bus.
pub struct Qmc5883l<I2C> {
    i2c: I2C,
    address: u8,
    scale: f32,
    rotation: Rotation,
    source_id: u8,
}

impl<I2C> Qmc5883l<I2C>
where
    I2C: I2c,
{
    /// Fixed I2C address for QMC5883L
    pub const ADDRESS: u8 = 0x0D;

    /// Create a new QMC5883L driver
    ///
    /// # Arguments
    /// * `i2c` - I2C bus
    ///
    /// # Returns
    /// Initialized QMC5883L driver or error
    pub fn new(i2c: I2C) -> SensorResult<Self> {
        Self::new_with_rotation(i2c, Rotation::None)
    }

    /// Create a new QMC5883L driver with rotation
    ///
    /// # Arguments
    /// * `i2c` - I2C bus
    /// * `rotation` - Sensor mounting rotation
    ///
    /// # Returns
    /// Initialized QMC5883L driver or error
    pub fn new_with_rotation(i2c: I2C, rotation: Rotation) -> SensorResult<Self> {
        let mut driver = Self {
            i2c,
            address: Self::ADDRESS,
            scale: Range::default().scale_factor(),
            rotation,
            source_id: 0,
        };

        driver.init()?;
        Ok(driver)
    }

    /// Create a new QMC5883L driver with custom source ID
    pub fn new_with_source_id(i2c: I2C, rotation: Rotation, source_id: u8) -> SensorResult<Self> {
        let mut driver = Self::new_with_rotation(i2c, rotation)?;
        driver.source_id = source_id;
        Ok(driver)
    }

    /// Initialize the sensor
    fn init(&mut self) -> SensorResult<()> {
        // Soft reset
        self.write_reg(reg::CONTROL_2, 0x80)?;

        // Wait for reset
        for _ in 0..10 {
            let _ = self.read_reg(reg::CHIP_ID);
        }

        // Verify chip ID
        let id = self.read_reg(reg::CHIP_ID)?;
        if id != reg::CHIP_ID_VALUE {
            return Err(SensorError::DeviceNotFound);
        }

        // Set SET/RESET period register (recommended value)
        self.write_reg(reg::SET_RESET_PERIOD, 0x01)?;

        // Configure: Continuous mode, 200Hz, ±2G, 512x oversampling
        let control1 = Mode::Continuous as u8
            | DataRate::Hz200 as u8
            | Range::G2 as u8
            | Oversampling::X512 as u8;
        self.write_reg(reg::CONTROL_1, control1)?;

        // Enable pointer roll-over (auto-increment address)
        self.write_reg(reg::CONTROL_2, 0x00)?;

        Ok(())
    }

    /// Set measurement range
    pub fn set_range(&mut self, range: Range) -> SensorResult<()> {
        let current = self.read_reg(reg::CONTROL_1)?;
        let new_val = (current & 0xCF) | (range as u8);
        self.write_reg(reg::CONTROL_1, new_val)?;
        self.scale = range.scale_factor();
        Ok(())
    }

    /// Set data output rate
    pub fn set_data_rate(&mut self, rate: DataRate) -> SensorResult<()> {
        let current = self.read_reg(reg::CONTROL_1)?;
        let new_val = (current & 0xF3) | (rate as u8);
        self.write_reg(reg::CONTROL_1, new_val)?;
        Ok(())
    }

    /// Set oversampling ratio
    pub fn set_oversampling(&mut self, osr: Oversampling) -> SensorResult<()> {
        let current = self.read_reg(reg::CONTROL_1)?;
        let new_val = (current & 0x3F) | (osr as u8);
        self.write_reg(reg::CONTROL_1, new_val)?;
        Ok(())
    }

    /// Read temperature sensor
    pub fn read_temperature(&mut self) -> SensorResult<f32> {
        let lsb = self.read_reg(reg::TEMP_LSB)?;
        let msb = self.read_reg(reg::TEMP_MSB)?;
        let raw = i16::from_le_bytes([lsb, msb]);
        // Temperature in °C (approximate)
        Ok(raw as f32 / 100.0)
    }

    /// Read a register
    fn read_reg(&mut self, reg: u8) -> SensorResult<u8> {
        let mut buf = [0u8];
        self.i2c
            .write_read(self.address, &[reg], &mut buf)
            .map_err(|_| SensorError::BusError)?;
        Ok(buf[0])
    }

    /// Write a register
    fn write_reg(&mut self, reg: u8, value: u8) -> SensorResult<()> {
        self.i2c
            .write(self.address, &[reg, value])
            .map_err(|_| SensorError::BusError)?;
        Ok(())
    }

    /// Read all magnetic field data
    fn read_data(&mut self) -> SensorResult<[i16; 3]> {
        let mut buf = [0u8; 6];
        self.i2c
            .write_read(self.address, &[reg::DATA_X_LSB], &mut buf)
            .map_err(|_| SensorError::BusError)?;

        Ok([
            i16::from_le_bytes([buf[0], buf[1]]),
            i16::from_le_bytes([buf[2], buf[3]]),
            i16::from_le_bytes([buf[4], buf[5]]),
        ])
    }
}

impl<I2C> MagDriver for Qmc5883l<I2C>
where
    I2C: I2c,
{
    fn read(&mut self) -> SensorResult<RawMagReading> {
        let raw = self.read_data()?;

        // Convert to µT
        let field = [
            raw[0] as f32 * self.scale,
            raw[1] as f32 * self.scale,
            raw[2] as f32 * self.scale,
        ];

        // Apply rotation
        let field_rotated = self.rotation.apply(field);

        Ok(RawMagReading {
            field_ut: field_rotated,
        })
    }

    fn data_ready(&mut self) -> SensorResult<bool> {
        let status = self.read_reg(reg::STATUS)?;
        // Bit 0: DRDY (data ready)
        Ok((status & 0x01) != 0)
    }

    fn source_id(&self) -> u8 {
        self.source_id
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_range_scale_factor() {
        // ±2G range: LSB = 4G * 100µT/G / 65536 ≈ 0.0061 µT
        let scale = Range::G2.scale_factor();
        assert!((scale - 0.0061).abs() < 0.001);

        // ±8G range: LSB = 16G * 100µT/G / 65536 ≈ 0.0244 µT
        let scale = Range::G8.scale_factor();
        assert!((scale - 0.0244).abs() < 0.001);
    }
}
