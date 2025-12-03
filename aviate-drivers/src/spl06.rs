//! SPL06 Barometric Pressure Sensor Driver
//!
//! The SPL06 is a digital barometric pressure and temperature sensor
//! from Goertek. It provides high-precision pressure measurements
//! suitable for altitude estimation.
//!
//! ## Hardware Notes
//!
//! - I2C interface (up to 3.4 MHz)
//! - I2C address: 0x76 or 0x77 (depending on SDO pin)
//! - Pressure range: 300-1100 hPa
//! - Resolution: 0.06 Pa (with 128x oversampling)
//! - Temperature sensor included
//!
//! ## Usage
//!
//! ```ignore
//! use aviate_drivers::spl06::{Spl06, OversamplingRate};
//!
//! let mut baro = Spl06::new(i2c, 0x77)?;
//! baro.set_pressure_oversampling(OversamplingRate::X16)?;
//!
//! let reading = baro.read()?;
//! println!("Pressure: {} Pa, Altitude: {} m", reading.pressure_pa, reading.altitude_m());
//! ```

use aviate_hal_io::error::{SensorError, SensorResult};
use aviate_hal_io::traits::{BaroDriver, RawBaroReading};
use embedded_hal::i2c::I2c;

// =============================================================================
// Register Addresses
// =============================================================================

mod reg {
    pub const PSR_B2: u8 = 0x00; // Pressure MSB
    pub const PSR_B1: u8 = 0x01;
    pub const PSR_B0: u8 = 0x02; // Pressure LSB
    pub const TMP_B2: u8 = 0x03; // Temperature MSB
    pub const TMP_B1: u8 = 0x04;
    pub const TMP_B0: u8 = 0x05; // Temperature LSB
    pub const PRS_CFG: u8 = 0x06; // Pressure config
    pub const TMP_CFG: u8 = 0x07; // Temperature config
    pub const MEAS_CFG: u8 = 0x08; // Measurement config
    pub const CFG_REG: u8 = 0x09; // Interrupt/FIFO config
    pub const INT_STS: u8 = 0x0A; // Interrupt status
    pub const FIFO_STS: u8 = 0x0B; // FIFO status
    pub const RESET: u8 = 0x0C; // Soft reset
    pub const ID: u8 = 0x0D; // Product/revision ID

    // Calibration coefficients (factory trimmed)
    pub const COEF_C0_H: u8 = 0x10;
    pub const COEF_C0_L_C1_H: u8 = 0x11;
    pub const COEF_C1_L: u8 = 0x12;
    pub const COEF_C00_H: u8 = 0x13;
    pub const COEF_C00_M: u8 = 0x14;
    pub const COEF_C00_L_C10_H: u8 = 0x15;
    pub const COEF_C10_M: u8 = 0x16;
    pub const COEF_C10_L: u8 = 0x17;
    pub const COEF_C01_H: u8 = 0x18;
    pub const COEF_C01_L: u8 = 0x19;
    pub const COEF_C11_H: u8 = 0x1A;
    pub const COEF_C11_L: u8 = 0x1B;
    pub const COEF_C20_H: u8 = 0x1C;
    pub const COEF_C20_L: u8 = 0x1D;
    pub const COEF_C21_H: u8 = 0x1E;
    pub const COEF_C21_L: u8 = 0x1F;
    pub const COEF_C30_H: u8 = 0x20;
    pub const COEF_C30_L: u8 = 0x21;

    pub const PRODUCT_ID: u8 = 0x10; // Expected product ID
}

// =============================================================================
// Configuration Types
// =============================================================================

/// Oversampling rate for pressure and temperature measurements
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
#[repr(u8)]
pub enum OversamplingRate {
    /// 1x (single measurement)
    X1 = 0,
    /// 2x
    X2 = 1,
    /// 4x
    X4 = 2,
    /// 8x
    X8 = 3,
    /// 16x (default)
    #[default]
    X16 = 4,
    /// 32x
    X32 = 5,
    /// 64x
    X64 = 6,
    /// 128x (highest precision)
    X128 = 7,
}

impl OversamplingRate {
    /// Get the scale factor (kP or kT) for this oversampling rate
    fn scale_factor(&self) -> f32 {
        match self {
            OversamplingRate::X1 => 524288.0,
            OversamplingRate::X2 => 1572864.0,
            OversamplingRate::X4 => 3670016.0,
            OversamplingRate::X8 => 7864320.0,
            OversamplingRate::X16 => 253952.0,
            OversamplingRate::X32 => 516096.0,
            OversamplingRate::X64 => 1040384.0,
            OversamplingRate::X128 => 2088960.0,
        }
    }
}

/// Measurement rate
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
#[repr(u8)]
pub enum MeasurementRate {
    /// 1 Hz
    Hz1 = 0,
    /// 2 Hz
    Hz2 = 1,
    /// 4 Hz
    Hz4 = 2,
    /// 8 Hz
    Hz8 = 3,
    /// 16 Hz
    Hz16 = 4,
    /// 32 Hz
    Hz32 = 5,
    /// 64 Hz
    Hz64 = 6,
    /// 128 Hz
    #[default]
    Hz128 = 7,
}

/// Measurement mode
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
#[repr(u8)]
pub enum MeasurementMode {
    /// Idle (standby)
    #[default]
    Idle = 0,
    /// Single pressure measurement
    SinglePressure = 1,
    /// Single temperature measurement
    SingleTemperature = 2,
    /// Continuous pressure measurement
    ContinuousPressure = 5,
    /// Continuous temperature measurement
    ContinuousTemperature = 6,
    /// Continuous pressure and temperature measurement
    ContinuousBoth = 7,
}

// =============================================================================
// Calibration Coefficients
// =============================================================================

/// Factory calibration coefficients
#[derive(Debug, Clone, Copy, Default)]
struct Coefficients {
    c0: i16,
    c1: i16,
    c00: i32,
    c10: i32,
    c01: i16,
    c11: i16,
    c20: i16,
    c21: i16,
    c30: i16,
}

// =============================================================================
// Driver
// =============================================================================

/// SPL06 barometer driver
///
/// Generic over an I2C bus.
pub struct Spl06<I2C> {
    i2c: I2C,
    address: u8,
    coef: Coefficients,
    kp: f32,
    kt: f32,
    source_id: u8,
}

impl<I2C> Spl06<I2C>
where
    I2C: I2c,
{
    /// Default I2C address (SDO pin high)
    pub const DEFAULT_ADDRESS: u8 = 0x77;

    /// Alternate I2C address (SDO pin low)
    pub const ALTERNATE_ADDRESS: u8 = 0x76;

    /// Create a new SPL06 driver
    ///
    /// # Arguments
    /// * `i2c` - I2C bus
    /// * `address` - I2C address (0x76 or 0x77)
    ///
    /// # Returns
    /// Initialized SPL06 driver or error
    pub fn new(i2c: I2C, address: u8) -> SensorResult<Self> {
        let mut driver = Self {
            i2c,
            address,
            coef: Coefficients::default(),
            kp: OversamplingRate::X16.scale_factor(),
            kt: OversamplingRate::X16.scale_factor(),
            source_id: 0,
        };

        driver.init()?;
        Ok(driver)
    }

    /// Create a new SPL06 driver with default address
    pub fn new_default(i2c: I2C) -> SensorResult<Self> {
        Self::new(i2c, Self::DEFAULT_ADDRESS)
    }

    /// Initialize the sensor
    fn init(&mut self) -> SensorResult<()> {
        // Soft reset
        self.write_reg(reg::RESET, 0x89)?;

        // Wait for reset and sensor ready
        for _ in 0..100 {
            let status = self.read_reg(reg::MEAS_CFG)?;
            // Check SENSOR_RDY (bit 6) and COEF_RDY (bit 7)
            if (status & 0xC0) == 0xC0 {
                break;
            }
        }

        // Verify product ID
        let id = self.read_reg(reg::ID)?;
        if (id & 0xF0) != reg::PRODUCT_ID {
            return Err(SensorError::DeviceNotFound);
        }

        // Read calibration coefficients
        self.read_coefficients()?;

        // Configure pressure: 16x oversampling, 128 Hz rate
        self.set_pressure_config(MeasurementRate::Hz128, OversamplingRate::X16)?;

        // Configure temperature: 16x oversampling, 128 Hz rate
        // Use internal temperature sensor (bit 7 = 0)
        self.set_temperature_config(MeasurementRate::Hz128, OversamplingRate::X16)?;

        // Set bit shift for >8x oversampling
        self.write_reg(reg::CFG_REG, 0x04)?; // P_SHIFT=1, T_SHIFT=0
        // Note: T_SHIFT should also be set for >8x temp oversampling
        self.write_reg(reg::CFG_REG, 0x0C)?; // P_SHIFT=1, T_SHIFT=1

        // Start continuous pressure and temperature measurement
        self.set_mode(MeasurementMode::ContinuousBoth)?;

        Ok(())
    }

    /// Read calibration coefficients from sensor
    fn read_coefficients(&mut self) -> SensorResult<()> {
        let mut buf = [0u8; 18];

        // Read all coefficient registers
        for (i, byte) in buf.iter_mut().enumerate() {
            *byte = self.read_reg(reg::COEF_C0_H + i as u8)?;
        }

        // Parse coefficients (they are stored in a packed format)
        // c0: 12 bits
        self.coef.c0 = ((buf[0] as i16) << 4) | ((buf[1] as i16) >> 4);
        if self.coef.c0 > 2047 {
            self.coef.c0 -= 4096;
        }

        // c1: 12 bits
        self.coef.c1 = (((buf[1] & 0x0F) as i16) << 8) | (buf[2] as i16);
        if self.coef.c1 > 2047 {
            self.coef.c1 -= 4096;
        }

        // c00: 20 bits
        self.coef.c00 = ((buf[3] as i32) << 12) | ((buf[4] as i32) << 4) | ((buf[5] as i32) >> 4);
        if self.coef.c00 > 524287 {
            self.coef.c00 -= 1048576;
        }

        // c10: 20 bits
        self.coef.c10 =
            (((buf[5] & 0x0F) as i32) << 16) | ((buf[6] as i32) << 8) | (buf[7] as i32);
        if self.coef.c10 > 524287 {
            self.coef.c10 -= 1048576;
        }

        // c01: 16 bits
        self.coef.c01 = i16::from_be_bytes([buf[8], buf[9]]);

        // c11: 16 bits
        self.coef.c11 = i16::from_be_bytes([buf[10], buf[11]]);

        // c20: 16 bits
        self.coef.c20 = i16::from_be_bytes([buf[12], buf[13]]);

        // c21: 16 bits
        self.coef.c21 = i16::from_be_bytes([buf[14], buf[15]]);

        // c30: 16 bits
        self.coef.c30 = i16::from_be_bytes([buf[16], buf[17]]);

        Ok(())
    }

    /// Set pressure measurement configuration
    pub fn set_pressure_config(
        &mut self,
        rate: MeasurementRate,
        osr: OversamplingRate,
    ) -> SensorResult<()> {
        let cfg = ((rate as u8) << 4) | (osr as u8);
        self.write_reg(reg::PRS_CFG, cfg)?;
        self.kp = osr.scale_factor();
        Ok(())
    }

    /// Set temperature measurement configuration
    pub fn set_temperature_config(
        &mut self,
        rate: MeasurementRate,
        osr: OversamplingRate,
    ) -> SensorResult<()> {
        // Bit 7 = 0 for internal sensor, 1 for external
        let cfg = ((rate as u8) << 4) | (osr as u8);
        self.write_reg(reg::TMP_CFG, cfg)?;
        self.kt = osr.scale_factor();
        Ok(())
    }

    /// Set measurement mode
    pub fn set_mode(&mut self, mode: MeasurementMode) -> SensorResult<()> {
        self.write_reg(reg::MEAS_CFG, mode as u8)?;
        Ok(())
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

    /// Read raw pressure value (24 bits, 2's complement)
    fn read_raw_pressure(&mut self) -> SensorResult<i32> {
        let b2 = self.read_reg(reg::PSR_B2)? as i32;
        let b1 = self.read_reg(reg::PSR_B1)? as i32;
        let b0 = self.read_reg(reg::PSR_B0)? as i32;

        let raw = (b2 << 16) | (b1 << 8) | b0;

        // Sign extend 24-bit to 32-bit
        if raw > 0x7FFFFF {
            Ok(raw - 0x1000000)
        } else {
            Ok(raw)
        }
    }

    /// Read raw temperature value (24 bits, 2's complement)
    fn read_raw_temperature(&mut self) -> SensorResult<i32> {
        let b2 = self.read_reg(reg::TMP_B2)? as i32;
        let b1 = self.read_reg(reg::TMP_B1)? as i32;
        let b0 = self.read_reg(reg::TMP_B0)? as i32;

        let raw = (b2 << 16) | (b1 << 8) | b0;

        // Sign extend 24-bit to 32-bit
        if raw > 0x7FFFFF {
            Ok(raw - 0x1000000)
        } else {
            Ok(raw)
        }
    }

    /// Calculate compensated pressure and temperature
    fn calculate_compensated(&mut self) -> SensorResult<(f32, f32)> {
        let psr_raw = self.read_raw_pressure()? as f32;
        let tmp_raw = self.read_raw_temperature()? as f32;

        // Scale raw values
        let psr_sc = psr_raw / self.kp;
        let tmp_sc = tmp_raw / self.kt;

        // Calculate temperature (°C)
        let temp_c = self.coef.c0 as f32 * 0.5 + self.coef.c1 as f32 * tmp_sc;

        // Calculate pressure (Pa)
        let pressure_pa = self.coef.c00 as f32
            + psr_sc
                * (self.coef.c10 as f32
                    + psr_sc * (self.coef.c20 as f32 + psr_sc * self.coef.c30 as f32))
            + tmp_sc * (self.coef.c01 as f32 + psr_sc * self.coef.c11 as f32)
            + tmp_sc * tmp_sc * self.coef.c21 as f32;

        Ok((pressure_pa, temp_c))
    }
}

impl<I2C> BaroDriver for Spl06<I2C>
where
    I2C: I2c,
{
    fn read(&mut self) -> SensorResult<RawBaroReading> {
        let (pressure_pa, temperature_c) = self.calculate_compensated()?;

        Ok(RawBaroReading {
            pressure_pa,
            temperature_c,
        })
    }

    fn data_ready(&mut self) -> SensorResult<bool> {
        let status = self.read_reg(reg::MEAS_CFG)?;
        // Bit 4: PRS_RDY, Bit 5: TMP_RDY
        Ok((status & 0x30) == 0x30)
    }

    fn source_id(&self) -> u8 {
        self.source_id
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_oversampling_scale_factors() {
        // Verify scale factors match datasheet
        assert!((OversamplingRate::X1.scale_factor() - 524288.0).abs() < 1.0);
        assert!((OversamplingRate::X16.scale_factor() - 253952.0).abs() < 1.0);
    }

    #[test]
    fn test_altitude_calculation() {
        // At sea level (101325 Pa), altitude should be ~0m
        let reading = RawBaroReading {
            pressure_pa: 101325.0,
            temperature_c: 15.0,
        };
        assert!(reading.altitude_m().abs() < 10.0);

        // At 1000m, pressure is approximately 89875 Pa
        let reading = RawBaroReading {
            pressure_pa: 89875.0,
            temperature_c: 15.0,
        };
        assert!((reading.altitude_m() - 1000.0).abs() < 100.0);
    }
}
