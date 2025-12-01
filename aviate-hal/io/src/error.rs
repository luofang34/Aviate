//! Error types for embedded sensor operations

use aviate_core::sensor::SensorHealth;

/// Sensor error types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SensorError {
    /// I2C/SPI bus error (NACK, arbitration lost, etc.)
    BusError,
    /// Communication timeout
    Timeout,
    /// Device not responding or not found
    DeviceNotFound,
    /// Invalid data received (CRC error, out of range)
    InvalidData,
    /// Device in wrong state (not initialized, sleeping)
    InvalidState,
    /// Calibration data invalid or missing
    CalibrationError,
}

impl SensorError {
    /// Convert sensor error to health status
    pub fn to_health(self) -> SensorHealth {
        match self {
            SensorError::BusError | SensorError::DeviceNotFound => SensorHealth::Failed,
            SensorError::Timeout | SensorError::InvalidData => SensorHealth::Degraded,
            SensorError::InvalidState | SensorError::CalibrationError => SensorHealth::Degraded,
        }
    }
}

/// Result type for sensor operations
pub type SensorResult<T> = Result<T, SensorError>;

/// Actuator error types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ActuatorError {
    /// Hardware fault (overcurrent, overtemp, etc.)
    HardwareFault,
    /// Communication error with ESC/servo
    CommError,
    /// Output clamped (value out of range)
    OutputClamped,
    /// Not armed (outputs disabled)
    NotArmed,
}

/// Result type for actuator operations
pub type ActuatorResult<T> = Result<T, ActuatorError>;
