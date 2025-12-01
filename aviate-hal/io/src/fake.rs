//! Fake sensor and actuator drivers for SITL testing
//!
//! These drivers don't read from/write to real hardware - instead they exchange data
//! with external sources (e.g., MAVLink HIL messages from Gazebo).
//!
//! ## Usage
//!
//! ```ignore
//! // Create fake sensors
//! let mut imu = FakeImu::new();
//! let mut baro = FakeBaro::new();
//! let mut mag = FakeMag::new();
//! let mut gnss = FakeGnss::new();
//!
//! // Feed data from HIL messages (called by MAVLink handler)
//! imu.feed(RawImuReading {
//!     accel: [sensor.xacc, sensor.yacc, sensor.zacc],
//!     gyro: [sensor.xgyro, sensor.ygyro, sensor.zgyro],
//!     temperature: Some(sensor.temperature),
//! });
//!
//! // SensorBridge reads from fake sensors (same interface as real hardware)
//! let sensors = SensorBridge::new(imu, baro, mag, gnss, time);
//! ```
//!
//! ## Fault Injection
//!
//! All fake sensors support fault injection for SITL testing:
//!
//! ```ignore
//! // Inject sensor fault
//! imu.inject_fault(SensorFault::HealthDegraded);
//!
//! // Now read() returns error mapping to degraded health
//! assert!(matches!(imu.read(), Err(SensorError::InvalidData)));
//!
//! // Clear faults
//! imu.clear_faults();
//! ```

use crate::error::{ActuatorResult, SensorError, SensorResult};

// ============================================================================
// Fault Injection Types
// ============================================================================

/// Sensor fault types for SITL testing
///
/// These faults are injected at the FakeDriver layer, independent of the
/// physics backend (Gazebo/Unity), enabling deterministic testing.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SensorFault {
    /// Sensor reports degraded health (e.g., noisy data, intermittent issues)
    /// Maps to SensorError::InvalidData → SensorHealth::Degraded
    HealthDegraded,

    /// Sensor has completely failed
    /// Maps to SensorError::DeviceNotFound → SensorHealth::Failed
    HealthFailed,

    /// Inject NaN values into sensor readings
    /// Reading returns Ok but contains NaN in data fields
    NaN,

    /// Sensor stops providing data for specified cycles
    /// After countdown expires, fault auto-clears
    Dropout {
        /// Number of read cycles to drop
        remaining_cycles: u32,
    },

    /// Add constant bias offset to readings (for IMU/Mag 3-axis sensors)
    BiasShift {
        /// Offset to add to each axis
        offset: [f32; 3],
    },

    /// Add constant bias to single-value readings (for Baro)
    BiasShiftScalar {
        /// Offset to add
        offset: f32,
    },
}
use crate::traits::{
    ActuatorDriver, ActuatorStatus, BaroDriver, GnssDriver, GnssFix, ImuDriver, MagDriver,
    RawActuatorCmd, RawBaroReading, RawGnssReading, RawImuReading, RawMagReading,
};

/// Fake IMU driver for SITL
///
/// Receives accelerometer and gyroscope data from external source
/// (e.g., HIL_SENSOR MAVLink message). Supports fault injection for testing.
#[derive(Debug, Default)]
pub struct FakeImu {
    /// Buffered reading (None if no data available)
    reading: Option<RawImuReading>,
    /// Source ID for this sensor
    source_id: u8,
    /// Active fault (if any)
    fault: Option<SensorFault>,
}

impl FakeImu {
    /// Create a new fake IMU
    pub fn new() -> Self {
        Self::default()
    }

    /// Create with specific source ID
    pub fn with_source_id(source_id: u8) -> Self {
        Self {
            reading: None,
            source_id,
            fault: None,
        }
    }

    /// Feed new IMU data (called by MAVLink handler)
    pub fn feed(&mut self, reading: RawImuReading) {
        self.reading = Some(reading);
    }

    /// Clear buffered data
    pub fn clear(&mut self) {
        self.reading = None;
    }

    /// Check if data is available
    pub fn has_data(&self) -> bool {
        self.reading.is_some()
    }

    /// Inject a sensor fault
    pub fn inject_fault(&mut self, fault: SensorFault) {
        self.fault = Some(fault);
    }

    /// Clear all injected faults
    pub fn clear_faults(&mut self) {
        self.fault = None;
    }

    /// Check if a fault is active
    pub fn has_fault(&self) -> bool {
        self.fault.is_some()
    }

    /// Get the active fault (if any)
    pub fn get_fault(&self) -> Option<&SensorFault> {
        self.fault.as_ref()
    }
}

impl ImuDriver for FakeImu {
    fn read(&mut self) -> SensorResult<RawImuReading> {
        // Handle faults first
        if let Some(ref mut fault) = self.fault {
            match fault {
                SensorFault::HealthDegraded => {
                    return Err(SensorError::InvalidData);
                }
                SensorFault::HealthFailed => {
                    return Err(SensorError::DeviceNotFound);
                }
                SensorFault::NaN => {
                    // Return reading with NaN values
                    self.reading.take();
                    return Ok(RawImuReading {
                        accel: [f32::NAN, f32::NAN, f32::NAN],
                        gyro: [f32::NAN, f32::NAN, f32::NAN],
                        temperature: Some(f32::NAN),
                    });
                }
                SensorFault::Dropout { remaining_cycles } => {
                    if *remaining_cycles > 0 {
                        *remaining_cycles -= 1;
                        return Err(SensorError::InvalidState);
                    } else {
                        // Dropout expired, clear fault
                        self.fault = None;
                    }
                }
                SensorFault::BiasShift { offset } => {
                    // Apply bias to reading if available
                    if let Some(mut reading) = self.reading.take() {
                        reading.accel[0] += offset[0];
                        reading.accel[1] += offset[1];
                        reading.accel[2] += offset[2];
                        return Ok(reading);
                    }
                    return Err(SensorError::InvalidState);
                }
                SensorFault::BiasShiftScalar { .. } => {
                    // Not applicable to IMU, ignore
                }
            }
        }

        self.reading.take().ok_or(SensorError::InvalidState)
    }

    fn data_ready(&mut self) -> SensorResult<bool> {
        // If health faults are active, still report data ready
        // (the error will come from read())
        if let Some(fault) = &self.fault {
            match fault {
                SensorFault::HealthDegraded | SensorFault::HealthFailed | SensorFault::NaN => {
                    return Ok(true);
                }
                SensorFault::Dropout { remaining_cycles } if *remaining_cycles > 0 => {
                    return Ok(false);
                }
                _ => {}
            }
        }
        Ok(self.reading.is_some())
    }

    fn source_id(&self) -> u8 {
        self.source_id
    }
}

/// Fake barometer driver for SITL
///
/// Receives pressure and temperature data from external source
/// (e.g., HIL_SENSOR MAVLink message). Supports fault injection for testing.
#[derive(Debug, Default)]
pub struct FakeBaro {
    /// Buffered reading
    reading: Option<RawBaroReading>,
    /// Source ID
    source_id: u8,
    /// Active fault (if any)
    fault: Option<SensorFault>,
}

impl FakeBaro {
    /// Create a new fake barometer
    pub fn new() -> Self {
        Self::default()
    }

    /// Create with specific source ID
    pub fn with_source_id(source_id: u8) -> Self {
        Self {
            reading: None,
            source_id,
            fault: None,
        }
    }

    /// Feed new barometer data
    pub fn feed(&mut self, reading: RawBaroReading) {
        self.reading = Some(reading);
    }

    /// Clear buffered data
    pub fn clear(&mut self) {
        self.reading = None;
    }

    /// Check if data is available
    pub fn has_data(&self) -> bool {
        self.reading.is_some()
    }

    /// Inject a sensor fault
    pub fn inject_fault(&mut self, fault: SensorFault) {
        self.fault = Some(fault);
    }

    /// Clear all injected faults
    pub fn clear_faults(&mut self) {
        self.fault = None;
    }

    /// Check if a fault is active
    pub fn has_fault(&self) -> bool {
        self.fault.is_some()
    }
}

impl BaroDriver for FakeBaro {
    fn read(&mut self) -> SensorResult<RawBaroReading> {
        // Handle faults first
        if let Some(ref mut fault) = self.fault {
            match fault {
                SensorFault::HealthDegraded => {
                    return Err(SensorError::InvalidData);
                }
                SensorFault::HealthFailed => {
                    return Err(SensorError::DeviceNotFound);
                }
                SensorFault::NaN => {
                    self.reading.take();
                    return Ok(RawBaroReading {
                        pressure_pa: f32::NAN,
                        temperature_c: f32::NAN,
                    });
                }
                SensorFault::Dropout { remaining_cycles } => {
                    if *remaining_cycles > 0 {
                        *remaining_cycles -= 1;
                        return Err(SensorError::InvalidState);
                    } else {
                        self.fault = None;
                    }
                }
                SensorFault::BiasShiftScalar { offset } => {
                    if let Some(mut reading) = self.reading.take() {
                        reading.pressure_pa += *offset;
                        return Ok(reading);
                    }
                    return Err(SensorError::InvalidState);
                }
                SensorFault::BiasShift { .. } => {
                    // Not applicable to Baro, ignore
                }
            }
        }

        self.reading.take().ok_or(SensorError::InvalidState)
    }

    fn data_ready(&mut self) -> SensorResult<bool> {
        if let Some(fault) = &self.fault {
            match fault {
                SensorFault::HealthDegraded | SensorFault::HealthFailed | SensorFault::NaN => {
                    return Ok(true);
                }
                SensorFault::Dropout { remaining_cycles } if *remaining_cycles > 0 => {
                    return Ok(false);
                }
                _ => {}
            }
        }
        Ok(self.reading.is_some())
    }

    fn source_id(&self) -> u8 {
        self.source_id
    }
}

/// Fake magnetometer driver for SITL
///
/// Receives magnetic field data from external source
/// (e.g., HIL_SENSOR MAVLink message). Supports fault injection for testing.
#[derive(Debug, Default)]
pub struct FakeMag {
    /// Buffered reading
    reading: Option<RawMagReading>,
    /// Source ID
    source_id: u8,
    /// Active fault (if any)
    fault: Option<SensorFault>,
}

impl FakeMag {
    /// Create a new fake magnetometer
    pub fn new() -> Self {
        Self::default()
    }

    /// Create with specific source ID
    pub fn with_source_id(source_id: u8) -> Self {
        Self {
            reading: None,
            source_id,
            fault: None,
        }
    }

    /// Feed new magnetometer data
    pub fn feed(&mut self, reading: RawMagReading) {
        self.reading = Some(reading);
    }

    /// Clear buffered data
    pub fn clear(&mut self) {
        self.reading = None;
    }

    /// Check if data is available
    pub fn has_data(&self) -> bool {
        self.reading.is_some()
    }

    /// Inject a sensor fault
    pub fn inject_fault(&mut self, fault: SensorFault) {
        self.fault = Some(fault);
    }

    /// Clear all injected faults
    pub fn clear_faults(&mut self) {
        self.fault = None;
    }

    /// Check if a fault is active
    pub fn has_fault(&self) -> bool {
        self.fault.is_some()
    }
}

impl MagDriver for FakeMag {
    fn read(&mut self) -> SensorResult<RawMagReading> {
        // Handle faults first
        if let Some(ref mut fault) = self.fault {
            match fault {
                SensorFault::HealthDegraded => {
                    return Err(SensorError::InvalidData);
                }
                SensorFault::HealthFailed => {
                    return Err(SensorError::DeviceNotFound);
                }
                SensorFault::NaN => {
                    self.reading.take();
                    return Ok(RawMagReading {
                        field_ut: [f32::NAN, f32::NAN, f32::NAN],
                    });
                }
                SensorFault::Dropout { remaining_cycles } => {
                    if *remaining_cycles > 0 {
                        *remaining_cycles -= 1;
                        return Err(SensorError::InvalidState);
                    } else {
                        self.fault = None;
                    }
                }
                SensorFault::BiasShift { offset } => {
                    if let Some(mut reading) = self.reading.take() {
                        reading.field_ut[0] += offset[0];
                        reading.field_ut[1] += offset[1];
                        reading.field_ut[2] += offset[2];
                        return Ok(reading);
                    }
                    return Err(SensorError::InvalidState);
                }
                SensorFault::BiasShiftScalar { .. } => {
                    // Not applicable to Mag, ignore
                }
            }
        }

        self.reading.take().ok_or(SensorError::InvalidState)
    }

    fn data_ready(&mut self) -> SensorResult<bool> {
        if let Some(fault) = &self.fault {
            match fault {
                SensorFault::HealthDegraded | SensorFault::HealthFailed | SensorFault::NaN => {
                    return Ok(true);
                }
                SensorFault::Dropout { remaining_cycles } if *remaining_cycles > 0 => {
                    return Ok(false);
                }
                _ => {}
            }
        }
        Ok(self.reading.is_some())
    }

    fn source_id(&self) -> u8 {
        self.source_id
    }
}

/// Fake GNSS driver for SITL
///
/// Receives position and velocity data from external source
/// (e.g., HIL_GPS MAVLink message). Supports fault injection for testing.
#[derive(Debug, Default)]
pub struct FakeGnss {
    /// Buffered reading
    reading: Option<RawGnssReading>,
    /// Source ID
    source_id: u8,
    /// Active fault (if any)
    fault: Option<SensorFault>,
}

impl FakeGnss {
    /// Create a new fake GNSS
    pub fn new() -> Self {
        Self::default()
    }

    /// Create with specific source ID
    pub fn with_source_id(source_id: u8) -> Self {
        Self {
            reading: None,
            source_id,
            fault: None,
        }
    }

    /// Feed new GNSS data
    pub fn feed(&mut self, reading: RawGnssReading) {
        self.reading = Some(reading);
    }

    /// Clear buffered data
    pub fn clear(&mut self) {
        self.reading = None;
    }

    /// Check if data is available
    pub fn has_data(&self) -> bool {
        self.reading.is_some()
    }

    /// Inject a sensor fault
    pub fn inject_fault(&mut self, fault: SensorFault) {
        self.fault = Some(fault);
    }

    /// Clear all injected faults
    pub fn clear_faults(&mut self) {
        self.fault = None;
    }

    /// Check if a fault is active
    pub fn has_fault(&self) -> bool {
        self.fault.is_some()
    }
}

impl GnssDriver for FakeGnss {
    fn read(&mut self) -> SensorResult<RawGnssReading> {
        // Handle faults first
        if let Some(ref mut fault) = self.fault {
            match fault {
                SensorFault::HealthDegraded => {
                    return Err(SensorError::InvalidData);
                }
                SensorFault::HealthFailed => {
                    return Err(SensorError::DeviceNotFound);
                }
                SensorFault::NaN => {
                    self.reading.take();
                    return Ok(RawGnssReading {
                        lat_deg: f64::NAN,
                        lon_deg: f64::NAN,
                        alt_m: f32::NAN,
                        vel_ned: [f32::NAN, f32::NAN, f32::NAN],
                        fix: GnssFix::None,
                        h_acc: f32::NAN,
                        v_acc: f32::NAN,
                        satellites: 0,
                    });
                }
                SensorFault::Dropout { remaining_cycles } => {
                    if *remaining_cycles > 0 {
                        *remaining_cycles -= 1;
                        return Err(SensorError::InvalidState);
                    } else {
                        self.fault = None;
                    }
                }
                SensorFault::BiasShift { offset } => {
                    // For GNSS, bias shift applies to position (NED offset in meters)
                    // This is a simplified model - real GNSS errors are more complex
                    if let Some(mut reading) = self.reading.take() {
                        // Approximate: 1 degree lat ≈ 111km, so offset[0] meters ≈ offset[0]/111000 degrees
                        reading.lat_deg += (offset[0] as f64) / 111000.0;
                        reading.lon_deg += (offset[1] as f64) / 111000.0;
                        reading.alt_m += offset[2];
                        return Ok(reading);
                    }
                    return Err(SensorError::InvalidState);
                }
                SensorFault::BiasShiftScalar { .. } => {
                    // Not directly applicable to GNSS, ignore
                }
            }
        }

        self.reading.take().ok_or(SensorError::InvalidState)
    }

    fn data_ready(&mut self) -> SensorResult<bool> {
        if let Some(fault) = &self.fault {
            match fault {
                SensorFault::HealthDegraded | SensorFault::HealthFailed | SensorFault::NaN => {
                    return Ok(true);
                }
                SensorFault::Dropout { remaining_cycles } if *remaining_cycles > 0 => {
                    return Ok(false);
                }
                _ => {}
            }
        }
        Ok(self.reading.is_some())
    }

    fn source_id(&self) -> u8 {
        self.source_id
    }
}

/// Fake actuator driver for SITL
///
/// Buffers actuator commands and telemetry for bidirectional simulation.
///
/// ## Data Flow
///
/// **Commands (FC → Simulator):**
/// ```text
/// BoardHal.write(&cmd) → FakeActuator.write() → transport.send_actuator()
/// ```
///
/// **Telemetry (Simulator → FC):**
/// ```text
/// transport.take_actuator_telemetry() → FakeActuator.feed_status() → BoardHal.read_actuator_status()
/// ```
///
/// ## Telemetry Support
///
/// Unlike simple PWM drivers, FakeActuator supports telemetry because simulators
/// like Gazebo can report motor RPM, which is useful for:
/// - Testing EKF motor-based velocity estimation
/// - Validating motor health monitoring logic
/// - Simulating ESC telemetry scenarios
#[derive(Debug)]
pub struct FakeActuator {
    /// Buffered command (None if no new command)
    cmd: Option<RawActuatorCmd>,
    /// Buffered telemetry from simulator (None if no new data)
    status: Option<ActuatorStatus>,
    /// Armed state
    armed: bool,
}

impl Default for FakeActuator {
    fn default() -> Self {
        Self::new()
    }
}

impl FakeActuator {
    /// Create a new fake actuator driver
    pub fn new() -> Self {
        Self {
            cmd: None,
            status: None,
            armed: false,
        }
    }

    /// Take the buffered command (called by transport layer)
    ///
    /// Returns the last command written, or None if no new command.
    /// After calling, the buffer is cleared.
    pub fn take_cmd(&mut self) -> Option<RawActuatorCmd> {
        self.cmd.take()
    }

    /// Check if a command is buffered
    pub fn has_cmd(&self) -> bool {
        self.cmd.is_some()
    }

    /// Peek at the buffered command without taking it
    pub fn peek_cmd(&self) -> Option<&RawActuatorCmd> {
        self.cmd.as_ref()
    }

    /// Feed actuator telemetry from simulator
    ///
    /// Called by the transport layer when telemetry is received from the simulator.
    /// The kernel can then read this via `BoardHal.actuator().read_status()`.
    pub fn feed_status(&mut self, status: ActuatorStatus) {
        self.status = Some(status);
    }

    /// Check if telemetry is available
    pub fn has_status(&self) -> bool {
        self.status.is_some()
    }

    /// Clear telemetry buffer
    pub fn clear_status(&mut self) {
        self.status = None;
    }
}

impl ActuatorDriver for FakeActuator {
    fn write(&mut self, cmd: &RawActuatorCmd) -> ActuatorResult<()> {
        self.cmd = Some(*cmd);
        Ok(())
    }

    fn read_status(&mut self) -> Option<ActuatorStatus> {
        self.status.take()
    }

    fn status_ready(&mut self) -> bool {
        self.status.is_some()
    }

    fn arm(&mut self) {
        self.armed = true;
    }

    fn disarm(&mut self) {
        self.armed = false;
        // Clear any pending command on disarm
        self.cmd = None;
    }

    fn is_armed(&self) -> bool {
        self.armed
    }
}

/// Combined fake sensor set for SITL
///
/// Convenience struct holding all fake sensors with helper methods
/// for feeding data from HIL messages.
#[derive(Debug, Default)]
pub struct FakeSensorSet {
    pub imu: FakeImu,
    pub baro: FakeBaro,
    pub mag: FakeMag,
    pub gnss: FakeGnss,
}

impl FakeSensorSet {
    /// Create a new fake sensor set
    pub fn new() -> Self {
        Self::default()
    }

    /// Feed IMU data from HIL_SENSOR message
    ///
    /// Converts HIL_SENSOR fields to RawImuReading
    #[allow(clippy::too_many_arguments)]
    pub fn feed_hil_sensor_imu(
        &mut self,
        xacc: f32,
        yacc: f32,
        zacc: f32,
        xgyro: f32,
        ygyro: f32,
        zgyro: f32,
        temperature: f32,
    ) {
        self.imu.feed(RawImuReading {
            accel: [xacc, yacc, zacc],
            gyro: [xgyro, ygyro, zgyro],
            temperature: Some(temperature),
        });
    }

    /// Feed barometer data from HIL_SENSOR message
    ///
    /// Converts HIL_SENSOR pressure (mbar) to Pascals
    pub fn feed_hil_sensor_baro(&mut self, abs_pressure_mbar: f32, temperature: f32) {
        self.baro.feed(RawBaroReading {
            pressure_pa: abs_pressure_mbar * 100.0, // mbar to Pa
            temperature_c: temperature,
        });
    }

    /// Feed magnetometer data from HIL_SENSOR message
    ///
    /// Converts HIL_SENSOR mag (Gauss) to microtesla
    pub fn feed_hil_sensor_mag(&mut self, xmag: f32, ymag: f32, zmag: f32) {
        self.mag.feed(RawMagReading {
            field_ut: [xmag * 100.0, ymag * 100.0, zmag * 100.0], // Gauss to µT
        });
    }

    /// Feed GNSS data from HIL_GPS message
    #[allow(clippy::too_many_arguments)]
    pub fn feed_hil_gps(
        &mut self,
        lat: i32, // degE7
        lon: i32, // degE7
        alt: i32, // mm
        vn: i16,  // cm/s
        ve: i16,  // cm/s
        vd: i16,  // cm/s
        fix_type: u8,
        satellites: u8,
        eph: u16, // cm
        epv: u16, // cm
    ) {
        let fix = match fix_type {
            0 | 1 => GnssFix::None,
            2 => GnssFix::TwoD,
            3 | 4 => GnssFix::ThreeD,
            5 => GnssFix::RtkFloat,
            6 => GnssFix::RtkFixed,
            _ => GnssFix::None,
        };

        self.gnss.feed(RawGnssReading {
            lat_deg: (lat as f64) / 1e7,
            lon_deg: (lon as f64) / 1e7,
            alt_m: (alt as f32) / 1000.0,
            vel_ned: [
                (vn as f32) / 100.0,
                (ve as f32) / 100.0,
                (vd as f32) / 100.0,
            ],
            fix,
            h_acc: (eph as f32) / 100.0,
            v_acc: (epv as f32) / 100.0,
            satellites,
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_fake_imu_no_data_initially() {
        let mut imu = FakeImu::new();
        assert!(!imu.has_data());
        assert!(matches!(imu.data_ready(), Ok(false)));
        assert!(imu.read().is_err());
    }

    #[test]
    fn test_fake_imu_feed_and_read() {
        let mut imu = FakeImu::new();

        imu.feed(RawImuReading {
            accel: [1.0, 2.0, 3.0],
            gyro: [0.1, 0.2, 0.3],
            temperature: Some(25.0),
        });

        assert!(imu.has_data());
        assert!(matches!(imu.data_ready(), Ok(true)));

        let reading = imu.read().unwrap();
        assert_eq!(reading.accel, [1.0, 2.0, 3.0]);
        assert_eq!(reading.gyro, [0.1, 0.2, 0.3]);

        // After read, data should be consumed
        assert!(!imu.has_data());
    }

    #[test]
    fn test_fake_sensor_set_hil_sensor() {
        let mut sensors = FakeSensorSet::new();

        // Feed HIL_SENSOR data
        sensors.feed_hil_sensor_imu(0.0, 0.0, -9.81, 0.0, 0.0, 0.0, 25.0);
        sensors.feed_hil_sensor_baro(1013.25, 15.0);
        sensors.feed_hil_sensor_mag(0.2, 0.0, 0.4);

        // Read back
        let imu = sensors.imu.read().unwrap();
        assert!((imu.accel[2] - (-9.81)).abs() < 0.01);

        let baro = sensors.baro.read().unwrap();
        assert!((baro.pressure_pa - 101325.0).abs() < 1.0);

        let mag = sensors.mag.read().unwrap();
        assert!((mag.field_ut[0] - 20.0).abs() < 0.1); // 0.2 Gauss = 20 µT
    }

    #[test]
    fn test_fake_sensor_set_hil_gps() {
        let mut sensors = FakeSensorSet::new();

        // Feed HIL_GPS data (lat=47.3977°, lon=8.5456°, alt=500m)
        sensors.feed_hil_gps(
            473977000, // lat degE7
            85456000,  // lon degE7
            500000,    // alt mm
            100,       // vn cm/s = 1 m/s
            200,       // ve cm/s = 2 m/s
            -50,       // vd cm/s = -0.5 m/s
            3,         // 3D fix
            12,        // satellites
            100,       // eph cm
            150,       // epv cm
        );

        let gnss = sensors.gnss.read().unwrap();
        assert!((gnss.lat_deg - 47.3977).abs() < 0.0001);
        assert!((gnss.lon_deg - 8.5456).abs() < 0.0001);
        assert!((gnss.alt_m - 500.0).abs() < 0.1);
        assert_eq!(gnss.fix, GnssFix::ThreeD);
        assert_eq!(gnss.satellites, 12);
    }

    #[test]
    fn test_fake_actuator_no_cmd_initially() {
        let mut actuator = FakeActuator::new();
        assert!(!actuator.has_cmd());
        assert!(actuator.take_cmd().is_none());
        assert!(!actuator.is_armed());
    }

    #[test]
    fn test_fake_actuator_write_and_take() {
        let mut actuator = FakeActuator::new();

        let cmd = RawActuatorCmd {
            outputs: [0.5; 16],
            count: 4,
        };

        actuator.write(&cmd).unwrap();
        assert!(actuator.has_cmd());

        let taken = actuator.take_cmd().unwrap();
        assert_eq!(taken.outputs[0], 0.5);
        assert_eq!(taken.count, 4);

        // After take, buffer should be empty
        assert!(!actuator.has_cmd());
    }

    #[test]
    fn test_fake_actuator_arm_disarm() {
        let mut actuator = FakeActuator::new();

        // Initially disarmed
        assert!(!actuator.is_armed());

        // Arm
        actuator.arm();
        assert!(actuator.is_armed());

        // Write a command
        let cmd = RawActuatorCmd {
            outputs: [0.5; 16],
            count: 4,
        };
        actuator.write(&cmd).unwrap();
        assert!(actuator.has_cmd());

        // Disarm should clear buffered command
        actuator.disarm();
        assert!(!actuator.is_armed());
        assert!(!actuator.has_cmd());
    }

    // =========================================================================
    // FAULT INJECTION TESTS
    // =========================================================================

    #[test]
    fn test_imu_fault_health_degraded() {
        let mut imu = FakeImu::new();
        imu.feed(RawImuReading {
            accel: [0.0, 0.0, -9.81],
            gyro: [0.0, 0.0, 0.0],
            temperature: Some(25.0),
        });

        // Inject degraded health fault
        imu.inject_fault(SensorFault::HealthDegraded);
        assert!(imu.has_fault());
        assert!(matches!(imu.data_ready(), Ok(true)));

        // read() should return InvalidData error
        assert!(matches!(imu.read(), Err(SensorError::InvalidData)));

        // Clear fault
        imu.clear_faults();
        assert!(!imu.has_fault());
    }

    #[test]
    fn test_imu_fault_health_failed() {
        let mut imu = FakeImu::new();
        imu.inject_fault(SensorFault::HealthFailed);

        // read() should return DeviceNotFound error
        assert!(matches!(imu.read(), Err(SensorError::DeviceNotFound)));
    }

    #[test]
    fn test_imu_fault_nan_injection() {
        let mut imu = FakeImu::new();
        imu.feed(RawImuReading {
            accel: [0.0, 0.0, -9.81],
            gyro: [0.0, 0.0, 0.0],
            temperature: Some(25.0),
        });

        imu.inject_fault(SensorFault::NaN);

        let reading = imu.read().expect("Should return Ok with NaN values");
        assert!(reading.accel[0].is_nan());
        assert!(reading.accel[1].is_nan());
        assert!(reading.accel[2].is_nan());
        assert!(reading.gyro[0].is_nan());
    }

    #[test]
    fn test_imu_fault_dropout() {
        let mut imu = FakeImu::new();

        // Inject 3-cycle dropout
        // Behavior: remaining_cycles decrements on each read()
        // When remaining_cycles reaches 0, the fault clears on the NEXT read attempt
        imu.inject_fault(SensorFault::Dropout {
            remaining_cycles: 3,
        });

        // First 3 reads should fail (remaining goes 3→2→1→0)
        for i in 0..3 {
            imu.feed(RawImuReading {
                accel: [0.0, 0.0, -9.81],
                gyro: [0.0, 0.0, 0.0],
                temperature: Some(25.0),
            });
            assert!(
                matches!(imu.read(), Err(SensorError::InvalidState)),
                "Cycle {} should fail",
                i
            );
        }

        // Fault is still present but remaining_cycles = 0
        // Next read will clear it and proceed normally
        imu.feed(RawImuReading {
            accel: [0.0, 0.0, -9.81],
            gyro: [0.0, 0.0, 0.0],
            temperature: Some(25.0),
        });
        assert!(imu.read().is_ok());

        // Now fault should be cleared
        assert!(!imu.has_fault());
    }

    #[test]
    fn test_imu_fault_bias_shift() {
        let mut imu = FakeImu::new();
        imu.feed(RawImuReading {
            accel: [0.0, 0.0, -9.81],
            gyro: [0.0, 0.0, 0.0],
            temperature: Some(25.0),
        });

        // Inject bias: add 1.0 to X, 2.0 to Y, 3.0 to Z
        imu.inject_fault(SensorFault::BiasShift {
            offset: [1.0, 2.0, 3.0],
        });

        let reading = imu.read().expect("Should return biased reading");
        assert!((reading.accel[0] - 1.0).abs() < 1e-5);
        assert!((reading.accel[1] - 2.0).abs() < 1e-5);
        assert!((reading.accel[2] - (-9.81 + 3.0)).abs() < 1e-5);
    }

    #[test]
    fn test_baro_fault_bias_shift_scalar() {
        let mut baro = FakeBaro::new();
        baro.feed(RawBaroReading {
            pressure_pa: 101325.0,
            temperature_c: 25.0,
        });

        // Inject bias: add 1000 Pa
        baro.inject_fault(SensorFault::BiasShiftScalar { offset: 1000.0 });

        let reading = baro.read().expect("Should return biased reading");
        assert!((reading.pressure_pa - 102325.0).abs() < 1e-5);
    }

    #[test]
    fn test_mag_fault_health_degraded() {
        let mut mag = FakeMag::new();
        mag.inject_fault(SensorFault::HealthDegraded);

        assert!(matches!(mag.read(), Err(SensorError::InvalidData)));
    }

    #[test]
    fn test_gnss_fault_dropout_recovery() {
        let mut gnss = FakeGnss::new();

        // Inject 2-cycle dropout
        gnss.inject_fault(SensorFault::Dropout {
            remaining_cycles: 2,
        });

        // During dropout, data_ready should return false
        assert!(matches!(gnss.data_ready(), Ok(false)));

        // First 2 reads fail (remaining goes 2→1→0)
        assert!(matches!(gnss.read(), Err(SensorError::InvalidState)));
        assert!(matches!(gnss.read(), Err(SensorError::InvalidState)));

        // Fault is still present but remaining = 0
        // Normal operation resumes on next read (which clears the fault)
        gnss.feed(RawGnssReading {
            lat_deg: 47.0,
            lon_deg: 8.0,
            alt_m: 500.0,
            vel_ned: [0.0, 0.0, 0.0],
            fix: GnssFix::ThreeD,
            h_acc: 1.0,
            v_acc: 1.5,
            satellites: 10,
        });
        assert!(gnss.read().is_ok());

        // Now fault should be cleared
        assert!(!gnss.has_fault());
    }

    #[test]
    fn test_gnss_fault_nan_injection() {
        let mut gnss = FakeGnss::new();
        gnss.inject_fault(SensorFault::NaN);

        let reading = gnss.read().expect("Should return Ok with NaN values");
        assert!(reading.lat_deg.is_nan());
        assert!(reading.lon_deg.is_nan());
        assert!(reading.alt_m.is_nan());
        assert_eq!(reading.fix, GnssFix::None);
    }
}
