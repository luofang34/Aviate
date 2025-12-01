//! Fake sensor drivers for SITL testing
//!
//! These drivers don't read from real hardware - instead they receive sensor data
//! from external sources (e.g., MAVLink HIL_SENSOR/HIL_GPS messages from Gazebo).
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

use crate::error::{SensorError, SensorResult};
use crate::traits::{
    BaroDriver, GnssDriver, GnssFix, ImuDriver, MagDriver, RawBaroReading, RawGnssReading,
    RawImuReading, RawMagReading,
};

/// Fake IMU driver for SITL
///
/// Receives accelerometer and gyroscope data from external source
/// (e.g., HIL_SENSOR MAVLink message)
#[derive(Debug, Default)]
pub struct FakeImu {
    /// Buffered reading (None if no data available)
    reading: Option<RawImuReading>,
    /// Source ID for this sensor
    source_id: u8,
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
}

impl ImuDriver for FakeImu {
    fn read(&mut self) -> SensorResult<RawImuReading> {
        self.reading.take().ok_or(SensorError::InvalidState)
    }

    fn data_ready(&mut self) -> SensorResult<bool> {
        Ok(self.reading.is_some())
    }

    fn source_id(&self) -> u8 {
        self.source_id
    }
}

/// Fake barometer driver for SITL
///
/// Receives pressure and temperature data from external source
/// (e.g., HIL_SENSOR MAVLink message)
#[derive(Debug, Default)]
pub struct FakeBaro {
    /// Buffered reading
    reading: Option<RawBaroReading>,
    /// Source ID
    source_id: u8,
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
}

impl BaroDriver for FakeBaro {
    fn read(&mut self) -> SensorResult<RawBaroReading> {
        self.reading.take().ok_or(SensorError::InvalidState)
    }

    fn data_ready(&mut self) -> SensorResult<bool> {
        Ok(self.reading.is_some())
    }

    fn source_id(&self) -> u8 {
        self.source_id
    }
}

/// Fake magnetometer driver for SITL
///
/// Receives magnetic field data from external source
/// (e.g., HIL_SENSOR MAVLink message)
#[derive(Debug, Default)]
pub struct FakeMag {
    /// Buffered reading
    reading: Option<RawMagReading>,
    /// Source ID
    source_id: u8,
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
}

impl MagDriver for FakeMag {
    fn read(&mut self) -> SensorResult<RawMagReading> {
        self.reading.take().ok_or(SensorError::InvalidState)
    }

    fn data_ready(&mut self) -> SensorResult<bool> {
        Ok(self.reading.is_some())
    }

    fn source_id(&self) -> u8 {
        self.source_id
    }
}

/// Fake GNSS driver for SITL
///
/// Receives position and velocity data from external source
/// (e.g., HIL_GPS MAVLink message)
#[derive(Debug, Default)]
pub struct FakeGnss {
    /// Buffered reading
    reading: Option<RawGnssReading>,
    /// Source ID
    source_id: u8,
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
}

impl GnssDriver for FakeGnss {
    fn read(&mut self) -> SensorResult<RawGnssReading> {
        self.reading.take().ok_or(SensorError::InvalidState)
    }

    fn data_ready(&mut self) -> SensorResult<bool> {
        Ok(self.reading.is_some())
    }

    fn source_id(&self) -> u8 {
        self.source_id
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
}
