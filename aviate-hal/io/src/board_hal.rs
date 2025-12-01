//! BoardHal - Composes I/O drivers into SensorHal
//!
//! The `BoardHal` takes individual sensor drivers (ImuDriver, BaroDriver, etc.)
//! and implements `SensorHal` from aviate-core. This allows the same flight controller
//! code to work with:
//!
//! - Real hardware sensors (ICM426xx, BMP390, etc. via embedded-hal)
//! - Simulated sensors (fake sensors fed from Gazebo HIL messages)
//!
//! ## Example
//!
//! ```ignore
//! // For real hardware (on STM32H7):
//! let imu = Icm426xx::new(i2c);
//! let baro = Bmp390::new(spi);
//! let mag = Qmc5883l::new(i2c);
//! let gnss = UbloxGnss::new(uart);
//! let hal = BoardHal::new(imu, baro, mag, gnss, time_source);
//!
//! // For SITL (fake sensors from Gazebo):
//! let imu = FakeImu::new();
//! let baro = FakeBaro::new();
//! let mag = FakeMag::new();
//! let gnss = FakeGnss::new();
//! let hal = BoardHal::new(imu, baro, mag, gnss, time_source);
//!
//! // Both use the same SensorHal interface
//! if let Some(imu_reading) = hal.read_imu() {
//!     // Process IMU data
//! }
//! ```

use aviate_core::hal::SensorHal;
use aviate_core::sensor::{
    AirData, BaroData, GnssData, GnssFix as CoreGnssFix, GnssHealth, ImuData, MagData,
    SensorHealth, SensorReading,
};
use aviate_core::time::{TimeSource as CoreTimeSource, Timestamp};
use aviate_core::types::{
    Celsius, Meters, MetersPerSecond, MetersPerSecondSquared, Microtesla, Pascals, RadiansPerSecond,
};

use crate::error::SensorError;
use crate::traits::{BaroDriver, GnssDriver, GnssFix, ImuDriver, MagDriver, TimeSource};

/// Board-level HAL that composes I/O drivers into SensorHal
///
/// Generic over:
/// - `I`: IMU driver implementing `ImuDriver`
/// - `B`: Barometer driver implementing `BaroDriver`
/// - `M`: Magnetometer driver implementing `MagDriver`
/// - `G`: GNSS driver implementing `GnssDriver`
/// - `T`: Time source implementing `TimeSource`
///
/// Future: Will also compose actuator drivers to implement full AviateHal
pub struct BoardHal<I, B, M, G, T> {
    imu: I,
    baro: B,
    mag: M,
    gnss: G,
    time: T,
}

impl<I, B, M, G, T> BoardHal<I, B, M, G, T>
where
    I: ImuDriver,
    B: BaroDriver,
    M: MagDriver,
    G: GnssDriver,
    T: TimeSource,
{
    /// Create a new board HAL with all sensors
    pub fn new(imu: I, baro: B, mag: M, gnss: G, time: T) -> Self {
        Self {
            imu,
            baro,
            mag,
            gnss,
            time,
        }
    }

    /// Get a reference to the IMU driver
    pub fn imu(&self) -> &I {
        &self.imu
    }

    /// Get a mutable reference to the IMU driver
    pub fn imu_mut(&mut self) -> &mut I {
        &mut self.imu
    }

    /// Get a reference to the barometer driver
    pub fn baro(&self) -> &B {
        &self.baro
    }

    /// Get a mutable reference to the barometer driver
    pub fn baro_mut(&mut self) -> &mut B {
        &mut self.baro
    }

    /// Get a reference to the magnetometer driver
    pub fn mag(&self) -> &M {
        &self.mag
    }

    /// Get a mutable reference to the magnetometer driver
    pub fn mag_mut(&mut self) -> &mut M {
        &mut self.mag
    }

    /// Get a reference to the GNSS driver
    pub fn gnss(&self) -> &G {
        &self.gnss
    }

    /// Get a mutable reference to the GNSS driver
    pub fn gnss_mut(&mut self) -> &mut G {
        &mut self.gnss
    }

    /// Get current timestamp
    fn timestamp(&self) -> Timestamp {
        Timestamp {
            ticks: self.time.now_us(),
            source: CoreTimeSource::Internal,
        }
    }

    /// Convert sensor error to health status
    fn error_to_health(err: SensorError) -> SensorHealth {
        err.to_health()
    }
}

impl<I, B, M, G, T> SensorHal for BoardHal<I, B, M, G, T>
where
    I: ImuDriver,
    B: BaroDriver,
    M: MagDriver,
    G: GnssDriver,
    T: TimeSource,
{
    fn read_imu(&mut self) -> Option<SensorReading<ImuData>> {
        // Check if data is ready first (for interrupt-driven operation)
        match self.imu.data_ready() {
            Ok(true) => {}
            Ok(false) => return None,
            Err(_) => return None,
        }

        let ts = self.timestamp();

        match self.imu.read() {
            Ok(raw) => Some(SensorReading {
                value: ImuData {
                    accel: [
                        MetersPerSecondSquared(raw.accel[0]),
                        MetersPerSecondSquared(raw.accel[1]),
                        MetersPerSecondSquared(raw.accel[2]),
                    ],
                    gyro: [
                        RadiansPerSecond(raw.gyro[0]),
                        RadiansPerSecond(raw.gyro[1]),
                        RadiansPerSecond(raw.gyro[2]),
                    ],
                },
                valid: true,
                source_id: self.imu.source_id(),
                timestamp: ts,
                health: SensorHealth::Good,
            }),
            Err(e) => Some(SensorReading {
                value: ImuData::default(),
                valid: false,
                source_id: self.imu.source_id(),
                timestamp: ts,
                health: Self::error_to_health(e),
            }),
        }
    }

    fn read_baro(&mut self) -> Option<SensorReading<BaroData>> {
        match self.baro.data_ready() {
            Ok(true) => {}
            Ok(false) => return None,
            Err(_) => return None,
        }

        let ts = self.timestamp();

        match self.baro.read() {
            Ok(raw) => Some(SensorReading {
                value: BaroData {
                    altitude: Some(Meters(raw.altitude_m())),
                    air: AirData {
                        static_pressure: Some(Pascals(raw.pressure_pa)),
                        dynamic_pressure: None,
                        total_pressure: None,
                        temperature: Some(Celsius(raw.temperature_c)),
                        indicated_airspeed: None,
                        true_airspeed: None,
                    },
                },
                valid: true,
                source_id: self.baro.source_id(),
                timestamp: ts,
                health: SensorHealth::Good,
            }),
            Err(e) => Some(SensorReading {
                value: BaroData::default(),
                valid: false,
                source_id: self.baro.source_id(),
                timestamp: ts,
                health: Self::error_to_health(e),
            }),
        }
    }

    fn read_mag(&mut self) -> Option<SensorReading<MagData>> {
        match self.mag.data_ready() {
            Ok(true) => {}
            Ok(false) => return None,
            Err(_) => return None,
        }

        let ts = self.timestamp();

        match self.mag.read() {
            Ok(raw) => Some(SensorReading {
                value: MagData {
                    field_ut: [
                        Microtesla(raw.field_ut[0]),
                        Microtesla(raw.field_ut[1]),
                        Microtesla(raw.field_ut[2]),
                    ],
                },
                valid: true,
                source_id: self.mag.source_id(),
                timestamp: ts,
                health: SensorHealth::Good,
            }),
            Err(e) => Some(SensorReading {
                value: MagData::default(),
                valid: false,
                source_id: self.mag.source_id(),
                timestamp: ts,
                health: Self::error_to_health(e),
            }),
        }
    }

    fn read_gnss(&mut self) -> Option<SensorReading<GnssData>> {
        match self.gnss.data_ready() {
            Ok(true) => {}
            Ok(false) => return None,
            Err(_) => return None,
        }

        let ts = self.timestamp();

        match self.gnss.read() {
            Ok(raw) => {
                let fix = match raw.fix {
                    GnssFix::None => CoreGnssFix::None,
                    GnssFix::TwoD => CoreGnssFix::TwoD,
                    GnssFix::ThreeD => CoreGnssFix::ThreeD,
                    GnssFix::RtkFloat => CoreGnssFix::RtkFloat,
                    GnssFix::RtkFixed => CoreGnssFix::RtkFixed,
                };

                let health = if raw.fix == GnssFix::None {
                    GnssHealth::Lost
                } else {
                    GnssHealth::Good
                };

                Some(SensorReading {
                    value: GnssData {
                        // Convert lat/lon to local NED (simplified - just use altitude for now)
                        position_ned: [Meters(0.0), Meters(0.0), Meters(-raw.alt_m)],
                        velocity_ned: [
                            MetersPerSecond(raw.vel_ned[0]),
                            MetersPerSecond(raw.vel_ned[1]),
                            MetersPerSecond(raw.vel_ned[2]),
                        ],
                        fix,
                        health,
                    },
                    valid: raw.fix != GnssFix::None,
                    source_id: self.gnss.source_id(),
                    timestamp: ts,
                    health: if health == GnssHealth::Good {
                        SensorHealth::Good
                    } else {
                        SensorHealth::Failed
                    },
                })
            }
            Err(e) => Some(SensorReading {
                value: GnssData::default(),
                valid: false,
                source_id: self.gnss.source_id(),
                timestamp: ts,
                health: Self::error_to_health(e),
            }),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::traits::{RawBaroReading, RawGnssReading, RawImuReading, RawMagReading};
    use crate::SensorResult;

    // Mock time source
    struct MockTime(u64);
    impl TimeSource for MockTime {
        fn now_us(&self) -> u64 {
            self.0
        }
    }

    // Mock IMU that returns fixed data
    struct MockImu {
        reading: RawImuReading,
        ready: bool,
    }

    impl ImuDriver for MockImu {
        fn read(&mut self) -> SensorResult<RawImuReading> {
            Ok(self.reading)
        }

        fn data_ready(&mut self) -> SensorResult<bool> {
            Ok(self.ready)
        }
    }

    // Mock baro
    struct MockBaro {
        reading: RawBaroReading,
    }

    impl BaroDriver for MockBaro {
        fn read(&mut self) -> SensorResult<RawBaroReading> {
            Ok(self.reading)
        }
    }

    // Mock mag
    struct MockMag {
        reading: RawMagReading,
    }

    impl MagDriver for MockMag {
        fn read(&mut self) -> SensorResult<RawMagReading> {
            Ok(self.reading)
        }
    }

    // Mock GNSS
    struct MockGnss {
        reading: RawGnssReading,
    }

    impl GnssDriver for MockGnss {
        fn read(&mut self) -> SensorResult<RawGnssReading> {
            Ok(self.reading)
        }
    }

    #[test]
    fn test_board_hal_reads_imu() {
        let imu = MockImu {
            reading: RawImuReading {
                accel: [0.0, 0.0, -9.81],
                gyro: [0.0, 0.0, 0.0],
                temperature: Some(25.0),
            },
            ready: true,
        };
        let baro = MockBaro {
            reading: RawBaroReading::default(),
        };
        let mag = MockMag {
            reading: RawMagReading::default(),
        };
        let gnss = MockGnss {
            reading: RawGnssReading::default(),
        };

        let mut hal = BoardHal::new(imu, baro, mag, gnss, MockTime(1000));

        let reading = hal.read_imu().unwrap();
        assert!(reading.valid);
        assert!((reading.value.accel[2].0 - (-9.81)).abs() < 0.01);
    }

    #[test]
    fn test_board_hal_no_data_when_not_ready() {
        let imu = MockImu {
            reading: RawImuReading::default(),
            ready: false,
        };
        let baro = MockBaro {
            reading: RawBaroReading::default(),
        };
        let mag = MockMag {
            reading: RawMagReading::default(),
        };
        let gnss = MockGnss {
            reading: RawGnssReading::default(),
        };

        let mut hal = BoardHal::new(imu, baro, mag, gnss, MockTime(1000));

        assert!(hal.read_imu().is_none());
    }
}
