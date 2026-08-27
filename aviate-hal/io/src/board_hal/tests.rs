//! Board HAL conversion and actuator tests.

use super::*;
use crate::error::ActuatorResult;
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

// Mock actuator
struct MockActuator {
    armed: bool,
    last_cmd: Option<RawActuatorCmd>,
}

impl MockActuator {
    fn new() -> Self {
        Self {
            armed: false,
            last_cmd: None,
        }
    }
}

impl ActuatorDriver for MockActuator {
    fn write(&mut self, cmd: &RawActuatorCmd) -> ActuatorResult<()> {
        self.last_cmd = Some(*cmd);
        Ok(())
    }

    fn arm(&mut self) {
        self.armed = true;
    }

    fn disarm(&mut self) {
        self.armed = false;
    }

    fn is_armed(&self) -> bool {
        self.armed
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
    let actuator = MockActuator::new();

    let mut hal = BoardHal::new(imu, baro, mag, gnss, MockTime(1000), actuator);

    let reading = hal.read_imu();
    assert!(reading.is_some());
    let Some(reading) = reading else {
        return;
    };
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
    let actuator = MockActuator::new();

    let mut hal = BoardHal::new(imu, baro, mag, gnss, MockTime(1000), actuator);

    assert!(hal.read_imu().is_none());
}

#[test]
fn explicit_pressure_lanes_reach_the_sensor_set() {
    let imu = MockImu {
        reading: RawImuReading::default(),
        ready: false,
    };
    let baro = MockBaro {
        reading: RawBaroReading {
            pressure_pa: 90_000.0,
            differential_pressure_pa: Some(1_250.0),
            pressure_altitude_m: Some(321.0),
            temperature_c: 20.0,
        },
    };
    let mag = MockMag {
        reading: RawMagReading::default(),
    };
    let gnss = MockGnss {
        reading: RawGnssReading::default(),
    };
    let actuator = MockActuator::new();
    let mut hal = BoardHal::new(imu, baro, mag, gnss, MockTime(1000), actuator);

    let Some(reading) = hal.read_baro() else {
        return;
    };
    assert_eq!(reading.value.altitude.map(|value| value.0), Some(321.0));
    assert_eq!(
        reading.value.air.dynamic_pressure.map(|value| value.0),
        Some(1_250.0)
    );
    assert_eq!(
        reading.value.air.static_pressure.map(|value| value.0),
        Some(90_000.0)
    );
}

#[test]
fn test_board_hal_actuator() {
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
    let actuator = MockActuator::new();

    let mut hal = BoardHal::new(imu, baro, mag, gnss, MockTime(1000), actuator);

    // Test arm/disarm
    assert!(!hal.is_armed());
    hal.arm();
    assert!(hal.is_armed());
    hal.disarm();
    assert!(!hal.is_armed());
}
