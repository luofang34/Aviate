//! Board HAL conversion and actuator tests.

use super::*;
use crate::error::ActuatorResult;
use crate::fake::{FakeActuator, FakeBaro, FakeGnss, FakeImu, FakeMag};
use crate::traits::{RawBaroReading, RawGnssReading, RawImuReading, RawMagReading};
use crate::SensorResult;

// Mock time source
#[derive(Clone)]
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

type FakeBoardHal = BoardHal<FakeImu, FakeBaro, FakeMag, FakeGnss, MockTime, FakeActuator>;

/// Read each sensor lane and return the values as bit patterns.
fn drain_lanes(hal: &mut FakeBoardHal) -> [Option<u32>; 4] {
    [
        hal.read_imu().map(|r| r.value.gyro[1].0.to_bits()),
        hal.read_baro()
            .and_then(|r| r.value.air.static_pressure)
            .map(|pressure| pressure.0.to_bits()),
        hal.read_mag().map(|r| r.value.field_ut[2].0.to_bits()),
        hal.read_gnss().map(|r| r.value.velocity_ned[0].0.to_bits()),
    ]
}

// A simulator makes a snapshot of the board with `Clone`. The clone must
// hold the buffered samples and the actuator state, and must not share
// them with the original.
#[test]
fn clone_of_fake_board_hal_is_an_independent_snapshot() {
    let mut hal: FakeBoardHal = BoardHal::new(
        FakeImu::new(),
        FakeBaro::new(),
        FakeMag::new(),
        FakeGnss::new(),
        MockTime(1000),
        FakeActuator::new(),
    );
    hal.imu_mut().feed(RawImuReading {
        accel: [0.1, 0.2, -9.81],
        gyro: [0.01, -0.02, 0.03],
        temperature: Some(25.0),
    });
    hal.baro_mut().feed(RawBaroReading {
        pressure_pa: 90_000.0,
        differential_pressure_pa: None,
        pressure_altitude_m: None,
        temperature_c: 20.0,
    });
    hal.mag_mut().feed(RawMagReading {
        field_ut: [20.0, 1.0, 40.0],
    });
    hal.gnss_mut().feed(RawGnssReading {
        vel_ned: [3.5, 0.0, 0.0],
        fix: GnssFix::ThreeD,
        ..Default::default()
    });
    hal.arm();
    let mut cmd = ActuatorCmd::default();
    cmd.outputs[0] = aviate_core::types::Normalized(0.75);
    hal.write(&cmd);

    let mut snapshot = hal.clone();
    assert!(snapshot.is_armed());
    assert_eq!(
        snapshot
            .actuator()
            .peek_cmd()
            .map(|c| c.outputs[0].to_bits()),
        hal.actuator().peek_cmd().map(|c| c.outputs[0].to_bits())
    );
    assert!(snapshot.actuator().has_cmd());

    // Reading takes the buffered sample, so the second read of the
    // original shows that the snapshot kept its own copy.
    let original = drain_lanes(&mut hal);
    assert!(original.iter().all(Option::is_some), "{original:?}");
    assert!(hal.read_imu().is_none());
    assert_eq!(drain_lanes(&mut snapshot), original);

    hal.disarm();
    assert!(snapshot.is_armed());

    #[cfg(feature = "xil-fault")]
    {
        hal.mag_mut().inject_fault(crate::fake::SensorFault::NaN);
        assert!(hal.clone().mag().has_fault());
        assert!(!snapshot.mag().has_fault());
    }
}
