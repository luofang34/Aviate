# aviate-hal-io

Platform-agnostic I/O device framework for Aviate flight systems.

## Quick Reference

| Component | Description |
|-----------|-------------|
| `BoardHal<I,B,M,G,T>` | Generic composition implementing `SensorHal` |
| `ImuDriver` | Trait for IMU sensors (accel + gyro) |
| `BaroDriver` | Trait for barometric pressure sensors |
| `MagDriver` | Trait for magnetometers |
| `GnssDriver` | Trait for GNSS receivers |
| `FakeImu/Baro/Mag/Gnss` | SITL drivers (fed from HIL messages) |

## Purpose

This crate abstracts sensor/actuator drivers behind traits, enabling the same `BoardHal`
to work with both real hardware and simulation:

```
Real HW:  SPI/I2C → Driver → BoardHal → SensorHal → Kernel
SITL:     MAVLink → FakeDriver → BoardHal → SensorHal → Kernel
```

**Platform-agnostic**: No MCU register operations here. This crate defines interfaces;
board-level driver instantiation happens in `aviate-boards/*`.

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│  aviate-core (SensorHal trait)                              │
└─────────────────────────────────────────────────────────────┘
                          ↑
             implements SensorHal
                          ↑
┌─────────────────────────────────────────────────────────────┐
│  BoardHal<I, B, M, G, T>                                    │
│  - Composes I/O drivers                                     │
│  - Converts raw readings to aviate-core types               │
│  - Handles timestamps and health                            │
└─────────────────────────────────────────────────────────────┘
                          ↑
             ImuDriver, BaroDriver, MagDriver, GnssDriver
                          ↑
┌───────────────────────────┬─────────────────────────────────┐
│  Real Hardware            │  SITL / Fake Sensors            │
│  - Icm426xx<I2C>          │  - FakeImu (from HIL_SENSOR)    │
│  - Bmp390<SPI>            │  - FakeBaro (from HIL_SENSOR)   │
│  - Qmc5883l<I2C>          │  - FakeMag (from HIL_SENSOR)    │
│  - UbloxGnss<UART>        │  - FakeGnss (from HIL_GPS)      │
└───────────────────────────┴─────────────────────────────────┘
```

## Key Components

### BoardHal

Generic composition of sensor drivers that implements `SensorHal`:

```rust
pub struct BoardHal<I, B, M, G, T> {
    imu: I,    // implements ImuDriver
    baro: B,   // implements BaroDriver
    mag: M,    // implements MagDriver
    gnss: G,   // implements GnssDriver
    time: T,   // implements TimeSource
}

impl<I: ImuDriver, B: BaroDriver, M: MagDriver, G: GnssDriver, T: TimeSource>
    SensorHal for BoardHal<I, B, M, G, T>
{
    fn read_imu(&mut self) -> Option<SensorReading<ImuData>> { ... }
    fn read_baro(&mut self) -> Option<SensorReading<BaroData>> { ... }
    fn read_mag(&mut self) -> Option<SensorReading<MagData>> { ... }
    fn read_gnss(&mut self) -> Option<SensorReading<GnssData>> { ... }
}
```

### Driver Traits

Each sensor type has a trait with raw SI-unit readings:

```rust
// IMU: accelerometer + gyroscope
pub trait ImuDriver {
    fn read(&mut self) -> SensorResult<RawImuReading>;
    fn data_ready(&mut self) -> SensorResult<bool>;
}

pub struct RawImuReading {
    pub accel: [f32; 3],  // m/s²
    pub gyro: [f32; 3],   // rad/s
    pub temperature: Option<f32>,  // °C
}

// Barometer
pub trait BaroDriver {
    fn read(&mut self) -> SensorResult<RawBaroReading>;
}

pub struct RawBaroReading {
    pub pressure_pa: f32,      // Pascals
    pub temperature_c: f32,    // °C
}

// Magnetometer
pub trait MagDriver {
    fn read(&mut self) -> SensorResult<RawMagReading>;
}

pub struct RawMagReading {
    pub field_ut: [f32; 3],   // microtesla
}

// GNSS
pub trait GnssDriver {
    fn read(&mut self) -> SensorResult<RawGnssReading>;
}

pub struct RawGnssReading {
    pub lat_deg: f64,
    pub lon_deg: f64,
    pub alt_m: f32,
    pub vel_ned: [f32; 3],    // m/s
    pub fix: GnssFix,
    pub h_acc: f32,           // meters
    pub v_acc: f32,           // meters
    pub satellites: u8,
}
```

### Fake Drivers (for SITL)

Fake drivers receive data from external sources (e.g., Gazebo via MAVLink):

```rust
// Create fake sensors
let mut imu = FakeImu::new();

// MAVLink handler feeds data
imu.feed(RawImuReading {
    accel: [sensor.xacc, sensor.yacc, sensor.zacc],
    gyro: [sensor.xgyro, sensor.ygyro, sensor.zgyro],
    temperature: Some(sensor.temperature),
});

// BoardHal reads (same interface as real hardware)
let reading = board_hal.read_imu();
```

## Adding a New Driver

To add support for a new sensor (e.g., BMI088 IMU):

```rust
use aviate_hal_io::{ImuDriver, RawImuReading, SensorResult};
use embedded_hal::i2c::I2c;

pub struct Bmi088<I2C> {
    bus: I2C,
    // ... configuration
}

impl<I2C: I2c> ImuDriver for Bmi088<I2C> {
    fn read(&mut self) -> SensorResult<RawImuReading> {
        // Read registers via I2C, convert to SI units
        let mut buf = [0u8; 12];
        self.bus.read(ACCEL_DATA_REG, &mut buf)?;

        Ok(RawImuReading {
            accel: [
                self.convert_accel(buf[0..2]),
                self.convert_accel(buf[2..4]),
                self.convert_accel(buf[4..6]),
            ],
            gyro: [
                self.convert_gyro(buf[6..8]),
                self.convert_gyro(buf[8..10]),
                self.convert_gyro(buf[10..12]),
            ],
            temperature: None,
        })
    }

    fn data_ready(&mut self) -> SensorResult<bool> {
        // Check interrupt or status register
        let status = self.read_status()?;
        Ok(status & DATA_READY_BIT != 0)
    }
}
```

## Usage Examples

### SITL Board

```rust
use aviate_hal_io::{BoardHal, FakeImu, FakeBaro, FakeMag, FakeGnss};

// Type alias for SITL
type SitlBoardHal = BoardHal<FakeImu, FakeBaro, FakeMag, FakeGnss, SitlTime>;

// Create board HAL
let board_hal = BoardHal::new(
    FakeImu::new(),
    FakeBaro::new(),
    FakeMag::new(),
    FakeGnss::new(),
    SitlTime::new(),
);

// In control loop:
// 1. MAVLink handler feeds data to fake sensors
board_hal.imu_mut().feed(sensor_data.imu);

// 2. Read via SensorHal interface
if let Some(imu) = board_hal.read_imu() {
    kernel.step(&imu);
}
```

### Real Hardware Board

```rust
use aviate_hal_io::BoardHal;
use icm426xx::Icm426xx;
use bmp3xx::Bmp3xx;

// Type alias for real hardware
type HwBoardHal = BoardHal<Icm426xx<I2C>, Bmp3xx<SPI>, Qmc5883l<I2C>, UbloxGnss<UART>, HwTime>;

// Create with real drivers
let board_hal = BoardHal::new(
    Icm426xx::new(i2c),
    Bmp3xx::new(spi),
    Qmc5883l::new(i2c2),
    UbloxGnss::new(uart),
    HwTime::new(),
);

// Same SensorHal interface - kernel code is unchanged!
if let Some(imu) = board_hal.read_imu() {
    kernel.step(&imu);
}
```

## Calibration

The crate provides calibration utilities:

```rust
use aviate_hal_io::{ImuCalibration, MagCalibration};

// IMU calibration (bias + scale)
let imu_cal = ImuCalibration {
    accel_bias: [0.01, -0.02, 0.03],  // m/s²
    accel_scale: [1.001, 0.999, 1.002],
    gyro_bias: [0.001, -0.002, 0.0005],  // rad/s
    gyro_scale: [1.0; 3],
};

let calibrated = imu_cal.apply(&raw_reading);

// Magnetometer calibration (hard/soft iron)
let mag_cal = MagCalibration {
    hard_iron: [5.0, -3.0, 2.0],  // µT
    soft_iron: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
};

let calibrated = mag_cal.apply(&raw_mag);
```

## Implementation Details

See [src/lib.rs](src/lib.rs) for the full API.
