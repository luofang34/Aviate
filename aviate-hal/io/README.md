# aviate-hal-io

Platform-agnostic I/O device framework for Aviate flight systems.

## Quick Reference

| Component | Description |
|-----------|-------------|
| `BoardHal<I,B,M,G,T,A>` | Generic composition implementing `SensorHal` + `ActuatorHal` |
| `ImuDriver` | Trait for IMU sensors (accel + gyro) |
| `BaroDriver` | Trait for barometric pressure sensors |
| `MagDriver` | Trait for magnetometers |
| `GnssDriver` | Trait for GNSS receivers |
| `ActuatorDriver` | Trait for actuators with optional telemetry |
| `FakeImu/Baro/Mag/Gnss` | SITL sensor drivers (fed from simulator) |
| `FakeActuator` | SITL actuator driver with telemetry support |

## Purpose

This crate abstracts sensor/actuator drivers behind traits, enabling the same `BoardHal`
to work with both real hardware and simulation:

```
SENSORS (Input):
Real HW:  SPI/I2C → Driver → BoardHal → SensorHal → Kernel
SITL:     Transport → FakeDriver → BoardHal → SensorHal → Kernel

ACTUATORS (Output + Optional Telemetry):
Real HW:  Kernel → ActuatorHal → BoardHal → Driver → PWM/DShot
SITL:     Kernel → ActuatorHal → BoardHal → FakeActuator → Transport → Simulator
                                                    ↑
                                   Same code path up to here
```

**Platform-agnostic**: No MCU register operations here. This crate defines interfaces;
board-level driver instantiation happens in `aviate-boards/*`.

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│  aviate-core (SensorHal + ActuatorHal traits)               │
└─────────────────────────────────────────────────────────────┘
                          ↑
           implements SensorHal + ActuatorHal
                          ↑
┌─────────────────────────────────────────────────────────────┐
│  BoardHal<I, B, M, G, T, A>                                 │
│  - Composes I/O drivers (sensors + actuators)               │
│  - Converts raw readings to aviate-core types               │
│  - Handles timestamps and health                            │
└─────────────────────────────────────────────────────────────┘
                          ↑
   ImuDriver, BaroDriver, MagDriver, GnssDriver, ActuatorDriver
                          ↑
┌───────────────────────────┬─────────────────────────────────┐
│  Real Hardware            │  SITL / Fake Devices            │
│  - Icm426xx<I2C>          │  - FakeImu                      │
│  - Bmp390<SPI>            │  - FakeBaro                     │
│  - Qmc5883l<I2C>          │  - FakeMag                      │
│  - UbloxGnss<UART>        │  - FakeGnss                     │
│  - DshotEscs<DMA>         │  - FakeActuator                 │
│  - CanEscs<FDCAN>         │    (supports telemetry)         │
└───────────────────────────┴─────────────────────────────────┘
```

## Key Components

### BoardHal

Generic composition of I/O drivers that implements both `SensorHal` and `ActuatorHal`:

```rust
pub struct BoardHal<I, B, M, G, T, A> {
    imu: I,       // implements ImuDriver
    baro: B,      // implements BaroDriver
    mag: M,       // implements MagDriver
    gnss: G,      // implements GnssDriver
    time: T,      // implements TimeSource
    actuator: A,  // implements ActuatorDriver
}

impl<I: ImuDriver, B: BaroDriver, M: MagDriver, G: GnssDriver, T: TimeSource, A: ActuatorDriver>
    SensorHal for BoardHal<I, B, M, G, T, A>
{
    fn read_imu(&mut self) -> Option<SensorReading<ImuData>> { ... }
    fn read_baro(&mut self) -> Option<SensorReading<BaroData>> { ... }
    fn read_mag(&mut self) -> Option<SensorReading<MagData>> { ... }
    fn read_gnss(&mut self) -> Option<SensorReading<GnssData>> { ... }
}

impl<I, B, M, G, T, A: ActuatorDriver> ActuatorHal for BoardHal<I, B, M, G, T, A> {
    fn write(&mut self, cmd: &ActuatorCmd) { ... }
    fn arm(&mut self) { self.actuator.arm(); }
    fn disarm(&mut self) { self.actuator.disarm(); }
    fn is_armed(&self) -> bool { self.actuator.is_armed() }
}
```

### Sensor Driver Traits

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

### Actuator Driver Trait

Actuators are bidirectional: commands out, optional telemetry in.

```rust
pub const MAX_ACTUATOR_OUTPUTS: usize = 16;

/// Commands sent to actuators
pub struct RawActuatorCmd {
    pub outputs: [f32; MAX_ACTUATOR_OUTPUTS],  // Normalized [0.0, 1.0]
    pub count: u8,
}

/// Telemetry from a single actuator channel
pub struct ActuatorTelemetry {
    pub speed_or_position: Option<f32>,  // RPM or servo angle
    pub current_a: Option<f32>,          // Amps
    pub temperature_c: Option<f32>,      // °C
    pub voltage_v: Option<f32>,          // Volts
    pub errors: ActuatorErrorFlags,
}

/// Aggregate status from all actuator channels
pub struct ActuatorStatus {
    pub channels: [ActuatorTelemetry; MAX_ACTUATOR_OUTPUTS],
    pub channel_count: u8,
    pub bus_voltage_v: Option<f32>,      // Power bus voltage
    pub total_current_a: Option<f32>,    // Total current draw
}

/// Error flags for actuator faults
pub struct ActuatorErrorFlags(pub u8);
impl ActuatorErrorFlags {
    pub const NONE: Self = Self(0);
    pub const OVERCURRENT: Self = Self(1 << 0);
    pub const OVERTEMPERATURE: Self = Self(1 << 1);
    pub const STALL: Self = Self(1 << 2);
    pub const COMMUNICATION: Self = Self(1 << 3);
    pub const ENCODER: Self = Self(1 << 4);
}

/// Trait for actuator drivers
pub trait ActuatorDriver {
    /// Send commands to actuators (required)
    fn write(&mut self, cmd: &RawActuatorCmd) -> ActuatorResult<()>;

    /// Read telemetry (optional, default: None)
    fn read_status(&mut self) -> Option<ActuatorStatus> { None }

    /// Check if telemetry is available (optional)
    fn status_ready(&mut self) -> bool { false }

    /// Arm the actuator system
    fn arm(&mut self);

    /// Disarm and stop all outputs
    fn disarm(&mut self);

    /// Check if system is armed
    fn is_armed(&self) -> bool;
}
```

### Actuator Types Supported

| Type | Commands | Telemetry | Example |
|------|----------|-----------|---------|
| PWM Motors | write() | None | Basic ESCs |
| DShot ESCs | write() | RPM, errors | BLHeli32 |
| CAN ESCs | write() | Full telemetry | DroneCAN |
| Servos | write() | Position | Digital servos |
| Other | write() | Varies | Airbrakes, parachutes, rocket engines |

### Fake Drivers (for SITL)

Fake drivers receive data from external sources (e.g., Gazebo via transport):

```rust
// Create fake sensors and actuator
let mut imu = FakeImu::new();
let mut actuator = FakeActuator::new();

// Transport feeds sensor data
imu.feed(RawImuReading {
    accel: [sensor.xacc, sensor.yacc, sensor.zacc],
    gyro: [sensor.xgyro, sensor.ygyro, sensor.zgyro],
    temperature: Some(sensor.temperature),
});

// BoardHal reads (same interface as real hardware)
let reading = board_hal.read_imu();

// Write actuator command via BoardHal
board_hal.write(&actuator_cmd);

// Transport takes command to send to simulator
if let Some(raw_cmd) = board_hal.actuator_mut().take_cmd() {
    transport.send_actuator(&raw_cmd);
}

// Simulator can send back telemetry
board_hal.actuator_mut().feed_status(ActuatorStatus {
    channel_count: 4,
    channels: [...],  // RPM, current, etc. from ESCs
    ..Default::default()
});

// Read actuator telemetry (if available)
if let Some(status) = board_hal.actuator_mut().read_status() {
    for (i, ch) in status.channels[..4].iter().enumerate() {
        if let Some(rpm) = ch.speed_or_position {
            log::info!("Motor {} RPM: {}", i, rpm);
        }
    }
}
```

## Adding a New Driver

### Sensor Driver Example

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
        let status = self.read_status()?;
        Ok(status & DATA_READY_BIT != 0)
    }
}
```

### Actuator Driver Example

To add support for a new actuator (e.g., DShot ESC with telemetry):

```rust
use aviate_hal_io::{ActuatorDriver, RawActuatorCmd, ActuatorStatus, ActuatorResult};

pub struct DshotEscGroup<DMA> {
    dma: DMA,
    armed: bool,
    telemetry_buf: Option<ActuatorStatus>,
}

impl<DMA> ActuatorDriver for DshotEscGroup<DMA> {
    fn write(&mut self, cmd: &RawActuatorCmd) -> ActuatorResult<()> {
        if !self.armed {
            return Ok(());  // Ignore commands when disarmed
        }
        // Convert normalized [0,1] to DShot values [48, 2047]
        for (i, &output) in cmd.outputs[..cmd.count as usize].iter().enumerate() {
            let dshot_val = 48 + (output * 1999.0) as u16;
            self.send_dshot(i, dshot_val);
        }
        Ok(())
    }

    fn read_status(&mut self) -> Option<ActuatorStatus> {
        self.telemetry_buf.take()
    }

    fn status_ready(&mut self) -> bool {
        self.telemetry_buf.is_some()
    }

    fn arm(&mut self) {
        self.armed = true;
    }

    fn disarm(&mut self) {
        self.armed = false;
        // Send zero throttle
        for i in 0..4 {
            self.send_dshot(i, 0);
        }
    }

    fn is_armed(&self) -> bool {
        self.armed
    }
}
```

## Usage Examples

### SITL Board

```rust
use aviate_hal_io::{BoardHal, FakeImu, FakeBaro, FakeMag, FakeGnss, FakeActuator};

// Type alias for SITL (6 type parameters)
type SitlBoardHal = BoardHal<FakeImu, FakeBaro, FakeMag, FakeGnss, SitlTime, FakeActuator>;

// Create board HAL
let board_hal = BoardHal::new(
    FakeImu::new(),
    FakeBaro::new(),
    FakeMag::new(),
    FakeGnss::new(),
    SitlTime::new(),
    FakeActuator::new(),
);

// In control loop:
// 1. Transport feeds data to fake sensors
board_hal.imu_mut().feed(sensor_data.imu);

// 2. Read via SensorHal interface
if let Some(imu) = board_hal.read_imu() {
    kernel.step(&imu);
}

// 3. Write actuator via ActuatorHal interface
board_hal.write(&actuator_cmd);

// 4. Transport takes command to send to simulator
if let Some(raw_cmd) = board_hal.actuator_mut().take_cmd() {
    transport.send_actuator(&raw_cmd);
}
```

### Real Hardware Board

```rust
use aviate_hal_io::BoardHal;
use icm426xx::Icm426xx;
use bmp3xx::Bmp3xx;
use dshot::DshotEscGroup;

// Type alias for real hardware (6 type parameters)
type HwBoardHal = BoardHal<
    Icm426xx<I2C>,
    Bmp3xx<SPI>,
    Qmc5883l<I2C>,
    UbloxGnss<UART>,
    HwTime,
    DshotEscGroup<DMA>,
>;

// Create with real drivers
let board_hal = BoardHal::new(
    Icm426xx::new(i2c),
    Bmp3xx::new(spi),
    Qmc5883l::new(i2c2),
    UbloxGnss::new(uart),
    HwTime::new(),
    DshotEscGroup::new(dma),
);

// Same SensorHal + ActuatorHal interface - kernel code is unchanged!
if let Some(imu) = board_hal.read_imu() {
    kernel.step(&imu);
}
board_hal.write(&actuator_cmd);

// Read ESC telemetry (if supported by hardware)
if board_hal.actuator().status_ready() {
    if let Some(status) = board_hal.actuator_mut().read_status() {
        for (i, ch) in status.channels[..4].iter().enumerate() {
            if let Some(rpm) = ch.speed_or_position {
                log::info!("Motor {} RPM: {}", i, rpm);
            }
        }
    }
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
