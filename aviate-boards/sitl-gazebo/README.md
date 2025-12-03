# sitl-x500

Reference SITL board implementation for Aviate - simulates a X500 quadcopter in Gazebo.

## Quick Reference

| Property | Value |
|----------|-------|
| Board ID | `sitl-x500` |
| Airframe | Quadcopter (X configuration) |
| Motors   | 4 |
| Simulator | Gazebo Harmonic |
| Loop Rate | 1 kHz |

## Purpose

This board serves as the **reference implementation** for SITL boards. It demonstrates
the key architectural principle: **same BoardHal code path for SITL and real hardware**.

```
SITL:  Gazebo → HIL_SENSOR → SitlMavlink → FakeImu/Baro/... → BoardHal → SensorHal
Real:  SPI/I2C → BMI088/BMP390/... → BoardHal → SensorHal
                                          ↓
                                   Same kernel code
```

## Architecture

```
Gazebo Physics
     ↓
AviateGzPlugin (C++, shared memory)
     ↓
gz-bridge (Rust, MAVLink conversion)
     ↓
SitlMavlink (UDP receive HIL_SENSOR/HIL_GPS)
     ↓
board.step():
  1. mavlink.poll()
  2. board_hal.imu_mut().feed(data)
  3. board_hal.read_imu() → SensorReading
  4. kernel.step()
  5. mavlink.write(actuator_cmd)
     ↓
gz-bridge → Gazebo (motor velocities)
```

## Motor Layout (Quad-X)

```
    Front
  1 (CW)   2 (CCW)
      \   /
       [X]
      /   \
  4 (CCW)  3 (CW)
    Rear
```

Motor directions:
- Motors 1,3: Clockwise (CW)
- Motors 2,4: Counter-clockwise (CCW)

## Quick Start

```bash
# Interactive mode (with Gazebo GUI)
./scripts/run_sitl.sh tests/quadcopter/basic_flight.toml

# Headless mode (CI/automation, uses EGL rendering for camera sensors)
HEADLESS=1 ./scripts/run_sitl.sh tests/quadcopter/basic_flight.toml
```

## Key Insight: BoardHal Reuse

The sitl-x500 board uses the **same `BoardHal`** that real hardware boards would use:

```rust
// Type alias showing we use the standard BoardHal with fake sensors
pub type SitlBoardHal = BoardHal<FakeImu, FakeBaro, FakeMag, FakeGnss, SitlTime>;

// For real hardware, the only difference is the driver types:
// pub type HwBoardHal = BoardHal<Icm426xx<I2C>, Bmp390<SPI>, Qmc5883l<I2C>, UbloxGnss<UART>, HwTime>;
```

Both implement `SensorHal`, so the kernel code is identical between SITL and real hardware.

## Control Loop

The `step()` method implements the standard control loop pattern:

```rust
pub fn step(&mut self) -> ActuatorCmd {
    // 1. Poll MAVLink for incoming HIL messages
    self.mavlink.poll();

    // 2. Feed fake sensors with HIL data (via BoardHal accessors)
    if let Some(sensor_data) = self.mavlink.take_sensor_data() {
        self.board_hal.imu_mut().feed(sensor_data.imu);
        self.board_hal.baro_mut().feed(sensor_data.baro);
        self.board_hal.mag_mut().feed(sensor_data.mag);
    }
    if let Some(gps_data) = self.mavlink.take_gps_data() {
        self.board_hal.gnss_mut().feed(gps_data.gnss);
    }

    // 3. Read sensors via BoardHal's SensorHal implementation
    if let Some(imu) = self.board_hal.read_imu() {
        // Process IMU data...
    }
    // ... other sensors ...

    // 4. Process commands
    if let Some(cmd) = self.mavlink.recv_command() {
        // Handle arm/disarm/flight commands
    }

    // 5. Step kernel
    let actuator_cmd = self.kernel.step(time_delta, &command, &sensors, 0);

    // 6. Write outputs
    self.mavlink.write(&actuator_cmd);

    actuator_cmd
}
```

## Multi-Vehicle Support

For multi-vehicle simulations, each instance uses separate ports:

| Instance | Sensor Port | Actuator Port | Shared Memory |
|----------|-------------|---------------|---------------|
| 0 | 14560 | 14561 | /aviate_gz_bridge |
| 1 | 14570 | 14571 | /aviate_gz_bridge_1 |
| N | 14560+N*10 | 14561+N*10 | /aviate_gz_bridge_N |

Set via environment variable:
```bash
AVIATE_INSTANCE=1 ./target/release/aviate-sitl-x500
```

## Configuration

The board uses `SitlConfig` from `aviate-hal-xil`:

```rust
let config = SitlConfig::for_instance(0);  // Instance 0
let board = X500SitlBoard::with_config(config)?;
```

## Dependencies

```toml
[dependencies]
aviate-core = { path = "../../aviate-core" }
aviate-hal-io = { path = "../../aviate-hal/io" }
aviate-hal-xil = { path = "../../aviate-hal/xil" }
aviate-airframe-quadcopter = { path = "../../aviate-airframes/quadcopter" }
```

## Test Configuration

Test scenarios are defined in TOML files. Example (`tests/quadcopter/basic_flight.toml`):

```toml
[test]
name = "basic_flight"
description = "Basic single vehicle flight test (takeoff and land)"

[world]
vehicles = 1
lockstep = true

[[phases]]
name = "arm"
duration_sec = 5.0
actions = ["arm"]
criteria = ["armed"]

[[phases]]
name = "takeoff"
duration_sec = 10.0
actions = [{ type = "takeoff", altitude = 5.0 }]
criteria = [{ type = "altitude", min = 4.0, max = 6.0 }]
```

See [aviate-hal/xil/README.md](../../aviate-hal/xil/README.md) for test framework details.

## Implementation Details

See [src/lib.rs](src/lib.rs) for the complete implementation.
