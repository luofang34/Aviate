# aviate-boards

Board configurations for Aviate flight systems.

## Quick Reference

| Board | Platform | Airframe | Status |
|-------|----------|----------|--------|
| [sitl-x500](sitl-x500/README.md) | Gazebo SITL | Quadcopter | Reference |

## What is a Board?

A **board** is the composition of:
- **HAL**: `BoardHal` with specific driver instantiation
- **Kernel**: `AviateKernel` with controller and mixer
- **I/O**: Communication interfaces (MAVLink, telemetry, etc.)

```
┌────────────────────────────────┐
│   MyBoard (aviate-boards)      │
│  - hal: BoardHal<...>          │
│  - kernel: AviateKernel        │
│  - comms: Mavlink/Telemetry    │
└────────────────────────────────┘
         ↑           ↑
   Real or Fake   Radio / USB / etc.
   Drivers         (platform dep.)
```

## Control Loop Pattern

All boards follow this standard pattern (from sitl-x500):

```rust
pub fn step(&mut self) -> ActuatorCmd {
    // 1. Poll sensors / receive data
    self.mavlink.poll();

    // 2. Feed drivers (for SITL: fake sensors)
    if let Some(data) = self.mavlink.take_sensor_data() {
        self.board_hal.imu_mut().feed(data.imu);
        self.board_hal.baro_mut().feed(data.baro);
        self.board_hal.mag_mut().feed(data.mag);
    }

    // 3. Read via BoardHal (SensorHal trait)
    if let Some(imu) = self.board_hal.read_imu() {
        self.sensor_cache.imu = Some(imu);
    }
    // ... other sensors ...

    // 4. Process commands
    if let Some(cmd) = self.mavlink.recv_command() {
        // Handle arm/disarm/flight commands
    }

    // 5. Step kernel
    let actuator_cmd = self.kernel.step(dt, &command, &sensors, 0);

    // 6. Write actuator commands
    self.mavlink.write(&actuator_cmd);

    actuator_cmd
}
```

## DO-178C Note

Board implementations are **flight software**. The IO driver + BoardHal behavior should be
verified via XIL/SITL with the same test cases as the real board.

```
Flight Software (deployed)        │   Test Tooling (not deployed)
──────────────────────────────────┼──────────────────────────────────
aviate-core                       │   aviate-hal-xil
aviate-hal/io                     │   aviate-hal-xil/backends/gz
aviate-boards/* (this directory)  │
```

---

# Tutorial: Adding a New SITL Board

This tutorial walks through creating a new SITL board using `sitl-x500` as a reference.

## Step 1: Create Board Crate

```bash
mkdir -p aviate-boards/my-board/src
```

Create `aviate-boards/my-board/Cargo.toml`:

```toml
[package]
name = "aviate-board-my-board"
edition.workspace = true
version.workspace = true
license.workspace = true
description = "My custom board configuration"

[lib]
name = "aviate_board_my_board"
path = "src/lib.rs"

[dependencies]
aviate-core = { path = "../../aviate-core" }
aviate-hal-io = { path = "../../aviate-hal/io" }
aviate-hal-xil = { path = "../../aviate-hal/xil" }
aviate-airframe-quadcopter = { path = "../../aviate-airframes/quadcopter" }
```

## Step 2: Define Board Struct

Create `aviate-boards/my-board/src/lib.rs`:

```rust
use aviate_core::control::multirotor::MultirotorController;
use aviate_core::mixer::QuadXMixer;
use aviate_core::AviateKernel;

use aviate_hal_io::{BoardHal, FakeBaro, FakeGnss, FakeImu, FakeMag};
use aviate_hal_xil::{SitlConfig, SitlMavlink};

// Time source for SITL
struct SitlTime {
    start: std::time::Instant,
}

impl SitlTime {
    fn new() -> Self {
        Self { start: std::time::Instant::now() }
    }
}

impl aviate_hal_io::TimeSource for SitlTime {
    fn now_us(&self) -> u64 {
        self.start.elapsed().as_micros() as u64
    }
}

// Type alias for your board's HAL
// For real hardware, swap FakeImu → RealImu<Bus>, etc.
pub type MyBoardHal = BoardHal<FakeImu, FakeBaro, FakeMag, FakeGnss, SitlTime>;

pub struct MyBoard {
    mavlink: SitlMavlink,
    board_hal: MyBoardHal,  // Same BoardHal that real hardware would use!
    kernel: AviateKernel<MultirotorController, QuadXMixer>,
    // ... other fields
}
```

## Step 3: Implement Control Loop

```rust
impl MyBoard {
    pub fn new() -> std::io::Result<Self> {
        let config = SitlConfig::default();
        let mavlink = SitlMavlink::new(config)?;

        let board_hal = BoardHal::new(
            FakeImu::new(),
            FakeBaro::new(),
            FakeMag::new(),
            FakeGnss::new(),
            SitlTime::new(),
        );

        let kernel = /* create kernel with controller + mixer */;

        Ok(Self { mavlink, board_hal, kernel })
    }

    pub fn step(&mut self) -> ActuatorCmd {
        // Copy pattern from sitl-x500 (see above)
    }

    pub fn run(&mut self) -> ! {
        let loop_period_us = 1000; // 1kHz
        loop {
            // Timing and step logic
            self.step();
        }
    }
}
```

## Step 4: Add to Workspace

Update root `Cargo.toml`:

```toml
[workspace]
members = [
    # ... existing members
    "aviate-boards/my-board",
]
```

## Step 5: Create Test Configuration

Create `tests/my-board/basic_flight.toml`:

```toml
[test]
name = "my_board_basic"
description = "Basic flight test for my board"

[world]
vehicles = 1
lockstep = true

[[vehicles]]
id = "vehicle_0"
model = "x500"  # or your custom model
instance = 0
spawn_position = [0.0, 0.0, 0.0]

[[vehicles.mission.phases]]
name = "arm"
duration_sec = 5.0
actions = ["arm"]
criteria = ["armed"]

[[vehicles.mission.phases]]
name = "takeoff"
duration_sec = 10.0
actions = [{ type = "takeoff", altitude = 5.0 }]
criteria = [{ type = "altitude", min = 4.0, max = 6.0 }]

[[vehicles.mission.phases]]
name = "land"
duration_sec = 10.0
actions = ["land"]
criteria = [{ type = "altitude", min = -0.5, max = 0.5 }]

[[vehicles.mission.phases]]
name = "disarm"
duration_sec = 5.0
actions = ["disarm"]
criteria = ["disarmed"]
```

## Step 6: Run SITL Test

```bash
# Build the board
cargo build -p aviate-board-my-board

# Run test
./scripts/run_sitl.sh tests/my-board/basic_flight.toml
```

---

# Tutorial: Adding Real Hardware Support

To port your board from SITL to real hardware:

## Step 1: Create Hardware Drivers

Implement driver traits for your hardware:

```rust
use aviate_hal_io::{ImuDriver, RawImuReading, SensorResult};
use embedded_hal::i2c::I2c;

pub struct MyImu<I2C> {
    bus: I2C,
}

impl<I2C: I2c> ImuDriver for MyImu<I2C> {
    fn read(&mut self) -> SensorResult<RawImuReading> {
        // Read from hardware registers
    }

    fn data_ready(&mut self) -> SensorResult<bool> {
        // Check interrupt/status register
    }
}
```

## Step 2: Swap Type Aliases

```rust
// SITL:
pub type SitlBoardHal = BoardHal<FakeImu, FakeBaro, FakeMag, FakeGnss, SitlTime>;

// Real hardware:
pub type HwBoardHal = BoardHal<MyImu<I2C>, MyBaro<SPI>, MyMag<I2C>, MyGnss<UART>, HwTime>;
```

## Step 3: Adjust Control Loop

The control loop structure remains the same - only the sensor feeding changes:

```rust
// SITL: Fake sensors fed from MAVLink
if let Some(data) = self.mavlink.take_sensor_data() {
    self.board_hal.imu_mut().feed(data.imu);
}

// Real hardware: Drivers read directly from hardware
// (no feeding needed - read() talks to SPI/I2C)
```

The kernel code is completely unchanged because it only sees the `SensorHal` interface.

---

## Board Checklist

When creating a new board, verify:

- [ ] `BoardHal` type alias defined
- [ ] Control loop follows standard pattern
- [ ] Kernel created with appropriate controller/mixer
- [ ] Test configuration created
- [ ] SITL tests pass
- [ ] (For real hardware) Hardware drivers implement traits
- [ ] (For real hardware) Pre-arm checks configured

## Reference Implementations

- [sitl-x500](sitl-x500/README.md) - SITL quadcopter (reference)
