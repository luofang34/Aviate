# aviate-hal-xil

X-In-Loop (XIL) simulation infrastructure for Aviate flight systems.

## Quick Reference

| Component | Description |
|-----------|-------------|
| `SitlMavlink` | MAVLink I/O (receives HIL_SENSOR, sends HIL_ACTUATOR) |
| `KinematicsBackend` | Trait for simulator backends (Gazebo, future: Unity, AirSim) |
| `Mission` / `Phase` | Test framework for defining flight scenarios |
| `TestConfig` | TOML parser for test configuration files |
| `FlightLog` | Flight data recording and statistics |

## Purpose

This crate provides **HAL-facing XIL glue** - the integration layer between flight controllers
and simulators. It handles:

- MAVLink HIL I/O (SitlMavlink)
- Backend trait definition (KinematicsBackend)
- Test infrastructure (missions, criteria, multi-vehicle)
- Flight logging

**NOT handled here**: World modeling, Gazebo-specific code (that's in `backends/gz`).

## Scope Clarification

```
aviate-hal-xil (this crate, no backend deps)
       ↑
aviate-backend-gz (implements KinematicsBackend)
       ↑ (FFI/IPC)
aviate_gz_plugin (C++, Gazebo)
```

The xil core does NOT depend on any specific backend. Backends implement traits defined
here and are selected at runtime via configuration.

## Key Components

### SitlMavlink

MAVLink transceiver for SITL simulation:

```rust
use aviate_hal_xil::{SitlMavlink, XilConfig};

// Create MAVLink I/O
let config = XilConfig::for_instance(0);
let mut mavlink = SitlMavlink::new(config)?;

// In control loop:
mavlink.poll();  // Receive HIL_SENSOR, HIL_GPS messages

// Get sensor data (to feed fake sensors)
if let Some(sensor) = mavlink.take_sensor_data() {
    board_hal.imu_mut().feed(sensor.imu);
    board_hal.baro_mut().feed(sensor.baro);
    board_hal.mag_mut().feed(sensor.mag);
}

if let Some(gps) = mavlink.take_gps_data() {
    board_hal.gnss_mut().feed(gps.gnss);
}

// Send actuator commands
mavlink.write(&actuator_cmd);
```

`SitlMavlink` implements `ActuatorHal`, `SystemHal`, and `CommandHal` from aviate-core.

### KinematicsBackend Trait

Interface for physics/kinematics backends:

```rust
pub trait KinematicsBackend: Send {
    fn name(&self) -> &str;
    fn start(&mut self, cfg: &BackendConfig) -> Result<(), BackendError>;
    fn step(&mut self, world: &mut World) -> Result<Duration, BackendError>;
    fn poll_ready(&self) -> bool;
    fn sim_time(&self) -> Duration;
    fn stop(&mut self) -> Result<(), BackendError>;
    fn reset(&mut self) -> Result<(), BackendError>;
}
```

Current implementations:
- `aviate-backend-gz`: Gazebo Harmonic via shared memory

Future planned:
- AirSim
- Unity
- Custom world kernel

### Test Framework

Define test scenarios with missions, phases, and success criteria:

```rust
use aviate_hal_xil::{Mission, Phase, Action, Criterion};

let mission = Mission {
    name: "takeoff_test".to_string(),
    phases: vec![
        Phase {
            name: "arm".to_string(),
            duration: Duration::from_secs(5),
            actions: vec![Action::Arm],
            criteria: vec![Criterion::Armed],
        },
        Phase {
            name: "takeoff".to_string(),
            duration: Duration::from_secs(10),
            actions: vec![Action::Takeoff { altitude: 5.0 }],
            criteria: vec![Criterion::Altitude { min: 4.0, max: 6.0 }],
        },
    ],
    ..Default::default()
};
```

## Port Allocation

Multi-vehicle simulations use instance-based port allocation:

| Instance | Sensor Port | Actuator Port | Shared Memory |
|----------|-------------|---------------|---------------|
| 0 | 14560 | 14561 | /aviate_gz_bridge |
| 1 | 14570 | 14571 | /aviate_gz_bridge_1 |
| N | 14560+N*10 | 14561+N*10 | /aviate_gz_bridge_N |

```rust
// Create config for specific instance
let config = XilConfig::for_instance(1);  // Instance 1: ports 14570/14571
```

## Timing Modes

The backend supports different timing modes:

```rust
pub enum TimingMode {
    Unlimited,           // Run as fast as possible
    RealTime,            // 1x real-time
    Scaled(f64),         // e.g., Scaled(2.0) = 2x faster than real-time
}

pub enum LockstepMode {
    Async,               // Backend runs independently
    Lockstep {           // Barrier-based synchronization
        timeout_us: u64,
    },
}
```

**Lockstep** is recommended for deterministic, reproducible tests.

## Test Configuration (TOML)

Tests are defined in TOML files:

```toml
[test]
name = "basic_flight"
description = "Basic single vehicle flight test"

[world]
vehicles = 1
lockstep = true

[[vehicles]]
id = "x500"
model = "x500"
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
name = "hover"
duration_sec = 10.0
actions = ["hold"]
criteria = [{ type = "altitude", min = 4.5, max = 5.5 }]

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

Parse with:
```rust
use aviate_hal_xil::parse_test_config;
let config = parse_test_config(Path::new("tests/quadcopter/basic_flight.toml"))?;
```

## Flight Logging

Record flight data for post-analysis:

```rust
use aviate_hal_xil::{FlightLog, FlightLogConfig};

let mut log = FlightLog::new(FlightLogConfig {
    sample_rate_hz: 100,
    ..Default::default()
});

// In control loop:
log.record(&sensors, &actuator_cmd, timestamp);

// After flight:
let stats = log.stats();
println!("Max altitude: {}", stats.max_altitude);
println!("Flight time: {:?}", stats.flight_time);
```

## DO-178C Note

This crate is **test tooling** - it is NOT deployed on aircraft.

```
Flight Software (deployed)    │   Test Tooling (not deployed)
──────────────────────────────┼──────────────────────────────────
aviate-core                   │   aviate-hal-xil (this crate)
aviate-hal/io                 │   aviate-hal-xil/backends/gz
aviate-boards/*               │
```

## Backend Implementations

### Gazebo (aviate-backend-gz)

The Gazebo backend uses shared memory for low-latency communication:

```
┌─────────────────┐     shared memory     ┌────────────────┐
│  AviateGzPlugin │ ←──────────────────→  │   gz-bridge    │
│  (C++, Gazebo)  │                       │    (Rust)      │
└─────────────────┘                       └────────────────┘
         ↓                                        ↓
   Physics step                             MAVLink UDP
         ↓                                        ↓
   Sensor readings                        SitlMavlink (FC)
```

See [backends/gz/README.md](backends/gz/README.md) for details.

## Implementation Details

See [src/lib.rs](src/lib.rs) for the complete API.
