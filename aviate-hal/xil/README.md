# aviate-hal-xil

X-In-Loop (XIL) simulation and testing infrastructure for Aviate flight systems.

## What is XIL?

XIL encompasses both:
- **SITL (Software-In-The-Loop)**: Full software simulation - flight controller runs on host PC
- **HITL (Hardware-In-The-Loop)**: Real FC hardware connected to simulated environment

```
┌─────────────────────────────────────────────────────────────────────────┐
│                        X-In-Loop (XIL) Modes                            │
├─────────────────────────────────┬───────────────────────────────────────┤
│            SITL                 │              HITL                     │
├─────────────────────────────────┼───────────────────────────────────────┤
│  FC: Host process               │  FC: Real MCU (STM32, etc.)           │
│  Sensors: FakeImu, FakeBaro     │  Sensors: Real drivers, simulated I/O │
│  Transport: UDP loopback        │  Transport: UART/USB to host          │
│  Use: CI/CD, rapid iteration    │  Use: Hardware validation, timing     │
└─────────────────────────────────┴───────────────────────────────────────┘
```

## Quick Reference

| Component | Description |
|-----------|-------------|
| `XilIO` | Transport layer for XIL (sensor input, actuator output) |
| `KinematicsBackend` | Trait for simulator backends (Gazebo, Unity, AirSim) |
| `Mission` / `Phase` | Test framework for defining flight scenarios |
| `TestConfig` | TOML parser for test configuration files |
| `FlightLog` | Flight data recording and statistics |

## Purpose

This crate provides **XIL infrastructure** - the integration layer between flight controllers
and simulators for both SITL and HITL testing:

- Transport I/O - Communication between FC and simulator
- Backend trait definition (KinematicsBackend)
- Test infrastructure (missions, criteria, multi-vehicle)
- Flight logging

**NOT handled here**: World modeling, backend-specific code (that's in `backends/*`).

## Architecture

### SITL Mode

Flight controller runs as a host process, using fake drivers:

```
┌──────────────┐                    ┌─────────────────────────────────────┐
│   Simulator  │                    │     Flight Controller (Host)        │
│   (Gazebo)   │                    │                                     │
│              │    UDP/MAVLink     │  ┌─────────┐     ┌──────────────┐  │
│  Sensors ────┼───────────────────→│  │ XilIO   │────→│ FakeImu/Baro │  │
│              │                    │  │(transport)    │ FakeMag/Gnss │  │
│              │                    │  └─────────┘     └──────┬───────┘  │
│              │                    │                         ↓          │
│              │                    │                   ┌──────────┐     │
│              │                    │                   │ BoardHal │     │
│              │                    │                   └────┬─────┘     │
│              │                    │                        ↓          │
│              │                    │                   ┌──────────┐     │
│  Motors  ←───┼────────────────────│←─ FakeActuator ←──│  Kernel  │     │
│              │                    │                   └──────────┘     │
└──────────────┘                    └─────────────────────────────────────┘
```

### HITL Mode

Flight controller runs on real hardware, I/O redirected to simulator:

```
┌──────────────┐                    ┌─────────────────────────────────────┐
│   Simulator  │                    │     Flight Controller (MCU)         │
│   (Gazebo)   │                    │                                     │
│              │    UART/USB        │  ┌─────────┐     ┌──────────────┐  │
│  Sensors ────┼───────────────────→│  │ HitlIO  │────→│ HitlImu/Baro │  │
│              │                    │  │(transport)    │  (redirect)  │  │
│              │                    │  └─────────┘     └──────┬───────┘  │
│              │                    │                         ↓          │
│              │                    │                   ┌──────────┐     │
│              │                    │                   │ BoardHal │     │
│              │                    │                   └────┬─────┘     │
│              │                    │                        ↓          │
│              │                    │                   ┌──────────┐     │
│  Motors  ←───┼────────────────────│←─ HitlActuator ←──│  Kernel  │     │
│              │                    │                   └──────────┘     │
└──────────────┘                    └─────────────────────────────────────┘
```

## Transport vs HAL

**Important architectural distinction:**

```
┌─────────────────────────────────────────────────────────────┐
│  XilIO = Transport Layer (NOT HAL)                          │
│  - Sends actuator commands to simulator                     │
│  - Receives sensor data from simulator                      │
│  - Does NOT implement ActuatorHal                           │
│  - Protocol: MAVLink HIL messages (UDP for SITL, UART/USB   │
│              for HITL)                                      │
└─────────────────────────────────────────────────────────────┘
                          ↑
               send_actuator(), poll()
                          ↑
┌─────────────────────────────────────────────────────────────┐
│  FakeActuator / HitlActuator (in aviate-hal-io)             │
│  - Implements ActuatorDriver                                 │
│  - Buffers commands for transport to collect                │
│  - Receives telemetry from transport                        │
└─────────────────────────────────────────────────────────────┘
```

The kernel writes to `BoardHal`, then the transport layer takes the buffered
command and sends it to the simulator. This keeps the HAL abstraction clean
and transport-agnostic.

## Scope Clarification

```
aviate-hal-xil (this crate, no backend deps)
       ↑
aviate-backend-gz (implements KinematicsBackend)
       ↑ (FFI/IPC)
aviate_gz_plugin (C++, Gazebo)
```

The xil core does NOT depend on any specific backend. Backends implement traits
defined here and are selected at runtime via configuration.

## Key Components

### XilIO (Transport)

Transport layer for XIL simulation:

```rust
use aviate_hal_xil::{SitlIO, XilConfig};

// Create transport (SITL mode)
let config = XilConfig::for_instance(0);
let mut transport = SitlIO::new(config)?;

// In control loop:
transport.poll();  // Receive sensor messages from simulator

// Get sensor data (to feed fake sensors in BoardHal)
if let Some(sensor) = transport.take_sensor_data() {
    board_hal.imu_mut().feed(sensor.imu);
    board_hal.baro_mut().feed(sensor.baro);
    board_hal.mag_mut().feed(sensor.mag);
}

if let Some(gps) = transport.take_gps_data() {
    board_hal.gnss_mut().feed(gps.gnss);
}

// Kernel writes to BoardHal
board_hal.write(&actuator_cmd);

// Transport takes command from FakeActuator and sends to simulator
if let Some(raw_cmd) = board_hal.actuator_mut().take_cmd() {
    transport.send_actuator(&raw_cmd);
}
```

Current transports:
- `SitlIO`: UDP-based for SITL (host-to-host)
- Future: `HitlIO` for UART/USB (host-to-MCU)

Legacy alias `SitlMavlink` is available for compatibility.

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
- Custom dynamics kernel

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

## Data Flow

Complete XIL data flow (same pattern for SITL and HITL, different transport):

```
SENSOR DATA (Simulator → Flight Controller):
┌──────────┐    Transport     ┌─────────┐    feed()    ┌───────────┐
│Simulator │ ───────────────→ │  XilIO  │ ──────────→  │ FakeImu   │
│ (sensor) │  UDP (SITL)      │         │              │ FakeBaro  │
└──────────┘  UART (HITL)     └─────────┘              │ etc.      │
                                                        └─────┬─────┘
                                                              ↓
                                                     BoardHal.read_imu()
                                                              ↓
                                                     ┌────────────────┐
                                                     │     Kernel     │
                                                     └────────────────┘

ACTUATOR DATA (Flight Controller → Simulator):
┌────────────────┐                 ┌───────────────┐
│     Kernel     │                 │   BoardHal    │
└───────┬────────┘                 │ (ActuatorHal) │
        │                          └───────┬───────┘
        │ write()                          │
        ↓                                  ↓
┌───────────────┐    take_cmd()    ┌─────────────┐
│ FakeActuator  │ ←─────────────── │    XilIO    │
│ (buffer cmd)  │                  │ (transport) │
└───────────────┘                  └──────┬──────┘
                                          │ send_actuator()
                                          ↓
                                   ┌──────────┐
                                   │Simulator │
                                   │ (motors) │
                                   └──────────┘
```

## Port Allocation

Multi-vehicle simulations use instance-based port allocation:

| Instance | Sensor Port | Actuator Port | Shared Memory |
|----------|-------------|---------------|---------------|
| 0 | 14560 | 14561 | /aviate_gz_bridge_v3 |
| 1 | 14570 | 14571 | /aviate_gz_bridge_v3_1 |
| N | 14560+N*10 | 14561+N*10 | /aviate_gz_bridge_v3_N |

```rust
// Create config for specific instance
let config = XilConfig::for_instance(1);  // Instance 1: ports 14570/14571
```

## Timing Modes

The backend supports different timing modes:

```rust
pub enum TimingMode {
    Unlimited,           // Run as fast as possible (CI/CD)
    RealTime,            // 1x real-time (HITL requirement)
    Scaled(f64),         // e.g., Scaled(2.0) = 2x faster
}

pub enum LockstepMode {
    Async,               // Backend runs independently
    Lockstep {           // Barrier-based synchronization
        timeout_us: u64,
    },
}
```

- **SITL**: Lockstep recommended for deterministic, reproducible tests
- **HITL**: RealTime + Async required (real hardware has real timing)

## Test Configuration (TOML)

Tests are defined in TOML files:

```toml
[test]
name = "basic_flight"
description = "Basic single vehicle flight test"

[world]
vehicles = 1
lockstep = true      # false for HITL
mode = "sitl"        # or "hitl"

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
   Sensor readings                          XilIO (FC)
```

See [backends/gz/README.md](backends/gz/README.md) for details.

## Implementation Details

See [src/lib.rs](src/lib.rs) for the complete API.
