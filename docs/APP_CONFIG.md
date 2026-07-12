# Aviate Application Configuration (TOML)

## Overview

Each Aviate application is configured via a `AviateApp.toml` file. Configuration is parsed **once at startup** (LOW-DAL init phase) and never touched by the high-DAL control loop.

## DO-178C DAL Separation

- **LOW-DAL**: TOML parsing (`aviate-config` crate)
- **HIGH-DAL**: Typed `AppConfig` struct (used by control loop)
- Parse failure = safe abort / fail to arm

## Configuration Format

### Top-Level Sections

```toml
[app]          # Required: App metadata
[telemetry]    # Optional: Telemetry queue config (uses defaults if omitted)
[security]     # Optional: Security profile (defaults to "none")
[[transports]]  # Optional: Transport configurations (can have multiple)
[simulator]    # Optional: Simulator config (SITL only)
```

### `[app]` Section (Required)

Application metadata and identification.

```toml
[app]
id = "sitl-gazebo-x500"        # App identifier (for logging, CM tracking)
board = "sitl-gazebo"          # Board ID (matches aviate-boards/* crate)
airframe = "x500"              # Airframe ID (matches airframe feature)
env = "sitl"                   # Environment: "flight", "sitl", or "hitl"
```

**Fields:**
- `id` (string, required): Unique app identifier
- `board` (string, required): Board crate name (e.g., "micoair-h743-v2", "sitl-gazebo")
- `airframe` (string, required): Airframe type (e.g., "quad-x", "x500")
- `env` (string, required): Runtime environment
  - `"flight"`: Real hardware
  - `"sitl"`: Software-in-the-loop simulation
  - `"hitl"`: Hardware-in-the-loop simulation

### `[telemetry]` Section (Optional)

Telemetry queue configuration. Defaults provided if omitted.

```toml
[telemetry]
frame_size = 280  # Maximum frame size in bytes
queue_len = 32    # Queue depth (number of frames)
heartbeat_hz = 1
attitude_hz = 10
position_hz = 4
estimator_status_hz = 4
```

**Defaults:**
- `frame_size`: 280 bytes
- `queue_len`: 32 frames
- `heartbeat_hz`: 1 Hz
- `attitude_hz`: 10 Hz
- `position_hz`: 4 Hz
- `estimator_status_hz`: 4 Hz

Rates are valid in `1..=255` Hz. Zero is a configuration error: telemetry
is disabled with a startup error naming the field, never reinterpreted.
A rate that does not divide the control-loop rate is rounded down to the
nearest achievable rate, so the achieved rate never exceeds the request
outside the loop-counter wrap interval (once per ~124 days at 400 Hz).

### `[security]` Section (Optional)

Security profile selection. Defaults to "none" if omitted.

```toml
[security]
profile = "none"  # Options: "none", "auth-only", "auth-and-encrypt"
```

**Profiles:**
- `"none"`: No authentication or encryption (development only)
- `"auth-only"`: Command authentication via signatures
- `"auth-and-encrypt"`: Authentication + AES-GCM encryption

**WARNING**: `"none"` profile is for development/testing only. Production systems MUST use authentication.

### `[[transports]]` Section (Optional, repeatable)

Transport configuration for communication ports. Multiple transports supported.

```toml
[[transports]]
protocol = "mavlink"              # Protocol type
port = "usb_cdc"                  # Port identifier
roles = ["telemetry", "command"]  # Transport roles
baudrate = 115200                 # Optional: Serial baudrate

[[transports]]
protocol = "crsf"
port = "uart1"
roles = ["rc_input"]
baudrate = 420000
```

**Common Fields:**
- `protocol` (string, required): Communication protocol
  - `"mavlink"`: MAVLink 2.0
  - `"crsf"`: Crossfire RC protocol
  - `"sbus"`: S.BUS RC protocol
- `port` (string, required): Port identifier
  - Hardware: `"usb_cdc"`, `"uart1"`, `"uart2"`, etc.
  - SITL: `"udp"`, `"tcp"`
- `roles` (array of strings, required): Transport roles
  - `"telemetry"`: Outbound telemetry data
  - `"command"`: Inbound flight commands
  - `"rc_input"`: RC receiver input
- `baudrate` (integer, optional): Serial port baudrate (hardware only)

**SITL-Specific Fields:**
- `port_sensor` (integer, optional): UDP port for sensor data (Gazebo)
- `port_actuator` (integer, optional): UDP port for actuator commands (Gazebo)

**Example: Multi-Transport Configuration**

```toml
# Transport 1: MAVLink telemetry + commands via USB
[[transports]]
protocol = "mavlink"
port = "usb_cdc"
roles = ["telemetry", "command"]
baudrate = 115200

# Transport 2: CRSF RC input via UART
[[transports]]
protocol = "crsf"
port = "uart1"
roles = ["rc_input"]
baudrate = 420000
```

### `[simulator]` Section (Optional, SITL only)

Simulator backend configuration. Only used when `env = "sitl"`.

```toml
[simulator]
backend = "gazebo"  # Options: "gazebo", "jmavsim"
headless = false    # Run without GUI
lockstep = false    # Enable lockstep simulation
```

**Fields:**
- `backend` (string, required): Simulator type
  - `"gazebo"`: Gazebo Garden/Harmonic
  - `"jmavsim"`: jMAVSim (PX4 lightweight sim)
- `headless` (boolean, optional, default: false): Run without GUI
- `lockstep` (boolean, optional, default: false): Enable lockstep mode

## Complete Examples

### Example 1: Gazebo SITL (Development)

```toml
# Gazebo SITL - X500 Quadcopter
# Config ID: CFG-DEV-SITL-001

[app]
id = "sitl-gazebo-x500"
board = "sitl-gazebo"
airframe = "x500"
env = "sitl"

[telemetry]
frame_size = 280
queue_len = 32

[security]
profile = "none"

[[transports]]
protocol = "mavlink"
port = "udp"
roles = ["telemetry", "command"]
port_sensor = 14560
port_actuator = 14561

[simulator]
backend = "gazebo"
headless = false
lockstep = false
```

### Example 2: Hardware Flight (Production)

```toml
# MicoAir H743-V2 AIO - Quad-X Development Board
# Config ID: CFG-FLT-H743-001

[app]
id = "micoair-h743-v2-quad-x-dev"
board = "micoair-h743-v2"
airframe = "quad-x"
env = "flight"

[telemetry]
frame_size = 280
queue_len = 32

[security]
profile = "auth-only"  # Production: MUST use authentication

# USB for ground station telemetry + commands
[[transports]]
protocol = "mavlink"
port = "usb_cdc"
roles = ["telemetry", "command"]

# UART for CRSF RC receiver
[[transports]]
protocol = "crsf"
port = "uart1"
roles = ["rc_input"]
baudrate = 420000
```

### Example 3: HITL (Hardware-in-the-Loop)

```toml
# HITL - Real FC with simulated sensors
# Config ID: CFG-HITL-001

[app]
id = "hitl-micoair-h743-v2"
board = "micoair-h743-v2"
airframe = "quad-x"
env = "hitl"

[security]
profile = "none"  # Development only

[[transports]]
protocol = "mavlink"
port = "usb_cdc"
roles = ["telemetry", "command"]

[simulator]
backend = "jmavsim"
headless = true
lockstep = true
```

## Configuration Loading

### Embedded TOML (Recommended)

For embedded targets, embed the TOML directly in the binary:

```rust
// main.rs
const APP_CONFIG_TOML: &str = include_str!("../AviateApp.toml");

fn main() -> ! {
    // LOW-DAL init phase
    let config = aviate_config::from_toml_str(APP_CONFIG_TOML)
        .expect("invalid AviateApp.toml");

    aviate_config::validate(&config)
        .expect("config validation failed");

    // HIGH-DAL: Use typed AppConfig
    AppRuntime::<Board, Airframe>::run(&config)
}
```

### File-Based Loading (Desktop/SITL Tools)

For desktop tools with filesystem access:

```rust
use std::path::Path;

let config = aviate_config::load_config_from_path(
    Path::new("AviateApp.toml")
)?;
```

## Configuration Management (DO-178C)

### Config ID Format

Each configuration must have a unique Config ID:

**Development:**
- `CFG-DEV-SITL-001` (SITL development)
- `CFG-DEV-FLT-001` (Flight development)

**Production:**
- `CFG-FLT-001` (Flight, post-certification)
- `CFG-SITL-001` (Simulation, regression tests)

### Version Control

- Configuration files are versioned with application code
- Config ID tracks: app name, TOML path, git commit hash
- Changes to config = new Config ID
- `docs/CONFIG_IDS.md` maintains Config ID registry

### Configuration Changes

1. Edit `AviateApp.toml`
2. Assign new Config ID
3. Update `CONFIG_IDS.md`
4. Commit changes together
5. Rebuild app with new config

## Validation Rules

The parser automatically validates:

✅ Required fields present
✅ Environment is "flight", "sitl", or "hitl"
✅ Security profile is valid
✅ Transport roles are recognized
✅ Protocol types are supported
✅ Simulator backend is valid (if present)

Parse failure = immediate abort during init phase.

## Phase 2 Status

**Implemented:**
- ✅ Full TOML schema
- ✅ Parser (`from_toml_str`)
- ✅ Validation logic
- ✅ Example configs

**TODO (Phase 3+):**
- Runtime wiring based on transport config
- Board-specific port validation
- Config-driven security profile selection

## See Also

- `aviate-config` crate documentation
- `docs/DEPENDENCY_STRUCTURE.md` - DAL separation
- `docs/CONFIG_IDS.md` - Config ID registry (TODO)
