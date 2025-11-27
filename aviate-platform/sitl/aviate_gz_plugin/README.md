# Aviate Gazebo Plugin

Zero-copy bridge between gz-sim (Gazebo Harmonic) and Aviate SITL via shared memory.

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                        gz-sim (Gazebo)                          │
│  ┌─────────────────────────────────────────────────────────┐    │
│  │              AviateGzPlugin (System Plugin)              │    │
│  │  - Reads pose/velocity from EntityComponentManager       │    │
│  │  - Writes to shared memory at physics rate (1kHz)        │    │
│  │  - Reads motor commands from shared memory               │    │
│  └───────────────────────┬─────────────────────────────────┘    │
└──────────────────────────┼──────────────────────────────────────┘
                           │ POSIX Shared Memory
                           │ /aviate_gz_bridge
┌──────────────────────────┼──────────────────────────────────────┐
│  ┌───────────────────────▼─────────────────────────────────┐    │
│  │           libaviate_gz_bridge.so (C Interface)          │    │
│  │  - aviate_gz_get_model_state()                          │    │
│  │  - aviate_gz_set_motor_speeds()                         │    │
│  └───────────────────────┬─────────────────────────────────┘    │
│                          │ Rust FFI                              │
│  ┌───────────────────────▼─────────────────────────────────┐    │
│  │               GzBridge (Rust wrapper)                    │    │
│  │  - Safe Rust API                                         │    │
│  │  - ENU to NED coordinate conversion                      │    │
│  └─────────────────────────────────────────────────────────┘    │
│                      Aviate SITL                                 │
└─────────────────────────────────────────────────────────────────┘
```

## Building

```bash
cd aviate_gz_plugin
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
```

## Usage

### 1. Add Plugin to World SDF

```xml
<world name="aviate_sitl">
  <!-- ... models ... -->

  <plugin filename="AviateGzPlugin" name="aviate::AviateGzPlugin">
    <model_name>x500</model_name>
  </plugin>
</world>
```

### 2. Use from Rust

```rust
use aviate_gz_plugin::{GzBridge, enu_to_ned};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Wait for plugin to be ready
    let bridge = GzBridge::connect_with_retry(10, 500)?;

    loop {
        // Read ground truth state
        if let Some(state) = bridge.get_model_state() {
            let ned_pos = enu_to_ned(state.pos);
            println!("Position (NED): x={:.2}, y={:.2}, z={:.2}",
                     ned_pos[0], ned_pos[1], ned_pos[2]);
        }

        // Send motor commands
        bridge.set_motor_speeds(&[700.0, 700.0, 700.0, 700.0])?;

        std::thread::sleep(std::time::Duration::from_millis(4));
    }
}
```

## Coordinate Systems

- **Gazebo**: ENU (East-North-Up)
  - X = East
  - Y = North
  - Z = Up

- **MAVLink/Aviate**: NED (North-East-Down)
  - X = North
  - Y = East
  - Z = Down

Use `enu_to_ned()` and `enu_vel_to_ned()` helper functions for conversion.

## Shared Memory Structure

The plugin and bridge communicate via POSIX shared memory (`/aviate_gz_bridge`):

```c
struct SharedState {
    // Model state (written by plugin)
    double pos[3];       // Position [x, y, z] meters
    double quat[4];      // Quaternion [w, x, y, z]
    double vel[3];       // Linear velocity [vx, vy, vz] m/s
    double ang_vel[3];   // Angular velocity [wx, wy, wz] rad/s
    uint64_t time_us;    // Simulation time (microseconds)
    uint32_t seq;        // Sequence number
    uint32_t valid;      // Data valid flag

    // Motor commands (written by Rust)
    double motor_vel[8]; // Motor velocities (rad/s)
    int num_motors;      // Number of motors
    uint32_t motor_seq;  // Command sequence number

    // Status
    uint32_t plugin_ready;
};
```

## Performance

- **Latency**: ~1μs (shared memory read)
- **Update rate**: Physics rate (typically 1kHz)
- **Zero-copy**: No serialization overhead
- **Thread-safe**: Atomic sequence numbers for synchronization

## Files

- `AviateGzPlugin.hh/cc` - gz-sim System plugin (runs inside Gazebo)
- `aviate_gz_bridge.h/cc` - C interface for FFI
- `gz_ffi.rs` - Rust FFI bindings
- `CMakeLists.txt` - Build configuration
