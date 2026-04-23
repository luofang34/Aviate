# Aviate Build System

The Aviate build system uses a three-tier architecture: configuration, runtime, and applications. It supports both SITL (Software-In-The-Loop) simulation and hardware flight targets with a single codebase.

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────────────┐
│  Applications (aviate-apps/*)                                       │
│  - Minimal entry points                                             │
│  - Load and validate AviateApp.toml                                 │
│  - Initialize board and run control loop                            │
└─────────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────────┐
│  Boards (aviate-boards/*)                                           │
│  - Board-specific initialization                                    │
│  - HAL trait implementations                                        │
│  - Sensor/actuator configuration                                    │
└─────────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────────┐
│  Runtime (aviate-runtime)                                           │
│  - Environment-specific control loops                               │
│  - SITL: SitlRunner with lockstep support                          │
│  - Flight: EmbeddedRunner with real-time scheduling                │
└─────────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────────┐
│  Core (aviate-core)                                                 │
│  - Flight controller kernel                                         │
│  - State estimation, control, mixing                               │
│  - Pure Rust, no_std, no I/O                                       │
└─────────────────────────────────────────────────────────────────────┘
```

## Environment Features

The build system uses compile-time features to select the target environment:

| Feature | Description | Use Case |
|---------|-------------|----------|
| `env-sitl` | Software simulation | Development, testing |
| `env-flight` | Real hardware | Production flight |
| `env-hitl` | Hardware-in-the-loop | Integration testing |

Features are mutually exclusive and automatically propagate through dependencies.

## Creating Applications

### Using cargo-generate (Recommended)

```bash
# Install cargo-generate if not already installed
cargo install cargo-generate

# Generate a new SITL application
cargo generate --path aviate-app-template --name my-quad

# Generate with specific options
cargo generate --path aviate-app-template --name my-flight \
  -d board=micoair-h743-v2 -d airframe=quad-x -d env=flight
```

### Template Options

| Option | Choices | Default |
|--------|---------|---------|
| board | sitl-gazebo, sitl-jmavsim, micoair-h743-v2 | sitl-gazebo |
| airframe | x500, quad-x, generic-quad | x500 |
| env | sitl, flight | sitl |

## Running Applications

### SITL (Simulation)

```bash
# Run with Gazebo
cargo run -p aviate-app-my-quad

# Run headless (CI/automated testing)
HEADLESS=1 cargo run -p aviate-app-my-quad

# Run test mission
cargo run -p aviate-app-my-quad -- --test tests/xil-missions/basic_flight.toml
```

### Flight (Hardware)

Single-command build and flash:

```bash
cargo xtask run my-app
```

This command:
1. Builds the app for `thumbv7em-none-eabihf` target
2. Converts ELF to binary with `arm-none-eabi-objcopy`
3. Enters DFU mode via serial command
4. Flashes firmware with `dfu-util`

Manual workflow:

```bash
# Build
cd aviate-apps/my-app
cargo build --release --target thumbv7em-none-eabihf

# Convert to binary
arm-none-eabi-objcopy -O binary \
  target/thumbv7em-none-eabihf/release/my-app app.bin

# Flash
cargo xtask flash app.bin
```

## Application Configuration

Each app has an `AviateApp.toml` file:

```toml
[app]
id = "my-quad"
board = "sitl-gazebo"
airframe = "x500"
env = "sitl"

[telemetry]
frame_size = 280
queue_len = 32

[security]
profile = "none"  # "none", "auth-only", "auth-and-encrypt"

[[transports]]
protocol = "mavlink"
port = "udp"
roles = ["telemetry", "command"]
```

See [APP_CONFIG.md](APP_CONFIG.md) for full configuration reference.

## Project Structure

```
aviate/
├── aviate-core/           # Flight controller kernel (no_std)
├── aviate-runtime/        # Environment-specific runners
├── aviate-config/         # Configuration parsing
├── aviate-link/           # Protocol abstraction (MAVLink)
├── aviate-security/       # Command authentication
├── aviate-hal/
│   ├── io/               # HAL traits + fake sensors
│   └── xil/              # SITL/HITL transports
│       └── backends/
│           ├── gz/       # Gazebo integration
│           └── mavlink-hil/  # jMAVSim integration
├── aviate-boards/
│   ├── sitl-gazebo/      # Gazebo SITL board
│   ├── sitl-jmavsim/     # jMAVSim SITL board
│   └── micoair-h743-v2/  # Hardware board
├── aviate-airframes/
│   └── multirotor/       # Quadcopter configs
├── aviate-apps/          # Application binaries
│   └── sitl-gazebo-x500/ # Example app
├── aviate-app-template/  # cargo-generate template
├── aviate-bootloader/    # Hardware bootloader
├── xtask/                # Development tools
├── tests/
│   └── xil-missions/     # Test mission configs
└── docs/                 # Documentation
```

## Verification Commands

Before committing changes:

```bash
# 1. Unit tests
cargo test --workspace

# 2. Coverage (100% required)
COVERAGE_MODE=branch ./scripts/check_coverage.sh

# 3. Static analysis
cargo clippy --workspace -- -D warnings
cargo fmt --all -- --check

# 4. SITL test
./scripts/run_sitl.sh tests/xil-missions/basic_flight.toml
```

## DO-178C Compliance Notes

The Aviate build system supports DAL-partitioned development:

- **HIGH-DAL (aviate-core)**: Pure computation, no I/O, bounded WCET
- **LOW-DAL (apps, runtime)**: I/O, error handling, can fail gracefully
- **Configuration Separation**: AviateApp.toml parsed at init, not runtime

Key architectural decisions:
- No dynamic allocation in aviate-core
- Time-deterministic data structures (ring buffers)
- Clear separation between provable and best-effort code
- Telemetry queue isolates HIGH-DAL control from LOW-DAL I/O

## Troubleshooting

### "cargo-generate not found"
```bash
cargo install cargo-generate
```

### "arm-none-eabi-objcopy not found"
```bash
# Ubuntu/Debian
sudo apt install gcc-arm-none-eabi

# macOS
brew install --cask gcc-arm-embedded
```

### "Build fails for hardware target"
Ensure the app Cargo.toml has:
```toml
[workspace]  # Standalone package
```

Hardware apps are excluded from the main workspace due to different targets.

### "SITL test hangs"
- Check Gazebo is installed: `gz sim --version`
- Check plugin is built: `ls aviate-hal/xil/backends/gz/plugin/build/`
- Kill stale processes: `pkill -f "gz sim"`
