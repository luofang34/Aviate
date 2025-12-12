# Aviate Application Template

Generate new Aviate flight controller applications with a single command.

## Prerequisites

Install cargo-generate:
```bash
cargo install cargo-generate
```

## Usage

### Generate a new SITL application (default)

```bash
# From the Aviate project root:
cargo generate --path aviate-app-template --name my-quad

# The app is created in aviate-apps/my-quad/
```

### Generate with specific options

```bash
# Gazebo SITL with X500 airframe
cargo generate --path aviate-app-template --name my-sitl \
  -d board=sitl-gazebo -d airframe=x500 -d env=sitl

# jMAVSim SITL
cargo generate --path aviate-app-template --name my-jmavsim \
  -d board=sitl-jmavsim -d airframe=quad-x -d env=sitl

# Hardware flight (MicoAir H743-V2)
cargo generate --path aviate-app-template --name my-flight \
  -d board=micoair-h743-v2 -d airframe=quad-x -d env=flight
```

## Running Applications

### SITL (Simulation)

```bash
# Run Gazebo SITL
cargo run -p aviate-app-my-sitl

# Run with test mission
cargo run -p aviate-app-my-sitl -- --test tests/xil-missions/basic_flight.toml
```

### Flight (Hardware)

```bash
# Build and flash in one command
cargo xtask run my-flight

# Or manually:
cd aviate-apps/my-flight
cargo build --release
cargo xtask flash target/thumbv7em-none-eabihf/release/my-flight
```

## Template Options

| Option | Choices | Default | Description |
|--------|---------|---------|-------------|
| board | sitl-gazebo, sitl-jmavsim, micoair-h743-v2 | sitl-gazebo | Target board |
| airframe | x500, quad-x, generic-quad | x500 | Airframe type |
| env | sitl, flight | sitl | Runtime environment |

## Generated Files

| File | Description |
|------|-------------|
| Cargo.toml | Package manifest with board-specific dependencies |
| AviateApp.toml | Application configuration (telemetry, security, transports) |
| src/main.rs | Entry point with board initialization |
| memory.x | Linker script (flight only) |
| build.rs | Build script (flight only) |
| .cargo/config.toml | Cargo config for embedded target (flight only) |
