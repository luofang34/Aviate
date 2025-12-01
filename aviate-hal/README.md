# aviate-hal

Hardware Abstraction Layer for Aviate flight systems.

## Quick Reference

| Crate | Purpose | DO-178C Scope |
|-------|---------|---------------|
| `aviate-hal/io` | I/O device traits + BoardHal | Flight software |
| `aviate-hal/xil` | X-In-Loop simulation infrastructure | Test tooling |

## Architecture

```
┌─────────────────────────────────────┐
│  aviate-apps / aviate-boards        │  Application layer
│  (flight controller main loop)      │
└─────────────────────────────────────┘
                  ↓
┌─────────────────────────────────────┐
│  BoardHal<I, B, M, G, T>            │  HAL composition layer
│  (implements SensorHal from core)   │
└─────────────────────────────────────┘
                  ↓
┌──────────────────┬──────────────────┐
│  Real Drivers    │  Fake Drivers    │  Driver layer
│  (embedded-hal)  │  (SITL / XIL)    │
└──────────────────┴──────────────────┘
                  ↓
┌──────────────────┬──────────────────┐
│  MCU Peripherals │  Simulator       │  Hardware/Simulation layer
│  (SPI/I2C/UART)  │  (Gazebo)        │
└──────────────────┴──────────────────┘
```

## Design Philosophy

**Same BoardHal for SITL and real hardware.**

The `BoardHal<I, B, M, G, T>` is generic over driver types:
- For SITL: `BoardHal<FakeImu, FakeBaro, FakeMag, FakeGnss, SitlTime>`
- For real hardware: `BoardHal<Icm426xx<I2C>, Bmp390<SPI>, Qmc5883l<I2C>, UbloxGnss<UART>, HwTime>`

Both instantiations implement `SensorHal`, allowing the flight controller kernel to remain
completely unaware of whether it's running in simulation or on real hardware.

```
Real HW:  SPI/I2C → Driver → BoardHal → SensorHal → Kernel
SITL:     MAVLink → FakeDriver → BoardHal → SensorHal → Kernel
                    ↑
          Same code path from here onward
```

## DO-178C Boundary

The HAL crates are split to clearly separate flight software from test tooling:

### Flight Software (deployed on aircraft)
- `aviate-core` - Flight control algorithms, state estimation
- `aviate-hal/io` - I/O device traits, BoardHal
- `aviate-boards/*` - Board-specific driver instantiation

### Test Tooling (not deployed)
- `aviate-hal/xil` - X-In-Loop simulation infrastructure
- `aviate-hal/xil/backends/gz` - Gazebo backend

This separation ensures that test infrastructure cannot accidentally be included in
flight-critical builds. The `xil` crate requires `std` and is excluded from `no_std` builds.

## Sub-crates

### [aviate-hal/io](io/README.md)

Platform-agnostic I/O device framework:
- Driver traits: `ImuDriver`, `BaroDriver`, `MagDriver`, `GnssDriver`
- Raw readings: `RawImuReading`, `RawBaroReading`, etc.
- `BoardHal<I,B,M,G,T>`: Generic composition implementing `SensorHal`
- Fake drivers: `FakeImu`, `FakeBaro`, etc. for SITL

### [aviate-hal/xil](xil/README.md)

X-In-Loop (SITL/HITL) simulation infrastructure:
- `SitlMavlink`: MAVLink I/O (receives HIL_SENSOR, sends HIL_ACTUATOR)
- `KinematicsBackend` trait: Interface for simulators (Gazebo, future: AirSim, Unity)
- Test framework: Mission definitions, success criteria, multi-vehicle support
- Flight logging and statistics

## Usage

See individual sub-crate READMEs for detailed usage examples:
- [BoardHal and driver traits](io/README.md)
- [SITL simulation setup](xil/README.md)
- [Adding a new board](../aviate-boards/README.md)
