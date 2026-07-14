# Aviate

> ⚠️ **Work in progress — experimental. No guarantees of any kind.**
>
> Aviate is under active development and is **not** production-ready. Nothing here has been qualified or certified for any use. It is provided **as-is, with no warranty or guarantee of any kind** — express or implied — including (without limitation) correctness, safety, reliability, fitness for a particular purpose, or airworthiness. Do not deploy it on real hardware or rely on it in any safety-critical context. APIs, behavior, and file layout may change without notice.

Aviate is a minimal, deterministic, hard-real-time motion control kernel responsible only for state estimation and stabilization control — not navigation, communication, mission management, or human-machine interface.


## 2. Design Philosophy

### 2.1 Aviate Does Exactly Three Things

1. **State Estimation** — Attitude, position, velocity, angular rate estimation
2. **Stabilization & Control** — Rate loop, attitude loop, velocity/altitude/position hold
3. **Actuation Output** — Force/torque commands mapped to actuator outputs via mixer

### 2.2 Aviate Never Does
- ❌ Navigation (waypoints, procedures, LNAV/VNAV)
- ❌ Mission systems / autopilot management
- ❌ Maps / charts / databases
- ❌ Networking (TCP/UDP/WiFi/LTE)
- ❌ File systems / logging
- ❌ UI / GCS / cloud platforms
- ❌ Operating system dependencies

## 3. Testing

Aviate uses `cargo xtask` for managing SITL (Software-In-The-Loop) tests.

### Running Tests

```bash
# Run a specific mission test
cargo xtask test tests/missions/basic_flight.toml

# Run multi-vehicle formation test
cargo xtask test tests/missions/two_vehicle_formation.toml
```

This command automatically:
1. Cleans up lingering SITL processes (`gz`, `sitl-gazebo`, `mavrouter`).
2. Rebuilds necessary binaries.
3. Launches Gazebo, flight controllers, and MAVLink router.
4. Executes the test mission.

### Environment Configuration

- **`XIL_BASE_PORT`**: Sets the base UDP port for SITL instances (default: 20000). Use this if you encounter "Address already in use" errors or need to run parallel tests.
  ```bash
  XIL_BASE_PORT=30000 cargo xtask test tests/missions/two_vehicle_formation.toml
  ```
- **`HEADLESS`**: Set to `false` to enable Gazebo GUI (default: `true`).
- **`RUST_LOG`**: Set to `info` or `debug` for verbose output.

### Manual Cleanup

If a test crashes or hangs, you can manually force clean the environment:

```bash
cargo xtask cleanup
```

## License

Aviate is dual-licensed under either of:

- MIT license ([LICENSE-MIT](LICENSE-MIT))
- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE))

at your option. You may use Aviate under the terms of either license.

Third-party components vendored under `external/` are licensed separately under their own terms; see the license files in those directories. Their inclusion does not license Aviate itself.
