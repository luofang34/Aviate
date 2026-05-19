# AVIATE-CORE Behavioral Requirements (DRAFT)

The existing 46 LLRs in `cert/trace/llr.toml` are **structural** — they assert facts
about the shape of the code (one state owner, replaceable estimator, immutable
config, cross-channel snapshot projection). This document proposes the
**behavioral** complement: what the kernel must DO in flight, not how its code
is shaped.

Status: DRAFT for review. Nothing has been written to `cert/trace/*.toml` yet.
Once the HLR shape below is agreed, the next PR lands SYS/HLR/LLR/TST rows.

Verification tags below:
- **U** — unit / integration test in `aviate-core/tests/` with synthetic
  dynamics. Fast, deterministic, cert-grade primary evidence.
- **X** — XIL scenario run in Gazebo SITL. Supplementary integrated proof.
  Requires the Gazebo stack restored (see §6).

Most rows are **U+X**: U is the primary cert evidence; X confirms the same
behavior under integrated physics.

---

## 1. System-level additions (SYS)

These complement the existing six SYS rows (which are all structural).

| ID | Title |
|----|-------|
| `SYS-EST-002` | Estimator delivers bounded state error in nominal flight |
| `SYS-CTL-001` | Stabilization closes the loop within authority and time bounds |
| `SYS-MIX-001` | Mixer maps axis commands to bounded, geometrically-valid actuator commands |
| `SYS-FLT-001` | Faults produce defined, latched, safe responses |
| `SYS-MORPH-001` | Mode and geometry transitions do not break the control loop |
| `SYS-INIT-001` | Kernel cold-start and arm sequence is deterministic |
| `SYS-ARM-001` | Disarmed kernel produces safe (zero/idle) actuator output |

## 2. High-level requirements (HLR) — behavioral

200-series numbering keeps these distinct from the existing 001-series
structural HLRs. Numeric thresholds below are starting points to be tuned by
the implementer against the actual airframe and gains; the cert evidence is
that each requirement HAS a measured number and a test that holds the line.

### 2.1 Estimation behavior

| ID | Requirement | V |
|----|-------------|---|
| `HLR-EST-201` | Attitude estimate converges within ±2° of truth within 10 s of cold-start when accelerometer + gyro + mag are static-valid. | U+X |
| `HLR-EST-202` | Attitude estimate tracks ground-truth angular rate with steady-state error ≤ 1 °/s during ±30 °/s smooth maneuvers. | X |
| `HLR-EST-203` | Position estimate (NED) tracks ground truth ≤ 0.5 m horizontal, ≤ 0.3 m vertical when GNSS reports `3D Fix` and baro is healthy. | X |
| `HLR-EST-204` | Estimator latches `NUMERIC_ERROR` within one cycle of receiving a non-finite IMU sample and clears it only via explicit `ground_reset()`. | U |
| `HLR-EST-205` | Estimator survives a 5 s GNSS dropout without divergence (position error growth ≤ 2 m, attitude unaffected). | X |

### 2.2 Stabilization behavior

| ID | Requirement | V |
|----|-------------|---|
| `HLR-CTL-201` | Rate controller tracks rate-setpoint with steady-state error ≤ 0.5 °/s under a ±60 °/s smooth command profile. | U+X |
| `HLR-CTL-202` | Attitude loop response to a 10° roll step: overshoot ≤ 30 %, settle (±5 %) ≤ 1.0 s, no sustained oscillation. | U+X |
| `HLR-CTL-203` | Hover holds altitude within ±0.3 m and horizontal position within ±0.5 m over a 30 s window in still air. | X |
| `HLR-CTL-204` | Authority limits (`max_tilt_rad`, `max_rate_rad_s`, `max_vertical_accel_mss`) are honored when the input command saturates beyond them. | U |

### 2.3 Mixing / actuation

| ID | Requirement | V |
|----|-------------|---|
| `HLR-MIX-201` | `QuadXMixer::mix()` produces actuator outputs in [0.0, 1.0] for every axis command whose components are within authority. | U (property) |
| `HLR-MIX-202` | For zero axis command (roll=pitch=yaw=0, hover throttle T) all four motors output T (geometric symmetry). | U |
| `HLR-MIX-203` | When an actuator group is reported `Unhealthy`, sanitizer falls back to the configured safe pattern within one cycle and `consecutive_fallback` increments monotonically. | U |

### 2.4 Faults & safety

| ID | Requirement | V |
|----|-------------|---|
| `HLR-FLT-201` | Disarmed kernel produces zero (or configured idle) actuator output regardless of estimator state or commanded setpoint. | U+X |
| `HLR-FLT-202` | `arm()` returns `Err(ArmError::*)` and remains disarmed when any pre-arm check fails (estimator uninitialized, sensors stale, watchdog not kicked, command stale at boot). | U |
| `HLR-FLT-203` | When `NUMERIC_ERROR` is latched, the controller emits configured safe-output and no further actuator changes occur until `ground_reset()`. | U+X |
| `HLR-FLT-204` | When `command_age_ms > command_timeout_ms`, kernel transitions to safe mode within one cycle; no actuator runaway. | U+X |

### 2.5 Mode / geometry morph

| ID | Requirement | V |
|----|-------------|---|
| `HLR-MORPH-201` | `ConfigMode` transition (e.g., Hover→Cruise) is atomic w.r.t. one control cycle: cycle N output reflects exactly mode A or mode B, never partial. | U |
| `HLR-MORPH-202` | A `GeometryState` change during flight does not produce a discontinuous actuator command spike (delta ≤ configured slew limit). | U+X |

### 2.6 Bootstrap

| ID | Requirement | V |
|----|-------------|---|
| `HLR-INIT-201` | Cold-start replay: identical sensor input + identical config produce bit-identical `KernelState` at cycle N for every N. | U |
| `HLR-INIT-202` | Pre-arm checks include at minimum: estimator initialized + numerically clean; config loaded + `canonical_hash` matches; watchdog kicked within deadline; all sensor channels `Healthy` within last 1 s. | U |

## 3. LLR shape (decomposition is per-HLR; written when HLR set is approved)

Each HLR decomposes into 1–4 LLRs in the 200-series. Examples:
- `HLR-EST-201` → `LLR-EST-201` (init branch in `EkfState::observe`),
  `LLR-EST-202` (`is_initialized()` after N valid cycles), `LLR-EST-203`
  (accelerometer-based attitude seed correctness).
- `HLR-CTL-202` → `LLR-CTL-201` (attitude-loop PID gain selection),
  `LLR-CTL-202` (anti-windup behavior), `LLR-CTL-203` (rate-feedback shaping).
- `HLR-FLT-204` → `LLR-FLT-201` (`command_age_ms` comparison site),
  `LLR-FLT-202` (safe-mode transition), `LLR-FLT-203` (latched until
  `ground_reset()`).

Estimated final count: 40–50 new LLRs.

## 4. Existing test coverage vs gaps

| HLR cluster | Existing test that likely covers it | Gap to close |
|---|---|---|
| Estimation (201, 204) | `aviate-core/tests/ekf_tests.rs` | Confirm NaN-latch test exists; add cold-start convergence assertion |
| Estimation (202, 203, 205) | none direct | Needs XIL — Gazebo scenarios |
| Stabilization (201, 202, 204) | `control_attitude.rs`, `control_rate.rs`, `control_envelope.rs` | Add closed-loop step-response harness with kinematic model |
| Stabilization (203) | none direct | XIL primary; U via kinematic surrogate optional |
| Mixing (201–203) | `mixer_tests.rs` | Verify symmetry test exists; otherwise add |
| Faults (201–204) | `kernel.rs`, `fault.rs` | Add command-timeout, NaN-mid-hover, disarm-with-nonzero-cmd tests |
| Morph (201, 202) | none obvious | Add atomic-transition + slew-limit tests |
| Init (201) | none direct | Add bit-replay determinism harness |
| Init (202) | `kernel.rs` arm path | Verify each pre-arm clause is asserted individually |

## 5. Gazebo SITL restoration (prerequisite for every X row)

State of the gz-sim plugin tree at the time the draft was authored:
- `external/PX4-gazebo-models` submodule is registered but uninitialized
  (`-947f75b…` in `git submodule status`).
- C++ plugin source IS present at `aviate-hal/xil/backends/gz/plugin/`
  (`AviateGzPlugin.cc/.hh`, `aviate_gz_bridge.cc/.h`, `MotorFailurePlugin.cc/.hh`,
  `CMakeLists.txt`, `gz_ffi.rs`, `README.md`).
- `cargo check -p gcs-test --features gazebo` fails: unresolved import
  `aviate_app_sitl_gazebo_x500` (crate was deleted in `0b63a22` "xtask sitl").
  Referenced symbols: `generate_temp_world`, `WorldParams`.

Restoration sequence:
1. `git submodule update --init external/PX4-gazebo-models` (registers X500 model + worlds).
2. Build the gz plugin: requires Gazebo Harmonic + gz-sim dev headers. Add `scripts/install_gz_deps.sh` documenting the install (`apt install libgz-sim8-dev` on Ubuntu; brew formula on macOS if available). `cd plugin && mkdir build && cd build && cmake .. && make`.
3. Restore world-gen helpers. Recommend **inlining** into `tests/gcs-test/src/world.rs` (gated by `feature = "gazebo"`) rather than restoring the deleted `aviate-app-sitl-gazebo-x500` crate — fewer moving parts, matches the comment already in the root `Cargo.toml` ("gz flow now lives behind `gcs-test`'s `gazebo` feature + the gz plugin alone").
4. Add a smoke scenario `tests/missions/gz_takeoff_hover_land.toml` that arms, climbs to 5 m, hovers 5 s, lands, disarms.
5. CI gate: add `cargo check -p gcs-test --features gazebo` (and the plugin build, conditionally) to the workspace CI so this regression can't repeat silently.

Estimated cost: 1 focused day for steps 1–4, 0.5 day for the CI gate.

## 6. Proposed PR sequence

1. **`cert: behavioral SYS/HLR draft`** — land §1 SYS rows + §2 HLR rows in `cert/trace/`. LLRs and TSTs deferred to subsequent PRs to keep the diff reviewable. No code changes. Bumps `floors.toml` accordingly.
2. **`xil: restore gazebo SITL`** — items 1–5 from §5. Standalone, no cert deltas; gives us a green `cargo check -p gcs-test --features gazebo` + a runnable smoke scenario.
3. **`cert: behavioral unit tests + LLR/TST`** — add the missing U-tagged tests in `aviate-core/tests/`, write the corresponding LLR + TST rows, ratchet floors. Cycles cleanly through `evidence_check` and `evidence_doctor`.
4. **`cert: gazebo XIL scenarios`** — author the X-tagged scenario suite under `tests/scenarios/gazebo/`, wire each into the relevant TST `traces_to`. Run green. Ratchet floors.
5. **`cert: floors final + evidence bundle`** — final floors equal current; `cargo evidence check --mode=bundle` green.

Each PR is independently revertible. Each lands the change plus its guardrail
(new test = new floor line; new scenario = CI invocation).
