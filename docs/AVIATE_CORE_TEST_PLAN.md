# AVIATE-CORE Test Plan

This document is the comprehensive verification plan for `aviate-core`.
It links every behavioral requirement in `cert/trace/hlr.toml` (200-series)
to the tests that witness it and records the current pass / fail status.

The plan has three verification tiers:

| Tier | Where it runs | What it proves | Best at |
|---|---|---|---|
| **U** — unit / integration | `cargo test -p aviate-core` | Cycle-level kernel behavior with synthetic dynamics. Deterministic, bit-repeatable. | Logical contracts (NaN latch, atomic mode swap, pre-arm gates, encoding stability). |
| **X** — XIL / SITL mission | `cargo run -p gcs-test --features gazebo -- run --xil tests/missions/<name>.toml` | End-to-end FC ↔ gz-sim loop with real x500 physics, MAVLink over UDP, shared-memory bridge. | Integration health (the kernel actually arms in flight, motors actually drive lift, EKF converges against gz sensor noise). |
| **DRQ** — derived | `cert/trace/derived.toml` | Documented gap between an HLR's target threshold and what the implementation achieves. | Honest disclosure when a contract is structurally provable but the *numeric* threshold needs controller-gain tuning to meet. |

## Verification matrix

200-series LLRs decompose 200-series HLRs (see `cert/trace/hlr.toml`).
This table maps each behavioral HLR to its verification tier and the
artifact that witnesses it.

| HLR | Title | Tier | Witness | Status |
|---|---|---|---|---|
| `HLR-EST-201` | Static cold-start attitude convergence | U | `behavioral_tests::ekf_cold_start_converges_under_static_imu` | PASS |
| `HLR-EST-202` | Angular-rate tracking in maneuvering flight | U | `ekf_tests::ekf_predict_integrates_angular_velocity` | PASS |
| `HLR-EST-203` | Position estimate tracks GNSS+baro | X (smoke) + DRQ | `tests/missions/position_hold.toml` + DRQ-CTL-002 | PASS smoke; cert-grade DRQ |
| `HLR-EST-204` | NUMERIC_ERROR latch on non-finite IMU | U | 3× `ekf_tests::ekf_predict_with_*_returns_early` | PASS |
| `HLR-EST-205` | Bounded GNSS-dropout dead-reckoning | U + X | `behavioral_tests::ekf_gnss_dropout_bounded_dead_reckoning_drift` + `tests/missions/gnss_dropout.toml` | PASS |
| `HLR-CTL-201` | Rate-loop steady-state tracking | U | `multirotor_controller_tests` + `control_rate` + `control_attitude` suites | PASS |
| `HLR-CTL-202` | Attitude step response (overshoot, settle) | U + X (smoke) + DRQ | `control_rate.rs` shape tests + `tests/missions/attitude_control.toml` + DRQ-CTL-002 | PASS smoke; cert-grade DRQ |
| `HLR-CTL-203` | Hover hold within bounds | X (smoke) + DRQ | `tests/missions/hover_stability.toml` + DRQ-CTL-002 | PASS smoke; cert-grade DRQ |
| `HLR-CTL-204` | Authority-envelope clamping | U | `control_envelope` suite | PASS |
| `HLR-MIX-201` | Bounded actuator outputs [0, 1] | U | `mixer_tests::test_quad_mixer_saturation` | PASS |
| `HLR-MIX-202` | QuadX bit-symmetric hover | U | `mixer_tests::test_quad_mixer_hover` | PASS |
| `HLR-MIX-203` | Sanitizer same-cycle fallback substitution | U | `mixer_tests::test_sanitizer_last_good_fallback` + critical-failure test | PASS |
| `HLR-FLT-201` | Disarmed kernel emits safe pattern | U + X | `kernel::kernel_outputs_safe_when_not_armed` + `tests/missions/basic_flight.toml` arm/disarm | PASS |
| `HLR-FLT-202` | Disarmed output independent of inputs | U | `behavioral_tests::disarmed_safe_output_is_independent_of_inputs` | PASS |
| `HLR-FLT-203` | Pre-arm gates → typed `ArmError` | U | 7× `kernel::kernel_arm_fails_*` | PASS |
| `HLR-FLT-204` | `ArmError` discriminants 1:1 with gates | U | `kernel::arm_error_all_variants_distinct` | PASS |
| `HLR-FLT-205` | NUMERIC_ERROR inhibits update output | U | `behavioral_tests::numeric_fault_latched_inhibits_actuator_output` | PASS |
| `HLR-FLT-206` | `ground_reset()` is the only clear path | U | 3× `kernel::test_ground_reset_*` | PASS |
| `HLR-FLT-207` | Command-age clears `COMMAND_RECENT` | U | `kernel::update_command_age_gates_command_recent_flag` | PASS |
| `HLR-FLT-208` | Command-loss safe slew limit | DRQ | DRQ-FLT-001 (slew-limit unimplemented) | DEFERRED |
| `HLR-FLT-209` | Command-recovery re-engages active law | U | `update_command_age_gates_command_recent_flag` (boundary case) | PASS |
| `HLR-MORPH-201` | Atomic ConfigMode swap | U | `behavioral_tests::config_mode_request_atomicity_under_pre_conditions` | PASS |
| `HLR-MORPH-202` | GeometryState slew-limited delta | DRQ | DRQ-MORPH-001 (geometry slew unimplemented) | DEFERRED |
| `HLR-INIT-201` | Bit-deterministic cold-start replay | U | `replicable_tests` | PASS |
| `HLR-INIT-202` | Pre-arm gates individually testable | U | Same 7 `kernel::kernel_arm_fails_*` tests as HLR-FLT-203 | PASS |

## Tier U — unit / integration tests

Located in `aviate-core/tests/`:
- `behavioral_tests.rs` (5) — new behavioral suite from this PR. NUMERIC_ERROR inhibition, disarmed-safe input-independence, atomic mode swap, EKF cold-start convergence, EKF GNSS-dropout bounded drift.
- `ekf_tests.rs` (43), `kernel.rs` (116), `mixer_tests.rs` (24), `control_*.rs` (~80), `fault.rs` (29), `replicable_tests.rs` (20), `integration_tests.rs` (10), and friends.

Run: `cargo test --workspace`. **Current state: 862 unit tests pass, 0 fail.**

## Tier X — SITL missions

Mission TOMLs live in `tests/missions/`. Each runs through `gcs-test
--features gazebo` against a freshly-spawned `gz sim` instance with
the AviateGzPlugin loaded. The Aviate FC binary (`sitl-gazebo-x500`)
runs in a second process, reads ground-truth model state from the
plugin via POSIX shared memory, synthesizes IMU + baro + mag + GNSS,
runs the kernel, and writes motor commands back.

| Mission | Phases | Witnessed HLRs | Stability (this PR) |
|---|---|---|---|
| `basic_flight` | arm → takeoff → hover → land → disarm | HLR-FLT-201 (arm/disarm) | 3/3 PASS |
| `hover_stability` | arm → takeoff → hover_hold (8 s) → descend → land → disarm | HLR-CTL-203 (smoke) | 3/3 PASS |
| `attitude_control` | takeoff → level_hover → pitch_forward → return_level → roll_right → stabilize → descend → land → disarm | HLR-CTL-202 (smoke) | 3/3 PASS |
| `position_hold` | takeoff → hold_origin → move_north → hold_waypoint → return_origin → descend → land → disarm | HLR-EST-203, HLR-CTL-203 (smoke) | 3/3 PASS |
| `gnss_dropout` | takeoff → hover_warmup → inject_gnss_lost → hover_under_dropout → clear_faults → hover_after_recovery → land → disarm | HLR-EST-205 | 5/5 PASS |

Stability sweeps from this PR: 100 % pass across 17 runs (3 per
mission × 5 missions, plus a 5-run gnss_dropout deep-stability
check). See `scripts/run_sitl_missions.sh` for the harness.

Stability is verified by running each mission ≥ 3 times and recording
the consensus. The thresholds in `tests/missions/*.toml` are set at
**integration smoke level** — the tests prove the FC ↔ gz-sim loop
closes and motors drive lift. Cert-grade hover precision (cm-level)
is tracked by **DRQ-CTL-002** and verified by tuning the controller
gains in a follow-up PR. The integration-level pass is the
prerequisite — controller tuning happens against a known-good
integration substrate.

### How to run

```bash
# All-in-one (build + run plugin + spawn FC + run mission):
cargo run -p gcs-test --features gazebo -- run --xil tests/missions/basic_flight.toml

# Multi-run stability sweep:
for m in basic_flight hover_stability attitude_control position_hold gnss_dropout; do
    for r in 1 2 3; do
        cargo run --quiet -p gcs-test --features gazebo -- run --xil tests/missions/${m}.toml
    done
done
```

Prerequisites (macOS, Homebrew):
- `brew install gz-harmonic` (provides gz-sim8 + gz-msgs10 + gz-transport13).
- `git submodule update --init external/PX4-gazebo-models`.
- `cd aviate-hal/xil/backends/gz/plugin && rm -rf build && mkdir build && cd build && CMAKE_PREFIX_PATH=/opt/homebrew:/opt/homebrew/opt/qt@5 cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_IGNORE_PATH=/Users/fangluo/anaconda3 .. && make`.

On Linux CI the same `cmake … && make` (without the anaconda
exclude) works against system gz-sim / protobuf packages.

## Tier DRQ — deferred cert-grade thresholds

The SITL runs surface where the current control-gain tuning falls
short of HLR thresholds. The honest disclosure lives in
`cert/trace/derived.toml`:

- **DRQ-CTL-001** (already in tree): tunable PID gains belong in
  `ResolvedKernelConfig`, not on the controller struct.
- **DRQ-CTL-002** (added this PR): controller-gain tuning to meet
  HLR-CTL-203's 0.3 m altitude / 0.5 m horizontal hover bounds, and
  the per-attitude min-altitude bounds in `attitude_control` /
  `position_hold` missions. Current thresholds are smoke-level —
  the SITL verifies integration health, not flight quality.
  Tuning passes target hardware-style hover trim + per-mode safe
  pattern + cascaded gain refinement, blocked by DRQ-CTL-001
  (centralizing gains in cfg) for byte-equal cross-channel
  verification.
- **DRQ-FLT-001** (added this PR): command-loss slew limit
  (HLR-FLT-208) — the kernel currently switches to safe-mode
  output in one cycle, no slew limiter between previous and new
  actuator command. Adding the slew limiter requires a new
  `ResolvedKernelConfig.slew_limit_per_cycle` field threaded into
  the mixer's per-cycle write.
- **DRQ-MORPH-001** (added this PR): geometry-state slew limit
  (HLR-MORPH-202) — same machinery as DRQ-FLT-001 but applied to
  airframe-morph parameter updates.

## What this plan does and does not assert

**Does assert:**
- Every 200-series LLR has a witness (unit test, mission, or DRQ
  acknowledging the gap).
- Every 200-series LLR with a `verification_methods = ["test"]`
  attribute has at least one passing test (unit or mission), with
  the test name recorded in `cert/trace/tests.toml`.
- The full FC ↔ gz-sim integration loop closes on a Mac dev machine
  and on Linux CI.

**Does not assert:**
- That the current controller gains meet cert-grade flight quality.
  Cert-grade thresholds are tracked as DRQs; the SITL passes here
  prove integration health, not control quality.
- That the kernel meets DO-178C Level B without further evidence
  beyond this trace tree. Coverage analysis, formal review, and
  the configuration-management artifacts are subsequent work
  (`cert/SHA256SUMS`, `cert/REVIEW_LOG.md`, etc.).
- That the SITL missions characterize hardware behavior. The
  `aviate-hal-stm32h7` + `aviate-boards/micoair-h743-v2` flight
  build is a separate target lane verified separately.

## Maintenance

- **Adding a new LLR**: add a row above, link to a witness, run
  `cargo evidence trace --validate`. Bump the appropriate floor in
  `cert/floors.toml` so the new entry is locked.
- **Tightening a SITL threshold**: edit the mission TOML, run
  3-run stability, update the matrix's "Status" column.
- **Closing a DRQ**: deleting a DRQ row requires linking the
  passing test that proves the requirement now holds, plus a
  changelog entry in this file's "Maintenance" section.
