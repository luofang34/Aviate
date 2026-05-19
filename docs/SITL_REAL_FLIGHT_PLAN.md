# SITL Real-Flight Test Harness — Plan

The current SITL harness reports `Result: PASS` for missions where
the vehicle briefly lifted off and crashed back. The harness needs
to demonstrate **actual** flight: a clean takeoff, a hover that
stays within a stationkeeping box, intended-course maneuvers, and a
controlled landing — verified by both Gazebo ground truth AND the
MAVLink telemetry the kernel itself reports.

## Architecture invariants

The redesign keeps the existing module separation:

```
            ┌────────────────────────────┐
            │ tests/missions/*.toml      │  ← mission DSL
            └─────────────┬──────────────┘
                          │
            ┌─────────────▼──────────────┐
            │ tests/gcs-test             │  ← MAVLink-only driver
            │   • spawns gz + FC         │      (no FC internals)
            │   • drives via MAVLink     │
            │   • reads ground truth     │
            │     from gz bridge         │
            │   • reads telemetry from   │
            │     FC's MAVLink stream    │
            └─────────────┬──────────────┘
                          │ MAVLink UDP
            ┌─────────────▼──────────────┐
            │ aviate-apps/sitl-gazebo-x500│ ← FC binary (black box)
            │   • aviate-runtime          │    to the harness
            │   • aviate-core kernel      │
            │   • aviate-hal-xil bridge   │
            └─────────────────────────────┘
```

- The harness never reaches into kernel internals; verification is
  via the two external observation channels (gz ground truth, FC
  MAVLink telemetry).
- The FC binary is the unit-under-test; controllers, EKF, and
  bridges all live inside it.
- The mission TOML is a declarative artifact; new actions/criteria
  are added through the parser, not by editing the runner.

## Cert-evidence chain

Each mission row in `cert/trace/tests.toml` cites:
1. The HLR/LLR(s) it witnesses (e.g. HLR-CTL-203 hover-hold).
2. The criterion the mission asserts (drift bound, attitude bound).
3. The execution command, reproducible offline.

A mission passes only when **both** observation channels report
within tolerance — disagreement between gz truth and FC telemetry
is itself a fault.

## The cascaded-flight contract

A multirotor SITL that "actually flies" needs five things working
together:

1. **Hover-thrust calibration**: the trim where motor lift =
   airframe weight. X500: 0.77 normalized (`sqrt(20.3/34.2)`).
2. **Cold-start attitude**: EKF starts with a quaternion that
   matches the actual body attitude (TRIAD from first IMU sample
   handles the pitch/roll; the world_gen yaw alignment makes the
   default `IDENTITY` initial guess match reality on yaw).
3. **Stable cascaded controllers** with the inner loop ≥ 5× faster
   than the outer (cascaded-control stability rule):
   - rate (inner)
   - attitude
   - velocity
   - position (outer)
4. **Integral action** on the position-error loop. Without an
   I-term the position controller has steady-state error
   proportional to hover-trim mismatch and gain ratios; the
   vehicle visibly drifts.
5. **A min-thrust gate on axis control**: collective below ~0.1
   means we're on the ground; running the attitude loop against
   ground reaction force will yaw the chassis but not lift it,
   eating thrust on takeoff.

## Mission criteria — what "real flight" means

Three classes of criterion live in `aviate-hal-xil::mission::Criterion`:

| Class           | Examples                                | Source       |
|-----------------|-----------------------------------------|--------------|
| End-of-phase    | `Armed`, `AltitudeHold`, `PositionHold` | gz truth     |
| Trace-aware     | `MinAltitude`, `ReachedWaypoint`        | gz truth     |
| Window-sliding  | `StableHover`, `StableAttitude`         | gz truth     |
| Telemetry       | `MavTelemetryAltitudeAgrees`            | MAVLink      |

The mission framework today already carries the gz-truth criteria.
The MAVLink-telemetry criteria are the new piece this PR adds.

## Implementation plan — in priority order

### Step 1 — Controller integral term (close steady-state drift)

`aviate-core/src/control/position.rs` is P-only. Add an integral
accumulator + anti-windup clamp. The accumulator lives in the
controller's runtime state (`Pos­CtrlRuntime`), reset on disarm.

Result: the position controller's vertical channel converges on
the commanded altitude even when hover-trim is slightly off.

### Step 2 — Tune the X500 cascade

With trim 0.77 and the new I-term, walk the gains:
- pos_xy: 0.5, pos_z: 0.8
- vel_xy: 0.4, vel_z: 0.6 (with I-term on z)
- att: 6, 6, 2 (current default)
- rate: 0.4, 0.4, 0.3

The cascade ratio rule (att/rate ≤ 0.2) must be respected. The
existing 6/0.15 ratio of 40 is far outside that — the rate loop
saturates against any non-trivial attitude error. Lower att and
raise rate together until the ratio is ≤ 5 (the standard rule of
thumb for stability).

### Step 3 — TRIAD-style EKF init + world_gen yaw alignment

In `aviate-runtime/src/sim/step.rs`: the first IMU sample lets us
recover pitch/roll from gravity direction. Yaw stays at zero and
the mag update refines it during the settle phase. In
`tests/gcs-test/src/world_gen.rs`: convert NED-yaw to ENU-pose-yaw
(`π/2 - heading`) so the gz spawn orientation matches NED+FRD
`IDENTITY` when `spawn_heading == 0`.

### Step 4 — MAVLink telemetry observer

Today the harness only reads ground truth from the gz bridge. Add
a MAVLink listener thread in `gcs-test` that captures
`GLOBAL_POSITION_INT` and `ATTITUDE` messages from the FC's
stream. Both go into the per-phase trace so criteria can compare.

New criterion: `TelemetryPositionAgreesWithTruth { tolerance }`
that asserts the FC's reported position matches gz ground truth
within tolerance for the entire phase. Disagreement is a fault
even if the vehicle physically flew the right course.

### Step 5 — Real-flight missions

Three replace the existing smoke-level missions:

- `static_hover.toml` — arm → settle → climb to 5 m (position
  target) → 10 s stationkeeping (`StableHover` ±0.5 m + ±0.3 m
  horizontal drift) → descend to 1 m → land. Passes only if the
  vehicle holds position; closed-loop is the witness.
- `square_course.toml` — same takeoff/landing scaffolding, but
  between hover and land the vehicle flies N 10 m → E 10 m → S
  10 m → W 10 m, with `ReachedWaypoint` ±1 m at each corner.
- `attitude_test.toml` — open-loop attitude commands (level →
  +15° roll → level → +15° pitch → level) with
  `StableAttitude { roll_pitch_max_deg: 20, yaw_max_deg: 30 }`
  asserting the vehicle holds the commanded attitude.

### Step 6 — Reliability gate

Each mission runs 5× in the sweep. The reliability bar is **5/5
PASS, no exceptions**. Below that the work is not done.

## What's deferred

Cert-grade thresholds (0.3 m altitude, 0.5 m horizontal) remain on
DRQ-CTL-002. This plan delivers integration-grade flight — the
vehicle visibly flies a course you can watch in the GUI — but the
quantitative bounds still need a control-law refresh (LQR or full
PID with gain scheduling). The integration witness is the first
step; the cert-grade tightening is its own iteration.
