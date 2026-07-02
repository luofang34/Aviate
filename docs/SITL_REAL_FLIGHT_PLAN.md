# SITL Real-Flight Test Harness — Plan (v2)

## The problem with v1

The previous harness reported `Result: PASS` for runs where the
vehicle briefly climbed, drifted away, and crashed. Three failure
modes hid behind the "PASS":

1. **End-of-phase-only checks**: criteria like `MinAltitude` and
   `MaxAltitude` only sample the vehicle at the END of a phase
   (or its peak). A vehicle that pops up to 4 m, drifts 20 m
   sideways, and crashes at 0.3 m can still pass an `EndAltitude
   < 5 m` gate.

2. **Altitude-only verification**: position has three axes. A
   vehicle that holds altitude while flying laterally off into
   the distance is not hovering. A criterion that only checks Z
   can not catch a runaway in XY.

3. **No attitude verification**: a tumbling vehicle that happens
   to be near the right altitude at the right moment passes a
   geometric criterion. Real flight requires the body axes to be
   roughly aligned with the commanded ones throughout the phase.

A test harness that PASSES non-flight is **worse than no harness**
— it gives false confidence and hides regressions. The harness
must report PASS only when the vehicle did the intended thing.

## Verification axes

Every flight phase has three observation channels and the harness
must check all three:

|  Axis        | Source       | What it catches                          |
|--------------|--------------|------------------------------------------|
| Position 3D  | gz pose      | drift, runaway, wrong-direction flight   |
| Attitude     | gz pose      | tumble, sustained tilt, yaw rotation     |
| Telemetry    | MAVLink      | FC-vs-truth disagreement (EKF divergence)|

Every criterion must specify which sample(s) it checks. Three
shapes apply, in increasing strictness:

- **End-state**: one sample at phase end. (E.g. armed/disarmed.)
- **At-some-point**: any sample in the trace. (E.g. "vehicle
  visited waypoint X.")
- **Throughout-phase**: every sample in the trace. (E.g. "vehicle
  stayed within the station-keeping box.")

The earlier framework only used the first two. Real flight needs
the third — that is where "no jumping around" is enforced.

## Strict criteria

The replacement criterion set:

### Position

| Name | Shape | Asserts |
|---|---|---|
| `StationKeeping { center, xy_tol, z_tol }` | throughout | every trace sample is inside `(center.xy ± xy_tol, center.z ± z_tol)` |
| `TrajectoryTracking { waypoints, tolerance, max_time_s }` | trace-walking | the vehicle visits each waypoint in order, each within `tolerance` of the target NED point, completing the sequence within `max_time_s` |
| `ReturnedNear { target, tolerance }` | end-state | end-of-phase 3D position within `tolerance` of `target` |
| `MaxExcursion { center, xy_max, z_max }` | throughout | every sample's deviation from `center` is bounded; catches runaway flight |

### Attitude

| Name | Shape | Asserts |
|---|---|---|
| `AttitudeBounded { roll_pitch_max_deg }` | throughout | `|roll|` and `|pitch|` (extracted from quaternion) are bounded for every sample. Yaw is unbounded (it drifts naturally). |
| `YawDriftBounded { max_drift_deg }` | end-state | total yaw rotation from start to end of phase is bounded |
| `AttitudeRateBounded { max_rad_per_s }` | throughout | per-sample angular velocity is bounded; catches tumble |

### Cross-channel

| Name | Shape | Asserts |
|---|---|---|
| `TelemetryAgreesWithTruth { xy_tol, z_tol }` | throughout | the FC's reported position (via MAVLink GLOBAL_POSITION_INT) is within tolerance of the gz ground-truth position. Disagreement means the EKF is diverging. |

### The cost

These criteria are designed to FAIL. With the current control
stack (no I-term, no integral position hold) the vehicle drifts.
The harness must report that drift as failure, not paper it over.
A failing real-flight test is the correct output until the
controller can deliver real flight.

## Design rules

The user's feedback shapes these:

1. **A vehicle's altitude profile alone is not flight evidence.**
   At least one criterion in every flight phase must check XY too.
2. **An end-of-phase snapshot alone is not flight evidence.** At
   least one criterion in every flight phase must walk the trace.
3. **Criteria are not "weak by default".** Default tolerance is
   the tight bound; loosening it requires a comment explaining
   what we're observing instead.
4. **Criteria fail loudly.** The failing criterion's actual value
   is logged with the expected. No "PASSED" lines hide real
   failures.
5. **Cross-channel agreement is its own criterion.** EKF divergence
   that doesn't appear in gz ground truth is a fault even if the
   vehicle physically flew the right course (because in real
   hardware the FC drives the actuator from EKF, not truth).

## Mission shape

Each mission has phases. Each phase has an action and a set of
criteria. The action commands the FC; the criteria observe the
result. Per-phase verification:

```
arm     → Armed(true)
takeoff → ReachedAltitude(target_z) AND AttitudeBounded(20°)
hover   → StationKeeping(center, 0.5m XY, 0.3m Z, 5s) AND
          AttitudeBounded(10°) AND
          TelemetryAgreesWithTruth(0.5m XY, 0.3m Z)
course  → TrajectoryTracking(waypoints, 1.0m tol, 30s)
return  → ReturnedNear(home, 1.5m) AND AttitudeBounded(20°)
land    → MaxExcursion(home, 1m XY, 6m Z) AND end-state
          on ground (alt < 0.5m)
disarm  → Armed(false)
```

This is the contract real-flight evidence must satisfy. Anything
less is not real flight.

## Implementation plan

1. **Extend the per-phase trace** to record `(elapsed, position,
   attitude_quat, mavlink_telemetry_position)`. The MAVLink
   listener is its own piece of work — telemetry agreement can
   be deferred behind a feature gate while we get the position
   + attitude criteria in.
2. **Add the new Criterion variants** to
   `aviate-hal-xil::mission::Criterion` and their evaluators in
   `runner.rs`. Each new variant has a `verify_criterion_*`
   helper that walks the trace.
3. **Update the TOML parser** in `config.rs` to accept the new
   criterion shapes (multi-field inline tables, nested arrays
   of waypoints).
4. **Add explicit failure messages** to every criterion. The
   message includes: criterion name, what was checked, the
   actual value (the failing sample), the expected bound, and
   the phase time at which the failure occurred. No `PASSED`
   line ever appears without those details.
5. **Rewrite the missions** that ship with the harness against
   the new criteria. `hover_trim_check` becomes a real hover
   mission with `StationKeeping` and `AttitudeBounded`. The
   `square_course` mission keeps its waypoints but the criteria
   become `TrajectoryTracking`. Both are expected to fail today
   — that is the honest record of the controller's state.
6. **Document the failures** as evidence: the mission TOMLs
   carry the cert-grade thresholds; `cert/trace/derived.toml`
   carries the DRQ describing the gap; the test plan file
   carries the verification-axis matrix.
7. **No criterion is weakened to make the test pass.** If a
   real test fails today, the fix is to the controller / EKF,
   not to the harness.

## Cert-grade thresholds

The current HLRs (HLR-CTL-203, HLR-EST-203) require:
- Altitude hover bound: ≤ 0.3 m
- Horizontal hover bound: ≤ 0.5 m
- Position estimate error: ≤ 0.5 m XY, ≤ 0.3 m Z
- Attitude bound during hover: ≤ 10° roll/pitch
- Position estimate vs truth agreement: same tolerances as above

These are the bounds the new criteria check by default. The DRQ
acknowledging the gap between today's controller and these
thresholds is DRQ-CTL-002 + DRQ-CTL-003. Until those close, the
real-flight missions are honest test failures.
