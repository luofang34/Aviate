# AVIATE-CORE Test Plan

This document is a human navigation view over the behavioral
verification evidence for `aviate-core`. It is **not** a second
requirements database: requirement text and numeric thresholds live
in `cert/trace/hlr.toml` / `cert/trace/llr.toml`, test witnesses and
their selectors live in `cert/trace/tests.toml`, derived requirements
live in `cert/trace/derived.toml`, and SITL mission orchestration
lives in `tests/missions/manifest.json`. When this document and those
sources disagree, the sources win.

Pass/fail status is not recorded here. `cargo evidence check
--mode=source` runs in CI on every push and validates every
requirement claim against its witnesses — an unsatisfied claim
(REQ_GAP) fails CI, so a green main means every row below is
satisfied by the witnesses listed in the trace.

## Verification tiers

| Tier | Where it runs | What it proves |
|---|---|---|
| **U** — unit / integration | `cargo test --workspace` (behavioral suite also under `-p aviate-core --features test-hooks`) | Cycle-level kernel behavior with synthetic dynamics. Deterministic, bit-repeatable. Recorded as TST rows in `cert/trace/tests.toml`. |
| **X** — XIL / SITL mission | `gcs-test --features gazebo` against gz-sim, per `tests/missions/manifest.json` | End-to-end FC ↔ gz-sim loop with x500 physics, MAVLink over UDP, shared-memory bridge. Integration health, not flight quality. |
| **DRQ** — derived requirement | `cert/trace/derived.toml` | A recorded design decision, or an honest disclosure of a gap between an HLR's target and what the implementation achieves. |

## Verification matrix

Navigation only — decomposition and witness links are normative in
`cert/trace/{hlr,llr,tests}.toml`, and `cargo evidence trace
--validate` plus `cargo evidence check` gate them in CI. Titles
abbreviate the HLR rows. `scripts/check_test_plan_sync.py`
(CI-gated) fails when this matrix drifts from the trace.

| HLR | Title | Decomposed by | Witnessed by |
|---|---|---|---|
| `HLR-EST-201` | Cold-start attitude convergence | LLR-EST-201 | TST-EST-201 |
| `HLR-EST-202` | Angular-rate tracking | LLR-EST-202 | TST-EST-202 |
| `HLR-EST-203` | Position tracking under healthy GNSS+baro | LLR-EST-203 | TST-EST-203; `position_hold` mission adds real-physics confirmation |
| `HLR-EST-204` | NUMERIC_ERROR latch on non-finite input | LLR-EST-204, LLR-EST-205 | TST-EST-204, TST-EST-205 |
| `HLR-EST-205` | Bounded GNSS-dropout dead reckoning | LLR-EST-206, LLR-EST-207 | TST-EST-206, TST-EST-207; `gnss_dropout` mission exercises the injected-dropout path |
| `HLR-CTL-201` | Rate-loop setpoint tracking | LLR-CTL-201 | TST-CTL-201 |
| `HLR-CTL-202` | Attitude step response bounded | LLR-CTL-202, LLR-CTL-205 | TST-CTL-202, TST-CTL-205 |
| `HLR-CTL-203` | Hover hold within bounds | LLR-CTL-203 | TST-CTL-203; `hover_stability` mission adds real-physics confirmation; cert-grade bounds tracked by DRQ-CTL-002 |
| `HLR-CTL-204` | Authority limits under saturation | LLR-CTL-204 | TST-CTL-204 |
| `HLR-MIX-201` | Actuator outputs inside [0.0, 1.0] | LLR-MIX-201 | TST-MIX-201 |
| `HLR-MIX-202` | QuadX zero-command symmetry | LLR-MIX-202 | TST-MIX-202 |
| `HLR-MIX-203` | Sanitizer safe-pattern fallback | LLR-MIX-203, LLR-MIX-204 | TST-MIX-203, TST-MIX-204 |
| `HLR-FLT-201` | Disarmed kernel emits safe pattern | LLR-FLT-201, LLR-FLT-202 | TST-FLT-201, TST-FLT-202 |
| `HLR-FLT-202` | Pre-arm checks reject with typed ArmError | LLR-FLT-203, LLR-FLT-204 | TST-FLT-203, TST-FLT-204 |
| `HLR-FLT-203` | NUMERIC_ERROR inhibits output until ground reset | LLR-FLT-205, LLR-FLT-206 | TST-FLT-205, TST-FLT-206; `numeric_fault_inject` mission exercises the injected-NaN path |
| `HLR-FLT-204` | Command timeout → safe mode within one cycle | LLR-FLT-207, LLR-FLT-208, LLR-FLT-209 | TST-FLT-207, TST-FLT-208 (slew), TST-FLT-209, TST-FLT-209B; `command_timeout` mission exercises the over-the-wire path |
| `HLR-MORPH-201` | Atomic ConfigMode swap | LLR-MORPH-201 | TST-MORPH-201 |
| `HLR-MORPH-202` | GeometryState change without actuator spikes | LLR-MORPH-202 | TST-FLT-208 (shared slew-limiter witness) |
| `HLR-INIT-201` | Bit-deterministic cold start | LLR-INIT-201 | TST-INIT-201 |
| `HLR-INIT-202` | Pre-arm gates individually testable | LLR-FLT-203 (shared with HLR-FLT-202), LLR-INIT-202 | TST-FLT-203 (shared pre-arm witness) |

## Tier U — unit / integration tests

The U-tier suites live in `aviate-core/tests/`. The exact test
selectors that witness each LLR are recorded per TST row in
`cert/trace/tests.toml`; test counts are not tracked here (the CI
test, coverage, and evidence gates adjudicate them on every push).

Run locally before paying the SITL setup cost:

```bash
cargo test --workspace
cargo test -p aviate-core --test behavioral_tests --features test-hooks
```

## Tier X — SITL missions

Mission TOMLs live in `tests/missions/`. `tests/missions/manifest.json`
is the single source of truth for orchestration: every mission has an
entry (bijection enforced by `scripts/check_mission_manifest.py`), a
gate class, and a runs/pass-threshold reliability bar.

- **blocking** — runs in the required CI shard lanes and feeds
  CI Success.
- **quarantined** — runs visibly in the non-blocking quarantine lane
  and must link a tracking issue; quarantine is never a passing gate.
- **manual** — run on demand for evidence campaigns, with a stated
  reason.

Each mission runs through `gcs-test --features gazebo` against a
freshly spawned gz-sim instance with the AviateGzPlugin loaded. The
flight-control binary (`sitl-gazebo-x500`) runs in a second process,
reads ground-truth model state from the plugin via POSIX shared
memory, synthesizes IMU + baro + mag + GNSS, runs the kernel, and
writes motor commands back.

Fault injection is wired end to end: `inject_fault` / `clear_faults`
mission actions go through the mission runner's fault client
(`aviate-hal/xil/src/runner.rs`) over the fault-injection protocol
(`aviate-hal-xil::fault_protocol`). The `gnss_dropout` and
`numeric_fault_inject` blocking missions exercise this path against
a live kernel.

Where a mission adds real-physics confirmation of a trace row, the
TST row's description in `cert/trace/tests.toml` names the mission;
mission runs are CI integration gates, not standalone trace rows.

### How to run

```bash
# Manifest-driven runner (same harness CI uses; applies each
# mission's reliability bar):
scripts/run_sitl_missions.sh

# Single mission:
cargo run -p gcs-test --features gazebo -- run --xil tests/missions/basic_flight.toml
```

Building the gz plugin is scripted in `scripts/build_gz_plugin.sh`;
CI installs Gazebo Harmonic from the OSRF apt repo (see the SITL jobs
in `.github/workflows/ci.yml` and `docs/SITL_CI_SHARDING.md`).
Mission runs require a Linux host with the gz-sim plugin built —
macOS Homebrew's gz-msgs10 protobuf skew blocks the plugin build, so
Linux CI is the ground truth for the X tier.

## Tier DRQ — derived requirements

Normative text lives in `cert/trace/derived.toml`. One lifecycle
status per DRQ, mirroring the `status` field there
(`scripts/check_test_plan_sync.py` enforces equality):

| DRQ | Status | Anchor |
|---|---|---|
| `DRQ-EST-001` | Standing design record (not a gap) | `Estimator::reset()` preserves construction-time tuning |
| `DRQ-CFG-001` | Open | `load_config()` returns typed `InvalidFormat` until the validation parser lands (`aviate-core/src/kernel_trait.rs`) |
| `DRQ-MIX-001` | Open | Global `safe_output` fallback retained until every airframe declares per-mode `safe_pattern` |
| `DRQ-CTL-001` | Closed | Tuning lives in `ResolvedKernelConfig.cascade_gains` + `hover_thrust_norm`; `verify_config_binding` rejects controller/config mismatch at build (`aviate-core/tests/config_binding_tests.rs`) |
| `DRQ-CTL-002` | Open | Controller-gain tuning to cert-grade hover/position bounds; visible as the quarantined `attitude_control` mission in the manifest |
| `DRQ-CTL-003` | Open | Closed-loop position hold end to end; `square_course` is quarantined, `hover_trim_check` (open-loop) and `closed_loop_landing` are manual evidence campaigns |
| `DRQ-FLT-001` | Closed | Per-cycle slew writer: `ResolvedKernelConfig.slew_limit_per_cycle` + `aviate-core/src/kernel/slew.rs::apply_slew_limit`, witnessed by TST-FLT-208 |
| `DRQ-MORPH-001` | Closed | Same slew writer; TST-FLT-208 traces to LLR-MORPH-202. Non-zero per-airframe limits are config tuning that becomes binding when a morphing airframe ships |

## Open gaps acknowledged but not DRQ'd

These remain open. They are not DRQs because either the contract
surface still needs design work or the cert binding is contingent on
an airframe variant the tree does not ship.

- **Mode-morph SITL scenario** (Hover → Cruise round-trip). The X500
  ships with `ConfigMode::Hover` only; a mode-morph mission needs
  either a VTOL airframe with both mode configs or a stub `Cruise`
  config tuned for the X500. The unit-tier atomicity witness
  (TST-MORPH-201) is in place; the integrated witness lands once a
  multi-mode airframe is in tree.
- **MotorFailurePlugin SITL scenario** (one motor stuck at zero
  mid-flight). The sanitizer LLRs (`LLR-MIX-203`/`LLR-MIX-204`) are
  pinned at the unit tier; the Gazebo-level witness needs the
  plugin's mid-mission rotor override wired into a mission AND a
  controller whose closed-loop authority can compensate at hover.
  The second part is blocked by DRQ-CTL-002 — until the controller
  holds steady-state hover, a stuck motor simply crashes the
  vehicle, masking whether the rebalance fired.
- **MC/DC coverage tracking**. The blocking CI coverage gate is
  region + branch coverage (LLVM source-based) after documented
  `COV:EXCL` exclusions. MC/DC is not measured in a qualified way —
  rustc's `-Zcoverage-options=mcdc` is experimental and unqualified —
  so no MC/DC or DO-178C structural-coverage credit is claimed.
  MC/DC is tracked non-blocking as a readiness indicator only
  (`.github/workflows/nightly-mcdc.yml`).

## What this plan does and does not assert

**Does assert:**
- Every 200-series HLR and LLR has a recorded witness in
  `cert/trace/tests.toml` (or a DRQ disclosing the gap), and
  `cargo evidence check --mode=source` gates those claims in CI.
- The full FC ↔ gz-sim integration loop closes on Linux CI via the
  blocking mission shards.

**Does not assert:**
- That the current controller gains meet cert-grade flight quality.
  That gap is DRQ-CTL-002 / DRQ-CTL-003; the SITL thresholds prove
  integration health, not control quality.
- That the kernel meets DO-178C Level B without evidence beyond this
  trace tree. Coverage qualification, formal review, and
  configuration-management artifacts are separate work.
- That the SITL missions characterize hardware behavior. The
  `aviate-hal-stm32h7` + `aviate-boards/micoair-h743-v2` flight
  build is a separate target lane verified separately.

## Maintenance

- Requirement or witness changes happen in `cert/trace/*.toml` (with
  the matching `cert/floors.toml` bump); update the navigation matrix
  here to mirror them. `cargo evidence trace --validate` and
  `cargo evidence check` catch drift between trace and tests;
  `scripts/check_test_plan_sync.py` catches drift between this file
  and the trace.
- Mission gating changes happen in `tests/missions/manifest.json`;
  `scripts/check_mission_manifest.py` enforces the manifest ↔
  mission-file bijection.
- Do not record pass histories, run counts, or test totals in this
  file; CI adjudicates those on every push.
