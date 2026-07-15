# AVIATE-CORE Behavioral Requirements — Scope and Index

The behavioral requirement set for `aviate-core` lives in the trace
tree. This document is a stable index to where each artifact lives;
it defines no requirements, thresholds, or statuses of its own. When
this index and the trace tree disagree, the trace tree wins.

## Normative sources

| Artifact | Location | Contents |
|---|---|---|
| System requirements | `cert/trace/sys.toml` | Structural rows plus the behavioral SYS rows (estimation accuracy, loop closure, mixing validity, fault response, morph safety, init determinism, disarmed safety) |
| High-level requirements | `cert/trace/hlr.toml` | 200-series behavioral HLRs. Numeric thresholds live here and only here |
| Low-level requirements | `cert/trace/llr.toml` | 200-series LLR decomposition of each behavioral HLR |
| Test witnesses | `cert/trace/tests.toml` | TST rows with the test selectors that witness each LLR |
| Derived requirements | `cert/trace/derived.toml` | DRQ rows: recorded design decisions and disclosed gaps |
| Mission orchestration | `tests/missions/manifest.json` | Single source of truth for which SITL missions run, their gate class (blocking / quarantined / manual), and each mission's reliability bar |

## Numbering

Behavioral rows use 200-series IDs to keep them distinct from the
001-series structural rows (which assert facts about the shape of
the code rather than what it does in flight). Families: `EST`
(estimation), `CTL` (stabilization), `MIX` (mixing/actuation),
`FLT` (faults, arming, safety), `MORPH` (mode and geometry
transitions), `INIT` (cold-start determinism).

## Verification tiers

- **U** — unit / integration tests in `aviate-core/tests/` with
  synthetic dynamics. Deterministic; the primary cert evidence.
  Recorded as TST rows in `cert/trace/tests.toml`.
- **X** — Gazebo SITL missions in `tests/missions/*.toml`, run by
  `gcs-test` against gz-sim. Integrated real-physics confirmation;
  gated in CI per the mission manifest.

## Enforcement

The CI evidence job runs `cargo evidence doctor`,
`cargo evidence trace --validate`, `cargo evidence floors`, and
`cargo evidence check --mode=source`; an unsatisfied requirement
claim (REQ_GAP) fails CI. Blocking SITL mission shards and the
non-blocking quarantine lane run per the mission manifest (see
`docs/SITL_CI_SHARDING.md`).

## Human navigation view

`docs/AVIATE_CORE_TEST_PLAN.md` maps each behavioral HLR to its
witnesses for human readers. It is a navigation view, not a second
requirements database; `scripts/check_test_plan_sync.py` (CI-gated)
fails when it drifts from the trace.
