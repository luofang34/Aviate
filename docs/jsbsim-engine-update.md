# JSBSim and Simulate CI gates

Date: 2026-11-11.

## Scope

Continuous Integration (CI) runs two flight-dynamics simulation gates:

- `SITL Missions (JSBSim X500)` runs Software-in-the-Loop (SITL) missions through the Pilotage sidecar (`pilotage-jsbsim-sidecar`).
- `Simulate XIL Gates (JSBSim C172 & X500)` runs Anything-in-the-Loop (XIL) tests from `luofang34/Simulate` against the checked-out Aviate commit.

Both gates use the Rust `jsbsim` crate.
Neither gate builds or links the C++ JSBSim library.

## SITL mission gate

The `sitl_jsbsim` job checks out `luofang34/Pilotage` at `PILOTAGE_REV` (`9296611ee3bab20b925092b63eda26dcce176135`).
The sidecar uses `jsbsim 0.2.0` by default.
The job builds `pilotage-jsbsim-sidecar` without the `cpp` feature and without a C++ JSBSim directory.
The sidecar starts with `--engine rust` and `--realtime-factor 0` at latitude 47.3977419 degrees, longitude 8.5455938 degrees, heading 0 degrees, and site elevation 488 m.

The mission harness (`sitl-jsbsim-missions`) runs the blocking manifest:

```sh
target/release/sitl-jsbsim-missions blocking
```

The six blocking missions are `basic_flight`, `command_timeout`, `gnss_dropout`, `hover_stability`, `numeric_fault_inject`, and `position_hold`.
The manifest permits three runs per mission and requires two passes.

## Simulate XIL gate

The `simulate-gates` job checks out `luofang34/Simulate` at `SIMULATE_REV` (`eb3201ec906a8740b0bfe1f0c0a62ed60a8e1ec9`).
The script `simulate/scripts/use-local-siblings.sh` redirects Simulate's `Aviate` git dependencies to the current Aviate workspace.
The job then runs three test suites in release mode:

```sh
(cd simulate && cargo test --release -p sim-aviate -p ironbird -p sim-flight-chain)
```

- `sim-aviate` checks the fixed-wing C172 and multirotor X500 bridges, sensor models, and envelope limits.
- `ironbird` runs Tier-0, Tier-1, and Tier-2 closed-loop JSBSim flights for the C172 and X500 airframes.
- `sim-flight-chain` runs the hosted guidance, Aviate controller, and JSBSim C172 flight chain across nominal flight, wind shifts, Global Navigation Satellite System (GNSS) outages, and frame rebases.

## Earlier parity record

Before CI removed the C++ matrix entry on 2026-11-11, a parity review on 2026-11-02 compared `jsbsim 0.2.0` against C++ JSBSim 1.3.3 on Aviate `971fdd4cadd14789a974957b3b096146225fd337`.

| Engine | Missions | Runs | Passed phases |
| --- | --- | --- | --- |
| Rust `jsbsim 0.2.0` | 6 passed | 2 passes per mission | 84 |
| C++ JSBSim 1.3.3 | 6 passed | 2 passes per mission | 84 |

Pilotage PR #886 earlier reported 96 phases per engine with an older Rust dependency and Aviate commit.
The 2026-11-02 review did not confirm that phase count.
The local logs from the 2026-11-02 review recorded 84 passed phases per engine.
The 2026-11-02 review did not run the earlier PX4 mission reports again.
