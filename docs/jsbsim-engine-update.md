# JSBSim engine update

Date: 2026-11-02.

The CI job uses Pilotage `9296611ee3bab20b925092b63eda26dcce176135`.
Pilotage PR #886 contains this commit.
The sidecar uses the Rust `jsbsim 0.2.0` crate by default.
This dependency requires Rust 1.99 or later.
The CI job uses the stable toolchain.
The C++ JSBSim 1.3.3 backend remains available through the `cpp` feature.

## CI behavior

The SITL matrix runs the same blocking mission manifest on both engines.
Each matrix entry has a separate sidecar cache key.
The Rust entry builds without a C++ JSBSim checkout.
The C++ entry builds the library and enables the `cpp` feature.
Each entry selects its engine explicitly at sidecar startup.
The Rust entry keeps the existing required-check name.
The CI Success job waits for both entries.

## Independent checks

These checks used Aviate main at `971fdd4cadd14789a974957b3b096146225fd337`.
They used the Pilotage source at the pin above.
The host was Linux x86-64 with Rust 1.99.1.

The mission harness built with:

```sh
cargo build --release --locked -p aviate-app-sitl-jsbsim-x500 --bin sitl-jsbsim-missions
```

Each engine served an X500 at latitude 47.3977419, longitude 8.5455938,
heading 0 degrees, and site elevation 488 m.
The sidecar used `--realtime-factor 0`.
The Rust run used no `--jsbsim-root` option.
The C++ run used `--engine cpp --jsbsim-root DIR`.
The mission command was:

```sh
target/release/sitl-jsbsim-missions blocking
```

| Engine | Missions | Runs | Passed phases |
| --- | --- | --- | --- |
| Rust `jsbsim 0.2.0` | 6 passed | 2 passes per mission | 84 |
| C++ JSBSim 1.3.3 | 6 passed | 2 passes per mission | 84 |

The manifest permits three runs and requires two passes.
Both engines passed on the first two runs.
The six missions are `basic_flight`, `command_timeout`, `gnss_dropout`,
`hover_stability`, `numeric_fault_inject`, and `position_hold`.

The Pilotage sidecar passed 53 tests with the default build and 58 tests
with `cpp`. These include model-load errors, shared data paths,
5000-step X500 and Rascal flights, and landing sensor comparisons.
See Pilotage `docs/jsbsim-rust-review.md` for the full review scope.

## Separate toolchain failure

Aviate PR #377 changes the Release Scripts toolchain from 1.95.0 to 1.120.0.
Run `41377667765` failed while the action downloaded Rust 1.120.0.
The distribution server returned HTTP 404 before compilation.
The JSBSim mission job in that run passed against the old Pilotage pin.
This update starts from main and does not include that toolchain change.

## Prior claims

Pilotage PR #886 originally reported 96 phases per engine with an older
Rust dependency and Aviate source. That count was not confirmed here.
The local logs from this review contain 84 passed phases per engine.
The earlier PX4 mission reports were not rerun here.
