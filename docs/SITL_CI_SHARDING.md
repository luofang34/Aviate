# SITL CI missions

The `SITL Missions (JSBSim X500)` job flies the missions in `tests/missions`
against Pilotage's JSBSim sidecar. The sidecar owns the physics and the
simulation clock and runs unpaced (`--realtime-factor 0`): the lockstep with
the flight controller, not wall time, sets the pace, so a mission's tens of
seconds of simulated flight take a few seconds of wall time and the whole
blocking set fits in one job.

The job builds JSBSim with Pilotage's xtask and the sidecar from Pilotage at
the full commit `PILOTAGE_REV` names (both cached on their pins), starts the
sidecar with the X500 at the default SITL origin, and runs
`sitl-jsbsim-missions` from the `aviate-app-sitl-jsbsim-x500` crate:

- `sitl-jsbsim-missions blocking` flies every `gate=blocking` mission of
  `tests/missions/manifest.json` and fails the job when one misses its
  reliability bar (`pass_threshold` passes out of `runs`).
- `sitl-jsbsim-missions quarantine` flies the quarantined missions on pushes
  to main. It never fails the job; a red quarantine run is truth about the
  quarantine, and each quarantined mission links its tracking issue.

Each mission run reconnects to the sidecar, which starts a new world for every
session, so runs never share a vehicle.

## Running missions locally

Build and start the sidecar from a Pilotage checkout (`cargo xtask
build-jsbsim`, then `cargo build --release -p pilotage-jsbsim-sidecar`), with
the same arguments the job uses, on port 4560 plus the mission vehicle's
instance. Then, from this repository:

```sh
cargo run --release -p aviate-app-sitl-jsbsim-x500 --bin sitl-jsbsim-missions -- blocking
cargo run --release -p aviate-app-sitl-jsbsim-x500 --bin sitl-jsbsim-missions -- position_hold
```

A selection is `blocking`, a manifest shard, or a mission name. If another
flight controller already holds the XIL ports, set `XIL_BASE_PORT` to a free
range; the board and the mission runner both read it. Each run writes a
per-sample trace CSV to the temporary directory.

## Adding or changing a mission

1. Add `tests/missions/<name>.toml` and an entry in
   `tests/missions/manifest.json`; `scripts/check_mission_manifest.py`
   enforces that every mission has exactly one entry.
2. Fly it locally until it meets the bar you give it.
3. Land under position control (a `goto` to the ground with a
   `touchdown_velocity` check): a free fall from an open-loop thrust cut is a
   crash on JSBSim, and the disarm that follows is refused.
