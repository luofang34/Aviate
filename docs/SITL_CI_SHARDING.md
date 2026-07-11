# SITL CI sharding

The SITL gate builds the Gazebo plugins, `gcs-test`, and the flight-control
binary once. The build job packages those files with checksums, and every
mission shard downloads that same artifact. This keeps compilation out of the
mission jobs and guarantees that all shards exercise identical binaries.

Missions run concurrently on separate GitHub-hosted runners. They must not run
concurrently inside one runner: the harness uses fixed UDP ports and POSIX
shared-memory names, and its orphan cleanup terminates matching Gazebo and
flight-control processes.

## Balancing policy

Balance shards using the duration of the `Run SITL mission shard` step. Exclude
checkout, artifact transfer, and Gazebo installation because those costs apply
once per runner and can vary with external package mirrors.

Target three to five minutes of mission execution per shard. Prefer rebalancing
missions among existing shards before adding another runner. Add a shard when
the missions cannot stay within the upper bound after rebalancing; merge shards
when their combined mission time fits within the target range. This range
amortizes runner setup while keeping the workflow's critical path short.

The wall-clock critical path is:

```text
build job + slowest(setup + mission shard)
```

The runner cost is proportional to:

```text
build job + sum(setup + mission shard)
```

More shards reduce the first expression but repeat setup in the second. The
target range is the default tradeoff between feedback latency and runner cost.

## Adding or changing a mission

1. Run the mission three times with `scripts/run_sitl_missions.sh` and record
   its mission-step duration from CI.
2. Add the mission name to exactly one entry in
   `tests/missions/ci-shards.json`.
3. Repack the entries by measured duration so each shard remains in the target
   range.
4. Confirm the build job's shard validation succeeds. It rejects duplicate
   mission names and entries without a matching `tests/missions/<name>.toml`.
5. Read every shard log and confirm the configured reliability bar is applied
   to every mission.

The matrix uses `fail-fast: false` so one failure does not hide results from
the other shards. Artifact checksum verification is mandatory in every shard.
