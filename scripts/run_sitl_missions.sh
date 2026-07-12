#!/usr/bin/env bash
#
# Run every SITL mission in tests/missions/ N times and report
# per-mission PASS/FAIL counts. Exits non-zero if any mission
# falls below the reliability bar (default: 2/3 minimum).
#
# Used by hand for local stability verification, and by CI as
# the integration-tier gate paired with
#   `cargo test --workspace --features test-hooks`.
#
# Usage:
#   ./scripts/run_sitl_missions.sh              # 3 runs each, default
#   RUNS_PER_MISSION=5 ./scripts/run_sitl_missions.sh
#   GCS_TEST_BIN=./target/debug/gcs-test ./scripts/run_sitl_missions.sh
#
# CI sets GCS_TEST_BIN so every shard runs the exact artifact produced by
# the build job. Local runs omit it and let cargo build on demand.
#
# Companion U-tier invocation (run first to surface logic failures
# before paying the SITL setup cost):
#   cargo test --workspace
#   cargo test -p aviate-core --test behavioral_tests --features test-hooks

set -euo pipefail

MISSIONS_DIR="${MISSIONS_DIR:-tests/missions}"
MANIFEST_QUERY="scripts/check_mission_manifest.py"

# The manifest is the single mission list; hand-written duplicates in
# this script or workflow YAML are exactly what let a mission vanish
# from orchestration silently. RUNS_PER_MISSION, when set, overrides
# the per-mission plan for local experimentation.
DEFAULT_MISSIONS=($("${MANIFEST_QUERY}" --emit-default-missions))

MISSIONS=("${@:-${DEFAULT_MISSIONS[@]}}")
overall_fail=0

for mission in "${MISSIONS[@]}"; do
    plan=$("${MANIFEST_QUERY}" --mission-plan "${mission}")
    plan_runs=${plan%% *}
    plan_threshold=${plan##* }
    runs="${RUNS_PER_MISSION:-${plan_runs}}"
    echo "=== ${mission} (${runs} runs, bar ${plan_threshold}) ==="
    pass=0
    fail=0
    for ((r = 1; r <= runs; r++)); do
        # Reclaim orphaned gz / FC processes + shared memory left by a
        # prior run (or an earlier CI step) before starting, so a crash
        # cannot poison its successors. gcs-test does this on a clean
        # exit; this covers timeouts and hard kills.
        pkill -9 -f 'gz sim' 2>/dev/null || true
        pkill -9 -f 'sitl-gazebo-x500' 2>/dev/null || true
        rm -f /dev/shm/aviate_gz_bridge* 2>/dev/null || true
        sleep 1

        # gcs-test exits non-zero when a mission fails; that's
        # expected here, so disable `set -e` for the run.
        set +e
        if [[ -n "${GCS_TEST_BIN:-}" ]]; then
            run_log=$(
                timeout 150 "${GCS_TEST_BIN}" run --xil --headless \
                    "${MISSIONS_DIR}/${mission}.toml" 2>&1
            )
        else
            run_log=$(
                timeout 150 cargo run --quiet -p gcs-test --features gazebo -- \
                    run --xil --headless "${MISSIONS_DIR}/${mission}.toml" 2>&1
            )
        fi
        set -e
        result=$(printf '%s\n' "${run_log}" | grep "^Result:" | tail -1)
        if [[ "${result}" == "Result: PASS" ]]; then
            pass=$((pass + 1))
            echo "  run ${r}: PASS"
        else
            fail=$((fail + 1))
            echo "  run ${r}: ${result:-Result: TIMEOUT}"
            # Surface the failing phase / launch error that the
            # `^Result:` filter would otherwise hide.
            printf '%s\n' "${run_log}" | tail -40 | sed 's/^/    | /'
        fi
    done
    echo "  -> ${pass}/${runs} PASS"
    if [[ "${pass}" -lt "${plan_threshold}" ]]; then
        echo "  !! ${mission} fell below ${plan_threshold}/${runs} reliability bar"
        overall_fail=1
    fi
done

if [[ "${overall_fail}" -ne 0 ]]; then
    exit 1
fi
echo "All missions met the reliability bar."
