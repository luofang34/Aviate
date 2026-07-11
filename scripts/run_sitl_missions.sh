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
#
# Companion U-tier invocation (run first to surface logic failures
# before paying the SITL setup cost):
#   cargo test --workspace
#   cargo test -p aviate-core --test behavioral_tests --features test-hooks

set -euo pipefail

RUNS_PER_MISSION="${RUNS_PER_MISSION:-3}"
MISSIONS_DIR="${MISSIONS_DIR:-tests/missions}"

# Single-vehicle missions only — two_vehicle_formation needs a
# multi-instance spawner that gcs-test doesn't drive today.
DEFAULT_MISSIONS=(
    basic_flight
    hover_stability
    attitude_control
    position_hold
    square_course
    gnss_dropout
    command_timeout
)

MISSIONS=("${@:-${DEFAULT_MISSIONS[@]}}")
overall_fail=0

for mission in "${MISSIONS[@]}"; do
    echo "=== ${mission} (${RUNS_PER_MISSION} runs) ==="
    pass=0
    fail=0
    for ((r = 1; r <= RUNS_PER_MISSION; r++)); do
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
        run_log=$(
            timeout 150 cargo run --quiet -p gcs-test --features gazebo -- \
                run --xil --headless "${MISSIONS_DIR}/${mission}.toml" 2>&1
        )
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
    echo "  -> ${pass}/${RUNS_PER_MISSION} PASS"
    if [[ "${pass}" -lt 2 && "${RUNS_PER_MISSION}" -ge 3 ]]; then
        echo "  !! ${mission} fell below 2/${RUNS_PER_MISSION} reliability bar"
        overall_fail=1
    fi
done

if [[ "${overall_fail}" -ne 0 ]]; then
    exit 1
fi
echo "All missions met the reliability bar."
