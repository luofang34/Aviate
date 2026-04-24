#!/usr/bin/env bash
#
# File-size gate for Aviate source code.
#
# Enforces the two size limits declared in CLAUDE.md's "Code Size Limits"
# section:
#
#   * Any .rs source file: 500 lines max. Split into sub-modules when
#     exceeded.
#   * lib.rs: 100 lines max. Re-exports and module declarations only.
#
# Scope: .rs files under aviate-*/src/ trees. Tests/ and tests-support
# files (aviate-core/tests/*, aviate-hal/io/src/fake.rs) are test
# infrastructure rather than flight-build code and stay out of scope.
#
# Existing violations that predate this gate are recorded in the
# `ALLOWLIST` array below. Each entry carries a reason. Adding a new
# entry blocks merge by failing this script — intentional. Removing an
# entry requires that the file actually meet the limit now (otherwise
# the script fails again with the raw violation).
#
# Exit codes:
#   0 — all files within limits (or on the allowlist).
#   1 — at least one file exceeds its limit and is not allowlisted.
#
# Usage:
#   scripts/check_file_sizes.sh              # run from repo root
#   scripts/check_file_sizes.sh --verbose    # also print per-file line counts

set -euo pipefail

MAX_RS_LINES=500
MAX_LIB_RS_LINES=100

# Pre-existing violations. Pairs: `path|limit|reason`.
# Split them or bring them down when you touch the file; when the file
# falls under its limit, remove the entry and the gate keeps it honest.
ALLOWLIST=(
    "aviate-link/src/mavlink/protocol.rs|${MAX_RS_LINES}|MAVLink field layouts: mechanical, one big match over message IDs; splitting loses locality"
    "aviate-hal/io/src/fake.rs|${MAX_RS_LINES}|Test HAL fakes for every sensor/actuator kind; split candidate in a follow-up"
    "aviate-hal/xil/src/runner.rs|${MAX_RS_LINES}|SITL mission-runner; split candidate"
    "aviate-boards/micoair-h743-v2/src/hw.rs|${MAX_RS_LINES}|Board init wiring: one function, grows with peripherals; split candidate"
    "aviate-boards/micoair-h743-v2/src/usb_cdc.rs|${MAX_RS_LINES}|Board USB CDC driver; excluded from parent workspace"
    "aviate-boards/micoair-h743-v2/src/lib.rs|${MAX_LIB_RS_LINES}|Board lib; excluded from parent workspace, to be split when touched"
    "aviate-hal/xil/backends/mavlink-hil/src/lib.rs|${MAX_LIB_RS_LINES}|MAVLink HIL backend lib; follow-up split"
    "aviate-boards/sitl-jmavsim/src/lib.rs|${MAX_LIB_RS_LINES}|SITL board lib; follow-up split"
    "aviate-hal/xil/src/lib.rs|${MAX_LIB_RS_LINES}|XIL HAL lib; follow-up split"
    "aviate-chips/rp2350/src/lib.rs|${MAX_LIB_RS_LINES}|RP2350 chip lib; excluded from parent workspace"
    "aviate-boot-core/src/lib.rs|${MAX_LIB_RS_LINES}|Bootloader core lib; excluded from parent workspace"
    "aviate-boards/sitl-gazebo/src/lib.rs|${MAX_LIB_RS_LINES}|Gazebo SITL board lib; follow-up split"
    "aviate-airframes/multirotor/src/lib.rs|${MAX_LIB_RS_LINES}|Multirotor airframe lib; follow-up split"
    "aviate-drivers/src/lib.rs|${MAX_LIB_RS_LINES}|Drivers crate lib; follow-up split"
    "aviate-link/src/lib.rs|${MAX_LIB_RS_LINES}|Link crate lib; follow-up split"
    "aviate-chips/stm32h743/src/lib.rs|${MAX_LIB_RS_LINES}|Chip lib; excluded from parent workspace"
    "aviate-config/src/lib.rs|${MAX_LIB_RS_LINES}|Config crate lib; follow-up split"
    "aviate-hal/io/src/board_hal.rs|${MAX_RS_LINES}|Board HAL trait surface; split candidate"
    "aviate-hal/io/src/lib.rs|${MAX_LIB_RS_LINES}|HAL-io lib; follow-up split"
    "aviate-hal/io/src/traits.rs|${MAX_RS_LINES}|HAL trait definitions; split by device kind in a follow-up"
    "aviate-hal/stm32h7/src/transport.rs|${MAX_RS_LINES}|STM32H7 USB transport; excluded from parent workspace"
    "aviate-hal/stm32h7/src/usb_cdc.rs|${MAX_RS_LINES}|STM32H7 USB CDC driver; excluded from parent workspace"
    "aviate-hal/xil/backends/mavlink-hil/src/messages.rs|${MAX_RS_LINES}|MAVLink message layouts; mechanical serialization"
    "aviate-hal/xil/backends/mavlink-hil/src/wire.rs|${MAX_RS_LINES}|MAVLink v1/v2 frame codec"
    "aviate-hal/xil/src/config.rs|${MAX_RS_LINES}|XIL TOML config parser; follow-up split"
    "aviate-hal/xil/src/fault_ctrl.rs|${MAX_RS_LINES}|XIL fault injection controller; follow-up split"
    "aviate-hal/xil/src/fault_protocol.rs|${MAX_RS_LINES}|XIL fault-injection wire protocol"
    "aviate-hal/xil/src/sitl_io.rs|${MAX_RS_LINES}|SITL I/O transport; follow-up split"
)

VERBOSE=0
for arg in "$@"; do
    case "$arg" in
        -v|--verbose) VERBOSE=1 ;;
        -h|--help)
            sed -n '2,/^set -e/p' "$0" | sed 's/^# \{0,1\}//'
            exit 0
            ;;
        *)
            echo "Unknown argument: $arg" >&2
            exit 2
            ;;
    esac
done

# Resolve repo root relative to this script.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$REPO_ROOT"

# Build the allowlist lookup once.
declare -A ALLOW_LIMIT
declare -A ALLOW_REASON
for entry in "${ALLOWLIST[@]}"; do
    IFS='|' read -r path limit reason <<<"$entry"
    ALLOW_LIMIT["$path"]="$limit"
    ALLOW_REASON["$path"]="$reason"
done

# Collect .rs files under aviate-*/src/ (excludes tests/, target/,
# external/, any *.rs in a tests subtree).
mapfile -t FILES < <(
    find aviate-* -type f -name '*.rs' 2>/dev/null \
        | grep -E '/src/' \
        | grep -v '/target/' \
        | grep -v '/external/' \
        | sort
)

fail=0
grandfathered=0
checked=0

for f in "${FILES[@]}"; do
    # Determine the applicable limit.
    if [[ "$(basename "$f")" == "lib.rs" ]]; then
        limit=$MAX_LIB_RS_LINES
    else
        limit=$MAX_RS_LINES
    fi

    lines=$(wc -l < "$f" | tr -d ' ')
    checked=$((checked + 1))

    if [[ "$lines" -le "$limit" ]]; then
        [[ "$VERBOSE" -eq 1 ]] && printf '  %s: %d/%d lines\n' "$f" "$lines" "$limit"
        continue
    fi

    # Over the limit. Grandfathered?
    if [[ -n "${ALLOW_LIMIT[$f]:-}" ]]; then
        grandfathered=$((grandfathered + 1))
        [[ "$VERBOSE" -eq 1 ]] && \
            printf '  [ALLOWLIST] %s: %d/%d lines — %s\n' \
                "$f" "$lines" "$limit" "${ALLOW_REASON[$f]}"
        continue
    fi

    printf 'FAIL: %s is %d lines; limit is %d\n' "$f" "$lines" "$limit" >&2
    fail=$((fail + 1))
done

if [[ "$fail" -gt 0 ]]; then
    echo "" >&2
    echo "Over-limit files above are not on the allowlist." >&2
    echo "Options (in order of preference):" >&2
    echo "  1. Split the file into sub-modules (see docs/AVIATE_SPEC.md §lib.rs" >&2
    echo "     or any of the recent split PRs for the foo.rs + foo/ pattern)." >&2
    echo "  2. If the file genuinely can't be split (e.g. generated layouts)," >&2
    echo "     add it to ALLOWLIST in scripts/check_file_sizes.sh with a reason." >&2
    exit 1
fi

echo "File-size gate: OK"
echo "  checked:       $checked"
echo "  grandfathered: $grandfathered"
