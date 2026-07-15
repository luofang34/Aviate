#!/usr/bin/env bash
#
# Cross-language ABI gate for the XIL shm contract (#262).
#
# The gz plugin is C++ and consumes the cbindgen-generated
# `aviate_xil_contract.h`; the Rust side owns the layout. Two ways
# that pairing breaks, both of which are silent until a Gazebo build
# runs (a 20-minute feedback loop, and a runtime mystery if the build
# is skipped):
#
#   * the plugin's include path stops resolving the generated header;
#   * the header's layout drifts from the values the plugin and every
#     consumer compile against.
#
# This gate compiles a translation unit with the SAME relative include
# path the plugin's CMakeLists uses, asserting size, block offsets,
# cache-line isolation of the two high-rate cross-process writers, and
# the enum wire values. No Gazebo toolchain needed — just a C++
# compiler, which every lane that can build this workspace has.
#
# Usage:
#   scripts/check_xil_contract_header.sh
#   scripts/check_xil_contract_header.sh --self-test
#
# Exit codes: 0 header compiles and matches, 1 mismatch, 2 bad
# invocation or no compiler.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

PLUGIN_DIR="$REPO_ROOT/aviate-hal/xil/backends/gz/plugin"
# The include path exactly as plugin/CMakeLists.txt spells it, so a
# CMake path edit that breaks the plugin build breaks this gate first.
CONTRACT_INC="$PLUGIN_DIR/../../../contract/include"

find_cxx() {
    if [[ -n "${CXX:-}" ]]; then
        printf '%s' "$CXX"
        return 0
    fi
    local candidate
    for candidate in c++ g++ clang++; do
        if command -v "$candidate" >/dev/null 2>&1; then
            printf '%s' "$candidate"
            return 0
        fi
    done
    return 1
}

# The assertions the plugin itself carries, in a standalone TU.
write_probe() {
    cat > "$1" <<'EOF'
#include "aviate_xil_contract.h"
#include <cstddef>

// Layout freeze. These are the same numbers the Rust const asserts
// pin and the plugin's own static_asserts repeat; a drift on either
// side fails here.
static_assert(sizeof(AviateSharedStateV2) == 448, "block size drifted");
static_assert(offsetof(AviateSharedStateV2, header) == 0, "header offset drifted");
static_assert(offsetof(AviateSharedStateV2, state) == 64, "state offset drifted");
static_assert(offsetof(AviateSharedStateV2, command) == 256, "command offset drifted");
static_assert(offsetof(AviateSharedStateV2, control) == 384, "control offset drifted");

// The generation rides inside the seqlock payload: a consumer's
// {generation, step, time, state} quadruple is one coherent read.
static_assert(offsetof(AviateSharedStateHeader, writer_incarnation) == 24, "writer_incarnation drifted");
static_assert(offsetof(AviateModelStateBlock, reset_generation) == 4, "snapshot generation drifted");
static_assert(offsetof(AviateModelStateBlock, sim_step) == 8, "sim_step drifted");
static_assert(offsetof(AviateModelStateBlock, time_us) == 16, "time_us drifted");
static_assert(offsetof(AviateMotorCommandBlock, fc_step_ack) == 72, "fc_step_ack drifted");
static_assert(offsetof(AviateControlBlock, lifecycle_request) == 0, "lifecycle_request drifted");
static_assert(offsetof(AviateControlBlock, fc_status) == 16, "fc_status drifted");

// Cache-line isolation: the plugin writes `state` at 1 kHz while the
// FC writes `command` at 1 kHz from another process. Sharing a line
// would make every publish steal the peer's line.
static_assert(offsetof(AviateSharedStateV2, state) % 64 == 0, "state not cache-line aligned");
static_assert(offsetof(AviateSharedStateV2, command) % 64 == 0, "command not cache-line aligned");
static_assert(offsetof(AviateSharedStateV2, control) % 64 == 0, "control not cache-line aligned");

// Alignment of every lane the two sides access atomically. An
// under-aligned u64/f64 lane makes __atomic_load_n/AtomicU64 either
// slow-path-lock or tear, silently, only on some hosts.
static_assert(alignof(AviateSharedStateV2) >= 8, "block under-aligned");
static_assert(alignof(AviateSharedStateHeader) >= 8, "header under-aligned");
static_assert(alignof(AviateModelStateBlock) >= 8, "state block under-aligned");
static_assert(alignof(AviateMotorCommandBlock) >= 8, "command block under-aligned");
static_assert(alignof(AviateControlBlock) >= 8, "control block under-aligned");
static_assert(offsetof(AviateModelStateBlock, sim_step) % 8 == 0, "sim_step lane under-aligned");
static_assert(offsetof(AviateModelStateBlock, time_us) % 8 == 0, "time_us lane under-aligned");
static_assert(offsetof(AviateModelStateBlock, pos) % 8 == 0, "pos lanes under-aligned");
static_assert(offsetof(AviateModelStateBlock, quat) % 8 == 0, "quat lanes under-aligned");
static_assert(offsetof(AviateModelStateBlock, vel) % 8 == 0, "vel lanes under-aligned");
static_assert(offsetof(AviateModelStateBlock, ang_vel) % 8 == 0, "ang_vel lanes under-aligned");
static_assert(offsetof(AviateMotorCommandBlock, motor_vel) % 8 == 0, "motor lanes under-aligned");
static_assert(offsetof(AviateMotorCommandBlock, fc_step_ack) % 8 == 0, "fc_step_ack under-aligned");
static_assert(offsetof(AviateControlBlock, lifecycle_request) % 8 == 0, "lifecycle word under-aligned");
static_assert(offsetof(AviateControlBlock, fc_status) % 8 == 0, "fc_status word under-aligned");

// Wire values consumers switch on.
static_assert(AviateLifecycleRequest_None == 0, "LifecycleRequest::None drifted");
static_assert(AviateLifecycleRequest_Reset == 1, "LifecycleRequest::Reset drifted");
static_assert(AviateLifecycleRequest_Stop == 2, "LifecycleRequest::Stop drifted");
static_assert(AviateLifecycleRequest_Start == 3, "LifecycleRequest::Start drifted");
static_assert(AviateFcState_Init == 0, "FcState::Init drifted");
static_assert(AviateFcState_Ready == 3, "FcState::Ready drifted");

int main() { return 0; }
EOF
}

check_repo() {
    local cxx
    if ! cxx="$(find_cxx)"; then
        echo "XIL_CONTRACT_HEADER: no C++ compiler (set CXX, or install c++/g++/clang++)" >&2
        return 2
    fi

    # The plugin must actually DECLARE the contract include directory.
    # Compiling the probe against a path this script knows would pass
    # happily while the plugin's own build cannot find the header —
    # exactly the break the Gazebo lane catches 20 minutes later.
    if ! grep -q 'contract/include' "$PLUGIN_DIR/CMakeLists.txt"; then
        echo "XIL_CONTRACT_HEADER: plugin/CMakeLists.txt does not include the generated-header directory;" >&2
        echo "  add \${CMAKE_CURRENT_SOURCE_DIR}/../../../contract/include to AviateGzPlugin's" >&2
        echo "  target_include_directories, or the plugin cannot see aviate_xil_contract.h" >&2
        return 1
    fi

    if [[ ! -f "$CONTRACT_INC/aviate_xil_contract.h" ]]; then
        echo "XIL_CONTRACT_HEADER: generated header missing at the path plugin/CMakeLists.txt includes:" >&2
        echo "  $CONTRACT_INC/aviate_xil_contract.h" >&2
        echo "  (regenerate: cargo test -p aviate-xil-contract --test header_sync)" >&2
        return 1
    fi

    local tmp
    tmp="$(mktemp -d)"
    # shellcheck disable=SC2064
    trap "rm -rf '$tmp'" RETURN
    write_probe "$tmp/probe.cc"

    if ! "$cxx" -std=c++17 -fsyntax-only -I "$CONTRACT_INC" "$tmp/probe.cc" 2>"$tmp/err"; then
        echo "XIL_CONTRACT_HEADER: the generated header does not match the pinned layout:" >&2
        cat "$tmp/err" >&2
        return 1
    fi
    return 0
}

self_test() {
    local cxx tmp
    if ! cxx="$(find_cxx)"; then
        echo "SELF_TEST: no C++ compiler available" >&2
        exit 2
    fi
    tmp="$(mktemp -d)"
    # shellcheck disable=SC2064
    trap "rm -rf '$tmp'" EXIT

    # A header whose layout drifted must FAIL the probe — proving the
    # gate detects drift rather than merely compiling something.
    mkdir -p "$tmp/inc"
    sed 's/uint64_t magic;/uint64_t magic; uint64_t drift_me;/' \
        "$CONTRACT_INC/aviate_xil_contract.h" > "$tmp/inc/aviate_xil_contract.h"
    write_probe "$tmp/probe.cc"
    if "$cxx" -std=c++17 -fsyntax-only -I "$tmp/inc" "$tmp/probe.cc" 2>/dev/null; then
        echo "SELF_TEST FAIL: a drifted header passed the probe" >&2
        exit 1
    fi
    echo "self-test ok: drifted layout is rejected"

    # A CMakeLists that stops declaring the contract include directory
    # must FAIL — this is the exact break the Gazebo lane caught.
    local saved="$tmp/CMakeLists.saved"
    cp "$PLUGIN_DIR/CMakeLists.txt" "$saved"
    grep -v 'contract/include' "$saved" > "$PLUGIN_DIR/CMakeLists.txt"
    local rc=0
    check_repo >/dev/null 2>&1 || rc=$?
    cp "$saved" "$PLUGIN_DIR/CMakeLists.txt"
    if [[ "$rc" == 0 ]]; then
        echo "SELF_TEST FAIL: a CMakeLists without the contract include path passed" >&2
        exit 1
    fi
    echo "self-test ok: missing plugin include path is rejected"

    # The real header must pass.
    if ! check_repo; then
        echo "SELF_TEST FAIL: the checked-in header does not pass its own probe" >&2
        exit 1
    fi
    echo "self-test ok: checked-in header passes"
    echo "XIL_CONTRACT_HEADER_SELF_TEST_OK"
}

case "${1:-}" in
    '')
        if check_repo; then
            echo "XIL_CONTRACT_HEADER_OK: the C++ view of the contract matches the Rust layout"
        else
            exit $?
        fi
        ;;
    --self-test)
        self_test
        ;;
    *)
        echo "usage: $0 [--self-test]" >&2
        exit 2
        ;;
esac
