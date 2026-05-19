#!/usr/bin/env bash
#
# Configure + build the Aviate Gazebo C++ plugin under
# aviate-hal/xil/backends/gz/plugin/.
#
# Wraps the cmake invocation with the prefix-path / ignore-path
# arguments needed on a typical macOS developer machine
# (Homebrew protobuf shadowed by anaconda) and on Linux CI
# (system protobuf). The CMakeLists itself discovers the
# Homebrew protobuf prefix via `brew --prefix protobuf`; the
# shell script's job is to surface the macOS-specific
# `-DCMAKE_IGNORE_PATH` argument so a user-installed anaconda
# doesn't shadow the gz-msgs10 / protobuf version checks.
#
# Usage:
#   ./scripts/build_gz_plugin.sh           # configure + build
#   ./scripts/build_gz_plugin.sh clean     # rm -rf build/, then build
#
# Override the protobuf install prefix:
#   AVIATE_PROTOBUF_PREFIX=/opt/local ./scripts/build_gz_plugin.sh

set -euo pipefail

PLUGIN_DIR="$(cd "$(dirname "$0")/.." && pwd)/aviate-hal/xil/backends/gz/plugin"
BUILD_DIR="${PLUGIN_DIR}/build"

if [[ "${1:-}" == "clean" ]]; then
    rm -rf "${BUILD_DIR}"
fi

mkdir -p "${BUILD_DIR}"

cmake_args=(-DCMAKE_BUILD_TYPE=Release)

case "$(uname -s)" in
    Darwin)
        # Default Homebrew prefixes on Apple Silicon / Intel. The
        # CMakeLists asks brew for the exact protobuf prefix; the
        # shell only needs to feed Qt5 and the broad Homebrew root
        # to find_package and (optionally) exclude anaconda.
        if [[ -d /opt/homebrew ]]; then
            cmake_args+=(-DCMAKE_PREFIX_PATH="/opt/homebrew;/opt/homebrew/opt/qt@5")
        elif [[ -d /usr/local/Homebrew ]]; then
            cmake_args+=(-DCMAKE_PREFIX_PATH="/usr/local;/usr/local/opt/qt@5")
        fi
        if [[ -d "${HOME}/anaconda3" ]]; then
            cmake_args+=(-DCMAKE_IGNORE_PATH="${HOME}/anaconda3")
        elif [[ -d "${HOME}/miniconda3" ]]; then
            cmake_args+=(-DCMAKE_IGNORE_PATH="${HOME}/miniconda3")
        fi
        ;;
    Linux)
        # System protobuf / gz packages live under /usr; cmake's
        # default find paths cover them.
        :
        ;;
esac

if [[ -n "${AVIATE_PROTOBUF_PREFIX:-}" ]]; then
    cmake_args+=(-DAVIATE_PROTOBUF_PREFIX="${AVIATE_PROTOBUF_PREFIX}")
fi

cd "${BUILD_DIR}"
cmake "${cmake_args[@]}" ..
make -j"$(getconf _NPROCESSORS_ONLN 2>/dev/null || echo 2)"
