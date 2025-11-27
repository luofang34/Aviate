#!/bin/bash
set -e

# Aviate SITL Launcher
#
# This script builds and runs the Aviate SITL quadcopter application
# with Gazebo Sim (Harmonic).
#
# Usage:
#   ./scripts/run_sitl.sh              # Interactive mode with GUI
#   ./scripts/run_sitl.sh --headless   # Headless mode (no GUI)
#   ./scripts/run_sitl.sh --test       # Automated test (async mode, real-time)
#   ./scripts/run_sitl.sh --test --lockstep  # Deterministic test (lockstep mode)
#
# Modes:
#   Async (default): Gazebo runs at real-time, FC reads when ready
#   Lockstep:        Gazebo waits for FC acknowledgment each step (deterministic)

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
AVIATE_DIR="$(dirname "$SCRIPT_DIR")"

HEADLESS=0
AUTO_TEST=0
LOCKSTEP=0

# Parse arguments
while [[ "$#" -gt 0 ]]; do
    case $1 in
        --headless) HEADLESS=1 ;;
        --test) AUTO_TEST=1; HEADLESS=1 ;; # Test mode implies headless
        --lockstep) LOCKSTEP=1 ;;
        -h|--help)
            echo "Aviate SITL Launcher"
            echo ""
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --headless   Run without GUI (uses EGL rendering)"
            echo "  --test       Run automated flight test (implies --headless)"
            echo "  --lockstep   Use lockstep mode (deterministic simulation)"
            echo "  -h, --help   Show this help message"
            echo ""
            echo "Examples:"
            echo "  $0                      # Interactive with GUI"
            echo "  $0 --test               # Quick functional test (async)"
            echo "  $0 --test --lockstep    # Deterministic test (lockstep)"
            exit 0
            ;;
        *) echo "Unknown parameter: $1"; exit 1 ;;
    esac
    shift
done

# Export for sub-scripts
export HEADLESS
export LOCKSTEP

# Check gz is available
if ! command -v gz &> /dev/null; then
    echo "Error: Gazebo (gz) not found. Please install Gazebo Harmonic."
    exit 1
fi

# Kill any existing SITL processes first
echo "Cleaning up existing processes..."
pkill -9 -f "gz sim" 2>/dev/null || true
pkill -9 -f "gz-bridge" 2>/dev/null || true
pkill -9 -f "aviate-app-quadcopter-sitl" 2>/dev/null || true
pkill -9 -f "sitl-test" 2>/dev/null || true
pkill -9 -f "lockstep-test" 2>/dev/null || true
# Clean up shared memory from previous runs
rm -f /dev/shm/aviate_gz_bridge 2>/dev/null || true
sleep 2

echo "=== Aviate SITL Launcher ==="
if [ "$LOCKSTEP" -eq 1 ]; then
    echo "Mode: Headless=$HEADLESS, AutoTest=$AUTO_TEST, Lockstep=YES (deterministic)"
else
    echo "Mode: Headless=$HEADLESS, AutoTest=$AUTO_TEST, Lockstep=NO (async/real-time)"
fi

# Build the Aviate SITL app
echo "Building Aviate SITL app..."
cargo build -p aviate-app-quadcopter-sitl

# Build the gz-bridge (requires libaviate_gz_bridge.so to be built)
# Check new location first, then legacy location
PLUGIN_DIR="${AVIATE_DIR}/aviate-platform/aviate_gz_plugin/build"
if [ ! -f "${PLUGIN_DIR}/libaviate_gz_bridge.so" ]; then
    # Fallback to legacy location
    PLUGIN_DIR="${AVIATE_DIR}/aviate-platform/sitl/aviate_gz_plugin/build"
fi

if [ -f "${PLUGIN_DIR}/libaviate_gz_bridge.so" ]; then
    echo "Building Gazebo bridge..."
    cargo build -p aviate-platform-sitl --features gz-plugin
    # Also build config-test and lockstep-test with gz-plugin
    cargo build -p aviate-app-quadcopter-sitl --features gz-plugin
    GZ_BRIDGE_AVAILABLE=1
    echo "Gazebo bridge built successfully."
else
    GZ_BRIDGE_AVAILABLE=0
    echo "Warning: libaviate_gz_bridge.so not found."
    echo "Build it first: cd aviate-platform/aviate_gz_plugin/build && cmake .. && make"
    echo "The test will run but won't have real sensor data from Gazebo."
fi

# Set library path for FFI
export LD_LIBRARY_PATH="${PLUGIN_DIR}:${LD_LIBRARY_PATH:-}"

cleanup() {
    echo ""
    echo "Shutting down..."
    pkill -f "gz sim" 2>/dev/null || true
    pkill -f "gz-bridge" 2>/dev/null || true
    pkill -f "aviate-app-quadcopter-sitl" 2>/dev/null || true
    # Clean up shared memory
    rm -f /dev/shm/aviate_gz_bridge 2>/dev/null || true
    sleep 1
}

trap cleanup EXIT

if [ "$AUTO_TEST" -eq 1 ]; then
    echo ""
    echo "=== Starting Gazebo ==="
    "$SCRIPT_DIR/launch_gazebo.sh"

    # Wait a bit more for plugin to initialize shared memory
    echo "Waiting for AviateGzPlugin to initialize..."
    for i in {1..10}; do
        if [ -f /dev/shm/aviate_gz_bridge ]; then
            echo "Shared memory ready."
            break
        fi
        sleep 0.5
    done

    if [ ! -f /dev/shm/aviate_gz_bridge ]; then
        echo "Warning: Shared memory not found. Plugin may not have loaded."
    fi

    if [ "$LOCKSTEP" -eq 1 ]; then
        # Lockstep mode: Run dedicated lockstep test (no separate Aviate/bridge)
        echo ""
        echo "=== Running Lockstep Flight Test ==="
        set +e
        ./target/debug/lockstep-test
        TEST_EXIT_CODE=$?
        set -e
    else
        # Async mode: Run full stack (Aviate + Bridge + Test)

        # Start Aviate FIRST so it binds port 14560 before gz-bridge sends to it
        echo ""
        echo "=== Starting Aviate Core (Background) ==="
        ./target/debug/aviate-app-quadcopter-sitl &
        AVIATE_PID=$!
        sleep 2  # Give Aviate time to bind ports

        # Start bridge if available
        if [ "$GZ_BRIDGE_AVAILABLE" -eq 1 ]; then
            echo ""
            echo "=== Starting Gazebo Bridge (Background) ==="
            ./target/debug/gz-bridge &
            BRIDGE_PID=$!
            sleep 2  # Give bridge time to connect
        fi

        echo ""
        echo "Waiting for system to stabilize (3s)..."
        sleep 3

        echo ""
        echo "=== Running Flight Test ==="
        set +e
        ./target/debug/sitl-test
        TEST_EXIT_CODE=$?
        set -e

        # Kill Aviate and Bridge
        kill $AVIATE_PID 2>/dev/null || true
        [ -n "$BRIDGE_PID" ] && kill $BRIDGE_PID 2>/dev/null || true
    fi

    echo ""
    echo "=== Test Completed with Exit Code: $TEST_EXIT_CODE ==="

    if [ $TEST_EXIT_CODE -eq 0 ]; then
        if [ "$LOCKSTEP" -eq 1 ]; then
            echo "PASSED: Lockstep SITL Test"
        else
            echo "PASSED: Async SITL Test"
        fi
        exit 0
    else
        if [ "$LOCKSTEP" -eq 1 ]; then
            echo "FAILED: Lockstep SITL Test"
        else
            echo "FAILED: Async SITL Test"
        fi
        exit 1
    fi
else
    echo ""
    echo "=== Starting Gazebo ==="
    "$SCRIPT_DIR/launch_gazebo.sh"

    # Wait for plugin
    echo "Waiting for AviateGzPlugin to initialize..."
    for i in {1..10}; do
        if [ -f /dev/shm/aviate_gz_bridge ]; then
            echo "Shared memory ready."
            break
        fi
        sleep 0.5
    done

    # Start bridge if available
    if [ "$GZ_BRIDGE_AVAILABLE" -eq 1 ]; then
        echo ""
        echo "=== Starting Gazebo Bridge (Background) ==="
        ./target/debug/gz-bridge &
        BRIDGE_PID=$!
        sleep 2
    fi

    echo ""
    echo "=== Starting Aviate Core (Interactive) ==="
    ./target/debug/aviate-app-quadcopter-sitl

    # Kill bridge on exit
    [ -n "$BRIDGE_PID" ] && kill $BRIDGE_PID 2>/dev/null || true
fi
