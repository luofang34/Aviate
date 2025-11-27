#!/bin/bash
set -e

# Aviate SITL Launcher
#
# This script builds and runs the Aviate SITL quadcopter application
# with Gazebo Sim (Harmonic). No PX4 dependency required.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
AVIATE_DIR="$(dirname "$SCRIPT_DIR")"

HEADLESS=0
AUTO_TEST=0

# Parse arguments
while [[ "$#" -gt 0 ]]; do
    case $1 in
        --headless) HEADLESS=1 ;;
        --test) AUTO_TEST=1; HEADLESS=1 ;; # Test mode implies headless
        *) echo "Unknown parameter: $1"; exit 1 ;;
    esac
    shift
done

# Export HEADLESS for the launch script
export HEADLESS

# Check gz is available
if ! command -v gz &> /dev/null; then
    echo "Error: Gazebo (gz) not found. Please install Gazebo Harmonic."
    exit 1
fi

echo "=== Aviate SITL Launcher ==="
echo "Mode: Headless=$HEADLESS, AutoTest=$AUTO_TEST"

echo "Building Aviate..."
cargo build -p aviate-app-quadcopter-sitl

# Try to build the gz-bridge (optional, requires Gazebo libraries)
echo "Building Gazebo bridge (optional)..."
if cargo build -p aviate-app-quadcopter-sitl --features gz-bridge --bin gz-bridge 2>/dev/null; then
    GZ_BRIDGE_AVAILABLE=1
    echo "Gazebo bridge built successfully."
else
    GZ_BRIDGE_AVAILABLE=0
    echo "Note: Gazebo bridge not available (missing gz-transport libraries)."
    echo "      The test will run but won't have real sensor data from Gazebo."
fi

cleanup() {
    echo "Shutting down..."
    pkill -f "gz sim" 2>/dev/null || true
    # Give processes time to exit gracefully
    sleep 1
}

trap cleanup EXIT

if [ "$AUTO_TEST" -eq 1 ]; then
    echo "=== Starting Gazebo ==="
    "$SCRIPT_DIR/launch_gazebo.sh"

    # Start bridge if available
    if [ "$GZ_BRIDGE_AVAILABLE" -eq 1 ]; then
        echo "=== Starting Gazebo Bridge (Background) ==="
        ./target/debug/gz-bridge &
        BRIDGE_PID=$!
        sleep 2  # Give bridge time to connect
    fi

    echo "=== Starting Aviate Core (Background) ==="
    ./target/debug/aviate-app-quadcopter-sitl &
    AVIATE_PID=$!

    echo "Waiting for Aviate to initialize (5s)..."
    sleep 5

    echo "=== Running Flight Test Script ==="
    set +e
    python3 tests/sitl_gcs_test.py
    TEST_EXIT_CODE=$?
    set -e

    echo "=== Test Completed with Exit Code: $TEST_EXIT_CODE ==="

    # Kill Aviate and Bridge
    kill $AVIATE_PID 2>/dev/null || true
    [ -n "$BRIDGE_PID" ] && kill $BRIDGE_PID 2>/dev/null || true

    if [ $TEST_EXIT_CODE -eq 0 ]; then
        echo "✅ SITL Flight Test PASSED"
        exit 0
    else
        echo "❌ SITL Flight Test FAILED"
        exit 1
    fi
else
    echo "=== Starting Gazebo ==="
    "$SCRIPT_DIR/launch_gazebo.sh"

    # Start bridge if available
    if [ "$GZ_BRIDGE_AVAILABLE" -eq 1 ]; then
        echo "=== Starting Gazebo Bridge (Background) ==="
        ./target/debug/gz-bridge &
        BRIDGE_PID=$!
        sleep 2
    fi

    echo "=== Starting Aviate Core (Interactive) ==="
    ./target/debug/aviate-app-quadcopter-sitl

    # Kill bridge on exit
    [ -n "$BRIDGE_PID" ] && kill $BRIDGE_PID 2>/dev/null || true
fi
