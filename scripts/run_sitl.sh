#!/bin/bash
set -e

# Configuration
PX4_DIR=${PX4_AUTOPILOT_REPO:-"../PX4-Autopilot"}
AVIATE_DIR=$(pwd)

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

# Export HEADLESS for the launch script called by the app
export HEADLESS

# Validate paths
if [ ! -d "$PX4_DIR" ]; then
    echo "Error: PX4-Autopilot repository not found at $PX4_DIR"
    exit 1
fi

echo "=== Aviate SITL Launcher ==="
echo "Mode: Headless=$HEADLESS, AutoTest=$AUTO_TEST"

echo "Building Aviate..."
cargo build -p aviate-app-quadcopter-sitl

if [ "$AUTO_TEST" -eq 1 ]; then
    echo "=== Starting Aviate Core (Background) ==="
    # Aviate will launch Gazebo via scripts/launch_gazebo.sh
    ./target/debug/aviate-app-quadcopter-sitl &
    AVIATE_PID=$!
    
    echo "Waiting for Aviate & Gazebo to initialize (20s)..."
    sleep 20
    
    echo "=== Running Flight Test Script ==="
    set +e
    python3 sitl_flight_test.py
    TEST_EXIT_CODE=$?
    set -e
    
    echo "=== Test Completed with Exit Code: $TEST_EXIT_CODE ==="
    
    # Cleanup
    echo "Shutting down..."
    kill $AVIATE_PID || true
    pkill -f "gz sim" || true
    pkill -f "ruby" || true
    pkill -x px4 || true
    
    if [ $TEST_EXIT_CODE -eq 0 ]; then
        echo "✅ SITL Flight Test PASSED"
        exit 0
    else
        echo "❌ SITL Flight Test FAILED"
        exit 1
    fi
else
    echo "=== Starting Aviate Core (Interactive) ==="
    # Run Aviate in foreground. It will launch Gazebo.
    ./target/debug/aviate-app-quadcopter-sitl
    
    # Cleanup after user exits
    echo "Shutting down..."
    pkill -f "gz sim" || true
    pkill -x px4 || true
fi
