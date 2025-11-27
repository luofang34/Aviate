#!/bin/bash
set -e

# Configuration
PX4_DIR=${PX4_AUTOPILOT_REPO:-"../PX4-Autopilot"}
export HEADLESS=${HEADLESS:-0}

if [ ! -d "$PX4_DIR" ]; then
    echo "Error: PX4-Autopilot repository not found at $PX4_DIR"
    exit 1
fi

echo "Launching Gazebo (HEADLESS=$HEADLESS)..."

# Run make in background
(
    cd "$PX4_DIR"
    # Suppress output?
    make px4_sitl gz_x500 > /dev/null 2>&1
) &
MAKE_PID=$!

# Wait for Gazebo to initialize
# We can wait for the port 14560 to be targeted?
# Or just sleep.
echo "Waiting for simulator startup (10s)..."
sleep 10

# Kill the PX4 binary that was started by make, to free up the ports for Aviate
echo "Stopping default PX4 binary..."
pkill -x px4 || true

echo "Gazebo ready (Make PID: $MAKE_PID)."
# We don't wait for MAKE_PID here, we exit so the caller can proceed.
# The make process keeps running in background managing gz.
