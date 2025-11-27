#!/bin/bash
set -e

# Aviate Gazebo SITL Launcher
#
# This script launches Gazebo Sim with the Aviate quadcopter world.
# It uses pre-installed gz (Gazebo Harmonic) directly, not PX4.

export HEADLESS=${HEADLESS:-0}
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
AVIATE_DIR="$(dirname "$SCRIPT_DIR")"
SITL_DIR="${AVIATE_DIR}/aviate-apps/quadcopter-sitl"
WORLD_FILE="${SITL_DIR}/worlds/x500_quadcopter.sdf"
MODELS_DIR="${SITL_DIR}/models"

# Export model path so Gazebo can find x500/x500_base models
export GZ_SIM_RESOURCE_PATH="${MODELS_DIR}:${GZ_SIM_RESOURCE_PATH:-}"

if [ ! -f "$WORLD_FILE" ]; then
    echo "Error: World file not found at $WORLD_FILE"
    exit 1
fi

# Check gz is available
if ! command -v gz &> /dev/null; then
    echo "Error: Gazebo (gz) not found. Please install Gazebo Harmonic."
    exit 1
fi

echo "Launching Gazebo (HEADLESS=$HEADLESS)..."

# Launch Gazebo in server-only mode (headless) or with GUI
if [ "$HEADLESS" -eq 1 ]; then
    echo "Starting Gazebo in headless mode..."
    gz sim -s -r "$WORLD_FILE" &
else
    echo "Starting Gazebo with GUI..."
    gz sim -r "$WORLD_FILE" &
fi
GZ_PID=$!

# Wait for Gazebo to initialize and topics to become available
echo "Waiting for simulator startup..."
for i in {1..30}; do
    if gz topic -l 2>/dev/null | grep -q "/x500/"; then
        echo "Gazebo topics available."
        break
    fi
    sleep 1
done

echo "Gazebo ready (PID: $GZ_PID)."
