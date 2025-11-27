#!/bin/bash
set -e

# Aviate Gazebo SITL Launcher
#
# This script launches Gazebo Sim with the Aviate quadcopter world.
# It uses pre-installed gz (Gazebo Harmonic) directly, not PX4.
#
# Environment variables:
#   HEADLESS=1  - Run without GUI (uses EGL rendering)
#   LOCKSTEP=1  - Use lockstep world (deterministic simulation)

export HEADLESS=${HEADLESS:-0}
export LOCKSTEP=${LOCKSTEP:-0}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
AVIATE_DIR="$(dirname "$SCRIPT_DIR")"
SITL_DIR="${AVIATE_DIR}/aviate-apps/quadcopter-sitl"
# Local models override PX4-gazebo-models (e.g., x500 without MotorFailurePlugin)
LOCAL_MODELS_DIR="${AVIATE_DIR}/models"
PX4_MODELS_DIR="${AVIATE_DIR}/external/PX4-gazebo-models/models"

# Select world file based on LOCKSTEP mode
if [ "$LOCKSTEP" -eq 1 ]; then
    WORLD_FILE="${SITL_DIR}/worlds/x500_quadcopter_lockstep.sdf"
    echo "Using LOCKSTEP world (deterministic simulation)"
else
    WORLD_FILE="${SITL_DIR}/worlds/x500_quadcopter.sdf"
    echo "Using ASYNC world (real-time simulation)"
fi

# Check submodule is initialized
if [ ! -d "$PX4_MODELS_DIR" ]; then
    echo "Error: PX4-gazebo-models submodule not found."
    echo "Run: git submodule update --init external/PX4-gazebo-models"
    exit 1
fi

# Export model path - local models first (override), then PX4-gazebo-models (for x500_base, etc.)
export GZ_SIM_RESOURCE_PATH="${LOCAL_MODELS_DIR}:${PX4_MODELS_DIR}:${GZ_SIM_RESOURCE_PATH:-}"

# Export plugin path so Gazebo can find AviateGzPlugin
# Check new location first, then legacy location
PLUGIN_DIR="${AVIATE_DIR}/aviate-platform/aviate_gz_plugin/build"
if [ ! -f "${PLUGIN_DIR}/libAviateGzPlugin.so" ]; then
    # Fallback to legacy location
    PLUGIN_DIR="${AVIATE_DIR}/aviate-platform/sitl/aviate_gz_plugin/build"
fi

if [ -f "${PLUGIN_DIR}/libAviateGzPlugin.so" ]; then
    export GZ_SIM_SYSTEM_PLUGIN_PATH="${PLUGIN_DIR}:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
    echo "AviateGzPlugin found at ${PLUGIN_DIR}"
else
    echo "Warning: AviateGzPlugin not built. Run: cd aviate-platform/aviate_gz_plugin/build && cmake .. && make"
fi

if [ ! -f "$WORLD_FILE" ]; then
    echo "Error: World file not found at $WORLD_FILE"
    exit 1
fi

# Check gz is available
if ! command -v gz &> /dev/null; then
    echo "Error: Gazebo (gz) not found. Please install Gazebo Harmonic."
    exit 1
fi

echo "Launching Gazebo (HEADLESS=$HEADLESS, LOCKSTEP=$LOCKSTEP)..."

# Launch Gazebo in headless mode (for automated tests) or with GUI (for manual testing)
# Headless rendering uses EGL backend for GPU-accelerated rendering without X server
# See: https://gazebosim.org/api/sim/9/headless_rendering.html
if [ "$HEADLESS" -eq 1 ]; then
    echo "Starting Gazebo in headless mode (with EGL rendering)..."
    # Clear DISPLAY to force EGL backend, use --headless-rendering for sensor support
    DISPLAY= gz sim -s -r --headless-rendering "$WORLD_FILE" &
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
