#!/bin/bash
set -e

# Aviate SITL Test Runner
#
# Runs SITL flight tests with Gazebo simulation using gcs-test.
# All tests are config-based using TOML test definitions.
#
# Usage:
#   ./scripts/run_sitl.sh                              # Default test (basic_flight)
#   ./scripts/run_sitl.sh tests/xil-missions/hover.toml   # Specific test
#   ./scripts/run_sitl.sh --help                       # Show help
#
# The test runner handles:
#   - Process cleanup (kill existing Gazebo/test processes)
#   - World file generation from TOML config
#   - Gazebo launch with proper environment
#   - Test execution with mission verification
#   - Multi-vehicle support (concurrent vehicle testing)
#   - Proper cleanup on exit

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
AVIATE_DIR="$(dirname "$SCRIPT_DIR")"

# Default test config
DEFAULT_TEST="tests/missions/basic_flight.toml"
TEST_CONFIG=""

# Parse arguments
while [[ "$#" -gt 0 ]]; do
    case $1 in
        -h|--help)
            echo "Aviate SITL Test Runner"
            echo ""
            echo "Usage: $0 [TEST_CONFIG]"
            echo ""
            echo "Arguments:"
            echo "  TEST_CONFIG  Path to test TOML file (default: $DEFAULT_TEST)"
            echo ""
            echo "Examples:"
            echo "  $0                                       # Run default test"
            echo "  $0 tests/missions/basic_flight.toml  # Basic flight test"
            echo "  $0 tests/missions/two_vehicle_formation.toml  # Multi-vehicle test"
            echo ""
            echo "Test configs define:"
            echo "  - Vehicles: model, spawn position, instance ID"
            echo "  - Mission: phases with actions and verification criteria"
            echo "  - World: lockstep mode (world files generated dynamically)"
            exit 0
            ;;
        *)
            if [ -z "$TEST_CONFIG" ]; then
                TEST_CONFIG="$1"
            else
                echo "Error: unexpected argument: $1"
                exit 1
            fi
            ;;
    esac
    shift
done

# Use default if not specified
if [ -z "$TEST_CONFIG" ]; then
    TEST_CONFIG="$DEFAULT_TEST"
fi

# Validate test config exists
if [ ! -f "$TEST_CONFIG" ]; then
    echo "Error: Test config not found: $TEST_CONFIG"
    echo "Available tests:"
    ls -1 tests/missions/*.toml 2>/dev/null || echo "  (none found)"
    exit 1
fi

# Set headless mode for CI/automated testing
export HEADLESS=1

echo "=== Aviate SITL Test Runner ==="
echo "Test: $TEST_CONFIG"
echo ""

# --- Step 0: Kill existing processes ---
cleanup() {
    echo ""
    echo "Cleaning up..."
    pkill -9 -f "gz sim" 2>/dev/null || true
    pkill -9 -f "sitl-gazebo-x500" 2>/dev/null || true
    pkill -9 -f "gcs-test" 2>/dev/null || true
    rm -f /dev/shm/aviate_gz_bridge* 2>/dev/null || true
    echo "Done."
}

# Register cleanup on exit
trap cleanup EXIT

# Initial cleanup
echo "Killing existing processes..."
cleanup
sleep 1

# --- Step 1: Check gz is available ---
if ! command -v gz &> /dev/null; then
    echo "Error: Gazebo (gz) not found. Please install Gazebo Harmonic."
    exit 1
fi

# --- Step 2: Build gcs-test with Gazebo support ---
echo "Building gcs-test with Gazebo support..."
cargo build -p gcs-test --features gazebo 2>&1 | tail -5

# Check plugin is available
PLUGIN_DIR="${AVIATE_DIR}/aviate-hal/xil/backends/gz/plugin/build"

if [ ! -f "${PLUGIN_DIR}/libAviateGzPlugin.so" ]; then
    echo "Error: AviateGzPlugin not found."
    echo "Build it first: cd aviate-hal/xil/backends/gz/plugin/build && cmake .. && make"
    exit 1
fi

# Set library path for FFI
export LD_LIBRARY_PATH="${PLUGIN_DIR}:${LD_LIBRARY_PATH:-}"

echo ""
echo "=== Running Test ==="

# --- Step 3: Run the test via gcs-test ---
# gcs-test --xil handles:
#   - World file generation from TOML
#   - Gazebo launch with proper environment
#   - Mission execution with lockstep
#   - Multi-vehicle coordination (with mavrouter)
#   - Result reporting

./target/debug/gcs-test run --xil --headless "$TEST_CONFIG"
TEST_EXIT=$?

echo ""
echo "=== Test Result ==="
if [ $TEST_EXIT -eq 0 ]; then
    echo "PASSED: $TEST_CONFIG"
else
    echo "FAILED: $TEST_CONFIG (exit code: $TEST_EXIT)"
fi

exit $TEST_EXIT
