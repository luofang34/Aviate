#!/bin/bash
# Kill all SITL-related processes
# Usage: ./scripts/kill_sitl.sh

echo "Killing SITL processes..."

# Kill Gazebo
pkill -9 -f "gz sim" 2>/dev/null && echo "  Killed: gz sim" || true

# Kill FC processes
pkill -9 -f "sitl-gazebo-x500" 2>/dev/null && echo "  Killed: sitl-gazebo-x500" || true
pkill -9 -f "sitl-gazebo" 2>/dev/null && echo "  Killed: sitl-gazebo" || true

# Kill gcs-test
pkill -9 -f "gcs-test" 2>/dev/null && echo "  Killed: gcs-test" || true

# Kill mavrouter
pkill -9 -f "mavrouter" 2>/dev/null && echo "  Killed: mavrouter" || true

# Kill ruby (Gazebo uses it)
pkill -9 ruby 2>/dev/null && echo "  Killed: ruby" || true

# Clean up shared memory
rm -f /dev/shm/aviate_gz_bridge* 2>/dev/null && echo "  Cleaned: /dev/shm/aviate_gz_bridge*" || true

# Brief pause
sleep 0.5

# Verify
REMAINING=$(pgrep -f "gz sim|sitl-gazebo|gcs-test" 2>/dev/null | wc -l)
if [ "$REMAINING" -eq 0 ]; then
    echo "All SITL processes cleaned up."
else
    echo "Warning: $REMAINING processes still running"
    pgrep -af "gz sim|sitl-gazebo|gcs-test" 2>/dev/null || true
fi
