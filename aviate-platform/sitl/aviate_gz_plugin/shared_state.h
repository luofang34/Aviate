// Aviate Gazebo Bridge - Shared Memory Layout
//
// This header defines the shared memory structure used for IPC between
// the gz-sim plugin (AviateGzPlugin) and the Rust FFI bridge.
//
// This file is standalone C and can be included from both C++ and Rust.

#ifndef AVIATE_SHARED_STATE_H
#define AVIATE_SHARED_STATE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/// Shared memory name for POSIX shm_open
#define AVIATE_SHM_NAME "/aviate_gz_bridge"

/// Shared memory structure for IPC between plugin and Rust bridge
typedef struct AviateSharedState {
    // Model state (written by plugin, read by Rust)
    double pos[3];          // Position [x, y, z] meters (ENU frame)
    double quat[4];         // Quaternion [w, x, y, z]
    double vel[3];          // Linear velocity [vx, vy, vz] m/s
    double ang_vel[3];      // Angular velocity [wx, wy, wz] rad/s
    uint64_t time_us;       // Simulation time (microseconds)
    uint32_t seq;           // Sequence number for detecting updates
    uint32_t valid;         // Data valid flag (non-zero when valid)

    // Motor commands (written by Rust, read by plugin)
    double motor_vel[8];    // Motor velocities (rad/s)
    int32_t num_motors;     // Number of motors (typically 4)
    uint32_t motor_seq;     // Command sequence number

    // Status
    uint32_t plugin_ready;  // Set by plugin when ready
} AviateSharedState;

#ifdef __cplusplus
}
#endif

#endif // AVIATE_SHARED_STATE_H
