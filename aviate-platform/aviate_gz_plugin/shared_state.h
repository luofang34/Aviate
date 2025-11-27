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

/// Shared memory name base for POSIX shm_open
/// For multi-vehicle: /aviate_gz_bridge_0, /aviate_gz_bridge_1, etc.
/// Default (instance 0) uses /aviate_gz_bridge for backwards compatibility
#define AVIATE_SHM_NAME_BASE "/aviate_gz_bridge"
#define AVIATE_SHM_NAME "/aviate_gz_bridge"  // Instance 0 default

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

    // Lockstep synchronization (optional)
    // When lockstep_enabled=1, Gazebo waits for fc_step_ack before proceeding
    uint32_t lockstep_enabled;  // 0=async (default), 1=lockstep mode
    uint64_t sim_step;          // Incremented by plugin after each physics step
    uint64_t fc_step_ack;       // Written by FC after processing a step
} AviateSharedState;

#ifdef __cplusplus
}
#endif

#endif // AVIATE_SHARED_STATE_H
