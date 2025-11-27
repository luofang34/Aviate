// Aviate Gazebo Bridge - C Interface for Rust FFI
//
// This header provides a C-compatible interface to gz-sim's EntityComponentManager
// for zero-copy access to physics simulation data from Rust.
//
// Usage:
//   1. Load this plugin in your SDF world file
//   2. Call aviate_gz_get_model_state() from Rust via FFI
//   3. Call aviate_gz_set_motor_speeds() to command actuators

#ifndef AVIATE_GZ_BRIDGE_H
#define AVIATE_GZ_BRIDGE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/// Model state returned from gz-sim ECM (all values in SI units)
typedef struct AviateModelState {
    /// Position in world frame [x, y, z] (meters, ENU)
    double pos[3];
    /// Orientation quaternion [w, x, y, z]
    double quat[4];
    /// Linear velocity in world frame [vx, vy, vz] (m/s)
    double vel[3];
    /// Angular velocity in body frame [wx, wy, wz] (rad/s)
    double ang_vel[3];
    /// Timestamp (simulation time in microseconds)
    uint64_t time_us;
    /// Valid flag (non-zero if data is valid)
    int valid;
} AviateModelState;

/// Motor command to send to gz-sim
typedef struct AviateMotorCommand {
    /// Motor velocities in rad/s (up to 8 motors)
    double velocities[8];
    /// Number of motors (typically 4 for quadcopter)
    int num_motors;
} AviateMotorCommand;

/// Initialize the bridge (called once at startup)
/// Returns 0 on success, non-zero on error
int aviate_gz_init(void);

/// Shutdown the bridge (called at cleanup)
void aviate_gz_shutdown(void);

/// Get current model state (zero-copy read from shared memory)
/// Returns 0 on success, non-zero if data not available
int aviate_gz_get_model_state(AviateModelState* out);

/// Set motor speeds (writes to shared memory, picked up by plugin)
/// Returns 0 on success, non-zero on error
int aviate_gz_set_motor_speeds(const AviateMotorCommand* cmd);

/// Get the simulation time in microseconds
uint64_t aviate_gz_get_sim_time_us(void);

/// Check if the bridge is connected to gz-sim
/// Returns non-zero if connected
int aviate_gz_is_connected(void);

#ifdef __cplusplus
}
#endif

#endif // AVIATE_GZ_BRIDGE_H
