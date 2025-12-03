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

/// Maximum number of vehicle instances supported
#define AVIATE_MAX_INSTANCES 8

/// Initialize the bridge for a specific instance (multi-vehicle support)
/// Instance 0 uses /aviate_gz_bridge, instance N uses /aviate_gz_bridge_N
/// Returns 0 on success, non-zero on error
int aviate_gz_init_instance(int instance);

/// Shutdown the bridge for a specific instance
void aviate_gz_shutdown_instance(int instance);

/// Get current model state for a specific instance
/// Returns 0 on success, non-zero if data not available
int aviate_gz_get_model_state_instance(int instance, AviateModelState* out);

/// Set motor speeds for a specific instance
/// Returns 0 on success, non-zero on error
int aviate_gz_set_motor_speeds_instance(int instance, const AviateMotorCommand* cmd);

/// Get the simulation time in microseconds for a specific instance
uint64_t aviate_gz_get_sim_time_us_instance(int instance);

/// Check if a specific instance is connected to gz-sim
/// Returns non-zero if connected
int aviate_gz_is_connected_instance(int instance);

/// Enable/disable lockstep mode for a specific instance
void aviate_gz_set_lockstep_instance(int instance, int enabled);

/// Get the current simulation step count for a specific instance
uint64_t aviate_gz_get_sim_step_instance(int instance);

/// Acknowledge a simulation step for a specific instance
void aviate_gz_ack_step_instance(int instance, uint64_t step);

// ============================================================================
// Legacy API (instance 0 only, for backwards compatibility)
// ============================================================================

/// Initialize the bridge for instance 0 (backwards compatible)
int aviate_gz_init(void);
void aviate_gz_shutdown(void);
int aviate_gz_get_model_state(AviateModelState* out);
int aviate_gz_set_motor_speeds(const AviateMotorCommand* cmd);
uint64_t aviate_gz_get_sim_time_us(void);
int aviate_gz_is_connected(void);
void aviate_gz_set_lockstep(int enabled);
uint64_t aviate_gz_get_sim_step(void);
void aviate_gz_ack_step(uint64_t step);

#ifdef __cplusplus
}
#endif

#endif // AVIATE_GZ_BRIDGE_H
