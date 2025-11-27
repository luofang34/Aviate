//! Gazebo Plugin FFI - Zero-copy physics data access
//!
//! This module provides Rust bindings to the AviateGzPlugin running inside gz-sim.
//! The plugin writes physics state to shared memory, which we read via FFI.
//!
//! ## Usage
//!
//! ```rust,ignore
//! use aviate_platform_sitl::gz_plugin::{GzPluginBridge, enu_to_ned};
//!
//! // Connect to the plugin (requires AviateGzPlugin loaded in gz-sim)
//! let bridge = GzPluginBridge::connect_with_retry(10, 500)?;
//!
//! // Read physics state
//! if let Some(state) = bridge.get_model_state() {
//!     let ned_pos = enu_to_ned(state.pos);
//!     println!("Position (NED): {:?}", ned_pos);
//! }
//!
//! // Send motor commands
//! bridge.set_motor_speeds(&[700.0, 700.0, 700.0, 700.0])?;
//! ```

use std::ffi::c_int;

/// Model state from gz-sim (all values in SI units, ENU frame)
#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct AviateModelState {
    /// Position in world frame [x, y, z] (meters, ENU)
    pub pos: [f64; 3],
    /// Orientation quaternion [w, x, y, z]
    pub quat: [f64; 4],
    /// Linear velocity in world frame [vx, vy, vz] (m/s)
    pub vel: [f64; 3],
    /// Angular velocity in body frame [wx, wy, wz] (rad/s)
    pub ang_vel: [f64; 3],
    /// Timestamp (simulation time in microseconds)
    pub time_us: u64,
    /// Valid flag (non-zero if data is valid)
    pub valid: c_int,
}

/// Motor command to send to gz-sim
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct AviateMotorCommand {
    /// Motor velocities in rad/s (up to 8 motors)
    pub velocities: [f64; 8],
    /// Number of motors (typically 4 for quadcopter)
    pub num_motors: c_int,
}

impl Default for AviateMotorCommand {
    fn default() -> Self {
        Self {
            velocities: [0.0; 8],
            num_motors: 4,
        }
    }
}

// FFI declarations - link against libaviate_gz_bridge.so
extern "C" {
    #[allow(dead_code)]
    fn aviate_gz_init() -> c_int;
    fn aviate_gz_init_instance(instance: c_int) -> c_int;
    fn aviate_gz_shutdown();
    fn aviate_gz_get_model_state(out: *mut AviateModelState) -> c_int;
    fn aviate_gz_set_motor_speeds(cmd: *const AviateMotorCommand) -> c_int;
    fn aviate_gz_get_sim_time_us() -> u64;
    fn aviate_gz_is_connected() -> c_int;
    fn aviate_gz_set_lockstep(enabled: c_int);
    fn aviate_gz_get_sim_step() -> u64;
    fn aviate_gz_ack_step(step: u64);
}

/// Error type for GzPluginBridge operations
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GzPluginError {
    /// Bridge not initialized or shared memory not available
    NotInitialized,
    /// Plugin not running (shared memory doesn't exist)
    PluginNotRunning,
    /// Data not valid yet
    DataNotValid,
    /// Failed to set motor speeds
    MotorCommandFailed,
}

impl std::fmt::Display for GzPluginError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::NotInitialized => write!(f, "GzPluginBridge not initialized"),
            Self::PluginNotRunning => write!(f, "AviateGzPlugin not running in Gazebo"),
            Self::DataNotValid => write!(f, "Model state data not valid"),
            Self::MotorCommandFailed => write!(f, "Failed to send motor command"),
        }
    }
}

impl std::error::Error for GzPluginError {}

/// Safe wrapper around the gz-sim plugin bridge
///
/// This struct manages the lifecycle of the shared memory connection
/// to the AviateGzPlugin running inside gz-sim.
///
/// Supports multi-vehicle simulation via instance IDs.
pub struct GzPluginBridge {
    initialized: bool,
    instance: u8,
}

impl GzPluginBridge {
    /// Create a new GzPluginBridge connection (instance 0)
    ///
    /// Returns an error if the AviateGzPlugin is not running in Gazebo.
    pub fn new() -> Result<Self, GzPluginError> {
        Self::for_instance(0)
    }

    /// Create a new GzPluginBridge connection for a specific instance
    ///
    /// For multi-vehicle simulation, each vehicle uses a different instance ID.
    /// Instance 0 connects to /aviate_gz_bridge, instance N to /aviate_gz_bridge_N.
    pub fn for_instance(instance: u8) -> Result<Self, GzPluginError> {
        let result = unsafe { aviate_gz_init_instance(instance as c_int) };
        match result {
            0 => Ok(Self { initialized: true, instance }),
            -1 => Err(GzPluginError::PluginNotRunning),
            _ => Err(GzPluginError::NotInitialized),
        }
    }

    /// Try to connect to the bridge, retrying if plugin not ready
    pub fn connect_with_retry(max_attempts: u32, delay_ms: u64) -> Result<Self, GzPluginError> {
        Self::connect_instance_with_retry(0, max_attempts, delay_ms)
    }

    /// Try to connect to a specific instance, retrying if plugin not ready
    pub fn connect_instance_with_retry(
        instance: u8,
        max_attempts: u32,
        delay_ms: u64,
    ) -> Result<Self, GzPluginError> {
        for attempt in 0..max_attempts {
            match Self::for_instance(instance) {
                Ok(bridge) => {
                    if attempt > 0 {
                        eprintln!(
                            "[GzPluginBridge] Instance {} connected after {} attempts",
                            instance,
                            attempt + 1
                        );
                    }
                    return Ok(bridge);
                }
                Err(GzPluginError::PluginNotRunning) => {
                    std::thread::sleep(std::time::Duration::from_millis(delay_ms));
                }
                Err(e) => return Err(e),
            }
        }
        Err(GzPluginError::PluginNotRunning)
    }

    /// Get the instance ID this bridge is connected to
    pub fn instance(&self) -> u8 {
        self.instance
    }

    /// Get the current model state from gz-sim
    ///
    /// Returns None if data is not yet valid (simulation hasn't started).
    pub fn get_model_state(&self) -> Option<AviateModelState> {
        if !self.initialized {
            return None;
        }

        let mut state = AviateModelState::default();
        let result = unsafe { aviate_gz_get_model_state(&mut state) };

        if result == 0 && state.valid != 0 {
            Some(state)
        } else {
            None
        }
    }

    /// Set motor speeds (rad/s)
    ///
    /// The velocities slice should contain 4 values for a quadcopter.
    pub fn set_motor_speeds(&self, velocities: &[f64]) -> Result<(), GzPluginError> {
        if !self.initialized {
            return Err(GzPluginError::NotInitialized);
        }

        let mut cmd = AviateMotorCommand::default();
        let n = velocities.len().min(8);
        cmd.velocities[..n].copy_from_slice(&velocities[..n]);
        cmd.num_motors = n as c_int;

        let result = unsafe { aviate_gz_set_motor_speeds(&cmd) };
        if result == 0 {
            Ok(())
        } else {
            Err(GzPluginError::MotorCommandFailed)
        }
    }

    /// Get simulation time in microseconds
    pub fn sim_time_us(&self) -> u64 {
        if !self.initialized {
            return 0;
        }
        unsafe { aviate_gz_get_sim_time_us() }
    }

    /// Check if connected to the gz-sim plugin
    pub fn is_connected(&self) -> bool {
        if !self.initialized {
            return false;
        }
        unsafe { aviate_gz_is_connected() != 0 }
    }

    /// Enable or disable lockstep mode
    ///
    /// When lockstep is enabled, Gazebo waits for the flight controller
    /// to acknowledge each simulation step before proceeding.
    /// This ensures deterministic simulation at the cost of real-time performance.
    pub fn set_lockstep(&self, enabled: bool) {
        if !self.initialized {
            return;
        }
        unsafe { aviate_gz_set_lockstep(if enabled { 1 } else { 0 }) }
    }

    /// Get the current simulation step count
    ///
    /// In lockstep mode, this increments after each physics step.
    /// Use this to detect new data and acknowledge steps.
    pub fn sim_step(&self) -> u64 {
        if !self.initialized {
            return 0;
        }
        unsafe { aviate_gz_get_sim_step() }
    }

    /// Acknowledge a simulation step
    ///
    /// Call this after processing a step's data to allow Gazebo to proceed.
    /// In lockstep mode, Gazebo blocks until this is called.
    pub fn ack_step(&self, step: u64) {
        if !self.initialized {
            return;
        }
        unsafe { aviate_gz_ack_step(step) }
    }

    /// Wait for a new simulation step and process it
    ///
    /// This is a convenience method for lockstep mode that:
    /// 1. Waits for new state data (step > last_step)
    /// 2. Calls the processor function with the state
    /// 3. Acknowledges the step
    ///
    /// Returns None if no new state is available within timeout.
    pub fn wait_and_process<F, R>(
        &self,
        last_step: u64,
        timeout_us: u64,
        processor: F,
    ) -> Option<(u64, R)>
    where
        F: FnOnce(&AviateModelState) -> R,
    {
        let start = std::time::Instant::now();
        let timeout = std::time::Duration::from_micros(timeout_us);

        loop {
            let current_step = self.sim_step();
            if current_step > last_step {
                if let Some(state) = self.get_model_state() {
                    let result = processor(&state);
                    self.ack_step(current_step);
                    return Some((current_step, result));
                }
            }

            if start.elapsed() >= timeout {
                return None;
            }

            std::thread::sleep(std::time::Duration::from_micros(10));
        }
    }
}

impl Drop for GzPluginBridge {
    fn drop(&mut self) {
        if self.initialized {
            unsafe { aviate_gz_shutdown() };
            self.initialized = false;
        }
    }
}

/// Convert ENU position to NED
///
/// Gazebo uses ENU (East-North-Up), MAVLink uses NED (North-East-Down)
/// - ENU x (east)  -> NED y (east)
/// - ENU y (north) -> NED x (north)
/// - ENU z (up)    -> NED z (down, negated)
#[inline]
pub fn enu_to_ned(enu: [f64; 3]) -> [f64; 3] {
    [enu[1], enu[0], -enu[2]]
}

/// Convert ENU velocity to NED
#[inline]
pub fn enu_vel_to_ned(enu_vel: [f64; 3]) -> [f64; 3] {
    [enu_vel[1], enu_vel[0], -enu_vel[2]]
}

/// Convert ENU position to NED (f32 version)
#[inline]
pub fn enu_to_ned_f32(enu: [f64; 3]) -> [f32; 3] {
    [enu[1] as f32, enu[0] as f32, -enu[2] as f32]
}

/// Convert ENU velocity to NED (f32 version)
#[inline]
pub fn enu_vel_to_ned_f32(enu_vel: [f64; 3]) -> [f32; 3] {
    [enu_vel[1] as f32, enu_vel[0] as f32, -enu_vel[2] as f32]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_enu_to_ned() {
        // ENU: x=east, y=north, z=up
        // NED: x=north, y=east, z=down
        let enu = [1.0, 2.0, 3.0]; // 1m east, 2m north, 3m up
        let ned = enu_to_ned(enu);
        assert_eq!(ned, [2.0, 1.0, -3.0]); // 2m north, 1m east, 3m down
    }

    #[test]
    fn test_motor_command_default() {
        let cmd = AviateMotorCommand::default();
        assert_eq!(cmd.num_motors, 4);
        assert_eq!(cmd.velocities, [0.0; 8]);
    }
}
