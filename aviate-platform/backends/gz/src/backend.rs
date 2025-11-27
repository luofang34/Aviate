//! Gazebo Backend Implementation
//!
//! Implements the `KinematicsBackend` trait from aviate-platform-xil for Gazebo Sim.

use std::time::Duration;

#[cfg(feature = "gz-plugin")]
use aviate_platform_xil::{
    BackendConfig, BackendError, KinematicsBackend, LockstepMode, World,
    Position, Velocity, Quaternion, AngularVelocity,
};

#[cfg(not(feature = "gz-plugin"))]
use aviate_platform_xil::{
    BackendConfig, BackendError, KinematicsBackend, World,
};

#[cfg(feature = "gz-plugin")]
use crate::plugin::{GzPluginBridge, enu_to_ned};

/// Gazebo backend for XIL simulation
///
/// Connects to gz-sim via shared memory FFI through the AviateGzPlugin.
pub struct GazeboBackend {
    #[cfg(feature = "gz-plugin")]
    plugin: Option<GzPluginBridge>,

    #[cfg(not(feature = "gz-plugin"))]
    _phantom: std::marker::PhantomData<()>,

    step_count: u64,
    sim_time: Duration,
    last_step: u64,
}

impl GazeboBackend {
    /// Create a new Gazebo backend (not yet connected)
    pub fn new() -> Self {
        Self {
            #[cfg(feature = "gz-plugin")]
            plugin: None,
            #[cfg(not(feature = "gz-plugin"))]
            _phantom: std::marker::PhantomData,
            step_count: 0,
            sim_time: Duration::ZERO,
            last_step: 0,
        }
    }
}

impl Default for GazeboBackend {
    fn default() -> Self {
        Self::new()
    }
}

impl KinematicsBackend for GazeboBackend {
    fn name(&self) -> &str {
        "gazebo"
    }

    fn start(&mut self, cfg: &BackendConfig) -> Result<(), BackendError> {
        #[cfg(feature = "gz-plugin")]
        {
            let timeout_ms = match cfg.lockstep {
                LockstepMode::Lockstep { timeout_us } => timeout_us / 1000,
                LockstepMode::Async => 5000,
            };

            let max_attempts = (timeout_ms / 500).max(1) as u32;

            match GzPluginBridge::connect_with_retry(max_attempts, 500) {
                Ok(plugin) => {
                    // Enable lockstep if configured
                    if let LockstepMode::Lockstep { .. } = cfg.lockstep {
                        plugin.set_lockstep(true);
                    }
                    self.plugin = Some(plugin);
                    Ok(())
                }
                Err(_) => Err(BackendError::ConnectionFailed("Failed to connect to Gazebo plugin".into())),
            }
        }

        #[cfg(not(feature = "gz-plugin"))]
        {
            let _ = cfg;
            Err(BackendError::NotSupported("gz-plugin feature not enabled".into()))
        }
    }

    fn step(&mut self, world: &mut World) -> Result<Duration, BackendError> {
        #[cfg(feature = "gz-plugin")]
        {
            let plugin = self.plugin.as_ref()
                .ok_or_else(|| BackendError::NotInitialized)?;

            // Wait for new step from Gazebo
            let current_step = plugin.sim_step();
            if current_step > self.last_step {
                if let Some(state) = plugin.get_model_state() {
                    // Convert from ENU to NED
                    let ned_pos = enu_to_ned(state.pos);
                    let ned_vel = enu_to_ned(state.vel);

                    // Update world state for the first entity (instance 0)
                    if let Some(entity) = world.get_by_instance_mut(0) {
                        entity.state.position = Position::new(ned_pos[0], ned_pos[1], ned_pos[2]);
                        entity.state.velocity = Velocity::new(ned_vel[0], ned_vel[1], ned_vel[2]);
                        entity.state.orientation = Quaternion::new(
                            state.quat[0], state.quat[1], state.quat[2], state.quat[3]
                        );
                        entity.state.angular_velocity = AngularVelocity {
                            roll_rate: state.ang_vel[0],
                            pitch_rate: state.ang_vel[1],
                            yaw_rate: state.ang_vel[2],
                        };
                    }

                    // Update simulation time
                    self.sim_time = Duration::from_micros(state.time_us);

                    // Acknowledge the step for lockstep mode
                    plugin.ack_step(current_step);
                }

                self.last_step = current_step;
                self.step_count += 1;
            }

            Ok(self.sim_time)
        }

        #[cfg(not(feature = "gz-plugin"))]
        {
            let _ = world;
            Err(BackendError::NotSupported("gz-plugin feature not enabled".into()))
        }
    }

    fn poll_ready(&self) -> bool {
        #[cfg(feature = "gz-plugin")]
        {
            self.plugin.as_ref().map(|p| p.is_connected()).unwrap_or(false)
        }

        #[cfg(not(feature = "gz-plugin"))]
        false
    }

    fn sim_time(&self) -> Duration {
        self.sim_time
    }

    fn step_count(&self) -> u64 {
        self.step_count
    }

    fn stop(&mut self) -> Result<(), BackendError> {
        #[cfg(feature = "gz-plugin")]
        {
            self.plugin = None;
        }
        Ok(())
    }

    fn reset(&mut self) -> Result<(), BackendError> {
        self.step_count = 0;
        self.sim_time = Duration::ZERO;
        self.last_step = 0;
        Ok(())
    }
}
