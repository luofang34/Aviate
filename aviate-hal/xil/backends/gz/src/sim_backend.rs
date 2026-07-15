//! SimulatorBackend implementation for Gazebo
//!
//! This module provides the `GazeboSimBackend` which implements the
//! `SimulatorBackend` trait from aviate-hal-xil, enabling backend-agnostic
//! mission execution with Gazebo.

#[cfg(feature = "gz-plugin")]
use crate::plugin::{
    enu_quat_to_ned_f32, enu_to_ned_f32, flu_to_frd_f32, GzPluginBridge, GzPluginError,
};

#[cfg(feature = "gz-plugin")]
use aviate_hal_xil::{SimulatorBackend, SimulatorError, VehicleState};

/// Gazebo simulator backend
///
/// Wraps `GzPluginBridge` to implement the `SimulatorBackend` trait.
/// This enables using the generic `MissionRunner` with Gazebo.
#[cfg(feature = "gz-plugin")]
pub struct GazeboSimBackend {
    bridge: Option<GzPluginBridge>,
    instance: u8,
}

#[cfg(feature = "gz-plugin")]
impl GazeboSimBackend {
    /// Create a new Gazebo backend (not yet connected)
    pub fn new(instance: u8) -> Self {
        Self {
            bridge: None,
            instance,
        }
    }

    /// Create and connect to Gazebo in one step
    pub fn connect_new(instance: u8, timeout_ms: u64) -> Result<Self, SimulatorError> {
        let mut backend = Self::new(instance);
        backend.connect(instance, timeout_ms)?;
        Ok(backend)
    }
}

#[cfg(feature = "gz-plugin")]
impl SimulatorBackend for GazeboSimBackend {
    fn name(&self) -> &str {
        "gazebo"
    }

    fn connect(&mut self, instance: u8, timeout_ms: u64) -> Result<(), SimulatorError> {
        let max_attempts = (timeout_ms / 500).max(1) as u32;

        match GzPluginBridge::connect_instance_with_retry(instance, max_attempts, 500) {
            Ok(bridge) => {
                self.bridge = Some(bridge);
                self.instance = instance;
                Ok(())
            }
            Err(GzPluginError::PluginNotRunning) => Err(SimulatorError::NotAvailable(
                "Gazebo plugin not running".to_string(),
            )),
            Err(e) => Err(SimulatorError::ConnectionFailed(e.to_string())),
        }
    }

    fn is_connected(&self) -> bool {
        self.bridge.as_ref().is_some_and(|b| b.is_connected())
    }

    fn get_vehicle_state(&self) -> Option<VehicleState> {
        let bridge = self.bridge.as_ref()?;
        let state = bridge.get_model_state()?;

        // Convert from gz's ENU-world / FLU-body convention to
        // NED-world / FRD-body, the convention every aviate
        // consumer (FC kernel, test harness criteria, MAVLink
        // bridge) operates in. Without the orientation conversion
        // here the vehicle state surfaces gz's raw quaternion —
        // mixed-frame data that silently breaks attitude
        // criteria and any post-mortem trace.
        let ned_pos = enu_to_ned_f32(state.pos);
        let ned_vel = enu_to_ned_f32(state.vel);
        let ned_quat = enu_quat_to_ned_f32(state.quat);
        let body_ang_vel_frd = flu_to_frd_f32(state.ang_vel);

        Some(VehicleState {
            position: ned_pos,
            velocity: ned_vel,
            orientation: ned_quat,
            angular_velocity: body_ang_vel_frd,
            time_us: state.time_us,
            valid: state.valid != 0,
        })
    }

    fn set_motor_speeds(&mut self, speeds: &[f64]) -> Result<(), SimulatorError> {
        let bridge = self
            .bridge
            .as_ref()
            .ok_or_else(|| SimulatorError::NotAvailable("Not connected".to_string()))?;

        bridge
            .set_motor_speeds(speeds)
            .map_err(|e| SimulatorError::ConnectionFailed(e.to_string()))
    }

    fn set_lockstep(&mut self, enabled: bool) {
        if let Some(ref bridge) = self.bridge {
            // The trait cannot report failure, so surface it here:
            // a run that quietly free-runs when the harness asked to
            // step is non-reproducible evidence, not a minor warning.
            if let Err(e) = bridge.set_lockstep(enabled) {
                log::error!("lockstep {enabled} requested but not armed: {e}");
            }
        }
    }

    fn sim_step(&self) -> u64 {
        self.bridge.as_ref().map(|b| b.sim_step()).unwrap_or(0)
    }

    fn ack_step(&mut self, step: u64) {
        if let Some(ref bridge) = self.bridge {
            bridge.ack_step(step);
        }
    }

    fn instance(&self) -> u8 {
        self.instance
    }
}

// Stub implementation when gz-plugin feature is not enabled
#[cfg(not(feature = "gz-plugin"))]
pub struct GazeboSimBackend {
    instance: u8,
}

#[cfg(not(feature = "gz-plugin"))]
impl GazeboSimBackend {
    pub fn new(instance: u8) -> Self {
        Self { instance }
    }

    pub fn connect_new(
        _instance: u8,
        _timeout_ms: u64,
    ) -> Result<Self, aviate_hal_xil::SimulatorError> {
        Err(aviate_hal_xil::SimulatorError::NotAvailable(
            "gz-plugin feature not enabled".to_string(),
        ))
    }
}

#[cfg(not(feature = "gz-plugin"))]
impl aviate_hal_xil::SimulatorBackend for GazeboSimBackend {
    fn name(&self) -> &str {
        "gazebo"
    }

    fn connect(
        &mut self,
        _instance: u8,
        _timeout_ms: u64,
    ) -> Result<(), aviate_hal_xil::SimulatorError> {
        Err(aviate_hal_xil::SimulatorError::NotAvailable(
            "gz-plugin feature not enabled".to_string(),
        ))
    }

    fn is_connected(&self) -> bool {
        false
    }

    fn get_vehicle_state(&self) -> Option<aviate_hal_xil::VehicleState> {
        None
    }

    fn set_motor_speeds(&mut self, _speeds: &[f64]) -> Result<(), aviate_hal_xil::SimulatorError> {
        Err(aviate_hal_xil::SimulatorError::NotAvailable(
            "gz-plugin feature not enabled".to_string(),
        ))
    }

    fn set_lockstep(&mut self, _enabled: bool) {}

    fn sim_step(&self) -> u64 {
        0
    }

    fn ack_step(&mut self, _step: u64) {}

    fn instance(&self) -> u8 {
        self.instance
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_gazebo_backend_new() {
        let backend = GazeboSimBackend::new(0);
        assert_eq!(backend.instance, 0);
    }
}
