//! Run simulator missions through the `SimulatorBackend` contract.

mod commands;
mod criteria;
mod execution;
mod frames;
mod mavlink;
mod multi;
mod trace;

#[cfg(test)]
mod tests;

pub use mavlink::MavClient;
pub use multi::{run_test_config, run_test_config_from_current_state, TestResult};

use crate::{ResetGeneration, SimulatorBackend, VehicleState};

/// One state sample from a mission phase.
#[derive(Debug, Clone, Copy)]
pub struct TraceSample {
    /// Time from the start of the phase, in seconds.
    pub elapsed: f32,
    /// Simulation time for the sample, in microseconds.
    pub sim_time_us: u64,
    /// North-east-down position, in meters.
    pub position: [f32; 3],
    /// North-east-down velocity, in meters per second.
    pub velocity: [f32; 3],
    /// Body-to-world attitude quaternion.
    pub attitude: [f32; 4],
    /// Body angular velocity, in radians per second.
    pub angular_velocity: [f32; 3],
}

/// Run one mission on one simulator backend.
pub struct MissionRunner<B: SimulatorBackend> {
    backend: B,
    fault_client: Option<crate::fault_protocol::FaultClient>,
    vehicle_id: String,
    last_step: u64,
    last_simulation_time: std::time::Duration,
    current_state: VehicleState,
    start_position: [f32; 3],
    armed: bool,
    max_altitude: f32,
    generation: ResetGeneration,
    next_directive_id: u64,
    next_command_sequence: u32,
}
