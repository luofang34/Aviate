//! Simulator truth as the synthesizer's `TruthSample`, and the world
//! generation it belongs to.

use aviate_backend_mavlink_hil::HilStateQuaternion;
use aviate_hal_xil::synthesis::TruthSample;
use aviate_hal_xil::VehicleState;
use world_core::{GeodeticPosition, LocalTangentPlane};

/// Which simulated world the truth stream is in.
///
/// The sidecar starts a new world with each TCP session and restarts
/// simulation time with it. A new session, or a time that runs
/// backwards within one, is therefore a new generation.
#[derive(Debug, Default)]
pub(crate) struct GenerationTracker {
    session: Option<u64>,
    last_time_us: Option<u64>,
    generation: u32,
    steps: u64,
}

impl GenerationTracker {
    /// The generation of a state stamped `time_us` on session `session`,
    /// and its step index within that generation.
    pub(crate) fn observe(&mut self, session: u64, time_us: u64) -> (u32, u64) {
        let new_session = self.session.is_some_and(|known| known != session);
        let rewound = self.last_time_us.is_some_and(|last| time_us < last);
        if new_session || rewound {
            self.generation = self.generation.wrapping_add(1);
            self.steps = 0;
        }
        self.session = Some(session);
        self.last_time_us = Some(time_us);
        let step = self.steps;
        self.steps = self.steps.wrapping_add(1);
        (self.generation, step)
    }
}

/// The truth sample a state gives in `plane`, or `None` for a position
/// the plane cannot express.
///
/// The sidecar's altitude is on its ellipsoid, which it treats as the
/// world's sea-level surface; the origin's sea-level elevation is taken
/// as its ellipsoidal height the same way, so the two meet on one
/// surface.
pub(crate) fn truth_sample(
    state: &HilStateQuaternion,
    plane: &LocalTangentPlane,
    generation: u32,
    step: u64,
) -> Option<TruthSample> {
    let position = GeodeticPosition::wgs84(
        f64::from(state.lat) * 1e-7,
        f64::from(state.lon) * 1e-7,
        f64::from(state.alt) * 1e-3,
    );
    let ned = plane.to_ned(position).ok()?;
    Some(TruthSample {
        time_us: state.time_usec,
        reset_generation: generation,
        step,
        position_ned_m: [ned.north_m as f32, ned.east_m as f32, ned.down_m as f32],
        attitude: state.attitude_quaternion,
    })
}

/// The vehicle a state states: position from its truth sample, the
/// state's NED velocity (cm/s) and body rates, and its attitude.
pub(crate) fn vehicle_state(state: &HilStateQuaternion, truth: &TruthSample) -> VehicleState {
    VehicleState {
        position: truth.position_ned_m,
        velocity: [
            f32::from(state.vx) / 100.0,
            f32::from(state.vy) / 100.0,
            f32::from(state.vz) / 100.0,
        ],
        orientation: state.attitude_quaternion,
        angular_velocity: [state.rollspeed, state.pitchspeed, state.yawspeed],
        valid: true,
    }
}

#[cfg(test)]
mod tests;
