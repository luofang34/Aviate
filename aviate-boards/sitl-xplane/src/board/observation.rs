//! Exact actuator observations at the X-Plane plant boundary.

use aviate_core::kernel::config::ActuatorCurveKind;
use aviate_hal_xil::sim_types::{SimActuatorCmd, SimImuData};

use super::wire::WireConstraints;

/// One causal sensor-to-actuator observation.
#[derive(Clone, Copy, Debug)]
pub struct XPlaneControlObservation {
    /// Simulator timestamp of the sensor sample.
    pub timestamp_us: u64,
    /// IMU sample that caused this actuator answer.
    pub imu: Option<SimImuData>,
    /// Force-domain lanes after injection and before wire constraints.
    pub pre_wire_force_lanes: [f32; 4],
    /// Force-domain lanes after all wire constraints and before the actuator curve.
    pub applied_force_lanes: [f32; 4],
    /// Constraints and safe fallbacks applied to this packet.
    pub constraint_flags: XPlaneConstraintFlags,
}

/// Causal constraint flags for one sensor-to-actuator packet.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct XPlaneConstraintFlags {
    /// Lane injection exceeded the normalized force range.
    pub injection_clamp: bool,
    /// The actuator lane count did not match the model.
    pub invalid_actuator_count: bool,
    /// The kernel did not produce an actuator answer for this packet.
    pub missing_actuator_answer: bool,
    /// The collective rate limiter changed the command.
    pub collective_rate: bool,
    /// The collective mean ceiling changed the command.
    pub mean_ceiling: bool,
    /// The per-lane ceiling changed the command.
    pub lane_ceiling: bool,
    /// The on-ground authority limit changed the command.
    pub ground_squeeze: bool,
    /// The external tuning trace did not accept this packet.
    pub tuning_trace_failure: bool,
}

impl XPlaneConstraintFlags {
    /// Return true when the plant did not receive the requested input unchanged.
    #[must_use]
    pub fn any(self) -> bool {
        self.injection_clamp
            || self.invalid_actuator_count
            || self.missing_actuator_answer
            || self.collective_rate
            || self.mean_ceiling
            || self.lane_ceiling
            || self.ground_squeeze
            || self.tuning_trace_failure
    }
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct PreparedActuatorCommand {
    pub(crate) valid_count: bool,
    pub(crate) pre_wire_force_lanes: [f32; 4],
    pub(crate) applied_force_lanes: [f32; 4],
    pub(crate) constraint_flags: XPlaneConstraintFlags,
}

#[allow(clippy::too_many_arguments)]
pub(crate) fn prepare_actuator_command(
    cmd: &mut SimActuatorCmd,
    lane_injection: [f32; 4],
    wire: &mut WireConstraints,
    curve: ActuatorCurveKind,
    lane_order: [u8; 4],
    motor_count: u8,
    fix_alt_m: Option<f32>,
    dt_sec: f32,
) -> PreparedActuatorCommand {
    let mut injection_clamped = false;
    for (lane, injection) in cmd.outputs.iter_mut().zip(lane_injection) {
        let requested = *lane + injection;
        let applied = requested.clamp(0.0, 1.0);
        injection_clamped |= applied != requested;
        *lane = applied;
    }
    let pre_wire_force_lanes = first_four(&cmd.outputs);
    if cmd.count != motor_count {
        cmd.outputs.fill(0.0);
        cmd.armed = false;
        return PreparedActuatorCommand {
            valid_count: false,
            pre_wire_force_lanes,
            applied_force_lanes: [0.0; 4],
            constraint_flags: XPlaneConstraintFlags {
                injection_clamp: injection_clamped,
                invalid_actuator_count: true,
                ..XPlaneConstraintFlags::default()
            },
        };
    }
    let wire_flags = wire.constrain(&mut cmd.outputs, cmd.count, cmd.armed, fix_alt_m, dt_sec);
    let applied_force_lanes = first_four(&cmd.outputs);
    apply_actuator_curve(curve, cmd);
    reorder_lanes(&mut cmd.outputs, lane_order);
    PreparedActuatorCommand {
        valid_count: true,
        pre_wire_force_lanes,
        applied_force_lanes,
        constraint_flags: XPlaneConstraintFlags {
            injection_clamp: injection_clamped,
            collective_rate: wire_flags.collective_rate,
            mean_ceiling: wire_flags.mean_ceiling,
            lane_ceiling: wire_flags.lane_ceiling,
            ground_squeeze: wire_flags.ground_squeeze,
            ..XPlaneConstraintFlags::default()
        },
    }
}

pub(crate) fn apply_actuator_curve(curve: ActuatorCurveKind, cmd: &mut SimActuatorCmd) {
    let lanes = usize::from(cmd.count).min(cmd.outputs.len());
    for lane in &mut cmd.outputs[..lanes] {
        *lane = curve
            .boundary_command(aviate_core::types::NormalizedThrust(*lane))
            .0;
    }
}

pub(crate) fn reorder_lanes(outputs: &mut [f32; 16], lane_order: [u8; 4]) {
    let source = *outputs;
    for (target, source_index) in lane_order.into_iter().enumerate() {
        outputs[target] = source[usize::from(source_index)];
    }
}

fn first_four(outputs: &[f32; 16]) -> [f32; 4] {
    [outputs[0], outputs[1], outputs[2], outputs[3]]
}
