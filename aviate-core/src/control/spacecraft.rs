//! Inner-loop spacecraft and launch/reentry vehicle controller.
//!
//! Tracks attitude, angular-rate, SAS pointing direction, and main-engine
//! throttle commands projected from outer-loop trajectory guidance (`burn`,
//! `rendezvous`, `entry`, `powered_descent`). Outer-loop orbital and entry
//! guidance stays in `Navigate` (`f64`); this controller executes the 1 kHz
//! `f32` Stability Augmentation System (SAS) attitude/rate stabilization loop,
//! quadratic proper-acceleration limiting (`MAX_PROPER_ACCEL_G`), low-altitude
//! aerodynamic fin ground-lock (`LANDING_FIN_LOCK_ALTITUDE_M`), reaction-wheel
//! momentum-dumping allocation, and 6-DOF translational RCS allocation.

use crate::control::law_invariants::STANDARD_GRAVITY_MPS2;
use crate::control::runtime::NoControllerState;
use crate::control::{
    AxisCommand, Command, ConfigMode, ControlMode, ControllerConfigMismatch, Limits,
    VehicleControlMode, VehicleController,
};
use crate::kernel::config::ResolvedKernelConfig;
use crate::math::{Quaternion, Vector3};
use crate::state::StateEstimate;
#[allow(unused_imports)]
use crate::types::FloatExt;
use crate::types::{
    MetersPerSecond, MetersPerSecondSquared, Normalized, NormalizedSigned, NormalizedThrust,
    Radians, RadiansPerSecond, Scalar,
};

/// Maximum allowable proper acceleration `[g]` during main-engine burns
/// and powered descent. Bounds structural and crew load factor to `4.0 g`.
pub const MAX_PROPER_ACCEL_G: Scalar = 4.0;

/// Altitude floor `[m]` below which aerodynamic fins lock to neutral
/// (`0.0`) in `ConfigMode::Cruise` so ground effect and landing-plume
/// recirculation cannot drive fin stall near touchdown.
pub const LANDING_FIN_LOCK_ALTITUDE_M: Scalar = 50.0;

/// Proportional attitude-error gain `[1/s]` mapping body-frame
/// quaternion vector error (`2 * q_err.xyz`) to desired angular rate.
pub const SPACECRAFT_ATT_KP: Scalar = 2.5;

/// Proportional angular-rate damping gain `[s/rad]` mapping body-rate
/// error to normalized torque command in `[-1.0, 1.0]`.
pub const SPACECRAFT_RATE_KP: Scalar = 0.4;

/// Minimum vector norm for SAS direction-lock unit-vector normalization.
/// Prevents division by zero when a degenerate direction vector is supplied.
pub const SAS_DIRECTION_NORM_FLOOR: Scalar = 1e-4;

/// Reaction-wheel normalized momentum threshold `[0, 1]` above which
/// rotational RCS thrusters assist attitude control and momentum dumping.
pub const RW_SATURATION_THRESHOLD: Scalar = 0.85;

/// Velocity-error deadband `[m/s]` per body axis for 6-DOF translational
/// RCS firing during rendezvous, docking, and fine velocity trim.
pub const RCS_VEL_DEADBAND_MPS: Scalar = 0.02;

/// Proportional gain `[1/(m/s)]` mapping body-axis velocity error outside
/// [`RCS_VEL_DEADBAND_MPS`] to normalized translational RCS demand.
pub const RCS_VEL_KP: Scalar = 2.5;

const _: () = assert!(MAX_PROPER_ACCEL_G > 0.0);
const _: () = assert!(LANDING_FIN_LOCK_ALTITUDE_M > 0.0);
const _: () = assert!(SPACECRAFT_ATT_KP > 0.0);
const _: () = assert!(SPACECRAFT_RATE_KP > 0.0);
const _: () = assert!(SAS_DIRECTION_NORM_FLOOR > 0.0);
const _: () = assert!(RW_SATURATION_THRESHOLD > 0.0);
const _: () = assert!(RW_SATURATION_THRESHOLD <= 1.0);
const _: () = assert!(RCS_VEL_DEADBAND_MPS > 0.0);
const _: () = assert!(RCS_VEL_KP > 0.0);

/// Spacecraft Stability Augmentation System (SAS) pointing mode.
///
/// Selected by `Pilotage` operator/AI intent and driven by reference
/// vectors or attitudes resolved by `Navigate`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SasMode {
    /// Damp 3-axis body angular rates to zero (`Kill-Rot`).
    KillRot,
    /// Hold a 3D target attitude quaternion.
    HoldAttitude(Quaternion),
    /// Lock the spacecraft primary thrust/pointing axis (`+X` body in NED
    /// convention) onto a reference unit direction vector in the local frame
    /// (`Prograde`, `Retrograde`, `Radial`, `Normal`, `Target`, or `ThrustVector`).
    DirectionLock([Scalar; 3]),
    /// Track a commanded atmospheric-entry bank angle about the velocity/roll
    /// axis while damping pitch and yaw rates.
    EntryBank(Radians),
}

/// Compute the target attitude quaternion that rotates the spacecraft primary
/// body axis (`+X`) onto `direction_ned` via the shortest arc from `current_att`.
#[must_use]
pub fn sas_direction_lock_attitude(
    current_att: Quaternion,
    direction_ned: [Scalar; 3],
) -> Quaternion {
    let cur_inv = Quaternion::new(
        current_att.w,
        -current_att.x,
        -current_att.y,
        -current_att.z,
    );
    let dir_body = cur_inv.rotate_vector(Vector3::new(
        direction_ned[0],
        direction_ned[1],
        direction_ned[2],
    ));
    let norm = (dir_body.x * dir_body.x + dir_body.y * dir_body.y + dir_body.z * dir_body.z).sqrt();
    if norm < SAS_DIRECTION_NORM_FLOOR {
        return current_att;
    }
    let ux = (dir_body.x / norm).clamp(-1.0, 1.0);
    let uy = dir_body.y / norm;
    let uz = dir_body.z / norm;

    // Cross product of body +X (1, 0, 0) with (ux, uy, uz) is (0, -uz, uy).
    let cross_x = 0.0;
    let cross_y = -uz;
    let cross_z = uy;
    let cross_norm = (cross_y * cross_y + cross_z * cross_z).sqrt();
    if cross_norm < SAS_DIRECTION_NORM_FLOOR {
        if ux >= 0.0 {
            return current_att;
        }
        let flip = Quaternion::from_axis_angle(Vector3::new(0.0, 0.0, 1.0), core::f32::consts::PI);
        return current_att.mul(&flip).normalize();
    }
    let angle = cross_norm.atan2(ux);
    let axis = Vector3::new(cross_x, cross_y / cross_norm, cross_z / cross_norm);
    let delta_q = Quaternion::from_axis_angle(axis, angle);
    current_att.mul(&delta_q).normalize()
}

/// Evaluate one SAS attitude/rate cycle for the requested [`SasMode`].
#[must_use]
pub fn evaluate_sas_mode(
    mode: SasMode,
    state: &StateEstimate,
    limits: &Limits,
    collective_thrust: NormalizedThrust,
) -> AxisCommand {
    let target_att = match mode {
        SasMode::KillRot => None,
        SasMode::HoldAttitude(q) => Some(q),
        SasMode::DirectionLock(dir) => Some(sas_direction_lock_attitude(state.attitude, dir)),
        SasMode::EntryBank(bank) => Some(Quaternion::from_axis_angle(
            Vector3::new(1.0, 0.0, 0.0),
            bank.0,
        )),
    };
    let (wx_cmd, wy_cmd, wz_cmd) = if let Some(q_sp) = target_att {
        attitude_error_rate_cmd(state.attitude, q_sp, limits)
    } else {
        (0.0, 0.0, 0.0)
    };
    rate_damping_axis_cmd(
        wx_cmd,
        wy_cmd,
        wz_cmd,
        state.angular_velocity,
        collective_thrust,
    )
}

/// Partition 3-axis attitude torque demand between Reaction Wheels (RW) and
/// rotational RCS thrusters based on each wheel's normalized stored angular
/// momentum `wheel_momentum_norm` in `[0.0, 1.0]`.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct ReactionWheelRcsAllocation {
    /// Normalized torque command routed to the 3-axis reaction wheels.
    pub reaction_wheel_torque: [NormalizedSigned; 3],
    /// Normalized torque command routed to the rotational RCS thrusters
    /// (for high-rate slews or reaction-wheel momentum desaturation).
    pub rcs_attitude_torque: [NormalizedSigned; 3],
}

/// Allocate 3-axis rotational torque between reaction wheels and RCS thrusters.
///
/// Below [`RW_SATURATION_THRESHOLD`], reaction wheels absorb 100% of the fine
/// attitude torque demand (`rcs_attitude_torque = 0`). Above the threshold,
/// rotational RCS takes over the axis demand to prevent wheel saturation and
/// assist momentum dumping.
#[must_use]
pub fn allocate_rw_and_rotational_rcs(
    torque_demand: [NormalizedSigned; 3],
    wheel_momentum_norm: [Normalized; 3],
) -> ReactionWheelRcsAllocation {
    let mut rw = [NormalizedSigned(0.0); 3];
    let mut rcs = [NormalizedSigned(0.0); 3];
    for i in 0..3 {
        let tau = torque_demand[i].0.clamp(-1.0, 1.0);
        let h = wheel_momentum_norm[i].0.clamp(0.0, 1.0);
        if h >= RW_SATURATION_THRESHOLD {
            rw[i] = NormalizedSigned(0.0);
            rcs[i] = NormalizedSigned(tau);
        } else {
            rw[i] = NormalizedSigned(tau);
            rcs[i] = NormalizedSigned(0.0);
        }
    }
    ReactionWheelRcsAllocation {
        reaction_wheel_torque: rw,
        rcs_attitude_torque: rcs,
    }
}

/// Allocate 6-DOF translational CG RCS thrust commands in `[-1.0, 1.0]` per
/// body axis from a body-frame velocity error vector (projected from
/// `Navigate`'s `GuidanceSetpoint::CartesianVelocity` during rendezvous or
/// fine orbital trim).
#[must_use]
pub fn allocate_translational_rcs(
    velocity_error_body_mps: [MetersPerSecond; 3],
) -> [NormalizedSigned; 3] {
    let mut out = [NormalizedSigned(0.0); 3];
    for i in 0..3 {
        let err = velocity_error_body_mps[i].0;
        let abs_err = err.abs();
        if abs_err > RCS_VEL_DEADBAND_MPS {
            let sign = if err >= 0.0 { 1.0 } else { -1.0 };
            let cmd = sign * (abs_err - RCS_VEL_DEADBAND_MPS) * RCS_VEL_KP;
            out[i] = NormalizedSigned(cmd.clamp(-1.0, 1.0));
        }
    }
    out
}

/// Clamp requested main-engine throttle so total proper acceleration
/// `||a_non_thrust + u * a_max * thrust_unit_body||` does not exceed
/// `MAX_PROPER_ACCEL_G * STANDARD_GRAVITY_MPS2`.
#[must_use]
pub fn limit_thrust_for_proper_accel(
    requested_throttle: NormalizedThrust,
    non_thrust_accel_mps2: [MetersPerSecondSquared; 3],
    max_thrust_accel_mps2: MetersPerSecondSquared,
    thrust_unit_body: [Scalar; 3],
) -> NormalizedThrust {
    let req = requested_throttle.0.clamp(0.0, 1.0);
    let a_max = max_thrust_accel_mps2.0;
    let lim = MAX_PROPER_ACCEL_G * STANDARD_GRAVITY_MPS2;
    let ax = non_thrust_accel_mps2[0].0;
    let ay = non_thrust_accel_mps2[1].0;
    let az = non_thrust_accel_mps2[2].0;
    let a0_sq = ax * ax + ay * ay + az * az;
    let lim_sq = lim * lim;

    if a_max <= 0.0 {
        return NormalizedThrust(0.0);
    }
    if a0_sq >= lim_sq {
        return NormalizedThrust(0.0);
    }

    let proj = ax * thrust_unit_body[0] + ay * thrust_unit_body[1] + az * thrust_unit_body[2];
    let disc = proj * proj + (lim_sq - a0_sq);
    let max_thrust_mps2 = (-proj + disc.sqrt()).max(0.0);
    let u_max = (max_thrust_mps2 / a_max).clamp(0.0, 1.0);
    NormalizedThrust(req.min(u_max))
}

fn attitude_error_rate_cmd(
    current_att: Quaternion,
    target_att: Quaternion,
    limits: &Limits,
) -> (Scalar, Scalar, Scalar) {
    let cur_inv = Quaternion::new(
        current_att.w,
        -current_att.x,
        -current_att.y,
        -current_att.z,
    );
    let q_err = cur_inv.mul(&target_att);
    let sign = if q_err.w < 0.0 { -2.0 } else { 2.0 };
    (
        (sign * q_err.x * SPACECRAFT_ATT_KP).clamp(-limits.max_roll_rate.0, limits.max_roll_rate.0),
        (sign * q_err.y * SPACECRAFT_ATT_KP)
            .clamp(-limits.max_pitch_rate.0, limits.max_pitch_rate.0),
        (sign * q_err.z * SPACECRAFT_ATT_KP).clamp(-limits.max_yaw_rate.0, limits.max_yaw_rate.0),
    )
}

fn rate_damping_axis_cmd(
    wx_cmd: Scalar,
    wy_cmd: Scalar,
    wz_cmd: Scalar,
    angular_velocity: [RadiansPerSecond; 3],
    collective: NormalizedThrust,
) -> AxisCommand {
    let roll = ((wx_cmd - angular_velocity[0].0) * SPACECRAFT_RATE_KP).clamp(-1.0, 1.0);
    let pitch = ((wy_cmd - angular_velocity[1].0) * SPACECRAFT_RATE_KP).clamp(-1.0, 1.0);
    let yaw = ((wz_cmd - angular_velocity[2].0) * SPACECRAFT_RATE_KP).clamp(-1.0, 1.0);

    AxisCommand {
        roll: NormalizedSigned(roll),
        pitch: NormalizedSigned(pitch),
        yaw: NormalizedSigned(yaw),
        collective: NormalizedThrust(collective.0.clamp(0.0, 1.0)),
    }
}

/// Inner-loop attitude, rate, SAS pointing, and proper-acceleration controller
/// for spacecraft, orbital stages, and reentry/landing vehicles.
#[derive(Debug, Clone, Copy, Default)]
pub struct SpacecraftController;

impl VehicleController for SpacecraftController {
    type RuntimeState = NoControllerState;

    // Registered in cert/algorithm_id_registry.toml as
    // "controller.spacecraft.v1".
    const ALGORITHM_ID: u64 = 0x4354_4C53_5043_5631; // "CTLSPCV1"

    fn verify_config_binding(
        &self,
        _cfg: &ResolvedKernelConfig,
    ) -> Result<(), ControllerConfigMismatch> {
        Ok(())
    }

    fn supports_mode(&self, mode: ControlMode) -> bool {
        !matches!(mode, ControlMode::PositionHold)
    }

    fn step(
        &self,
        _runtime: &mut Self::RuntimeState,
        state: &StateEstimate,
        command: &Command,
        flags: &VehicleControlMode,
        mode: ConfigMode,
        limits: &Limits,
    ) -> AxisCommand {
        if flags.flag_control_termination_enabled {
            return AxisCommand::default();
        }
        if !flags.flag_control_rates_enabled {
            return AxisCommand::default();
        }

        let max_thrust_accel = MetersPerSecondSquared(
            limits.max_load_factor.max(MAX_PROPER_ACCEL_G) * STANDARD_GRAVITY_MPS2,
        );
        let non_thrust_z = command.setpoint.vertical_deviation.map_or(0.0, |d| d.0);
        let collective = limit_thrust_for_proper_accel(
            command.setpoint.collective_thrust,
            [
                MetersPerSecondSquared(0.0),
                MetersPerSecondSquared(0.0),
                MetersPerSecondSquared(non_thrust_z),
            ],
            max_thrust_accel,
            [0.0, 0.0, 1.0],
        );

        let altitude_m = -state.position_ned[2].0;
        if mode == ConfigMode::Cruise && altitude_m < LANDING_FIN_LOCK_ALTITUDE_M {
            return AxisCommand {
                roll: NormalizedSigned(0.0),
                pitch: NormalizedSigned(0.0),
                yaw: NormalizedSigned(0.0),
                collective,
            };
        }

        if let Some(q_sp) = command.setpoint.attitude {
            return evaluate_sas_mode(SasMode::HoldAttitude(q_sp), state, limits, collective);
        }
        if let Some(rates) = command.setpoint.angular_rate {
            let wx_cmd = rates[0]
                .0
                .clamp(-limits.max_roll_rate.0, limits.max_roll_rate.0);
            let wy_cmd = rates[1]
                .0
                .clamp(-limits.max_pitch_rate.0, limits.max_pitch_rate.0);
            let wz_cmd = rates[2]
                .0
                .clamp(-limits.max_yaw_rate.0, limits.max_yaw_rate.0);
            return rate_damping_axis_cmd(
                wx_cmd,
                wy_cmd,
                wz_cmd,
                state.angular_velocity,
                collective,
            );
        }
        if let Some(vel) = command.setpoint.velocity {
            return evaluate_sas_mode(
                SasMode::DirectionLock([vel[0].0, vel[1].0, vel[2].0]),
                state,
                limits,
                collective,
            );
        }
        if let Some(bank) = command.setpoint.heading {
            return evaluate_sas_mode(SasMode::EntryBank(bank), state, limits, collective);
        }

        evaluate_sas_mode(SasMode::KillRot, state, limits, collective)
    }
}

#[cfg(test)]
mod tests;
