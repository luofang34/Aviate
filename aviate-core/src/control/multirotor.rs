//! Multirotor cascade controller: position → velocity → attitude
//! → rate, driving the mixer's `AxisCommand`. Tuning lives in
//! `ResolvedKernelConfig.cascade_gains` (DRQ-CTL-001); persistent
//! state (integrators, derivative memories) lives in
//! `KernelState.controller` as a `MultirotorRuntimeState`.

use crate::control::attitude::AttitudeController;
use crate::control::cascade_gains::CascadeGains;
use crate::control::position::PositionController;
use crate::control::rate::{RateController, RateLoopState};
use crate::control::velocity::{VelocityController, VelocityLoopState};
use crate::control::{
    AxisCommand, Command, ConfigMode, Limits, Scalar, Setpoint, VehicleControlMode,
    VehicleController,
};
use crate::math::{Quaternion, Vector3};
use crate::state::StateEstimate;
use crate::types::{MetersPerSecond, Radians};

/// Persistent runtime state for the multirotor cascade. Owned by
/// `KernelState.controller`. Reset on every transition that
/// invalidates accumulated memory (`disarm`, `ground_reset`,
/// `check_critical_faults`, control-law degradation).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MultirotorRuntimeState {
    pub velocity_loop: VelocityLoopState,
    pub rate_loop: RateLoopState,
    /// Last velocity setpoint seen by the velocity loop. Used to
    /// derive an acceleration feedforward via finite difference,
    /// closing the position-loop time derivative without needing
    /// an analytical form per axis.
    pub last_vel_sp_ned: Vector3<MetersPerSecond>,
    /// Whether `last_vel_sp_ned` carries a real previous sample.
    /// First cycle outputs zero feedforward instead of
    /// differentiating against the default (zero) value.
    pub vel_sp_primed: bool,
    /// Per-cycle interval used for the velocity-loop integrator
    /// and rate-loop derivative. The kernel writes a validated
    /// interval before each controller step.
    pub dt_sec: Scalar,
    /// Effective mode from the preceding controller cycle.
    pub previous_effective_mode: Option<crate::control::ControlMode>,
    /// Effective topology from the preceding controller cycle.
    pub previous_topology: Option<crate::control::EffectiveControlTopology>,
    /// Exact controller output from the preceding cycle.
    pub last_axis_command: AxisCommand,
    /// Whether `last_axis_command` contains a completed controller cycle.
    pub axis_command_primed: bool,
}

impl Default for MultirotorRuntimeState {
    fn default() -> Self {
        Self {
            velocity_loop: VelocityLoopState::default(),
            rate_loop: RateLoopState::default(),
            last_vel_sp_ned: Vector3::new(
                MetersPerSecond(0.0),
                MetersPerSecond(0.0),
                MetersPerSecond(0.0),
            ),
            vel_sp_primed: false,
            dt_sec: 0.0,
            previous_effective_mode: None,
            previous_topology: None,
            last_axis_command: AxisCommand::default(),
            axis_command_primed: false,
        }
    }
}

impl crate::control::runtime::ControllerRuntimeState for MultirotorRuntimeState {
    fn set_cycle_interval(&mut self, interval: crate::types::Seconds) {
        self.dt_sec = if interval.0.is_finite() && interval.0 > 0.0 {
            interval.0
        } else {
            0.0
        };
    }

    fn reset(&mut self) {
        self.velocity_loop.reset();
        self.rate_loop.reset();
        self.last_vel_sp_ned = Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        );
        self.vel_sp_primed = false;
        // Timed terms stay inactive until the caller supplies a valid
        // interval for a controller step.
        self.dt_sec = 0.0;
        self.previous_effective_mode = None;
        self.previous_topology = None;
        self.last_axis_command = AxisCommand::default();
        self.axis_command_primed = false;
    }
}

impl crate::replicable::Replicable for MultirotorRuntimeState {
    // 19 f32 lanes, two stable option tags, four output lanes, and one
    // priming tag. EVERY persistent field of
    // the runtime state must appear here: an omitted field lets two
    // lockstep channels diverge in hidden state while comparing
    // byte-equal, surfacing one cycle later as differing actuator
    // outputs with no witness (#141 — last_vel_filt_ned and d_primed
    // were missing). The per-field mutation test below is the
    // guardrail: adding a field without encoding it fails the test.
    const ENCODED_LEN: usize = 95;

    fn encode_canonical(&self, buf: &mut [u8]) -> usize {
        let fields: [f32; 19] = [
            self.velocity_loop.integrator_ned.x.0,
            self.velocity_loop.integrator_ned.y.0,
            self.velocity_loop.integrator_ned.z.0,
            self.velocity_loop.last_vel_filt_ned.x.0,
            self.velocity_loop.last_vel_filt_ned.y.0,
            self.velocity_loop.last_vel_filt_ned.z.0,
            self.rate_loop.meas_filtered_prev.x.0,
            self.rate_loop.meas_filtered_prev.y.0,
            self.rate_loop.meas_filtered_prev.z.0,
            self.rate_loop.integral[0],
            self.rate_loop.integral[1],
            self.rate_loop.integral[2],
            self.last_vel_sp_ned.x.0,
            self.last_vel_sp_ned.y.0,
            self.last_vel_sp_ned.z.0,
            // Booleans serialized as 0.0/1.0 to keep the
            // encoding all-f32; they're picked back up by the
            // cross-channel reader by structural shape.
            if self.vel_sp_primed { 1.0 } else { 0.0 },
            if self.velocity_loop.d_primed {
                1.0
            } else {
                0.0
            },
            if self.rate_loop.primed { 1.0 } else { 0.0 },
            self.dt_sec,
        ];
        // Clamp rather than slice: `Replicable` promises
        // `min(buf.len(), ENCODED_LEN)` bytes written, and a caller
        // walking a composite state hands over whatever is left of its
        // buffer. An unclamped `buf[i * 4..i * 4 + 4]` turns a short
        // buffer into a panic in a crate that denies them.
        let mut written = 0usize;
        for &v in fields.iter() {
            written += crate::replicable::copy_into(buf, written, &v.to_le_bytes());
        }
        written += crate::replicable::copy_into(
            buf,
            written,
            &[crate::control::transfer::mode_option_tag(
                self.previous_effective_mode,
            )],
        );
        written += crate::replicable::copy_into(
            buf,
            written,
            &[crate::control::transfer::topology_option_tag(
                self.previous_topology,
            )],
        );
        for value in [
            self.last_axis_command.roll.0,
            self.last_axis_command.pitch.0,
            self.last_axis_command.yaw.0,
            self.last_axis_command.collective.0,
        ] {
            written += crate::replicable::copy_into(buf, written, &value.to_le_bytes());
        }
        written +=
            crate::replicable::copy_into(buf, written, &[u8::from(self.axis_command_primed)]);
        written
    }
}

pub struct MultirotorController {
    /// Canonical identity over the gains and hover seed this
    /// controller copied at construction; the builder compares it
    /// against the resolved configuration before a kernel exists.
    tuning_identity: u64,
    pos_ctrl: PositionController,
    vel_ctrl: VelocityController,
    rate_ctrl: RateController,
    att_ctrl: AttitudeController,
}

impl Default for MultirotorController {
    fn default() -> Self {
        Self::from_gains(CascadeGains::x500_defaults(), 0.5)
    }
}

impl MultirotorController {
    /// Gains the velocity loop actually flies. Read-only: tuning is
    /// fixed at construction together with the identity the builder
    /// verifies, and cannot be edited apart from it afterwards.
    ///
    /// ```compile_fail
    /// let mut c = aviate_core::control::multirotor::MultirotorController::default();
    /// c.vel_ctrl.gains = aviate_core::control::cascade_gains::CascadeGains::x500_defaults();
    /// ```
    pub fn velocity_gains(&self) -> &crate::control::cascade_gains::CascadeGains {
        &self.vel_ctrl.gains
    }

    /// Gains the rate loop actually flies (read-only; see
    /// [`Self::velocity_gains`]).
    pub fn rate_gains(&self) -> &crate::control::cascade_gains::CascadeGains {
        &self.rate_ctrl.gains
    }

    /// Hover trim the velocity loop actually flies (read-only).
    pub fn hover_thrust_norm(&self) -> Scalar {
        self.vel_ctrl.hover_thrust_norm
    }

    /// Construct from explicit tuning. The single authoritative
    /// source of gains is `CascadeGains` (mirrored from
    /// `ResolvedKernelConfig`); the four sub-controllers carry
    /// the same struct by value so a kernel construction step
    /// that builds both the config and this controller from the
    /// same `CascadeGains` instance keeps them in lockstep
    /// by construction.
    pub fn from_gains(gains: CascadeGains, hover_thrust_norm: Scalar) -> Self {
        Self {
            tuning_identity: crate::kernel::config::canonical_controller_tuning_identity(
                &gains,
                hover_thrust_norm,
            ),
            pos_ctrl: PositionController::with_limits(
                gains.pos_p,
                gains.pos_accel_limits,
                gains.pos_vel_caps,
            ),
            vel_ctrl: VelocityController::new(gains, hover_thrust_norm),
            rate_ctrl: RateController::new(gains),
            att_ctrl: AttitudeController::new(gains.att_p, gains.att_max_rate_cmd),
        }
    }

    /// Vertical velocity command for the altitude / climb-rate hold.
    ///
    /// A `vertical_speed` setpoint is a direct climb-rate command; an
    /// `altitude` setpoint is held by shaping the altitude error through
    /// the position loop's vertical sqrt-controller (altitude error →
    /// climb rate). The result is clamped to the climb/descent envelope
    /// so the vertical loop never chases a rate the airframe is not
    /// authorized to fly. `None` when no vertical setpoint is present.
    fn vertical_velocity_setpoint(
        &self,
        setpoint: &Setpoint,
        state: &StateEstimate,
        limits: &Limits,
    ) -> Option<MetersPerSecond> {
        let vertical = if let Some(vspeed) = setpoint.vertical_speed {
            vspeed.0
        } else {
            // NED z is down-positive; altitude is up-positive, so the
            // held target in NED is the negated altitude setpoint.
            let target_ned_z = -setpoint.altitude?.0;
            let error_ned_z = target_ned_z - state.position_ned[2].0;
            crate::control::position::sqrt_shape(
                error_ned_z,
                self.pos_ctrl.gains[2],
                self.pos_ctrl.accel_limits[2],
                self.pos_ctrl.vel_caps[2],
            )
        };
        Some(MetersPerSecond(
            vertical.clamp(-limits.max_climb_rate.0, limits.max_descent_rate.0),
        ))
    }
}

/// Replace the yaw of an attitude setpoint with a commanded heading,
/// preserving its roll and pitch. Altitude mode keeps horizontal
/// attitude manual (roll/pitch) but slaves yaw to the heading setpoint.
fn attitude_with_heading(att_sp: &Quaternion, heading: Radians) -> Quaternion {
    let (roll, pitch, _yaw) = att_sp.to_euler();
    let qz = Quaternion::from_axis_angle(Vector3::new(0.0, 0.0, 1.0), heading.0);
    let qy = Quaternion::from_axis_angle(Vector3::new(0.0, 1.0, 0.0), pitch);
    let qx = Quaternion::from_axis_angle(Vector3::new(1.0, 0.0, 0.0), roll);
    qz.mul(&qy).mul(&qx).normalize()
}

impl VehicleController for MultirotorController {
    type RuntimeState = MultirotorRuntimeState;

    // Registered in cert/algorithm_id_registry.toml as
    // "controller.multirotor.v4". This identity includes the kernel
    // cycle interval that activates the controller dynamic terms.
    // It also includes explicit
    // production mode refusal, topology-edge derivative hygiene,
    // and automatic fallback output continuity.
    const ALGORITHM_ID: u64 = 0x4354_4C4D_5552_5634; // "CTLMURV4"

    fn verify_config_binding(
        &self,
        cfg: &crate::kernel::config::ResolvedKernelConfig,
    ) -> Result<(), crate::control::ControllerConfigMismatch> {
        let config_identity = cfg.controller_tuning_identity();
        if self.tuning_identity != config_identity {
            return Err(crate::control::ControllerConfigMismatch {
                controller_identity: self.tuning_identity,
                config_identity,
            });
        }
        Ok(())
    }

    fn supports_mode(&self, mode: crate::control::ControlMode) -> bool {
        crate::control::multirotor_mode_capability(mode)
            != crate::control::MultirotorModeCapability::Unsupported
    }

    fn step(
        &self,
        runtime: &mut MultirotorRuntimeState,
        state: &StateEstimate,
        command: &Command,
        flags: &VehicleControlMode,
        _mode: ConfigMode,
        limits: &Limits,
    ) -> AxisCommand {
        self.run_step(runtime, state, command, flags, limits)
            .axis_command
    }

    fn step_with_observation(
        &self,
        runtime: &mut MultirotorRuntimeState,
        state: &StateEstimate,
        command: &Command,
        flags: &VehicleControlMode,
        _mode: ConfigMode,
        limits: &Limits,
    ) -> crate::control::ControllerStep {
        self.run_step(runtime, state, command, flags, limits)
    }
}

mod step;

#[cfg(test)]
mod replicable_tests;
