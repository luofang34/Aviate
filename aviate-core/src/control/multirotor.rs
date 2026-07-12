//! Multirotor cascade controller: position → velocity → attitude
//! → rate, driving the mixer's `AxisCommand`. Tuning lives in
//! `ResolvedKernelConfig.cascade_gains` (DRQ-CTL-001); persistent
//! state (integrators, derivative memories) lives in
//! `KernelState.controller` as a `MultirotorRuntimeState`.

use crate::control::attitude::AttitudeController;
use crate::control::cascade_gains::CascadeGains;
use crate::control::position::PositionController;
use crate::control::rate::{RateController, RateLoopState};
use crate::control::velocity::{AccelFeedforward, VelocityController, VelocityLoopState};
use crate::control::{
    AxisCommand, Command, ConfigMode, Limits, OuterLoopSelection, Scalar, Setpoint,
    VehicleControlMode, VehicleController,
};
use crate::math::{Quaternion, Vector3};
use crate::state::StateEstimate;
use crate::types::{MetersPerSecond, MetersPerSecondSquared, Normalized, Radians};

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
    /// and rate-loop derivative. The kernel `update` path writes
    /// it before each `step()` so the controller doesn't need a
    /// separate trait-signature change for `dt`.
    pub dt_sec: Scalar,
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
        }
    }
}

impl crate::control::runtime::ControllerRuntimeState for MultirotorRuntimeState {
    fn reset(&mut self) {
        self.velocity_loop.reset();
        self.rate_loop.reset();
        self.last_vel_sp_ned = Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        );
        self.vel_sp_primed = false;
        // dt_sec is overwritten each cycle; resetting to zero
        // makes the next step a quiet "no time has passed" cycle.
        self.dt_sec = 0.0;
    }
}

impl crate::replicable::Replicable for MultirotorRuntimeState {
    // 16 f32 lanes x 4 bytes = 64 bytes. EVERY persistent field of
    // the runtime state must appear here: an omitted field lets two
    // lockstep channels diverge in hidden state while comparing
    // byte-equal, surfacing one cycle later as differing actuator
    // outputs with no witness (#141 — last_vel_filt_ned and d_primed
    // were missing). The per-field mutation test below is the
    // guardrail: adding a field without encoding it fails the test.
    const ENCODED_LEN: usize = 64;

    fn encode_canonical(&self, buf: &mut [u8]) -> usize {
        let fields: [f32; 16] = [
            self.velocity_loop.integrator_ned.x.0,
            self.velocity_loop.integrator_ned.y.0,
            self.velocity_loop.integrator_ned.z.0,
            self.velocity_loop.last_vel_filt_ned.x.0,
            self.velocity_loop.last_vel_filt_ned.y.0,
            self.velocity_loop.last_vel_filt_ned.z.0,
            self.rate_loop.meas_filtered_prev.x.0,
            self.rate_loop.meas_filtered_prev.y.0,
            self.rate_loop.meas_filtered_prev.z.0,
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
        for (i, &v) in fields.iter().enumerate() {
            let bytes = v.to_le_bytes();
            buf[i * 4..i * 4 + 4].copy_from_slice(&bytes);
        }
        fields.len() * 4
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
            att_ctrl: AttitudeController::new(gains.att_p),
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

    /// Collective for the vertical-only path. Runs the velocity loop
    /// with a zeroed horizontal setpoint against zeroed horizontal
    /// error, so it derives no tilt, and keeps only the collective
    /// output. Horizontal attitude stays with the manual setpoint.
    fn vertical_collective(
        &self,
        vel_state: &mut VelocityLoopState,
        state: &StateEstimate,
        vertical_sp: MetersPerSecond,
        dt_sec: Scalar,
    ) -> Normalized {
        let zero = MetersPerSecond(0.0);
        let vel_sp = Vector3::new(zero, zero, vertical_sp);
        let current = Vector3::new(zero, zero, state.velocity_ned[2]);
        self.vel_ctrl
            .step(
                vel_state,
                vel_sp,
                current,
                AccelFeedforward::default(),
                &state.attitude,
                // AltitudeHold is a manual-yaw behavior: hold current.
                None,
                dt_sec,
            )
            .collective
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
    // "controller.multirotor.v2" — v2 rotates the velocity loop's
    // NED acceleration command into the heading frame before
    // deriving roll/pitch; v1 skipped the rotation, which is only
    // correct at yaw 0 and turns the horizontal loop anti-corrective
    // past 90° of heading (#110). Same cascade structure, different
    // control arithmetic, so a v1 image must not match a v2 one at
    // the lockstep gate.
    const ALGORITHM_ID: u64 = 0x4354_4C4D_5552_5632; // "CTLMURV2"

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

    fn step(
        &self,
        runtime: &mut MultirotorRuntimeState,
        state: &StateEstimate,
        command: &Command,
        flags: &VehicleControlMode,
        _mode: ConfigMode,
        limits: &Limits,
    ) -> AxisCommand {
        let dt_sec = runtime.dt_sec;
        let mut collective_sp = command.setpoint.collective_thrust;
        let mut att_sp = command.setpoint.attitude.unwrap_or(Quaternion::IDENTITY);
        let mut accel_ff_ned = Vector3::new(
            MetersPerSecondSquared(0.0),
            MetersPerSecondSquared(0.0),
            MetersPerSecondSquared(0.0),
        );

        // ---- position → velocity setpoint ----
        // Loop selection is driven by the control-mode flags, not by
        // which setpoint fields are populated: `outer_loop` is the
        // single authority that maps mode → active loop and rejects
        // setpoints illegal for the active mode.
        let mut vel_sp_active = false;
        let mut vel_sp_ned = Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        );
        match flags.outer_loop(&command.setpoint) {
            OuterLoopSelection::Position(pos_sp_arr) => {
                let pos_sp = Vector3::new(pos_sp_arr[0], pos_sp_arr[1], pos_sp_arr[2]);
                vel_sp_ned = self.pos_ctrl.step(
                    pos_sp,
                    Vector3 {
                        x: state.position_ned[0],
                        y: state.position_ned[1],
                        z: state.position_ned[2],
                    },
                );
                vel_sp_active = true;
            }
            OuterLoopSelection::Velocity(vel_sp_arr) => {
                vel_sp_ned = Vector3::new(vel_sp_arr[0], vel_sp_arr[1], vel_sp_arr[2]);
                vel_sp_active = true;
            }
            OuterLoopSelection::None => {}
        }

        if vel_sp_active {
            // Feedforward = finite difference of vel_sp. Skip on
            // first cycle (no previous sample to difference).
            if runtime.vel_sp_primed && dt_sec > 0.0 {
                accel_ff_ned = Vector3::new(
                    MetersPerSecondSquared((vel_sp_ned.x.0 - runtime.last_vel_sp_ned.x.0) / dt_sec),
                    MetersPerSecondSquared((vel_sp_ned.y.0 - runtime.last_vel_sp_ned.y.0) / dt_sec),
                    MetersPerSecondSquared((vel_sp_ned.z.0 - runtime.last_vel_sp_ned.z.0) / dt_sec),
                );
            }
            runtime.last_vel_sp_ned = vel_sp_ned;
            runtime.vel_sp_primed = true;

            let cur_vel = Vector3::new(
                state.velocity_ned[0],
                state.velocity_ned[1],
                state.velocity_ned[2],
            );
            let vel_out = self.vel_ctrl.step(
                &mut runtime.velocity_loop,
                vel_sp_ned,
                cur_vel,
                AccelFeedforward {
                    accel_ned: accel_ff_ned,
                },
                &state.attitude,
                command.setpoint.heading,
                dt_sec,
            );
            collective_sp = vel_out.collective;
            att_sp = vel_out.attitude;
        } else if flags.flag_control_altitude_enabled {
            // Altitude / climb-rate hold: drive only the vertical branch
            // of the velocity loop around hover trim; roll/pitch stay
            // with the manual attitude setpoint and yaw slaves to the
            // heading setpoint. Horizontal is not closed here, so the
            // accel-feedforward prime is cleared.
            if let Some(vertical_sp) =
                self.vertical_velocity_setpoint(&command.setpoint, state, limits)
            {
                collective_sp = self.vertical_collective(
                    &mut runtime.velocity_loop,
                    state,
                    vertical_sp,
                    dt_sec,
                );
            }
            if let Some(heading) = command.setpoint.heading {
                att_sp = attitude_with_heading(&att_sp, heading);
            }
            runtime.vel_sp_primed = false;
        } else {
            // Open-loop / manual: nothing to derive feedforward
            // from; ensure the next closed-loop entry doesn't
            // see a stale prev_vel_sp.
            runtime.vel_sp_primed = false;
        }

        // ---- thrust gate ----
        // Disarmed / "no commanded thrust" state: when the
        // commanded collective is essentially zero we treat
        // the airframe as on the ground and silence axis
        // control. Mid-flight the cascade keeps running even
        // at low collective; the mixer's priority desaturation
        // resolves any roll/pitch demand the collective can't
        // physically support. This gate is also what keeps the
        // desaturation's collective boost from spinning motors
        // on the ground — with the axes silenced the boost is a
        // no-op.
        const DISARMED_THRESHOLD: Scalar = 0.02;
        if collective_sp.0 < DISARMED_THRESHOLD {
            runtime.velocity_loop.reset();
            runtime.rate_loop.reset();
            runtime.vel_sp_primed = false;
            return AxisCommand {
                roll: crate::types::NormalizedSigned(0.0),
                pitch: crate::types::NormalizedSigned(0.0),
                yaw: crate::types::NormalizedSigned(0.0),
                collective: collective_sp,
            };
        }

        // ---- attitude → rate ----
        let rate_sp = self.att_ctrl.step(&att_sp, &state.attitude);

        // ---- rate → torque ----
        let cur_rate = [
            state.angular_velocity[0],
            state.angular_velocity[1],
            state.angular_velocity[2],
        ];
        let torque_norm = self
            .rate_ctrl
            .step(&mut runtime.rate_loop, rate_sp, cur_rate, dt_sec);

        // Pass through the rate-loop's normalized torque
        // commands. The mixer clamps individual motor outputs to
        // `[0, 1]`; the cascade is free to ask for full
        // authority and let the mixer surface the saturation.
        // Previously a collective-aware cap was applied here to
        // protect against motors pinning to zero, but that
        // hard-clipped attitude authority during steep brake
        // (high collective) and steep descent (low collective)
        // alike — the cascade couldn't deliver the torque the
        // LLR-CTL-202 step response requires.
        AxisCommand {
            roll: torque_norm[0],
            pitch: torque_norm[1],
            yaw: torque_norm[2],
            collective: collective_sp,
        }
    }
}
