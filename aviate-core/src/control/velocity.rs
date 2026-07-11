//! Velocity loop — converts a velocity setpoint into collective
//! thrust + tilt (attitude setpoint). Cascade middle layer between
//! the position controller (kinematic shaper) above and the
//! attitude/rate loops below.
//!
//! P + I + acceleration feedforward (DRQ-CTL-002). Integrator
//! eliminates the steady-state error a pure-P loop has when the
//! hover trim is mis-estimated or there's a constant wind/payload
//! bias. Feedforward bypasses the P-loop's phase lag when the
//! position controller's vel_sp is changing — the velocity loop
//! is not forced to "discover" the upcoming change via the error
//! signal alone.
//!
//! Conditional anti-windup: the integrator does NOT accumulate
//! when the output is saturated against actuator authority. Plain
//! windup would integrate against a saturated actuator until the
//! input cleared, then overshoot in the opposite direction — the
//! reset-windup pathology that makes I-term a worse fix than no
//! fix for transient overshoot.

use crate::control::cascade_gains::CascadeGains;
use crate::math::{Quaternion, Vector3};
#[allow(unused_imports)] // FloatExt needed for no_std math methods
use crate::types::{
    FloatExt, MetersPerSecond, MetersPerSecondSquared, Normalized, Radians, Scalar,
};

/// Per-step clamp on the applied commanded-vs-current yaw error
/// [rad]: a large heading setpoint change slews the vehicle through
/// intermediate attitude setpoints instead of stepping the attitude
/// loop. Stateless — the clamp re-evaluates against measured yaw each
/// cycle, so the vehicle converges on the commanded heading at the
/// attitude loop's own pace.
const MAX_YAW_ERROR_STEP: Scalar = 0.6;

/// Persistent state owned by the velocity loop. Lives inside
/// `MultirotorRuntimeState`, not on the controller struct — the
/// controller carries only tuning (`CascadeGains`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct VelocityLoopState {
    /// NED-frame integrator [m·s of accumulated velocity error].
    /// Multiplied by `vel_i[axis]` to produce the I term's
    /// contribution to the corresponding actuator (thrust for Z,
    /// tilt angle for X/Y).
    pub integrator_ned: Vector3<MetersPerSecond>,
    /// Previous filtered NED velocity sample, used to compute
    /// the derivative-on-measurement contribution to the
    /// vertical velocity loop. Filtered to keep the D term from
    /// amplifying the finite-difference noise inherent in
    /// position-derived velocity.
    pub last_vel_filt_ned: Vector3<MetersPerSecond>,
    /// First-cycle marker. While unset the D term outputs zero
    /// instead of differentiating against the default sample
    /// (the same derivative-kick guard the rate loop uses).
    pub d_primed: bool,
}

impl Default for VelocityLoopState {
    fn default() -> Self {
        Self {
            integrator_ned: Vector3::new(
                MetersPerSecond(0.0),
                MetersPerSecond(0.0),
                MetersPerSecond(0.0),
            ),
            last_vel_filt_ned: Vector3::new(
                MetersPerSecond(0.0),
                MetersPerSecond(0.0),
                MetersPerSecond(0.0),
            ),
            d_primed: false,
        }
    }
}

impl VelocityLoopState {
    pub fn reset(&mut self) {
        self.integrator_ned = Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        );
        self.last_vel_filt_ned = Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        );
        self.d_primed = false;
    }
}

/// Output of one velocity-loop cycle.
#[derive(Clone, Copy, Debug)]
pub struct VelocityCommand {
    /// Collective thrust setpoint for the rate-loop / mixer chain.
    pub collective: Normalized,
    /// Attitude setpoint (yaw-from-current-attitude, roll/pitch
    /// derived from horizontal acceleration command).
    pub attitude: Quaternion,
}

/// Acceleration feedforward signal supplied by the position loop.
/// Components are NED-frame inertial accelerations [m/s²]. When
/// zero (`Default`), the loop runs pure feedback.
#[derive(Clone, Copy, Debug)]
pub struct AccelFeedforward {
    pub accel_ned: Vector3<MetersPerSecondSquared>,
}

impl Default for AccelFeedforward {
    fn default() -> Self {
        Self {
            accel_ned: Vector3::new(
                MetersPerSecondSquared(0.0),
                MetersPerSecondSquared(0.0),
                MetersPerSecondSquared(0.0),
            ),
        }
    }
}

#[derive(Clone, Debug)]
pub struct VelocityController {
    /// Authoritative tuning; copied from `ResolvedKernelConfig` at
    /// kernel construction. Per LLR-CTL-101 the controller carries
    /// gains by value, not as a config reference — the config
    /// itself is immutable for the flight period, so a copy is
    /// strictly equivalent and avoids a borrow-life lifetime
    /// chasing through the trait.
    pub gains: CascadeGains,
    /// Hover thrust trim [Normalized] — vertical loop commands
    /// corrections around this value. Mirrors
    /// `ResolvedKernelConfig.hover_thrust_norm`.
    pub hover_thrust_norm: Scalar,
}

impl VelocityController {
    pub fn new(gains: CascadeGains, hover_thrust_norm: Scalar) -> Self {
        Self {
            gains,
            hover_thrust_norm,
        }
    }

    /// One control cycle.
    ///
    /// `setpoint` / `current` are NED-frame velocities. `accel_ff`
    /// is the position-loop's commanded acceleration; multiply by
    /// `gains.vel_accel_ff` to scale (e.g. set to zero to run pure
    /// feedback for testing). `current_att` extracts the vehicle's
    /// current yaw to keep the attitude setpoint aligned with
    /// heading; roll/pitch are derived from horizontal acceleration.
    /// `dt_sec` advances the integrator (zero or negative → skip).
    #[allow(clippy::too_many_arguments)] // cascade inputs are individually meaningful (kernel_update.rs precedent)
    pub fn step(
        &self,
        state: &mut VelocityLoopState,
        setpoint: Vector3<MetersPerSecond>,
        current: Vector3<MetersPerSecond>,
        accel_ff: AccelFeedforward,
        current_att: &Quaternion,
        heading_sp: Option<Radians>,
        dt_sec: Scalar,
    ) -> VelocityCommand {
        let error = Vector3 {
            x: setpoint.x.0 - current.x.0,
            y: setpoint.y.0 - current.y.0,
            z: setpoint.z.0 - current.z.0,
        };

        // ---- vertical (Z) ----
        // The vertical axis runs as a P+I controller around the
        // hover trim, with an acceleration feedforward term that
        // bypasses the P-loop's phase lag when vel_sp is changing.
        let trim = self.hover_thrust_norm;
        let max_up = 1.0 - trim;
        let max_dn = trim;
        // Filter the vertical velocity measurement (single-pole
        // LPF) and take its discrete derivative for the D term.
        // Derivative-on-measurement, not on error: a step in
        // vel_sp doesn't produce a derivative kick. Skipped on
        // the first cycle. Sign convention: `p_z` carries a
        // leading `-`, so for the D term to DAMP (positive d
        // measurement → more brake collective), the gain
        // multiplier here is `+gain · d_meas`, not the rate
        // loop's `-gain · d_meas` — the rate loop's `p_term`
        // doesn't carry the same negation.
        let alpha = self.gains.rate_d_lpf_alpha;
        let raw_z = current.z.0;
        let filt_z = alpha * state.last_vel_filt_ned.z.0 + (1.0 - alpha) * raw_z;
        let d_z = if state.d_primed && dt_sec > 0.0 && self.gains.vel_d[2] > 0.0 {
            let d_meas = (filt_z - state.last_vel_filt_ned.z.0) / dt_sec;
            d_meas * self.gains.vel_d[2]
        } else {
            0.0
        };

        let p_z = -error.z * self.gains.vel_p[2];
        let i_z = -state.integrator_ned.z.0 * self.gains.vel_i[2];
        // Convert NED accel ff to thrust offset. Newton's second
        // law: thrust = m·(g − a_ned_z); divided by max-thrust to
        // get the Normalized command. Approximated as
        // `−a_ned_z·trim/g` because at hover trim is `m·g/max_thrust`
        // — the linearization around hover trim that the rest of
        // the velocity loop already assumes.
        let g_si: Scalar = 9.81;
        let ff_z = -accel_ff.accel_ned.z.0 * (trim / g_si) * self.gains.vel_accel_ff;
        let z_correction = (p_z + i_z + d_z + ff_z).clamp(-max_dn, max_up);
        let z_saturated = z_correction == -max_dn || z_correction == max_up;
        let collective_unscaled = trim + z_correction;
        // Tilt compensation: the body's −z axis (thrust
        // direction) has a vertical (world −z) component of
        // `cos(tilt)`. When the vehicle is tilted the cascade
        // needs more collective to keep the SAME vertical
        // force — otherwise a horizontal correction (tilting)
        // simultaneously starves the vertical loop, and the
        // vehicle descends faster than commanded. Compensation
        // factor is `1 / cos(tilt)`, where `cos(tilt) = R[2,2]`
        // (the third column of the body→world rotation matrix
        // dotted with world +z). Floored at 0.5 to avoid
        // unbounded amplification when the vehicle is past 60°
        // tilt and recovering — the mixer's per-motor clamp will
        // handle the residual.
        let r22 = 1.0 - 2.0 * (current_att.x * current_att.x + current_att.y * current_att.y);
        let cos_tilt = r22.max(0.5);
        let collective_cmd = collective_unscaled / cos_tilt;
        let collective = Normalized(collective_cmd.clamp(0.0, 1.0));

        // ---- horizontal (X/Y) ----
        // Convert velocity error → horizontal acceleration → tilt.
        // P + I; feedforward in NED-accel sums with the feedback.
        // The tilt cap is the velocity loop's authority limit, set
        // by `vel_max_roll_pitch`.
        let acc_x_cmd = error.x * self.gains.vel_p[0]
            + state.integrator_ned.x.0 * self.gains.vel_i[0]
            + accel_ff.accel_ned.x.0 * self.gains.vel_accel_ff;
        let acc_y_cmd = error.y * self.gains.vel_p[1]
            + state.integrator_ned.y.0 * self.gains.vel_i[1]
            + accel_ff.accel_ned.y.0 * self.gains.vel_accel_ff;
        let acc_x_clamped = acc_x_cmd.clamp(-g_si, g_si);
        let acc_y_clamped = acc_y_cmd.clamp(-g_si, g_si);
        // The acceleration command is in NED, but roll/pitch are
        // body-frame tilts composed AFTER the yaw quaternion — so
        // the command must first rotate by −yaw into the vehicle's
        // heading frame. Skipping that rotation is only correct at
        // yaw = 0: at 90° a north push tilts the vehicle east, and
        // past 90° the horizontal velocity feedback turns positive
        // and holds spiral away (the #110 divergence).
        let cur_yaw = (2.0 * (current_att.w * current_att.z + current_att.x * current_att.y))
            .atan2(1.0 - 2.0 * (current_att.y * current_att.y + current_att.z * current_att.z));
        let (sin_yaw, cos_yaw) = (cur_yaw.sin(), cur_yaw.cos());
        let acc_fwd = cos_yaw * acc_x_clamped + sin_yaw * acc_y_clamped;
        let acc_right = -sin_yaw * acc_x_clamped + cos_yaw * acc_y_clamped;
        // Thrust direction in world is `-R·body_z`: a forward push
        // requires negative pitch (nose down), a rightward push
        // positive roll (right-wing-down). The unit tests pin these
        // signs at yaw 0 and at ±90°.
        let pitch_sp_raw = -acc_fwd.atan2(g_si);
        let roll_sp_raw = acc_right.atan2(g_si);
        let pitch_sp = pitch_sp_raw.clamp(
            -self.gains.vel_max_roll_pitch,
            self.gains.vel_max_roll_pitch,
        );
        let roll_sp = roll_sp_raw.clamp(
            -self.gains.vel_max_roll_pitch,
            self.gains.vel_max_roll_pitch,
        );
        let x_saturated = pitch_sp != pitch_sp_raw;
        let y_saturated = roll_sp != roll_sp_raw;

        // Persist filtered velocity sample for the next cycle's
        // D-term. Updated unconditionally — the d_primed gate
        // is on the OUTPUT, so the first sample becomes the
        // baseline for the second cycle's derivative.
        state.last_vel_filt_ned = Vector3::new(
            MetersPerSecond(state.last_vel_filt_ned.x.0),
            MetersPerSecond(state.last_vel_filt_ned.y.0),
            MetersPerSecond(filt_z),
        );
        state.d_primed = true;

        // ---- integrator (conditional anti-windup) ----
        // Only integrate when the corresponding axis is NOT
        // saturated against its actuator authority. The integrator
        // freeze on saturation prevents wind-up against an actuator
        // that cannot do more work — without this, the I term
        // would overshoot in the opposite direction once the input
        // returns to authority.
        if dt_sec > 0.0 {
            // The saturation flags live on the body-frame tilt axes
            // while the integrators are NED; a saturated tilt axis is
            // a mix of both NED axes, so freeze both rather than
            // guessing an attribution.
            if !x_saturated && !y_saturated {
                state.integrator_ned.x =
                    MetersPerSecond(state.integrator_ned.x.0 + error.x * dt_sec);
                state.integrator_ned.y =
                    MetersPerSecond(state.integrator_ned.y.0 + error.y * dt_sec);
            }
            if !z_saturated {
                state.integrator_ned.z =
                    MetersPerSecond(state.integrator_ned.z.0 + error.z * dt_sec);
            }
        }

        // ---- attitude setpoint ----
        // Yaw: hold current unless the command carries a heading
        // setpoint (DRQ: guided modes must honor commanded heading).
        // The applied yaw target is the current yaw plus the wrapped
        // error clamped to MAX_YAW_ERROR_STEP, composed with the
        // freshly-computed roll/pitch. Small-angle composition is fine
        // here: the velocity loop's tilt cap is in tens of degrees,
        // not hundreds.
        let yaw_quat = if let Some(heading) = heading_sp {
            let mut err = heading.0 - cur_yaw;
            const PI: Scalar = core::f32::consts::PI;
            while err > PI {
                err -= 2.0 * PI;
            }
            while err < -PI {
                err += 2.0 * PI;
            }
            let applied = cur_yaw + err.clamp(-MAX_YAW_ERROR_STEP, MAX_YAW_ERROR_STEP);
            Quaternion::from_axis_angle(Vector3::new(0.0, 0.0, 1.0), applied)
        } else {
            Quaternion::new(current_att.w, 0.0, 0.0, current_att.z).normalize()
        };
        let roll_pitch_quat =
            Quaternion::from_axis_angle(Vector3::new(1.0, 0.0, 0.0), roll_sp).mul(
                &Quaternion::from_axis_angle(Vector3::new(0.0, 1.0, 0.0), pitch_sp),
            );
        let attitude = yaw_quat.mul(&roll_pitch_quat).normalize();

        VelocityCommand {
            collective,
            attitude,
        }
    }
}

#[cfg(test)]
#[path = "velocity_tests.rs"]
mod tests;
