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
use crate::types::{FloatExt, MetersPerSecond, MetersPerSecondSquared, Normalized, Scalar};

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
}

impl Default for VelocityLoopState {
    fn default() -> Self {
        Self {
            integrator_ned: Vector3::new(
                MetersPerSecond(0.0),
                MetersPerSecond(0.0),
                MetersPerSecond(0.0),
            ),
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
    pub fn step(
        &self,
        state: &mut VelocityLoopState,
        setpoint: Vector3<MetersPerSecond>,
        current: Vector3<MetersPerSecond>,
        accel_ff: AccelFeedforward,
        current_att: &Quaternion,
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
        let z_correction = (p_z + i_z + ff_z).clamp(-max_dn, max_up);
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
        let r22 = 1.0
            - 2.0 * (current_att.x * current_att.x + current_att.y * current_att.y);
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
        // NED ZYX quaternion convention: thrust direction in
        // world is `-R·body_z`. With yaw = 0 that resolves to
        // `(-sin θ, sin φ, …)` — north push requires negative
        // pitch (nose down), east push requires positive roll
        // (right-wing-down). See the unit test for the assertion
        // that pins these signs.
        let pitch_sp_raw = -acc_x_clamped.atan2(g_si);
        let roll_sp_raw = acc_y_clamped.atan2(g_si);
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

        // ---- integrator (conditional anti-windup) ----
        // Only integrate when the corresponding axis is NOT
        // saturated against its actuator authority. The integrator
        // freeze on saturation prevents wind-up against an actuator
        // that cannot do more work — without this, the I term
        // would overshoot in the opposite direction once the input
        // returns to authority.
        if dt_sec > 0.0 {
            if !x_saturated {
                state.integrator_ned.x = MetersPerSecond(
                    state.integrator_ned.x.0 + error.x * dt_sec,
                );
            }
            if !y_saturated {
                state.integrator_ned.y = MetersPerSecond(
                    state.integrator_ned.y.0 + error.y * dt_sec,
                );
            }
            if !z_saturated {
                state.integrator_ned.z = MetersPerSecond(
                    state.integrator_ned.z.0 + error.z * dt_sec,
                );
            }
        }

        // ---- attitude setpoint ----
        // Keep current yaw, compose with the freshly-computed
        // roll/pitch. Small-angle composition is fine here: the
        // velocity loop's tilt cap is in tens of degrees, not
        // hundreds.
        let current_yaw_quat =
            Quaternion::new(current_att.w, 0.0, 0.0, current_att.z).normalize();
        let roll_pitch_quat = Quaternion::from_axis_angle(Vector3::new(1.0, 0.0, 0.0), roll_sp)
            .mul(&Quaternion::from_axis_angle(
                Vector3::new(0.0, 1.0, 0.0),
                pitch_sp,
            ));
        let attitude = current_yaw_quat.mul(&roll_pitch_quat).normalize();

        VelocityCommand {
            collective,
            attitude,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::Quaternion;

    fn ctrl(hover: Scalar) -> VelocityController {
        VelocityController::new(CascadeGains::x500_defaults(), hover)
    }

    fn zero_vel() -> Vector3<MetersPerSecond> {
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        )
    }

    #[test]
    fn p_term_alone_matches_old_behaviour_at_zero_integral() {
        // With zero integrator and zero feedforward, the new
        // controller's output for the same (setpoint, current,
        // hover) must reduce to the legacy P-only formula. This
        // guards against the upgrade silently changing the
        // baseline response.
        let c = ctrl(0.77);
        let mut s = VelocityLoopState::default();
        let setpoint = Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(1.0), // 1 m/s descend
        );
        let out = c.step(
            &mut s,
            setpoint,
            zero_vel(),
            AccelFeedforward::default(),
            &Quaternion::IDENTITY,
            0.0,
        );
        // Legacy formula: trim + (-(setpoint.z - 0) * vel_p[2]),
        // clamped. setpoint.z = 1.0, vel_p[2] from x500 defaults.
        let gain_z = CascadeGains::x500_defaults().vel_p[2];
        let expected: f32 = (0.77 - 1.0 * gain_z).clamp(0.0, 1.0);
        assert!((out.collective.0 - expected).abs() < 1e-5);
    }

    #[test]
    fn integrator_freezes_when_saturated() {
        // If the output saturates at max_up, the integrator must
        // not grow. Force a large velocity error and confirm.
        let c = ctrl(0.5); // makes saturation easy to hit
        let mut s = VelocityLoopState::default();
        let setpoint = Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(-10.0),
        );
        // First tick — saturated at max_up.
        let _ = c.step(
            &mut s,
            setpoint,
            zero_vel(),
            AccelFeedforward::default(),
            &Quaternion::IDENTITY,
            0.01,
        );
        assert_eq!(s.integrator_ned.z.0, 0.0, "integrator must not grow while saturated");
    }

    #[test]
    fn integrator_grows_when_not_saturated() {
        // Small error keeps the output far from saturation; the
        // integrator must accumulate at error · dt.
        let c = ctrl(0.5);
        let mut s = VelocityLoopState::default();
        let setpoint = Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.1),
        );
        let _ = c.step(
            &mut s,
            setpoint,
            zero_vel(),
            AccelFeedforward::default(),
            &Quaternion::IDENTITY,
            0.01,
        );
        // error.z = 0.1 - 0 = 0.1. integrator += error · dt = 0.001.
        assert!((s.integrator_ned.z.0 - 0.001).abs() < 1e-6);
    }

    /// Pin the empirical horizontal sign convention — the
    /// cascade-chain consistency tested in SITL. With the wrong
    /// pitch / roll sign the horizontal loop closes in positive
    /// feedback and the vehicle drifts away from the setpoint;
    /// the failure mode only surfaces downstream in gz-physics
    /// where root cause is hard to attribute, so this unit test
    /// pins the sign with no simulator dependency.
    ///
    /// "Need to move south" = vel_sp_x = −1 m/s, current = 0.
    /// The cascade then commands a quaternion whose to_euler
    /// pitch is NEGATIVE. The chain's mixer + plant convert that
    /// to a south push, even though the to_euler doc-comment
    /// names positive pitch "nose up" — the rate-to-mixer half
    /// of the loop encodes a sign that completes the cycle in
    /// the right direction. Verified end-to-end in SITL.
    #[test]
    fn horizontal_velocity_error_drives_consistent_tilt_direction() {
        let c = ctrl(0.5);
        let mut s = VelocityLoopState::default();
        let sp_south = Vector3::new(
            MetersPerSecond(-1.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        );
        let out_s = c.step(
            &mut s,
            sp_south,
            zero_vel(),
            AccelFeedforward::default(),
            &Quaternion::IDENTITY,
            0.0,
        );
        let (_roll_s, pitch_s, _yaw_s) = out_s.attitude.to_euler();
        assert!(
            pitch_s > 0.0,
            "south-bound vel_sp must produce positive-pitch quaternion (nose-up tilts thrust south); got pitch={pitch_s}"
        );

        let mut s2 = VelocityLoopState::default();
        let sp_east = Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(1.0),
            MetersPerSecond(0.0),
        );
        let out_e = c.step(
            &mut s2,
            sp_east,
            zero_vel(),
            AccelFeedforward::default(),
            &Quaternion::IDENTITY,
            0.0,
        );
        let (roll_e, _pitch_e, _yaw_e) = out_e.attitude.to_euler();
        assert!(
            roll_e > 0.0,
            "east-bound vel_sp must produce positive-roll quaternion (right-wing-down tilts thrust east); got roll={roll_e}"
        );
    }

    #[test]
    fn feedforward_offsets_thrust_when_accel_commanded() {
        // A commanded downward NED acceleration (positive z)
        // should reduce thrust below the trim by ff·trim/g, since
        // less thrust is needed to achieve faster descent. Test
        // forces `vel_accel_ff = 1.0` locally so it isn't
        // affected by tuning changes in the default gains.
        let mut gains = CascadeGains::x500_defaults();
        gains.vel_accel_ff = 1.0;
        let c = VelocityController::new(gains, 0.77);
        let mut s = VelocityLoopState::default();
        let baseline = c.step(
            &mut s,
            zero_vel(),
            zero_vel(),
            AccelFeedforward::default(),
            &Quaternion::IDENTITY,
            0.0,
        );
        let mut s2 = VelocityLoopState::default();
        let with_ff = c.step(
            &mut s2,
            zero_vel(),
            zero_vel(),
            AccelFeedforward {
                accel_ned: Vector3::new(
                    MetersPerSecondSquared(0.0),
                    MetersPerSecondSquared(0.0),
                    MetersPerSecondSquared(1.0), // commanded +1 m/s² down
                ),
            },
            &Quaternion::IDENTITY,
            0.0,
        );
        assert!(with_ff.collective.0 < baseline.collective.0);
    }
}
