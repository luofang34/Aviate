//! Rate loop — innermost cascade layer. Converts angular-rate
//! error into normalized torque commands for the mixer.
//!
//! P + I + D, with derivative-on-measurement (not derivative-on-
//! error). The I term exists for standing torque disturbances: a
//! P-only rate loop answers a constant disturbance with a constant
//! rate error, which the attitude loop above can only convert into a
//! standing attitude error — on the yaw axis that presents as an
//! uncommanded heading walk no P gain can remove within actuator
//! authority. The integrator accumulates conditionally (never while
//! the output is saturated) and is bounded, so it trims disturbances
//! without becoming a windup hazard. The setpoint can step instantaneously when the
//! attitude loop commands a maneuver; differentiating that step
//! produces a "derivative kick" that bangs the actuators. Taking
//! the derivative of the gyro measurement instead gives the same
//! steady-state damping without the kick.
//!
//! The derivative is single-pole LPF-filtered against gyro noise
//! (`gains.rate_d_lpf_alpha`). Without filtering, the D term is
//! essentially a high-pass amplifier on whatever noise the EKF's
//! `last_gyro_body` carries forward — which is a lot, especially
//! when the synth IMU is noise-free and the EKF integrates that
//! pristine signal into a slightly-quantized state.

use crate::control::cascade_gains::CascadeGains;
use crate::control::{ControllerLoopObservation, IntegratorAction};
use crate::math::Vector3;
use crate::types::{NormalizedSigned, RadiansPerSecond, Scalar};

/// Persistent state owned by the rate loop. Lives inside
/// `MultirotorRuntimeState`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RateLoopState {
    /// Previous filtered rate measurement (rad/s body frame), one
    /// per axis. Used to compute Δ(meas)/Δt for the D term.
    pub meas_filtered_prev: Vector3<RadiansPerSecond>,
    /// Accumulated integral torque per axis (normalized torque
    /// units), bounded by `RATE_I_LIMIT`.
    pub integral: [Scalar; 3],
    /// First-cycle marker. Until set, the D term outputs zero
    /// instead of differentiating against the default (zero)
    /// previous value — that would produce a large bogus
    /// derivative kick on the first cycle.
    pub primed: bool,
}

impl Default for RateLoopState {
    fn default() -> Self {
        Self {
            meas_filtered_prev: Vector3::new(
                RadiansPerSecond(0.0),
                RadiansPerSecond(0.0),
                RadiansPerSecond(0.0),
            ),
            integral: [0.0; 3],
            primed: false,
        }
    }
}

impl RateLoopState {
    pub fn reset(&mut self) {
        self.meas_filtered_prev = Vector3::new(
            RadiansPerSecond(0.0),
            RadiansPerSecond(0.0),
            RadiansPerSecond(0.0),
        );
        self.integral = [0.0; 3];
        self.primed = false;
    }
}

#[derive(Clone, Debug)]
pub struct RateController {
    pub gains: CascadeGains,
}

/// Bound on the accumulated integral torque: enough to trim a standing
/// disturbance that consumes most of an axis's authority — a tilted
/// hover puts a sustained torque on the yaw axis that a 0.4 bound
/// could not out-trim — while still leaving the P/D path headroom,
/// which the conditional advance (no growth while the axis output
/// saturates) protects better than a tight bound does.
const RATE_I_LIMIT: Scalar = 0.8;

impl RateController {
    pub fn new(gains: CascadeGains) -> Self {
        Self { gains }
    }

    pub fn step(
        &self,
        state: &mut RateLoopState,
        setpoint: [RadiansPerSecond; 3],
        current: [RadiansPerSecond; 3],
        dt_sec: Scalar,
    ) -> [NormalizedSigned; 3] {
        self.step_with_observation(state, setpoint, current, dt_sec)
            .0
    }

    /// Run one cycle and return terms that do not feed later control.
    pub(crate) fn step_with_observation(
        &self,
        state: &mut RateLoopState,
        setpoint: [RadiansPerSecond; 3],
        current: [RadiansPerSecond; 3],
        dt_sec: Scalar,
    ) -> ([NormalizedSigned; 3], ControllerLoopObservation) {
        let integrator_before = state.integral;
        // Update the filtered measurement (single-pole LPF).
        let alpha = self.gains.rate_d_lpf_alpha;
        let mut meas_filtered = state.meas_filtered_prev;
        for (i, c) in current.iter().enumerate() {
            let filtered = alpha * state.meas_filtered_prev.axis_get(i) + (1.0 - alpha) * c.0;
            meas_filtered.axis_set(i, filtered);
        }

        let mut out = [NormalizedSigned(0.0); 3];
        let mut p = [0.0; 3];
        let mut d = [0.0; 3];
        let mut action = [IntegratorAction::FrozenInactive; 3];
        let mut saturated = [false; 3];
        for i in 0..3 {
            let p_error = setpoint[i].0 - current[i].0;
            let p_term = p_error * self.gains.rate_p[i];
            p[i] = p_term;

            // Derivative-on-measurement, sign flipped so a positive
            // measurement-derivative damps the loop (positive
            // measurement rate of change → negative torque
            // contribution). The first cycle outputs no D term —
            // there's no previous sample to difference against.
            let d_term = if state.primed && dt_sec > 0.0 && self.gains.rate_d[i] > 0.0 {
                let d_meas =
                    (meas_filtered.axis_get(i) - state.meas_filtered_prev.axis_get(i)) / dt_sec;
                -d_meas * self.gains.rate_d[i]
            } else {
                0.0
            };
            d[i] = d_term;

            // Conditional anti-windup: the integral advances while
            // the un-clamped command has authority left, so a
            // saturated maneuver cannot wind a correction the
            // actuators never delivered — AND it may always shrink:
            // gating shrinking updates too would freeze a stale
            // integral through the whole saturated transient and bias
            // the recovery by up to the bound.
            let i_gain = self.gains.rate_i[i];
            let unsat = p_term + d_term + state.integral[i];
            let increment = p_error * i_gain * dt_sec;
            let shrinks = increment * state.integral[i] < 0.0;
            if i_gain > 0.0 && dt_sec > 0.0 && (unsat.abs() < 1.0 || shrinks) {
                state.integral[i] =
                    (state.integral[i] + increment).clamp(-RATE_I_LIMIT, RATE_I_LIMIT);
                action[i] = IntegratorAction::Integrated;
            } else if i_gain > 0.0 && dt_sec > 0.0 {
                action[i] = IntegratorAction::FrozenSaturation;
            }
            let command_unclamped = p_term + d_term + state.integral[i];
            saturated[i] = command_unclamped.abs() > 1.0;
            let cmd = command_unclamped.clamp(-1.0, 1.0);
            out[i] = NormalizedSigned(cmd);
        }

        state.meas_filtered_prev = meas_filtered;
        state.primed = true;
        (
            out,
            ControllerLoopObservation {
                p,
                i: state.integral,
                d,
                feedforward: [0.0; 3],
                integrator_before,
                integrator_after: state.integral,
                integrator_action: action,
                saturated,
            },
        )
    }
}

/// Tiny helper to index a `Vector3` by axis (0/1/2). Avoids a
/// public Index impl that would clash with the existing field
/// access API.
trait Vector3AxisAccess {
    fn axis_get(&self, i: usize) -> Scalar;
    fn axis_set(&mut self, i: usize, v: Scalar);
}

impl Vector3AxisAccess for Vector3<RadiansPerSecond> {
    fn axis_get(&self, i: usize) -> Scalar {
        match i {
            0 => self.x.0,
            1 => self.y.0,
            _ => self.z.0,
        }
    }
    fn axis_set(&mut self, i: usize, v: Scalar) {
        match i {
            0 => self.x = RadiansPerSecond(v),
            1 => self.y = RadiansPerSecond(v),
            _ => self.z = RadiansPerSecond(v),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Test-local gains with `rate_p = 2.5` on all axes and
    /// `rate_d = 0.05` on roll/pitch. Lets the unit assertions
    /// pin a specific P/D contribution without depending on
    /// whatever the X500 default happens to be tuned to — the
    /// per-axis behaviour the test asserts is unchanged.
    fn test_gains() -> CascadeGains {
        let mut g = CascadeGains::x500_defaults();
        g.rate_p = [2.5, 2.5, 2.5];
        g.rate_d = [0.05, 0.05, 0.0];
        g
    }

    fn ctrl() -> RateController {
        RateController::new(test_gains())
    }

    fn zero_rate() -> [RadiansPerSecond; 3] {
        [
            RadiansPerSecond(0.0),
            RadiansPerSecond(0.0),
            RadiansPerSecond(0.0),
        ]
    }

    #[test]
    fn first_cycle_outputs_no_d_term() {
        // Derivative on an uninitialized previous sample would be
        // a delta function; explicitly skip it on cycle one.
        let c = ctrl();
        let mut s = RateLoopState::default();
        let sp = [
            RadiansPerSecond(1.0),
            RadiansPerSecond(0.0),
            RadiansPerSecond(0.0),
        ];
        let cur = zero_rate();
        let out = c.step(&mut s, sp, cur, 0.001);
        // P only: 1.0 · 2.5 = 2.5, clamped to 1.0.
        assert!((out[0].0 - 1.0).abs() < 1e-5);
        assert!(s.primed);
    }

    #[test]
    fn d_term_damps_against_measurement_change() {
        // Setpoint zero, measurement steps up between cycles —
        // the D term should produce a NEGATIVE torque (damping).
        let c = ctrl();
        let mut s = RateLoopState::default();
        // Prime with a zero sample first.
        let _ = c.step(&mut s, zero_rate(), zero_rate(), 0.001);
        // Then a sample where measurement rose.
        let cur = [
            RadiansPerSecond(0.5),
            RadiansPerSecond(0.0),
            RadiansPerSecond(0.0),
        ];
        let out = c.step(&mut s, zero_rate(), cur, 0.001);
        // P term: -0.5 · 2.5 = -1.25 → clamped to -1.0.
        // D term: also negative (measurement rising → damping).
        // Both push the same direction, so output is at -1.0.
        assert!(out[0].0 < -0.99);
    }

    #[test]
    fn no_d_term_when_d_gain_is_zero() {
        // With rate_d disabled for an axis (e.g. yaw on a small
        // multirotor), the loop reduces to a plain P controller.
        let mut gains = test_gains();
        gains.rate_d = [0.0; 3];
        let c = RateController::new(gains);
        let mut s = RateLoopState::default();
        let _ = c.step(&mut s, zero_rate(), zero_rate(), 0.001);
        let cur = [
            RadiansPerSecond(0.1),
            RadiansPerSecond(0.0),
            RadiansPerSecond(0.0),
        ];
        let out = c.step(&mut s, zero_rate(), cur, 0.001);
        // P only: -0.1 · 2.5 = -0.25.
        assert!((out[0].0 + 0.25).abs() < 1e-5);
    }
}
