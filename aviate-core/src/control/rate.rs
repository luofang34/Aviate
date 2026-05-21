//! Rate loop — innermost cascade layer. Converts angular-rate
//! error into normalized torque commands for the mixer.
//!
//! P + D, with derivative-on-measurement (not derivative-on-
//! error). The setpoint can step instantaneously when the
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
use crate::math::Vector3;
use crate::types::{NormalizedSigned, RadiansPerSecond, Scalar};

/// Persistent state owned by the rate loop. Lives inside
/// `MultirotorRuntimeState`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RateLoopState {
    /// Previous filtered rate measurement (rad/s body frame), one
    /// per axis. Used to compute Δ(meas)/Δt for the D term.
    pub meas_filtered_prev: Vector3<RadiansPerSecond>,
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
        self.primed = false;
    }
}

#[derive(Clone, Debug)]
pub struct RateController {
    pub gains: CascadeGains,
}

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
        // Update the filtered measurement (single-pole LPF).
        let alpha = self.gains.rate_d_lpf_alpha;
        let mut meas_filtered = state.meas_filtered_prev;
        for i in 0..3 {
            let raw = current[i].0;
            let filtered = alpha * state.meas_filtered_prev.axis_get(i) + (1.0 - alpha) * raw;
            meas_filtered.axis_set(i, filtered);
        }

        let mut out = [NormalizedSigned(0.0); 3];
        for i in 0..3 {
            let p_error = setpoint[i].0 - current[i].0;
            let p_term = p_error * self.gains.rate_p[i];

            // Derivative-on-measurement, sign flipped so a positive
            // measurement-derivative damps the loop (positive
            // measurement rate of change → negative torque
            // contribution). The first cycle outputs no D term —
            // there's no previous sample to difference against.
            let d_term = if state.primed && dt_sec > 0.0 && self.gains.rate_d[i] > 0.0 {
                let d_meas = (meas_filtered.axis_get(i)
                    - state.meas_filtered_prev.axis_get(i))
                    / dt_sec;
                -d_meas * self.gains.rate_d[i]
            } else {
                0.0
            };

            let cmd = (p_term + d_term).clamp(-1.0, 1.0);
            out[i] = NormalizedSigned(cmd);
        }

        state.meas_filtered_prev = meas_filtered;
        state.primed = true;
        out
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
