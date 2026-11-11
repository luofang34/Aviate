//! Airframe description and loop tuning that the fixed-wing control law
//! reads from `ResolvedKernelConfig`.
//!
//! The block has no default values: an airframe crate supplies every
//! number, and `canonical_hash` covers each of them. The control law holds
//! no airframe number of its own.

use crate::types::{MetersPerSecond, NormalizedThrust, Radians, Scalar, Seconds};

/// Airframe description and loop tuning for one fixed-wing vehicle.
///
/// Airspeeds are indicated airspeeds. Rates and limits are magnitudes.
/// Axis arrays are in roll, pitch, yaw order.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FixedWingParams {
    /// Stall speed in the clean configuration at 1 g.
    pub stall_airspeed: MetersPerSecond,
    /// Lowest airspeed that the law commands and protects.
    pub min_airspeed: MetersPerSecond,
    /// Airspeed that the law holds when the command gives no speed.
    /// It is also the reference speed of the surface gains.
    pub cruise_airspeed: MetersPerSecond,
    /// Highest airspeed that the law commands.
    pub max_airspeed: MetersPerSecond,
    /// Lowest throttle that the energy loop commands.
    pub min_throttle: NormalizedThrust,
    /// Throttle for level flight at `cruise_airspeed`.
    pub cruise_throttle: NormalizedThrust,
    /// Highest throttle that the energy loop commands.
    pub max_throttle: NormalizedThrust,
    /// Climb rate at `max_throttle` and `cruise_airspeed`.
    pub max_climb_rate: MetersPerSecond,
    /// Sink rate at `min_throttle` and `cruise_airspeed`.
    pub max_sink_rate: MetersPerSecond,
    /// Pitch attitude in level flight at `cruise_airspeed`.
    pub trim_pitch: Radians,
    /// Time constant of the height error to climb-rate demand.
    pub height_time_constant: Seconds,
    /// Time constant of the airspeed error to acceleration demand.
    pub speed_time_constant: Seconds,
    /// Throttle for each m/s of total-energy-rate error.
    pub energy_rate_p: Scalar,
    /// Throttle rate for each m/s of total-energy-rate error.
    pub energy_rate_i: Scalar,
    /// Pitch for each radian of energy-balance-rate error.
    pub balance_rate_p: Scalar,
    /// Pitch rate for each radian of energy-balance-rate error.
    pub balance_rate_i: Scalar,
    /// Largest bank angle that the course, heading and track loops
    /// command. The envelope limit in `Limits` stays the bound of a direct
    /// bank command and of the protections.
    pub route_bank_angle: Radians,
    /// Time constant of the course error to turn-rate demand.
    pub course_time_constant: Seconds,
    /// Integral time of the course loop: the time in which a constant
    /// course error adds the turn-rate demand that it already commands.
    pub course_integral_time: Seconds,
    /// Time ahead on the track at which the deviation loop aims.
    pub track_lookahead_time: Seconds,
    /// Largest angle between the commanded course and the track course.
    pub max_intercept_angle: Radians,
    /// Time constant of the roll error to roll-rate demand.
    pub roll_time_constant: Seconds,
    /// Time constant of the pitch error to pitch-rate demand.
    pub pitch_time_constant: Seconds,
    /// Surface deflection for each rad/s of body-rate error.
    pub rate_p: [Scalar; 3],
    /// Surface deflection rate for each rad/s of body-rate error.
    pub rate_i: [Scalar; 3],
    /// Surface deflection for each rad/s of body-rate demand.
    pub rate_ff: [Scalar; 3],
    /// Largest surface deflection that a rate integrator holds.
    pub rate_integrator_limit: Scalar,
    /// Lowest gain scale, used at high airspeed.
    pub airspeed_scale_min: Scalar,
    /// Highest gain scale, used at low airspeed.
    pub airspeed_scale_max: Scalar,
    /// Time constant of the air-data filters: one stage on the airspeed
    /// and the static pressure, and a second stage on their rates.
    pub airspeed_filter_time_constant: Seconds,
}

/// Why a [`FixedWingParams`] block cannot fly.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FixedWingParamsError {
    /// A value is NaN or infinite.
    NotFinite,
    /// A value that must be more than zero is not.
    NotPositive,
    /// A gain is negative.
    NegativeGain,
    /// The airspeeds are not in the order stall, minimum, cruise, maximum.
    AirspeedOrder,
    /// The throttles are not in the order minimum, cruise, maximum inside 0 to 1.
    ThrottleOrder,
    /// The gain-scale bounds do not enclose 1.
    ScaleOrder,
}

impl core::fmt::Display for FixedWingParamsError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str(match self {
            Self::NotFinite => "a fixed-wing parameter is not finite",
            Self::NotPositive => "a fixed-wing parameter that must be positive is not",
            Self::NegativeGain => "a fixed-wing gain is negative",
            Self::AirspeedOrder => {
                "fixed-wing airspeeds are not stall < minimum < cruise < maximum"
            }
            Self::ThrottleOrder => {
                "fixed-wing throttles are not 0 <= minimum < cruise < maximum <= 1"
            }
            Self::ScaleOrder => "fixed-wing gain-scale bounds do not enclose 1",
        })
    }
}

impl core::error::Error for FixedWingParamsError {}

impl FixedWingParams {
    /// Every value in a fixed order: the order of the canonical encoding.
    pub(crate) fn values(&self) -> [Scalar; 36] {
        let [rate_p_roll, rate_p_pitch, rate_p_yaw] = self.rate_p;
        let [rate_i_roll, rate_i_pitch, rate_i_yaw] = self.rate_i;
        let [rate_ff_roll, rate_ff_pitch, rate_ff_yaw] = self.rate_ff;
        [
            self.stall_airspeed.0,
            self.min_airspeed.0,
            self.cruise_airspeed.0,
            self.max_airspeed.0,
            self.min_throttle.0,
            self.cruise_throttle.0,
            self.max_throttle.0,
            self.max_climb_rate.0,
            self.max_sink_rate.0,
            self.trim_pitch.0,
            self.height_time_constant.0,
            self.speed_time_constant.0,
            self.energy_rate_p,
            self.energy_rate_i,
            self.balance_rate_p,
            self.balance_rate_i,
            self.route_bank_angle.0,
            self.course_time_constant.0,
            self.course_integral_time.0,
            self.track_lookahead_time.0,
            self.max_intercept_angle.0,
            self.roll_time_constant.0,
            self.pitch_time_constant.0,
            rate_p_roll,
            rate_p_pitch,
            rate_p_yaw,
            rate_i_roll,
            rate_i_pitch,
            rate_i_yaw,
            rate_ff_roll,
            rate_ff_pitch,
            rate_ff_yaw,
            self.rate_integrator_limit,
            self.airspeed_scale_min,
            self.airspeed_scale_max,
            self.airspeed_filter_time_constant.0,
        ]
    }

    /// Values that divide, or that bound a range from above: each must be
    /// more than zero.
    fn positives(&self) -> [Scalar; 15] {
        [
            self.stall_airspeed.0,
            self.max_climb_rate.0,
            self.max_sink_rate.0,
            self.height_time_constant.0,
            self.speed_time_constant.0,
            self.route_bank_angle.0,
            self.course_time_constant.0,
            self.course_integral_time.0,
            self.track_lookahead_time.0,
            self.max_intercept_angle.0,
            self.roll_time_constant.0,
            self.pitch_time_constant.0,
            self.rate_integrator_limit,
            self.airspeed_scale_min,
            self.airspeed_filter_time_constant.0,
        ]
    }

    /// Loop gains: zero switches a term off, a negative value inverts it.
    fn gains(&self) -> [Scalar; 13] {
        let [rate_p_roll, rate_p_pitch, rate_p_yaw] = self.rate_p;
        let [rate_i_roll, rate_i_pitch, rate_i_yaw] = self.rate_i;
        let [rate_ff_roll, rate_ff_pitch, rate_ff_yaw] = self.rate_ff;
        [
            self.energy_rate_p,
            self.energy_rate_i,
            self.balance_rate_p,
            self.balance_rate_i,
            rate_p_roll,
            rate_p_pitch,
            rate_p_yaw,
            rate_i_roll,
            rate_i_pitch,
            rate_i_yaw,
            rate_ff_roll,
            rate_ff_pitch,
            rate_ff_yaw,
        ]
    }

    /// Check that the block describes a vehicle that the law can fly.
    ///
    /// # Errors
    ///
    /// Returns the first [`FixedWingParamsError`] that applies.
    pub fn validate(&self) -> Result<(), FixedWingParamsError> {
        if self.values().iter().any(|value| !value.is_finite()) {
            return Err(FixedWingParamsError::NotFinite);
        }
        if self.positives().iter().any(|value| *value <= 0.0) {
            return Err(FixedWingParamsError::NotPositive);
        }
        if self.gains().iter().any(|gain| *gain < 0.0) {
            return Err(FixedWingParamsError::NegativeGain);
        }
        let airspeeds = [
            self.stall_airspeed.0,
            self.min_airspeed.0,
            self.cruise_airspeed.0,
            self.max_airspeed.0,
        ];
        if !strictly_increasing(&airspeeds) {
            return Err(FixedWingParamsError::AirspeedOrder);
        }
        let throttles = [
            self.min_throttle.0,
            self.cruise_throttle.0,
            self.max_throttle.0,
        ];
        if self.min_throttle.0 < 0.0
            || self.max_throttle.0 > 1.0
            || !strictly_increasing(&throttles)
        {
            return Err(FixedWingParamsError::ThrottleOrder);
        }
        if self.airspeed_scale_min > 1.0 || self.airspeed_scale_max < 1.0 {
            return Err(FixedWingParamsError::ScaleOrder);
        }
        Ok(())
    }
}

fn strictly_increasing(values: &[Scalar]) -> bool {
    values
        .iter()
        .zip(values.iter().skip(1))
        .all(|(lower, upper)| lower < upper)
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]
pub(crate) mod tests;
