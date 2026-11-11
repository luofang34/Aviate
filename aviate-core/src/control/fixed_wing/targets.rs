//! What one command asks of a fixed wing: a lateral target, a vertical
//! target, an airspeed and a throttle.
//!
//! A fixed wing cannot hold an arbitrary velocity vector: it cannot stop,
//! hover or fly sideways, and its speed has a lower bound. A velocity
//! setpoint is therefore read as a course, an airspeed and a climb rate.

use crate::control::{Command, ControlMode, Limits, Setpoint, VehicleControlMode};
use crate::kernel::config::FixedWingParams;
use crate::state::{StateEstimate, StateValidFlags};
#[allow(unused_imports)] // FloatExt supplies the no_std math methods
use crate::types::FloatExt;
use crate::types::Scalar;

/// Limits that apply in this cycle: the tighter of the airframe
/// description and the configured envelope.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) struct Envelope {
    /// Lowest indicated airspeed to command \[m/s\].
    pub min_airspeed: Scalar,
    /// Highest indicated airspeed to command \[m/s\].
    pub max_airspeed: Scalar,
    /// Largest bank angle \[rad\].
    pub bank_limit: Scalar,
    /// Largest pitch angle, nose up and nose down \[rad\].
    pub pitch_limit: Scalar,
    /// Largest climb rate \[m/s\].
    pub climb_limit: Scalar,
    /// Largest sink rate \[m/s\].
    pub sink_limit: Scalar,
    /// Largest load factor.
    pub max_load_factor: Scalar,
    /// Smallest load factor.
    pub min_load_factor: Scalar,
    /// Largest body rates in roll, pitch, yaw order \[rad/s\].
    pub rate_limits: [Scalar; 3],
}

/// A limit magnitude that is safe as a clamp bound: zero or more, and
/// finite. A limit that is not a number, infinite or negative becomes
/// zero, which permits nothing, rather than a bound that inverts a range.
fn magnitude(limit: Scalar) -> Scalar {
    if limit.is_finite() {
        limit.max(0.0)
    } else {
        0.0
    }
}

impl Envelope {
    pub(super) fn resolve(params: &FixedWingParams, limits: &Limits) -> Self {
        // `max` and `min` return the other operand when one is not a
        // number, so a bad airspeed limit leaves the airframe value.
        let min_airspeed = limits.min_airspeed.map_or(params.min_airspeed.0, |limit| {
            limit.0.max(params.min_airspeed.0)
        });
        let max_airspeed = limits
            .max_airspeed
            .map_or(params.max_airspeed.0, |limit| {
                limit.0.min(params.max_airspeed.0)
            })
            .max(min_airspeed);
        // A load factor that is not a number allows level flight only.
        let max_load_factor = limits.max_load_factor.max(1.0);
        // A minimum that is not a number, or that is above the maximum,
        // is no limit that the law can honour: zero g takes its place.
        let min_load_factor = if limits.min_load_factor <= max_load_factor {
            limits.min_load_factor
        } else {
            0.0
        };
        // A level turn at bank angle b needs a load factor of 1/cos(b).
        let load_bank = libm::acosf(1.0 / max_load_factor);
        Self {
            min_airspeed,
            max_airspeed,
            bank_limit: magnitude(limits.max_roll.0).min(load_bank),
            pitch_limit: magnitude(limits.max_pitch.0),
            climb_limit: magnitude(limits.max_climb_rate.0).min(params.max_climb_rate.0),
            sink_limit: magnitude(limits.max_descent_rate.0).min(params.max_sink_rate.0),
            max_load_factor,
            min_load_factor,
            rate_limits: [
                magnitude(limits.max_roll_rate.0),
                magnitude(limits.max_pitch_rate.0),
                magnitude(limits.max_yaw_rate.0),
            ],
        }
    }
}

/// Lateral target of one cycle.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) enum Lateral {
    /// Hold the wings level.
    WingsLevel,
    /// Hold a bank angle \[rad\], positive right wing down.
    Bank(Scalar),
    /// Fly a course over the ground \[rad\], true.
    Course(Scalar),
    /// Fly a heading \[rad\], true. Needs no velocity estimate.
    Heading(Scalar),
}

/// Vertical target of one cycle.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) enum Vertical {
    /// Hold a pitch angle \[rad\] with the commanded throttle.
    Pitch(Scalar),
    /// Hold the airspeed with pitch. The throttle holds the pressure
    /// height of the probe when it has a static port, and the cruise
    /// throttle when it has none. Needs neither a position nor a
    /// velocity estimate.
    AirspeedOnPitch,
    /// Hold a climb rate \[m/s\], positive up.
    ClimbRate(Scalar),
    /// Hold a height above the origin \[m\], positive up.
    Height(Scalar),
}

/// Resolved targets of one cycle.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) enum Targets {
    /// Body rates in roll, pitch, yaw order \[rad/s\] and a throttle.
    Rates {
        rates: [Scalar; 3],
        throttle: Scalar,
    },
    /// Attitude and flight-path targets.
    Flight {
        lateral: Lateral,
        vertical: Vertical,
        /// Indicated airspeed to hold \[m/s\].
        airspeed: Scalar,
        /// Throttle for the vertical targets that do not command it.
        throttle: Scalar,
    },
}

/// Smallest angle equal to `angle` modulo one turn, in `[-pi, pi]`.
pub(super) fn wrap_angle(angle: Scalar) -> Scalar {
    angle.sin().atan2(angle.cos())
}

/// Speed over the ground \[m/s\].
pub(super) fn ground_speed(state: &StateEstimate) -> Scalar {
    let [north, east, _down] = state.velocity_ned;
    (north.0 * north.0 + east.0 * east.0).sqrt()
}

/// Course over the ground \[rad\], or the heading when the vehicle has
/// no usable velocity over the ground.
pub(super) fn ground_course(state: &StateEstimate) -> Scalar {
    let (_roll, _pitch, yaw) = state.attitude.to_euler();
    if state.valid_flags.contains(StateValidFlags::VELOCITY)
        && ground_speed(state) > Scalar::EPSILON
    {
        let [north, east, _down] = state.velocity_ned;
        east.0.atan2(north.0)
    } else {
        yaw
    }
}

/// Course and airspeed from a horizontal velocity setpoint. `None` when
/// the setpoint has no horizontal direction.
fn course_and_speed(setpoint: &Setpoint) -> Option<(Scalar, Scalar)> {
    let [north, east, _down] = setpoint.velocity?;
    let speed = (north.0 * north.0 + east.0 * east.0).sqrt();
    (speed > Scalar::EPSILON).then(|| (east.0.atan2(north.0), speed))
}

/// Height or climb-rate target from the vertical setpoint fields.
fn climb_target(setpoint: &Setpoint) -> Vertical {
    if let Some(altitude) = setpoint.altitude {
        return Vertical::Height(altitude.0);
    }
    // NED: a positive down rate is a descent.
    let down_rate = setpoint
        .vertical_speed
        .or(setpoint.velocity.map(|[_north, _east, down]| down))
        .map_or(0.0, |rate| rate.0);
    Vertical::ClimbRate(-down_rate)
}

/// Lateral target of a mode that holds a height and may hold a heading:
/// the heading when the command has one, else the course of its velocity
/// flown as a heading, else the commanded bank.
fn heading_or_bank(setpoint: &Setpoint) -> Lateral {
    let course = course_and_speed(setpoint).map(|(course, _speed)| course);
    match (
        setpoint.heading.map(|heading| heading.0).or(course),
        setpoint.attitude,
    ) {
        (Some(heading), _) => Lateral::Heading(heading),
        (None, Some(attitude)) => Lateral::Bank(attitude.to_euler().0),
        (None, None) => Lateral::WingsLevel,
    }
}

/// Targets for a command that tracks a reference path.
fn deviation_targets(
    setpoint: &Setpoint,
    state: &StateEstimate,
    params: &FixedWingParams,
    cruise: Scalar,
) -> Targets {
    let track_course = setpoint
        .heading
        .map_or_else(|| ground_course(state), |course| course.0);
    // Positive deviation: the vehicle is right of the track, so the
    // commanded course turns left of the track course to close it.
    let lookahead =
        params.track_lookahead_time.0 * ground_speed(state).max(params.stall_airspeed.0);
    let intercept = setpoint.lateral_deviation.map_or(0.0, |deviation| {
        libm::atanf(deviation.0 / lookahead)
            .clamp(-params.max_intercept_angle.0, params.max_intercept_angle.0)
    });
    // Positive deviation: the vehicle is above the profile.
    let [_north, _east, down] = state.position_ned;
    let vertical = match setpoint.vertical_deviation {
        Some(deviation) => Vertical::Height(-down.0 - deviation.0),
        None => climb_target(setpoint),
    };
    Targets::Flight {
        lateral: Lateral::Course(wrap_angle(track_course - intercept)),
        vertical,
        airspeed: course_and_speed(setpoint).map_or(cruise, |(_course, speed)| speed),
        throttle: params.cruise_throttle.0,
    }
}

/// Targets for a velocity command: course, airspeed and climb.
fn velocity_targets(setpoint: &Setpoint, params: &FixedWingParams, cruise: Scalar) -> Targets {
    let (lateral, airspeed) = match course_and_speed(setpoint) {
        Some((course, speed)) => (Lateral::Course(course), speed),
        None => (heading_or_bank(setpoint), cruise),
    };
    Targets::Flight {
        lateral,
        vertical: climb_target(setpoint),
        airspeed,
        throttle: params.cruise_throttle.0,
    }
}

/// Targets for a direct attitude command.
fn attitude_targets(
    setpoint: &Setpoint,
    flags: &VehicleControlMode,
    params: &FixedWingParams,
    cruise: Scalar,
) -> Targets {
    if flags.automatic_fallback_from.is_some() {
        // The command was written for a guidance mode that the estimate
        // can no longer support. Its attitude and throttle fields say
        // nothing, but its course and airspeed still state the intent.
        // The course needs a velocity estimate, so the law flies it as a
        // heading, and it holds the airspeed with pitch.
        let intent = course_and_speed(setpoint);
        let direction = intent
            .map(|(course, _speed)| course)
            .or(setpoint.heading.map(|heading| heading.0));
        return Targets::Flight {
            lateral: direction.map_or(Lateral::WingsLevel, Lateral::Heading),
            vertical: Vertical::AirspeedOnPitch,
            airspeed: intent.map_or(cruise, |(_course, speed)| speed),
            throttle: params.cruise_throttle.0,
        };
    }
    let (roll, pitch) = setpoint
        .attitude
        .map_or((0.0, params.trim_pitch.0), |attitude| {
            let (roll, pitch, _yaw) = attitude.to_euler();
            (roll, pitch)
        });
    Targets::Flight {
        lateral: Lateral::Bank(roll),
        vertical: Vertical::Pitch(pitch),
        airspeed: cruise,
        throttle: setpoint.collective_thrust.0,
    }
}

impl Vertical {
    /// The target that the present estimate can support.
    ///
    /// A height and a climb rate need a valid height channel. The kernel
    /// can hand over a command whose mode the gate did not check against
    /// the estimate, so the law checks here: it must not close a loop on
    /// a number that the estimator disowns.
    fn supported_by(self, valid: StateValidFlags) -> Self {
        match self {
            Vertical::ClimbRate(_) | Vertical::Height(_)
                if !valid.contains(StateValidFlags::HEIGHT) =>
            {
                Vertical::AirspeedOnPitch
            }
            other => other,
        }
    }
}

impl Targets {
    /// Read `command` as fixed-wing targets that the estimate supports.
    pub(super) fn resolve(
        command: &Command,
        flags: &VehicleControlMode,
        state: &StateEstimate,
        params: &FixedWingParams,
    ) -> Self {
        match Self::requested(command, flags, state, params) {
            Targets::Flight {
                lateral,
                vertical,
                airspeed,
                throttle,
            } => Targets::Flight {
                lateral,
                vertical: vertical.supported_by(state.valid_flags),
                airspeed,
                throttle,
            },
            rates @ Targets::Rates { .. } => rates,
        }
    }

    /// Read `command` as fixed-wing targets.
    fn requested(
        command: &Command,
        flags: &VehicleControlMode,
        state: &StateEstimate,
        params: &FixedWingParams,
    ) -> Self {
        let setpoint = &command.setpoint;
        let cruise = params.cruise_airspeed.0;
        match command.mode {
            ControlMode::Rate => Targets::Rates {
                rates: setpoint
                    .angular_rate
                    .map_or([0.0; 3], |rates| rates.map(|rate| rate.0)),
                throttle: setpoint.collective_thrust.0,
            },
            ControlMode::Attitude => attitude_targets(setpoint, flags, params, cruise),
            // A fixed wing cannot hold a position. A caller that asks
            // for it past the capability gate gets a height hold.
            ControlMode::AltitudeHold | ControlMode::PositionHold => Targets::Flight {
                lateral: heading_or_bank(setpoint),
                vertical: climb_target(setpoint),
                airspeed: course_and_speed(setpoint).map_or(cruise, |(_course, speed)| speed),
                throttle: params.cruise_throttle.0,
            },
            ControlMode::VelocityControl => velocity_targets(setpoint, params, cruise),
            ControlMode::DeviationTracking => deviation_targets(setpoint, state, params, cruise),
        }
    }
}
