//! Unit tests of the fixed-wing law, one file per concern.

mod air_data_tests;
mod fallback_tests;
mod lateral_tests;
mod loops_tests;
mod runtime_tests;
mod targets_tests;

use crate::control::{
    AxisCommand, Command, CommandSource, ConfigMode, ControlMode, Limits, Setpoint,
    VehicleControlMode, VehicleController,
};
use crate::kernel::config::fixed_wing::tests::reference_params;
use crate::kernel::config::{FixedWingParams, ResolvedKernelConfig};
use crate::math::{Quaternion, Vector3};
use crate::sensor::AirData;
use crate::state::{EstimateQuality, StateEstimate, StateValidFlags};
use crate::types::{
    Meters, MetersPerSecond, NormalizedThrust, Pascals, Radians, RadiansPerSecond, Scalar, Seconds,
};

use super::{FixedWingController, FixedWingRuntimeState};
use crate::control::runtime::ControllerRuntimeState;

/// Interval of one test cycle \[s\].
const DT: Scalar = 0.01;

fn controller() -> FixedWingController {
    FixedWingController::new(reference_params()).expect("valid block")
}

/// Envelope of a light aeroplane: 45 degrees of bank, 20 of pitch, 2.5 g.
fn limits() -> Limits {
    Limits {
        max_roll: Radians(0.785),
        max_pitch: Radians(0.35),
        max_roll_rate: RadiansPerSecond(0.8),
        max_pitch_rate: RadiansPerSecond(0.3),
        max_yaw_rate: RadiansPerSecond(0.3),
        max_horizontal_speed: MetersPerSecond(85.0),
        max_climb_rate: MetersPerSecond(5.0),
        max_descent_rate: MetersPerSecond(5.0),
        max_altitude: Meters(4_000.0),
        min_altitude: Meters(0.0),
        min_airspeed: None,
        max_airspeed: None,
        max_load_factor: 2.5,
        min_load_factor: 0.0,
    }
}

/// Attitude from roll, pitch and yaw in radians.
fn attitude(roll: Scalar, pitch: Scalar, yaw: Scalar) -> Quaternion {
    let about = |x, y, z, angle| Quaternion::from_axis_angle(Vector3::new(x, y, z), angle);
    about(0.0, 0.0, 1.0, yaw)
        .mul(&about(0.0, 1.0, 0.0, pitch))
        .mul(&about(1.0, 0.0, 0.0, roll))
}

/// Level flight north at 55 m/s, 300 m above the origin, fully valid.
fn cruise_state() -> StateEstimate {
    StateEstimate {
        attitude: Quaternion::IDENTITY,
        angular_velocity: [RadiansPerSecond(0.0); 3],
        position_ned: [Meters(0.0), Meters(0.0), Meters(-300.0)],
        velocity_ned: [
            MetersPerSecond(55.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ],
        quality: EstimateQuality::Good,
        valid_flags: StateValidFlags::all(),
    }
}

fn command(mode: ControlMode, setpoint: Setpoint) -> Command {
    Command {
        mode,
        setpoint,
        config_mode_request: None,
        sensor_overrides: None,
        sequence: 1,
        source: CommandSource::Autopilot,
    }
}

/// Dynamic pressure of `indicated` m/s at sea-level density.
fn pitot(indicated: Scalar) -> AirData {
    AirData {
        dynamic_pressure: Some(Pascals(
            0.5 * crate::sensor::SEA_LEVEL_AIR_DENSITY_KGPM3 * indicated * indicated,
        )),
        ..AirData::default()
    }
}

pub(super) fn course_command(course: f32) -> Command {
    command(
        ControlMode::VelocityControl,
        Setpoint {
            velocity: Some([
                MetersPerSecond(55.0 * course.cos()),
                MetersPerSecond(55.0 * course.sin()),
                MetersPerSecond(0.0),
            ]),
            ..Setpoint::default()
        },
    )
}

pub(super) fn height_command(height: f32) -> Command {
    command(
        ControlMode::VelocityControl,
        Setpoint {
            velocity: Some([
                MetersPerSecond(55.0),
                MetersPerSecond(0.0),
                MetersPerSecond(0.0),
            ]),
            altitude: Some(Meters(height)),
            ..Setpoint::default()
        },
    )
}

pub(super) fn climb_command(climb: f32) -> Command {
    command(
        ControlMode::VelocityControl,
        Setpoint {
            velocity: Some([
                MetersPerSecond(55.0),
                MetersPerSecond(0.0),
                MetersPerSecond(-climb),
            ]),
            ..Setpoint::default()
        },
    )
}

pub(super) fn attitude_command(roll: f32, pitch: f32, throttle: f32) -> Command {
    command(
        ControlMode::Attitude,
        Setpoint {
            attitude: Some(attitude(roll, pitch, 0.0)),
            collective_thrust: NormalizedThrust(throttle),
            ..Setpoint::default()
        },
    )
}

pub(super) fn rate_command(roll: f32, pitch: f32, yaw: f32) -> Command {
    command(
        ControlMode::Rate,
        Setpoint {
            angular_rate: Some([
                RadiansPerSecond(roll),
                RadiansPerSecond(pitch),
                RadiansPerSecond(yaw),
            ]),
            collective_thrust: NormalizedThrust(0.5),
            ..Setpoint::default()
        },
    )
}

pub(super) fn banked(roll: f32) -> StateEstimate {
    StateEstimate {
        attitude: attitude(roll, 0.0, 0.0),
        ..cruise_state()
    }
}

/// One kernel-shaped controller cycle: interval, air data, step.
struct Rig {
    controller: FixedWingController,
    runtime: FixedWingRuntimeState,
    limits: Limits,
    /// Indicated airspeed that the probe reports, or `None` for no probe.
    probe: Option<Scalar>,
    /// The kernel selected an automatic fallback from this mode.
    fallback_from: Option<ControlMode>,
}

impl Rig {
    fn new() -> Self {
        Self::with(reference_params())
    }

    fn with(params: FixedWingParams) -> Self {
        Self {
            controller: FixedWingController::new(params).expect("valid block"),
            runtime: FixedWingRuntimeState::default(),
            limits: limits(),
            probe: Some(55.0),
            fallback_from: None,
        }
    }

    fn step(&mut self, state: &StateEstimate, command: &Command) -> AxisCommand {
        self.runtime.set_cycle_interval(Seconds(DT));
        let air = self.probe.map(pitot);
        self.runtime.set_air_data(air.as_ref());
        let mut flags = VehicleControlMode::from_control_mode(command.mode);
        flags.automatic_fallback_from = self.fallback_from;
        self.controller.step(
            &mut self.runtime,
            state,
            command,
            &flags,
            ConfigMode::Cruise,
            &self.limits,
        )
    }

    /// Run `cycles` identical cycles and return the last output.
    fn run(&mut self, cycles: usize, state: &StateEstimate, command: &Command) -> AxisCommand {
        let mut output = AxisCommand::default();
        for _ in 0..cycles {
            output = self.step(state, command);
        }
        output
    }
}

#[test]
fn controller_binds_to_the_configuration_that_holds_its_block() {
    let controller = controller();
    let bound = ResolvedKernelConfig {
        fixed_wing: Some(reference_params()),
        ..ResolvedKernelConfig::default()
    };
    assert!(controller.verify_config_binding(&bound).is_ok());
    assert_eq!(*controller.params(), reference_params());

    let mut other = reference_params();
    other.cruise_airspeed.0 += 1.0;
    let mismatch = controller
        .verify_config_binding(&ResolvedKernelConfig {
            fixed_wing: Some(other),
            ..ResolvedKernelConfig::default()
        })
        .expect_err("another block must be refused");
    assert_ne!(mismatch.controller_identity, mismatch.config_identity);

    // A configuration without a block vouches for nothing.
    let missing = controller
        .verify_config_binding(&ResolvedKernelConfig::default())
        .expect_err("a missing block must be refused");
    assert_eq!(missing.config_identity, 0);
}

#[test]
fn a_block_that_cannot_fly_builds_no_controller() {
    let mut params = reference_params();
    params.stall_airspeed.0 = 0.0;
    assert!(FixedWingController::new(params).is_err());
}

#[test]
fn every_mode_but_position_hold_is_supported() {
    let controller = controller();
    for mode in [
        ControlMode::Rate,
        ControlMode::Attitude,
        ControlMode::AltitudeHold,
        ControlMode::VelocityControl,
        ControlMode::DeviationTracking,
    ] {
        assert!(controller.supports_mode(mode), "{mode:?}");
    }
    assert!(!controller.supports_mode(ControlMode::PositionHold));
}
