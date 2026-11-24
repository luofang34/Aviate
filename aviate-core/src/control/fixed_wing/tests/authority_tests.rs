//! Control memory of the law while its outputs do not move the vehicle.

use super::super::FixedWingRuntimeState;
use super::fallback_tests::{fallback_cycle, probe_at};
use super::{command, cruise_state, Rig};
use crate::control::runtime::ControllerRuntimeState;
use crate::control::{AxisCommand, Command, ControlMode, Setpoint};
use crate::kernel::config::fixed_wing::tests::reference_params;
use crate::sensor::ActuatorAuthority;
use crate::state::StateEstimate;
use crate::types::{Meters, MetersPerSecond, RadiansPerSecond};

/// Cycles of 10 ms in the 6 s that a pilot flies by hand.
const HAND_FLOWN_CYCLES: usize = 600;

/// Level cruise with a body rate on each axis that the law did not ask for.
fn disturbed_state() -> StateEstimate {
    StateEstimate {
        angular_velocity: [
            RadiansPerSecond(0.02),
            RadiansPerSecond(-0.02),
            RadiansPerSecond(0.02),
        ],
        ..cruise_state()
    }
}

/// A course 0.05 rad right of the track and a climb of 1 m/s: each loop
/// of the law sees an error that level cruise does not answer.
fn course_and_climb() -> Command {
    command(
        ControlMode::VelocityControl,
        Setpoint {
            velocity: Some([
                MetersPerSecond(55.0 * 0.05_f32.cos()),
                MetersPerSecond(55.0 * 0.05_f32.sin()),
                MetersPerSecond(-1.0),
            ]),
            ..Setpoint::default()
        },
    )
}

/// A height `above` metres over the vehicle, flown at `airspeed`.
fn height_at_airspeed(above: f32, airspeed: f32) -> Command {
    command(
        ControlMode::VelocityControl,
        Setpoint {
            velocity: Some([
                MetersPerSecond(55.0),
                MetersPerSecond(0.0),
                MetersPerSecond(0.0),
            ]),
            altitude: Some(Meters(300.0 + above)),
            airspeed: Some(MetersPerSecond(airspeed)),
            ..Setpoint::default()
        },
    )
}

fn bits(output: &AxisCommand) -> [u32; 4] {
    [
        output.roll.0.to_bits(),
        output.pitch.0.to_bits(),
        output.yaw.0.to_bits(),
        output.collective.0.to_bits(),
    ]
}

fn rig_with(authority: ActuatorAuthority) -> Rig {
    let mut rig = Rig::new();
    rig.authority = Some(authority);
    rig
}

#[test]
fn authority_is_available_until_the_kernel_reports_its_loss() {
    assert_eq!(ActuatorAuthority::default(), ActuatorAuthority::Available);
    let mut runtime = FixedWingRuntimeState::default();
    assert_eq!(runtime.actuator_authority, ActuatorAuthority::Available);
    runtime.set_actuator_authority(ActuatorAuthority::Absent);
    assert_eq!(runtime.actuator_authority, ActuatorAuthority::Absent);
    runtime.reset();
    assert_eq!(runtime.actuator_authority, ActuatorAuthority::Available);
}

#[test]
fn integrators_fill_with_authority_and_stay_empty_without_it() {
    let (state, demand) = (disturbed_state(), course_and_climb());

    // With authority the persistent errors fill every integrator.
    let mut engaged = rig_with(ActuatorAuthority::Available);
    let _ = engaged.run(HAND_FLOWN_CYCLES, &state, &demand);
    let memory = &engaged.runtime;
    assert!(memory.lateral.turn_rate_integrator > 1.0e-3);
    assert!(memory.energy.throttle_integrator > 0.05);
    assert!(memory.energy.pitch_integrator > 0.01);
    let [roll, pitch, yaw] = memory.rates.integrators;
    assert!(roll > 0.05, "{roll}");
    assert!(pitch > 0.05, "{pitch}");
    assert!(yaw < -0.005, "{yaw}");

    // Without authority no cycle leaves control memory behind.
    let mut released = rig_with(ActuatorAuthority::Absent);
    for cycle in 0..HAND_FLOWN_CYCLES {
        let output = released.step(&state, &demand);
        let memory = &released.runtime;
        assert_eq!(memory.lateral, Default::default(), "cycle {cycle}");
        assert_eq!(memory.energy, Default::default(), "cycle {cycle}");
        assert_eq!(memory.rates, Default::default(), "cycle {cycle}");
        // The law still gives its targets and its command.
        assert!(memory.targets.bank > 0.01);
        assert!(memory.targets.pitch > reference_params().trim_pitch.0);
        assert!(output.roll.0 > 0.01);
        assert!(output.collective.0 > reference_params().cruise_throttle.0);
    }
}

#[test]
fn the_cycle_that_loses_authority_releases_the_memory_that_was_there() {
    let (state, demand) = (disturbed_state(), course_and_climb());
    let mut rig = rig_with(ActuatorAuthority::Available);
    let _ = rig.run(HAND_FLOWN_CYCLES, &state, &demand);
    assert_ne!(rig.runtime.rates, Default::default());

    rig.authority = Some(ActuatorAuthority::Absent);
    let _ = rig.step(&state, &demand);
    assert_eq!(rig.runtime.lateral, Default::default());
    assert_eq!(rig.runtime.energy, Default::default());
    assert_eq!(rig.runtime.rates, Default::default());
    // The measurement memory stays.
    assert!(rig.runtime.air.is_fresh());
}

#[test]
fn the_first_cycle_with_authority_is_that_of_fresh_control_memory() {
    let state = disturbed_state();
    let mut rig = rig_with(ActuatorAuthority::Available);
    let _ = rig.run(HAND_FLOWN_CYCLES, &state, &course_and_climb());

    // By hand the pilot slows below the 33 m/s minimum and recovers to
    // 40 m/s: with memory the underspeed protection would stay latched
    // below the 44 m/s recovery airspeed.
    rig.authority = Some(ActuatorAuthority::Absent);
    let demand = height_at_airspeed(1.0, 40.0);
    rig.probe = Some(30.0);
    let _ = rig.run(HAND_FLOWN_CYCLES, &state, &demand);
    assert!(rig.runtime.air.indicated_mps < 33.0);
    rig.probe = Some(40.0);
    let _ = rig.run(HAND_FLOWN_CYCLES, &state, &demand);

    // The reference law has the same air-data memory and nothing else.
    let mut fresh = rig_with(ActuatorAuthority::Available);
    fresh.probe = rig.probe;
    fresh.runtime = FixedWingRuntimeState {
        air: rig.runtime.air,
        ..FixedWingRuntimeState::default()
    };

    rig.authority = Some(ActuatorAuthority::Available);
    let returned = rig.step(&state, &demand);
    let reference = fresh.step(&state, &demand);
    assert_eq!(bits(&returned), bits(&reference));
    assert_eq!(rig.runtime, fresh.runtime);
    assert!(!rig.runtime.energy.underspeed);
    assert!(returned.collective.0 < 0.9, "{}", returned.collective.0);

    // From that cycle the two laws are one law.
    for cycle in 0..HAND_FLOWN_CYCLES {
        let returned = rig.step(&state, &demand);
        let reference = fresh.step(&state, &demand);
        assert_eq!(bits(&returned), bits(&reference), "cycle {cycle}");
    }
    assert_eq!(rig.runtime, fresh.runtime);
    assert_ne!(rig.runtime.rates, Default::default());
}

#[test]
fn a_pressure_level_is_not_held_without_authority() {
    let mut rig = Rig::new();
    let cruise_throttle = reference_params().cruise_throttle.0;
    let _ = fallback_cycle(&mut rig, &probe_at(95_000.0));
    assert_eq!(rig.runtime.held_static_pa, Some(95_000.0));

    // By hand the pilot descends to the 95 100 Pa level. Without
    // authority the law holds no level, so it does not ask the throttle
    // to climb back: a held level 8.9 m above would open the throttle
    // by more than 0.05.
    rig.runtime
        .set_actuator_authority(ActuatorAuthority::Absent);
    let mut output = AxisCommand::default();
    for _ in 0..HAND_FLOWN_CYCLES {
        output = fallback_cycle(&mut rig, &probe_at(95_100.0));
        assert_eq!(rig.runtime.held_static_pa, None);
    }
    assert!((output.collective.0 - cruise_throttle).abs() < 5.0e-3);

    // The level that the law holds is the one at which authority returns.
    rig.runtime
        .set_actuator_authority(ActuatorAuthority::Available);
    let output = fallback_cycle(&mut rig, &probe_at(95_100.0));
    let held = rig.runtime.held_static_pa.expect("a level is taken");
    assert!((held - 95_100.0).abs() < 1.0, "{held}");
    assert!((output.collective.0 - cruise_throttle).abs() < 5.0e-3);
}

#[test]
fn a_kernel_that_reports_no_authority_lane_flies_as_with_authority() {
    let (state, demand) = (disturbed_state(), course_and_climb());
    let mut silent = Rig::new();
    let mut engaged = rig_with(ActuatorAuthority::Available);
    for cycle in 0..HAND_FLOWN_CYCLES {
        let expected = silent.step(&state, &demand);
        let output = engaged.step(&state, &demand);
        assert_eq!(bits(&output), bits(&expected), "cycle {cycle}");
    }
    assert_eq!(engaged.runtime, silent.runtime);
}
