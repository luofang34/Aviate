//! What the law flies when the estimate or the airspeed is not available.

use super::{attitude_command, cruise_state, height_command, Rig, DT};
use crate::control::ControlMode;
use crate::kernel::config::fixed_wing::tests::reference_params;
use crate::state::StateValidFlags;

/// Air data of a probe with a static port: 55 m/s at `pressure` Pa, 15 C.
fn probe_at(pressure: f32) -> crate::sensor::AirData {
    crate::sensor::AirData {
        static_pressure: Some(crate::types::Pascals(pressure)),
        temperature: Some(crate::types::Celsius(15.0)),
        ..super::pitot(55.0)
    }
}

/// One fallback cycle with the given air data.
fn fallback_cycle(rig: &mut Rig, air: &crate::sensor::AirData) -> crate::control::AxisCommand {
    use crate::control::runtime::ControllerRuntimeState;
    use crate::control::{ConfigMode, VehicleControlMode, VehicleController};
    let demand = attitude_command(0.0, 0.0, 0.0);
    let mut flags = VehicleControlMode::from_control_mode(demand.mode);
    flags.automatic_fallback_from = Some(ControlMode::VelocityControl);
    rig.runtime.set_cycle_interval(crate::types::Seconds(DT));
    rig.runtime.set_air_data(Some(air));
    let mut blind = cruise_state();
    blind
        .valid_flags
        .remove(StateValidFlags::VELOCITY | StateValidFlags::POSITION);
    rig.controller.step(
        &mut rig.runtime,
        &blind,
        &demand,
        &flags,
        ConfigMode::Cruise,
        &rig.limits,
    )
}

#[test]
fn fallback_holds_the_pressure_level_of_the_probe_with_the_throttle() {
    let mut rig = Rig::new();
    let cruise_throttle = reference_params().cruise_throttle.0;
    let output = fallback_cycle(&mut rig, &probe_at(95_000.0));
    assert_eq!(rig.runtime.held_static_pa, Some(95_000.0));
    assert!((output.collective.0 - cruise_throttle).abs() < 1.0e-6);

    // The vehicle sinks: the static pressure rises by 100 Pa, 8.9 m at
    // this level. The throttle opens to climb back, and the held level
    // stays the one at which the fallback began.
    let mut output = output;
    for _ in 0..600 {
        output = fallback_cycle(&mut rig, &probe_at(95_100.0));
    }
    assert_eq!(rig.runtime.held_static_pa, Some(95_000.0));
    assert!(
        output.collective.0 > cruise_throttle + 0.05,
        "{}",
        output.collective.0
    );

    // Above the level the throttle closes.
    let mut rig = Rig::new();
    let _ = fallback_cycle(&mut rig, &probe_at(95_000.0));
    let mut output = rig.step(&cruise_state(), &attitude_command(0.0, 0.0, 0.0));
    assert_eq!(
        rig.runtime.held_static_pa, None,
        "a direct command releases the level"
    );
    let _ = fallback_cycle(&mut rig, &probe_at(95_000.0));
    for _ in 0..600 {
        output = fallback_cycle(&mut rig, &probe_at(94_900.0));
    }
    assert!(
        output.collective.0 < cruise_throttle - 0.05,
        "{}",
        output.collective.0
    );
}

#[test]
fn fallback_without_a_static_port_uses_the_cruise_throttle() {
    let mut rig = Rig::new();
    let output = fallback_cycle(&mut rig, &super::pitot(55.0));
    assert_eq!(rig.runtime.held_static_pa, None);
    assert_eq!(output.collective.0, reference_params().cruise_throttle.0);

    // A port that goes away after the level was taken: the throttle
    // returns to cruise and the level is kept for the port's return.
    let _ = fallback_cycle(&mut rig, &probe_at(95_000.0));
    let output = fallback_cycle(&mut rig, &super::pitot(55.0));
    assert_eq!(rig.runtime.held_static_pa, Some(95_000.0));
    assert_eq!(output.collective.0, reference_params().cruise_throttle.0);
}

#[test]
fn underspeed_needs_a_measurement_and_a_height_target() {
    // No probe: the ground speed is not the airflow.
    let mut rig = Rig::new();
    rig.probe = None;
    let mut slow = cruise_state();
    slow.velocity_ned[0].0 = 28.0;
    let _ = rig.step(&slow, &height_command(400.0));
    assert!(!rig.runtime.energy.underspeed);

    // Airspeed already on pitch: there is no height target to give up.
    let mut rig = Rig::new();
    rig.probe = Some(28.0);
    rig.fallback_from = Some(ControlMode::VelocityControl);
    let output = rig.step(&cruise_state(), &attitude_command(0.0, 0.0, 0.0));
    assert!(!rig.runtime.energy.underspeed);
    assert_eq!(output.collective.0, reference_params().cruise_throttle.0);
    assert!(rig.runtime.targets.pitch < reference_params().trim_pitch.0);
    assert_eq!(rig.runtime.energy.throttle_integrator, 0.0);
}

#[test]
fn fallback_without_any_airspeed_holds_the_trim_pitch() {
    let mut rig = Rig::new();
    rig.probe = None;
    rig.fallback_from = Some(ControlMode::VelocityControl);
    let mut blind = cruise_state();
    blind.valid_flags.remove(StateValidFlags::VELOCITY);
    let output = rig.step(&blind, &attitude_command(0.4, 0.2, 0.0));
    assert_eq!(rig.runtime.targets.bank, 0.0);
    assert_eq!(rig.runtime.targets.pitch, reference_params().trim_pitch.0);
    assert_eq!(output.collective.0, reference_params().cruise_throttle.0);
}
