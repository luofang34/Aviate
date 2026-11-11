//! Lateral guidance through the controller step: bank from course and heading.

use super::{
    attitude_command, banked, command, course_command, cruise_state, rate_command, Rig, DT,
};
use crate::control::{ControlMode, Setpoint};
use crate::kernel::config::fixed_wing::tests::reference_params;
use crate::types::Radians;

#[test]
fn course_error_becomes_a_coordinated_turn_bank() {
    let mut rig = Rig::new();
    let _ = rig.step(&cruise_state(), &course_command(0.2));
    // 0.2 rad of error over the 5 s time constant is 0.04 rad/s, and
    // a 55 m/s turn at that rate banks atan(55 * 0.04 / 9.81).
    assert!((rig.runtime.targets.bank - 0.2206).abs() < 1.0e-3);

    let _ = rig.step(&cruise_state(), &course_command(-0.2));
    assert!((rig.runtime.targets.bank + 0.2206).abs() < 1.0e-3);

    // A large error stops at the route bank of the airframe, on either
    // side: the 45 degree envelope limit is not for a guidance turn.
    let _ = rig.step(&cruise_state(), &course_command(2.5));
    assert_eq!(rig.runtime.targets.bank, 0.44);
    let _ = rig.step(&cruise_state(), &course_command(-2.5));
    assert_eq!(rig.runtime.targets.bank, -0.44);

    // A route bank above the envelope limit does not widen the envelope.
    rig.limits.max_roll = Radians(0.3);
    let _ = rig.step(&cruise_state(), &course_command(2.5));
    assert_eq!(rig.runtime.targets.bank, 0.3);
}

#[test]
fn course_integrator_trims_straight_flight_and_stays_out_of_turns() {
    let mut rig = Rig::new();
    // A constant 0.02 rad of course error: 0.004 rad/s of demand over the
    // 5 s time constant, integrated over the 20 s integral time.
    let _ = rig.run(100, &cruise_state(), &course_command(0.02));
    let expected = 100.0 * 0.004 / 20.0 * DT;
    assert!((rig.runtime.lateral.turn_rate_integrator - expected).abs() < 1.0e-6);
    // The stored turn rate adds to the bank.
    let trimmed = rig.runtime.targets.bank;
    assert!(trimmed > (55.0_f32 * 0.004 / 9.81).atan());

    // A turn that the proportional term commands with more than half of
    // the route bank leaves the integrator as it is.
    let before = rig.runtime.lateral.turn_rate_integrator;
    let _ = rig.run(100, &cruise_state(), &course_command(1.0));
    assert_eq!(rig.runtime.lateral.turn_rate_integrator, before);

    // Held for a long time it stops when the bank reaches the route
    // bank: the stored turn rate plus the 0.01 rad/s of the proportional
    // term is then the turn rate of that bank, and no more is stored.
    let mut params = reference_params();
    params.course_integral_time.0 = 0.01;
    let mut rig = Rig::with(params);
    let _ = rig.run(2_000, &cruise_state(), &course_command(0.05));
    let route_turn_rate = 0.44_f32.tan() * 9.81 / 55.0;
    let stored = rig.runtime.lateral.turn_rate_integrator;
    assert!(stored <= route_turn_rate, "{stored}");
    assert!(stored > route_turn_rate - 0.011, "{stored}");
    assert_eq!(rig.runtime.targets.bank, 0.44);

    // A target that is not a direction clears it.
    let _ = rig.step(&cruise_state(), &attitude_command(0.1, 0.0, 0.5));
    assert_eq!(rig.runtime.lateral.turn_rate_integrator, 0.0);
    let _ = rig.run(10, &cruise_state(), &course_command(0.02));
    assert!(rig.runtime.lateral.turn_rate_integrator > 0.0);
    let _ = rig.step(
        &cruise_state(),
        &command(ControlMode::AltitudeHold, Setpoint::default()),
    );
    assert_eq!(rig.runtime.lateral.turn_rate_integrator, 0.0);
    let _ = rig.run(10, &cruise_state(), &course_command(0.02));
    let _ = rig.step(&cruise_state(), &rate_command(0.0, 0.0, 0.0));
    assert_eq!(rig.runtime.lateral.turn_rate_integrator, 0.0);
}

#[test]
fn heading_hold_turns_on_the_attitude_and_wings_level_does_not_turn() {
    let mut rig = Rig::new();
    let heading = command(
        ControlMode::AltitudeHold,
        Setpoint {
            heading: Some(Radians(-0.2)),
            ..Setpoint::default()
        },
    );
    let _ = rig.step(&cruise_state(), &heading);
    assert!((rig.runtime.targets.bank + 0.2206).abs() < 1.0e-3);

    let level = command(ControlMode::AltitudeHold, Setpoint::default());
    let _ = rig.step(&banked(0.3), &level);
    assert_eq!(rig.runtime.targets.bank, 0.0);
}

#[test]
fn bank_is_limited_by_the_envelope_and_by_the_stall_margin() {
    let mut rig = Rig::new();
    let _ = rig.step(&cruise_state(), &attitude_command(1.2, 0.0, 0.5));
    assert_eq!(rig.runtime.targets.bank, 0.785);
    let _ = rig.step(&cruise_state(), &attitude_command(-1.2, 0.0, 0.5));
    assert_eq!(rig.runtime.targets.bank, -0.785);

    // At 26 m/s the wing has (26/25)^2 = 1.08 times the lift of level
    // flight: a level turn can bank acos(1/1.08) = 0.39 rad.
    rig.probe = Some(26.0);
    rig.runtime = Default::default();
    let _ = rig.step(&cruise_state(), &course_command(2.5));
    assert!((rig.runtime.targets.bank - 0.391).abs() < 2.0e-3);

    // Below the stall speed no bank is available.
    rig.probe = Some(20.0);
    rig.runtime = Default::default();
    let _ = rig.step(&cruise_state(), &course_command(2.5));
    assert!(rig.runtime.targets.bank.abs() < 1.0e-3);
}
