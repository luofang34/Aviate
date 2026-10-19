//! The rotor command reproduces the commanded thrust under JSBSim's
//! rotor law, and agrees with the kernel's resolved actuator curve.

use aviate_core::kernel::config::ActuatorCurveKind;
use aviate_core::mixer::ActuatorCmd;
use aviate_core::types::{Normalized, NormalizedThrust};

use super::*;

/// The thrust fraction JSBSim's X500 makes from command `c`.
fn jsbsim_thrust(c: f32) -> f32 {
    let omega = OMEGA_IDLE + OMEGA_SPAN * c;
    (omega / OMEGA_MAX).powi(2)
}

#[test]
fn the_command_reproduces_the_thrust_above_idle() {
    for thrust in [0.03, 0.1, 0.25, 0.5929, 0.8, 1.0] {
        let c = command_for_thrust(thrust);
        assert!((0.0..=1.0).contains(&c));
        assert!((jsbsim_thrust(c) - thrust).abs() < 1e-5, "{thrust}: c {c}");
    }
    // The X500's hover trim, 20.25 N of 34.19 N, needs 77 % rotor speed.
    assert!((command_for_thrust(0.5929) - 0.729_4).abs() < 1e-3);
}

#[test]
fn below_idle_thrust_and_out_of_range_commands_saturate() {
    assert_eq!(command_for_thrust(0.0), 0.0);
    assert_eq!(command_for_thrust(0.01), 0.0, "idle already gives 2.25 %");
    assert_eq!(command_for_thrust(-0.5), 0.0);
    assert_eq!(command_for_thrust(1.5), 1.0);
    assert_eq!(command_for_thrust(f32::NAN), 0.0);
}

#[test]
fn the_rotor_speed_agrees_with_the_kernels_actuator_curve() {
    // The kernel resolves the X500 as a quadratic rotor: its boundary
    // command is the rotor-speed fraction √f. The JSBSim command must
    // drive the rotor to that same speed.
    let kernel = aviate_app_sitl_gazebo_x500_kernel::build_x500_kernel();
    let Ok(kernel) = kernel else {
        panic!("the X500 kernel builds");
    };
    let curve = kernel.cfg().actuator_curve;
    assert_eq!(curve, ActuatorCurveKind::QuadraticRotor);
    for thrust in [0.1, 0.5929, 0.9] {
        let speed_fraction = curve.boundary_command(NormalizedThrust(thrust)).0;
        let omega = OMEGA_IDLE + OMEGA_SPAN * command_for_thrust(thrust);
        assert!(
            (omega - speed_fraction * OMEGA_MAX).abs() < 1e-2,
            "{thrust}"
        );
    }
}

#[test]
fn disarmed_rotors_get_no_command() {
    let mut command = ActuatorCmd::default();
    command.outputs[..4].fill(Normalized(0.6));
    let disarmed = x500_controls(&command, false);
    assert_eq!(disarmed.outputs, [0.0; 16]);
    assert!(!disarmed.armed);
    let armed = x500_controls(&command, true);
    assert_eq!(armed.count, 4);
    assert!(armed.outputs[..4]
        .iter()
        .all(|&c| c == command_for_thrust(0.6)));
    assert_eq!(&armed.outputs[4..], &[0.0; 12]);
}
