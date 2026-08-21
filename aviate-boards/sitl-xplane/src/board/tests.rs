//! Board behavior that does not need a live simulator: the actuator
//! curve is applied exactly once per active lane, and the board names
//! itself for the harness that selects it.

use aviate_core::kernel::config::ActuatorCurveKind;
use aviate_hal_xil::sim_types::SimActuatorCmd;

use super::{apply_actuator_curve, reorder_lanes, BOARD_INFO};

#[test]
fn the_quadratic_curve_applies_per_active_lane_only() {
    let mut cmd = SimActuatorCmd {
        count: 4,
        ..SimActuatorCmd::default()
    };
    cmd.outputs[..5].copy_from_slice(&[0.25, 1.0, 0.0, 0.5929, 0.36]);
    apply_actuator_curve(ActuatorCurveKind::QuadraticRotor, &mut cmd);
    assert!((cmd.outputs[0] - 0.5).abs() < 1e-6, "sqrt(0.25)");
    assert!((cmd.outputs[1] - 1.0).abs() < 1e-6);
    assert!((cmd.outputs[2] - 0.0).abs() < 1e-6);
    assert!((cmd.outputs[3] - 0.77).abs() < 1e-3, "the hover seed");
    assert!(
        (cmd.outputs[4] - 0.36).abs() < 1e-6,
        "a lane beyond count is untouched"
    );
}

#[test]
fn the_linear_curve_is_the_identity() {
    let mut cmd = SimActuatorCmd {
        count: 4,
        ..SimActuatorCmd::default()
    };
    cmd.outputs[..4].copy_from_slice(&[0.1, 0.4, 0.7, 1.0]);
    apply_actuator_curve(ActuatorCurveKind::Linear, &mut cmd);
    assert!((cmd.outputs[0] - 0.1).abs() < 1e-6);
    assert!((cmd.outputs[3] - 1.0).abs() < 1e-6);
}

#[test]
fn the_board_names_itself() {
    assert_eq!(BOARD_INFO.name, "sitl-xplane");
}

#[test]
fn the_model_lane_order_is_applied_once() {
    let mut outputs = [0.0_f32; 16];
    outputs[..4].copy_from_slice(&[0.1, 0.2, 0.3, 0.4]);
    reorder_lanes(&mut outputs, 4, [0, 2, 1, 3]);
    assert_eq!(&outputs[..4], &[0.1, 0.3, 0.2, 0.4]);
}

#[test]
fn a_short_command_does_not_use_the_four_lane_map() {
    let mut outputs = [0.0_f32; 16];
    outputs[..2].copy_from_slice(&[0.7, 0.8]);
    reorder_lanes(&mut outputs, 2, [0, 2, 1, 3]);
    assert_eq!(&outputs[..2], &[0.7, 0.8]);
}
