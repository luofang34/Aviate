//! Board behavior that does not need a live simulator: the actuator
//! curve is applied exactly once per active lane, and the board names
//! itself for the harness that selects it.
#![allow(clippy::expect_used)]

use aviate_config::xplane_model::XPlaneSimulatorModel;
use aviate_core::kernel::config::ActuatorCurveKind;
use aviate_hal_xil::sim_types::SimActuatorCmd;

use super::{
    observation::{apply_actuator_curve, prepare_actuator_command, reorder_lanes},
    wire::WireConstraints,
    BOARD_INFO,
};

mod backend_session;

const MODEL: &str = include_str!("../../../../presets/alia250-xplane.toml");

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
    reorder_lanes(&mut outputs, [0, 2, 1, 3]);
    assert_eq!(&outputs[..4], &[0.1, 0.3, 0.2, 0.4]);
}

#[test]
fn the_complete_wire_curve_and_lane_pipeline_uses_one_model() {
    let model = XPlaneSimulatorModel::from_toml_str(MODEL).expect("valid model");
    let mut wire = WireConstraints::new(model.wire());
    wire.arm(Some(10.0));
    for _ in 0..1_000 {
        let mut warm = SimActuatorCmd {
            count: 4,
            armed: true,
            ..SimActuatorCmd::default()
        };
        warm.outputs[..4].fill(0.43);
        assert!(
            prepare_actuator_command(
                &mut warm,
                [0.0; 4],
                &mut wire,
                ActuatorCurveKind::QuadraticRotor,
                model.lane_order(),
                model.motor_count(),
                Some(11.0),
                0.05,
            )
            .valid_count
        );
    }
    let mut cmd = SimActuatorCmd {
        count: 4,
        armed: true,
        ..SimActuatorCmd::default()
    };
    cmd.outputs[..4].copy_from_slice(&[0.33, 0.43, 0.53, 0.43]);
    assert!(
        prepare_actuator_command(
            &mut cmd,
            [0.0; 4],
            &mut wire,
            ActuatorCurveKind::QuadraticRotor,
            model.lane_order(),
            model.motor_count(),
            Some(11.0),
            0.01,
        )
        .valid_count
    );
    assert!(
        cmd.outputs[1] > cmd.outputs[2],
        "lane one receives mixer lane two"
    );
    assert!(cmd.outputs[..4].iter().all(|value| *value <= 1.0));
}

#[test]
fn an_actuator_count_mismatch_produces_a_disarmed_safe_command() {
    let model = XPlaneSimulatorModel::from_toml_str(MODEL).expect("valid model");
    let mut wire = WireConstraints::new(model.wire());
    let mut cmd = SimActuatorCmd {
        count: 3,
        armed: true,
        ..SimActuatorCmd::default()
    };
    cmd.outputs[..4].fill(0.5);
    assert!(
        !prepare_actuator_command(
            &mut cmd,
            [0.0; 4],
            &mut wire,
            ActuatorCurveKind::QuadraticRotor,
            model.lane_order(),
            model.motor_count(),
            None,
            0.01,
        )
        .valid_count
    );
    assert!(!cmd.armed);
    assert!(cmd.outputs.iter().all(|value| *value == 0.0));
}

#[test]
fn observation_reports_the_constrained_force_input() {
    let model = XPlaneSimulatorModel::from_toml_str(MODEL).expect("valid model");
    let mut wire = WireConstraints::new(model.wire());
    wire.arm(Some(10.0));
    let mut cmd = SimActuatorCmd {
        count: 4,
        armed: true,
        ..SimActuatorCmd::default()
    };
    cmd.outputs[..4].fill(0.43);
    let prepared = prepare_actuator_command(
        &mut cmd,
        [0.6, -0.6, 0.6, -0.6],
        &mut wire,
        ActuatorCurveKind::QuadraticRotor,
        model.lane_order(),
        model.motor_count(),
        Some(10.0),
        0.01,
    );
    assert!(prepared.valid_count);
    assert!(prepared.constraint_flags.any());
    assert_ne!(prepared.pre_wire_force_lanes, prepared.applied_force_lanes);
    assert!(prepared
        .applied_force_lanes
        .iter()
        .all(|value| *value <= model.wire().lane_ceiling));
}

#[test]
fn a_clamp_is_only_blamed_on_injection_when_injection_caused_it() {
    // The flag is attribution, not detection. A lane can reach the clamp on
    // its own — the perturbation authority scale runs past unity and is
    // applied upstream — and recording that as an injection cause writes a
    // reason into a trial record that the run did not have.
    let model = XPlaneSimulatorModel::from_toml_str(MODEL).expect("valid model");
    let mut wire = WireConstraints::new(model.wire());
    let prepared = |wire: &mut WireConstraints, lanes: [f32; 4], injection: [f32; 4]| {
        let mut cmd = SimActuatorCmd {
            count: 4,
            armed: true,
            ..SimActuatorCmd::default()
        };
        cmd.outputs[..4].copy_from_slice(&lanes);
        prepare_actuator_command(
            &mut cmd,
            injection,
            wire,
            ActuatorCurveKind::QuadraticRotor,
            model.lane_order(),
            model.motor_count(),
            Some(11.0),
            0.01,
        )
    };

    // Out of range on its own: clamped, but not by an injection.
    let alone = prepared(&mut wire, [1.4, 0.4, 0.4, 0.4], [0.0; 4]);
    assert!(
        !alone.constraint_flags.injection_clamp,
        "a lane that clamped with no injection was blamed on one"
    );

    // The same clamp, this time caused by the injection.
    let mut fresh = WireConstraints::new(model.wire());
    let injected = prepared(&mut fresh, [0.4, 0.4, 0.4, 0.4], [1.0, 0.0, 0.0, 0.0]);
    assert!(
        injected.constraint_flags.injection_clamp,
        "an injection that pushed a lane past the range was not recorded"
    );
}
