use super::*;

fn sine() -> ExcitationWaveform {
    ExcitationWaveform::Sine {
        amplitude: 0.06,
        frequency_rad_s: 2.5,
    }
}

fn three_actions() -> [CalibrationAction; 3] {
    [
        CalibrationAction::LaneInjection(LaneInjection {
            axis: InjectionAxis::Roll,
            waveform: sine(),
            window: Duration::from_secs(16),
        }),
        CalibrationAction::TestStand(TestStandCommand::Engage),
        CalibrationAction::HoldCurrentAttitude,
    ]
}

#[test]
fn actions_construct_programmatically() {
    let actions = three_actions();
    assert_eq!(actions[0].kind(), CalibrationActionKind::LaneInjection);
    assert_eq!(actions[1].kind(), CalibrationActionKind::TestStand);
    assert_eq!(
        actions[2].kind(),
        CalibrationActionKind::HoldCurrentAttitude
    );
}

#[test]
fn every_action_is_simulator_only() {
    for action in three_actions() {
        assert!(action.simulator_only());
        assert!(action.admit(TargetKind::Simulator).is_ok());
    }
}

#[test]
fn non_simulator_target_refuses_each_action_with_typed_error() {
    for action in three_actions() {
        let result = action.admit(TargetKind::Hardware);
        assert_eq!(
            result,
            Err(CalibrationError::SimulatorOnly {
                action: action.kind(),
                target: TargetKind::Hardware,
            })
        );
    }
}

#[test]
fn lane_injection_receipt_confirms_expected_and_actual() {
    let window = ExcitationWindowReceipt {
        window_start: Duration::from_secs(40),
        window_end: Duration::from_secs(56),
        waveform_digest: sine().digest(),
    };
    let expected = [0.06, -0.06, 0.06, -0.06];
    let receipt =
        LaneInjectionReceipt::confirm(expected, expected, window, Duration::from_secs(56), 0.001);
    assert!(receipt.is_ok());
    let Some(receipt) = receipt.ok() else {
        return;
    };
    assert_eq!(receipt.expected_lanes, expected);
    assert_eq!(receipt.actual_lanes, expected);
    assert_eq!(receipt.window.window_start, Duration::from_secs(40));
    assert_eq!(receipt.window.window_end, Duration::from_secs(56));
    assert_eq!(receipt.window.waveform_digest, sine().digest());
    assert_eq!(receipt.simulation_time, Duration::from_secs(56));
}

#[test]
fn lane_injection_receipt_refuses_a_mismatch() {
    let window = ExcitationWindowReceipt {
        window_start: Duration::ZERO,
        window_end: Duration::from_secs(16),
        waveform_digest: sine().digest(),
    };
    let result = LaneInjectionReceipt::confirm(
        [0.06, -0.06, 0.06, -0.06],
        [0.06, -0.06, 0.06, 0.10],
        window,
        Duration::from_secs(16),
        0.001,
    );
    assert!(matches!(
        result,
        Err(CalibrationError::Readback {
            action: CalibrationActionKind::LaneInjection,
            ..
        })
    ));
}

#[test]
fn test_stand_receipt_confirms_expected_and_actual() {
    let receipt = TestStandReceipt::confirm(
        TestStandCommand::Engage,
        "local_y",
        -5.0,
        -5.1,
        Duration::from_secs(20),
        0.5,
    );
    assert!(receipt.is_ok());
    let Some(receipt) = receipt.ok() else {
        return;
    };
    assert_eq!(receipt.command, TestStandCommand::Engage);
    assert_eq!(receipt.expected, -5.0);
    assert_eq!(receipt.actual, -5.1);
    assert_eq!(receipt.simulation_time, Duration::from_secs(20));
}

#[test]
fn test_stand_receipt_refuses_a_mismatch() {
    let result = TestStandReceipt::confirm(
        TestStandCommand::Pin,
        "local_y",
        -5.0,
        -7.0,
        Duration::from_secs(21),
        0.5,
    );
    assert!(matches!(
        result,
        Err(CalibrationError::Readback {
            action: CalibrationActionKind::TestStand,
            field: "local_y",
            ..
        })
    ));
}

#[test]
fn attitude_hold_receipt_confirms_expected_and_actual() {
    let expected = [1.0, 0.0, 0.0, 0.0];
    let receipt = AttitudeHoldReceipt::confirm(expected, expected, Duration::from_secs(42), 0.001);
    assert!(receipt.is_ok());
    let Some(receipt) = receipt.ok() else {
        return;
    };
    assert_eq!(receipt.expected, expected);
    assert_eq!(receipt.actual, expected);
    assert_eq!(receipt.simulation_time, Duration::from_secs(42));
}

#[test]
fn attitude_hold_receipt_refuses_a_mismatch() {
    let result = AttitudeHoldReceipt::confirm(
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        Duration::from_secs(42),
        0.001,
    );
    assert!(matches!(
        result,
        Err(CalibrationError::Readback {
            action: CalibrationActionKind::HoldCurrentAttitude,
            ..
        })
    ));
}

#[test]
fn waveform_digest_is_stable_and_kind_sensitive() {
    assert_eq!(sine().digest(), sine().digest());
    let other = ExcitationWaveform::Sine {
        amplitude: 0.08,
        frequency_rad_s: 2.5,
    };
    assert_ne!(sine().digest(), other.digest());
}

#[test]
fn validate_accepts_a_well_formed_injection() {
    let injection = LaneInjection {
        axis: InjectionAxis::Roll,
        waveform: sine(),
        window: Duration::from_secs(16),
    };
    assert!(injection.validate().is_ok());
}

#[test]
fn validate_refuses_a_nan_amplitude() {
    let injection = LaneInjection {
        axis: InjectionAxis::Roll,
        waveform: ExcitationWaveform::Sine {
            amplitude: f32::NAN,
            frequency_rad_s: 2.5,
        },
        window: Duration::from_secs(16),
    };
    assert!(matches!(
        injection.validate(),
        Err(CalibrationError::InvalidParameter {
            action: CalibrationActionKind::LaneInjection,
            field: "amplitude",
            ..
        })
    ));
}

#[test]
fn validate_refuses_a_zero_amplitude() {
    let injection = LaneInjection {
        axis: InjectionAxis::Roll,
        waveform: ExcitationWaveform::Sine {
            amplitude: 0.0,
            frequency_rad_s: 2.5,
        },
        window: Duration::from_secs(16),
    };
    assert!(matches!(
        injection.validate(),
        Err(CalibrationError::InvalidParameter {
            field: "amplitude",
            ..
        })
    ));
}

#[test]
fn validate_refuses_a_zero_window() {
    let injection = LaneInjection {
        axis: InjectionAxis::Roll,
        waveform: sine(),
        window: Duration::ZERO,
    };
    assert!(matches!(
        injection.validate(),
        Err(CalibrationError::InvalidParameter {
            field: "window",
            ..
        })
    ));
}

#[test]
fn lane_readback_names_the_refusing_lane() {
    let window = ExcitationWindowReceipt {
        window_start: Duration::ZERO,
        window_end: Duration::from_secs(16),
        waveform_digest: sine().digest(),
    };
    let result = LaneInjectionReceipt::confirm(
        [0.06, -0.06, 0.06, -0.06],
        [0.06, -0.06, 0.06, 0.10],
        window,
        Duration::from_secs(16),
        0.001,
    );
    assert!(matches!(
        result,
        Err(CalibrationError::Readback { field: "lane3", .. })
    ));
}

#[test]
fn receipt_sum_type_carries_simulator_time() {
    let window = ExcitationWindowReceipt {
        window_start: Duration::ZERO,
        window_end: Duration::from_secs(16),
        waveform_digest: sine().digest(),
    };
    let receipts = [
        CalibrationReceipt::LaneInjection(LaneInjectionReceipt {
            expected_lanes: [0.0; 4],
            actual_lanes: [0.0; 4],
            window,
            simulation_time: Duration::from_secs(1),
        }),
        CalibrationReceipt::TestStand(TestStandReceipt {
            command: TestStandCommand::Release,
            expected: 0.0,
            actual: 0.0,
            simulation_time: Duration::from_secs(2),
        }),
        CalibrationReceipt::AttitudeHold(AttitudeHoldReceipt {
            expected: [1.0, 0.0, 0.0, 0.0],
            actual: [1.0, 0.0, 0.0, 0.0],
            simulation_time: Duration::from_secs(3),
        }),
    ];
    let times: Vec<Duration> = receipts
        .iter()
        .map(CalibrationReceipt::simulation_time)
        .collect();
    assert_eq!(
        times,
        [
            Duration::from_secs(1),
            Duration::from_secs(2),
            Duration::from_secs(3)
        ]
    );
}
