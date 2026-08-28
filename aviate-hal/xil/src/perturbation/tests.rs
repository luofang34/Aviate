#![allow(clippy::expect_used)]

use super::actuator::schedule;
use super::*;
use crate::sim_types::{SimBaroData, SimImuData, SimMagData, SimSensorPacket};

mod exhaustion;
mod validation;

const GOLDEN_DIGEST: [u8; 32] = [
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25,
    26, 27, 28, 29, 30, 31,
];
const GOLDEN_RUN_SEED: u64 = 0x1112_1314_1516_1718;

fn identity() -> PerturbationIdentity {
    PerturbationIdentity {
        condition_digest: GOLDEN_DIGEST,
        run_seed: GOLDEN_RUN_SEED,
    }
}

fn actuator_config() -> PerturbationConfig {
    PerturbationConfig {
        identity: identity(),
        sensor_noise: Vec::new(),
        actuator: ActuatorPerturbation {
            authority_scale_basis_points: 8_000,
            command_hold: Some(CommandHoldPerturbation {
                fraction_basis_points: 1_000,
                decision_interval_samples: 100,
            }),
        },
    }
}

fn sensor_config(run_seed: u64) -> PerturbationConfig {
    PerturbationConfig {
        identity: PerturbationIdentity {
            condition_digest: GOLDEN_DIGEST,
            run_seed,
        },
        sensor_noise: vec![
            SensorNoise {
                lane: SensorLane::AccelerometerX,
                peak_amplitude: 2.0,
                update_interval_samples: 2,
            },
            SensorNoise {
                lane: SensorLane::DifferentialPressure,
                peak_amplitude: 200.0,
                update_interval_samples: 1,
            },
        ],
        actuator: ActuatorPerturbation::default(),
    }
}

fn packet() -> SimSensorPacket {
    SimSensorPacket::new(10)
        .with_imu(SimImuData {
            accel: [1.0, 2.0, 3.0],
            gyro: [4.0, 5.0, 6.0],
            temperature: Some(20.0),
        })
        .with_mag(SimMagData {
            field_ut: [7.0, 8.0, 9.0],
        })
        .with_baro(SimBaroData {
            pressure_pa: 100_000.0,
            differential_pressure_pa: Some(500.0),
            pressure_altitude_m: Some(100.0),
            temperature_c: 20.0,
        })
}

fn command(value: f32) -> SimActuatorCmd {
    let mut command = SimActuatorCmd::new(0, 4, true);
    command.outputs[..4].fill(value);
    command
}

fn apply_sample(
    engine: &mut PerturbationEngine,
    sequence: u64,
    command: &mut SimActuatorCmd,
    eligibility: ActuatorEligibility,
) -> ActuatorApplication {
    let mut sample = SimSensorPacket::new(sequence);
    engine
        .apply_sensor(sequence, &mut sample)
        .expect("sensor application");
    let application = engine
        .apply_actuator(sequence, command, eligibility, 0)
        .expect("actuator application");
    engine
        .complete_actuator_send(sequence, true)
        .expect("actuator send completion");
    application
}

#[test]
fn command_hold_matches_the_pilotage_golden_interval() {
    let decisions = schedule(
        identity(),
        0,
        0,
        1_001,
        CommandHoldPerturbation {
            fraction_basis_points: 100,
            decision_interval_samples: 100,
        },
    )
    .expect("valid schedule");
    let held = decisions
        .iter()
        .enumerate()
        .filter_map(|(index, selected)| selected.then_some(index))
        .collect::<Vec<_>>();

    assert_eq!(held, vec![45]);
}

#[test]
fn prime_is_outside_interval_zero_and_each_complete_interval_is_exact() {
    let mut engine = PerturbationEngine::new(actuator_config()).expect("engine");
    let mut selected = 0_u32;
    for sequence in 0_u64..=100 {
        let mut command = command((sequence.wrapping_add(1) as f32) / 200.0);
        let application = apply_sample(
            &mut engine,
            sequence,
            &mut command,
            ActuatorEligibility::Eligible,
        );
        if sequence == 0 {
            assert!(application.prime);
            assert!(application.interval_position.is_none());
        } else {
            assert_eq!(application.interval_position, Some((sequence - 1) as u32));
            selected = selected.wrapping_add(u32::from(application.selected_hold));
        }
    }

    assert_eq!(selected, 10);
}

#[test]
fn safety_bypass_preserves_the_command_and_clears_hold_history() {
    let mut engine = PerturbationEngine::new(actuator_config()).expect("engine");
    let mut first = command(0.5);
    let prime = apply_sample(&mut engine, 0, &mut first, ActuatorEligibility::Eligible);
    assert!(prime.prime);
    assert_eq!(first.outputs[0], 0.4);

    let mut safety = command(0.25);
    let bypass = apply_sample(
        &mut engine,
        1,
        &mut safety,
        ActuatorEligibility::Bypass(ActuatorBypassReason::ArmTransition),
    );
    assert_eq!(safety.outputs[0], 0.25);
    assert_eq!(bypass.authority_scaled_lanes[0], 0.25);

    let mut next = command(0.75);
    let new_prime = apply_sample(&mut engine, 2, &mut next, ActuatorEligibility::Eligible);
    assert!(new_prime.prime);
    let mut interval_start = command(0.6);
    let application = apply_sample(
        &mut engine,
        3,
        &mut interval_start,
        ActuatorEligibility::Eligible,
    );
    assert_eq!(application.interval_epoch, Some(1));
    assert_eq!(application.interval_index, Some(0));
    assert_eq!(application.interval_position, Some(0));
}

#[test]
fn actuator_evidence_records_the_eligibility_preimage() {
    let mut engine = PerturbationEngine::new(actuator_config()).expect("engine");
    let mut sample = SimSensorPacket::new(7);
    engine
        .apply_sensor(7, &mut sample)
        .expect("sensor application");
    let mut command = command(0.5);
    let application = engine
        .apply_actuator(
            7,
            &mut command,
            ActuatorEligibility::Bypass(ActuatorBypassReason::FallbackMask),
            0b0101,
        )
        .expect("actuator application");

    assert!(application.requested_armed);
    assert_eq!(application.kernel_fallback_mask, 0b0101);
    assert_eq!(application.lane_count, 4);
}

#[test]
fn sensor_noise_is_repeatable_and_uses_zero_order_hold_buckets() {
    let mut first = PerturbationEngine::new(sensor_config(7)).expect("first engine");
    let mut repeated = PerturbationEngine::new(sensor_config(7)).expect("repeated engine");
    let mut changed = PerturbationEngine::new(sensor_config(8)).expect("changed engine");
    let mut first_zero = packet();
    let mut repeated_zero = packet();
    let mut changed_zero = packet();
    let evidence = first
        .apply_sensor(0, &mut first_zero)
        .expect("first sensor application");
    repeated
        .apply_sensor(0, &mut repeated_zero)
        .expect("repeated sensor application");
    changed
        .apply_sensor(0, &mut changed_zero)
        .expect("changed sensor application");
    complete_reply(&mut first, 0);
    complete_reply(&mut repeated, 0);
    complete_reply(&mut changed, 0);

    assert_eq!(
        first_zero.imu.map(|value| value.accel[0]),
        repeated_zero.imu.map(|value| value.accel[0])
    );
    assert_ne!(
        first_zero.imu.map(|value| value.accel[0]),
        changed_zero.imu.map(|value| value.accel[0])
    );
    assert_eq!(
        evidence.update_buckets[SensorLane::AccelerometerX as usize],
        Some(0)
    );
    assert_ne!(evidence.raw_digest, evidence.effective_digest);
    assert_eq!(evidence.raw_value_bits[0], Some(1.0_f32.to_bits()));
    assert_eq!(
        evidence.effective_value_bits[0],
        Some(first_zero.imu.expect("IMU").accel[0].to_bits())
    );

    let mut first_one = packet();
    first
        .apply_sensor(1, &mut first_one)
        .expect("second sample");
    assert_eq!(
        first_zero.imu.map(|value| value.accel[0]),
        first_one.imu.map(|value| value.accel[0])
    );
}

#[test]
fn a_missing_lane_does_not_change_or_record_an_update() {
    let mut engine = PerturbationEngine::new(sensor_config(7)).expect("engine");
    let mut sample = SimSensorPacket::new(0);
    let evidence = engine
        .apply_sensor(0, &mut sample)
        .expect("missing lane application");

    assert_eq!(evidence.raw_digest, evidence.effective_digest);
    assert_eq!(evidence.raw_value_bits, evidence.effective_value_bits);
    assert_eq!(evidence.changed_mask, 0);
    assert_eq!(evidence.update_buckets, [None; 12]);
}

#[test]
fn rounded_sensor_noise_does_not_claim_a_changed_value() {
    let mut config = sensor_config(7);
    config.sensor_noise = vec![SensorNoise {
        lane: SensorLane::AccelerometerX,
        peak_amplitude: 1.0e-9,
        update_interval_samples: 1,
    }];
    let mut engine = PerturbationEngine::new(config).expect("engine");
    let mut sample = packet();
    sample.imu.as_mut().expect("IMU").accel[0] = 1.0e20;

    let evidence = engine
        .apply_sensor(0, &mut sample)
        .expect("sensor application");

    assert_eq!(evidence.raw_value_bits[0], evidence.effective_value_bits[0]);
    assert_eq!(evidence.changed_mask, 0);
    assert_eq!(evidence.update_buckets[0], Some(0));
}

#[test]
fn every_safety_reason_preserves_output_and_starts_a_new_epoch() {
    let reasons = [
        ActuatorBypassReason::MissingAnswer,
        ActuatorBypassReason::InvalidActuatorCount,
        ActuatorBypassReason::Backup,
        ActuatorBypassReason::Direct,
        ActuatorBypassReason::Failsafe,
        ActuatorBypassReason::FallbackMask,
        ActuatorBypassReason::ArmTransition,
        ActuatorBypassReason::Disarmed,
        ActuatorBypassReason::EmergencyTermination,
    ];
    for reason in reasons {
        let mut engine = PerturbationEngine::new(actuator_config()).expect("engine");
        let mut prime = command(0.5);
        assert!(apply_sample(&mut engine, 10, &mut prime, ActuatorEligibility::Eligible).prime);
        let mut safety = command(0.25);
        let bypass = apply_sample(
            &mut engine,
            11,
            &mut safety,
            ActuatorEligibility::Bypass(reason),
        );
        assert_eq!(safety.outputs[0], 0.25);
        assert_eq!(bypass.eligibility, ActuatorEligibility::Bypass(reason));
        let mut new_prime = command(0.75);
        assert!(
            apply_sample(
                &mut engine,
                12,
                &mut new_prime,
                ActuatorEligibility::Eligible
            )
            .prime
        );
        let mut interval = command(0.6);
        let application = apply_sample(
            &mut engine,
            13,
            &mut interval,
            ActuatorEligibility::Eligible,
        );
        assert_eq!(application.interval_epoch, Some(1));
        assert_eq!(application.interval_position, Some(0));
    }
}

#[test]
fn a_sequence_fault_quarantines_the_engine() {
    let mut engine = PerturbationEngine::new(sensor_config(7)).expect("engine");
    let mut sample = packet();
    engine
        .apply_sensor(41, &mut sample)
        .expect("nonzero first sequence");
    complete_reply(&mut engine, 41);
    assert!(matches!(
        engine.apply_sensor(43, &mut sample),
        Err(PerturbationError::SampleSequence {
            expected: 42,
            received: 43
        })
    ));
    assert_eq!(
        engine.apply_sensor(42, &mut sample),
        Err(PerturbationError::Quarantined)
    );
}

#[test]
fn duplicate_and_missing_actuator_lifecycle_events_quarantine_the_run() {
    let mut duplicate_decision = PerturbationEngine::new(actuator_config()).expect("engine");
    let mut sample = SimSensorPacket::new(9);
    duplicate_decision
        .apply_sensor(9, &mut sample)
        .expect("sensor sample");
    let mut first = command(0.5);
    duplicate_decision
        .apply_actuator(9, &mut first, ActuatorEligibility::Eligible, 0)
        .expect("first actuator decision");
    assert!(matches!(
        duplicate_decision.apply_actuator(9, &mut first, ActuatorEligibility::Eligible, 0),
        Err(PerturbationError::DuplicateActuatorDecision(9))
    ));

    let mut missing_send = PerturbationEngine::new(actuator_config()).expect("engine");
    let mut sample = SimSensorPacket::new(80);
    missing_send
        .apply_sensor(80, &mut sample)
        .expect("sensor sample");
    let mut first = command(0.5);
    missing_send
        .apply_actuator(80, &mut first, ActuatorEligibility::Eligible, 0)
        .expect("actuator decision");
    assert!(matches!(
        missing_send.apply_sensor(81, &mut SimSensorPacket::new(81)),
        Err(PerturbationError::MissingActuatorSend(80))
    ));

    let mut duplicate_send = PerturbationEngine::new(actuator_config()).expect("engine");
    let mut sample = SimSensorPacket::new(90);
    duplicate_send
        .apply_sensor(90, &mut sample)
        .expect("sensor sample");
    let mut first = command(0.5);
    duplicate_send
        .apply_actuator(90, &mut first, ActuatorEligibility::Eligible, 0)
        .expect("actuator decision");
    duplicate_send
        .complete_actuator_send(90, true)
        .expect("first send completion");
    assert!(matches!(
        duplicate_send.complete_actuator_send(90, true),
        Err(PerturbationError::DuplicateActuatorSend(90))
    ));
}

#[test]
fn a_failed_actuator_send_is_never_accepted_as_execution() {
    let mut engine = PerturbationEngine::new(actuator_config()).expect("engine");
    let mut sample = SimSensorPacket::new(120);
    engine
        .apply_sensor(120, &mut sample)
        .expect("sensor sample");
    let mut first = command(0.5);
    engine
        .apply_actuator(120, &mut first, ActuatorEligibility::Eligible, 0)
        .expect("actuator decision");

    assert_eq!(
        engine.complete_actuator_send(120, false),
        Err(PerturbationError::ActuatorSendFailed(120))
    );
    assert_eq!(
        engine.apply_sensor(121, &mut SimSensorPacket::new(121)),
        Err(PerturbationError::Quarantined)
    );
}

#[test]
fn presence_and_decoded_values_must_agree() {
    let mut engine = PerturbationEngine::new(sensor_config(7)).expect("engine");
    let mut partial_vector = SimSensorPacket::new(7);
    partial_vector.presence_mask = 0b110;
    assert!(matches!(
        engine.apply_sensor(7, &mut partial_vector),
        Err(PerturbationError::SensorPresenceMismatch("IMU"))
    ));

    let mut engine = PerturbationEngine::new(sensor_config(7)).expect("engine");
    let mut missing_pressure = SimSensorPacket::new(12);
    missing_pressure.presence_mask = 1 << 10;
    assert!(matches!(
        engine.apply_sensor(12, &mut missing_pressure),
        Err(PerturbationError::SensorPresenceMismatch("pressure"))
    ));
}

#[test]
fn a_failed_sensor_application_does_not_modify_the_packet() {
    let mut engine = PerturbationEngine::new(sensor_config(7)).expect("engine");
    let mut sample = packet();
    sample
        .baro
        .as_mut()
        .expect("barometer")
        .differential_pressure_pa = Some(f32::NAN);
    let original_acceleration = sample.imu.expect("IMU").accel;

    assert!(matches!(
        engine.apply_sensor(0, &mut sample),
        Err(PerturbationError::NonFiniteSensor(
            SensorLane::DifferentialPressure
        ))
    ));
    assert_eq!(sample.imu.expect("IMU").accel, original_acceleration);
    assert!(sample
        .baro
        .expect("barometer")
        .differential_pressure_pa
        .expect("differential pressure")
        .is_nan());
    assert_eq!(
        engine.apply_sensor(0, &mut packet()),
        Err(PerturbationError::Quarantined)
    );
}

fn complete_reply(engine: &mut PerturbationEngine, sequence: u64) {
    let mut command = command(0.5);
    engine
        .apply_actuator(sequence, &mut command, ActuatorEligibility::Eligible, 0)
        .expect("actuator decision");
    engine
        .complete_actuator_send(sequence, true)
        .expect("send completion");
}
