//! Fake-runner tests for the tuning trace transport.

#![allow(clippy::expect_used)]

use std::net::TcpListener;
use std::thread;

use aviate_core::control::CommandSource;
use aviate_core::state::StateEstimate;
use aviate_hal_xil::{MavlinkCommandFamily, MavlinkCommandProvenance};

use super::*;
use crate::{XPlaneConstraintFlags, XPlaneControlObservation, XPlaneHoverInitialization};

fn digest(byte: char) -> String {
    core::iter::repeat_n(byte, 64).collect()
}

fn identity() -> XPlaneTuningTraceIdentity {
    XPlaneTuningTraceIdentity {
        run_manifest_digest: digest('1'),
        build_identity: digest('2'),
        source_identity: digest('3'),
        lock_identity: digest('4'),
        simulator_model_digest: digest('5'),
        runtime_handshake_digest: digest('6'),
        candidate_digest: None,
        candidate_lineage_digest: None,
        plant_artifact_digest: None,
        algorithm_identity_hash: "1234567890abcdef".to_owned(),
        kernel_config_hash: "fedcba0987654321".to_owned(),
        condition_artifact_path: None,
        condition_artifact_sha256: None,
        condition_digest: None,
        condition_run_seed: None,
        condition_required_capabilities: None,
        hover_baseline_force_bits: 0.43_f32.to_bits(),
        hover_effective_force_bits: 0.43_f32.to_bits(),
        hover_scale_basis_points: 10_000,
        hover_estimator_mode: TuningHoverEstimatorMode::Disabled,
        hover_kernel_config_hash: "fedcba0987654321".to_owned(),
    }
}

fn observation() -> XPlaneControlObservation {
    XPlaneControlObservation {
        sample_sequence: 41,
        timestamp_us: 10_000,
        imu: None,
        sensor_application: None,
        actuator_application: None,
        lane_injection: [0.0; 4],
        fix_altitude_m: Some(3.0),
        sample_dt_sec: 0.01,
        pre_wire_force_lanes: [0.2; 4],
        applied_force_lanes: [0.2; 4],
        sent_lanes: [0.2; 4],
        send: crate::XPlaneSendEvidence {
            reply_attempted: true,
            reply_succeeded: true,
            echoed_timestamp_us: 10_000,
            lockstep: true,
        },
        hover_initialization: XPlaneHoverInitialization {
            baseline_force_bits: 0.43_f32.to_bits(),
            effective_force_bits: 0.43_f32.to_bits(),
            scale_basis_points: 10_000,
            kernel_config_hash: 7,
        },
        constraint_flags: XPlaneConstraintFlags::default(),
    }
}

#[test]
fn fake_runner_accepts_identity_and_each_exact_sequence() {
    let listener = TcpListener::bind("127.0.0.1:0").expect("listener");
    let endpoint = listener.local_addr().expect("address");
    let server = thread::spawn(move || {
        let (mut stream, _) = listener.accept().expect("accept");
        let handshake: TuningHandshake = read_frame(&mut stream).expect("handshake");
        assert_eq!(handshake.frame_type, TuningFrameType::AviateTuningHandshake);
        assert_eq!(handshake.schema_version, 3);
        assert!(handshake.condition_artifact_path.is_none());
        assert_eq!(handshake.hover_baseline_force_bits, 0.43_f32.to_bits());
        assert_eq!(handshake.hover_effective_force_bits, 0.43_f32.to_bits());
        assert_eq!(
            handshake.hover_estimator_mode,
            TuningHoverEstimatorMode::Disabled
        );
        assert_eq!(handshake.hover_kernel_config_hash, "fedcba0987654321");
        write_frame(
            &mut stream,
            &TuningReady {
                frame_type: TuningFrameType::AviateTuningReady,
                schema_version: TUNING_TRACE_SCHEMA_VERSION,
                run_manifest_digest: handshake.run_manifest_digest.clone(),
            },
        )
        .expect("ready");
        let frame: TuningControlObservation = read_frame(&mut stream).expect("observation");
        assert_eq!(frame.sequence, 0);
        assert_eq!(frame.simulator_timestamp_us, 10_000);
        assert_eq!(frame.global_sample_sequence, 41);
        assert_eq!(frame.lane_injection_bits, [0; 4]);
        assert_eq!(frame.fix_altitude_m_bits, Some(3.0_f32.to_bits()));
        assert_eq!(frame.sample_dt_sec_bits, 0.01_f32.to_bits());
        assert_eq!(frame.pre_wire_force_lane_bits, [0.2_f32.to_bits(); 4]);
        assert_eq!(frame.applied_force_lane_bits, [0.2_f32.to_bits(); 4]);
        assert_eq!(frame.sent_lane_bits, [0.2_f32.to_bits(); 4]);
        assert!(frame.send.reply_succeeded);
        assert_eq!(frame.hover_initialization.kernel_config_hash, 7);
        assert_eq!(
            frame.requested_command.as_ref().map(|value| value.sequence),
            Some(255)
        );
        let raw = frame.command_provenance.expect("raw command provenance");
        assert_eq!(
            raw.source_endpoint,
            "127.0.0.1:30000".parse().expect("source")
        );
        assert_eq!(raw.source_epoch, 41);
        assert_eq!(raw.mavlink_system_id, 255);
        assert_eq!(raw.mavlink_component_id, 190);
        assert_eq!(raw.mavlink_frame_sequence, 255);
        assert_eq!(raw.time_boot_ms, 5_000);
        assert_eq!(
            raw.command_family,
            MavlinkCommandFamily::AttitudeTarget.into()
        );
        assert_eq!(raw.frame_digest, [9; 32]);
        write_frame(
            &mut stream,
            &TuningObservationAck {
                frame_type: TuningFrameType::AviateTuningObservationAck,
                schema_version: TUNING_TRACE_SCHEMA_VERSION,
                run_manifest_digest: handshake.run_manifest_digest,
                sequence: frame.sequence,
            },
        )
        .expect("ack");
    });
    let config = XPlaneTuningTraceConfig::new(endpoint, identity()).expect("config");
    let mut publisher = TuningTracePublisher::connect(config).expect("publisher");
    let mut command = aviate_runtime::default_command();
    command.source = CommandSource::Gcs;
    command.sequence = 255;
    let provenance = MavlinkCommandProvenance {
        source_endpoint: "127.0.0.1:30000".parse().expect("source"),
        source_epoch: 41,
        mavlink_system_id: 255,
        mavlink_component_id: 190,
        mavlink_frame_sequence: 255,
        time_boot_ms: 5_000,
        command_family: MavlinkCommandFamily::AttitudeTarget,
        frame_digest: [9; 32],
    };
    publisher.publish(
        observation(),
        Some(&command),
        Some(provenance),
        &command,
        &StateEstimate::default(),
        false,
    );
    assert!(
        publisher.failure().is_none(),
        "the publisher recorded a failure: {:?}",
        publisher.failure()
    );
    assert!(publisher.is_ready());
    server.join().expect("server join");
}

#[test]
fn accepted_maximum_observation_sequence_is_never_wrapped() {
    let listener = TcpListener::bind("127.0.0.1:0").expect("listener");
    let endpoint = listener.local_addr().expect("address");
    let server = thread::spawn(move || {
        let (mut stream, _) = listener.accept().expect("accept");
        let handshake: TuningHandshake = read_frame(&mut stream).expect("handshake");
        write_frame(
            &mut stream,
            &TuningReady {
                frame_type: TuningFrameType::AviateTuningReady,
                schema_version: TUNING_TRACE_SCHEMA_VERSION,
                run_manifest_digest: handshake.run_manifest_digest.clone(),
            },
        )
        .expect("ready");
        let frame: TuningControlObservation = read_frame(&mut stream).expect("observation");
        assert_eq!(frame.sequence, u64::MAX);
        write_frame(
            &mut stream,
            &TuningObservationAck {
                frame_type: TuningFrameType::AviateTuningObservationAck,
                schema_version: TUNING_TRACE_SCHEMA_VERSION,
                run_manifest_digest: handshake.run_manifest_digest,
                sequence: frame.sequence,
            },
        )
        .expect("ack");
        assert!(matches!(
            read_frame::<TuningControlObservation>(&mut stream),
            Err(TuningTraceError::Read(_))
        ));
    });
    let config = XPlaneTuningTraceConfig::new(endpoint, identity()).expect("config");
    let mut publisher = TuningTracePublisher::connect(config).expect("publisher");
    publisher.sequence = u64::MAX;
    let command = aviate_runtime::default_command();
    publisher.publish(
        observation(),
        None,
        None,
        &command,
        &StateEstimate::default(),
        false,
    );
    assert!(
        publisher.failure().is_none(),
        "the publisher recorded a failure: {:?}",
        publisher.failure()
    );

    publisher.publish(
        observation(),
        None,
        None,
        &command,
        &StateEstimate::default(),
        false,
    );
    assert!(matches!(
        publisher.failure(),
        Some(TuningTraceError::ObservationSequenceExhausted)
    ));
    assert!(!publisher.is_ready());
    server.join().expect("server join");
}

#[test]
fn wrong_ack_sequence_permanently_fails_the_transport() {
    let listener = TcpListener::bind("127.0.0.1:0").expect("listener");
    let endpoint = listener.local_addr().expect("address");
    let server = thread::spawn(move || {
        let (mut stream, _) = listener.accept().expect("accept");
        let handshake: TuningHandshake = read_frame(&mut stream).expect("handshake");
        write_frame(
            &mut stream,
            &TuningReady {
                frame_type: TuningFrameType::AviateTuningReady,
                schema_version: TUNING_TRACE_SCHEMA_VERSION,
                run_manifest_digest: handshake.run_manifest_digest.clone(),
            },
        )
        .expect("ready");
        let frame: TuningControlObservation = read_frame(&mut stream).expect("observation");
        write_frame(
            &mut stream,
            &TuningObservationAck {
                frame_type: TuningFrameType::AviateTuningObservationAck,
                schema_version: TUNING_TRACE_SCHEMA_VERSION,
                run_manifest_digest: handshake.run_manifest_digest,
                sequence: frame.sequence.wrapping_add(1),
            },
        )
        .expect("bad ack");
    });
    let config = XPlaneTuningTraceConfig::new(endpoint, identity()).expect("config");
    let mut publisher = TuningTracePublisher::connect(config).expect("publisher");
    publisher.publish(
        observation(),
        None,
        None,
        &aviate_runtime::default_command(),
        &StateEstimate::default(),
        false,
    );
    assert!(matches!(
        publisher.failure(),
        Some(TuningTraceError::ObservationAckSequenceMismatch {
            expected: 0,
            received: 1
        })
    ));
    assert!(!publisher.is_ready());
    server.join().expect("server join");
}

#[test]
fn disconnect_before_ack_permanently_fails_the_transport() {
    let listener = TcpListener::bind("127.0.0.1:0").expect("listener");
    let endpoint = listener.local_addr().expect("address");
    let server = thread::spawn(move || {
        let (mut stream, _) = listener.accept().expect("accept");
        let handshake: TuningHandshake = read_frame(&mut stream).expect("handshake");
        write_frame(
            &mut stream,
            &TuningReady {
                frame_type: TuningFrameType::AviateTuningReady,
                schema_version: TUNING_TRACE_SCHEMA_VERSION,
                run_manifest_digest: handshake.run_manifest_digest,
            },
        )
        .expect("ready");
        let _: TuningControlObservation = read_frame(&mut stream).expect("observation");
    });
    let config = XPlaneTuningTraceConfig::new(endpoint, identity()).expect("config");
    let mut publisher = TuningTracePublisher::connect(config).expect("publisher");
    publisher.publish(
        observation(),
        None,
        None,
        &aviate_runtime::default_command(),
        &StateEstimate::default(),
        false,
    );
    assert!(matches!(
        publisher.failure(),
        Some(TuningTraceError::Read(_))
    ));
    assert!(!publisher.is_ready());
    server.join().expect("server join");
}

#[test]
fn non_loopback_and_partial_candidate_identities_fail_closed() {
    assert!(matches!(
        XPlaneTuningTraceConfig::new("192.0.2.1:80".parse().expect("address"), identity()),
        Err(TuningTraceError::NonLoopbackEndpoint(_))
    ));
    let mut partial = identity();
    partial.candidate_digest = Some(digest('a'));
    assert!(matches!(
        XPlaneTuningTraceConfig::new("127.0.0.1:1".parse().expect("address"), partial),
        Err(TuningTraceError::InvalidIdentity("candidate identity"))
    ));
}

#[test]
fn absent_candidate_digests_are_omitted_from_json() {
    let handshake = handshake_from_identity(identity());
    let text = serde_json::to_string(&handshake).expect("json");
    assert!(!text.contains("candidate_digest"));
    assert!(!text.contains("candidate_lineage_digest"));
    assert!(!text.contains("plant_artifact_digest"));
    assert!(!text.contains("condition_artifact_path"));
    assert!(!text.contains("condition_required_capabilities"));
}

#[test]
fn complete_condition_identity_is_sent_exactly() {
    let mut value = identity();
    bind_condition(&mut value);
    let handshake = handshake_from_identity(value);
    assert_eq!(
        handshake.condition_artifact_path.as_deref(),
        Some("/tmp/condition.json")
    );
    assert_eq!(
        handshake.condition_artifact_sha256.as_deref(),
        Some(digest('a').as_str())
    );
    assert_eq!(
        handshake.condition_digest.as_deref(),
        Some(digest('b').as_str())
    );
    assert_eq!(handshake.condition_run_seed, Some(77));
    assert_eq!(
        handshake.condition_required_capabilities,
        Some(vec![
            TuningPerturbationCapability::ActuatorAuthority,
            TuningPerturbationCapability::SensorPerturbation,
        ])
    );
}

#[test]
fn partial_or_unsorted_condition_identity_fails_closed() {
    let mut partial = identity();
    partial.condition_artifact_path = Some("/tmp/condition.json".to_owned());
    assert!(matches!(
        XPlaneTuningTraceConfig::new("127.0.0.1:1".parse().expect("address"), partial),
        Err(TuningTraceError::InvalidIdentity("condition identity"))
    ));

    let mut unsorted = identity();
    bind_condition(&mut unsorted);
    unsorted.condition_required_capabilities = Some(vec![
        TuningPerturbationCapability::SensorPerturbation,
        TuningPerturbationCapability::ActuatorAuthority,
    ]);
    assert!(matches!(
        XPlaneTuningTraceConfig::new("127.0.0.1:1".parse().expect("address"), unsorted),
        Err(TuningTraceError::InvalidIdentity("condition capabilities"))
    ));
}

#[test]
fn duplicate_or_unknown_condition_capability_fails_closed() {
    let mut duplicate = identity();
    bind_condition(&mut duplicate);
    duplicate.condition_required_capabilities = Some(vec![
        TuningPerturbationCapability::ActuatorAuthority,
        TuningPerturbationCapability::ActuatorAuthority,
    ]);
    assert!(
        XPlaneTuningTraceConfig::new("127.0.0.1:1".parse().expect("address"), duplicate).is_err()
    );

    let mut value = identity();
    bind_condition(&mut value);
    let text = serde_json::to_string(&handshake_from_identity(value))
        .expect("condition handshake")
        .replace("actuator_authority", "unknown_capability");
    assert!(serde_json::from_str::<TuningHandshake>(&text).is_err());
}

#[test]
fn hover_identity_must_match_exact_disabled_initialization() {
    let endpoint = "127.0.0.1:1".parse().expect("address");
    let mut scaled = identity();
    scaled.hover_scale_basis_points = 8_000;
    assert!(matches!(
        XPlaneTuningTraceConfig::new(endpoint, scaled),
        Err(TuningTraceError::InvalidIdentity("hover initialization"))
    ));

    let mut online = identity();
    online.hover_estimator_mode = TuningHoverEstimatorMode::Online;
    assert!(matches!(
        XPlaneTuningTraceConfig::new(endpoint, online),
        Err(TuningTraceError::InvalidIdentity("hover initialization"))
    ));
}

fn bind_condition(value: &mut XPlaneTuningTraceIdentity) {
    value.condition_artifact_path = Some("/tmp/condition.json".to_owned());
    value.condition_artifact_sha256 = Some(digest('a'));
    value.condition_digest = Some(digest('b'));
    value.condition_run_seed = Some(77);
    value.condition_required_capabilities = Some(vec![
        TuningPerturbationCapability::ActuatorAuthority,
        TuningPerturbationCapability::SensorPerturbation,
    ]);
}
