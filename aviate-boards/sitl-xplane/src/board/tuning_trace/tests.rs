//! Fake-runner tests for the tuning trace transport.

#![allow(clippy::expect_used)]

use std::net::TcpListener;
use std::thread;

use aviate_core::state::StateEstimate;

use super::*;
use crate::{XPlaneConstraintFlags, XPlaneControlObservation};

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
    }
}

fn observation() -> XPlaneControlObservation {
    XPlaneControlObservation {
        timestamp_us: 10_000,
        imu: None,
        pre_wire_force_lanes: [0.2; 4],
        applied_force_lanes: [0.2; 4],
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
    let command = aviate_runtime::default_command();
    publisher.publish(
        observation(),
        None,
        &command,
        &StateEstimate::default(),
        false,
    );
    assert!(publisher.failure().is_none());
    assert!(publisher.is_ready());
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
}
