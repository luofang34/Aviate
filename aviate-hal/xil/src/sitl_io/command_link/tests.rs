//! MAVLink command buffering and provenance tests.

#![allow(clippy::expect_used, clippy::panic)]

use super::*;
use crate::XilNetConfig;
use aviate_core::control::ControlMode;
use aviate_link::mavlink::protocol::{attitude_target_typemask, position_target_typemask};

/// Ephemeral-port config so tests never collide on a fixed port
/// (base_port 0 + SensorIn slot 0 → OS-assigned bind).
fn test_io() -> SitlIO {
    let net = XilNetConfig {
        base_port: 0,
        stride: 16,
    };
    SitlIO::new(XilConfig::for_instance_with_net(0, net)).expect("bind ephemeral UDP")
}

fn arm_msg() -> CommandLong {
    CommandLong {
        param1: 1.0,
        param2: 0.0,
        param3: 0.0,
        param4: 0.0,
        param5: 0.0,
        param6: 0.0,
        param7: 0.0,
        command: mav_cmd::COMPONENT_ARM_DISARM,
        target_system: 1,
        target_component: 1,
        confirmation: 0,
    }
}

fn attitude_msg(thrust: f32) -> SetAttitudeTarget {
    SetAttitudeTarget {
        time_boot_ms: 0,
        target_system: 1,
        target_component: 1,
        type_mask: attitude_target_typemask::BODY_ROLL_RATE_IGNORE
            | attitude_target_typemask::BODY_PITCH_RATE_IGNORE
            | attitude_target_typemask::BODY_YAW_RATE_IGNORE,
        q: [1.0, 0.0, 0.0, 0.0],
        body_roll_rate: 0.0,
        body_pitch_rate: 0.0,
        body_yaw_rate: 0.0,
        thrust,
        thrust_body: [0.0, 0.0, 0.0],
    }
}

fn position_msg(type_mask: u16, time_boot_ms: u32) -> SetPositionTargetLocalNed {
    SetPositionTargetLocalNed {
        time_boot_ms,
        target_system: 1,
        target_component: 1,
        coordinate_frame: 1,
        type_mask,
        x: 1.0,
        y: 2.0,
        z: -3.0,
        vx: 0.1,
        vy: 0.2,
        vz: -0.3,
        afx: 0.0,
        afy: 0.0,
        afz: 0.0,
        yaw: 0.0,
        yaw_rate: 0.0,
    }
}

fn raw_attitude(
    io: &mut SitlIO,
    source: std::net::SocketAddr,
    sequence: u8,
    time_boot_ms: u32,
) -> (ReceivedCommand, Vec<u8>) {
    let mut target = attitude_msg(0.5);
    target.time_boot_ms = time_boot_ms;
    let message = MavMessage::SetAttitudeTarget(target);
    let mut frame = [0_u8; 128];
    let length =
        serialize_mavlink(&message, sequence, 255, 190, &mut frame).expect("serialize setpoint");
    io.process_mavlink_data(&frame[..length], source);
    let received = io
        .recv_command_with_provenance()
        .expect("received setpoint");
    (received, frame[..length].to_vec())
}

#[test]
fn raw_setpoint_provenance_tracks_wrap_sender_and_boot_epoch() {
    use sha2::{Digest as _, Sha256};

    let mut io = test_io();
    let first_source = std::net::SocketAddr::from(([127, 0, 0, 1], 30_000));
    let second_source = std::net::SocketAddr::from(([127, 0, 0, 1], 30_001));
    let (first, first_frame) = raw_attitude(&mut io, first_source, 255, 5_000);
    let first_raw = first.provenance.expect("first provenance");
    let (wrapped, _) = raw_attitude(&mut io, first_source, 0, 5_001);
    let wrapped_raw = wrapped.provenance.expect("wrapped provenance");
    assert_eq!(first_raw.source_epoch, wrapped_raw.source_epoch);
    assert_eq!(first_raw.mavlink_frame_sequence, 255);
    assert_eq!(wrapped_raw.mavlink_frame_sequence, 0);
    assert_eq!(
        first_raw.frame_digest,
        <[u8; 32]>::from(Sha256::digest(first_frame))
    );

    let (restarted, _) = raw_attitude(&mut io, first_source, 1, 1);
    let restarted_raw = restarted.provenance.expect("restart provenance");
    assert_ne!(restarted_raw.source_epoch, wrapped_raw.source_epoch);
    let (competitor, _) = raw_attitude(&mut io, second_source, 1, 1);
    let competitor_raw = competitor.provenance.expect("competitor provenance");
    assert_ne!(competitor_raw.source_epoch, restarted_raw.source_epoch);
    assert_eq!(competitor_raw.source_endpoint, second_source);
    assert!(matches!(
        competitor.command,
        SystemCommand::FlightControl(command) if command.sequence == 1
    ));
}

/// A setpoint parsed in the same poll batch after an Arm must not
/// clobber it: both survive, discrete command first.
#[test]
fn setpoint_in_same_batch_does_not_clobber_arm() {
    let mut io = test_io();
    io.handle_command_long(arm_msg());
    io.inject_attitude_target(attitude_msg(0.4));

    assert!(matches!(io.recv_command(), Some(SystemCommand::Arm)));
    match io.recv_command() {
        Some(SystemCommand::FlightControl(cmd)) => {
            assert!((cmd.setpoint.collective_thrust.0 - 0.4).abs() < 1e-6);
        }
        other => panic!("expected buffered FlightControl, got {other:?}"),
    }
}

/// Arm parsed after a setpoint in the same batch: discrete command
/// still drains first, the setpoint is preserved behind it.
#[test]
fn arm_after_setpoint_in_same_batch_preserves_both() {
    let mut io = test_io();
    io.inject_attitude_target(attitude_msg(0.7));
    io.handle_command_long(arm_msg());

    assert!(matches!(io.recv_command(), Some(SystemCommand::Arm)));
    assert!(matches!(
        io.recv_command(),
        Some(SystemCommand::FlightControl(_))
    ));
    assert!(io.recv_command().is_none());
}

/// Setpoints remain latest-wins: only the newest survives a batch.
#[test]
fn setpoint_slot_is_latest_wins() {
    let mut io = test_io();
    io.inject_attitude_target(attitude_msg(0.2));
    io.inject_attitude_target(attitude_msg(0.9));

    match io.recv_command() {
        Some(SystemCommand::FlightControl(cmd)) => {
            assert!((cmd.setpoint.collective_thrust.0 - 0.9).abs() < 1e-6);
        }
        other => panic!("expected latest setpoint, got {other:?}"),
    }
    assert!(io.recv_command().is_none());
}

#[test]
fn partial_position_and_velocity_targets_preserve_the_retained_command() {
    let mut io = test_io();
    let mut retained = attitude_msg(0.42);
    retained.time_boot_ms = 100;
    io.inject_attitude_target(retained);
    let provenance = io.flight_cmd.as_ref().expect("retained command").provenance;

    let required_ignored = position_target_typemask::AX_IGNORE
        | position_target_typemask::AY_IGNORE
        | position_target_typemask::AZ_IGNORE
        | position_target_typemask::YAW_IGNORE
        | position_target_typemask::YAW_RATE_IGNORE;
    let partial_position = position_target_typemask::X_IGNORE
        | position_target_typemask::VX_IGNORE
        | position_target_typemask::VY_IGNORE
        | position_target_typemask::VZ_IGNORE
        | required_ignored;
    io.inject_position_target(position_msg(partial_position, 200));
    let partial_velocity = position_target_typemask::X_IGNORE
        | position_target_typemask::Y_IGNORE
        | position_target_typemask::Z_IGNORE
        | position_target_typemask::VX_IGNORE
        | required_ignored;
    io.inject_position_target(position_msg(partial_velocity, 300));

    let after = io.flight_cmd.as_ref().expect("retained command remains");
    assert_eq!(after.provenance, provenance);
    assert!(matches!(
        &after.command,
        SystemCommand::FlightControl(command)
            if command.mode == ControlMode::Attitude
                && command.setpoint.collective_thrust.0 == 0.42
    ));
}

#[test]
fn invalid_attitude_target_preserves_the_retained_command_and_provenance() {
    let mut io = test_io();
    let mut retained = attitude_msg(0.61);
    retained.time_boot_ms = 100;
    io.inject_attitude_target(retained);
    let provenance = io.flight_cmd.as_ref().expect("retained command").provenance;

    let mut invalid = attitude_msg(0.9);
    invalid.time_boot_ms = 200;
    invalid.q[0] = f32::NAN;
    io.inject_attitude_target(invalid);

    let after = io.flight_cmd.as_ref().expect("retained command remains");
    assert_eq!(after.provenance, provenance);
    assert!(matches!(
        &after.command,
        SystemCommand::FlightControl(command)
            if command.mode == ControlMode::Attitude
                && command.setpoint.collective_thrust.0 == 0.61
    ));
}
