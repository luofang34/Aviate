//! The estimate stream belongs to its CONFIGURED consumer.
//!
//! Three behaviors pinned here, all event/state-synchronized (steps
//! are driven synchronously; UDP loopback delivery completes inside
//! `send_to`, so a drained socket is a settled socket — no sleeps):
//!
//! * A fixed-endpoint consumer receives HEARTBEAT,
//!   ATTITUDE_QUATERNION, LOCAL_POSITION_NED and the Aviate
//!   estimator-status at their configured rates while a live
//!   commander is simultaneously sending — the commander must never
//!   steal the stream (the pre-fix path re-pointed the telemetry
//!   socket at whichever address commanded last, every cycle).
//! * Estimator-status content is the estimator's REAL validity:
//!   pre-convergence frames say unusable/no-flags; feeding real
//!   sensor packets until the EKF converges flips the wire to
//!   authorized. Nothing is hardcoded valid.
//! * Invalid telemetry config disables the stream loudly and changes
//!   nothing else; a subsequent valid config still enables it.
//!
//! Tests run concurrently in one process: each takes its own XIL
//! instance on a pid-derived base port so parallel CI shards and
//! sibling tests never contend for a socket.

#![allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]

use std::net::UdpSocket;

use aviate_app_sitl_gazebo_x500_kernel::build_x500_kernel;
use aviate_board_sitl_gazebo::GazeboSitlBoard;
use aviate_hal_xil::{SimGnssData, SimGnssFix, SimImuData, SimSensorPacket, SitlConfig};
use aviate_link::mavlink::protocol::Heartbeat;
use aviate_link::mavlink::{
    aviate_estimate_quality, aviate_state_valid_flags, parse_mavlink, serialize_mavlink,
    AviateEstimatorStatus, MavMessage,
};

/// Pid-derived base port: distinct per test process, clear of the
/// production default (20000) and of the ephemeral range floor.
fn test_net() -> aviate_hal_xil::XilNetConfig {
    aviate_hal_xil::XilNetConfig {
        base_port: 22000 + (std::process::id() % 700) as u16 * 16,
        stride: 16,
    }
}

fn make_board(
    instance: u8,
) -> GazeboSitlBoard<
    aviate_core::control::multirotor::MultirotorController,
    aviate_core::mixer::QuadXMixerX500,
> {
    let kernel = build_x500_kernel().expect("x500 kernel builds");
    let config = SitlConfig::for_instance_with_net(instance, test_net());
    GazeboSitlBoard::with_config(kernel, config).expect("bind XIL instance ports")
}

/// App config TOML with the telemetry endpoint substituted — the
/// same parse + validate path the real binary runs.
fn app_config_with_endpoint(endpoint: &str) -> aviate_config::AppConfig {
    let toml = format!(
        r#"
[app]
id = "telemetry-stream-test"
board = "sitl-gazebo"
airframe = "x500"
env = "sitl"

[telemetry]
frame_size = 280
queue_len = 32
heartbeat_hz = 1
attitude_hz = 10
position_hz = 4
estimator_status_hz = 4

[[transports]]
protocol = "mavlink"
port = "udp"
roles = ["telemetry"]
endpoint = "{endpoint}"
"#
    );
    let cfg = aviate_config::from_toml_str(&toml).expect("test config parses");
    aviate_config::validate(&cfg).expect("test config validates");
    cfg
}

fn bind_consumer() -> (UdpSocket, String) {
    let sock = UdpSocket::bind("127.0.0.1:0").expect("bind consumer socket");
    sock.set_nonblocking(true).expect("consumer nonblocking");
    let addr = sock.local_addr().expect("consumer addr").to_string();
    (sock, addr)
}

/// Per-message-id counts plus every estimator-status payload seen.
#[derive(Default)]
struct Drained {
    heartbeat: usize,
    attitude: usize,
    position: usize,
    aviate_status: Vec<AviateEstimatorStatus>,
    standard_status: usize,
}

fn drain(consumer: &UdpSocket, into: &mut Drained) {
    let mut buf = [0u8; 512];
    loop {
        match consumer.recv_from(&mut buf) {
            Ok((len, _)) => match parse_mavlink(&buf[..len]) {
                Ok((MavMessage::Heartbeat(_), _, _)) => into.heartbeat += 1,
                Ok((MavMessage::AttitudeQuaternion(_), _, _)) => into.attitude += 1,
                Ok((MavMessage::LocalPositionNed(_), _, _)) => into.position += 1,
                Ok((MavMessage::AviateEstimatorStatus(s), _, _)) => into.aviate_status.push(s),
                Ok((MavMessage::EstimatorStatus(_), _, _)) => into.standard_status += 1,
                Ok(_) => {}
                Err(e) => panic!("consumer received an unparseable frame: {e:?}"),
            },
            Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => return,
            Err(e) => panic!("consumer recv failed: {e}"),
        }
    }
}

/// One MAVLink heartbeat datagram from a "commander" socket to the
/// FC's command port — exactly what makes SitlIO learn a GCS address.
fn command_from(commander: &UdpSocket, fc_port: u16) {
    let hb = Heartbeat {
        mav_type: 6, // GCS
        autopilot: 8,
        base_mode: 0,
        custom_mode: 0,
        system_status: 4,
        mavlink_version: 3,
    };
    let mut buf = [0u8; 64];
    let mut seq = 0u8;
    let len = serialize_mavlink(&MavMessage::Heartbeat(hb), seq, 255, 190, &mut buf)
        .expect("serialize heartbeat");
    seq = seq.wrapping_add(1);
    let _ = seq;
    commander
        .send_to(&buf[..len], ("127.0.0.1", fc_port))
        .expect("commander send");
}

/// A still, grounded vehicle's sensor packet at simulated time `t_us`.
fn still_packet(t_us: u64) -> SimSensorPacket {
    SimSensorPacket::new(t_us)
        .with_imu(SimImuData {
            // NED body at rest: specific force opposes gravity.
            accel: [0.0, 0.0, -9.81],
            gyro: [0.0, 0.0, 0.0],
            temperature: Some(20.0),
        })
        .with_baro(aviate_hal_xil::SimBaroData {
            pressure_pa: 101_325.0,
            temperature_c: 20.0,
        })
        .with_mag(aviate_hal_xil::SimMagData {
            // Plausible mid-latitude NED field, microtesla.
            field_ut: [21.0, 0.0, 43.0],
        })
        .with_gnss(SimGnssData {
            lat_deg: 47.397742,
            lon_deg: 8.545594,
            alt_m: 488.0,
            position_ned: [0.0, 0.0, 0.0],
            vel_ned: [0.0, 0.0, 0.0],
            fix: SimGnssFix::ThreeD,
            h_acc: 0.5,
            v_acc: 0.8,
            satellites: 12,
        })
}

#[test]
fn fixed_consumer_keeps_the_stream_while_a_commander_is_live() {
    let mut board = make_board(0);
    let (consumer, endpoint) = bind_consumer();
    let cfg = app_config_with_endpoint(&endpoint);
    board.init_telemetry(&cfg, 1_000);
    assert!(board.telemetry_enabled(), "valid config must enable");

    let commander = UdpSocket::bind("127.0.0.1:0").expect("bind commander");
    // Slot 0 of the instance's port block is the MAVLink/GCS port.
    let fc_port = SitlConfig::for_instance_with_net(0, test_net()).sensor_port();

    // Commander speaks BEFORE the stream starts and again mid-run:
    // the address is learned, and the stream must not follow it.
    command_from(&commander, fc_port);

    let mut seen = Drained::default();
    for i in 1..=1_000u32 {
        if i == 500 {
            command_from(&commander, fc_port);
        }
        board.step();
        // Drain as we go so the consumer's socket buffer can never
        // become the thing this test actually measures.
        if i % 100 == 0 {
            drain(&consumer, &mut seen);
        }
    }
    drain(&consumer, &mut seen);

    // The commander really was learned (its heartbeats reached the
    // command port) — the precondition of the old hijack.
    assert_eq!(
        board.transport_mut().gcs_addr(),
        Some(commander.local_addr().expect("commander addr")),
        "commander address must be learned for this test to prove anything"
    );

    // 1000 iterations at 1 kHz with 1/10/4/4 Hz rates: exact counts,
    // all at the configured endpoint.
    assert_eq!(seen.heartbeat, 1, "heartbeat_hz = 1");
    assert_eq!(seen.attitude, 10, "attitude_hz = 10");
    assert_eq!(seen.position, 4, "position_hz = 4");
    // The estimator-status pair rides with every group emission:
    // iterations divisible by 100 (attitude) or 250 (position/status).
    assert_eq!(seen.aviate_status.len(), 12);
    assert_eq!(seen.standard_status, 12);

    // No sensors were fed: the estimator never left Unusable, and the
    // wire must say so — validity is real, never hardcoded.
    for status in &seen.aviate_status {
        assert_eq!(status.quality, aviate_estimate_quality::UNUSABLE);
        assert_eq!(status.valid_flags, 0);
    }
}

/// One MAVLink ARM command from the commander socket. Arming matters
/// here because the kernel's estimator only OBSERVES while armed —
/// pre-arm, `Degraded` (attitude-only) is the designed ceiling and
/// GNSS/baro aiding cannot fuse (see `kernel_logic::init_step` docs).
fn arm_from(commander: &UdpSocket, fc_port: u16) {
    let cmd = aviate_link::mavlink::protocol::CommandLong {
        param1: 1.0,
        param2: 0.0,
        param3: 0.0,
        param4: 0.0,
        param5: 0.0,
        param6: 0.0,
        param7: 0.0,
        command: aviate_link::mavlink::mav_cmd::COMPONENT_ARM_DISARM,
        target_system: 1,
        target_component: 1,
        confirmation: 0,
    };
    let mut buf = [0u8; 64];
    let len = serialize_mavlink(&MavMessage::CommandLong(cmd), 0, 255, 190, &mut buf)
        .expect("serialize arm");
    commander
        .send_to(&buf[..len], ("127.0.0.1", fc_port))
        .expect("commander send arm");
}

/// A low-thrust level attitude setpoint — keeps the armed vehicle's
/// command age fresh, exactly like a GCS streaming sticks.
fn setpoint_from(commander: &UdpSocket, fc_port: u16, t_ms: u32) {
    let tgt = aviate_link::mavlink::protocol::SetAttitudeTarget {
        time_boot_ms: t_ms,
        target_system: 1,
        target_component: 1,
        type_mask: 0,
        q: [1.0, 0.0, 0.0, 0.0],
        body_roll_rate: 0.0,
        body_pitch_rate: 0.0,
        body_yaw_rate: 0.0,
        thrust: 0.0,
        thrust_body: [0.0; 3],
    };
    let mut buf = [0u8; 96];
    let len = serialize_mavlink(&MavMessage::SetAttitudeTarget(tgt), 0, 255, 190, &mut buf)
        .expect("serialize setpoint");
    commander
        .send_to(&buf[..len], ("127.0.0.1", fc_port))
        .expect("commander send setpoint");
}

#[test]
fn estimator_status_flips_to_authorized_when_the_ekf_converges() {
    let mut board = make_board(1);
    let (consumer, endpoint) = bind_consumer();
    let cfg = app_config_with_endpoint(&endpoint);
    board.init_telemetry(&cfg, 1_000);
    assert!(board.telemetry_enabled());

    let commander = UdpSocket::bind("127.0.0.1:0").expect("bind commander");
    let fc_port = SitlConfig::for_instance_with_net(1, test_net()).sensor_port();

    let authorized = |s: &AviateEstimatorStatus| {
        s.quality == aviate_estimate_quality::GOOD
            && s.valid_flags & aviate_state_valid_flags::ATTITUDE != 0
            && s.valid_flags & aviate_state_valid_flags::POSITION != 0
            && s.valid_flags & aviate_state_valid_flags::VELOCITY != 0
    };

    // Feed real still-vehicle sensor packets, arm through the wire,
    // stream setpoints like a GCS, and step until the WIRE (not an
    // internal probe) reports an authorized estimate. Bounded: 30
    // simulated seconds of 1 kHz cycles is far beyond the init state
    // machine plus the EKF's aiding-freshness windows.
    let mut seen = Drained::default();
    let mut converged = false;
    for i in 1..=30_000u64 {
        board
            .transport_mut()
            .feed_sensor_packet(&still_packet(i * 1_000));
        // Re-request ARM until the init state machine accepts it;
        // keep the command channel fresh at a GCS-like 10 Hz.
        if i % 200 == 0 {
            arm_from(&commander, fc_port);
        }
        if i % 100 == 0 {
            setpoint_from(&commander, fc_port, i as u32);
        }
        board.step();
        if i % 250 == 0 {
            drain(&consumer, &mut seen);
            if seen.aviate_status.iter().any(authorized) {
                converged = true;
                break;
            }
        }
    }
    assert!(
        converged,
        "estimator never published an authorized status; last frame: {:?}",
        seen.aviate_status.last()
    );

    // The pre-convergence prefix must have been honest too: the very
    // first status frame cannot already claim authorization.
    let first = seen.aviate_status.first().expect("at least one status");
    assert!(
        !authorized(first),
        "first status frame already authorized — validity looks hardcoded"
    );
}

#[test]
fn invalid_telemetry_config_disables_loudly_and_changes_nothing_else() {
    let mut board = make_board(2);

    // Malformed endpoint: refused, disabled.
    let bad_endpoint = app_config_with_endpoint("not-a-socket-addr");
    board.init_telemetry(&bad_endpoint, 1_000);
    assert!(!board.telemetry_enabled(), "bad endpoint must disable");

    // Zero rate: refused by validate_telemetry_config, disabled.
    let mut zero_rate = app_config_with_endpoint("127.0.0.1:14550");
    if let Some(t) = zero_rate.telemetry.as_mut() {
        t.attitude_hz = 0;
    }
    board.init_telemetry(&zero_rate, 1_000);
    assert!(!board.telemetry_enabled(), "zero rate must disable");

    // Missing [telemetry] section: disabled.
    let mut missing = app_config_with_endpoint("127.0.0.1:14550");
    missing.telemetry = None;
    board.init_telemetry(&missing, 1_000);
    assert!(!board.telemetry_enabled(), "missing section must disable");

    // "Changes nothing else": the control loop still steps and
    // produces sane (disarmed, zeroed) actuator output.
    let cmd = board.step();
    assert!(
        cmd.outputs.iter().all(|o| o.0 == 0.0),
        "disarmed board must command zero outputs"
    );

    // And a refusal is not a wedge: a valid config still enables.
    let (_consumer, endpoint) = bind_consumer();
    let good = app_config_with_endpoint(&endpoint);
    board.init_telemetry(&good, 1_000);
    assert!(board.telemetry_enabled(), "valid config after refusals");
}
