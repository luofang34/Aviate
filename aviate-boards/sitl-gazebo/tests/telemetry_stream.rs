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
//! * The stream states the declination the estimator applies: the
//!   model value at the vehicle's GNSS fix.
//! * With a declared world origin, the stream states the receiver's fix
//!   (GPS_RAW_INT) and the estimate's global position
//!   (GLOBAL_POSITION_INT) where the vehicle is.
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
    aviate_estimate_quality, aviate_geomagnetic_source, aviate_state_valid_flags, parse_mavlink,
    serialize_mavlink, AviateEstimatorStatus, AviateGeomagneticReference, GlobalPositionInt,
    GpsRawInt, MavMessage,
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
    geomagnetic: Vec<AviateGeomagneticReference>,
    gps_raw: Vec<GpsRawInt>,
    global: Vec<GlobalPositionInt>,
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
                Ok((MavMessage::AviateGeomagneticReference(r), _, _)) => into.geomagnetic.push(r),
                Ok((MavMessage::GpsRawInt(r), _, _)) => into.gps_raw.push(r),
                Ok((MavMessage::GlobalPositionInt(g), _, _)) => into.global.push(g),
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
            ..aviate_hal_xil::SimBaroData::default()
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

#[test]
fn estimator_status_authorizes_without_arming() {
    let mut board = make_board(1);
    let (consumer, endpoint) = bind_consumer();
    let cfg = app_config_with_endpoint(&endpoint);
    board.init_telemetry(&cfg, 1_000);
    assert!(board.telemetry_enabled());

    let authorized = |s: &AviateEstimatorStatus| {
        s.quality == aviate_estimate_quality::GOOD
            && s.valid_flags & aviate_state_valid_flags::ATTITUDE != 0
            && s.valid_flags & aviate_state_valid_flags::POSITION != 0
            && s.valid_flags & aviate_state_valid_flags::VELOCITY != 0
    };

    // Feed real still-vehicle sensor packets and step until the WIRE
    // (not an internal probe) reports an authorized estimate — with
    // NO arm and NO commander: the estimator observes from boot
    // (#277), so a bench vehicle authorizes POSITION/VELOCITY before
    // the first ARM, which is exactly what a pose-gated GCS needs to
    // send that ARM. Bounded: 30 simulated seconds of 1 kHz cycles
    // is far beyond the init state machine plus the EKF's
    // aiding-freshness windows.
    let mut seen = Drained::default();
    let mut converged = false;
    for i in 1..=30_000u64 {
        board
            .transport_mut()
            .feed_sensor_packet(&still_packet(i * 1_000));
        let cmd = board.step();
        // Disarmed the whole time: the forced-safe actuator pattern
        // must hold on every cycle while the estimator converges.
        assert!(
            cmd.outputs.iter().all(|o| o.0 == 0.0),
            "disarmed board must command zero outputs while converging"
        );
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
        "estimator never published an authorized status without arming; \
         last frame: {:?}",
        seen.aviate_status.last()
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

#[test]
fn the_stream_states_the_model_declination_the_estimator_applies() {
    let mut board = make_board(3);
    let (consumer, endpoint) = bind_consumer();
    board.init_telemetry(&app_config_with_endpoint(&endpoint), 1_000);
    assert!(board.telemetry_enabled());

    let mut seen = Drained::default();
    for i in 1..=2_000u64 {
        board
            .transport_mut()
            .feed_sensor_packet(&still_packet(i * 1_000));
        board.step_sim(1_000);
        if i % 250 == 0 {
            drain(&consumer, &mut seen);
        }
    }
    let reference = seen
        .geomagnetic
        .last()
        .expect("AVIATE_GEOMAGNETIC_REFERENCE on the stream");
    assert_eq!(reference.source, aviate_geomagnetic_source::MODEL);
    // WMM2025 at the packet's 47.4 N, 8.5 E: about 3.5 degrees east.
    let declination_deg = reference.declination.to_degrees();
    assert!(
        (2.5..4.5).contains(&declination_deg),
        "declination {declination_deg:.2} deg"
    );
}

#[test]
fn a_declared_origin_places_the_fix_and_the_global_estimate_at_the_vehicle() {
    let mut board = make_board(4);
    let (consumer, endpoint) = bind_consumer();
    board.init_telemetry(&app_config_with_endpoint(&endpoint), 1_000);
    assert!(board.telemetry_enabled());
    // The still packet's receiver sits at the world origin.
    board.set_geodetic_origin(aviate_board_sitl_gazebo::GeodeticOrigin {
        latitude_deg: 47.397742,
        longitude_deg: 8.545594,
        altitude_msl_m: 488.0,
    });

    let mut seen = Drained::default();
    for i in 1..=2_000u64 {
        board
            .transport_mut()
            .feed_sensor_packet(&still_packet(i * 1_000));
        board.step_sim(1_000);
        if i % 250 == 0 {
            drain(&consumer, &mut seen);
        }
    }

    let raw = seen.gps_raw.last().expect("GPS_RAW_INT on the stream");
    assert_eq!((raw.lat, raw.lon), (473_977_420, 85_455_940));
    assert_eq!(raw.alt, 488_000);
    assert_eq!(raw.fix_type, 3);
    assert_eq!((raw.h_acc, raw.v_acc), (500, 800));
    assert_eq!(raw.satellites_visible, 12);

    // The estimate of a vehicle at the local origin is stated at the
    // world origin, to within the estimator's settling error.
    let global = seen
        .global
        .last()
        .expect("GLOBAL_POSITION_INT on the stream");
    assert!((global.lat - 473_977_420).abs() <= 100, "{global:?}");
    assert!((global.lon - 85_455_940).abs() <= 100, "{global:?}");
}
