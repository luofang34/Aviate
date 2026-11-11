//! The direct board flies with no ground-station link: an embedded host
//! sends commands as MAVLink bytes and never a `HEARTBEAT` or a stream
//! request. Readiness, arming, command acceptance and telemetry rates
//! must not depend on one, while the setpoint-refresh rule still holds.

#![allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]

use aviate_airframe_fixed_wing::c172;
use aviate_core::control::{CommandSource, ControlLawV1, ControlMode};
use aviate_core::fault::FaultFlags;
use aviate_link::mavlink::protocol::{
    position_target_typemask, CommandLong, Heartbeat, SetPositionTargetLocalNed,
};
use aviate_link::mavlink::{
    mav_cmd, mav_result, parse_mavlink_frame, serialize_mavlink, MavMessage, MavModeFlag, MavType,
};
use aviate_xil_direct::{DirectNoiseTier, DirectSensorRates, DirectTelemetryRates};

#[path = "plant/flight.rs"]
#[allow(dead_code)]
mod flight;
use flight::{Flight, PlantModel, DT_S};

/// An embedded host that is not a ground station: neither system 255
/// nor a GCS component, so nothing can key off the conventional address.
const HOST: (u8, u8) = (42, 191);
/// `MAV_AUTOPILOT_INVALID`: the autopilot field of a ground station.
const MAV_AUTOPILOT_INVALID: u8 = 8;
/// `MAV_CMD_SET_MESSAGE_INTERVAL`.
const SET_MESSAGE_INTERVAL: u16 = 511;
/// `MAVLINK_MSG_ID_ATTITUDE_QUATERNION`.
const ATTITUDE_QUATERNION_ID: f32 = 31.0;
const CRUISE_MPS: f32 = 55.0;
/// 100 ms between setpoints, inside the 500 ms refresh rule.
const SETPOINT_REFRESH_STEPS: u64 = 25;
const STEPS_PER_SECOND: u64 = 250;

/// Rates that are whole multiples of the 4 ms step, so the expected
/// count of each stream over a window is exact.
const RATES: DirectTelemetryRates = DirectTelemetryRates {
    heartbeat_hz: 2,
    sys_status_hz: 1,
    attitude_hz: 25,
    position_hz: 5,
    estimator_status_hz: 5,
    vfr_hud_hz: 4,
};

fn frame(message: &MavMessage, seq: u8) -> Vec<u8> {
    let mut buf = [0_u8; 280];
    let len = serialize_mavlink(message, seq, HOST.0, HOST.1, &mut buf).expect("serialize");
    buf[..len].to_vec()
}

fn arm() -> MavMessage {
    MavMessage::CommandLong(CommandLong {
        param1: 1.0,
        command: mav_cmd::COMPONENT_ARM_DISARM,
        target_system: 1,
        target_component: 1,
        ..CommandLong::default()
    })
}

/// Level cruise north at the cruise airspeed with no climb.
fn cruise_setpoint() -> MavMessage {
    let mask = position_target_typemask::X_IGNORE
        | position_target_typemask::Y_IGNORE
        | position_target_typemask::Z_IGNORE
        | position_target_typemask::AX_IGNORE
        | position_target_typemask::AY_IGNORE
        | position_target_typemask::AZ_IGNORE
        | position_target_typemask::YAW_IGNORE
        | position_target_typemask::YAW_RATE_IGNORE;
    MavMessage::SetPositionTargetLocalNed(SetPositionTargetLocalNed {
        target_system: 1,
        target_component: 1,
        coordinate_frame: 1,
        type_mask: mask,
        vx: CRUISE_MPS,
        ..SetPositionTargetLocalNed::default()
    })
}

fn gcs_heartbeat() -> MavMessage {
    MavMessage::Heartbeat(Heartbeat {
        mav_type: MavType::Gcs as u8,
        autopilot: MAV_AUTOPILOT_INVALID,
        mavlink_version: 3,
        ..Heartbeat::default()
    })
}

fn stream_request() -> MavMessage {
    MavMessage::CommandLong(CommandLong {
        command: SET_MESSAGE_INTERVAL,
        param1: ATTITUDE_QUATERNION_ID,
        param2: 10_000.0,
        target_system: 1,
        target_component: 1,
        ..CommandLong::default()
    })
}

#[derive(Debug, Default, PartialEq, Eq)]
struct Downlink {
    heartbeat: u32,
    sys_status: u32,
    attitude: u32,
    local_position: u32,
    arm_accepted_to_host: u32,
    other_acks: Vec<(u16, u8)>,
}

impl Downlink {
    fn drain(&mut self, flight: &mut Flight) {
        let mut buf = [0_u8; 8192];
        let n = flight.board.pop_mavlink_tx(&mut buf);
        let mut cursor = 0;
        while cursor < n {
            let parsed = parse_mavlink_frame(&buf[cursor..n]).expect("outbound frame");
            cursor += parsed.consumed;
            match parsed.message {
                MavMessage::Heartbeat(heartbeat) => {
                    // The host learns the airframe and that the board is
                    // simulated from the heartbeat alone.
                    assert_eq!(heartbeat.mav_type, MavType::FixedWing as u8);
                    assert_ne!(heartbeat.base_mode & MavModeFlag::HIL_ENABLED.0, 0);
                    self.heartbeat += 1;
                }
                MavMessage::SysStatus(_) => self.sys_status += 1,
                MavMessage::AttitudeQuaternion(_) => self.attitude += 1,
                MavMessage::LocalPositionNed(_) => self.local_position += 1,
                MavMessage::CommandAck(ack)
                    if ack.command == mav_cmd::COMPONENT_ARM_DISARM
                        && ack.result == mav_result::ACCEPTED =>
                {
                    assert_eq!((ack.target_system, ack.target_component), HOST);
                    self.arm_accepted_to_host += 1;
                }
                MavMessage::CommandAck(ack) => self.other_acks.push((ack.command, ack.result)),
                _ => {}
            }
        }
    }
}

/// Everything the vehicle did, step by step, for comparing two runs.
struct Run {
    flight: Flight,
    seq: u8,
    /// Push a GCS heartbeat every second and a stream request at start.
    with_ground_station: bool,
    outputs: Vec<[f32; 16]>,
}

impl Run {
    fn new(with_ground_station: bool, rates: DirectTelemetryRates) -> Self {
        let mut flight = Flight::cruising_disarmed(
            &c172(),
            PlantModel::C172,
            DirectNoiseTier::Off,
            DirectSensorRates::default().with_airspeed(50_000),
        );
        flight.board.set_telemetry_rates(rates);
        Self {
            flight,
            seq: 0,
            with_ground_station,
            outputs: Vec::new(),
        }
    }

    fn push(&mut self, message: &MavMessage) {
        self.seq = self.seq.wrapping_add(1);
        let bytes = frame(message, self.seq);
        assert_eq!(self.flight.board.push_mavlink_rx(&bytes), bytes.len());
    }

    fn step(&mut self, step: u64, downlink: &mut Downlink) {
        if self.with_ground_station && step.is_multiple_of(STEPS_PER_SECOND) {
            self.push(&gcs_heartbeat());
        }
        self.flight.advance();
        self.outputs.push(self.flight.last.outputs);
        downlink.drain(&mut self.flight);
    }

    /// Hands-off until ready, then one setpoint and the arm request.
    fn engage(&mut self, downlink: &mut Downlink) -> u64 {
        if self.with_ground_station {
            self.push(&stream_request());
        }
        let mut step = 0;
        while !self.flight.board.is_ready() {
            assert!(step < 1_000, "kernel not ready after 4 s");
            self.step(step, downlink);
            step += 1;
        }
        self.push(&cruise_setpoint());
        self.push(&arm());
        self.step(step, downlink);
        assert!(self.flight.board.is_armed(), "arm refused");
        step + 1
    }

    fn cruise(&mut self, from: u64, steps: u64, downlink: &mut Downlink) -> u64 {
        for step in from..from + steps {
            if step.is_multiple_of(SETPOINT_REFRESH_STEPS) {
                self.push(&cruise_setpoint());
            }
            self.step(step, downlink);
            assert!(self.flight.last.armed, "disarmed at step {step}");
            assert_eq!(self.flight.board.kernel().state.faults, FaultFlags::empty());
            assert_eq!(
                self.flight.board.kernel().state.control_law,
                ControlLawV1::Primary,
                "degraded at step {step}"
            );
        }
        from + steps
    }
}

#[test]
fn c172_flies_on_mavlink_bytes_without_a_ground_station_link() {
    let mut run = Run::new(false, RATES);
    let mut before_arm = Downlink::default();
    let mut step = run.engage(&mut before_arm);
    // The board streams from its first cycle and acknowledges the arm
    // to the host that sent it, with nothing heard from a ground station.
    assert!(before_arm.heartbeat >= 1, "{before_arm:?}");
    assert_eq!(before_arm.arm_accepted_to_host, 1, "{before_arm:?}");

    let mut settle = Downlink::default();
    step = run.cruise(step, 5 * STEPS_PER_SECOND, &mut settle);

    let window_s = 60;
    let mut window = Downlink::default();
    step = run.cruise(step, window_s * STEPS_PER_SECOND, &mut window);
    let effective = run.flight.board.last_effective_command();
    assert_eq!(effective.mode, ControlMode::VelocityControl);
    assert_eq!(effective.source, CommandSource::Gcs);
    let near = |count: u32, hz: u8| count.abs_diff(u32::from(hz) * window_s as u32) <= 1;
    assert!(near(window.heartbeat, RATES.heartbeat_hz), "{window:?}");
    assert!(near(window.sys_status, RATES.sys_status_hz), "{window:?}");
    assert!(near(window.attitude, RATES.attitude_hz), "{window:?}");
    assert!(near(window.local_position, RATES.position_hz), "{window:?}");
    assert!(window.other_acks.is_empty(), "{window:?}");
    let plant = &run.flight.plant;
    assert!(
        (plant.height() - 300.0).abs() < 5.0,
        "height {}",
        plant.height()
    );
    assert!((plant.indicated_airspeed() - CRUISE_MPS).abs() < 1.5);

    // The refresh rule is about commands, not the link: a 0.6 s gap in
    // setpoints engages the command-loss terminal and the next setpoint
    // releases it.
    let gap_steps = (0.6 / DT_S) as u64;
    let mut quiet = Downlink::default();
    for k in step..step + gap_steps {
        run.step(k, &mut quiet);
    }
    let state = &run.flight.board.kernel().state;
    assert_eq!(state.control_law, ControlLawV1::Direct);
    assert_eq!(
        run.flight.board.last_effective_command().source,
        CommandSource::Failsafe
    );
    run.push(&cruise_setpoint());
    run.cruise(step + gap_steps, SETPOINT_REFRESH_STEPS, &mut quiet);
}

#[test]
fn a_ground_station_heartbeat_and_stream_request_change_nothing() {
    let fly = |with_ground_station: bool| {
        let mut run = Run::new(with_ground_station, DirectTelemetryRates::default());
        let mut engage = Downlink::default();
        let step = run.engage(&mut engage);
        let mut downlink = Downlink::default();
        run.cruise(step, WINDOW_S * STEPS_PER_SECOND, &mut downlink);
        engage.other_acks.append(&mut downlink.other_acks);
        downlink.other_acks = engage.other_acks;
        (run.outputs, downlink)
    };
    const WINDOW_S: u64 = 20;
    let (alone, alone_downlink) = fly(false);
    // The default rates do not all divide into the 4 ms step; each
    // stream still averages its configured rate.
    let rates = DirectTelemetryRates::default();
    let near = |count: u32, hz: u8| count.abs_diff(u32::from(hz) * WINDOW_S as u32) <= 1;
    assert!(
        near(alone_downlink.heartbeat, rates.heartbeat_hz),
        "{alone_downlink:?}"
    );
    assert!(
        near(alone_downlink.sys_status, rates.sys_status_hz),
        "{alone_downlink:?}"
    );
    assert!(
        near(alone_downlink.attitude, rates.attitude_hz),
        "{alone_downlink:?}"
    );
    assert!(
        near(alone_downlink.local_position, rates.position_hz),
        "{alone_downlink:?}"
    );
    let (attended, attended_downlink) = fly(true);
    assert_eq!(alone, attended, "a ground station changed the outputs");
    // The board has no stream configuration over the link: it refuses
    // the request, and every stream keeps its configured rate.
    assert_eq!(
        attended_downlink.other_acks,
        vec![(SET_MESSAGE_INTERVAL, mav_result::UNSUPPORTED)]
    );
    assert_eq!(
        (
            alone_downlink.heartbeat,
            alone_downlink.sys_status,
            alone_downlink.attitude,
            alone_downlink.local_position
        ),
        (
            attended_downlink.heartbeat,
            attended_downlink.sys_status,
            attended_downlink.attitude,
            attended_downlink.local_position
        )
    );
}
