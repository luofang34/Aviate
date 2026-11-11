//! The MAVLink surface that a mission host uses for a fixed wing: a
//! height and a heading as a position target, the airspeed as
//! `MAV_CMD_DO_CHANGE_SPEED`, and `GPS_GLOBAL_ORIGIN` for the local frame.

#![allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]

use aviate_app_sitl_gazebo_x500_kernel::build_x500_kernel;
use aviate_core::control::ControlMode;
use aviate_link::mavlink::protocol::{
    position_target_typemask, CommandLong, SetPositionTargetLocalNed,
};
use aviate_link::mavlink::{
    mav_cmd, mav_result, parse_mavlink_frame, serialize_mavlink, GpsGlobalOrigin, MavMessage,
};
use aviate_xil_direct::{
    DirectBoardConfig, DirectCommandRefusal, DirectNoiseTier, DirectTruthSample, DirectXilBoard,
};
use world_core::{DecimalYear, GeodeticPosition};

#[path = "plant/flight.rs"]
#[allow(dead_code)]
mod flight;
use flight::{wrap, Flight};

#[path = "plant/x500.rs"]
#[allow(dead_code)]
mod x500_plant;
use x500_plant::QuadX500Plant;

const HOST: (u8, u8) = (42, 191);
/// Type mask 2555: `z` and `yaw` active, everything else ignored.
const HEIGHT_HEADING: u16 = position_target_typemask::X_IGNORE
    | position_target_typemask::Y_IGNORE
    | position_target_typemask::VX_IGNORE
    | position_target_typemask::VY_IGNORE
    | position_target_typemask::VZ_IGNORE
    | position_target_typemask::AX_IGNORE
    | position_target_typemask::AY_IGNORE
    | position_target_typemask::AZ_IGNORE
    | position_target_typemask::YAW_RATE_IGNORE;

fn frame(message: &MavMessage, seq: u8) -> Vec<u8> {
    let mut buf = [0_u8; 280];
    let len = serialize_mavlink(message, seq, HOST.0, HOST.1, &mut buf).expect("serialize");
    buf[..len].to_vec()
}

fn height_heading(height_m: f32, heading_rad: f32) -> MavMessage {
    MavMessage::SetPositionTargetLocalNed(SetPositionTargetLocalNed {
        target_system: 1,
        target_component: 1,
        coordinate_frame: 1,
        type_mask: HEIGHT_HEADING,
        z: -height_m,
        yaw: heading_rad,
        ..SetPositionTargetLocalNed::default()
    })
}

fn change_speed(speed_type: f32, speed: f32) -> MavMessage {
    MavMessage::CommandLong(CommandLong {
        command: mav_cmd::DO_CHANGE_SPEED,
        param1: speed_type,
        param2: speed,
        param3: -1.0,
        target_system: 1,
        target_component: 1,
        ..CommandLong::default()
    })
}

/// Acknowledgements and origins the board sent, in order.
#[derive(Default, Debug)]
struct Downlink {
    acks: Vec<(u16, u8, (u8, u8))>,
    origins: Vec<GpsGlobalOrigin>,
}

impl Downlink {
    fn drain<C, M>(&mut self, board: &mut DirectXilBoard<C, M>)
    where
        C: aviate_core::control::VehicleController,
        M: aviate_core::mixer::Mixer,
    {
        let mut buf = [0_u8; 8192];
        let n = board.pop_mavlink_tx(&mut buf);
        let mut cursor = 0;
        while cursor < n {
            let parsed = parse_mavlink_frame(&buf[cursor..n]).expect("outbound frame");
            cursor += parsed.consumed;
            match parsed.message {
                MavMessage::CommandAck(ack) => self.acks.push((
                    ack.command,
                    ack.result,
                    (ack.target_system, ack.target_component),
                )),
                MavMessage::GpsGlobalOrigin(origin) => self.origins.push(origin),
                _ => {}
            }
        }
    }
}

/// Fly `seconds` with the height-and-heading target refreshed each 100 ms.
fn fly_height_heading(
    flight: &mut Flight,
    downlink: &mut Downlink,
    seq: &mut u8,
    seconds: f32,
    height_m: f32,
    heading_rad: f32,
) {
    for step in 0..(seconds / flight::DT_S) as u32 {
        if step % 25 == 0 {
            *seq = seq.wrapping_add(1);
            flight
                .board
                .push_mavlink_rx(&frame(&height_heading(height_m, heading_rad), *seq));
        }
        flight.advance();
        downlink.drain(&mut flight.board);
    }
}

#[test]
fn a_fixed_wing_flies_a_height_heading_and_a_commanded_airspeed() {
    let mut flight = Flight::c172_with_probe(DirectNoiseTier::Off);
    let mut downlink = Downlink::default();
    let mut seq = 0_u8;
    flight
        .board
        .push_mavlink_rx(&frame(&change_speed(0.0, 50.0), seq));
    fly_height_heading(&mut flight, &mut downlink, &mut seq, 60.0, 320.0, 0.5);

    assert_eq!(
        flight.board.last_effective_command().mode,
        ControlMode::AltitudeHold
    );
    assert_eq!(flight.board.endpoint().airspeed_target(), Some(50.0));
    assert!(wrap(flight.plant.heading - 0.5).abs() < 1.0_f32.to_radians());
    assert!(
        (flight.plant.height() - 320.0).abs() < 2.0,
        "{}",
        flight.plant.height()
    );
    assert!((flight.plant.indicated_airspeed() - 50.0).abs() < 1.0);

    // Back to the cruise airspeed; a ground speed is not flown.
    flight
        .board
        .push_mavlink_rx(&frame(&change_speed(0.0, -2.0), seq));
    flight
        .board
        .push_mavlink_rx(&frame(&change_speed(1.0, 30.0), seq));
    fly_height_heading(&mut flight, &mut downlink, &mut seq, 60.0, 320.0, 0.5);
    assert_eq!(flight.board.endpoint().airspeed_target(), None);
    assert!((flight.plant.indicated_airspeed() - 55.0).abs() < 1.0);
    let speed_acks: Vec<_> = downlink
        .acks
        .iter()
        .filter(|ack| ack.0 == mav_cmd::DO_CHANGE_SPEED)
        .map(|ack| (ack.1, ack.2))
        .collect();
    assert_eq!(
        speed_acks,
        vec![
            (mav_result::ACCEPTED, HOST),
            (mav_result::ACCEPTED, HOST),
            (mav_result::DENIED, HOST)
        ]
    );
}

fn x500_board() -> (
    DirectXilBoard<impl aviate_core::control::VehicleController, impl aviate_core::mixer::Mixer>,
    QuadX500Plant,
) {
    let config = DirectBoardConfig {
        origin: GeodeticPosition::wgs84(39.1582, -75.5244, 15.0),
        epoch: DecimalYear(2026.5),
        noise: DirectNoiseTier::Off,
        noise_seed: 1,
        sys_id: 1,
        comp_id: 1,
    };
    let board = DirectXilBoard::new(build_x500_kernel().expect("kernel"), config).expect("board");
    (board, QuadX500Plant::new_on_pad(config.origin))
}

fn truth(plant: &QuadX500Plant, step: u64) -> DirectTruthSample {
    plant.truth(step * 2_500, 0, step)
}

#[test]
fn a_multirotor_refuses_a_height_heading_target_and_an_airspeed() {
    let (mut board, plant) = x500_board();
    let mut downlink = Downlink::default();
    board.push_mavlink_rx(&frame(&height_heading(5.0, 0.0), 1));
    board.push_mavlink_rx(&frame(&change_speed(0.0, 12.0), 2));
    for step in 0..40 {
        let _ = board.step_truth(&truth(&plant, step));
        downlink.drain(&mut board);
    }
    assert_eq!(
        board.last_command_refusal(),
        Some(DirectCommandRefusal::ModeNotCommandable(
            ControlMode::AltitudeHold
        ))
    );
    assert_ne!(
        board.last_effective_command().mode,
        ControlMode::AltitudeHold
    );
    assert_eq!(
        downlink.acks,
        vec![(mav_cmd::DO_CHANGE_SPEED, mav_result::UNSUPPORTED, HOST)]
    );
}

#[test]
fn the_origin_is_announced_once_per_start_with_its_mean_sea_level_altitude() {
    let (mut board, plant) = x500_board();
    let mut downlink = Downlink::default();
    for step in 0..400 {
        let _ = board.step_truth(&truth(&plant, step));
        downlink.drain(&mut board);
    }
    // Without a geoid separation the board cannot state an MSL altitude.
    assert!(downlink.origins.is_empty());

    board.set_geoid_separation(-33.4);
    for step in 400..800 {
        let _ = board.step_truth(&truth(&plant, step));
        downlink.drain(&mut board);
    }
    let expected = GpsGlobalOrigin {
        latitude: 391_582_000,
        longitude: -755_244_000,
        // 15 m above the ellipsoid, the geoid 33.4 m below it.
        altitude: 48_400,
        time_usec: downlink
            .origins
            .first()
            .map_or(0, |origin| origin.time_usec),
    };
    assert_eq!(downlink.origins, vec![expected]);

    board.announce_origin();
    let _ = board.step_truth(&truth(&plant, 800));
    downlink.drain(&mut board);
    assert_eq!(downlink.origins.len(), 2);
}
