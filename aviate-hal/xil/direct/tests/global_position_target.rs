//! `SET_POSITION_TARGET_GLOBAL_INT`: a geodetic target with an altitude
//! above mean sea level. The board expresses it in its present frame,
//! so a target stays right however often the frame origin moves, and it
//! refuses the target while it cannot turn the altitude into a height.

#![allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]

use aviate_app_sitl_gazebo_x500_kernel::build_x500_kernel;
use aviate_core::control::ControlMode;
use aviate_hal_io::SystemCommand;
use aviate_link::mavlink::protocol::{position_target_typemask, SetPositionTargetGlobalInt};
use aviate_link::mavlink::{serialize_mavlink, MavMessage};
use aviate_xil_direct::{DirectBoardConfig, DirectCommandRefusal, DirectNoiseTier, DirectXilBoard};
use world_core::{DecimalYear, GeodeticPosition, LocalTangentPlane, Ned};

#[path = "plant/x500.rs"]
#[allow(dead_code)]
mod x500_plant;
use x500_plant::QuadX500Plant;

/// Geoid height above the ellipsoid at the origin \[m\].
const GEOID_M: f64 = -33.4;
/// The target, in the configured frame: 3 m north, 2 m east, 5 m up.
const TARGET_NED: [f64; 3] = [3.0, 2.0, -5.0];

fn config() -> DirectBoardConfig {
    DirectBoardConfig {
        origin: GeodeticPosition::wgs84(39.1582, -75.5244, 15.0),
        epoch: DecimalYear(2026.5),
        noise: DirectNoiseTier::Off,
        noise_seed: 5,
        sys_id: 1,
        comp_id: 1,
    }
}

/// The target as a host sends it: latitude, longitude and MSL altitude.
fn target_frame(seq: u8) -> Vec<u8> {
    let plane = LocalTangentPlane::new(config().origin).expect("origin");
    let [north_m, east_m, down_m] = TARGET_NED;
    let at = plane.to_geodetic(Ned {
        north_m,
        east_m,
        down_m,
    });
    let mask = position_target_typemask::VX_IGNORE
        | position_target_typemask::VY_IGNORE
        | position_target_typemask::VZ_IGNORE
        | position_target_typemask::AX_IGNORE
        | position_target_typemask::AY_IGNORE
        | position_target_typemask::AZ_IGNORE
        | position_target_typemask::YAW_RATE_IGNORE;
    let message = MavMessage::SetPositionTargetGlobalInt(SetPositionTargetGlobalInt {
        lat_int: (at.latitude_deg * 1e7).round() as i32,
        lon_int: (at.longitude_deg * 1e7).round() as i32,
        alt: (at.ellipsoidal_height_m - GEOID_M) as f32,
        type_mask: mask,
        target_system: 1,
        target_component: 1,
        coordinate_frame: SetPositionTargetGlobalInt::FRAME_GLOBAL_INT,
        ..SetPositionTargetGlobalInt::default()
    });
    let mut buf = [0_u8; 280];
    let len = serialize_mavlink(&message, seq, 42, 191, &mut buf).expect("serialize");
    buf[..len].to_vec()
}

type Board = DirectXilBoard<
    aviate_core::control::multirotor::MultirotorController,
    aviate_core::mixer::QuadXMixerX500,
>;

/// Fly `steps` cycles at 400 Hz, sending the target each 100 ms.
fn fly(board: &mut Board, plant: &mut QuadX500Plant, step: &mut u64, steps: u64) {
    for local in 0..steps {
        if local % 40 == 0 {
            board.push_mavlink_rx(&target_frame((*step / 40) as u8));
        }
        let cmd = board.step_truth(&plant.truth(*step * 2_500, 0, *step));
        plant.step(&cmd, 0.0025);
        *step += 1;
    }
}

fn ready_board() -> (Board, QuadX500Plant, u64) {
    let config = config();
    let mut board =
        DirectXilBoard::new(build_x500_kernel().expect("kernel"), config).expect("board");
    let plant = QuadX500Plant::new_on_pad(config.origin);
    let mut step = 0_u64;
    while !board.is_ready() {
        let _ = board.step_truth(&plant.truth(step * 2_500, 0, step));
        step += 1;
    }
    (board, plant, step)
}

#[test]
fn a_target_with_a_sea_level_altitude_needs_the_geoid_separation() {
    let (mut board, mut plant, mut step) = ready_board();
    fly(&mut board, &mut plant, &mut step, 40);
    assert_eq!(
        board.last_command_refusal(),
        Some(DirectCommandRefusal::GeoidSeparationUnknown)
    );
    assert_ne!(
        board.last_effective_command().mode,
        ControlMode::PositionHold
    );
}

#[test]
fn a_geodetic_target_holds_its_point_while_the_frame_origin_moves() {
    let (mut board, mut plant, mut step) = ready_board();
    board.set_geoid_separation(GEOID_M);
    // A small radius makes the origin move on the way to the target.
    board.set_rebase_radius(1.5);
    board.send_command(SystemCommand::Arm);
    fly(&mut board, &mut plant, &mut step, 1);
    let held = board
        .last_effective_command()
        .setpoint
        .position
        .expect("position");
    for (got, want) in held.iter().zip(TARGET_NED) {
        assert!((f64::from(got.0) - want).abs() < 0.01, "{held:?}");
    }
    fly(&mut board, &mut plant, &mut step, 8_000);

    let rebases = board.kernel().state.estimator.frame.rebases;
    assert!(rebases >= 1, "the origin moved: {rebases}");
    assert_eq!(
        board.last_effective_command().mode,
        ControlMode::PositionHold
    );
    // The plant flies in the configured frame: it is at the target.
    for (got, want) in plant.pos_ned.iter().zip(TARGET_NED) {
        assert!((f64::from(*got) - want).abs() < 0.5, "{:?}", plant.pos_ned);
    }
}
