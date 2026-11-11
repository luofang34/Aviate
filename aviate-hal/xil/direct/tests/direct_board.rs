//! Closed-loop quadrotor hover, MAVLink wire parity, and world-reset integration tests.

#![allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]

use aviate_app_sitl_gazebo_x500_kernel::build_x500_kernel;
use aviate_core::control::{Command, CommandSource, ControlMode, Setpoint};
use aviate_core::types::Meters;
use aviate_hal_io::SystemCommand;
use aviate_link::mavlink::protocol::{
    position_target_typemask, CommandLong, SetPositionTargetLocalNed,
};
use aviate_link::mavlink::{
    mav_cmd, mav_result, parse_mavlink_frame, serialize_mavlink, MavAutopilot, MavMessage,
    MavModeFlag, MavState,
};
use aviate_xil_direct::{DirectBoardConfig, DirectNoiseTier, DirectXilBoard};
use world_core::{DecimalYear, GeodeticPosition};

#[path = "plant/x500.rs"]
mod x500_plant;
use x500_plant::QuadX500Plant;

fn test_origin() -> GeodeticPosition {
    GeodeticPosition::wgs84(39.1582, -75.5244, 15.0)
}

fn test_config(noise: DirectNoiseTier, noise_seed: u64) -> DirectBoardConfig {
    DirectBoardConfig {
        origin: test_origin(),
        epoch: DecimalYear(2026.5),
        noise,
        noise_seed,
        sys_id: 1,
        comp_id: 1,
    }
}

fn encode_position_hold_frame(seq: u8, target_z_ned: f32, time_boot_ms: u32) -> ([u8; 280], usize) {
    let mask = position_target_typemask::VX_IGNORE
        | position_target_typemask::VY_IGNORE
        | position_target_typemask::VZ_IGNORE
        | position_target_typemask::AX_IGNORE
        | position_target_typemask::AY_IGNORE
        | position_target_typemask::AZ_IGNORE
        | position_target_typemask::YAW_RATE_IGNORE;
    let msg = MavMessage::SetPositionTargetLocalNed(SetPositionTargetLocalNed {
        time_boot_ms,
        target_system: 1,
        target_component: 1,
        coordinate_frame: 1,
        type_mask: mask,
        x: 0.0,
        y: 0.0,
        z: target_z_ned,
        yaw: 0.0,
        ..SetPositionTargetLocalNed::default()
    });
    let mut buf = [0_u8; 280];
    let len = serialize_mavlink(&msg, seq, 255, 190, &mut buf).expect("serialize setpoint");
    (buf, len)
}

fn encode_arm_frame(seq: u8) -> ([u8; 280], usize) {
    let msg = MavMessage::CommandLong(CommandLong {
        param1: 1.0,
        command: mav_cmd::COMPONENT_ARM_DISARM,
        target_system: 1,
        target_component: 1,
        ..CommandLong::default()
    });
    let mut buf = [0_u8; 280];
    let len = serialize_mavlink(&msg, seq, 255, 190, &mut buf).expect("serialize arm");
    (buf, len)
}

#[derive(Default)]
struct DecodedTelemetryCounts {
    heartbeat_active: usize,
    sys_status: usize,
    attitude: usize,
    local_pos: usize,
    global_pos: usize,
    gps_raw: usize,
    estimator_status: usize,
    aviate_estimator_status: usize,
    geomag: usize,
    arm_ack_accepted: usize,
}

fn drain_and_count_telemetry<C, M>(
    board: &mut DirectXilBoard<C, M>,
    counts: &mut DecodedTelemetryCounts,
) where
    C: aviate_core::control::VehicleController,
    M: aviate_core::mixer::Mixer,
{
    let mut buf = [0_u8; 4096];
    let n = board.pop_mavlink_tx(&mut buf);
    let mut cursor = 0_usize;
    while cursor < n {
        let parsed = parse_mavlink_frame(&buf[cursor..n]).expect("valid outbound MAVLink v2 frame");
        cursor += parsed.consumed;
        match parsed.message {
            MavMessage::Heartbeat(hb) => {
                assert_eq!(hb.autopilot, MavAutopilot::Aviate as u8);
                if hb.system_status == MavState::Active as u8
                    && (hb.base_mode & MavModeFlag::SAFETY_ARMED.0) != 0
                {
                    counts.heartbeat_active += 1;
                }
            }
            MavMessage::SysStatus(_) => counts.sys_status += 1,
            MavMessage::AttitudeQuaternion(_) => counts.attitude += 1,
            MavMessage::LocalPositionNed(_) => counts.local_pos += 1,
            MavMessage::GlobalPositionInt(_) => counts.global_pos += 1,
            MavMessage::GpsRawInt(_) => counts.gps_raw += 1,
            MavMessage::EstimatorStatus(_) => counts.estimator_status += 1,
            MavMessage::AviateEstimatorStatus(_) => counts.aviate_estimator_status += 1,
            MavMessage::AviateGeomagneticReference(_) => counts.geomag += 1,
            MavMessage::CommandAck(ack)
                if ack.command == mav_cmd::COMPONENT_ARM_DISARM
                    && ack.result == mav_result::ACCEPTED =>
            {
                counts.arm_ack_accepted += 1;
            }
            _ => {}
        }
    }
}

#[test]
fn x500_boot_converge_arm_hover_hold_and_mavlink_wire_parity() {
    let kernel = build_x500_kernel().expect("build X500 kernel");
    let cfg = test_config(DirectNoiseTier::Off, 42);
    let mut board = DirectXilBoard::new(kernel, cfg).expect("create DirectXilBoard");
    let mut plant = QuadX500Plant::new_on_pad(cfg.origin);
    let mut counts = DecodedTelemetryCounts::default();

    let dt_us = 2_500_u64;
    let dt_s = 0.0025_f32;
    let mut step = 0_u64;

    while !board.is_ready() {
        assert!(step < 500, "kernel failed to reach Ready within 500 steps");
        let cmd = board.step_truth(&plant.truth(step * dt_us, 0, step));
        assert!(!cmd.armed);
        assert_eq!(cmd.outputs, [0.0; 16]);
        drain_and_count_telemetry(&mut board, &mut counts);
        step += 1;
    }

    let (sp_buf, sp_len) = encode_position_hold_frame(1, -5.0, (step * dt_us / 1_000) as u32);
    let (arm_buf, arm_len) = encode_arm_frame(2);
    assert_eq!(board.push_mavlink_rx(&sp_buf[..11]), 11);
    assert_eq!(board.push_mavlink_rx(&sp_buf[11..sp_len]), sp_len - 11);
    assert_eq!(board.push_mavlink_rx(&arm_buf[..7]), 7);
    assert_eq!(board.push_mavlink_rx(&arm_buf[7..arm_len]), arm_len - 7);

    let mut mav_seq = 3_u8;
    for k in 0..3_200_u64 {
        let time_us = step * dt_us;
        if k > 0 && k % 40 == 0 {
            let (buf, len) = encode_position_hold_frame(mav_seq, -5.0, (time_us / 1_000) as u32);
            mav_seq = mav_seq.wrapping_add(1);
            assert_eq!(board.push_mavlink_rx(&buf[..len]), len);
        }
        let cmd = board.step_truth(&plant.truth(time_us, 0, step));
        assert!(cmd.armed, "vehicle must stay armed during hover climb");
        plant.step(&cmd, dt_s);
        drain_and_count_telemetry(&mut board, &mut counts);
        step += 1;
    }

    assert!(board.is_armed());
    assert!(
        (plant.pos_ned[2] - (-5.0)).abs() < 0.25,
        "expected NED Z near -5.0 m, got {}",
        plant.pos_ned[2]
    );
    assert!(
        plant.vel_ned[2].abs() < 0.15,
        "expected vertical velocity settled near 0, got {}",
        plant.vel_ned[2]
    );
    assert_eq!(counts.arm_ack_accepted, 1);
    assert!(counts.heartbeat_active >= 7);
    assert!(counts.sys_status >= 14);
    assert!(counts.attitude >= 140);
    assert!(counts.local_pos >= 70);
    assert!(counts.global_pos >= 70);
    assert!(counts.gps_raw >= 70);
    assert!(counts.estimator_status >= 140);
    assert_eq!(counts.estimator_status, counts.aviate_estimator_status);
    assert!(counts.geomag >= 7);
}

#[test]
fn world_reset_generation_and_reset_with_kernel_disarm_and_reconverge() {
    let kernel = build_x500_kernel().expect("build X500 kernel");
    let cfg = test_config(DirectNoiseTier::Tactical, 7);
    let mut board = DirectXilBoard::new(kernel, cfg).expect("create board");
    let mut plant = QuadX500Plant::new_on_pad(cfg.origin);

    let dt_us = 2_500_u64;
    let mut step = 0_u64;
    while !board.is_ready() {
        let _ = board.step_truth(&plant.truth(step * dt_us, 0, step));
        step += 1;
    }

    let hold = Command {
        mode: ControlMode::PositionHold,
        setpoint: Setpoint {
            position: Some([Meters(0.0), Meters(0.0), Meters(-3.0)]),
            ..Setpoint::default()
        },
        config_mode_request: None,
        sensor_overrides: None,
        sequence: 1,
        source: CommandSource::Gcs,
    };
    board.send_command(SystemCommand::FlightControl(hold.clone()));
    board.send_command(SystemCommand::Arm);

    for _ in 0..200 {
        let cmd = board.step_truth(&plant.truth(step * dt_us, 0, step));
        assert!(cmd.armed);
        plant.step(&cmd, 0.0025);
        step += 1;
    }
    assert!(board.is_armed());

    plant = QuadX500Plant::new_on_pad(cfg.origin);
    let reset_cmd = board.step_truth(&plant.truth(0, 1, 0));
    assert!(!board.is_armed(), "world reset must disarm immediately");
    assert!(!reset_cmd.armed);
    assert_eq!(reset_cmd.outputs, [0.0; 16]);

    let mut step_gen1 = 1_u64;
    while !board.is_ready() {
        assert!(step_gen1 < 500, "failed to re-converge in generation 1");
        let _ = board.step_truth(&plant.truth(step_gen1 * dt_us, 1, step_gen1));
        step_gen1 += 1;
    }
    board.send_command(SystemCommand::FlightControl(hold.clone()));
    board.send_command(SystemCommand::Arm);
    let rearmed = board.step_truth(&plant.truth(step_gen1 * dt_us, 1, step_gen1));
    assert!(board.is_armed());
    assert!(rearmed.armed);

    board.reset_with_kernel(build_x500_kernel().expect("fresh X500 kernel"));
    assert!(!board.is_armed());
    assert!(!board.is_ready());
    let mut step_gen2 = 0_u64;
    while !board.is_ready() {
        assert!(step_gen2 < 500);
        let _ = board.step_truth(&plant.truth(step_gen2 * dt_us, 2, step_gen2));
        step_gen2 += 1;
    }
    board.send_command(SystemCommand::FlightControl(hold));
    board.send_command(SystemCommand::Arm);
    let after_swap = board.step_truth(&plant.truth(step_gen2 * dt_us, 2, step_gen2));
    assert!(after_swap.armed);
}
