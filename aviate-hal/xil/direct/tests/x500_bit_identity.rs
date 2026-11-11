//! The X500 flies bit for bit as it did before the estimator gained
//! optional blocks: the actuator outputs and the 15-state filter of a
//! noisy hover, climb and GNSS gap fold to a pinned digest.
//!
//! A change of this digest is a change of X500 flight behaviour. It
//! must come with a new estimator or controller identity and a reason.

#![allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]

use aviate_app_sitl_gazebo_x500_kernel::build_x500_kernel;
use aviate_core::control::{Command, CommandSource, ControlMode, Setpoint};
use aviate_core::types::{Meters, Radians};
use aviate_hal_io::SystemCommand;
use aviate_xil_direct::{DirectBoardConfig, DirectNoiseTier, DirectSensorRates, DirectXilBoard};
use world_core::{DecimalYear, GeodeticPosition};

#[path = "plant/x500.rs"]
#[allow(dead_code)]
mod x500_plant;
use x500_plant::QuadX500Plant;

/// FNV-1a over 32-bit words.
struct Digest(u64);

impl Digest {
    fn add(&mut self, value: f32) {
        for byte in value.to_bits().to_le_bytes() {
            self.0 ^= u64::from(byte);
            self.0 = self.0.wrapping_mul(0x0100_0000_01b3);
        }
    }
}

fn hold(z: f32, yaw: f32, sequence: u32) -> SystemCommand {
    SystemCommand::FlightControl(Command {
        mode: ControlMode::PositionHold,
        setpoint: Setpoint {
            position: Some([Meters(1.0), Meters(-2.0), Meters(z)]),
            heading: Some(Radians(yaw)),
            ..Setpoint::default()
        },
        config_mode_request: None,
        sensor_overrides: None,
        sequence,
        source: CommandSource::Gcs,
    })
}

#[test]
fn x500_hover_climb_turn_and_gnss_gap_match_the_pinned_digest() {
    let config = DirectBoardConfig {
        origin: GeodeticPosition::wgs84(39.1582, -75.5244, 15.0),
        epoch: DecimalYear(2026.5),
        noise: DirectNoiseTier::Industrial,
        noise_seed: 99,
        sys_id: 1,
        comp_id: 1,
    };
    let kernel = build_x500_kernel().expect("build X500 kernel");
    let mut board = DirectXilBoard::new(kernel, config).expect("create board");
    let mut plant = QuadX500Plant::new_on_pad(config.origin);
    let dt_us = 2_500_u64;
    let mut digest = Digest(0xcbf2_9ce4_8422_2325);
    let mut step = 0_u64;
    while !board.is_ready() {
        assert!(step < 2_000, "not ready");
        let _ = board.step_truth(&plant.truth(step * dt_us, 0, step));
        step += 1;
    }
    board.send_command(hold(-5.0, 0.0, 1));
    board.send_command(SystemCommand::Arm);
    // 20 s at 400 Hz: climb, a heading change at 8 s, and a 3 s GNSS
    // gap at 12 s.
    for k in 0..8_000_u64 {
        let seconds = k / 400;
        if k % 40 == 0 {
            let yaw = if seconds >= 8 { 1.2 } else { 0.0 };
            board.send_command(hold(-5.0, yaw, k as u32 + 2));
        }
        let rates = if (12..15).contains(&seconds) {
            DirectSensorRates {
                gnss_period_us: u64::MAX,
                ..DirectSensorRates::default()
            }
        } else {
            DirectSensorRates::default()
        };
        board.set_sensor_rates(rates);
        let cmd = board.step_truth(&plant.truth(step * dt_us, 0, step));
        assert!(cmd.armed);
        plant.step(&cmd, 0.0025);
        for output in &cmd.outputs[..usize::from(cmd.count)] {
            digest.add(*output);
        }
        step += 1;
    }
    let ekf = &board.kernel().state.estimator;
    for value in [
        ekf.quat.w,
        ekf.quat.x,
        ekf.quat.y,
        ekf.quat.z,
        ekf.pos.x.0,
        ekf.pos.y.0,
        ekf.pos.z.0,
        ekf.vel.x.0,
        ekf.vel.y.0,
        ekf.vel.z.0,
        ekf.gyro_bias.x.0,
        ekf.gyro_bias.y.0,
        ekf.gyro_bias.z.0,
        ekf.accel_bias.x.0,
        ekf.accel_bias.y.0,
        ekf.accel_bias.z.0,
    ] {
        digest.add(value);
    }
    for row in &ekf.p_cov.data {
        for value in row {
            digest.add(*value);
        }
    }
    println!("X500 digest {:#018x}", digest.0);
    assert!(
        (plant.pos_ned[2] + 5.0).abs() < 1.0,
        "the hold flew: {:?}",
        plant.pos_ned
    );
    assert_eq!(digest.0, PINNED);
}

/// Digest of the X500 flight above. The estimator blocks that only a
/// fixed wing configures leave it unchanged.
const PINNED: u64 = 0xf4d0_d696_1562_e7e2;
