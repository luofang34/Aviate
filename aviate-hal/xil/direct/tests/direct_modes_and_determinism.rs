//! Fixed-wing controller, noise-seed determinism, and ring/synthesizer edge-case tests.

#![allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]

use aviate_app_sitl_gazebo_x500_kernel::build_x500_kernel;
use aviate_core::control::fixed_wing::FixedWingController;
use aviate_core::control::{
    AxisCommand, Command, CommandSource, ConfigMode, ControlMode, Setpoint,
};
use aviate_core::ekf::Ekf;
use aviate_core::kernel::builder::AviateKernelBuilder;
use aviate_core::kernel::config::{MixerGeometry, ResolvedKernelConfig};
use aviate_core::math::Quaternion;
use aviate_core::mixer::{ActuatorCmd, Mixer, ModeConfig, Sanitizer, MAX_ACTUATORS};
use aviate_core::time::{TimeSource, Timestamp};
use aviate_core::types::{Meters, Normalized, NormalizedThrust};
use aviate_hal_io::SystemCommand;
use aviate_link::mavlink::protocol::CommandLong;
use aviate_link::mavlink::{mav_cmd, serialize_mavlink, MavMessage, FORCE_ARM_DISARM_MAGIC};
use aviate_xil_direct::{
    DirectBoardConfig, DirectByteRing, DirectMeasuredKinematics, DirectNoiseTier, DirectOrigin,
    DirectSensorRates, DirectSynthesizer, DirectTruthSample, DirectXilBoard,
};
use world_core::{normal_gravity, DecimalYear, GeodeticPosition};

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

struct PassthroughFwMixer;

impl Mixer for PassthroughFwMixer {
    const ALGORITHM_ID: u64 = 0x5453_5446_574D_4958;
    const GEOMETRY: MixerGeometry = MixerGeometry::QuadX;

    fn mix(&self, axis: &AxisCommand) -> ActuatorCmd {
        let mut outputs = [Normalized(0.0); MAX_ACTUATORS];
        outputs[0] = Normalized(axis.collective.0.clamp(0.0, 1.0));
        outputs[1] = Normalized(0.5 + 0.5 * axis.roll.0);
        outputs[2] = Normalized(0.5 + 0.5 * axis.pitch.0);
        outputs[3] = Normalized(0.5 + 0.5 * axis.yaw.0);
        ActuatorCmd {
            outputs,
            active_mask: 0b1111,
            sequence: 0,
            timestamp: Timestamp {
                ticks: 0,
                source: TimeSource::Internal,
            },
            fallback_mask: 0,
            sanitized: false,
        }
    }
}

fn cruise_truth(step: u64, dt_us: u64, gravity_mps2: f32) -> DirectTruthSample {
    let x_m = (step as f32) * (dt_us as f32 * 1.0e-6) * 18.0;
    DirectTruthSample {
        time_us: step * dt_us,
        reset_generation: 0,
        step,
        position_ned_m: [x_m, 0.0, -50.0],
        attitude_wxyz: [1.0, 0.0, 0.0, 0.0],
        measured: Some(DirectMeasuredKinematics {
            velocity_ned_mps: [18.0, 0.0, 0.0],
            body_rates_frd_rps: [0.0, 0.0, 0.0],
            specific_force_frd_mps2: [0.0, 0.0, -gravity_mps2],
        }),
    }
}

#[test]
fn fixed_wing_controller_steps_through_direct_board() {
    assert_eq!(aviate_airframe_fixed_wing::airframe_id(), "fixed-wing");

    let cfg_kernel = ResolvedKernelConfig {
        mixer_geometry: MixerGeometry::QuadX,
        mode_config: ModeConfig {
            mode: ConfigMode::Cruise,
            groups: &[],
        },
        ..ResolvedKernelConfig::default()
    };
    let kernel = AviateKernelBuilder::new()
        .estimator(Ekf::default())
        .controller(FixedWingController)
        .mixer(PassthroughFwMixer)
        .sanitizer(Sanitizer)
        .config(cfg_kernel)
        .build()
        .expect("build fixed-wing kernel");

    let cfg = test_config(DirectNoiseTier::Industrial, 99);
    let mut board = DirectXilBoard::new(kernel, cfg).expect("create FW board");
    let g = normal_gravity(cfg.origin).expect("gravity") as f32;

    let dt_us = 2_500_u64;
    let mut step = 0_u64;
    while !board.is_ready() {
        assert!(step < 500);
        let _ = board.step_truth(&cruise_truth(step, dt_us, g));
        step += 1;
    }

    board.send_command(SystemCommand::Arm);
    let _ = board.step_truth(&cruise_truth(step, dt_us, g));
    assert!(board.is_armed());
    step += 1;

    let cruise_cmd = Command {
        mode: ControlMode::Attitude,
        setpoint: Setpoint {
            attitude: Some(Quaternion::IDENTITY),
            collective_thrust: NormalizedThrust(0.65),
            ..Setpoint::default()
        },
        config_mode_request: None,
        sensor_overrides: None,
        sequence: 1,
        source: CommandSource::Gcs,
    };
    board.send_command(SystemCommand::FlightControl(cruise_cmd));
    let out = board.step_truth(&cruise_truth(step, dt_us, g));
    assert!(out.armed);
    assert_eq!(out.count, 4);
    assert!((out.outputs[0] - 0.65).abs() < 1.0e-6);
}

#[test]
fn identical_seed_and_truth_stream_yield_bit_identical_outputs() {
    let seed = 0xCAFE_BABE_1234_5678_u64;
    let cfg_same = test_config(DirectNoiseTier::Consumer, seed);
    let cfg_diff = test_config(DirectNoiseTier::Consumer, 0xDEAD_BEEF_0000_0001);

    let mut board_a =
        DirectXilBoard::new(build_x500_kernel().expect("k1"), cfg_same).expect("board_a");
    let mut board_b =
        DirectXilBoard::new(build_x500_kernel().expect("k2"), cfg_same).expect("board_b");
    let mut board_c =
        DirectXilBoard::new(build_x500_kernel().expect("k3"), cfg_diff).expect("board_c");

    let g = normal_gravity(cfg_same.origin).expect("gravity") as f32;
    let dt_us = 2_500_u64;
    let mut any_diff_seed_diverged = false;

    for step in 0..600_u64 {
        let time_us = step * dt_us;
        if step == 140 {
            let hold = Command {
                mode: ControlMode::PositionHold,
                setpoint: Setpoint {
                    position: Some([Meters(1.0), Meters(-0.5), Meters(-2.5)]),
                    ..Setpoint::default()
                },
                config_mode_request: None,
                sensor_overrides: None,
                sequence: 1,
                source: CommandSource::Gcs,
            };
            for b in [&mut board_a, &mut board_b, &mut board_c] {
                b.send_command(SystemCommand::FlightControl(hold.clone()));
                b.send_command(SystemCommand::Arm);
            }
        }
        let z_ned = if step > 140 {
            -((step - 140) as f32 * 0.004).min(2.0)
        } else {
            0.0
        };
        let truth = DirectTruthSample {
            time_us,
            reset_generation: 0,
            step,
            position_ned_m: [0.0, 0.0, z_ned],
            attitude_wxyz: [1.0, 0.0, 0.0, 0.0],
            measured: Some(DirectMeasuredKinematics {
                velocity_ned_mps: [0.0, 0.0, if step > 140 { -0.5 } else { 0.0 }],
                body_rates_frd_rps: [0.0, 0.0, 0.0],
                specific_force_frd_mps2: [0.0, 0.0, -g],
            }),
        };
        let cmd_a = board_a.step_truth(&truth);
        let cmd_b = board_b.step_truth(&truth);
        let cmd_c = board_c.step_truth(&truth);

        assert_eq!(cmd_a.timestamp_us, cmd_b.timestamp_us);
        assert_eq!(cmd_a.armed, cmd_b.armed);
        assert_eq!(cmd_a.count, cmd_b.count);
        assert_eq!(
            cmd_a.outputs.map(f32::to_bits),
            cmd_b.outputs.map(f32::to_bits),
            "bit mismatch at step {step}"
        );
        if cmd_a.armed && cmd_a.outputs.map(f32::to_bits) != cmd_c.outputs.map(f32::to_bits) {
            any_diff_seed_diverged = true;
        }
    }

    assert!(board_a.is_armed());
    assert!(
        any_diff_seed_diverged,
        "distinct noise seeds must produce distinct noisy actuator trajectories"
    );
}

fn encode_arm_disarm_frame(
    seq: u8,
    target_sys: u8,
    param1: f32,
    param2: f32,
) -> ([u8; 280], usize) {
    let msg = MavMessage::CommandLong(CommandLong {
        param1,
        param2,
        command: mav_cmd::COMPONENT_ARM_DISARM,
        target_system: target_sys,
        target_component: 1,
        ..CommandLong::default()
    });
    let mut buf = [0_u8; 280];
    let len = serialize_mavlink(&msg, seq, 255, 190, &mut buf).expect("serialize arm/disarm");
    (buf, len)
}

#[test]
fn synthesizer_ring_and_mavlink_addressing_edge_cases() {
    let mut ring = DirectByteRing::<8>::new();
    assert_eq!(ring.push(b"abcdef"), 6);
    let mut out = [0_u8; 4];
    assert_eq!(ring.pop(&mut out), 4);
    assert_eq!(&out, b"abcd");
    assert!(!ring.push_all(b"1234567"));
    assert!(ring.push_all(b"123456"));
    let mut all = [0_u8; 8];
    assert_eq!(ring.pop(&mut all), 8);
    assert_eq!(&all, b"ef123456");

    let origin = DirectOrigin::new(test_origin(), DecimalYear(2026.5));
    let mut synth = DirectSynthesizer::new(
        origin,
        DirectSensorRates::every_step(),
        DirectNoiseTier::Off,
        1,
    )
    .expect("synth");
    let t0 = DirectTruthSample {
        time_us: 1_000,
        reset_generation: 0,
        step: 0,
        position_ned_m: [0.0, 0.0, 0.0],
        attitude_wxyz: [1.0, 0.0, 0.0, 0.0],
        measured: None,
    };
    assert!(synth.synthesize(&t0).is_none());
    assert!(synth.synthesize(&t0).is_none());
    let t1 = DirectTruthSample {
        time_us: 11_000,
        reset_generation: 0,
        step: 1,
        position_ned_m: [0.2, 0.0, 0.0],
        attitude_wxyz: [1.0, 0.0, 0.0, 0.0],
        measured: None,
    };
    let s1 = synth
        .synthesize(&t1)
        .expect("advancing timestamp produces sample");
    assert_eq!(s1.delta_us, 10_000);
    let gnss = s1.gnss.expect("every_step GNSS");
    assert!((gnss.vel_ned[0] - 20.0).abs() < 1.0e-3);

    let g = normal_gravity(test_origin()).expect("gravity") as f32;
    let mut board = DirectXilBoard::new(
        build_x500_kernel().expect("kernel"),
        test_config(DirectNoiseTier::Off, 0),
    )
    .expect("board");
    for s in 0..150_u64 {
        let _ = board.step_truth(&cruise_truth(s, 2_500, g));
    }
    assert!(board.is_ready());
    let (wrong_arm, wrong_len) = encode_arm_disarm_frame(1, 2, 1.0, 0.0);
    board.push_mavlink_rx(&wrong_arm[..wrong_len]);
    let _ = board.step_truth(&cruise_truth(150, 2_500, g));
    assert!(
        !board.is_armed(),
        "command aimed at sysid 2 must be ignored"
    );

    let (right_arm, right_len) = encode_arm_disarm_frame(2, 1, 1.0, 0.0);
    board.push_mavlink_rx(&right_arm[..right_len]);
    let _ = board.step_truth(&cruise_truth(151, 2_500, g));
    assert!(board.is_armed());

    let (kill_buf, kill_len) = encode_arm_disarm_frame(3, 1, 0.0, FORCE_ARM_DISARM_MAGIC);
    board.push_mavlink_rx(&kill_buf[..kill_len]);
    let killed = board.step_truth(&cruise_truth(152, 2_500, g));
    assert!(!board.is_armed());
    assert!(!killed.armed);
}
