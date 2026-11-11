//! Host-built sensor samples through `DirectXilBoard::step_sensors`, on the X500 hover profile.

#![allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]

use aviate_app_sitl_gazebo_x500_kernel::build_x500_kernel;
use aviate_core::control::{Command, CommandSource, ControlMode, Setpoint};
use aviate_core::ekf::Estimator;
use aviate_core::sensor::{GeomagneticReference, GeomagneticSource};
use aviate_core::state::{StateEstimate, StateValidFlags};
use aviate_core::types::{Meters, Radians};
use aviate_hal_io::{GnssFix, SystemCommand};
use aviate_xil_direct::{
    DirectActuatorCmd, DirectBoardConfig, DirectHostSensors, DirectMeasuredKinematics,
    DirectNoiseTier, DirectOrigin, DirectSynthesizer, DirectTruthSample, DirectXilBoard,
};
use world_core::{normal_gravity, DecimalYear, GeodeticPosition};

#[path = "plant/x500.rs"]
mod x500_plant;
use x500_plant::QuadX500Plant;

const DT_US: u64 = 2_500;
const DT_S: f32 = 0.0025;
/// Steps between setpoint refreshes: 100 ms, inside the 500 ms command timeout.
const SETPOINT_REFRESH_STEPS: u64 = 40;
const HOVER_DOWN_M: f32 = -5.0;

fn test_config(noise: DirectNoiseTier, noise_seed: u64) -> DirectBoardConfig {
    DirectBoardConfig {
        origin: GeodeticPosition::wgs84(39.1582, -75.5244, 15.0),
        epoch: DecimalYear(2026.5),
        noise,
        noise_seed,
        sys_id: 1,
        comp_id: 1,
    }
}

fn hover_hold(sequence: u32) -> Command {
    Command {
        mode: ControlMode::PositionHold,
        setpoint: Setpoint {
            position: Some([Meters(0.0), Meters(0.0), Meters(HOVER_DOWN_M)]),
            ..Setpoint::default()
        },
        config_mode_request: None,
        sensor_overrides: None,
        sequence,
        source: CommandSource::Gcs,
    }
}

type X500Board = DirectXilBoard<
    aviate_core::control::multirotor::MultirotorController,
    aviate_core::mixer::QuadXMixerX500,
>;

fn new_board(cfg: DirectBoardConfig) -> X500Board {
    DirectXilBoard::new(build_x500_kernel().expect("build X500 kernel"), cfg)
        .expect("create DirectXilBoard")
}

/// Whether the board accepts a setpoint: ready to arm, or already armed.
fn commandable(board: &X500Board) -> bool {
    board.is_ready() || board.is_armed()
}

fn estimate_of(board: &X500Board) -> StateEstimate {
    let kernel = board.kernel();
    kernel
        .pipeline()
        .estimator
        .estimate(&kernel.state.estimator)
}

/// The host side of the sensor boundary: Aviate's synthesizer run outside the board.
struct HostSensorModel {
    synthesizer: DirectSynthesizer,
}

impl HostSensorModel {
    fn new(cfg: &DirectBoardConfig) -> Self {
        Self {
            synthesizer: DirectSynthesizer::with_default_rates(
                DirectOrigin::new(cfg.origin, cfg.epoch),
                cfg.noise,
                cfg.noise_seed,
            )
            .expect("host synthesizer"),
        }
    }

    /// Samples for `truth` with the declination of the synthesizer environment.
    fn sample(&mut self, truth: &DirectTruthSample) -> DirectHostSensors {
        match self.synthesizer.synthesize(truth) {
            Some(sample) => sample.host_sensors(Some(GeomagneticReference {
                declination: Radians(self.synthesizer.environment().declination_rad()),
                source: GeomagneticSource::Model,
            })),
            None => DirectHostSensors {
                time_us: truth.time_us,
                reset_generation: truth.reset_generation,
                ..DirectHostSensors::default()
            },
        }
    }
}

fn actuator_bits(cmd: &DirectActuatorCmd) -> ([u32; 16], u8, bool, u64) {
    (
        cmd.outputs.map(f32::to_bits),
        cmd.count,
        cmd.armed,
        cmd.timestamp_us,
    )
}

fn estimate_bits(e: &StateEstimate) -> ([u32; 13], u8) {
    (
        [
            e.attitude.w.to_bits(),
            e.attitude.x.to_bits(),
            e.attitude.y.to_bits(),
            e.attitude.z.to_bits(),
            e.angular_velocity[0].0.to_bits(),
            e.angular_velocity[1].0.to_bits(),
            e.angular_velocity[2].0.to_bits(),
            e.position_ned[0].0.to_bits(),
            e.position_ned[1].0.to_bits(),
            e.position_ned[2].0.to_bits(),
            e.velocity_ned[0].0.to_bits(),
            e.velocity_ned[1].0.to_bits(),
            e.velocity_ned[2].0.to_bits(),
        ],
        e.valid_flags.bits(),
    )
}

fn drain_tx(board: &mut X500Board, into: &mut Vec<u8>) {
    let mut buf = [0_u8; 4096];
    loop {
        let n = board.pop_mavlink_tx(&mut buf);
        if n == 0 {
            break;
        }
        into.extend_from_slice(&buf[..n]);
    }
}

#[test]
fn host_built_samples_match_step_truth_bit_for_bit() {
    // Sensor noise is on, so the comparison also covers the noise draw order.
    let cfg = test_config(DirectNoiseTier::Tactical, 42);
    let mut truth_board = new_board(cfg);
    let mut host_board = new_board(cfg);
    let mut host = HostSensorModel::new(&cfg);
    let mut plant = QuadX500Plant::new_on_pad(cfg.origin);
    let (mut truth_tx, mut host_tx) = (Vec::new(), Vec::new());

    let mut armed_cycles = 0_u32;
    let mut sequence = 0_u32;
    // 1 s on the pad plus 9 s of climb and hold.
    for step in 0..4_000_u64 {
        let truth = plant.truth(step * DT_US, 0, step);
        if commandable(&truth_board) && step % SETPOINT_REFRESH_STEPS == 0 {
            sequence += 1;
            for board in [&mut truth_board, &mut host_board] {
                board.send_command(SystemCommand::FlightControl(hover_hold(sequence)));
                if !board.is_armed() {
                    board.send_command(SystemCommand::Arm);
                }
            }
        }

        let from_truth = truth_board.step_truth(&truth);
        let from_host = host_board.step_sensors(&host.sample(&truth));

        assert_eq!(
            actuator_bits(&from_truth),
            actuator_bits(&from_host),
            "actuator output differs at step {step}"
        );
        assert_eq!(
            estimate_bits(&estimate_of(&truth_board)),
            estimate_bits(&estimate_of(&host_board)),
            "estimate differs at step {step}"
        );
        drain_tx(&mut truth_board, &mut truth_tx);
        drain_tx(&mut host_board, &mut host_tx);

        if from_truth.armed {
            armed_cycles += 1;
        }
        plant.step(&from_truth, DT_S);
    }

    // The profile is a real flight, not two boards that agree on doing nothing.
    assert!(armed_cycles > 3_000, "armed for {armed_cycles} cycles");
    assert!(
        (plant.pos_ned[2] - HOVER_DOWN_M).abs() < 0.25,
        "hover height {} m",
        plant.pos_ned[2]
    );
    assert!(!truth_tx.is_empty());
    assert_eq!(truth_tx, host_tx, "MAVLink telemetry bytes differ");
}

/// How a receiver that has no fix presents itself to the board.
#[derive(Clone, Copy)]
enum SilentGnss {
    /// The lane carries no sample.
    NoSample,
    /// The lane carries samples whose fix type is "no fix".
    NoFix,
}

struct GnssOutage {
    /// Seconds from the last delivered fix to the first cycle without POSITION validity.
    position_drop_s: f32,
    /// Seconds from the last delivered fix to the first cycle without VELOCITY validity.
    velocity_drop_s: f32,
    /// Effective control mode after the drop.
    mode_after: ControlMode,
    /// Effective control mode one GNSS period before the drop.
    mode_before: ControlMode,
    /// Actuator outputs of every cycle, for comparison between outage forms.
    outputs: Vec<[u32; 16]>,
}

fn fly_gnss_outage(form: SilentGnss) -> GnssOutage {
    const OUTAGE_START_STEP: u64 = 2_800; // 7 s: in the hover hold
    let cfg = test_config(DirectNoiseTier::Off, 7);
    let mut board = new_board(cfg);
    let mut host = HostSensorModel::new(&cfg);
    let mut plant = QuadX500Plant::new_on_pad(cfg.origin);

    let mut last_fix_us = None;
    let (mut position_drop_us, mut velocity_drop_us) = (None, None);
    let mut mode_before = None;
    let mut outputs = Vec::new();
    let mut sequence = 0_u32;
    for step in 0..5_200_u64 {
        let truth = plant.truth(step * DT_US, 0, step);
        if commandable(&board) && step % SETPOINT_REFRESH_STEPS == 0 {
            sequence += 1;
            board.send_command(SystemCommand::FlightControl(hover_hold(sequence)));
            if !board.is_armed() {
                board.send_command(SystemCommand::Arm);
            }
        }

        let mut sensors = host.sample(&truth);
        if step >= OUTAGE_START_STEP {
            match form {
                SilentGnss::NoSample => sensors.gnss = None,
                SilentGnss::NoFix => {
                    if let Some(gnss) = sensors.gnss.as_mut() {
                        gnss.reading.fix = GnssFix::None;
                    }
                }
            }
        } else if sensors.gnss.is_some() {
            last_fix_us = Some(truth.time_us);
        }

        let cmd = board.step_sensors(&sensors);
        outputs.push(cmd.outputs.map(f32::to_bits));

        let valid = estimate_of(&board).valid_flags;
        if step == OUTAGE_START_STEP {
            assert!(
                valid.contains(StateValidFlags::POSITION | StateValidFlags::VELOCITY),
                "the hover must be fully aided before the outage"
            );
            assert_eq!(
                board.last_effective_command().mode,
                ControlMode::PositionHold
            );
        }
        if step > OUTAGE_START_STEP {
            if position_drop_us.is_none() && !valid.contains(StateValidFlags::POSITION) {
                position_drop_us = Some(truth.time_us);
            }
            if velocity_drop_us.is_none() && !valid.contains(StateValidFlags::VELOCITY) {
                velocity_drop_us = Some(truth.time_us);
            }
            if position_drop_us.is_none() {
                mode_before = Some(board.last_effective_command().mode);
            }
        }
        plant.step(&cmd, DT_S);
    }

    let last_fix_us = last_fix_us.expect("a fix before the outage");
    let since_fix = |at: Option<u64>| (at.expect("validity dropped") - last_fix_us) as f32 * 1.0e-6;
    GnssOutage {
        position_drop_s: since_fix(position_drop_us),
        velocity_drop_s: since_fix(velocity_drop_us),
        mode_after: board.last_effective_command().mode,
        mode_before: mode_before.expect("cycles before the drop"),
        outputs,
    }
}

#[test]
fn silent_gnss_drops_validity_after_five_seconds_and_falls_back_to_attitude() {
    let no_sample = fly_gnss_outage(SilentGnss::NoSample);
    let no_fix = fly_gnss_outage(SilentGnss::NoFix);

    for outage in [&no_sample, &no_fix] {
        // The estimator holds POSITION and VELOCITY for 5 s after the last
        // accepted fix (HLR-EST-205). The age is a sum of 2000 steps of
        // 2.5 ms in f32, so its rounding can move the drop by one cycle to
        // either side; the window allows two cycles before and eight after.
        assert!(
            (4.995..5.02).contains(&outage.position_drop_s),
            "POSITION dropped {} s after the last fix",
            outage.position_drop_s
        );
        assert!(
            (4.995..5.02).contains(&outage.velocity_drop_s),
            "VELOCITY dropped {} s after the last fix",
            outage.velocity_drop_s
        );
        assert_eq!(outage.mode_before, ControlMode::PositionHold);
        assert_eq!(outage.mode_after, ControlMode::Attitude);
    }

    // A receiver that reports "no fix" and a receiver that reports nothing
    // are the same event for the kernel, cycle for cycle.
    assert_eq!(no_sample.position_drop_s, no_fix.position_drop_s);
    assert_eq!(no_sample.velocity_drop_s, no_fix.velocity_drop_s);
    assert_eq!(no_sample.outputs, no_fix.outputs);
}

/// Rotate a body-frame field about the body down axis, as a yaw of `angle_rad` would.
fn yawed_field(field_ut: [f32; 3], angle_rad: f32) -> [f32; 3] {
    let (sin, cos) = angle_rad.sin_cos();
    [
        cos * field_ut[0] + sin * field_ut[1],
        -sin * field_ut[0] + cos * field_ut[1],
        field_ut[2],
    ]
}

struct HoverResult {
    /// Estimated minus true position in local NED meters.
    position_error_m: [f32; 3],
    /// Estimated minus true yaw in radians.
    yaw_error_rad: f32,
    /// True height above the pad in meters.
    height_m: f32,
}

fn hover_with_host_offsets(gnss_offset_ned_m: [f32; 3], heading_offset_rad: f32) -> HoverResult {
    let cfg = test_config(DirectNoiseTier::Off, 11);
    let mut board = new_board(cfg);
    let mut host = HostSensorModel::new(&cfg);
    let mut plant = QuadX500Plant::new_on_pad(cfg.origin);

    let mut sequence = 0_u32;
    for step in 0..4_800_u64 {
        let truth = plant.truth(step * DT_US, 0, step);
        if commandable(&board) && step % SETPOINT_REFRESH_STEPS == 0 {
            sequence += 1;
            board.send_command(SystemCommand::FlightControl(hover_hold(sequence)));
            if !board.is_armed() {
                board.send_command(SystemCommand::Arm);
            }
        }

        // The host owns the sensor models, so the host owns their errors.
        let mut sensors = host.sample(&truth);
        if let Some(gnss) = sensors.gnss.as_mut() {
            for (axis, offset) in gnss.reading.position_ned.iter_mut().zip(gnss_offset_ned_m) {
                *axis += offset;
            }
        }
        if let Some(mag) = sensors.mag.as_mut() {
            mag.reading.field_ut = yawed_field(mag.reading.field_ut, heading_offset_rad);
        }

        let cmd = board.step_sensors(&sensors);
        plant.step(&cmd, DT_S);
    }

    let estimate = estimate_of(&board);
    let [w, x, y, z] = plant.quat_wxyz;
    let true_yaw = (2.0 * (w * z + x * y)).atan2(1.0 - 2.0 * (y * y + z * z));
    let (_, _, estimated_yaw) = estimate.attitude.to_euler();
    let wrap = |a: f32| {
        (a + core::f32::consts::PI).rem_euclid(core::f32::consts::TAU) - core::f32::consts::PI
    };
    HoverResult {
        position_error_m: [
            estimate.position_ned[0].0 - plant.pos_ned[0],
            estimate.position_ned[1].0 - plant.pos_ned[1],
            estimate.position_ned[2].0 - plant.pos_ned[2],
        ],
        yaw_error_rad: wrap(estimated_yaw - true_yaw),
        height_m: -plant.pos_ned[2],
    }
}

#[test]
fn host_injected_offsets_move_the_estimate_by_those_offsets() {
    let clean = hover_with_host_offsets([0.0; 3], 0.0);
    let gnss_offset = [3.0_f32, -2.0, 0.0];
    let shifted = hover_with_host_offsets(gnss_offset, 0.0);
    let heading_offset = 8.0_f32.to_radians();
    let turned = hover_with_host_offsets([0.0; 3], heading_offset);

    for run in [&clean, &shifted, &turned] {
        assert!(
            (run.height_m - 5.0).abs() < 0.3,
            "hover height {} m",
            run.height_m
        );
    }

    // Without sensor noise the settled hover estimate sits on the truth: the
    // measured residuals are below 1 mm and 0.01 degree. The tolerances of
    // 2 cm and 0.1 degree leave room for another platform's libm and are
    // 100 and 80 times smaller than the injected offsets.
    const POSITION_TOL_M: f32 = 0.02;
    let yaw_tol = 0.1_f32.to_radians();
    for axis in 0..2 {
        assert!(
            clean.position_error_m[axis].abs() < POSITION_TOL_M,
            "clean run position error {:?}",
            clean.position_error_m
        );
        assert!(
            (shifted.position_error_m[axis] - gnss_offset[axis]).abs() < POSITION_TOL_M,
            "GNSS offset {gnss_offset:?} moved the estimate by {:?}",
            shifted.position_error_m
        );
    }
    assert!(
        clean.yaw_error_rad.abs() < yaw_tol,
        "clean yaw error {}",
        clean.yaw_error_rad
    );
    assert!(shifted.yaw_error_rad.abs() < yaw_tol);
    assert!(
        (turned.yaw_error_rad - heading_offset).abs() < yaw_tol,
        "heading offset {heading_offset} rad moved the yaw estimate by {} rad",
        turned.yaw_error_rad
    );
}

/// Level cruise due north at 18 m/s, 50 m above the origin.
fn cruise_truth(step: u64, gravity_mps2: f32) -> DirectTruthSample {
    const SPEED_MPS: f32 = 18.0;
    DirectTruthSample {
        time_us: step * DT_US,
        reset_generation: 0,
        step,
        position_ned_m: [SPEED_MPS * step as f32 * DT_S, 0.0, -50.0],
        attitude_wxyz: [1.0, 0.0, 0.0, 0.0],
        measured: Some(DirectMeasuredKinematics {
            velocity_ned_mps: [SPEED_MPS, 0.0, 0.0],
            body_rates_frd_rps: [0.0; 3],
            specific_force_frd_mps2: [0.0, 0.0, -gravity_mps2],
        }),
    }
}

/// What the GNSS lane carries before the receiver has its first fix.
#[derive(Clone, Copy)]
enum BeforeFirstFix {
    NoSample,
    NoFix,
}

fn airborne_start(before: BeforeFirstFix) -> (StateEstimate, DirectTruthSample) {
    // The first fix arrives 300 ms after the first IMU and magnetometer
    // samples: the lanes of a real sensor set do not start together.
    const FIRST_FIX_STEP: u64 = 120;
    let cfg = test_config(DirectNoiseTier::Off, 3);
    let gravity = normal_gravity(cfg.origin).expect("gravity") as f32;
    let mut board = new_board(cfg);
    let mut host = HostSensorModel::new(&cfg);

    let mut truth = cruise_truth(0, gravity);
    for step in 0..1_200_u64 {
        truth = cruise_truth(step, gravity);
        let mut sensors = host.sample(&truth);
        if step < FIRST_FIX_STEP {
            match before {
                BeforeFirstFix::NoSample => sensors.gnss = None,
                BeforeFirstFix::NoFix => {
                    if let Some(gnss) = sensors.gnss.as_mut() {
                        gnss.reading.fix = GnssFix::None;
                        gnss.reading.position_ned = [0.0; 3];
                        gnss.reading.vel_ned = [0.0; 3];
                    }
                }
            }
        }
        let _ = board.step_sensors(&sensors);
    }
    (estimate_of(&board), truth)
}

#[test]
fn vehicle_that_starts_in_motion_is_seeded_from_its_first_fix() {
    for before in [BeforeFirstFix::NoSample, BeforeFirstFix::NoFix] {
        let (estimate, truth) = airborne_start(before);
        assert!(
            estimate
                .valid_flags
                .contains(StateValidFlags::POSITION | StateValidFlags::VELOCITY),
            "validity {:?}",
            estimate.valid_flags
        );
        // Noise is off and the flight is unaccelerated, so the estimate
        // sits on the truth. A seed at rest would leave an 18 m/s velocity
        // error that the innovation gate never lets the fixes remove; the
        // tolerances are two orders of magnitude below that failure.
        let truth_velocity = truth.measured.expect("measured").velocity_ned_mps;
        let velocity = estimate.velocity_ned.iter().zip(truth_velocity);
        for (estimated, truth_axis) in velocity {
            let velocity_error = estimated.0 - truth_axis;
            assert!(
                velocity_error.abs() < 0.1,
                "velocity error {velocity_error} m/s"
            );
        }
        let position = estimate.position_ned.iter().zip(truth.position_ned_m);
        for (estimated, truth_axis) in position {
            let position_error = estimated.0 - truth_axis;
            assert!(
                position_error.abs() < 0.5,
                "position error {position_error} m"
            );
        }
    }
}
