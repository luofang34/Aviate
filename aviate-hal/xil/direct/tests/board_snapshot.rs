//! A clone of the direct board is a snapshot of the board.
//!
//! A simulator makes a snapshot of the board with `Clone` and continues
//! from the snapshot later. Each test steps a board to an armed flight,
//! makes a clone, and steps the board and the clone on the same link
//! bytes and the same samples. The actuator command, the downlink bytes
//! and the canonical kernel state must be equal in each cycle. The test
//! then puts the snapshot in the place of the board and replays the same
//! inputs: the second pass must repeat the first.

#![allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]

use aviate_app_sitl_gazebo_x500_kernel::build_x500_kernel;
use aviate_core::control::fixed_wing::FixedWingController;
use aviate_core::control::multirotor::MultirotorController;
use aviate_core::control::{ControlMode, VehicleController};
use aviate_core::mixer::fixed_wing::FixedWingMixer;
use aviate_core::mixer::{Mixer, QuadXMixerX500};
use aviate_core::ChannelId;
use aviate_hal_io::{SystemCommand, MAX_ACTUATOR_OUTPUTS};
use aviate_link::mavlink::protocol::{
    position_target_typemask, CommandLong, SetPositionTargetLocalNed,
};
use aviate_link::mavlink::{mav_cmd, serialize_mavlink, MavMessage};
use aviate_xil_direct::{
    DirectActuatorCmd, DirectBoardConfig, DirectBoardHal, DirectHostSensors, DirectMavlinkEndpoint,
    DirectNoiseTier, DirectSynthesizer, DirectTruthSample, DirectXilBoard,
};
use world_core::{DecimalYear, GeodeticPosition};

#[path = "plant/flight.rs"]
#[allow(dead_code)]
mod flight;
use flight::Flight;

#[path = "plant/x500.rs"]
#[allow(dead_code)]
mod x500_plant;
use x500_plant::QuadX500Plant;

const HOST: (u8, u8) = (42, 191);
/// Steps between the snapshot and the end of the comparison.
const COMPARE_STEPS: usize = 600;
/// Last steps before the snapshot in which the test leaves the downlink
/// in the board, so that the snapshot holds bytes that are not sent.
const BACKLOG_STEPS: usize = 30;
/// Size of the buffer for the canonical kernel state.
const STATE_BUF: usize = 8192;

/// Velocity and `yaw` active: a velocity over the ground with a heading.
const VELOCITY_HEADING: u16 = position_target_typemask::X_IGNORE
    | position_target_typemask::Y_IGNORE
    | position_target_typemask::Z_IGNORE
    | position_target_typemask::AX_IGNORE
    | position_target_typemask::AY_IGNORE
    | position_target_typemask::AZ_IGNORE
    | position_target_typemask::YAW_RATE_IGNORE;

fn frame(message: &MavMessage, seq: u8) -> Vec<u8> {
    let mut buf = [0_u8; 280];
    let len = serialize_mavlink(message, seq, HOST.0, HOST.1, &mut buf).expect("serialize");
    buf[..len].to_vec()
}

fn position_target(type_mask: u16, velocity: [f32; 3], z: f32, yaw: f32) -> MavMessage {
    MavMessage::SetPositionTargetLocalNed(SetPositionTargetLocalNed {
        target_system: 1,
        target_component: 1,
        coordinate_frame: 1,
        type_mask,
        z,
        vx: velocity[0],
        vy: velocity[1],
        vz: velocity[2],
        yaw,
        ..SetPositionTargetLocalNed::default()
    })
}

/// Link bytes and samples of one step.
#[derive(Clone)]
struct Inputs<I> {
    rx: Vec<u8>,
    samples: I,
}

/// All that a host can see of one step of a board.
#[derive(Debug, Clone, PartialEq)]
struct Observed {
    cmd: DirectActuatorCmd,
    output_bits: [u32; MAX_ACTUATOR_OUTPUTS],
    downlink: Vec<u8>,
    kernel_state: Vec<u8>,
    algorithm_identity_hash: u64,
    config_hash: u64,
    mode: ControlMode,
    heading_bits: Option<u32>,
    effective_command: String,
}

fn observe<C, M>(board: &mut DirectXilBoard<C, M>, cmd: DirectActuatorCmd) -> Observed
where
    C: VehicleController,
    M: Mixer,
{
    let mut downlink = vec![0_u8; 16_384];
    let sent = board.pop_mavlink_tx(&mut downlink);
    downlink.truncate(sent);
    let mut state = [0_u8; STATE_BUF];
    let snapshot = board
        .kernel()
        .project_for_cross_channel(0, ChannelId(0), &mut state);
    assert!(
        !snapshot.state_bytes.is_empty() && snapshot.state_bytes.len() < STATE_BUF,
        "canonical state of {} bytes",
        snapshot.state_bytes.len()
    );
    let effective = board.last_effective_command();
    Observed {
        cmd,
        output_bits: cmd.outputs.map(f32::to_bits),
        downlink,
        kernel_state: snapshot.state_bytes.to_vec(),
        algorithm_identity_hash: snapshot.algorithm_identity_hash,
        config_hash: snapshot.config_hash,
        mode: effective.mode,
        heading_bits: effective
            .setpoint
            .heading
            .map(|heading| heading.0.to_bits()),
        effective_command: format!("{effective:?}"),
    }
}

/// A board with the host that feeds it.
trait Rig {
    type Controller: VehicleController + Clone;
    type Mixer: Mixer + Clone;
    type Samples: Copy;

    fn board(&mut self) -> &mut DirectXilBoard<Self::Controller, Self::Mixer>;
    /// The link bytes and the samples of the next step.
    fn inputs(&mut self, step: usize) -> Inputs<Self::Samples>;
    fn step_board(
        board: &mut DirectXilBoard<Self::Controller, Self::Mixer>,
        samples: &Self::Samples,
    ) -> DirectActuatorCmd;
    /// Move the plant one step under `cmd`.
    fn apply(&mut self, cmd: DirectActuatorCmd);
}

fn step_observed<R: Rig>(
    board: &mut DirectXilBoard<R::Controller, R::Mixer>,
    inputs: &Inputs<R::Samples>,
) -> Observed {
    assert_eq!(board.push_mavlink_rx(&inputs.rx), inputs.rx.len());
    let cmd = R::step_board(board, &inputs.samples);
    observe(board, cmd)
}

/// Fly `steps` in closed loop. The downlink stays in the board when
/// `keep_downlink` is set.
fn fly<R: Rig>(rig: &mut R, first_step: usize, steps: usize, keep_downlink: bool) {
    let mut scratch = [0_u8; 16_384];
    for step in first_step..first_step + steps {
        let inputs = rig.inputs(step);
        assert_eq!(rig.board().push_mavlink_rx(&inputs.rx), inputs.rx.len());
        let cmd = R::step_board(rig.board(), &inputs.samples);
        if !keep_downlink {
            let _ = rig.board().pop_mavlink_tx(&mut scratch);
        }
        rig.apply(cmd);
    }
}

/// Make a snapshot of the board of `rig`, then check the lockstep of the
/// board and a clone, and the replay from the snapshot. `split_frame` is
/// a link frame of which the snapshot holds only the first half.
///
/// Returns the first pass.
fn assert_snapshot_continues<R: Rig>(
    rig: &mut R,
    first_step: usize,
    split_frame: &[u8],
) -> Vec<Observed> {
    let (head, tail) = split_frame.split_at(split_frame.len() / 2);
    assert_eq!(rig.board().push_mavlink_rx(head), head.len());
    assert!(rig.board().is_armed(), "the snapshot must be of a flight");

    let snapshot = rig.board().clone();
    let mut twin = snapshot.clone();
    let mut probe = snapshot.clone();
    assert!(
        probe.pop_mavlink_tx(&mut [0_u8; 16_384]) > 0,
        "the snapshot must hold downlink bytes that are not sent"
    );

    let mut recorded: Vec<(Inputs<R::Samples>, Observed)> = Vec::new();
    for offset in 0..COMPARE_STEPS {
        let mut inputs = rig.inputs(first_step + offset);
        if offset == 0 {
            inputs.rx = tail.to_vec();
        }
        let first = step_observed::<R>(rig.board(), &inputs);
        let second = step_observed::<R>(&mut twin, &inputs);
        assert_eq!(first, second, "board and clone differ at step {offset}");
        rig.apply(first.cmd);
        recorded.push((inputs, first));
    }

    *rig.board() = snapshot.clone();
    for (offset, (inputs, expected)) in recorded.iter().enumerate() {
        let replayed = step_observed::<R>(rig.board(), inputs);
        assert_eq!(&replayed, expected, "the replay differs at step {offset}");
    }

    // The snapshot itself did not move while its clones flew.
    let mut untouched = snapshot;
    let (inputs, expected) = recorded.first().expect("one step");
    assert_eq!(&step_observed::<R>(&mut untouched, inputs), expected);

    let first_pass: Vec<Observed> = recorded.into_iter().map(|(_, seen)| seen).collect();
    let start = first_pass.first().expect("one step");
    assert!(
        first_pass
            .iter()
            .any(|seen| seen.output_bits != start.output_bits),
        "the outputs never changed: the comparison proves nothing"
    );
    assert!(first_pass.iter().all(|seen| seen.cmd.armed));
    assert!(
        first_pass
            .iter()
            .map(|seen| seen.downlink.len())
            .sum::<usize>()
            > 1_000
    );
    first_pass
}

struct FixedWingRig {
    flight: Flight,
    seq: u8,
}

impl FixedWingRig {
    const AIRSPEED_MPS: f32 = 50.0;

    /// A velocity over the ground along `heading_rad`, with that heading.
    fn target(&mut self, heading_rad: f32) -> Vec<u8> {
        self.seq = self.seq.wrapping_add(1);
        let velocity = [
            Self::AIRSPEED_MPS * heading_rad.cos(),
            Self::AIRSPEED_MPS * heading_rad.sin(),
            0.0,
        ];
        frame(
            &position_target(VELOCITY_HEADING, velocity, 0.0, heading_rad),
            self.seq,
        )
    }
}

impl Rig for FixedWingRig {
    type Controller = FixedWingController;
    type Mixer = FixedWingMixer;
    type Samples = DirectHostSensors;

    fn board(&mut self) -> &mut DirectXilBoard<FixedWingController, FixedWingMixer> {
        &mut self.flight.board
    }

    fn inputs(&mut self, step: usize) -> Inputs<DirectHostSensors> {
        // The heading target turns slowly, so each refresh is a new command.
        let rx = if step.is_multiple_of(25) {
            self.target(0.5 + 0.001 * step as f32)
        } else {
            Vec::new()
        };
        Inputs {
            rx,
            samples: self.flight.next_sensors(),
        }
    }

    fn step_board(
        board: &mut DirectXilBoard<FixedWingController, FixedWingMixer>,
        samples: &DirectHostSensors,
    ) -> DirectActuatorCmd {
        board.step_sensors(samples)
    }

    fn apply(&mut self, cmd: DirectActuatorCmd) {
        self.flight.apply(cmd);
    }
}

#[test]
fn a_fixed_wing_board_clone_continues_and_rewinds_bit_identically() {
    const WARM_STEPS: usize = 500;
    const SPLIT_HEADING_RAD: f32 = 1.25;
    let mut rig = FixedWingRig {
        flight: Flight::c172_with_probe(DirectNoiseTier::Industrial),
        seq: 0,
    };
    let speed = MavMessage::CommandLong(CommandLong {
        command: mav_cmd::DO_CHANGE_SPEED,
        param1: 0.0,
        param2: 50.0,
        param3: -1.0,
        target_system: 1,
        target_component: 1,
        ..CommandLong::default()
    });
    rig.flight.board.push_mavlink_rx(&frame(&speed, 0));
    fly(&mut rig, 0, WARM_STEPS - BACKLOG_STEPS, false);
    fly(&mut rig, WARM_STEPS - BACKLOG_STEPS, BACKLOG_STEPS, true);
    assert_eq!(rig.flight.board.last_command_refusal(), None);
    assert_eq!(
        rig.flight.board.last_effective_command().mode,
        ControlMode::VelocityControl
    );
    assert_eq!(rig.flight.board.endpoint().airspeed_target(), Some(50.0));

    let split = rig.target(SPLIT_HEADING_RAD);
    let first_pass = assert_snapshot_continues(&mut rig, WARM_STEPS + 1, &split);
    // The second half of the split frame came after the snapshot, so the
    // target is flown only if the snapshot kept the first half.
    assert!(first_pass
        .iter()
        .take(25)
        .any(|seen| seen.heading_bits == Some(SPLIT_HEADING_RAD.to_bits())));
}

struct X500Rig {
    board: DirectXilBoard<MultirotorController, QuadXMixerX500>,
    plant: QuadX500Plant,
    step: u64,
    seq: u8,
}

impl X500Rig {
    const DT_US: u64 = 2_500;

    fn target(&mut self, yaw_rad: f32) -> Vec<u8> {
        self.seq = self.seq.wrapping_add(1);
        frame(
            &position_target(VELOCITY_HEADING, [0.8, -0.4, -1.0], 0.0, yaw_rad),
            self.seq,
        )
    }
}

impl Rig for X500Rig {
    type Controller = MultirotorController;
    type Mixer = QuadXMixerX500;
    type Samples = DirectTruthSample;

    fn board(&mut self) -> &mut DirectXilBoard<MultirotorController, QuadXMixerX500> {
        &mut self.board
    }

    fn inputs(&mut self, step: usize) -> Inputs<DirectTruthSample> {
        let rx = if step.is_multiple_of(40) {
            self.target(0.3 + 0.0005 * step as f32)
        } else {
            Vec::new()
        };
        Inputs {
            rx,
            samples: self.plant.truth(self.step * Self::DT_US, 0, self.step),
        }
    }

    fn step_board(
        board: &mut DirectXilBoard<MultirotorController, QuadXMixerX500>,
        samples: &DirectTruthSample,
    ) -> DirectActuatorCmd {
        board.step_truth(samples)
    }

    fn apply(&mut self, cmd: DirectActuatorCmd) {
        self.plant.step(&cmd, 0.0025);
        self.step += 1;
    }
}

#[test]
fn a_multirotor_board_clone_continues_and_rewinds_bit_identically() {
    const WARM_STEPS: usize = 800;
    const SPLIT_YAW_RAD: f32 = 1.1;
    // The board makes its own noisy samples from the truth here, so the
    // snapshot must also hold the state of the noise generator.
    let config = DirectBoardConfig {
        origin: GeodeticPosition::wgs84(39.1582, -75.5244, 15.0),
        epoch: DecimalYear(2026.5),
        noise: DirectNoiseTier::Industrial,
        noise_seed: 99,
        sys_id: 1,
        comp_id: 1,
    };
    let board = DirectXilBoard::new(build_x500_kernel().expect("kernel"), config).expect("board");
    let mut rig = X500Rig {
        board,
        plant: QuadX500Plant::new_on_pad(config.origin),
        step: 0,
        seq: 0,
    };
    while !rig.board.is_ready() {
        assert!(rig.step < 2_000, "not ready");
        let truth = rig.plant.truth(rig.step * X500Rig::DT_US, 0, rig.step);
        let _ = rig.board.step_truth(&truth);
        rig.step += 1;
    }
    let first_target = rig.target(0.3);
    rig.board.push_mavlink_rx(&first_target);
    rig.board.send_command(SystemCommand::Arm);
    fly(&mut rig, 1, WARM_STEPS - BACKLOG_STEPS, false);
    fly(
        &mut rig,
        1 + WARM_STEPS - BACKLOG_STEPS,
        BACKLOG_STEPS,
        true,
    );
    assert_eq!(
        rig.board.last_effective_command().mode,
        ControlMode::VelocityControl
    );

    let split = rig.target(SPLIT_YAW_RAD);
    let first_pass = assert_snapshot_continues(&mut rig, WARM_STEPS + 2, &split);
    assert!(first_pass
        .iter()
        .take(40)
        .any(|seen| seen.heading_bits == Some(SPLIT_YAW_RAD.to_bits())));
}

fn assert_clone<T: Clone>() {}

/// A field that is not `Clone` in the board stops this test at compile
/// time, with the name of the type in the error.
#[test]
fn the_board_object_graph_is_clone() {
    assert_clone::<DirectXilBoard<FixedWingController, FixedWingMixer>>();
    assert_clone::<DirectXilBoard<MultirotorController, QuadXMixerX500>>();
    assert_clone::<DirectBoardHal>();
    assert_clone::<DirectMavlinkEndpoint>();
    assert_clone::<DirectSynthesizer>();
}
