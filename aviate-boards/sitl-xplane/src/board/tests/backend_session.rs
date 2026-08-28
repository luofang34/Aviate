//! Run backend directives on the TCP sample-paced transport.
#![allow(clippy::expect_used, clippy::panic)]

use std::io::{Read, Write};
use std::net::{SocketAddr, TcpListener, TcpStream};
use std::sync::mpsc::{self, Receiver, Sender};
use std::thread::JoinHandle;
use std::time::Duration;

use aviate_backend_mavlink_hil::{
    parse_frame, serialize_frame, HilActuatorControls, HilGps, HilMessage, HilSensor,
    HilStateQuaternion, RESET_ACK_SENSOR_FLAG, RESET_REQUEST_ACTUATOR_FLAG,
};
use aviate_core::control::{Command, CommandSource, ControlMode, Setpoint};
use aviate_core::math::Quaternion;
use aviate_core::mixer::QuadXMixerReversedSpin;
use aviate_core::types::NormalizedThrust;
use aviate_hal_xil::{
    DirectiveId, FrameEvent, ResetGeneration, SimulatorBackend, SimulatorDirective,
    SimulatorDirectiveKind, SimulatorError, SimulatorFrame, SimulatorLifecycle,
};

use crate::XPlaneSimulatorBackend;
mod backend_factory;
mod coherence;
mod reset_timeout;
mod runner;

use backend_factory::make_backend;

type AliaBackend = XPlaneSimulatorBackend<
    aviate_core::control::multirotor::MultirotorController,
    QuadXMixerReversedSpin,
>;

const INITIAL_LATITUDE: i32 = 400_000_000;

enum BridgeCommand {
    PrimeReset { state: HilStateQuaternion },
    HoldReset { observed: Sender<()> },
    ReleaseReset,
    Sample(u64),
    State(HilStateQuaternion),
    Burst([HilStateQuaternion; 2]),
    Stop,
}

#[test]
fn scripted_session_uses_the_sample_paced_transport() {
    let (address, bridge, bridge_thread) = start_bridge();
    let mut backend = make_backend(address);
    let connected = backend
        .connect(0, Duration::from_secs(1))
        .expect("backend connects");
    let mut directive_id = 0_u64;

    let first = run_generation(
        &mut backend,
        &bridge,
        &mut directive_id,
        connected.generation,
    );
    let second = run_generation(&mut backend, &bridge, &mut directive_id, first.generation);
    let failed_generation =
        assert_moved_reset_is_refused(&mut backend, &bridge, &mut directive_id, second.generation);
    let third = run_generation(&mut backend, &bridge, &mut directive_id, failed_generation);

    assert_eq!(first.vehicle, second.vehicle);
    assert_eq!(first.vehicle, third.vehicle);
    assert_eq!(first.step, second.step);
    assert_eq!(first.step, third.step);
    assert_eq!(first.simulation_time, second.simulation_time);
    assert_eq!(first.simulation_time, third.simulation_time);
    bridge.send(BridgeCommand::Stop).expect("stop bridge");
    bridge_thread.join().expect("bridge thread finishes");
    execute(
        &mut backend,
        &mut directive_id,
        third.generation,
        SimulatorDirectiveKind::Start,
    );
    let lost = backend
        .next_frame(Duration::from_secs(1))
        .expect_err("closed bridge must fail");
    assert!(matches!(
        lost,
        aviate_hal_xil::SimulatorError::BridgeLost { .. }
    ));
    drop(backend);
    verify_post_ready_clock_failure();
    runner::verify_mission_runner_session();
    reset_timeout::verify_timed_out_reset_refuses_retry();
    coherence::verify_buffered_groups_are_coherent();
}

fn verify_post_ready_clock_failure() {
    let (address, bridge, bridge_thread) = start_bridge();
    let mut backend = make_backend(address);
    let connected = backend
        .connect(0, Duration::from_secs(1))
        .expect("backend connects");
    let mut directive_id = 0_u64;
    bridge
        .send(BridgeCommand::PrimeReset {
            state: truth(0, INITIAL_LATITUDE),
        })
        .expect("prime reset sample");
    let reset = execute(
        &mut backend,
        &mut directive_id,
        connected.generation,
        SimulatorDirectiveKind::Reset,
    );
    execute(
        &mut backend,
        &mut directive_id,
        reset.generation,
        SimulatorDirectiveKind::Start,
    );
    let ready = converge(&mut backend, &bridge, reset.generation);
    execute(
        &mut backend,
        &mut directive_id,
        reset.generation,
        SimulatorDirectiveKind::Arm,
    );
    let timestamp = u64::try_from(ready.simulation_time.as_micros()).expect("time fits u64");
    bridge
        .send(BridgeCommand::Sample(timestamp))
        .expect("send duplicate sample");
    let FrameEvent::Frame(failed) = backend
        .next_frame(Duration::from_secs(1))
        .expect("failure frame")
    else {
        panic!("expected a frame");
    };
    assert_eq!(failed.lifecycle, SimulatorLifecycle::Converging);
    assert!(!failed.armed);
    assert_readiness_failure(&mut backend, &mut directive_id, reset.generation);
    bridge.send(BridgeCommand::Stop).expect("stop bridge");
    bridge_thread.join().expect("bridge thread finishes");
}

fn run_generation(
    backend: &mut AliaBackend,
    bridge: &Sender<BridgeCommand>,
    directive_id: &mut u64,
    generation: ResetGeneration,
) -> SimulatorFrame {
    bridge
        .send(BridgeCommand::PrimeReset {
            state: truth(0, INITIAL_LATITUDE),
        })
        .expect("prime reset sample");
    let reset = execute(
        backend,
        directive_id,
        generation,
        SimulatorDirectiveKind::Reset,
    );
    let generation = reset.generation;
    execute(
        backend,
        directive_id,
        generation,
        SimulatorDirectiveKind::Start,
    );
    assert_readiness_failure(backend, directive_id, generation);
    let initial = converge(backend, bridge, generation);
    execute(
        backend,
        directive_id,
        generation,
        SimulatorDirectiveKind::CheckArmReadiness,
    );
    assert_arm_refusal(backend, directive_id, generation);
    execute(
        backend,
        directive_id,
        generation,
        SimulatorDirectiveKind::Arm,
    );
    let initial_time = u64::try_from(initial.simulation_time.as_micros())
        .expect("initial simulation time fits u64");
    for sequence in 0..2 {
        execute(
            backend,
            directive_id,
            generation,
            SimulatorDirectiveKind::Setpoint(setpoint(sequence)),
        );
        bridge
            .send(BridgeCommand::Sample(initial_time.saturating_add(
                u64::from(sequence).wrapping_add(1).saturating_mul(10_000),
            )))
            .expect("send setpoint sample");
        let _frame = backend
            .next_frame(Duration::from_secs(1))
            .expect("setpoint frame");
    }
    execute(
        backend,
        directive_id,
        generation,
        SimulatorDirectiveKind::Disarm,
    );
    assert_readiness_failure(backend, directive_id, generation);
    execute(
        backend,
        directive_id,
        generation,
        SimulatorDirectiveKind::Stop,
    );
    initial
}

fn assert_moved_reset_is_refused(
    backend: &mut AliaBackend,
    bridge: &Sender<BridgeCommand>,
    directive_id: &mut u64,
    generation: ResetGeneration,
) -> ResetGeneration {
    bridge
        .send(BridgeCommand::PrimeReset {
            state: truth(0, INITIAL_LATITUDE + 100),
        })
        .expect("prime moved reset sample");
    let reset = execute(
        backend,
        directive_id,
        generation,
        SimulatorDirectiveKind::Reset,
    );
    execute(
        backend,
        directive_id,
        reset.generation,
        SimulatorDirectiveKind::Start,
    );
    for sample in 1..=500_u64 {
        bridge
            .send(BridgeCommand::Sample(sample * 10_000))
            .expect("send moved convergence sample");
        match backend.next_frame(Duration::from_secs(1)) {
            Err(SimulatorError::ReadinessFailed { .. }) => return reset.generation,
            Ok(FrameEvent::Frame(_)) => {}
            result => panic!("unexpected moved reset result: {result:?}"),
        }
    }
    panic!("the moved simulator reached Ready");
}

fn assert_readiness_failure(
    backend: &mut AliaBackend,
    next_id: &mut u64,
    generation: ResetGeneration,
) {
    let directive = next_directive(
        next_id,
        generation,
        SimulatorDirectiveKind::CheckArmReadiness,
    );
    let error = backend
        .execute(directive, Duration::from_secs(1))
        .expect_err("current state must refuse readiness");
    assert!(matches!(error, SimulatorError::ReadinessFailed { .. }));
}

fn assert_arm_refusal(backend: &mut AliaBackend, next_id: &mut u64, generation: ResetGeneration) {
    backend.board_mut().runner.kernel.state.init_state = aviate_core::InitState::PreArm;
    let directive = next_directive(next_id, generation, SimulatorDirectiveKind::Arm);
    let error = backend
        .execute(directive, Duration::from_secs(1))
        .expect_err("kernel must refuse arm while not ready");
    assert!(matches!(error, SimulatorError::ArmRefused { .. }));
    backend.board_mut().runner.kernel.state.init_state = aviate_core::InitState::Ready;
}

fn converge(
    backend: &mut AliaBackend,
    bridge: &Sender<BridgeCommand>,
    generation: ResetGeneration,
) -> SimulatorFrame {
    for sample in 1..=500_u64 {
        bridge
            .send(BridgeCommand::Sample(sample * 10_000))
            .expect("send convergence sample");
        let event = backend
            .next_frame(Duration::from_secs(1))
            .expect("convergence frame");
        if let FrameEvent::Frame(frame) = event {
            assert_eq!(frame.generation, generation);
            if frame.lifecycle == SimulatorLifecycle::Ready {
                return frame;
            }
        }
    }
    panic!(
        "backend did not become ready: status={:?}, runtime_failure={:?}, kernel_state={:?}",
        backend.status(),
        backend.board().runtime_handshake_failure(),
        backend.board().kernel().state.init_state
    );
}

fn execute(
    backend: &mut AliaBackend,
    next_id: &mut u64,
    generation: ResetGeneration,
    kind: SimulatorDirectiveKind,
) -> aviate_hal_xil::DirectiveReceipt {
    let directive = next_directive(next_id, generation, kind);
    backend
        .execute(directive, Duration::from_secs(1))
        .expect("directive succeeds")
}

fn next_directive(
    next_id: &mut u64,
    generation: ResetGeneration,
    kind: SimulatorDirectiveKind,
) -> SimulatorDirective {
    let id = DirectiveId(*next_id);
    *next_id = next_id.wrapping_add(1);
    SimulatorDirective {
        id,
        generation,
        kind,
    }
}

fn setpoint(sequence: u32) -> Command {
    Command {
        mode: ControlMode::Attitude,
        setpoint: Setpoint {
            attitude: Some(Quaternion::IDENTITY),
            collective_thrust: NormalizedThrust(0.0),
            ..Setpoint::default()
        },
        config_mode_request: None,
        sensor_overrides: None,
        sequence,
        source: CommandSource::Gcs,
    }
}

fn start_bridge() -> (SocketAddr, Sender<BridgeCommand>, JoinHandle<()>) {
    let listener = TcpListener::bind("127.0.0.1:0").expect("bind bridge");
    let address = listener.local_addr().expect("bridge address");
    let (sender, receiver) = mpsc::channel();
    let handle = std::thread::spawn(move || run_bridge(listener, receiver));
    (address, sender, handle)
}

fn run_bridge(listener: TcpListener, receiver: Receiver<BridgeCommand>) {
    let (mut stream, _) = listener.accept().expect("accept flight controller");
    stream
        .set_read_timeout(Some(Duration::from_secs(1)))
        .expect("set bridge timeout");
    let mut sequence = 0_u8;
    let mut latitude = INITIAL_LATITUDE;
    let mut actuator_buffer = Vec::new();
    while let Ok(command) = receiver.recv() {
        match command {
            BridgeCommand::PrimeReset { state } => {
                send_sample_group(&mut stream, 9_000_000, false, latitude, &mut sequence);
                let request = read_actuator_answer(&mut stream, &mut actuator_buffer);
                assert_ne!(request.flags & RESET_REQUEST_ACTUATOR_FLAG, 0);
                latitude = state.lat;
                send_truth_group(&mut stream, state, true, &mut sequence);
                read_non_reset_answer(&mut stream, &mut actuator_buffer);
            }
            BridgeCommand::HoldReset { observed } => {
                let request = read_actuator_answer(&mut stream, &mut actuator_buffer);
                assert_ne!(request.flags & RESET_REQUEST_ACTUATOR_FLAG, 0);
                observed.send(()).expect("report reset request");
                assert!(matches!(receiver.recv(), Ok(BridgeCommand::ReleaseReset)));
                send_sample_group(&mut stream, 0, true, latitude, &mut sequence);
                read_non_reset_answer(&mut stream, &mut actuator_buffer);
            }
            BridgeCommand::ReleaseReset => panic!("reset release has no held request"),
            BridgeCommand::Sample(timestamp) => {
                send_sample_group(&mut stream, timestamp, false, latitude, &mut sequence);
                read_non_reset_answer(&mut stream, &mut actuator_buffer);
            }
            BridgeCommand::State(state) => {
                send_truth_group(&mut stream, state, false, &mut sequence);
                read_non_reset_answer(&mut stream, &mut actuator_buffer);
            }
            BridgeCommand::Burst(states) => {
                coherence::send_burst(&mut stream, states, &mut sequence, &mut actuator_buffer)
            }
            BridgeCommand::Stop => return,
        }
    }
}

fn send_sample_group(
    stream: &mut TcpStream,
    timestamp: u64,
    reset: bool,
    latitude: i32,
    sequence: &mut u8,
) {
    send_truth_group(stream, truth(timestamp, latitude), reset, sequence);
}

fn send_truth_group(
    stream: &mut TcpStream,
    state: HilStateQuaternion,
    reset: bool,
    sequence: &mut u8,
) {
    let messages = [
        HilMessage::Gps(gps(state.time_usec, state.lat)),
        HilMessage::StateQuaternion(state),
        HilMessage::Sensor(sensor(state.time_usec, reset)),
    ];
    let mut group = Vec::new();
    for message in messages {
        let mut frame = [0_u8; 300];
        let length =
            serialize_frame(&message, *sequence, 1, 1, &mut frame).expect("message fits frame");
        *sequence = sequence.wrapping_add(1);
        group.extend_from_slice(&frame[..length]);
    }
    stream.write_all(&group).expect("send sample group");
}

fn read_non_reset_answer(stream: &mut TcpStream, buffer: &mut Vec<u8>) {
    loop {
        let answer = read_actuator_answer(stream, buffer);
        if answer.flags & RESET_REQUEST_ACTUATOR_FLAG == 0 {
            return;
        }
    }
}

fn read_actuator_answer(stream: &mut TcpStream, buffer: &mut Vec<u8>) -> HilActuatorControls {
    loop {
        if let Ok((frame, consumed)) = parse_frame(buffer) {
            buffer.drain(..consumed);
            if let HilMessage::ActuatorControls(controls) = frame.message {
                return controls;
            }
            continue;
        }
        let mut chunk = [0_u8; 300];
        let length = stream.read(&mut chunk).expect("read actuator answer");
        assert!(length > 0);
        buffer.extend_from_slice(&chunk[..length]);
    }
}

fn sensor(timestamp: u64, reset: bool) -> HilSensor {
    HilSensor {
        time_usec: timestamp,
        zacc: -9.806_65,
        xmag: 0.2,
        zmag: 0.4,
        abs_pressure: 1_013.25,
        pressure_alt: 100.0,
        temperature: 20.0,
        fields_updated: 0x0fff | if reset { RESET_ACK_SENSOR_FLAG } else { 0 },
        ..HilSensor::default()
    }
}

fn gps(timestamp: u64, latitude: i32) -> HilGps {
    HilGps {
        time_usec: timestamp,
        lat: latitude,
        lon: -1_050_000_000,
        alt: 100_000,
        eph: 50,
        epv: 75,
        fix_type: 3,
        satellites_visible: 12,
        ..HilGps::default()
    }
}

fn truth(timestamp: u64, latitude: i32) -> HilStateQuaternion {
    HilStateQuaternion {
        time_usec: timestamp,
        attitude_quaternion: [1.0, 0.0, 0.0, 0.0],
        lat: latitude,
        lon: -1_050_000_000,
        alt: 100_000,
        zacc: -1_000,
        ..HilStateQuaternion::default()
    }
}
