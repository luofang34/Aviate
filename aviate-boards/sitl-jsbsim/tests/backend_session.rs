//! The typed simulator backend against a lockstep scripted sidecar on
//! loopback TCP: a reset is a new session and a new world, a stopped
//! session gives no frames, and arming waits for a ready session.

#![allow(clippy::expect_used, clippy::panic)]

use std::io::{ErrorKind, Read, Write};
use std::net::{SocketAddr, TcpListener, TcpStream};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use aviate_backend_mavlink_hil::{parse_frame, serialize_frame, HilMessage, HilStateQuaternion};
use aviate_board_sitl_jsbsim::{JsbsimBoard, JsbsimConfig, JsbsimSimulatorBackend};
use aviate_core::control::multirotor::MultirotorController;
use aviate_core::mixer::{ActuatorCmd, QuadXMixerX500};
use aviate_hal_xil::synthesis::GeoOrigin;
use aviate_hal_xil::{
    DirectiveId, DirectiveOutcome, FrameEvent, ResetGeneration, SimActuatorCmd, SimulatorBackend,
    SimulatorDirective, SimulatorDirectiveKind, SimulatorError, SimulatorFrame, SimulatorLifecycle,
    SitlConfig, XilNetConfig,
};
use world_core::DecimalYear;

const STEP_US: u64 = 4_000;
const TIMEOUT: Duration = Duration::from_secs(5);

type Backend = JsbsimSimulatorBackend<MultirotorController, QuadXMixerX500>;

fn idle_map(_: &ActuatorCmd, armed: bool) -> SimActuatorCmd {
    SimActuatorCmd {
        count: 4,
        armed,
        ..SimActuatorCmd::default()
    }
}

/// A scripted sidecar: each accepted session is a new world whose clock
/// starts at 4 ms, and each state waits for its reply like JSBSim's
/// lockstep (2 s timeout) before the next.
struct Sidecar {
    address: SocketAddr,
    stop: Arc<AtomicBool>,
    sessions: Arc<AtomicU64>,
    thread: Option<JoinHandle<()>>,
}

impl Sidecar {
    fn start() -> Self {
        let listener = TcpListener::bind("127.0.0.1:0").expect("bind sidecar");
        listener.set_nonblocking(true).expect("nonblocking accept");
        let address = listener.local_addr().expect("address");
        let stop = Arc::new(AtomicBool::new(false));
        let sessions = Arc::new(AtomicU64::new(0));
        let (thread_stop, thread_sessions) = (stop.clone(), sessions.clone());
        let thread = std::thread::spawn(move || {
            while !thread_stop.load(Ordering::Acquire) {
                match listener.accept() {
                    Ok((stream, _)) => {
                        thread_sessions.fetch_add(1, Ordering::AcqRel);
                        run_session(stream, &thread_stop);
                    }
                    Err(e) if e.kind() == ErrorKind::WouldBlock => {
                        std::thread::sleep(Duration::from_millis(2));
                    }
                    Err(e) => panic!("accept: {e}"),
                }
            }
        });
        Self {
            address,
            stop,
            sessions,
            thread: Some(thread),
        }
    }
}

impl Drop for Sidecar {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Release);
        if let Some(thread) = self.thread.take() {
            thread.join().ok();
        }
    }
}

fn run_session(mut stream: TcpStream, stop: &AtomicBool) {
    stream.set_nonblocking(false).expect("blocking stream");
    stream
        .set_read_timeout(Some(Duration::from_millis(20)))
        .expect("read timeout");
    let origin = GeoOrigin::DEFAULT.position;
    let mut rx = Vec::new();
    let mut time = STEP_US;
    let mut seq = 0u8;
    while !stop.load(Ordering::Acquire) {
        let state = HilStateQuaternion {
            time_usec: time,
            attitude_quaternion: [1.0, 0.0, 0.0, 0.0],
            lat: (origin.latitude_deg * 1e7).round() as i32,
            lon: (origin.longitude_deg * 1e7).round() as i32,
            alt: (origin.ellipsoidal_height_m * 1e3).round() as i32,
            zacc: -1_000,
            ..HilStateQuaternion::default()
        };
        let mut frame = [0u8; 280];
        let len = serialize_frame(&HilMessage::StateQuaternion(state), seq, 1, 51, &mut frame)
            .expect("frame fits");
        seq = seq.wrapping_add(1);
        if stream.write_all(&frame[..len]).is_err() {
            return;
        }
        if !await_reply(&mut stream, &mut rx, time, stop) {
            return;
        }
        time += STEP_US;
    }
}

/// Wait up to 2 s for the reply to `time`; false when the session ended.
fn await_reply(stream: &mut TcpStream, rx: &mut Vec<u8>, time: u64, stop: &AtomicBool) -> bool {
    let deadline = Instant::now() + Duration::from_secs(2);
    loop {
        while let Ok((frame, used)) = parse_frame(rx) {
            rx.drain(..used);
            if let HilMessage::ActuatorControls(reply) = frame.message {
                if reply.time_usec == time {
                    return true;
                }
            }
        }
        if Instant::now() > deadline || stop.load(Ordering::Acquire) {
            return true;
        }
        let mut buf = [0u8; 512];
        match stream.read(&mut buf) {
            Ok(0) => return false,
            Ok(n) => rx.extend_from_slice(&buf[..n]),
            Err(e) if matches!(e.kind(), ErrorKind::WouldBlock | ErrorKind::TimedOut) => {}
            Err(_) => return false,
        }
    }
}

fn backend_for(sidecar: &Sidecar) -> Backend {
    let mut config = JsbsimConfig::for_instance(0, GeoOrigin::DEFAULT, DecimalYear(2026.8));
    config.sidecar_addr = sidecar.address;
    config.xil = SitlConfig::for_instance_with_net(
        0,
        XilNetConfig {
            base_port: 0,
            stride: 0,
        },
    );
    let kernel = aviate_app_sitl_gazebo_x500_kernel::build_x500_kernel().expect("x500 kernel");
    let board = JsbsimBoard::new(kernel, config, idle_map).expect("board builds");
    JsbsimSimulatorBackend::new(board, 0)
}

fn directive(
    id: u64,
    generation: ResetGeneration,
    kind: SimulatorDirectiveKind,
) -> SimulatorDirective {
    SimulatorDirective {
        id: DirectiveId(id),
        generation,
        kind,
    }
}

fn frame(backend: &mut Backend) -> SimulatorFrame {
    match backend.next_frame(TIMEOUT).expect("a frame") {
        FrameEvent::Frame(frame) => frame,
        FrameEvent::TimedOut { .. } => panic!("the sidecar stopped stepping"),
    }
}

#[test]
fn a_reset_starts_a_new_world_through_a_new_session() {
    let sidecar = Sidecar::start();
    let mut backend = backend_for(&sidecar);
    let connected = backend.connect(0, TIMEOUT).expect("connects");
    let first = connected.generation;

    let mut last = frame(&mut backend);
    for _ in 0..20 {
        last = frame(&mut backend);
    }
    assert_eq!(last.generation, first);
    assert!(last.step >= 20);
    assert!(last.vehicle.valid, "the frame states the vehicle");

    let receipt = backend
        .execute(directive(1, first, SimulatorDirectiveKind::Reset), TIMEOUT)
        .expect("reset");
    assert_eq!(receipt.outcome, DirectiveOutcome::ResetAccepted);
    assert_eq!(receipt.generation, first.next());
    assert_eq!(sidecar.sessions.load(Ordering::Acquire), 2, "a new session");
    assert_ne!(backend.status().lifecycle, SimulatorLifecycle::Resetting);

    // The old generation's directives are refused.
    assert!(matches!(
        backend.execute(directive(2, first, SimulatorDirectiveKind::Start), TIMEOUT),
        Err(SimulatorError::StaleGeneration { .. })
    ));
    let after = frame(&mut backend);
    assert_eq!(after.generation, first.next());
    assert!(after.step < 10, "the new world's steps start over");
    assert!(after.simulation_time < last.simulation_time);
}

#[test]
fn a_stopped_session_gives_no_frames_until_started() {
    let sidecar = Sidecar::start();
    let mut backend = backend_for(&sidecar);
    let generation = backend.connect(0, TIMEOUT).expect("connects").generation;
    frame(&mut backend);

    let stopped = backend
        .execute(
            directive(1, generation, SimulatorDirectiveKind::Stop),
            TIMEOUT,
        )
        .expect("stop");
    assert_eq!(stopped.outcome, DirectiveOutcome::Stopped);
    assert!(matches!(
        backend.next_frame(TIMEOUT),
        Err(SimulatorError::InvalidLifecycle {
            lifecycle: SimulatorLifecycle::Stopped,
            ..
        })
    ));
    backend
        .execute(
            directive(2, generation, SimulatorDirectiveKind::Start),
            TIMEOUT,
        )
        .expect("start");
    assert_eq!(frame(&mut backend).generation, generation);
}

#[test]
fn arming_waits_for_a_ready_session_and_disarm_follows() {
    let sidecar = Sidecar::start();
    let mut backend = backend_for(&sidecar);
    let generation = backend.connect(0, TIMEOUT).expect("connects").generation;
    // The estimator has not converged on the first state of a world.
    assert_eq!(
        frame(&mut backend).lifecycle,
        SimulatorLifecycle::Converging
    );
    assert!(matches!(
        backend.execute(
            directive(1, generation, SimulatorDirectiveKind::Arm),
            TIMEOUT
        ),
        Err(SimulatorError::ReadinessFailed { .. })
    ));

    // The estimator converges on the still vehicle within a minute of
    // simulation time.
    let mut ready = false;
    for _ in 0..15_000 {
        if frame(&mut backend).lifecycle == SimulatorLifecycle::Ready {
            ready = true;
            break;
        }
    }
    assert!(ready, "the session never became ready");
    let armed = backend
        .execute(
            directive(2, generation, SimulatorDirectiveKind::Arm),
            TIMEOUT,
        )
        .expect("arms");
    assert_eq!(armed.outcome, DirectiveOutcome::Armed);
    assert!(frame(&mut backend).armed);
    backend
        .execute(
            directive(3, generation, SimulatorDirectiveKind::Disarm),
            TIMEOUT,
        )
        .expect("disarms");
    assert!(!frame(&mut backend).armed);
}
