//! The board against a scripted JSBSim sidecar on loopback TCP: one
//! reply per truth state, echoing its time, and a new world for a new
//! session or rewound time.

#![allow(clippy::expect_used, clippy::panic)]

use std::io::{ErrorKind, Read, Write};
use std::net::{TcpListener, TcpStream};
use std::time::{Duration, Instant};

use aviate_backend_mavlink_hil::{
    parse_frame, serialize_frame, HilMessage, HilStateQuaternion, LOCKSTEP_ACTUATOR_FLAG,
};
use aviate_board_sitl_jsbsim::{ActuatorMap, JsbsimBoard, JsbsimConfig, TruthStep};
use aviate_core::control::multirotor::MultirotorController;
use aviate_core::mixer::{ActuatorCmd, QuadXMixerX500};
use aviate_hal_xil::synthesis::GeoOrigin;
use aviate_hal_xil::{SimActuatorCmd, SitlConfig, XilNetConfig};
use world_core::DecimalYear;

/// The sidecar's physics step.
const STEP_US: u64 = 4_000;
/// JSBSim starts each session's clock here.
const FIRST_TIME_US: u64 = 4_000;

type Board = JsbsimBoard<MultirotorController, QuadXMixerX500>;

fn identity_map(command: &ActuatorCmd, armed: bool) -> SimActuatorCmd {
    let mut outputs = [0.0; 16];
    for (out, cmd) in outputs.iter_mut().zip(command.outputs.iter().take(4)) {
        *out = cmd.0;
    }
    SimActuatorCmd {
        outputs,
        count: 4,
        armed,
        ..SimActuatorCmd::default()
    }
}

/// A map that drives every engine at 0.3 whatever the mixer says, so a
/// reply that did not come from a kernel cycle is visible.
fn constant_map(_: &ActuatorCmd, armed: bool) -> SimActuatorCmd {
    let mut outputs = [0.0; 16];
    outputs[..4].fill(0.3);
    SimActuatorCmd {
        outputs,
        count: 4,
        armed,
        ..SimActuatorCmd::default()
    }
}

fn board_for(listener: &TcpListener, map: ActuatorMap) -> Board {
    let mut config = JsbsimConfig::for_instance(0, GeoOrigin::DEFAULT, DecimalYear(2026.8));
    config.sidecar_addr = listener.local_addr().expect("listener address");
    // Port 0 lets the OS pick the GCS port, so tests never contend.
    config.xil = SitlConfig::for_instance_with_net(
        0,
        XilNetConfig {
            base_port: 0,
            stride: 0,
        },
    );
    let kernel = aviate_app_sitl_gazebo_x500_kernel::build_x500_kernel().expect("x500 kernel");
    JsbsimBoard::new(kernel, config, map).expect("board builds")
}

/// The scripted sidecar's end of one TCP session.
struct Session {
    stream: TcpStream,
    rx: Vec<u8>,
    seq: u8,
}

impl Session {
    /// A level vehicle at rest at the world origin, at `time_usec`.
    fn send_state(&mut self, time_usec: u64) {
        let origin = GeoOrigin::DEFAULT.position;
        let state = HilStateQuaternion {
            time_usec,
            attitude_quaternion: [1.0, 0.0, 0.0, 0.0],
            lat: (origin.latitude_deg * 1e7).round() as i32,
            lon: (origin.longitude_deg * 1e7).round() as i32,
            alt: (origin.ellipsoidal_height_m * 1e3).round() as i32,
            zacc: -1_000,
            ..HilStateQuaternion::default()
        };
        let mut frame = [0u8; 280];
        let len = serialize_frame(
            &HilMessage::StateQuaternion(state),
            self.seq,
            1,
            51,
            &mut frame,
        )
        .expect("frame fits");
        self.seq = self.seq.wrapping_add(1);
        self.stream.write_all(&frame[..len]).expect("state sent");
    }

    /// The next actuator reply: echoed time, flags, first four controls.
    fn reply_by(&mut self, deadline: Instant) -> Option<(u64, u64, [f32; 4])> {
        loop {
            while let Ok((frame, used)) = parse_frame(&self.rx) {
                self.rx.drain(..used);
                if let HilMessage::ActuatorControls(reply) = frame.message {
                    let mut controls = [0.0; 4];
                    controls.copy_from_slice(&reply.controls[..4]);
                    return Some((reply.time_usec, reply.flags, controls));
                }
            }
            if Instant::now() > deadline {
                return None;
            }
            let mut buf = [0u8; 512];
            match self.stream.read(&mut buf) {
                Ok(0) => return None,
                Ok(n) => self.rx.extend_from_slice(&buf[..n]),
                Err(e) if matches!(e.kind(), ErrorKind::WouldBlock | ErrorKind::TimedOut) => {}
                Err(e) => panic!("sidecar read: {e}"),
            }
        }
    }
}

/// One lockstep exchange: send a state, let the board take and answer
/// it, and read the answer.
fn exchange(board: &mut Board, session: &mut Session, time_usec: u64) -> (TruthStep, [f32; 4]) {
    session.send_state(time_usec);
    let deadline = Instant::now() + Duration::from_secs(5);
    let step = loop {
        board.wait_for_truth(Duration::from_millis(20));
        if let Some(step) = board.step() {
            break step;
        }
        assert!(Instant::now() < deadline, "the board never took the state");
    };
    let (echo, flags, controls) = session
        .reply_by(Instant::now() + Duration::from_secs(5))
        .expect("every state is answered");
    assert_eq!(echo, time_usec, "the reply echoes the state's time");
    assert_eq!(flags & LOCKSTEP_ACTUATOR_FLAG, LOCKSTEP_ACTUATOR_FLAG);
    assert_eq!(step.reply.map(|r| r.echoed_timestamp_us), Some(time_usec));
    (step, controls)
}

/// Accept the board's dial, stepping the board so its transport
/// retries the connection.
fn connect(board: &mut Board, listener: &TcpListener) -> Session {
    listener.set_nonblocking(true).expect("nonblocking accept");
    let deadline = Instant::now() + Duration::from_secs(5);
    loop {
        if let Ok((stream, _)) = listener.accept() {
            stream.set_nonblocking(false).expect("blocking stream");
            stream
                .set_read_timeout(Some(Duration::from_millis(20)))
                .expect("read timeout");
            return Session {
                stream,
                rx: Vec::new(),
                seq: 0,
            };
        }
        assert!(Instant::now() < deadline, "the board never dialed");
        assert!(board.step().is_none(), "no truth before a session");
        board.wait_for_truth(Duration::from_millis(10));
    }
}

#[test]
fn every_state_gets_one_lockstep_reply_echoing_its_time() {
    let listener = TcpListener::bind("127.0.0.1:0").expect("bind sidecar");
    let mut board = board_for(&listener, identity_map);
    let mut session = connect(&mut board, &listener);

    let steps: Vec<TruthStep> = (0..25)
        .map(|i| exchange(&mut board, &mut session, FIRST_TIME_US + i * STEP_US).0)
        .collect();
    // The first state of the world starts no step; every later one runs
    // one kernel cycle over the sidecar's 4 ms.
    assert_eq!(steps[0].kernel_delta_us, None);
    assert!(steps[1..]
        .iter()
        .all(|s| s.kernel_delta_us == Some(STEP_US)));
    assert!(steps.iter().all(|s| s.generation == steps[0].generation));
}

#[test]
fn rewound_time_and_a_new_session_are_new_worlds() {
    let listener = TcpListener::bind("127.0.0.1:0").expect("bind sidecar");
    let mut board = board_for(&listener, identity_map);
    let mut session = connect(&mut board, &listener);

    let (first, _) = exchange(&mut board, &mut session, FIRST_TIME_US);
    exchange(&mut board, &mut session, FIRST_TIME_US + STEP_US);

    // The sidecar resets its world in place: time starts over.
    let (rewound, _) = exchange(&mut board, &mut session, FIRST_TIME_US);
    assert_eq!(rewound.generation, first.generation.wrapping_add(1));
    assert_eq!(rewound.kernel_delta_us, None, "a new world starts over");
    let (next, _) = exchange(&mut board, &mut session, FIRST_TIME_US + STEP_US);
    assert_eq!(next.kernel_delta_us, Some(STEP_US));

    // A new connection is a new vehicle, even when its clock happens to
    // run on.
    drop(session);
    let mut session = connect(&mut board, &listener);
    let (reconnected, _) = exchange(&mut board, &mut session, FIRST_TIME_US + 2 * STEP_US);
    assert_eq!(reconnected.generation, rewound.generation.wrapping_add(1));
    assert_eq!(reconnected.kernel_delta_us, None);
}

#[test]
fn a_new_world_never_inherits_the_old_command() {
    let listener = TcpListener::bind("127.0.0.1:0").expect("bind sidecar");
    let mut board = board_for(&listener, constant_map);
    let mut session = connect(&mut board, &listener);

    let (_, first) = exchange(&mut board, &mut session, FIRST_TIME_US);
    assert_eq!(
        first, [0.0; 4],
        "no kernel cycle has commanded anything yet"
    );
    for i in 1..10 {
        let (_, controls) = exchange(&mut board, &mut session, FIRST_TIME_US + i * STEP_US);
        assert_eq!(controls, [0.3; 4], "each later state is a kernel cycle");
    }
    // A repeated state starts no step: the last command goes again.
    let (repeat, controls) = exchange(&mut board, &mut session, FIRST_TIME_US + 9 * STEP_US);
    assert_eq!(repeat.kernel_delta_us, None);
    assert_eq!(controls, [0.3; 4]);

    let (reset, controls) = exchange(&mut board, &mut session, FIRST_TIME_US);
    assert_eq!(reset.kernel_delta_us, None);
    assert_eq!(controls, [0.0; 4], "the first reply of a new world is idle");
}
