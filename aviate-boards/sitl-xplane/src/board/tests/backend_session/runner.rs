//! Run the mission runner on the TCP sample-paced backend.

use super::*;
use aviate_hal_xil::{Action, Criterion, Mission, MissionRunner, Phase, VehicleConfig};

pub(super) fn verify_mission_runner_session() {
    let (address, bridge_thread) = start_streaming_bridge();
    {
        let backend = make_backend(address);
        let mut runner = MissionRunner::new(backend, "alia").expect("runner is valid");
        let mission = scripted_mission();
        let mut initial_position = None;

        for _ in 0..3 {
            let result = runner.run(&mission);
            assert!(result.passed);
            assert_eq!(result.phases.len(), 3);
            let position = result.phases[0].trace[0].position;
            assert_eq!(*initial_position.get_or_insert(position), position);
        }
    }
    bridge_thread.join().expect("streaming bridge finishes");
}

fn scripted_mission() -> Mission {
    Mission {
        name: "xplane-backend-contract".to_owned(),
        description: "Run the acknowledged backend directives.".to_owned(),
        vehicle: VehicleConfig::default(),
        lockstep: true,
        phases: vec![
            phase("arm", Action::Arm, true, 10),
            phase("setpoint", Action::Thrust(0.0), true, 20),
            phase("disarm", Action::Disarm, false, 10),
        ],
        reset_between_runs: true,
    }
}

fn phase(name: &str, action: Action, armed: bool, duration_ms: u64) -> Phase {
    Phase {
        name: name.to_owned(),
        duration: Duration::from_millis(duration_ms),
        action,
        verify: vec![Criterion::Armed(armed)],
    }
}

fn start_streaming_bridge() -> (SocketAddr, JoinHandle<()>) {
    let listener = TcpListener::bind("127.0.0.1:0").expect("bind streaming bridge");
    let address = listener.local_addr().expect("streaming bridge address");
    let handle = std::thread::spawn(move || run_streaming_bridge(listener));
    (address, handle)
}

fn run_streaming_bridge(listener: TcpListener) {
    let (mut stream, _) = listener.accept().expect("accept flight controller");
    stream
        .set_read_timeout(Some(Duration::from_secs(10)))
        .expect("set streaming bridge timeout");
    let mut sequence = 0_u8;
    let mut timestamp = 0_u64;
    let mut buffer = Vec::new();
    let Some(mut answer) = read_answer_until_close(&mut stream, &mut buffer) else {
        return;
    };

    loop {
        if answer.flags & RESET_REQUEST_ACTUATOR_FLAG != 0 {
            timestamp = 0;
            send_sample_group(
                &mut stream,
                timestamp,
                true,
                INITIAL_LATITUDE,
                &mut sequence,
            );
            let Some(non_reset) = read_non_reset_until_close(&mut stream, &mut buffer) else {
                return;
            };
            answer = non_reset;
            continue;
        }
        timestamp = timestamp.saturating_add(10_000);
        send_sample_group(
            &mut stream,
            timestamp,
            false,
            INITIAL_LATITUDE,
            &mut sequence,
        );
        let Some(next) = read_answer_until_close(&mut stream, &mut buffer) else {
            return;
        };
        answer = next;
    }
}

fn read_non_reset_until_close(
    stream: &mut TcpStream,
    buffer: &mut Vec<u8>,
) -> Option<HilActuatorControls> {
    loop {
        let answer = read_answer_until_close(stream, buffer)?;
        if answer.flags & RESET_REQUEST_ACTUATOR_FLAG == 0 {
            return Some(answer);
        }
    }
}

fn read_answer_until_close(
    stream: &mut TcpStream,
    buffer: &mut Vec<u8>,
) -> Option<HilActuatorControls> {
    loop {
        if let Ok((frame, consumed)) = parse_frame(buffer) {
            buffer.drain(..consumed);
            if let HilMessage::ActuatorControls(controls) = frame.message {
                return Some(controls);
            }
            continue;
        }
        let mut chunk = [0_u8; 300];
        let length = stream.read(&mut chunk).ok()?;
        if length == 0 {
            return None;
        }
        buffer.extend_from_slice(&chunk[..length]);
    }
}
