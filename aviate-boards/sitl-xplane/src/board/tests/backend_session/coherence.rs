//! Verify coherent frames when the TCP link buffers more than one group.

use super::*;

pub(super) fn verify_buffered_groups_are_coherent() {
    verify_reset_boundary_is_coherent();
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
    let ready_time = u64::try_from(ready.simulation_time.as_micros()).expect("time fits u64");
    let first_time = ready_time.wrapping_add(10_000);
    let second_time = first_time.wrapping_add(10_000);
    let second_attitude = [0.5, 0.5, 0.5, 0.5];
    let states = [
        buffered_truth(
            first_time,
            INITIAL_LATITUDE + 100,
            [100, 0, 0],
            [1.0, 0.0, 0.0, 0.0],
        ),
        buffered_truth(
            second_time,
            INITIAL_LATITUDE + 200,
            [200, -100, 50],
            second_attitude,
        ),
    ];
    bridge
        .send(BridgeCommand::Burst(states))
        .expect("send buffered groups");
    let FrameEvent::Frame(frame) = backend
        .next_frame(Duration::from_secs(1))
        .expect("buffered frame")
    else {
        panic!("expected one buffered frame");
    };
    assert_eq!(frame.step, ready.step.wrapping_add(2));
    assert_eq!(frame.simulation_time, Duration::from_micros(second_time));
    assert_eq!(
        backend.board().last_fix().expect("latest GNSS fix").lat_deg,
        f64::from(INITIAL_LATITUDE + 200) / 1e7
    );
    assert!(
        frame.vehicle.position[0] > 2.0,
        "buffered frame: {:?}",
        frame.vehicle
    );
    assert_eq!(frame.vehicle.velocity, [2.0, -1.0, 0.5]);
    assert_eq!(frame.vehicle.orientation, second_attitude);
    assert_eq!(frame.vehicle.angular_velocity, [2.0, -1.0, 0.5]);

    let third_time = second_time.wrapping_add(10_000);
    bridge
        .send(BridgeCommand::Sample(third_time))
        .expect("send following group");
    let FrameEvent::Frame(following) = backend
        .next_frame(Duration::from_secs(1))
        .expect("following frame")
    else {
        panic!("expected one following frame");
    };
    assert_eq!(following.simulation_time, Duration::from_micros(third_time));
    assert_eq!(following.vehicle.velocity, [0.0; 3]);
    assert_eq!(following.vehicle.orientation, [1.0, 0.0, 0.0, 0.0]);

    bridge.send(BridgeCommand::Stop).expect("stop bridge");
    bridge_thread.join().expect("bridge thread finishes");
}

fn verify_reset_boundary_is_coherent() {
    let (address, bridge, bridge_thread) = start_bridge();
    let mut backend = make_backend(address);
    let connected = backend
        .connect(0, Duration::from_secs(1))
        .expect("backend connects");
    let mut directive_id = 0_u64;
    let reset_truth = buffered_truth(0, INITIAL_LATITUDE, [900, -800, 700], [0.0, 1.0, 0.0, 0.0]);
    bridge
        .send(BridgeCommand::PrimeReset { state: reset_truth })
        .expect("prime distinct reset sample");
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
    let normal_attitude = [0.5, 0.5, 0.5, 0.5];
    let normal_truth = buffered_truth(
        10_000,
        INITIAL_LATITUDE + 200,
        [200, -100, 50],
        normal_attitude,
    );
    bridge
        .send(BridgeCommand::State(normal_truth))
        .expect("send distinct normal sample");
    let FrameEvent::Frame(frame) = backend
        .next_frame(Duration::from_secs(1))
        .expect("first normal frame")
    else {
        panic!("expected first normal frame");
    };
    assert_eq!(frame.simulation_time, Duration::from_micros(10_000));
    assert_eq!(
        backend.board().last_fix().expect("normal GNSS fix").lat_deg,
        f64::from(INITIAL_LATITUDE + 200) / 1e7
    );
    assert!(frame.vehicle.position[0] > 2.0, "frame: {frame:?}");
    assert_eq!(frame.vehicle.velocity, [2.0, -1.0, 0.5]);
    assert_eq!(frame.vehicle.orientation, normal_attitude);
    assert_eq!(frame.vehicle.angular_velocity, [2.0, -1.0, 0.5]);
    bridge.send(BridgeCommand::Stop).expect("stop bridge");
    bridge_thread.join().expect("bridge thread finishes");
}

fn buffered_truth(
    timestamp: u64,
    latitude: i32,
    velocity: [i16; 3],
    attitude: [f32; 4],
) -> HilStateQuaternion {
    HilStateQuaternion {
        time_usec: timestamp,
        attitude_quaternion: attitude,
        rollspeed: f32::from(velocity[0]) / 100.0,
        pitchspeed: f32::from(velocity[1]) / 100.0,
        yawspeed: f32::from(velocity[2]) / 100.0,
        lat: latitude,
        lon: -1_050_000_000,
        alt: 100_000,
        vx: velocity[0],
        vy: velocity[1],
        vz: velocity[2],
        zacc: -1_000,
        ..HilStateQuaternion::default()
    }
}

pub(super) fn send_burst(
    stream: &mut TcpStream,
    states: [HilStateQuaternion; 2],
    sequence: &mut u8,
    actuator_buffer: &mut Vec<u8>,
) {
    for state in states {
        send_group(stream, state, sequence);
    }
    for _ in states {
        read_non_reset_answer(stream, actuator_buffer);
    }
}

fn send_group(stream: &mut TcpStream, state: HilStateQuaternion, sequence: &mut u8) {
    let messages = [
        HilMessage::Gps(gps(state.time_usec, state.lat)),
        HilMessage::StateQuaternion(state),
        HilMessage::Sensor(sensor(state.time_usec, false)),
    ];
    let mut group = Vec::new();
    for message in messages {
        let mut frame = [0_u8; 300];
        let length =
            serialize_frame(&message, *sequence, 1, 1, &mut frame).expect("message fits frame");
        *sequence = sequence.wrapping_add(1);
        group.extend_from_slice(&frame[..length]);
    }
    stream.write_all(&group).expect("send buffered group");
}
