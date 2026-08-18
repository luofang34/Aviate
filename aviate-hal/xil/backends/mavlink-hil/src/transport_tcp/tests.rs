//! TCP transport behavior against a real loopback listener: connect,
//! frame reassembly across segment boundaries, close detection, and the
//! refusal to report an unsent command as sent.
//!
//! The workspace forbids `expect`/`unwrap`/`panic`, so a setup step that
//! cannot proceed asserts and returns rather than unwinding.

use std::io::{Read, Write};
use std::net::{SocketAddr, TcpListener, TcpStream};
use std::time::{Duration, Instant};

use super::{HilTcpConfig, HilTcpTransport};
use crate::messages::{HilActuatorControls, HilMessage, HilSensor};
use crate::wire::{parse_frame, serialize_frame};

/// Binds an ephemeral loopback listener, never a fixed port.
fn listener() -> Option<(TcpListener, SocketAddr)> {
    let listener = TcpListener::bind("127.0.0.1:0"); // COV:EXCL(TEST)
    assert!(listener.is_ok());
    let Ok(listener) = listener else {
        return None;
    };
    let addr = listener.local_addr(); // COV:EXCL(TEST)
    assert!(addr.is_ok());
    let Ok(addr) = addr else {
        return None;
    };
    Some((listener, addr))
}

fn accept(listener: &TcpListener) -> Option<TcpStream> {
    let accepted = listener.accept(); // COV:EXCL(TEST)
    assert!(accepted.is_ok());
    let Ok((stream, _)) = accepted else {
        return None;
    };
    Some(stream)
}

fn transport_for(addr: SocketAddr) -> HilTcpTransport {
    HilTcpTransport::new(HilTcpConfig {
        simulator_addr: addr,
        sys_id: 1,
        comp_id: 1,
    })
}

fn sensor_frame(time_usec: u64) -> Option<Vec<u8>> {
    let sensor = HilSensor {
        time_usec,
        xacc: 0.0,
        yacc: 0.0,
        zacc: -9.81,
        xgyro: 0.01,
        ygyro: 0.02,
        zgyro: 0.03,
        xmag: 0.2,
        ymag: 0.0,
        zmag: 0.4,
        abs_pressure: 1013.25,
        diff_pressure: 0.0,
        pressure_alt: 0.0,
        temperature: 25.0,
        fields_updated: 0xFFFF_FFFF,
        id: 0,
    };
    let mut buf = [0u8; 512];
    let len = serialize_frame(&HilMessage::Sensor(sensor), 0, 1, 1, &mut buf); // COV:EXCL(TEST)
    assert!(len.is_some());
    len.map(|len| buf[..len].to_vec())
}

/// Polls until `ready` holds or the deadline passes, so a test
/// synchronizes on observable state instead of a fixed sleep.
fn poll_until(
    transport: &mut HilTcpTransport,
    mut ready: impl FnMut(&HilTcpTransport) -> bool,
) -> bool {
    let deadline = Instant::now() + Duration::from_secs(3);
    while Instant::now() < deadline {
        transport.poll();
        if ready(transport) {
            return true;
        }
        std::thread::sleep(Duration::from_millis(5));
    }
    false
}

#[test]
fn a_refused_connection_is_retried_not_fatal() {
    // Nothing listens on this port: construction must still succeed,
    // because a simulator commonly starts after the flight controller.
    let Some((listener, addr)) = listener() else {
        return;
    };
    drop(listener);
    let transport = transport_for(addr);
    assert!(!transport.connected());
}

#[test]
fn frames_split_across_segments_reassemble() {
    let Some((listener, addr)) = listener() else {
        return;
    };
    let mut transport = transport_for(addr);
    let Some(mut peer) = accept(&listener) else {
        return;
    };
    let Some(frame) = sensor_frame(1_000_000) else {
        return;
    };

    // Deliver one frame in two writes, splitting it mid-payload: a
    // stream link has no message boundaries.
    let split = frame.len() / 2;
    assert!(peer.write_all(&frame[..split]).is_ok()); // COV:EXCL(TEST)
    poll_until(&mut transport, |t| !t.sensors.is_empty());
    assert!(transport.sensors.is_empty(), "a half frame must not decode");

    assert!(peer.write_all(&frame[split..]).is_ok()); // COV:EXCL(TEST)
    assert!(
        poll_until(&mut transport, |t| !t.sensors.is_empty()),
        "the completed frame must decode"
    );
    let Some(sensor) = transport.take_sensor() else {
        return;
    };
    assert_eq!(sensor.time_usec, 1_000_000);
    assert!((sensor.zacc - (-9.81)).abs() < 1e-6);
}

#[test]
fn a_closed_stream_is_observed_and_redialed() {
    let Some((listener, addr)) = listener() else {
        return;
    };
    let mut transport = transport_for(addr);
    let Some(peer) = accept(&listener) else {
        return;
    };
    assert!(transport.connected());

    drop(peer);
    assert!(
        poll_until(&mut transport, |t| !t.connected()),
        "a closed stream must be observed, not read forever"
    );
    assert!(
        poll_until(&mut transport, HilTcpTransport::connected),
        "the transport must redial while the listener is up"
    );
    drop(accept(&listener));
}

#[test]
fn a_command_with_no_link_is_reported_unsent() {
    let Some((listener, addr)) = listener() else {
        return;
    };
    drop(listener);
    let mut transport = transport_for(addr);
    let sent = transport.send_actuator_controls(&HilActuatorControls::default());
    assert!(sent.is_err(), "a command with no link cannot succeed");
    if let Err(error) = sent {
        assert_eq!(error.kind(), std::io::ErrorKind::NotConnected);
    }
    let (_, tx, _, failures, _) = transport.stats();
    assert_eq!(tx, 0, "nothing was sent");
    assert_eq!(failures, 1, "the unsent command is counted, not swallowed");
}

#[test]
fn a_reconnect_drops_the_previous_partial_frame() {
    let Some((listener, addr)) = listener() else {
        return;
    };
    let mut transport = transport_for(addr);
    let Some(mut peer) = accept(&listener) else {
        return;
    };
    let Some(partial) = sensor_frame(2_000_000) else {
        return;
    };

    // Leave half a frame in the reassembly buffer, then break the link.
    assert!(peer.write_all(&partial[..partial.len() / 2]).is_ok()); // COV:EXCL(TEST)
    poll_until(&mut transport, |t| t.rx_len > 0);
    drop(peer);
    assert!(poll_until(&mut transport, |t| !t.connected()));
    assert_eq!(transport.rx_len, 0, "the partial frame is dropped");

    // A whole frame on the new connection decodes cleanly, which it
    // could not if the stale half were still buffered.
    assert!(poll_until(&mut transport, HilTcpTransport::connected));
    let Some(mut next) = accept(&listener) else {
        return;
    };
    let Some(whole) = sensor_frame(3_000_000) else {
        return;
    };
    assert!(next.write_all(&whole).is_ok()); // COV:EXCL(TEST)
    assert!(poll_until(&mut transport, |t| !t.sensors.is_empty()));
    if let Some(sensor) = transport.take_sensor() {
        assert_eq!(sensor.time_usec, 3_000_000);
    }
}

#[test]
fn a_sent_command_reaches_the_simulator() {
    let Some((listener, addr)) = listener() else {
        return;
    };
    let mut transport = transport_for(addr);
    let Some(mut peer) = accept(&listener) else {
        return;
    };
    assert!(peer.set_read_timeout(Some(Duration::from_secs(3))).is_ok()); // COV:EXCL(TEST)

    let mut command = HilActuatorControls {
        mode: HilActuatorControls::MODE_FLAG_ARMED,
        ..HilActuatorControls::default()
    };
    command.controls[0] = 0.5;
    assert!(transport.send_actuator_controls(&command).is_ok());

    let mut buf = [0u8; 512];
    let read = peer.read(&mut buf); // COV:EXCL(TEST)
    assert!(read.is_ok());
    let Ok(read) = read else {
        return;
    };
    let parsed = parse_frame(&buf[..read]);
    assert!(parsed.is_ok());
    let Ok((frame, _)) = parsed else {
        return;
    };
    // The transport sent actuator controls; anything else is a defect.
    assert!(matches!(frame.message, HilMessage::ActuatorControls(_)));
    let HilMessage::ActuatorControls(received) = frame.message else {
        return;
    };
    assert!((received.controls[0] - 0.5).abs() < 1e-6);
    assert!(received.is_armed());
    // The transport carries what it is given; the lockstep flag is the
    // backend's to set, and its own test pins that.
}

#[test]
fn an_unwritable_link_reports_the_refusal() {
    let Some((listener, addr)) = listener() else {
        return;
    };
    let mut transport = transport_for(addr);
    // Hold the peer without ever reading, so the socket buffer fills.
    let Some(peer) = accept(&listener) else {
        return;
    };
    let mut refused = false;
    for _ in 0..100_000 {
        if transport
            .send_actuator_controls(&HilActuatorControls::default())
            .is_err()
        {
            refused = true;
            break;
        }
    }
    assert!(refused, "a full socket must surface as an error");
    let (_, _, _, failures, _) = transport.stats();
    assert!(failures > 0, "refusals are counted");
    drop(peer);
}

#[test]
fn the_transport_reports_the_address_it_dials() {
    let Some((listener, addr)) = listener() else {
        return;
    };
    let transport = transport_for(addr);
    assert_eq!(transport.simulator_addr(), addr);
    drop(accept(&listener));
}
