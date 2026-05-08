//! MAVLink HIL UDP Transport
//!
//! Provides UDP-based communication with legacy HIL simulators.
//! The transport layer handles:
//! - Receiving HIL_SENSOR and HIL_GPS from the simulator
//! - Sending HIL_ACTUATOR_CONTROLS to the simulator

use std::io;
use std::net::{SocketAddr, UdpSocket};
use std::time::Instant;

use crate::messages::{
    Heartbeat, HilActuatorControls, HilGps, HilMessage, HilSensor, HilStateQuaternion,
};
use crate::wire::{parse_frame, serialize_frame, MavFrame, ParseError, MAX_FRAME_SIZE};

/// HIL transport configuration
#[derive(Clone, Debug)]
pub struct HilTransportConfig {
    /// Local port to bind for receiving HIL data
    pub local_port: u16,
    /// Remote simulator address (for sending actuator controls)
    pub simulator_addr: SocketAddr,
    /// System ID for outgoing messages
    pub sys_id: u8,
    /// Component ID for outgoing messages
    pub comp_id: u8,
}

impl Default for HilTransportConfig {
    fn default() -> Self {
        Self {
            local_port: 14560, // Standard PX4 SITL HIL port
            simulator_addr: SocketAddr::from(([127, 0, 0, 1], 14560)),
            sys_id: 1,
            comp_id: 1,
        }
    }
}

/// HIL UDP transport
pub struct HilTransport {
    socket: UdpSocket,
    config: HilTransportConfig,
    seq: u8,
    rx_buf: [u8; 2048],
    rx_len: usize,
    start_time: Instant,
    // Statistics
    rx_count: u64,
    tx_count: u64,
    crc_errors: u64,
    // Cached last received data
    last_sensor: Option<HilSensor>,
    last_gps: Option<HilGps>,
    last_state_quaternion: Option<HilStateQuaternion>,
}

impl HilTransport {
    /// Create a new HIL transport
    pub fn new(config: HilTransportConfig) -> io::Result<Self> {
        let socket = UdpSocket::bind(("0.0.0.0", config.local_port))?;
        socket.set_nonblocking(true)?;

        Ok(Self {
            socket,
            config,
            seq: 0,
            rx_buf: [0u8; 2048],
            rx_len: 0,
            start_time: Instant::now(),
            rx_count: 0,
            tx_count: 0,
            crc_errors: 0,
            last_sensor: None,
            last_gps: None,
            last_state_quaternion: None,
        })
    }

    /// Poll for incoming messages
    ///
    /// Receives all available data and parses HIL messages.
    /// Updates internal sensor/gps caches.
    pub fn poll(&mut self) {
        // Receive available data
        loop {
            let mut buf = [0u8; MAX_FRAME_SIZE];
            match self.socket.recv_from(&mut buf) {
                Ok((len, _src)) => {
                    self.process_data(&buf[..len]);
                }
                Err(ref e) if e.kind() == io::ErrorKind::WouldBlock => break,
                Err(_) => break,
            }
        }
    }

    /// Process received data
    fn process_data(&mut self, data: &[u8]) {
        // Append to buffer
        let space = self.rx_buf.len() - self.rx_len;
        let copy_len = data.len().min(space);
        self.rx_buf[self.rx_len..self.rx_len + copy_len].copy_from_slice(&data[..copy_len]);
        self.rx_len += copy_len;

        // Parse frames from buffer
        let mut offset = 0;
        while offset < self.rx_len {
            match parse_frame(&self.rx_buf[offset..self.rx_len]) {
                Ok((frame, consumed)) => {
                    self.handle_frame(frame);
                    offset += consumed;
                    self.rx_count += 1;
                }
                Err(ParseError::Incomplete) => break,
                Err(ParseError::CrcMismatch) => {
                    self.crc_errors += 1;
                    offset += 1; // Skip bad byte
                }
                Err(_) => {
                    offset += 1; // Skip unknown/invalid
                }
            }
        }

        // Compact buffer
        if offset > 0 {
            self.rx_buf.copy_within(offset..self.rx_len, 0);
            self.rx_len -= offset;
        }
    }

    /// Handle a parsed frame
    fn handle_frame(&mut self, frame: MavFrame) {
        match frame.message {
            HilMessage::Heartbeat(_) => {
                // Ignore incoming heartbeats (we receive these from GCS)
            }
            HilMessage::Sensor(sensor) => {
                self.last_sensor = Some(sensor);
            }
            HilMessage::Gps(gps) => {
                self.last_gps = Some(gps);
            }
            HilMessage::StateQuaternion(state) => {
                self.last_state_quaternion = Some(state);
            }
            HilMessage::ActuatorControls(_) => {
                // Ignore incoming actuator controls (we send these)
            }
        }
    }

    /// Take the last received sensor data
    pub fn take_sensor(&mut self) -> Option<HilSensor> {
        self.last_sensor.take()
    }

    /// Take the last received GPS data
    pub fn take_gps(&mut self) -> Option<HilGps> {
        self.last_gps.take()
    }

    /// Take the last received state quaternion data
    pub fn take_state_quaternion(&mut self) -> Option<HilStateQuaternion> {
        self.last_state_quaternion.take()
    }

    /// Send actuator controls to the simulator
    pub fn send_actuator_controls(&mut self, controls: &HilActuatorControls) -> io::Result<()> {
        let msg = HilMessage::ActuatorControls(*controls);
        self.send_message(&msg)
    }

    /// Send a heartbeat message to the simulator
    pub fn send_heartbeat(&mut self, heartbeat: &Heartbeat) -> io::Result<()> {
        let msg = HilMessage::Heartbeat(*heartbeat);
        self.send_message(&msg)
    }

    /// Send a generic HIL message
    fn send_message(&mut self, msg: &HilMessage) -> io::Result<()> {
        let mut buf = [0u8; MAX_FRAME_SIZE];

        if let Some(len) = serialize_frame(
            msg,
            self.seq,
            self.config.sys_id,
            self.config.comp_id,
            &mut buf,
        ) {
            self.seq = self.seq.wrapping_add(1);
            self.socket
                .send_to(&buf[..len], self.config.simulator_addr)?;
            self.tx_count += 1;
        }

        Ok(())
    }

    /// Get current timestamp in microseconds
    pub fn now_us(&self) -> u64 {
        self.start_time.elapsed().as_micros() as u64
    }

    /// Get statistics (rx_count, tx_count, crc_errors)
    pub fn stats(&self) -> (u64, u64, u64) {
        (self.rx_count, self.tx_count, self.crc_errors)
    }

    /// Get the local port
    pub fn local_port(&self) -> u16 {
        self.config.local_port
    }

    /// Get the simulator address
    pub fn simulator_addr(&self) -> SocketAddr {
        self.config.simulator_addr
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::net::UdpSocket;
    use std::thread;
    use std::time::Duration;

    fn find_available_port() -> Option<u16> {
        // Bind to port 0 to get an available port
        let socket = UdpSocket::bind("127.0.0.1:0");
        assert!(socket.is_ok());
        let Ok(socket) = socket else {
            return None;
        };

        let addr = socket.local_addr();
        assert!(addr.is_ok());
        let Ok(addr) = addr else {
            return None;
        };
        Some(addr.port())
    }

    #[test]
    fn test_transport_create() {
        let Some(port) = find_available_port() else {
            return;
        };
        let config = HilTransportConfig {
            local_port: port,
            simulator_addr: SocketAddr::from(([127, 0, 0, 1], port + 1)),
            sys_id: 1,
            comp_id: 1,
        };
        let transport = HilTransport::new(config);
        assert!(transport.is_ok());
    }

    #[test]
    fn test_transport_send_receive() {
        let Some(port1) = find_available_port() else {
            return;
        };
        let Some(port2) = find_available_port() else {
            return;
        };

        // Create transport (FC side)
        let config = HilTransportConfig {
            local_port: port1,
            simulator_addr: SocketAddr::from(([127, 0, 0, 1], port2)),
            sys_id: 1,
            comp_id: 1,
        };
        let transport = HilTransport::new(config);
        assert!(transport.is_ok());
        let Ok(mut transport) = transport else {
            return;
        };

        // Create simulator socket
        let sim_socket = UdpSocket::bind(("127.0.0.1", port2));
        assert!(sim_socket.is_ok());
        let Ok(sim_socket) = sim_socket else {
            return;
        };
        assert!(sim_socket.set_nonblocking(true).is_ok());

        // Send actuator controls
        let mut controls = HilActuatorControls {
            time_usec: 1234567890,
            ..Default::default()
        };
        controls.controls[0] = 0.5;
        controls.mode = HilActuatorControls::MODE_FLAG_ARMED;

        assert!(transport.send_actuator_controls(&controls).is_ok());

        // Give network time
        thread::sleep(Duration::from_millis(10));

        // Receive on simulator side
        let mut buf = [0u8; 256];
        let received = sim_socket.recv_from(&mut buf);
        assert!(received.is_ok());
        let Ok((len, _)) = received else {
            return;
        };
        assert!(len > 0);

        // Parse the received frame
        let frame = parse_frame(&buf[..len]);
        assert!(frame.is_ok());
        let Ok((frame, _)) = frame else {
            return;
        };
        assert!(matches!(&frame.message, HilMessage::ActuatorControls(_)));
        let HilMessage::ActuatorControls(parsed) = frame.message else {
            return;
        };
        assert!(parsed.is_armed());
        assert!((controls.controls[0] - parsed.controls[0]).abs() < 1e-6);
    }

    #[test]
    fn test_transport_receive_sensor() {
        let Some(port1) = find_available_port() else {
            return;
        };
        let Some(port2) = find_available_port() else {
            return;
        };

        // Create transport (FC side)
        let config = HilTransportConfig {
            local_port: port1,
            simulator_addr: SocketAddr::from(([127, 0, 0, 1], port2)),
            sys_id: 1,
            comp_id: 1,
        };
        let transport = HilTransport::new(config);
        assert!(transport.is_ok());
        let Ok(mut transport) = transport else {
            return;
        };

        // Create simulator socket
        let sim_socket = UdpSocket::bind(("127.0.0.1", port2));
        assert!(sim_socket.is_ok());
        let Ok(sim_socket) = sim_socket else {
            return;
        };

        // Build and send HIL_SENSOR from simulator
        let sensor = HilSensor {
            time_usec: 1234567890,
            xacc: 0.0,
            yacc: 0.0,
            zacc: -9.81,
            xgyro: 0.0,
            ygyro: 0.0,
            zgyro: 0.0,
            xmag: 0.2,
            ymag: 0.0,
            zmag: 0.4,
            abs_pressure: 1013.25,
            diff_pressure: 0.0,
            pressure_alt: 0.0,
            temperature: 25.0,
            fields_updated: 0xFFFFFFFF,
            id: 0,
        };

        let msg = HilMessage::Sensor(sensor);
        let mut buf = [0u8; 256];
        let len = serialize_frame(&msg, 1, 1, 1, &mut buf);
        assert!(len.is_some());
        let Some(len) = len else {
            return;
        };

        assert!(sim_socket
            .send_to(&buf[..len], ("127.0.0.1", port1))
            .is_ok());

        // Give network time
        thread::sleep(Duration::from_millis(10));

        // Poll and check
        transport.poll();
        let received = transport.take_sensor();
        assert!(received.is_some());

        let Some(received) = received else {
            return;
        };
        assert_eq!(received.time_usec, sensor.time_usec);
        assert!((received.zacc - sensor.zacc).abs() < 1e-6);
    }
}
