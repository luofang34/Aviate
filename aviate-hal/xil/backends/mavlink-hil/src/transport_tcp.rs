//! MAVLink HIL TCP transport.
//!
//! For bridges where the SIMULATOR listens and the flight controller
//! dials in — the shape X-Plane's bridge uses. The UDP transport in this
//! crate is peer-symmetric and connectionless; a stream link is not:
//!
//! - The simulator may not be listening yet, so connecting retries with
//!   a backoff instead of failing the board's construction.
//! - A closed stream is a distinct, observable event (`read` returning
//!   zero), and it must reset the frame reassembly buffer: a half-frame
//!   from the previous connection would corrupt the first frame of the
//!   next one.
//! - A nonblocking write can send short or refuse outright, so an
//!   unsent command is COUNTED and reported, never silently dropped.
//!
//! Frame parsing and serialization are shared with the UDP transport;
//! only the socket differs.

use std::collections::VecDeque;
use std::io::{self, Read, Write};
use std::net::{SocketAddr, TcpStream};
use std::time::{Duration, Instant};

use crate::messages::{
    Heartbeat, HilActuatorControls, HilGps, HilMessage, HilSensor, HilStateQuaternion,
};
use crate::wire::{parse_frame, serialize_frame, MavFrame, ParseError, MAX_FRAME_SIZE};

/// Seconds between connection attempts while the simulator is absent.
const RECONNECT_INTERVAL: Duration = Duration::from_millis(500);

/// Sensor samples held for the flight controller to answer.
///
/// A bridge that paces its stream on actuator feedback needs ONE answer
/// per sample; a latest-wins cache would coalesce several samples into
/// one answer and let the bridge's own queue grow until it gives up. The
/// queue is small on purpose: it absorbs a burst between control
/// iterations without ever becoming a place stale samples accumulate.
const SENSOR_QUEUE_DEPTH: usize = 16;

/// TCP HIL transport configuration.
#[derive(Clone, Debug)]
pub struct HilTcpConfig {
    /// The simulator's listening address the flight controller dials.
    pub simulator_addr: SocketAddr,
    /// System ID for outgoing messages.
    pub sys_id: u8,
    /// Component ID for outgoing messages.
    pub comp_id: u8,
}

impl Default for HilTcpConfig {
    fn default() -> Self {
        Self {
            // The standard SITL bridge port.
            simulator_addr: SocketAddr::from(([127, 0, 0, 1], 4560)),
            sys_id: 1,
            comp_id: 1,
        }
    }
}

/// HIL transport over a TCP stream the flight controller dials.
pub struct HilTcpTransport {
    stream: Option<TcpStream>,
    config: HilTcpConfig,
    next_attempt: Instant,
    seq: u8,
    rx_buf: [u8; 2048],
    rx_len: usize,
    start_time: Instant,
    rx_count: u64,
    tx_count: u64,
    tx_failures: u64,
    crc_errors: u64,
    connects: u64,
    sensors: VecDeque<HilSensor>,
    dropped_sensors: u64,
    gps: VecDeque<HilGps>,
    state_quaternions: VecDeque<HilStateQuaternion>,
}

impl HilTcpTransport {
    /// Creates the transport and attempts a first connection. A refused
    /// connection is NOT an error: the simulator commonly starts after
    /// the flight controller, and `poll` retries.
    #[must_use]
    pub fn new(config: HilTcpConfig) -> Self {
        let mut transport = Self {
            stream: None,
            config,
            next_attempt: Instant::now(),
            seq: 0,
            rx_buf: [0u8; 2048],
            rx_len: 0,
            start_time: Instant::now(),
            rx_count: 0,
            tx_count: 0,
            tx_failures: 0,
            crc_errors: 0,
            connects: 0,
            sensors: VecDeque::with_capacity(SENSOR_QUEUE_DEPTH),
            dropped_sensors: 0,
            gps: VecDeque::with_capacity(SENSOR_QUEUE_DEPTH),
            state_quaternions: VecDeque::with_capacity(SENSOR_QUEUE_DEPTH),
        };
        transport.try_connect();
        transport
    }

    /// True while the stream is up.
    #[must_use]
    pub fn connected(&self) -> bool {
        self.stream.is_some()
    }

    /// Attempts one connection if the retry interval has elapsed.
    fn try_connect(&mut self) {
        if self.stream.is_some() || Instant::now() < self.next_attempt {
            return;
        }
        self.next_attempt = Instant::now() + RECONNECT_INTERVAL;
        let Ok(stream) = TcpStream::connect(self.config.simulator_addr) else {
            return;
        };
        if stream.set_nonblocking(true).is_err() || stream.set_nodelay(true).is_err() {
            return;
        }
        // A reconnect must not inherit the previous stream's partial
        // frame, nor replay its last sample as if it were fresh.
        self.rx_len = 0;
        self.sensors.clear();
        self.gps.clear();
        self.state_quaternions.clear();
        self.stream = Some(stream);
        self.connects = self.connects.wrapping_add(1);
        log::info!(
            "HIL TCP connected to {} (attempt {})",
            self.config.simulator_addr,
            self.connects
        );
    }

    /// Drops the stream so the next `poll` redials.
    fn disconnect(&mut self, reason: &str) {
        if self.stream.take().is_some() {
            log::warn!("HIL TCP link down: {reason}");
        }
        self.rx_len = 0;
    }

    /// Reads everything available and parses HIL messages, redialing
    /// when the link is down.
    pub fn poll(&mut self) {
        if self.stream.is_none() {
            self.try_connect();
            return;
        }
        loop {
            let mut buf = [0u8; MAX_FRAME_SIZE];
            let read = match self.stream.as_mut() {
                Some(stream) => stream.read(&mut buf),
                None => return,
            };
            match read {
                // A stream read of zero is end-of-stream, not idleness:
                // the simulator closed the link.
                Ok(0) => {
                    self.disconnect("the simulator closed the connection");
                    return;
                }
                Ok(len) => {
                    let mut chunk = [0u8; MAX_FRAME_SIZE];
                    chunk[..len].copy_from_slice(&buf[..len]);
                    self.process_data(&chunk[..len]);
                }
                Err(ref error) if error.kind() == io::ErrorKind::WouldBlock => return,
                Err(ref error) if error.kind() == io::ErrorKind::Interrupted => {}
                Err(error) => {
                    self.disconnect(&format!("read failed: {error}"));
                    return;
                }
            }
        }
    }

    /// Appends `data` to the reassembly buffer and parses whole frames
    /// out of it.
    fn process_data(&mut self, data: &[u8]) {
        let space = self.rx_buf.len() - self.rx_len;
        if data.len() > space {
            // A stream that outruns the buffer means reassembly is
            // wedged on garbage; drop it rather than stall forever.
            self.rx_len = 0;
        }
        let copy_len = data.len().min(self.rx_buf.len());
        self.rx_buf[self.rx_len..self.rx_len + copy_len].copy_from_slice(&data[..copy_len]);
        self.rx_len += copy_len;

        let mut offset = 0;
        while offset < self.rx_len {
            match parse_frame(&self.rx_buf[offset..self.rx_len]) {
                Ok((frame, consumed)) => {
                    self.handle_frame(frame);
                    offset += consumed;
                    self.rx_count = self.rx_count.wrapping_add(1);
                }
                Err(ParseError::Incomplete) => break,
                Err(ParseError::CrcMismatch) => {
                    self.crc_errors = self.crc_errors.wrapping_add(1);
                    offset += 1;
                }
                // A well-formed frame this subset does not decode is
                // skipped WHOLE; only an unsyncable byte resyncs.
                Err(ParseError::UnknownMessage { consumed, .. }) => offset += consumed,
                Err(_) => offset += 1,
            }
        }
        if offset > 0 {
            self.rx_buf.copy_within(offset..self.rx_len, 0);
            self.rx_len -= offset;
        }
    }

    fn handle_frame(&mut self, frame: MavFrame) {
        match frame.message {
            HilMessage::Sensor(sensor) => {
                // A full queue means the control loop is not keeping up.
                // Drop the OLDEST sample and count it: the newest state
                // is the one worth answering, and a silent drop would
                // read as a healthy link.
                if self.sensors.len() == SENSOR_QUEUE_DEPTH {
                    self.sensors.pop_front();
                    self.dropped_sensors = self.dropped_sensors.wrapping_add(1);
                }
                self.sensors.push_back(sensor);
            }
            HilMessage::Gps(gps) => push_bounded(&mut self.gps, gps),
            HilMessage::StateQuaternion(state) => {
                push_bounded(&mut self.state_quaternions, state);
            }
            // Heartbeats and actuator controls travel the other way.
            HilMessage::Heartbeat(_) | HilMessage::ActuatorControls(_) => {}
        }
    }

    /// Takes the OLDEST unanswered sensor sample. Each sample the
    /// bridge sent gets its own answer, so callers drain in a loop.
    pub fn take_sensor(&mut self) -> Option<HilSensor> {
        self.sensors.pop_front()
    }

    /// Sensor samples dropped because the control loop fell behind.
    #[must_use]
    pub fn dropped_sensors(&self) -> u64 {
        self.dropped_sensors
    }

    /// Take the oldest GNSS sample that has not joined a sensor group.
    pub fn take_gps(&mut self) -> Option<HilGps> {
        self.gps.pop_front()
    }

    /// Take the GNSS sample with the specified simulator timestamp.
    pub fn take_gps_at(&mut self, timestamp_us: u64) -> Option<HilGps> {
        let index = self
            .gps
            .iter()
            .position(|sample| sample.time_usec == timestamp_us)?;
        self.gps.remove(index)
    }

    /// Take the newest truth sample and discard older truth samples.
    pub fn take_state_quaternion(&mut self) -> Option<HilStateQuaternion> {
        let latest = self.state_quaternions.pop_back();
        self.state_quaternions.clear();
        latest
    }

    /// Take the truth sample with the specified simulator timestamp.
    pub fn take_state_quaternion_at(&mut self, timestamp_us: u64) -> Option<HilStateQuaternion> {
        let index = self
            .state_quaternions
            .iter()
            .position(|sample| sample.time_usec == timestamp_us)?;
        self.state_quaternions.remove(index)
    }

    /// Discard all input that belongs to the current simulator generation.
    pub fn clear_generation_state(&mut self) {
        self.poll();
        self.rx_len = 0;
        self.sensors.clear();
        self.gps.clear();
        self.state_quaternions.clear();
    }

    /// Sends actuator controls to the simulator.
    ///
    /// # Errors
    ///
    /// Returns the underlying write failure. A link that is down returns
    /// `NotConnected` rather than pretending the command was sent.
    pub fn send_actuator_controls(&mut self, controls: &HilActuatorControls) -> io::Result<()> {
        self.send_message(&HilMessage::ActuatorControls(*controls))
    }

    /// Sends a heartbeat to the simulator.
    ///
    /// # Errors
    ///
    /// Returns the underlying write failure.
    pub fn send_heartbeat(&mut self, heartbeat: &Heartbeat) -> io::Result<()> {
        self.send_message(&HilMessage::Heartbeat(*heartbeat))
    }

    fn send_message(&mut self, msg: &HilMessage) -> io::Result<()> {
        let mut buf = [0u8; MAX_FRAME_SIZE];
        let Some(len) = serialize_frame(
            msg,
            self.seq,
            self.config.sys_id,
            self.config.comp_id,
            &mut buf,
        ) else {
            self.tx_failures = self.tx_failures.wrapping_add(1);
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "the message did not serialize",
            ));
        };
        let Some(stream) = self.stream.as_mut() else {
            self.tx_failures = self.tx_failures.wrapping_add(1);
            return Err(io::Error::new(
                io::ErrorKind::NotConnected,
                "no HIL link to the simulator",
            ));
        };
        // A partial write would split a frame across the wire and desync
        // the simulator's parser, so a short write is a failure, not a
        // retry: the next sample supersedes this one anyway.
        match stream.write(&buf[..len]) {
            Ok(written) if written == len => {
                self.seq = self.seq.wrapping_add(1);
                self.tx_count = self.tx_count.wrapping_add(1);
                Ok(())
            }
            Ok(_) => {
                self.tx_failures = self.tx_failures.wrapping_add(1);
                self.disconnect("a short write split a frame");
                Err(io::Error::new(
                    io::ErrorKind::WriteZero,
                    "the frame was written only in part",
                ))
            }
            Err(error) => {
                self.tx_failures = self.tx_failures.wrapping_add(1);
                if error.kind() != io::ErrorKind::WouldBlock {
                    self.disconnect(&format!("write failed: {error}"));
                }
                Err(error)
            }
        }
    }

    /// Blocks until the link has a byte to read or `timeout` elapses,
    /// returning whether data is waiting.
    ///
    /// A lockstep bridge holds its next sample until the command
    /// answering the previous one arrives, and it budgets only a
    /// millisecond or two per simulator frame to drain the samples that
    /// frame produced. A control loop paced by a sleep therefore answers
    /// a fraction of them and the bridge's queue overflows — the loop
    /// must be paced by the ARRIVAL of a sample instead. Peeking leaves
    /// the byte for `poll` to read normally.
    pub fn wait_readable(&mut self, timeout: Duration) -> bool {
        if !self.sensors.is_empty() {
            return true;
        }
        let Some(stream) = self.stream.as_ref() else {
            return false;
        };
        if stream.set_read_timeout(Some(timeout)).is_err() || stream.set_nonblocking(false).is_err()
        {
            self.disconnect("the link could not be put in blocking mode");
            return false;
        }
        let mut probe = [0u8; 1];
        let result = stream.peek(&mut probe);
        // Every other path in this transport assumes nonblocking reads,
        // so a link that cannot be restored is dropped rather than left
        // able to stall the control loop.
        if stream.set_nonblocking(true).is_err() {
            self.disconnect("the link could not be restored to nonblocking mode");
            return false;
        }
        match result {
            Ok(1) => true,
            Ok(0) => {
                self.disconnect("the simulator closed the connection");
                false
            }
            Ok(_) => false,
            Err(error)
                if matches!(
                    error.kind(),
                    io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut
                ) =>
            {
                false
            }
            Err(error) => {
                self.disconnect(&format!("read probe failed: {error}"));
                false
            }
        }
    }

    /// Microseconds since the transport was created.
    #[must_use]
    pub fn now_us(&self) -> u64 {
        u64::try_from(self.start_time.elapsed().as_micros()).unwrap_or(u64::MAX)
    }

    /// Received frames, sent frames, CRC failures, unsent commands, and
    /// connection attempts that succeeded.
    #[must_use]
    pub fn stats(&self) -> (u64, u64, u64, u64, u64) {
        (
            self.rx_count,
            self.tx_count,
            self.crc_errors,
            self.tx_failures,
            self.connects,
        )
    }

    /// The simulator address this transport dials.
    #[must_use]
    pub fn simulator_addr(&self) -> SocketAddr {
        self.config.simulator_addr
    }
}

fn push_bounded<T>(queue: &mut VecDeque<T>, value: T) {
    if queue.len() == SENSOR_QUEUE_DEPTH {
        queue.pop_front();
    }
    queue.push_back(value);
}

#[cfg(test)]
mod tests;
