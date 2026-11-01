//! In-memory zero-allocation MAVLink v2 command and telemetry endpoint.

mod mavlink_rx;
mod mavlink_tx;
mod ring;

pub(crate) use mavlink_tx::DirectTelemetryFrameInput;
pub use ring::DirectByteRing;

use aviate_hal_io::{CommandOutcome, SystemCommand};
use aviate_link::mavlink::protocol::CommandAck;
use aviate_link::mavlink::{
    mav_cmd, mav_result, serialize_mavlink, LocalAddress, MavMessage, MavState,
};

use mavlink_rx::{next_inbound_action, InboundAction};

/// Default inbound MAVLink RX byte ring capacity.
pub const DEFAULT_RX_RING_BYTES: usize = 2048;

/// Default outbound MAVLink TX byte ring capacity.
pub const DEFAULT_TX_RING_BYTES: usize = 8192;

const COMMAND_QUEUE_CAP: usize = 16;
pub(crate) const MAX_FRAME_BUF: usize = 280;

/// Outbound MAVLink stream rates in Hertz (`0` disables a stream).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DirectTelemetryRates {
    /// `HEARTBEAT` and `AVIATE_GEOMAGNETIC_REFERENCE` rate in Hertz (default `1`).
    pub heartbeat_hz: u8,
    /// `SYS_STATUS` rate in Hertz (default `2`).
    pub sys_status_hz: u8,
    /// `ATTITUDE_QUATERNION` rate in Hertz (default `20`).
    pub attitude_hz: u8,
    /// `LOCAL_POSITION_NED`, `GLOBAL_POSITION_INT`, `SCALED_PRESSURE`, and `GPS_RAW_INT` rate in Hertz (default `10`).
    pub position_hz: u8,
    /// Standalone `ESTIMATOR_STATUS` and `AVIATE_ESTIMATOR_STATUS` rate in Hertz (default `10`).
    pub estimator_status_hz: u8,
}

impl Default for DirectTelemetryRates {
    fn default() -> Self {
        Self {
            heartbeat_hz: 1,
            sys_status_hz: 2,
            attitude_hz: 20,
            position_hz: 10,
            estimator_status_hz: 10,
        }
    }
}

/// Stack-allocated MAVLink v2 RX/TX byte endpoint and typed command queue.
#[derive(Debug, Clone)]
pub struct DirectMavlinkEndpoint<
    const RX_CAP: usize = DEFAULT_RX_RING_BYTES,
    const TX_CAP: usize = DEFAULT_TX_RING_BYTES,
> {
    rx_ring: DirectByteRing<RX_CAP>,
    tx_ring: DirectByteRing<TX_CAP>,
    cmd_queue: [Option<SystemCommand>; COMMAND_QUEUE_CAP],
    cmd_head: usize,
    cmd_len: usize,
    sys_id: u8,
    comp_id: u8,
    seq: u8,
    armed: bool,
    mav_state: MavState,
    rates: DirectTelemetryRates,
    last_heartbeat_us: Option<u64>,
    last_sys_status_us: Option<u64>,
    last_attitude_us: Option<u64>,
    last_position_us: Option<u64>,
    last_estimator_status_us: Option<u64>,
}

impl<const RX_CAP: usize, const TX_CAP: usize> DirectMavlinkEndpoint<RX_CAP, TX_CAP> {
    /// Construct a [`DirectMavlinkEndpoint`] for `(sys_id, comp_id)` with default telemetry rates.
    #[must_use]
    pub const fn new(sys_id: u8, comp_id: u8) -> Self {
        Self {
            rx_ring: DirectByteRing::new(),
            tx_ring: DirectByteRing::new(),
            cmd_queue: [const { None }; COMMAND_QUEUE_CAP],
            cmd_head: 0,
            cmd_len: 0,
            sys_id,
            comp_id,
            seq: 0,
            armed: false,
            mav_state: MavState::Boot,
            rates: DirectTelemetryRates {
                heartbeat_hz: 1,
                sys_status_hz: 2,
                attitude_hz: 20,
                position_hz: 10,
                estimator_status_hz: 10,
            },
            last_heartbeat_us: None,
            last_sys_status_us: None,
            last_attitude_us: None,
            last_position_us: None,
            last_estimator_status_us: None,
        }
    }

    /// Return the configured [`DirectTelemetryRates`].
    #[must_use]
    pub const fn rates(&self) -> DirectTelemetryRates {
        self.rates
    }

    /// Update the outbound [`DirectTelemetryRates`].
    pub fn set_rates(&mut self, rates: DirectTelemetryRates) {
        self.rates = rates;
    }

    /// Return the number of undecoded bytes in the RX ring.
    #[must_use]
    pub const fn rx_available(&self) -> usize {
        self.rx_ring.len()
    }

    /// Return the number of encoded MAVLink v2 bytes waiting in the TX ring.
    #[must_use]
    pub const fn tx_available(&self) -> usize {
        self.tx_ring.len()
    }

    /// Push raw inbound MAVLink v2 bytes into the RX ring and return bytes accepted.
    pub fn push_rx(&mut self, bytes: &[u8]) -> usize {
        self.rx_ring.push(bytes)
    }

    /// Pop encoded outbound MAVLink v2 bytes from the TX ring into `buf`.
    pub fn pop_tx(&mut self, buf: &mut [u8]) -> usize {
        self.tx_ring.pop(buf)
    }

    /// Enqueue a typed [`SystemCommand`] directly without MAVLink serialization.
    ///
    /// Returns `true` if queued, or `false` when the 16-slot command FIFO is full.
    pub fn enqueue_command(&mut self, cmd: SystemCommand) -> bool {
        if self.cmd_len >= COMMAND_QUEUE_CAP {
            return false;
        }
        let tail = (self.cmd_head.wrapping_add(self.cmd_len)) % COMMAND_QUEUE_CAP;
        if let Some(slot) = self.cmd_queue.get_mut(tail) {
            *slot = Some(cmd);
        }
        self.cmd_len = self.cmd_len.wrapping_add(1);
        true
    }

    /// Decode pending RX frames and pop the next [`SystemCommand`] in FIFO order.
    pub fn poll_command(&mut self) -> Option<SystemCommand> {
        self.decode_inbound_frames();
        if self.cmd_len == 0 {
            return None;
        }
        let cmd = self.cmd_queue.get_mut(self.cmd_head).and_then(Option::take);
        self.cmd_head = (self.cmd_head.wrapping_add(1)) % COMMAND_QUEUE_CAP;
        self.cmd_len = self.cmd_len.saturating_sub(1);
        if self.cmd_len == 0 {
            self.cmd_head = 0;
        }
        cmd
    }

    /// Update the armed flag advertised in `HEARTBEAT`.
    pub fn set_armed(&mut self, armed: bool) {
        self.armed = armed;
    }

    /// Update the `MAV_STATE` advertised in `HEARTBEAT`.
    pub fn set_system_status(&mut self, state: MavState) {
        self.mav_state = state;
    }

    /// Clear transient queues and rate timers on a simulator world reset.
    pub fn clear_for_reset(&mut self) {
        self.rx_ring.clear();
        self.cmd_queue = [const { None }; COMMAND_QUEUE_CAP];
        self.cmd_head = 0;
        self.cmd_len = 0;
        self.armed = false;
        self.mav_state = MavState::Boot;
        self.last_heartbeat_us = None;
        self.last_sys_status_us = None;
        self.last_attitude_us = None;
        self.last_position_us = None;
        self.last_estimator_status_us = None;
    }

    /// Encode a `COMMAND_ACK` for `command` and `outcome` into the TX ring.
    pub fn report_outcome(&mut self, command: &SystemCommand, outcome: CommandOutcome) {
        let mav_command = match command {
            SystemCommand::Arm | SystemCommand::Disarm | SystemCommand::EmergencyTerminate => {
                mav_cmd::COMPONENT_ARM_DISARM
            }
            SystemCommand::FlightControl(_) => return,
        };
        let (result, detail) = match outcome {
            CommandOutcome::Accepted => (mav_result::ACCEPTED, 0),
            CommandOutcome::ArmRejected { missing, .. } if outcome.is_retryable() => (
                mav_result::TEMPORARILY_REJECTED,
                i32::try_from(missing.bits()).unwrap_or(i32::MAX),
            ),
            CommandOutcome::ArmRejected { missing, .. } => (
                mav_result::DENIED,
                i32::try_from(missing.bits()).unwrap_or(i32::MAX),
            ),
            CommandOutcome::DisarmRejected(_) => (mav_result::DENIED, 0),
        };
        self.emit_command_ack(mav_command, result, detail);
    }

    fn decode_inbound_frames(&mut self) {
        let local = LocalAddress {
            system_id: self.sys_id,
            component_id: self.comp_id,
        };
        while self.cmd_len < COMMAND_QUEUE_CAP {
            let Some(action) = next_inbound_action(&mut self.rx_ring, local) else {
                break;
            };
            match action {
                InboundAction::Enqueue(cmd) => {
                    let _ = self.enqueue_command(cmd);
                }
                InboundAction::ImmediateAck { command, result } => {
                    self.emit_command_ack(command, result, 0);
                }
                InboundAction::Ignore => {}
            }
        }
    }

    fn emit_command_ack(&mut self, command: u16, result: u8, detail: i32) {
        let ack = CommandAck {
            command,
            result,
            progress: 0,
            result_param2: detail,
            target_system: 255,
            target_component: 0,
        };
        self.push_mav_message(&MavMessage::CommandAck(ack));
    }

    fn push_mav_message(&mut self, msg: &MavMessage) {
        let mut buf = [0_u8; MAX_FRAME_BUF];
        if let Some(len) = serialize_mavlink(msg, self.seq, self.sys_id, self.comp_id, &mut buf) {
            if let Some(frame) = buf.get(..len) {
                if self.tx_ring.push_all(frame) {
                    self.seq = self.seq.wrapping_add(1);
                }
            }
        }
    }
}
