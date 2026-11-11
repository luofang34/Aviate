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

use mavlink_rx::{next_inbound_action, AirspeedRequest, InboundAction, Sender};

/// Default inbound MAVLink RX byte ring capacity.
pub const DEFAULT_RX_RING_BYTES: usize = 2048;

/// Default outbound MAVLink TX byte ring capacity.
pub const DEFAULT_TX_RING_BYTES: usize = 8192;

const COMMAND_QUEUE_CAP: usize = 16;
/// MAVLink system ID that ground stations use by convention.
const GROUND_STATION_SYSTEM_ID: u8 = 255;
/// MAVLink component ID that addresses every component of a system.
const ALL_COMPONENTS: u8 = 0;

/// A queued command and the link address that sent it, when it came
/// over the link.
#[derive(Debug, Clone)]
struct QueuedCommand {
    command: SystemCommand,
    sender: Option<Sender>,
    /// Geodetic position of a target that the board must express in
    /// its present frame before it flies the command.
    geodetic: Option<GeodeticTarget>,
}

/// A geodetic position target from `SET_POSITION_TARGET_GLOBAL_INT`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GeodeticTarget {
    /// Geodetic latitude \[deg\].
    pub latitude_deg: f64,
    /// Longitude \[deg\].
    pub longitude_deg: f64,
    /// Altitude above mean sea level \[m\].
    pub altitude_msl_m: f64,
}
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
    cmd_queue: [Option<QueuedCommand>; COMMAND_QUEUE_CAP],
    cmd_head: usize,
    cmd_len: usize,
    /// Sender of the command that `poll_command` returned last. Its
    /// acknowledgement goes back to this address.
    ack_target: Option<Sender>,
    /// Geodetic target of the command that `poll_command` returned last.
    polled_geodetic: Option<GeodeticTarget>,
    /// Indicated airspeed target \[m/s\] from `MAV_CMD_DO_CHANGE_SPEED`.
    airspeed_target: Option<f32>,
    /// Whether the vehicle flies a commanded airspeed.
    airspeed_commandable: bool,
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
            ack_target: None,
            polled_geodetic: None,
            airspeed_target: None,
            airspeed_commandable: false,
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
        self.enqueue(cmd, None, None)
    }

    fn enqueue(
        &mut self,
        command: SystemCommand,
        sender: Option<Sender>,
        geodetic: Option<GeodeticTarget>,
    ) -> bool {
        if self.cmd_len >= COMMAND_QUEUE_CAP {
            return false;
        }
        let tail = (self.cmd_head.wrapping_add(self.cmd_len)) % COMMAND_QUEUE_CAP;
        if let Some(slot) = self.cmd_queue.get_mut(tail) {
            *slot = Some(QueuedCommand {
                command,
                sender,
                geodetic,
            });
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
        let queued = self.cmd_queue.get_mut(self.cmd_head).and_then(Option::take);
        self.cmd_head = (self.cmd_head.wrapping_add(1)) % COMMAND_QUEUE_CAP;
        self.cmd_len = self.cmd_len.saturating_sub(1);
        if self.cmd_len == 0 {
            self.cmd_head = 0;
        }
        let queued = queued?;
        self.ack_target = queued.sender;
        self.polled_geodetic = queued.geodetic;
        Some(queued.command)
    }

    /// Whether the command that [`Self::poll_command`] returned last came
    /// over the link, as opposed to the typed entry.
    #[must_use]
    pub const fn polled_from_link(&self) -> bool {
        self.ack_target.is_some()
    }

    /// Geodetic position of the target that [`Self::poll_command`]
    /// returned last, when it came as `SET_POSITION_TARGET_GLOBAL_INT`.
    /// The returned command then carries no local position yet.
    #[must_use]
    pub const fn polled_geodetic_target(&self) -> Option<GeodeticTarget> {
        self.polled_geodetic
    }

    /// Declare whether the vehicle flies a commanded airspeed. A vehicle
    /// that does not refuses `MAV_CMD_DO_CHANGE_SPEED`.
    pub fn set_airspeed_commandable(&mut self, commandable: bool) {
        self.airspeed_commandable = commandable;
    }

    /// Indicated airspeed target \[m/s\] that the last accepted
    /// `MAV_CMD_DO_CHANGE_SPEED` set, `None` for the cruise airspeed.
    #[must_use]
    pub const fn airspeed_target(&self) -> Option<f32> {
        self.airspeed_target
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
        self.ack_target = None;
        self.polled_geodetic = None;
        self.airspeed_target = None;
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
        self.emit_command_ack(mav_command, result, detail, self.ack_target);
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
                InboundAction::Enqueue { command, sender } => {
                    let _ = self.enqueue(command, Some(sender), None);
                }
                InboundAction::EnqueueGeodetic {
                    command,
                    target,
                    sender,
                } => {
                    let _ = self.enqueue(command, Some(sender), Some(target));
                }
                InboundAction::Airspeed { request, sender } => {
                    let result = self.apply_airspeed(request);
                    self.emit_command_ack(mav_cmd::DO_CHANGE_SPEED, result, 0, Some(sender));
                }
                InboundAction::ImmediateAck {
                    command,
                    result,
                    sender,
                } => {
                    self.emit_command_ack(command, result, 0, Some(sender));
                }
                InboundAction::Ignore => {}
            }
        }
    }

    /// Apply an airspeed request and return its `MAV_RESULT`. The target
    /// persists until the next request or a world reset.
    fn apply_airspeed(&mut self, request: AirspeedRequest) -> u8 {
        if !self.airspeed_commandable {
            return mav_result::UNSUPPORTED;
        }
        match request {
            AirspeedRequest::Set(speed) => self.airspeed_target = Some(speed),
            AirspeedRequest::Default => self.airspeed_target = None,
            AirspeedRequest::NoChange => {}
        }
        mav_result::ACCEPTED
    }

    /// Acknowledge `command` to the system and component that sent it.
    ///
    /// A command that came through the typed entry has no sender on the
    /// link. Its acknowledgement goes to the ground-station system, all
    /// components.
    fn emit_command_ack(&mut self, command: u16, result: u8, detail: i32, to: Option<Sender>) {
        let to = to.unwrap_or(Sender {
            system: GROUND_STATION_SYSTEM_ID,
            component: ALL_COMPONENTS,
        });
        let ack = CommandAck {
            command,
            result,
            progress: 0,
            result_param2: detail,
            target_system: to.system,
            target_component: to.component,
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
