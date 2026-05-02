//! USB CDC ACM Serial Transport with MAVLink Protocol
//!
//! Real USB CDC implementation for MicoAir H743-V2 board.
//! Wraps `aviate_hal_stm32h7::Stm32h7UsbCdc` and adds MAVLink protocol layer.
//!
//! ## Features
//!
//! - USB CDC ACM serial port (VID:PID 0x0483:0x5740)
//! - MAVLink 2.0 protocol parsing
//! - 1Hz heartbeat output
//! - Command parsing (ARM/DISARM, SET_ATTITUDE_TARGET)
//!
//! ## DO-178C Compliance
//!
//! - No unwrap/expect/panic
//! - Bounded loops (SERVICE_MAX_ITERS, SERVICE_MAX_BYTES)
//! - Static buffers (no heap allocation)
//! - Counter-based timing (no floating point in timing)

use aviate_core::control::{Command, ControlMode, Setpoint};
use aviate_hal_io::SystemCommand;
use aviate_core::math::Quaternion;
use aviate_core::types::{FloatExt, Normalized, RadiansPerSecond};
use aviate_hal_io::transport_hal::{
    SystemState, TransportHal, TransportStatus as TransportHalStatus,
};
use aviate_hal_stm32h7::{Stm32h7UsbCdc, UsbMetrics};

use aviate_link::mavlink::protocol::{
    mav_cmd, parse_mavlink, serialize_mavlink, Heartbeat, MavAutopilot, MavMessage, MavModeFlag,
    MavState, MavType, MAVLINK_STX_V2,
};

use stm32h7xx_hal::usb_hs::USB2;

// =============================================================================
// Constants
// =============================================================================

/// Maximum bytes to process per service call (bounded WCET)

/// MAVLink frame accumulator size
const MAV_BUF_SIZE: usize = 300;

/// Heartbeat interval in ticks (1Hz at 1kHz tick rate = 1000 ticks)
const HEARTBEAT_INTERVAL_TICKS: u32 = 1000;

// =============================================================================
// MAVLink Parser State
// =============================================================================

/// Generic Link frame parser state
enum LinkParserState {
    /// Waiting for start byte
    WaitingForStx,
    /// Accumulating frame bytes
    Accumulating { expected_len: usize },
}

/// Generic Link frame parser (wrapping protocol parser)
struct LinkParser {
    state: LinkParserState,
    buf: [u8; MAV_BUF_SIZE],
    pos: usize,
}

impl LinkParser {
    const fn new() -> Self {
        Self {
            state: LinkParserState::WaitingForStx,
            buf: [0; MAV_BUF_SIZE],
            pos: 0,
        }
    }

    /// Feed a byte to the parser
    ///
    /// Returns true if a complete frame is ready (call parse_frame())
    fn feed(&mut self, byte: u8) -> bool {
        match &mut self.state {
            LinkParserState::WaitingForStx => {
                if byte == MAVLINK_STX_V2 {
                    self.buf[0] = byte;
                    self.pos = 1;
                    self.state = LinkParserState::Accumulating { expected_len: 0 };
                }
                false
            }
            LinkParserState::Accumulating { expected_len } => {
                if self.pos >= MAV_BUF_SIZE {
                    // Buffer overflow, reset
                    self.state = LinkParserState::WaitingForStx;
                    self.pos = 0;
                    return false;
                }

                self.buf[self.pos] = byte;
                self.pos += 1;

                // After receiving header (10 bytes), we can calculate expected length
                if self.pos == 10 && *expected_len == 0 {
                    // buf[1] = payload length
                    // buf[2] = incompat flags (bit 0 = signed)
                    let payload_len = self.buf[1] as usize;
                    let is_signed = (self.buf[2] & 0x01) != 0;
                    let sig_len = if is_signed { 13 } else { 0 };
                    *expected_len = 10 + payload_len + 2 + sig_len; // header + payload + crc + sig
                }

                // Check if frame complete
                *expected_len > 0 && self.pos >= *expected_len
            }
        }
    }

    /// Parse the accumulated frame
    ///
    /// Returns the parsed message if valid
    fn parse_frame(&mut self) -> Option<MavMessage> {
        let result = parse_mavlink(&self.buf[..self.pos]);

        // Reset parser state
        self.state = LinkParserState::WaitingForStx;
        self.pos = 0;

        match result {
            Ok((msg, _sig, _consumed)) => Some(msg),
            Err(_) => None,
        }
    }
}

// =============================================================================
// Quaternion Helper
// =============================================================================

/// Convert Euler angles (roll, pitch, yaw) to quaternion using ZYX convention
///
/// # Arguments
/// - `roll`: rotation about X axis in radians
/// - `pitch`: rotation about Y axis in radians
/// - `yaw`: rotation about Z axis in radians
fn quaternion_from_euler(roll: f32, pitch: f32, yaw: f32) -> Quaternion {
    let cr = (roll * 0.5).cos();
    let sr = (roll * 0.5).sin();
    let cp = (pitch * 0.5).cos();
    let sp = (pitch * 0.5).sin();
    let cy = (yaw * 0.5).cos();
    let sy = (yaw * 0.5).sin();

    Quaternion::new(
        cr * cp * cy + sr * sp * sy, // w
        sr * cp * cy - cr * sp * sy, // x
        cr * sp * cy + sr * cp * sy, // y
        cr * cp * sy - sr * sp * cy, // z
    )
}

// =============================================================================
// SerialTransport
// =============================================================================

/// USB CDC serial transport with Link protocol support (currently MAVLink)
///
/// Implements `TransportHal<SystemCommand>` for FlightRunner integration.
pub struct SerialTransport {
    /// Raw USB CDC driver from HAL
    usb: Stm32h7UsbCdc,
    /// Protocol frame parser
    parser: LinkParser,
    /// System state (for heartbeat)
    system_state: SystemState,
    /// Armed state (for heartbeat)
    armed: bool,
    /// Tx sequence counter
    tx_seq: u8,
    /// Tick counter for heartbeat timing
    tick_counter: u32,
    #[cfg(feature = "software-bootloader")]
    dfu_state: DfuState,
}

impl SerialTransport {
    /// Create new transport (USB not initialized yet)
    ///
    /// Call `init_usb()` after clocks are configured.
    pub fn new() -> Self {
        Self {
            usb: Stm32h7UsbCdc::new(),
            parser: LinkParser::new(),
            system_state: SystemState::Boot,
            armed: false,
            tx_seq: 0,
            tick_counter: 0,
            #[cfg(feature = "software-bootloader")]
            dfu_state: DfuState::default(),
        }
    }

    /// Initialize USB hardware with pre-configured USB2 peripheral (OTG_FS)
    pub fn init_usb(&mut self, usb2: USB2) {
        self.usb.init(usb2);
    }

    /// Service USB hardware (call at tick rate from poll())
    ///
    /// USB CDC requires frequent polling to process IN/OUT tokens.
    /// Multiple service() calls ensure data is flushed to host.
    fn service_usb(&mut self) {
        // Service multiple times per tick to ensure USB events are processed
        for _ in 0..4 {
            self.usb.service();
        }
    }

    /// Send heartbeat if interval elapsed
    fn maybe_send_heartbeat(&mut self) {
        self.tick_counter = self.tick_counter.wrapping_add(1);

        if self.tick_counter % HEARTBEAT_INTERVAL_TICKS == 0 {
            self.send_heartbeat();
        }
    }

    /// Send a protocol heartbeat
    fn send_heartbeat(&mut self) {
        // Map SystemState to MavState
        let mav_state = match self.system_state {
            SystemState::Uninit => MavState::Uninit,
            SystemState::Boot => MavState::Boot,
            SystemState::Calibrating => MavState::Calibrating,
            SystemState::Standby => MavState::Standby,
            SystemState::Active => MavState::Active,
            SystemState::Critical => MavState::Critical,
            SystemState::Emergency => MavState::Emergency,
        };

        // Build base_mode flags
        let mut base_mode = 0u8;
        if self.armed {
            base_mode |= MavModeFlag::SAFETY_ARMED.0;
        }
        base_mode |= MavModeFlag::CUSTOM_MODE_ENABLED.0;
        base_mode |= MavModeFlag::STABILIZE_ENABLED.0;

        let heartbeat = Heartbeat {
            mav_type: MavType::Quadrotor as u8,
            autopilot: MavAutopilot::Aviate as u8,
            base_mode,
            custom_mode: 0,
            system_status: mav_state as u8,
            mavlink_version: 3,
        };

        let msg = MavMessage::Heartbeat(heartbeat);
        let mut buf = [0u8; 32];

        if let Some(len) = serialize_mavlink(&msg, self.tx_seq, 1, 1, &mut buf) {
            self.tx_seq = self.tx_seq.wrapping_add(1);
            let _ = self.usb.try_write(&buf[..len]);
            // Flush immediately after write
            self.usb.service();
        }
    }

    /// Try to parse incoming protocol data and convert to SystemCommand
    fn try_parse_command(&mut self) -> Option<SystemCommand> {
        #[cfg(feature = "software-bootloader")]
        self.update_dfu_state();

        // Feed RX buffer bytes to parser
        let mut rx_buf = [0u8; 64];

        // Read available bytes from USB
        let count = self.usb.try_read(&mut rx_buf);

        for &byte in &rx_buf[..count] {
            #[cfg(feature = "software-bootloader")]
            if self.handle_dfu_protocol(byte) {
                continue;
            }

            if self.parser.feed(byte) {
                // Complete frame, try to parse
                if let Some(msg) = self.parser.parse_frame() {
                    if let Some(cmd) = self.decode_packet(msg) {
                        return Some(cmd);
                    }
                }
            }
        }
        None
    }

    /// Decode protocol message to SystemCommand
    fn decode_packet(&self, msg: MavMessage) -> Option<SystemCommand> {
        match msg {
            MavMessage::CommandLong(cmd) => {
                match cmd.command {
                    mav_cmd::COMPONENT_ARM_DISARM => {
                        // param1 > 0.5 = arm, otherwise disarm
                        if cmd.param1 > 0.5 {
                            Some(SystemCommand::Arm)
                        } else {
                            Some(SystemCommand::Disarm)
                        }
                    }
                    _ => None,
                }
            }
            MavMessage::SetAttitudeTarget(tgt) => {
                // Convert quaternion and thrust to Command
                let q = Quaternion::new(tgt.q[0], tgt.q[1], tgt.q[2], tgt.q[3]);
                let thrust = Normalized(tgt.thrust.clamp(0.0, 1.0));

                let command = Command {
                    mode: ControlMode::Attitude,
                    setpoint: Setpoint {
                        attitude: Some(q),
                        collective_thrust: thrust,
                        ..Default::default()
                    },
                    config_mode_request: None,
                    sensor_overrides: None,
                    sequence: 0,
                    source: aviate_core::control::CommandSource::Gcs,
                };

                Some(SystemCommand::FlightControl(command))
            }
            MavMessage::ManualControl(mc) => {
                // Convert joystick input to attitude setpoint
                // x = pitch, y = roll, z = throttle, r = yaw
                // Range: -1000 to 1000 (except z: 0-1000)
                let roll = (mc.y as f32) / 1000.0 * 0.5; // Max 0.5 rad
                let pitch = (mc.x as f32) / 1000.0 * 0.5;
                let yaw_rate = (mc.r as f32) / 1000.0 * 1.0; // Max 1 rad/s
                let thrust = Normalized(((mc.z as f32) / 1000.0).clamp(0.0, 1.0));

                // Convert to quaternion (simplified - just roll/pitch, no yaw)
                let q = quaternion_from_euler(roll, pitch, 0.0);

                let command = Command {
                    mode: ControlMode::Attitude,
                    setpoint: Setpoint {
                        attitude: Some(q),
                        angular_rate: Some([
                            RadiansPerSecond(0.0),
                            RadiansPerSecond(0.0),
                            RadiansPerSecond(yaw_rate),
                        ]),
                        collective_thrust: thrust,
                        ..Default::default()
                    },
                    config_mode_request: None,
                    sensor_overrides: None,
                    sequence: 0,
                    source: aviate_core::control::CommandSource::Pilot,
                };

                Some(SystemCommand::FlightControl(command))
            }
            // Ignore other messages (heartbeat from GCS, etc.)
            _ => None,
        }
    }

    /// Access inner metrics
    pub fn metrics(&self) -> UsbMetrics {
        *self.usb.metrics()
    }
}

// =============================================================================
// Software DFU Implementation
// =============================================================================

/// DFU confirmation timeout in ticks (10 seconds at 1kHz = 10000 ticks)
#[cfg(feature = "software-bootloader")]
const DFU_TIMEOUT_TICKS: u32 = 10000;

/// DFU retransmit interval in ticks (200ms at 1kHz = 200 ticks)
#[cfg(feature = "software-bootloader")]
const DFU_RETRANSMIT_TICKS: u32 = 200;

#[cfg(feature = "software-bootloader")]
#[derive(Debug, Clone, Copy, PartialEq)]
enum DfuState {
    Idle,
    /// Matching "dfu" command
    WaitDfu(usize),
    /// Waiting for confirmation code response
    WaitConfirm {
        /// Random 4-digit confirmation code (1000-9999)
        code: u16,
        /// Index into receive buffer
        idx: usize,
        /// Receive buffer for user response (4 digits + \r + \n)
        buf: [u8; 6],
        /// Tick counter when challenge was sent (for retransmit)
        last_sent: u32,
        /// Tick counter when challenge started (for timeout)
        start_tick: u32,
    },
}

#[cfg(feature = "software-bootloader")]
impl Default for DfuState {
    fn default() -> Self {
        Self::Idle
    }
}

#[cfg(feature = "software-bootloader")]
impl SerialTransport {
    /// Generate a pseudo-random 4-digit code (1000-9999) from DWT cycle counter
    fn generate_dfu_code() -> u16 {
        // Read DWT cycle counter for randomness
        // Safety: DWT is initialized in hw.rs before USB transport
        let cyccnt = unsafe { (*cortex_m::peripheral::DWT::PTR).cyccnt.read() };
        // Map to range 1000-9999 (4-digit code)
        1000 + ((cyccnt % 9000) as u16)
    }

    /// Handle software DFU protocol sniffed from RX stream
    /// Returns true if bytes were consumed (should not pass to MAVLink)
    fn handle_dfu_protocol(&mut self, byte: u8) -> bool {
        match self.dfu_state {
            DfuState::Idle => {
                if byte == b'd' {
                    self.dfu_state = DfuState::WaitDfu(1);
                    // Safe to consume as MAVLink starts with 0xFD
                    return true;
                }
                false
            }
            DfuState::WaitDfu(idx) => {
                let target = b"dfu";
                if byte == target[idx] {
                    if idx == target.len() - 1 {
                        // Match "dfu" complete! Generate code and send confirmation.
                        let now = self.tick_counter;
                        let code = Self::generate_dfu_code();
                        self.send_dfu_confirm(code);
                        self.dfu_state = DfuState::WaitConfirm {
                            code,
                            idx: 0,
                            buf: [0; 6],
                            last_sent: now,
                            start_tick: now,
                        };
                    } else {
                        self.dfu_state = DfuState::WaitDfu(idx + 1);
                    }
                    true
                } else {
                    // Mismatch, reset
                    self.dfu_state = DfuState::Idle;
                    false
                }
            }
            DfuState::WaitConfirm {
                code,
                mut idx,
                mut buf,
                last_sent,
                start_tick,
            } => {
                // Ignore whitespace/control chars
                if byte == b'\r' || byte == b'\n' {
                    // If we have 4 digits, check match
                    if idx >= 4 {
                        let received = Self::parse_code(&buf[..4]);
                        if received == Some(code) {
                            let _ = self.usb.try_write(b"REBOOTING\r\n");
                            self.usb.service();
                            self.reboot_to_bootloader();
                        }
                        // Wrong code, reset
                        let _ = self.usb.try_write(b"DFU:WRONGCODE\r\n");
                        self.dfu_state = DfuState::Idle;
                    }
                    return true;
                }

                // Only accept ASCII digits for the code
                if byte >= b'0' && byte <= b'9' {
                    if idx < 4 {
                        buf[idx] = byte;
                        idx += 1;

                        // Check match when we have exactly 4 digits
                        if idx == 4 {
                            let received = Self::parse_code(&buf[..4]);
                            if received == Some(code) {
                                let _ = self.usb.try_write(b"REBOOTING\r\n");
                                self.usb.service();
                                self.reboot_to_bootloader();
                            }
                            // Code doesn't match, but wait for \r\n to confirm
                        }

                        // Update state
                        self.dfu_state = DfuState::WaitConfirm {
                            code,
                            idx,
                            buf,
                            last_sent,
                            start_tick,
                        };
                    } else {
                        // More than 4 digits = wrong code, reset
                        self.dfu_state = DfuState::Idle;
                    }
                    return true;
                }

                // Non-digit, non-whitespace byte (e.g., MAVLink 0xFD) - ignore it
                // Don't consume so MAVLink parser can still process if needed
                // But stay in WaitConfirm state
                false
            }
        }
    }

    /// Parse 4 ASCII digit bytes into a u16
    fn parse_code(bytes: &[u8]) -> Option<u16> {
        if bytes.len() < 4 {
            return None;
        }
        let mut val: u16 = 0;
        for &b in &bytes[..4] {
            if b < b'0' || b > b'9' {
                return None;
            }
            val = val * 10 + (b - b'0') as u16;
        }
        Some(val)
    }

    /// Send DFU confirmation challenge with the given code
    fn send_dfu_confirm(&mut self, code: u16) {
        // Format: "CONFIRM:XXXX\r\n" where XXXX is the 4-digit code
        let mut msg = *b"CONFIRM:0000\r\n";
        msg[8] = b'0' + ((code / 1000) % 10) as u8;
        msg[9] = b'0' + ((code / 100) % 10) as u8;
        msg[10] = b'0' + ((code / 10) % 10) as u8;
        msg[11] = b'0' + (code % 10) as u8;
        let _ = self.usb.try_write(&msg);
        // Flush immediately after write
        self.usb.service();
    }

    /// Periodic update to handle retransmission and timeout
    fn update_dfu_state(&mut self) {
        match self.dfu_state {
            DfuState::WaitConfirm {
                code,
                idx,
                buf,
                last_sent,
                start_tick,
            } => {
                let now = self.tick_counter;

                // Check timeout (5 seconds)
                if now.wrapping_sub(start_tick) > DFU_TIMEOUT_TICKS {
                    let _ = self.usb.try_write(b"DFU:TIMEOUT\r\n");
                    self.dfu_state = DfuState::Idle;
                    return;
                }

                // Retransmit every 200ms
                if now.wrapping_sub(last_sent) > DFU_RETRANSMIT_TICKS {
                    self.send_dfu_confirm(code);
                    self.dfu_state = DfuState::WaitConfirm {
                        code,
                        idx,
                        buf,
                        last_sent: now,
                        start_tick,
                    };
                }
            }
            _ => {}
        }
    }

    fn reboot_to_bootloader(&mut self) -> ! {
        // Simplified approach matching the old working bootloader code:
        // - Single magic value 0xB007_B007 in RTC_BK0R
        // - No PWREN/RTCAPBEN, just DBP enable
        const RTC_BK0R: u32 = 0x5800_4050;
        const BOOT_MAGIC: u32 = 0xB007_B007;

        unsafe {
            // Enable backup domain write access (PWR.CR1.DBP bit 8)
            // The old working code only set DBP, didn't touch PWREN/RTCAPBEN
            let pwr = &*stm32h7xx_hal::pac::PWR::ptr();
            pwr.cr1.modify(|r, w| w.bits(r.bits() | (1 << 8)));

            // Memory barrier
            cortex_m::asm::dsb();

            // Write magic to RTC_BK0R
            let ptr = RTC_BK0R as *mut u32;
            core::ptr::write_volatile(ptr, BOOT_MAGIC);

            // Memory barrier to ensure write completes
            cortex_m::asm::dsb();

            // System Reset
            cortex_m::peripheral::SCB::sys_reset();
        }
    }
}

pub type BoardTransport = SerialTransport;

impl Default for SerialTransport {
    fn default() -> Self {
        Self::new()
    }
}

impl TransportHal<SystemCommand> for BoardTransport {
    fn poll(&mut self) {
        // Service USB hardware (read/write ring buffers)
        self.service_usb();

        // Send heartbeat if interval elapsed
        self.maybe_send_heartbeat();
    }

    fn try_recv_command(&mut self) -> Option<SystemCommand> {
        self.try_parse_command()
    }

    fn try_send_telemetry(&mut self, frame: &[u8]) -> bool {
        self.usb.try_write(frame) == frame.len()
    }

    fn set_system_state(&mut self, state: SystemState) {
        self.system_state = state;
    }

    fn set_armed(&mut self, armed: bool) {
        self.armed = armed;
    }

    fn status(&self) -> TransportHalStatus {
        let metrics = self.usb.metrics();
        TransportHalStatus {
            rx_errors: metrics.rx_dropped,
            tx_errors: metrics.tx_dropped,
            connected: self.usb.is_connected(),
        }
    }
}
