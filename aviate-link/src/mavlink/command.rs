//! MAVLink Commands (Inbound: Ground → App)
//!
//! This module implements command reception and parsing using MAVLink protocol.
//!
//! ## DO-178C Data Flow Direction
//!
//! **Inbound ONLY** - This module receives commands from ground station.
//!
//! - ✅ Uses `FrameRx` for reception
//! - ❌ MUST NOT use `FrameTx` (outbound is in telemetry.rs)
//! - ❌ MUST NOT contain security logic (belongs in aviate-security)
//! - ⚠️  All commands are UNVERIFIED - MUST use `CommandGateway`!
//!
//! ## Criticality Level
//!
//! - **DAL A/B** (affects flight safety, requires verification)
//! - Commands from this module are UNTRUSTED
//! - Applications MUST use `aviate-security::CommandGateway` for verification
//! - NEVER execute commands directly from `MavlinkCommandLink`
//!
//! ## Security Warning (DO-178C)
//!
//! ```text
//! ┌─────────────────────────────────────────────────────────────┐
//! │  ⚠️  CRITICAL: Commands from this module are UNVERIFIED! ⚠️  │
//! │                                                              │
//! │  CORRECT usage:                                              │
//! │  let link = MavlinkCommandLink::new(usb_rx);                │
//! │  let gateway = CommandGateway::new(link, auth);   // ✅      │
//! │  if let Ok(cmd) = gateway.poll_command(now_ms) {            │
//! │      kernel.execute(cmd);  // Safe: verified                │
//! │  }                                                           │
//! │                                                              │
//! │  WRONG usage (DO NOT DO THIS):                              │
//! │  let mut link = MavlinkCommandLink::new(usb_rx);            │
//! │  if let Ok(Some(cmd)) = link.poll_command(now_ms) {         │
//! │      kernel.execute(cmd);  // ❌ BYPASSES SECURITY!          │
//! │  }                                                           │
//! └─────────────────────────────────────────────────────────────┘
//! ```
//!
//! ## Audit Checklist
//!
//! When auditing this file, verify:
//! - ✅ No imports of `FrameTx` (only `FrameRx`)
//! - ✅ No imports from `aviate-security`
//! - ✅ No telemetry transmission logic
//! - ✅ No signature verification (must be in aviate-security)
//! - ✅ No anti-replay checks (must be in aviate-security)
//!
//! When auditing applications using this module, verify:
//! - ✅ MavlinkCommandLink is NEVER used directly
//! - ✅ ALL command paths go through CommandGateway
//! - ✅ No bypass paths exist (grep for "poll_command.*MavlinkCommandLink")

use aviate_hal_io::transport::FrameRx;

use super::protocol::{mav_cmd, parse_mavlink, CommandLong, MavMessage};

use crate::command::{Command, CommandKind, CommandLink, SignatureMeta};
use crate::errors::{LinkError, LinkResult};

/// MAVLink command link (parses MAVLink → Command)
///
/// This struct reads raw MAVLink frames from a transport and parses them
/// into domain-level Command structs.
///
/// ## Security Note
///
/// This module does NOT verify commands!
/// All commands from this module are UNVERIFIED.
/// Use `aviate-security::CommandGateway` for verification.
///
/// ## Type Parameters
///
/// - `T`: Transport implementing `FrameRx` (e.g., USB CDC, UART, CAN)
pub struct MavlinkCommandLink<T: FrameRx> {
    /// Transport for receiving frames
    rx: T,
}

impl<T: FrameRx> MavlinkCommandLink<T> {
    /// Create new MAVLink command link
    ///
    /// ## Parameters
    ///
    /// - `rx`: Transport implementing FrameRx
    pub fn new(rx: T) -> Self {
        Self { rx }
    }

    /// Get mutable reference to transport (for configuration)
    pub fn transport_mut(&mut self) -> &mut T {
        &mut self.rx
    }

    /// Map MAVLink message to domain-level Command
    ///
    /// This is a pure function that maps protocol-specific messages
    /// to domain-level commands.
    ///
    /// ## Parameters
    ///
    /// - `msg`: Parsed MAVLink message
    /// - `now_ms`: Current system time (for timestamp)
    /// - `signature`: Optional signature metadata (if frame was signed)
    ///
    /// ## Returns
    ///
    /// - `Some(cmd)`: Message mapped to Command
    /// - `None`: Message not recognized or not mapped
    fn map_mavlink_to_command(
        msg: MavMessage,
        now_ms: u32,
        signature: Option<SignatureMeta>,
    ) -> Option<Command> {
        match msg {
            MavMessage::CommandLong(cmd) => Self::map_command_long(&cmd, now_ms, signature),
            MavMessage::SetAttitudeTarget(tgt) => Some(Command {
                kind: CommandKind::SetAttitude,
                params: [tgt.q[0], tgt.q[1], tgt.q[2], tgt.q[3], tgt.thrust, 0.0, 0.0],
                timestamp_ms: now_ms,
                signature,
            }),
            // Future: Add support for other command messages
            // MavMessage::CommandInt(cmd) => ...
            // MavMessage::SetPositionTargetLocalNed(cmd) => ...
            _ => None, // Ignore unrecognized messages
        }
    }

    /// Map COMMAND_LONG message to Command
    fn map_command_long(
        cmd: &CommandLong,
        now_ms: u32,
        signature: Option<SignatureMeta>,
    ) -> Option<Command> {
        match cmd.command {
            // MAV_CMD_COMPONENT_ARM_DISARM (400)
            mav_cmd::COMPONENT_ARM_DISARM => {
                let arm = cmd.param1 > 0.5; // param1: 1.0 = arm, 0.0 = disarm
                Some(Command {
                    kind: if arm {
                        CommandKind::Arm
                    } else {
                        CommandKind::Disarm
                    },
                    params: [0.0; 7],
                    timestamp_ms: now_ms,
                    signature,
                })
            }

            // MAV_CMD_DO_SET_MODE (176)
            mav_cmd::DO_SET_MODE => {
                // param1: mode (custom mode interpretation)
                Some(Command {
                    kind: CommandKind::SetMode,
                    params: [cmd.param1, cmd.param2, 0.0, 0.0, 0.0, 0.0, 0.0],
                    timestamp_ms: now_ms,
                    signature,
                })
            }

            // Future: Add more MAVLink commands as needed
            // MAV_CMD_NAV_TAKEOFF, MAV_CMD_NAV_LAND, etc.
            _ => None, // Unsupported command
        }
    }
}

impl<T: FrameRx> CommandLink for MavlinkCommandLink<T> {
    fn poll_command(&mut self, now_ms: u32) -> LinkResult<Option<Command>> {
        let mut buf = [0u8; 512]; // Max MAVLink v2 frame: 280 bytes + signature

        // Try to receive a frame
        let len = self.rx.try_recv(&mut buf).map_err(LinkError::Transport)?;

        // No frame available
        if len == 0 {
            return Ok(None);
        }

        // Parse MAVLink frame (extracts signature if present)
        let (msg, mav_sig, consumed) =
            parse_mavlink(&buf[..len]).map_err(|_| LinkError::ParseError)?;

        // Convert MAVLink signature to SignatureMeta (if present)
        let signature = mav_sig.map(|sig| SignatureMeta {
            link_id: sig.link_id,
            timestamp: sig.timestamp,
            sig: sig.signature,
            raw_frame: buf[..consumed].to_vec(), // Owned copy for HMAC verification
        });

        // Map to domain-level Command
        match Self::map_mavlink_to_command(msg, now_ms, signature) {
            Some(cmd) => Ok(Some(cmd)),
            None => Err(LinkError::UnsupportedMsg),
        }
    }
}
