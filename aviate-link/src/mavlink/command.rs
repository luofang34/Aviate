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
use aviate_hal_io::SystemCommand;

use super::protocol::{mav_cmd, parse_mavlink, CommandLong, MavMessage};

use crate::command::{Command, CommandKind, CommandLink, SignatureMeta, MAX_SIGNED_FRAME_SIZE};
use crate::errors::{LinkError, LinkResult};

/// Everything one MAVLink frame yields for the security boundary.
///
/// Both fields are decoded from the SAME bytes in a single
/// [`parse_system_command`] call: `signature.raw_frame` is the buffer the
/// command was parsed from, so a signature can never be paired with a
/// command it did not cover.
#[derive(Debug)]
pub struct ParsedSystemCommand {
    /// The kernel-facing command the frame carries. Still UNVERIFIED —
    /// only `aviate-security`'s gateway may turn it into a trusted value.
    pub command: SystemCommand,
    /// Signature material parsed from the same frame, if signed.
    pub signature: Option<SignatureMeta>,
}

/// This vehicle's MAVLink address, for filtering commands aimed elsewhere.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct LocalAddress {
    /// This vehicle's system id.
    pub system_id: u8,
    /// This vehicle's component id.
    pub component_id: u8,
}

impl LocalAddress {
    /// Whether a command carrying these target ids is for us.
    ///
    /// Zero is MAVLink's broadcast address and is accepted for both
    /// fields, which is what a ground station sends when it has not yet
    /// learned the vehicle's ids.
    pub fn accepts(&self, target_system: u8, target_component: u8) -> bool {
        let system_ok = target_system == 0 || target_system == self.system_id;
        let component_ok = target_component == 0 || target_component == self.component_id;
        system_ok && component_ok
    }
}

/// Decode a kernel-facing [`SystemCommand`] and its signature metadata
/// from one MAVLink frame.
///
/// This is the parse entry the verified-command gateway builds on. The
/// command and the signature coverage come from the same buffer, and the
/// frame must occupy it exactly:
///
/// - protocol-level failures propagate as [`LinkError::Parse`];
/// - trailing bytes beyond the parsed frame, or a frame too large for the
///   fixed signature buffer, are [`LinkError::FrameLengthMismatch`] —
///   such bytes would sit outside the signature's coverage;
/// - messages with no system-command mapping are
///   [`LinkError::UnsupportedMsg`]. Only the discrete arm/disarm command
///   maps; setpoint streams use the separate flight-control path.
///
/// `local` names this vehicle so a command addressed elsewhere can be
/// dropped. That is an addressing filter, NOT an authorization input:
/// authority comes from the verified credential and nothing here may
/// widen it. Discarding addressing entirely is what lets a fleet ground
/// station holding one key disarm a second, airborne vehicle when the
/// operator disarms a first one on the ground.
pub fn parse_system_command(frame: &[u8], local: LocalAddress) -> LinkResult<ParsedSystemCommand> {
    let (msg, mav_sig, consumed) = parse_mavlink(frame).map_err(LinkError::Parse)?;
    if consumed != frame.len() || frame.len() > MAX_SIGNED_FRAME_SIZE {
        return Err(LinkError::FrameLengthMismatch {
            frame_len: frame.len(),
            consumed,
        });
    }

    let signature = mav_sig.map(|sig| {
        let mut raw_frame = [0u8; MAX_SIGNED_FRAME_SIZE];
        raw_frame[..frame.len()].copy_from_slice(frame);
        SignatureMeta {
            system_id: sig.system_id,
            component_id: sig.component_id,
            link_id: sig.link_id,
            timestamp: sig.timestamp,
            sig: sig.signature,
            raw_frame,
            raw_frame_len: frame.len(),
        }
    });

    let command = match &msg {
        MavMessage::CommandLong(cmd) if cmd.command == mav_cmd::COMPONENT_ARM_DISARM => {
            if !local.accepts(cmd.target_system, cmd.target_component) {
                return Err(LinkError::WrongAddressee {
                    target_system: cmd.target_system,
                    target_component: cmd.target_component,
                });
            }
            // Exact values only. `param1 > 0.5` maps NaN and every other
            // malformed value to Disarm, and Disarm is the direction that
            // ends a flight.
            if cmd.param1 == 1.0 {
                SystemCommand::Arm
            } else if cmd.param1 == 0.0 {
                SystemCommand::Disarm
            } else {
                return Err(LinkError::UnsupportedMsg);
            }
        }
        _ => return Err(LinkError::UnsupportedMsg),
    };

    Ok(ParsedSystemCommand { command, signature })
}

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
            _ => None,
        }
    }

    /// Map COMMAND_LONG message to Command
    fn map_command_long(
        cmd: &CommandLong,
        now_ms: u32,
        signature: Option<SignatureMeta>,
    ) -> Option<Command> {
        match cmd.command {
            mav_cmd::COMPONENT_ARM_DISARM => {
                let arm = cmd.param1 > 0.5;
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
            mav_cmd::DO_SET_MODE => Some(Command {
                kind: CommandKind::SetMode,
                params: [cmd.param1, cmd.param2, 0.0, 0.0, 0.0, 0.0, 0.0],
                timestamp_ms: now_ms,
                signature,
            }),
            _ => None,
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
        let (msg, mav_sig, consumed) = parse_mavlink(&buf[..len]).map_err(LinkError::Parse)?;

        // Convert MAVLink signature to SignatureMeta (if present)
        // Uses static buffer instead of Vec for DO-178C compliance
        let signature = mav_sig.map(|sig| {
            let mut raw_frame = [0u8; MAX_SIGNED_FRAME_SIZE];
            let copy_len = consumed.min(MAX_SIGNED_FRAME_SIZE);
            raw_frame[..copy_len].copy_from_slice(&buf[..copy_len]);
            SignatureMeta {
                system_id: sig.system_id,
                component_id: sig.component_id,
                link_id: sig.link_id,
                timestamp: sig.timestamp,
                sig: sig.signature,
                raw_frame,
                raw_frame_len: copy_len,
            }
        });

        // Map to domain-level Command
        match Self::map_mavlink_to_command(msg, now_ms, signature) {
            Some(cmd) => Ok(Some(cmd)),
            None => Err(LinkError::UnsupportedMsg),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mavlink::protocol::serialize_mavlink;

    // COMMAND_LONG, command=400 (COMPONENT_ARM_DISARM), param1=1.0,
    // unsigned. Golden frame produced by pymavlink 2.4.41
    // (pymavlink.dialects.v20.common, srcSystem=1, srcComponent=1).
    const PYMAVLINK_ARM: &[u8] = &[
        0xFD, 0x1E, 0x00, 0x00, 0x07, 0x01, 0x01, 0x4C, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x90, 0x01, 0xCA, 0xEA,
    ];

    // HEARTBEAT (msgid 0), unsigned — parses fine but has no
    // system-command mapping. Same pymavlink provenance.
    const PYMAVLINK_HEARTBEAT: &[u8] = &[
        0xFD, 0x09, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02,
        0x00, 0x00, 0x04, 0x03, 0x96, 0x44,
    ];

    #[test]
    fn arm_frame_decodes_to_arm_command() {
        let parsed = match parse_system_command(
            PYMAVLINK_ARM,
            LocalAddress {
                system_id: 1,
                component_id: 1,
            },
        ) {
            Ok(p) => p,
            Err(e) => unreachable!("golden ARM frame must parse: {e:?}"),
        };
        assert!(matches!(parsed.command, SystemCommand::Arm));
        assert!(parsed.signature.is_none());
    }

    #[test]
    fn disarm_maps_from_param1_zero() {
        let msg = MavMessage::CommandLong(CommandLong {
            param1: 0.0,
            param2: 0.0,
            param3: 0.0,
            param4: 0.0,
            param5: 0.0,
            param6: 0.0,
            param7: 0.0,
            command: mav_cmd::COMPONENT_ARM_DISARM,
            target_system: 1,
            target_component: 1,
            confirmation: 0,
        });
        let mut buf = [0u8; 64];
        let len = serialize_mavlink(&msg, 3, 1, 1, &mut buf).unwrap_or(0);
        assert!(len > 0);
        assert!(matches!(
            parse_system_command(
                &buf[..len],
                LocalAddress {
                    system_id: 1,
                    component_id: 1
                }
            )
            .map(|p| p.command),
            Ok(SystemCommand::Disarm)
        ));
    }

    /// Bytes past the parsed frame sit outside any signature coverage, so
    /// a buffer that is not exactly one frame is rejected.
    #[test]
    fn trailing_bytes_are_rejected() {
        let mut buf = [0u8; 64];
        let n = PYMAVLINK_ARM.len();
        buf[..n].copy_from_slice(PYMAVLINK_ARM);
        buf[n] = 0xAA;
        assert!(matches!(
            parse_system_command(&buf[..n + 1], LocalAddress { system_id: 1, component_id: 1 }),
            Err(LinkError::FrameLengthMismatch {
                frame_len,
                consumed,
            }) if frame_len == n + 1 && consumed == n
        ));
    }

    #[test]
    fn unmapped_message_is_rejected() {
        assert!(matches!(
            parse_system_command(
                PYMAVLINK_HEARTBEAT,
                LocalAddress {
                    system_id: 1,
                    component_id: 1
                }
            ),
            Err(LinkError::UnsupportedMsg)
        ));
    }
}

#[cfg(test)]
mod addressing_tests {
    use super::*;
    use crate::mavlink::protocol::{serialize_mavlink, CommandLong, MavMessage};

    const US: LocalAddress = LocalAddress {
        system_id: 1,
        component_id: 1,
    };

    /// An arm/disarm COMMAND_LONG aimed at `(target_system, target_component)`.
    fn addressed(target_system: u8, target_component: u8, param1: f32) -> ([u8; 280], usize) {
        let msg = MavMessage::CommandLong(CommandLong {
            target_system,
            target_component,
            command: mav_cmd::COMPONENT_ARM_DISARM,
            confirmation: 0,
            param1,
            param2: 0.0,
            param3: 0.0,
            param4: 0.0,
            param5: 0.0,
            param6: 0.0,
            param7: 0.0,
        });
        let mut buf = [0u8; 280];
        // The crate forbids expect/panic, so surface a failed serialize as
        // a zero length and let each caller's assertion report it.
        let n = serialize_mavlink(&msg, 0, 255, 190, &mut buf).unwrap_or(0);
        assert!(n > 0, "serialize failed");
        (buf, n)
    }

    #[test]
    fn a_command_for_this_vehicle_is_accepted() {
        let (buf, n) = addressed(1, 1, 1.0);
        let parsed = parse_system_command(&buf[..n], US);
        assert!(
            matches!(
                parsed,
                Ok(ParsedSystemCommand {
                    command: SystemCommand::Arm,
                    ..
                })
            ),
            "a command addressed to us must be accepted: {parsed:?}"
        );
    }

    #[test]
    fn a_broadcast_command_is_accepted() {
        // Zero is MAVLink's broadcast address, which is what a station
        // sends before it has learned the vehicle's ids.
        let (buf, n) = addressed(0, 0, 1.0);
        assert!(parse_system_command(&buf[..n], US).is_ok());
    }

    #[test]
    fn a_command_for_another_vehicle_is_refused() {
        // The hazard: a fleet station holding one key disarms vehicle 2 on
        // the ground, and vehicle 1 — airborne, same datalink, same key —
        // honours the same frame.
        let (buf, n) = addressed(2, 1, 0.0);
        assert!(
            matches!(
                parse_system_command(&buf[..n], US),
                Err(LinkError::WrongAddressee {
                    target_system: 2,
                    ..
                })
            ),
            "a disarm aimed at another vehicle must not be honoured here"
        );
    }

    #[test]
    fn a_command_for_another_component_is_refused() {
        let (buf, n) = addressed(1, 42, 0.0);
        assert!(matches!(
            parse_system_command(&buf[..n], US),
            Err(LinkError::WrongAddressee {
                target_component: 42,
                ..
            })
        ));
    }

    #[test]
    fn a_malformed_arm_parameter_is_refused_rather_than_read_as_disarm() {
        // `param1 > 0.5` mapped NaN to Disarm, and Disarm is the direction
        // that ends a flight.
        for bogus in [f32::NAN, 0.5, -1.0, 7.0] {
            let (buf, n) = addressed(1, 1, bogus);
            assert!(
                matches!(
                    parse_system_command(&buf[..n], US),
                    Err(LinkError::UnsupportedMsg)
                ),
                "param1={bogus} must be refused, not decoded"
            );
        }
    }
}
