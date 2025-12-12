//! Domain-level command abstraction (protocol-agnostic)
//!
//! This module defines the Command type that applications work with.
//! It's independent of any specific protocol (MAVLink, CCSDS, etc.).
//!
//! ## Security Note
//!
//! This module does NOT handle security! No signature checking, no anti-replay.
//! Security logic lives in `aviate-security` crate (CommandGateway).
//!
//! This module only handles: protocol bytes → Command struct (syntax parsing)

// Import alloc for Vec in no_std
extern crate alloc;

use crate::errors::LinkResult;

/// Signature metadata extracted from signed commands
///
/// This structure holds signature information from protocol-level signed
/// messages (e.g., MAVLink 2.0 signed frames). The actual verification
/// happens in `aviate-security::SignedAuth`.
///
/// ## Contents
///
/// - `link_id`: Identifies which key to use (0-255)
/// - `timestamp`: Remote monotonic counter (48-bit, 10μs resolution for MAVLink)
/// - `sig`: Truncated signature bytes (6 bytes for MAVLink HMAC-SHA256)
/// - `raw_frame`: Original frame bytes for HMAC verification
///
/// ## Lifetime
///
/// We store an owned copy of the frame (`Vec<u8>`) to avoid lifetime issues.
/// The security layer needs the raw frame bytes to recompute the HMAC.
#[derive(Clone, Debug)]
pub struct SignatureMeta {
    /// Link identifier (maps to KeySelector)
    pub link_id: u8,

    /// Remote monotonic counter (protocol-specific resolution)
    ///
    /// For MAVLink: 48-bit timestamp in 10 microsecond units
    pub timestamp: u64,

    /// Truncated signature bytes
    ///
    /// For MAVLink: First 6 bytes of HMAC-SHA256
    pub sig: [u8; 6],

    /// Original frame for HMAC verification
    ///
    /// Owned copy to avoid lifetime issues. The security layer will
    /// recompute HMAC over this exact byte sequence.
    pub raw_frame: alloc::vec::Vec<u8>,
}

/// Domain-level command representation (protocol-agnostic).
///
/// All external commands (MAVLink, CCSDS, DDS, etc.) are mapped to this type
/// before being passed to the application layer.
///
/// ## Security Note
///
/// Commands from this module are NOT verified!
/// ALL commands MUST go through `aviate-security::CommandGateway` for verification.
#[derive(Clone, Copy, Debug)]
pub enum CommandKind {
    /// Arm motors (start propellers)
    Arm,
    /// Disarm motors (stop propellers)
    Disarm,
    /// Set flight mode (OFFBOARD, HOLD, RTL, etc.)
    SetMode,
    /// Set attitude setpoint (quaternion + thrust)
    SetAttitude,
    /// Set body rate setpoint (roll/pitch/yaw rates + thrust)
    SetRate,
    /// Set thrust only (normalized [0, 1])
    SetThrust,
    // Future: Add more commands as needed
}

/// Domain-level command with parsed parameters
///
/// ## Security Note
///
/// This struct represents a PARSED but UNVERIFIED command!
/// Do NOT execute commands from this struct directly.
/// Always use `CommandGateway::poll()` to get verified commands.
///
/// ## Signature Support
///
/// If the command came from a signed protocol frame (e.g., MAVLink with
/// MAVLINK_IFLAG_SIGNED), the `signature` field contains the metadata
/// needed for verification in `aviate-security`.
#[derive(Clone, Debug)]
pub struct Command {
    /// Command type
    pub kind: CommandKind,

    /// Generic numeric payload (interpretation depends on kind)
    /// Example: For SetAttitude: [qw, qx, qy, qz, thrust, 0, 0]
    pub params: [f32; 7],

    /// Receiver timestamp (milliseconds since boot)
    pub timestamp_ms: u32,

    /// Optional signature metadata (if frame was signed)
    ///
    /// - `None`: Unsigned command (insecure, for development only)
    /// - `Some(meta)`: Signed command (must be verified by CommandGateway)
    pub signature: Option<SignatureMeta>,
}

/// Protocol-agnostic command link
///
/// Implementations parse protocol-specific messages (MAVLink, CCSDS, etc.)
/// into domain-level Command structs.
///
/// ## Security Note
///
/// This trait does NOT verify commands!
/// It only parses protocol bytes → Command struct.
/// Verification happens in `aviate-security::CommandGateway`.
pub trait CommandLink {
    /// Non-blocking poll for a new command from the transport.
    ///
    /// # Parameters
    ///
    /// - `now_ms`: Current system time (milliseconds since boot)
    ///
    /// # Returns
    ///
    /// - `Ok(None)`: No new command in the RX buffer
    /// - `Ok(Some(cmd))`: One command parsed and mapped successfully
    /// - `Err(LinkError)`: Transport error or parse error
    ///
    /// ## DO-178C Contract
    ///
    /// - Non-blocking: Returns immediately, never waits
    /// - Time complexity: O(frame_len), bounded by frame parsing
    /// - WCET (engineering target): ~10 μs for max frame size
    fn poll_command(&mut self, now_ms: u32) -> LinkResult<Option<Command>>;
}
