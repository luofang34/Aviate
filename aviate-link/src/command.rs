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

use crate::errors::LinkResult;

/// Maximum frame size for signature verification (MAVLink v2 max + signature)
/// MAVLink v2: 12 header + 255 payload + 2 CRC + 13 signature = 282 bytes
pub const MAX_SIGNED_FRAME_SIZE: usize = 300;

/// Number of trailing bytes of a signed frame that hold the truncated
/// signature itself. Per the MAVLink 2 signing spec the HMAC is computed
/// over the whole frame *except* these bytes; they are excluded from the
/// signed message, while `link_id` and the 48-bit timestamp are included.
pub const MAVLINK_SIGNATURE_TRAILER_LEN: usize = 6;

/// Signature metadata extracted from signed commands
///
/// This structure holds signature information from protocol-level signed
/// messages (e.g., MAVLink 2.0 signed frames). The actual verification
/// happens in `aviate-security::SignedAuth`.
///
/// ## Contents
///
/// - `system_id` / `component_id`: sender identity from the frame header.
///   Anti-replay is keyed on the full `(system_id, component_id, link_id)`
///   tuple, per the MAVLink signing spec — `link_id` alone is not a
///   sender identity.
/// - `link_id`: Identifies which key to use (0-255)
/// - `timestamp`: Remote monotonic counter (48-bit, 10μs resolution for MAVLink)
/// - `sig`: Truncated signature bytes (6 bytes for MAVLink HMAC-SHA256)
/// - `raw_frame`: Original frame bytes for HMAC verification (static buffer)
///
/// ## DO-178C Compliance
///
/// Uses fixed-size static buffer instead of Vec for deterministic memory usage.
/// No heap allocation required.
#[derive(Clone, Debug)]
pub struct SignatureMeta {
    /// Sender system id from the frame header.
    pub system_id: u8,

    /// Sender component id from the frame header.
    pub component_id: u8,

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

    /// Original frame for HMAC verification (static buffer)
    ///
    /// Fixed-size buffer holding the whole received frame, signature bytes
    /// included. Use [`SignatureMeta::signed_message`] to obtain the exact
    /// bytes the HMAC covers — never slice `raw_frame` directly with an
    /// untrusted `raw_frame_len`.
    pub raw_frame: [u8; MAX_SIGNED_FRAME_SIZE],

    /// Actual length of received data in `raw_frame`, signature included.
    pub raw_frame_len: usize,
}

impl SignatureMeta {
    /// The exact bytes the signature is computed over.
    ///
    /// Per the MAVLink 2 signing spec this is the complete frame excluding
    /// the trailing [`MAVLINK_SIGNATURE_TRAILER_LEN`] signature bytes;
    /// `link_id` and the 48-bit timestamp are part of the signed message.
    ///
    /// Returns `None` when the stored length is inconsistent — shorter than
    /// the trailer, or beyond the backing buffer. Callers MUST treat `None`
    /// as a verification failure; going through this accessor is what keeps
    /// a hostile `raw_frame_len` from panicking on an out-of-bounds slice.
    pub fn signed_message(&self) -> Option<&[u8]> {
        let end = self
            .raw_frame_len
            .checked_sub(MAVLINK_SIGNATURE_TRAILER_LEN)?;
        self.raw_frame.get(..end)
    }
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

#[cfg(test)]
mod tests {
    use super::*;

    fn meta_with(raw_frame_len: usize) -> SignatureMeta {
        let mut raw_frame = [0u8; MAX_SIGNED_FRAME_SIZE];
        for (i, b) in raw_frame.iter_mut().enumerate() {
            *b = i as u8;
        }
        SignatureMeta {
            system_id: 1,
            component_id: 1,
            link_id: 5,
            timestamp: 1000,
            sig: [0xAA; 6],
            raw_frame,
            raw_frame_len,
        }
    }

    #[test]
    fn signed_message_excludes_exactly_the_trailer() {
        let meta = meta_with(32);
        // Canonical coverage is the frame minus the trailing 6 sig bytes;
        // `unwrap_or(&[])` keeps the test panic-free (the crate forbids it)
        // while a wrong result still fails the length assertion.
        let msg = meta.signed_message().unwrap_or(&[]);
        assert_eq!(msg.len(), 32 - MAVLINK_SIGNATURE_TRAILER_LEN);
        assert_eq!(msg, &meta.raw_frame[..26]);
        // The final covered byte is index 25 — the 6 signature bytes
        // (indices 26..32) are excluded.
        assert_eq!(msg.last().copied(), Some(25));
    }

    #[test]
    fn signed_message_none_when_shorter_than_trailer() {
        // Any length below the 6-byte trailer is malformed.
        assert!(meta_with(0).signed_message().is_none());
        assert!(meta_with(5).signed_message().is_none());
        // Exactly the trailer → empty coverage, still Some.
        assert_eq!(meta_with(6).signed_message().map(<[u8]>::len), Some(0));
    }

    #[test]
    fn signed_message_bounded_and_never_panics_past_buffer() {
        // A length whose coverage still fits the buffer is returned as a
        // bounded slice (no panic), even if it exceeds the declared max.
        let within = meta_with(MAX_SIGNED_FRAME_SIZE);
        assert_eq!(
            within.signed_message().map(<[u8]>::len),
            Some(MAX_SIGNED_FRAME_SIZE - MAVLINK_SIGNATURE_TRAILER_LEN)
        );

        // A hostile length whose coverage runs past the backing buffer must
        // yield None rather than panic on an out-of-bounds slice.
        assert!(
            meta_with(MAX_SIGNED_FRAME_SIZE + MAVLINK_SIGNATURE_TRAILER_LEN + 1)
                .signed_message()
                .is_none()
        );
        assert!(meta_with(usize::MAX).signed_message().is_none());
    }
}
