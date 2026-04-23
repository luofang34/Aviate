//! Command gateway - unified entry point for all external commands
//!
//! This module implements the `CommandGateway`, which is the ONLY way external
//! commands should enter the flight control system.
//!
//! ## DO-178C Security Architecture
//!
//! ```text
//! ┌─────────────────────────────────────────────────────────────┐
//! │  ⚠️  CRITICAL: ALL external commands MUST use this gateway  │
//! │                                                              │
//! │  CORRECT usage:                                              │
//! │  let link = MavlinkCommandLink::new(usb_rx);                │
//! │  let auth = SignedAuth::new(keystore, crypto);              │
//! │  let mut gateway = CommandGateway::new(link, auth);   // ✅  │
//! │  if let Ok(Some(cmd)) = gateway.poll_command(now_ms) {      │
//! │      kernel.execute(cmd);  // Safe: verified                │
//! │  }                                                           │
//! │                                                              │
//! │  WRONG usage (PROHIBITED):                                  │
//! │  let mut link = MavlinkCommandLink::new(usb_rx);            │
//! │  if let Ok(Some(cmd)) = link.poll_command(now_ms) {         │
//! │      kernel.execute(cmd);  // ❌ BYPASSES SECURITY!          │
//! │  }                                                           │
//! └─────────────────────────────────────────────────────────────┘
//! ```
//!
//! ## Audit Checklist
//!
//! When auditing applications using this crate, verify:
//! - ✅ CommandGateway is the ONLY command source for kernel.execute()
//! - ✅ No direct calls to CommandLink::poll_command() in application code
//! - ✅ No bypass paths exist (grep for "execute.*Command" and trace back)
//!
//! ## Criticality
//!
//! - **DAL A/B**: Flight-critical security enforcement
//! - Commands that bypass this gateway are UNTRUSTED and UNSAFE

use aviate_link::command::{Command, CommandLink};

use crate::auth::CommandAuth;
use crate::errors::GatewayResult;

/// Command gateway - unified entry point for external commands
///
/// This struct combines:
/// 1. Protocol parsing (CommandLink)
/// 2. Security verification (CommandAuth)
///
/// It ensures that ALL external commands are properly verified before
/// reaching the application layer.
///
/// ## Type Parameters
///
/// - `L`: CommandLink implementation (e.g., `MavlinkCommandLink<UsbRx>`)
/// - `A`: CommandAuth implementation (e.g., `SignedAuth<KeyStore, CryptoEngine>`)
///
/// ## DO-178C Contract
///
/// - **poll_command()**: Non-blocking, deterministic, time-bounded
/// - **Security guarantee**: If poll_command() returns Ok(Some(cmd)),
///   then cmd has been verified by the CommandAuth implementation
///
/// ## Usage Example
///
/// ```ignore
/// use aviate_link::mavlink::MavlinkCommandLink;
/// use aviate_security::{CommandGateway, SignedAuth};
/// use aviate_hal_stm32h7::{Stm32h7KeyStore, Stm32h7CryptoEngine};
///
/// // Hardware layer
/// let keystore = Stm32h7KeyStore::new();
/// let crypto = Stm32h7CryptoEngine::new();
///
/// // Link layer (protocol parsing)
/// let link = MavlinkCommandLink::new(usb_rx);
///
/// // Security layer (verification)
/// let auth = SignedAuth::new(keystore, crypto);
/// let mut gateway = CommandGateway::new(link, auth);
///
/// // Application layer
/// loop {
///     if let Ok(Some(cmd)) = gateway.poll_command(now_ms) {
///         // cmd is verified! Safe to execute
///         kernel.execute(cmd);
///     }
/// }
/// ```
pub struct CommandGateway<L, A> {
    /// Protocol-level command link (parsing)
    link: L,

    /// Authentication and verification
    auth: A,
}

impl<L: CommandLink, A: CommandAuth> CommandGateway<L, A> {
    /// Create new command gateway
    ///
    /// ## Parameters
    ///
    /// - `link`: Protocol parsing layer (e.g., MavlinkCommandLink)
    /// - `auth`: Authentication layer (e.g., SignedAuth or PlainAuth)
    ///
    /// ## Returns
    ///
    /// CommandGateway instance ready to poll for verified commands
    pub fn new(link: L, auth: A) -> Self {
        Self { link, auth }
    }

    /// Poll for a verified command
    ///
    /// This is the ONLY way applications should receive external commands.
    ///
    /// ## Parameters
    ///
    /// - `now_ms`: Current system time (milliseconds since boot)
    ///
    /// ## Returns
    ///
    /// - `Ok(None)`: No command available (not an error, just no data)
    /// - `Ok(Some(cmd))`: Verified command ready to execute
    /// - `Err(GatewayError)`: Error in transport, parsing, or verification
    ///
    /// ## Processing Pipeline
    ///
    /// 1. **Link layer**: Parse protocol bytes → Command struct
    ///    - Transport receive (FrameRx::try_recv)
    ///    - Protocol parsing (MAVLink CRC, message decode)
    ///    - Domain mapping (MAVLink → Command)
    ///
    /// 2. **Security layer**: Verify command authenticity
    ///    - Anti-replay check (per-link_id monotonic counter)
    ///    - Signature verification (HMAC-SHA256)
    ///    - Key lookup (KeyStore)
    ///
    /// 3. **Return**: Only verified commands reach application
    ///
    /// ## DO-178C Contract
    ///
    /// - **Non-blocking**: Returns immediately, never waits
    /// - **Time complexity**: O(1) for each layer, bounded by frame size
    /// - **WCET target**: ~20 μs for max frame (parse + verify)
    /// - **Security guarantee**: If Ok(Some(cmd)) returned, cmd is verified
    ///
    /// ## Error Handling
    ///
    /// Applications should log errors but continue operation:
    ///
    /// ```ignore
    /// match gateway.poll_command(now_ms) {
    ///     Ok(Some(cmd)) => kernel.execute(cmd),
    ///     Ok(None) => { /* No command, continue */ },
    ///     Err(GatewayError::Link(e)) => {
    ///         log_error("Link error: {:?}", e);
    ///         // Continue - may be transient transport issue
    ///     },
    ///     Err(GatewayError::Auth(e)) => {
    ///         log_security_alert("Auth failed: {:?}", e);
    ///         // Alert operator - possible attack
    ///     },
    ///     Err(GatewayError::NoCommand) => { /* Shouldn't happen */ },
    /// }
    /// ```
    pub fn poll_command(&mut self, now_ms: u32) -> GatewayResult<Option<Command>> {
        // Step 1: Parse protocol bytes → Command
        match self.link.poll_command(now_ms)? {
            None => {
                // No command available (not an error)
                Ok(None)
            }
            Some(cmd) => {
                // Step 2: Verify command authenticity
                self.auth.verify(&cmd)?;

                // Step 3: Return verified command
                Ok(Some(cmd))
            }
        }
    }

    /// Get mutable reference to link layer (for configuration)
    ///
    /// This allows applications to configure the underlying transport
    /// (e.g., enable/disable protocol features) without breaking the
    /// security abstraction.
    pub fn link_mut(&mut self) -> &mut L {
        &mut self.link
    }

    /// Get mutable reference to auth layer (for diagnostics)
    ///
    /// This allows applications to query security state (e.g., anti-replay
    /// counters for telemetry) without breaking the security abstraction.
    pub fn auth_mut(&mut self) -> &mut A {
        &mut self.auth
    }
}

#[cfg(test)]
mod tests {
    extern crate alloc;
    use super::*;
    use crate::auth::PlainAuth;
    use alloc::vec;
    use alloc::vec::Vec;
    use aviate_link::command::{CommandKind, SignatureMeta};
    use aviate_link::errors::LinkResult;

    /// Mock command link for testing
    struct MockLink {
        commands: Vec<Command>,
    }

    impl MockLink {
        fn new(commands: Vec<Command>) -> Self {
            Self { commands }
        }
    }

    impl CommandLink for MockLink {
        fn poll_command(&mut self, _now_ms: u32) -> LinkResult<Option<Command>> {
            if self.commands.is_empty() {
                Ok(None)
            } else {
                Ok(Some(self.commands.remove(0)))
            }
        }
    }

    #[test]
    fn test_gateway_no_command() {
        let link = MockLink::new(vec![]);
        let auth = PlainAuth::new();
        let mut gateway = CommandGateway::new(link, auth);

        let result = gateway.poll_command(1000);
        assert!(result.is_ok());
        assert!(result.unwrap().is_none());
    }

    #[test]
    fn test_gateway_unsigned_command_with_plain_auth() {
        let cmd = Command {
            kind: CommandKind::Arm,
            params: [0.0; 7],
            timestamp_ms: 1000,
            signature: None,
        };
        let link = MockLink::new(vec![cmd]);
        let auth = PlainAuth::new();
        let mut gateway = CommandGateway::new(link, auth);

        let result = gateway.poll_command(1000);
        assert!(result.is_ok());
        let verified_cmd = result.unwrap();
        assert!(verified_cmd.is_some());
    }

    #[test]
    fn test_gateway_signed_command_with_plain_auth() {
        let cmd = Command {
            kind: CommandKind::Arm,
            params: [0.0; 7],
            timestamp_ms: 1000,
            signature: Some(SignatureMeta {
                link_id: 5,
                timestamp: 1000,
                sig: [0xAA; 6],
                raw_frame: vec![0u8; 32],
            }),
        };
        let link = MockLink::new(vec![cmd]);
        let auth = PlainAuth::new();
        let mut gateway = CommandGateway::new(link, auth);

        let result = gateway.poll_command(1000);
        assert!(result.is_ok());
        let verified_cmd = result.unwrap();
        assert!(verified_cmd.is_some());
    }

    // Note: Tests with SignedAuth require mock or real KeyStore/CryptoEngine
    // implementations. These will be added in integration tests.
}
