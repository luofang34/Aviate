//! The sealed proof-of-verification type.
//!
//! A [`VerifiedSystemCommand`] can only be minted by [`mint`], which is
//! `pub(super)` — reachable from the parent `gateway` module (the only
//! place that actually verifies commands) and nowhere else. External
//! crates and even sibling `aviate-security` modules cannot construct
//! one. There is deliberately:
//!
//! * no public constructor and no public fields;
//! * no `From<SystemCommand>` / `Default` / deserialization;
//! * no unchecked constructor outside `#[cfg(test)]`;
//! * no mutable access to the wrapped command.
//!
//! The wrapper is move-only (it holds a non-`Copy` [`SystemCommand`]),
//! so a verified command cannot be silently duplicated: an admitted
//! command is consumed exactly once on its way into the flight cycle.
//!
//! [`mint`]: VerifiedSystemCommand::mint

use aviate_hal_io::SystemCommand;

use super::receipt::VerificationReceipt;

/// A command that has passed the gateway's authentication, anti-replay,
/// and freshness checks. Possession of this value IS the proof it was
/// verified — the flight cycle accepts nothing else from an external
/// source.
///
/// External code cannot construct one. Each of these must fail to
/// compile, proving the boundary holds:
///
/// ```compile_fail
/// // No public constructor / no struct-literal (private fields).
/// use aviate_security::VerifiedSystemCommand;
/// use aviate_hal_io::SystemCommand;
/// let _ = VerifiedSystemCommand { command: SystemCommand::Arm, receipt: todo!() };
/// ```
///
/// ```compile_fail
/// // `mint` is pub(super) — unreachable from another crate.
/// use aviate_security::VerifiedSystemCommand;
/// use aviate_hal_io::SystemCommand;
/// let _ = VerifiedSystemCommand::mint(SystemCommand::Arm, todo!());
/// ```
///
/// ```compile_fail
/// // No Default.
/// let _ = <aviate_security::VerifiedSystemCommand as Default>::default();
/// ```
///
/// ```compile_fail
/// // No From<SystemCommand>.
/// use aviate_security::VerifiedSystemCommand;
/// use aviate_hal_io::SystemCommand;
/// let _: VerifiedSystemCommand = SystemCommand::Arm.into();
/// ```
#[derive(Debug)]
pub struct VerifiedSystemCommand {
    command: SystemCommand,
    receipt: VerificationReceipt,
}

impl VerifiedSystemCommand {
    /// Mint a verified command. `pub(super)`: only the parent `gateway`
    /// module — after it has verified `command` — may call this. This is
    /// the single choke point that turns an untrusted command into a
    /// trusted one.
    pub(super) fn mint(command: SystemCommand, receipt: VerificationReceipt) -> Self {
        Self { command, receipt }
    }

    /// Borrow the verified command. Used by the narrow runtime dispatch
    /// that feeds the security-agnostic kernel; the borrow keeps the
    /// proof intact while the retained setpoint is replayed each cycle.
    pub fn command(&self) -> &SystemCommand {
        &self.command
    }

    /// The trusted provenance stamped at admission (source, epoch,
    /// sequence, receive time, expiry).
    pub fn receipt(&self) -> &VerificationReceipt {
        &self.receipt
    }

    /// Consume the proof and yield the bare command — the single,
    /// deliberate point at which verification is "erased" immediately
    /// before the security-agnostic kernel is called. Taking `self` by
    /// value (move-only) means the proof cannot be reused afterwards.
    pub fn into_command(self) -> SystemCommand {
        self.command
    }
}
