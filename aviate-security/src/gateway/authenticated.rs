//! The sealed, scheme-neutral authenticated-command claim.
//!
//! An [`AuthenticatedCommand`] is what an admission adapter produces once
//! it has cryptographically verified a frame: a [`Principal`], a monotonic
//! freshness `counter`, and the command decoded from the *authenticated*
//! bytes. It is the ONLY input the [`CommandGateway`](super::CommandGateway)
//! accepts. The gateway never sees a `SignatureMeta`, a MAVLink frame, or
//! any scheme-specific evidence — only this claim — so authorization,
//! anti-replay, and receipt-stamping are identical across every security
//! scheme.
//!
//! Like [`VerifiedSystemCommand`](super::VerifiedSystemCommand), the claim
//! is sealed: [`seal`] is `pub(crate)`, reachable only from admission
//! adapters inside this crate, and there are no public fields, no public
//! constructor, no `Default`, and no deserialization. External code cannot
//! fabricate a claim, so it cannot reach the gateway with an
//! unauthenticated command.
//!
//! [`seal`]: AuthenticatedCommand::seal

use aviate_hal_io::SystemCommand;

use crate::principal::Principal;

/// A command that an admission adapter has authenticated.
///
/// Possession of this value asserts only that *some* adapter verified the
/// command's cryptography and derived its principal and freshness counter
/// from the authenticated material. It is NOT yet authorized or checked for
/// replay — that is the gateway's job.
///
/// External code cannot construct one. Each of these must fail to compile:
///
/// ```compile_fail
/// // No public constructor / no struct literal (private fields).
/// use aviate_security::AuthenticatedCommand;
/// use aviate_security::Principal;
/// use aviate_hal_io::SystemCommand;
/// let _ = AuthenticatedCommand {
///     principal: Principal::mavlink(1, 1, 5),
///     counter: 1,
///     command: SystemCommand::Arm,
/// };
/// ```
///
/// ```compile_fail
/// // `seal` is pub(crate) — unreachable from another crate.
/// use aviate_security::AuthenticatedCommand;
/// use aviate_security::Principal;
/// use aviate_hal_io::SystemCommand;
/// let _ = AuthenticatedCommand::seal(Principal::mavlink(1, 1, 5), 1, SystemCommand::Arm);
/// ```
#[derive(Debug)]
pub struct AuthenticatedCommand {
    principal: Principal,
    counter: u64,
    command: SystemCommand,
}

impl AuthenticatedCommand {
    /// Seal an authenticated command. `pub(crate)`: only an admission
    /// adapter in this crate — after it has verified the command's
    /// cryptography — may mint one. This is the single choke point that
    /// turns a scheme-specific verified frame into a scheme-neutral claim.
    pub(crate) fn seal(principal: Principal, counter: u64, command: SystemCommand) -> Self {
        Self {
            principal,
            counter,
            command,
        }
    }

    /// The authenticated principal: which credential authenticated the
    /// command and what identity it asserts. The gateway authorizes and
    /// tracks freshness against this.
    pub fn principal(&self) -> Principal {
        self.principal
    }

    /// The authenticated monotonic freshness counter (MAVLink: the signing
    /// timestamp). The gateway checks it against the principal's anti-replay
    /// high-water mark.
    pub fn counter(&self) -> u64 {
        self.counter
    }

    /// Borrow the command. The gateway inspects it for diagnostics; the
    /// command is not trusted until the gateway mints a
    /// [`VerifiedSystemCommand`](super::VerifiedSystemCommand).
    pub fn command(&self) -> &SystemCommand {
        &self.command
    }

    /// Consume the claim and yield the command. `pub(super)`: only the
    /// gateway takes it out, at the moment it mints a verified command.
    pub(super) fn into_command(self) -> SystemCommand {
        self.command
    }
}
