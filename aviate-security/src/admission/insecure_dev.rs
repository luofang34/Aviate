//! Insecure development admission adapter (SITL / bench only).
//!
//! [`InsecureDevAdmission`] decodes a frame's command but performs NO
//! cryptographic verification. It exists so software-in-the-loop and bench
//! harnesses can drive the real gateway without provisioning keys. It is
//! compiled ONLY under the non-default `insecure-dev-auth` feature (or this
//! crate's tests), so a flight assembly that never enables that feature
//! cannot name an authentication bypass.
//!
//! ## Security Warning
//!
//! This admits unauthenticated traffic as a fixed principal. NEVER include
//! it in a flight build.

use aviate_hal_io::SystemCommand;
use aviate_link::mavlink::parse_system_command;

use crate::errors::{GatewayError, GatewayResult};
use crate::gateway::AuthenticatedCommand;
use crate::principal::Principal;

/// Admits unsigned frames as a fixed development principal.
///
/// It supplies its own strictly-monotonic freshness counter (no wire
/// timestamp is trusted), so the gateway's anti-replay still functions in
/// simulation. Authorize the configured principal in the dev
/// [`SourcePolicy`](crate::SourcePolicy) exactly as you would a real one.
pub struct InsecureDevAdmission {
    principal: Principal,
    next_counter: u64,
}

impl InsecureDevAdmission {
    /// Build a dev adapter that seals every command under `principal`.
    ///
    /// The first command it produces carries counter `1`.
    pub fn new(principal: Principal) -> Self {
        Self {
            principal,
            next_counter: 1,
        }
    }

    /// Decode a frame's command (ignoring any signature) and seal it under
    /// the fixed dev principal with the next monotonic counter.
    pub fn authenticate(&mut self, frame: &[u8]) -> GatewayResult<AuthenticatedCommand> {
        let parsed = parse_system_command(frame).map_err(GatewayError::Link)?;
        Ok(self.seal(parsed.command))
    }

    /// Seal an already-decoded command (for harnesses that build
    /// [`SystemCommand`]s directly rather than from wire bytes).
    pub fn seal_command(&mut self, command: SystemCommand) -> AuthenticatedCommand {
        self.seal(command)
    }

    fn seal(&mut self, command: SystemCommand) -> AuthenticatedCommand {
        let counter = self.next_counter;
        // Monotonic counters must wrap explicitly (debug builds panic on
        // overflow); a dev session will never reach u64::MAX in practice.
        self.next_counter = self.next_counter.wrapping_add(1);
        AuthenticatedCommand::seal(self.principal, counter, command)
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::panic)]
mod tests {
    use super::*;
    use crate::{CommandGateway, CommandSource, FreshnessConfig, GatewayError, SourcePolicy};

    fn dev_gateway(principal: Principal) -> CommandGateway {
        let mut policy = SourcePolicy::new();
        policy.bind(principal, CommandSource::Offboard).unwrap();
        CommandGateway::new(
            policy,
            FreshnessConfig {
                initial_trusted_counter: 0,
                new_stream_max_age: 0,
            },
        )
    }

    #[test]
    fn dev_claims_admit_and_advance_monotonically() {
        let principal = Principal::mavlink(0, 0, 0);
        let mut adm = InsecureDevAdmission::new(principal);
        let mut gw = dev_gateway(principal);

        let first = adm.seal_command(SystemCommand::Arm);
        assert_eq!(first.counter(), 1);
        assert!(gw.admit(first, 10).is_ok());

        // The adapter's own counter advances, so the next claim is fresh.
        let second = adm.seal_command(SystemCommand::Disarm);
        assert_eq!(second.counter(), 2);
        assert!(gw.admit(second, 20).is_ok());
    }

    #[test]
    fn dev_principal_must_still_be_authorized() {
        let mut adm = InsecureDevAdmission::new(Principal::mavlink(7, 7, 7));
        // Gateway authorizes a DIFFERENT principal.
        let mut gw = dev_gateway(Principal::mavlink(0, 0, 0));
        let claim = adm.seal_command(SystemCommand::Arm);
        assert!(matches!(
            gw.admit(claim, 10),
            Err(GatewayError::Auth(crate::AuthError::UnauthorizedSource))
        ));
    }
}
