//! Command gateway — the ONLY place an external command becomes trusted.
//!
//! ## The type boundary
//!
//! ```text
//! transport bytes
//!   → UnverifiedSystemCommand      (public: any transport may build one)
//!   → CommandGateway::admit         (authenticate + source-bind + stamp)
//!   → VerifiedSystemCommand         (sealed: only admit() can mint one)
//!   → CommandIngress<Verified…>     (freshness; proof kept)
//!   → narrow runtime dispatch       (proof erased here, once)
//!   → SystemCommand → kernel        (security-agnostic)
//! ```
//!
//! [`VerifiedSystemCommand`] has no public constructor: it is minted only
//! by [`CommandGateway::admit`], which runs the configured
//! [`CommandAuth`] and then binds a trusted source and freshness from the
//! *authenticated* signature before stamping a [`VerificationReceipt`].
//! There is no way for application or transport code to fabricate a
//! verified command, so a "parse bytes → feed the kernel" bypass does not
//! type-check.
//!
//! ## Authority is bound to the credential, not the payload
//!
//! `admit` never reads a source or a sequence from the untrusted command.
//! The source comes from the gateway's [`SourcePolicy`], keyed on the
//! *verified* `(system_id, component_id, link_id)` identity; the freshness
//! counter is the verified signature's monotonic timestamp. A validly
//! signed frame from an unexpected peer maps to no source and is rejected,
//! and anti-replay (in [`CommandAuth`]) is committed only after the
//! signature verifies.
//!
//! Failsafe commands the FC generates itself use the separate
//! [`TrustedInternalCommand`] — trusted, but never mistakable for an
//! externally verified one.

mod receipt;
mod source_policy;
mod unverified;
mod verified;

pub use receipt::{CommandSource, VerificationReceipt};
pub use source_policy::{SourcePolicy, MAX_SOURCE_BINDINGS};
pub use unverified::{FailsafeAuthority, TrustedInternalCommand, UnverifiedSystemCommand};
pub use verified::VerifiedSystemCommand;

use crate::auth::CommandAuth;
use crate::errors::{AuthError, GatewayResult};

/// Turns untrusted commands into verified ones, or rejects them.
///
/// Owns the authentication policy, the credential→source binding, and the
/// current authority epoch. It does NOT own a transport: the runner polls
/// the transport for an [`UnverifiedSystemCommand`] and hands it to
/// [`Self::admit`], so there is no `link_mut()` through which raw parsed
/// commands could escape verification.
pub struct CommandGateway<A> {
    auth: A,
    source_policy: SourcePolicy,
    authority_epoch: u32,
}

impl<A: CommandAuth> CommandGateway<A> {
    /// Create a gateway with the given authentication policy and
    /// credential→source binding, at authority epoch 0.
    pub fn new(auth: A, source_policy: SourcePolicy) -> Self {
        Self {
            auth,
            source_policy,
            authority_epoch: 0,
        }
    }

    /// Admit an unverified command: authenticate it, bind a trusted source
    /// and freshness from the *authenticated* signature, and mint a
    /// [`VerifiedSystemCommand`].
    ///
    /// `now_us` is the trusted monotonic FC time at ingress; it becomes the
    /// receipt's `received_at_us`. Nothing from the payload is used as a
    /// timestamp, a source, or a sequence. A rejected command returns an
    /// error and mints nothing — there is no partial/unchecked path to a
    /// verified value.
    pub fn admit(
        &mut self,
        unverified: UnverifiedSystemCommand,
        now_us: u64,
    ) -> GatewayResult<VerifiedSystemCommand> {
        // 1. Authenticity. For signed auth this verifies the HMAC over the
        //    canonical coverage and, only on success, commits the
        //    per-identity anti-replay counter (see `CommandAuth`).
        let signature = unverified.signature();
        self.auth.authenticate(signature)?;

        // 2. Bind source + freshness from the AUTHENTICATED signature,
        //    never from the payload. A signed frame's authority is its
        //    credential identity; an unsigned frame is only admissible
        //    under a development policy that names an unsigned source.
        let (source, sequence) = match signature {
            Some(sig) => {
                let source = self
                    .source_policy
                    .resolve(sig.system_id, sig.component_id, sig.link_id)
                    .ok_or(AuthError::UnauthorizedSource)?;
                (source, sig.timestamp)
            }
            None => {
                let source = self
                    .source_policy
                    .unsigned_source()
                    .ok_or(AuthError::MissingSignature)?;
                (source, 0)
            }
        };

        // 3. Stamp trusted provenance and mint.
        let receipt =
            VerificationReceipt::new(source, self.authority_epoch, sequence, now_us, None);
        Ok(VerifiedSystemCommand::mint(
            unverified.into_command(),
            receipt,
        ))
    }

    /// Advance the authority epoch — called on recovery from a link loss.
    ///
    /// After a source's authority lapses, commands admitted at the new
    /// epoch are distinguishable from any that predate the recovery
    /// boundary, so a stale command cannot silently revive a dead
    /// authority. Configuration surface only; it grants no way to forge a
    /// command.
    pub fn begin_authority_epoch(&mut self) {
        self.authority_epoch = self.authority_epoch.wrapping_add(1);
    }

    /// Read-only diagnostic: the current authority epoch.
    pub fn authority_epoch(&self) -> u32 {
        self.authority_epoch
    }

    /// Read-only access to the credential→source binding, for telemetry.
    pub fn source_policy(&self) -> &SourcePolicy {
        &self.source_policy
    }
}

#[cfg(test)]
#[path = "gateway/tests.rs"]
mod tests;
