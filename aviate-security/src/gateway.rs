//! Command gateway — the scheme-neutral place an authenticated command
//! becomes trusted.
//!
//! ## The type boundary
//!
//! ```text
//! transport bytes
//!   → admission adapter        (scheme-specific: decode + verify crypto)
//!   → AuthenticatedCommand      (sealed: principal + counter + command)
//!   → CommandGateway::admit      (authorize principal + anti-replay + stamp)
//!   → VerifiedSystemCommand      (sealed: only admit() can mint one)
//!   → CommandIngress<Verified…>  (freshness; proof kept)
//!   → narrow runtime dispatch    (proof erased here, once)
//!   → SystemCommand → kernel     (security-agnostic)
//! ```
//!
//! The gateway is **scheme-neutral**: it consumes a sealed
//! [`AuthenticatedCommand`] and knows nothing about MAVLink, signatures, or
//! any wire format. Cryptographic verification and transport decoding live
//! in an admission adapter ([`crate::admission`]); everything downstream of
//! the claim — authorization, freshness, receipts, the runtime — is
//! identical for every security scheme. Adding an AEAD or CCSDS profile
//! (see issue SEC-CMD) means adding an adapter, not changing this file.
//!
//! [`VerifiedSystemCommand`] has no public constructor: it is minted only
//! by [`CommandGateway::admit`]. There is no way for application or
//! transport code to fabricate a verified command.
//!
//! ## Admission order: verify → authorize → commit
//!
//! 1. **Verify** (in the adapter, before the claim exists): the command's
//!    cryptography is checked and its principal derived from the
//!    authenticated material. No gateway state changes.
//! 2. **Authorize** the principal against the [`SourcePolicy`]. A validly
//!    authenticated command from an unbound principal is rejected here —
//!    and, because anti-replay has not run yet, it cannot consume a
//!    replay-table slot.
//! 3. **Commit** the per-principal anti-replay counter. Only a command that
//!    is both authentic and authorized may advance replay state.
//!
//! Failsafe commands the FC generates itself use the separate
//! [`TrustedInternalCommand`] — trusted, but never mistakable for an
//! externally verified one.

mod authenticated;
mod internal;
mod receipt;
mod source_policy;
mod verified;

pub use authenticated::AuthenticatedCommand;
pub use internal::{FailsafeAuthority, TrustedInternalCommand};
pub use receipt::{CommandSource, VerificationReceipt};
pub use source_policy::{CredentialError, SourcePolicy, MAX_SOURCE_BINDINGS};
pub use verified::VerifiedSystemCommand;

use crate::anti_replay::AntiReplayWindow;
use crate::errors::{AuthError, GatewayResult};

/// Freshness configuration for a gateway's anti-replay window.
///
/// Both fields are scheme-specific and supplied by the profile that builds
/// the gateway. For the MAVLink signing profile, the units are 10 µs
/// signing-timestamp ticks and `new_stream_max_age` is
/// [`NEW_STREAM_MAX_AGE_10US`](crate::anti_replay::NEW_STREAM_MAX_AGE_10US).
#[derive(Debug, Clone, Copy)]
pub struct FreshnessConfig {
    /// Trusted local counter seeded from a persisted high-water mark or an
    /// RTC — a value an attacker cannot rewind. `0` disables first-frame
    /// freshness (bench only).
    pub initial_trusted_counter: u64,
    /// How far behind the trusted local counter the first frame of a new
    /// principal's stream may be.
    pub new_stream_max_age: u64,
}

/// Turns authenticated commands into verified ones, or rejects them.
///
/// Owns the principal→source authorization policy, the per-principal
/// anti-replay window, and the current authority epoch. It does NOT own a
/// transport or a cryptographic verifier: an admission adapter produces an
/// [`AuthenticatedCommand`] and the runner hands it to [`Self::admit`].
pub struct CommandGateway {
    source_policy: SourcePolicy,
    anti_replay: AntiReplayWindow,
    authority_epoch: u32,
}

impl CommandGateway {
    /// Create a gateway with the given authorization policy and freshness
    /// configuration, at authority epoch 0.
    pub fn new(source_policy: SourcePolicy, freshness: FreshnessConfig) -> Self {
        Self {
            source_policy,
            anti_replay: AntiReplayWindow::new(
                freshness.initial_trusted_counter,
                freshness.new_stream_max_age,
            ),
            authority_epoch: 0,
        }
    }

    /// Admit an authenticated command: authorize its principal, commit
    /// anti-replay, and mint a [`VerifiedSystemCommand`].
    ///
    /// `now_us` is the trusted monotonic FC time at ingress; it becomes the
    /// receipt's `received_at_us`. Nothing from the payload is used as a
    /// timestamp, a source, or a sequence — the source comes from the
    /// authorization policy and the freshness counter from the
    /// authenticated claim. A rejected command returns an error and mints
    /// nothing.
    pub fn admit(
        &mut self,
        command: AuthenticatedCommand,
        now_us: u64,
    ) -> GatewayResult<VerifiedSystemCommand> {
        let principal = command.principal();

        // 1. Authorize the authenticated principal. Rejecting here, before
        //    anti-replay, is what stops an authenticated-but-unauthorized
        //    principal from consuming a bounded replay slot.
        let source = self
            .source_policy
            .resolve(principal)
            .ok_or(AuthError::UnauthorizedSource)?;

        // 2. Commit anti-replay on the (principal, counter). Only an
        //    authentic AND authorized command may advance a counter.
        self.anti_replay
            .check_and_update(principal, command.counter())?;

        // 3. Stamp trusted provenance and mint.
        let receipt = VerificationReceipt::new(
            source,
            self.authority_epoch,
            command.counter(),
            now_us,
            None,
        );
        Ok(VerifiedSystemCommand::mint(command.into_command(), receipt))
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

    /// Read-only access to the authorization policy, for telemetry.
    pub fn source_policy(&self) -> &SourcePolicy {
        &self.source_policy
    }

    /// The anti-replay window's trusted local counter, exposed so the
    /// assembly can persist it across reboots (reboot-replay defense).
    pub fn local_freshness_counter(&self) -> u64 {
        self.anti_replay.local_counter()
    }
}

#[cfg(test)]
#[path = "gateway/tests.rs"]
mod tests;
