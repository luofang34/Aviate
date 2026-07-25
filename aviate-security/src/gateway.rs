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

/// Where a gateway's trusted starting counter came from.
///
/// A bare integer let `0` — which disables first-frame freshness, because
/// `0.saturating_sub(max_age)` is `0` and nothing can be below it — be
/// passed by an assembly that simply had not wired persistence yet. Every
/// test would still pass and the reboot-replay defense would be gone with
/// nothing to grep for. Naming the insecure case makes it visible.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TrustedCounter {
    /// A high-water mark persisted by the previous boot, or an RTC reading
    /// converted to the scheme's counter units.
    Trusted(u64),
    /// No trusted time source. First-frame freshness is DISABLED: any
    /// previously captured frame from an unseen principal is replayable
    /// after a restart. Isolated benches only.
    NoTrustedTimeSource,
}

impl TrustedCounter {
    /// The counter value to seed the window with.
    pub fn value(self) -> u64 {
        match self {
            Self::Trusted(v) => v,
            Self::NoTrustedTimeSource => 0,
        }
    }
}

/// Freshness configuration for a gateway's anti-replay window.
///
/// Both fields are scheme-specific and supplied by the profile that builds
/// the gateway. For the MAVLink signing profile, the units are 10 µs
/// signing-timestamp ticks and `new_stream_max_age` is
/// [`NEW_STREAM_MAX_AGE_10US`](crate::anti_replay::NEW_STREAM_MAX_AGE_10US).
#[derive(Debug, Clone, Copy)]
pub struct FreshnessConfig {
    /// Trusted local counter seeded from a persisted high-water mark or an
    /// RTC — a value an attacker cannot rewind.
    ///
    /// Constructed through [`TrustedCounter`], so a build that has no
    /// trusted time source has to say so in a form that greps, rather than
    /// silently passing a bare `0` and losing first-frame freshness.
    pub initial_trusted_counter: TrustedCounter,
    /// How far behind the trusted local counter the first frame of a new
    /// principal's stream may be.
    pub new_stream_max_age: u64,
    /// Counter units per microsecond of local FC time, as a divisor: a
    /// counter tick is `counter_tick_us` microseconds. MAVLink signing
    /// timestamps tick every 10 µs.
    ///
    /// This converts the receiver's own elapsed time into counter units so
    /// the trusted counter can be bounded by something no peer controls.
    pub counter_tick_us: u64,
    /// Clock disagreement tolerated between the receiver and a legitimate
    /// sender, in counter units.
    pub max_skew: u64,
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
    freshness: FreshnessConfig,
    /// Local time at the first admission, against which elapsed time — and
    /// therefore the plausible counter ceiling — is measured.
    epoch_now_us: Option<u64>,
}

impl CommandGateway {
    /// Create a gateway with the given authorization policy and freshness
    /// configuration, at authority epoch 0.
    pub fn new(source_policy: SourcePolicy, freshness: FreshnessConfig) -> Self {
        Self {
            source_policy,
            anti_replay: AntiReplayWindow::new(
                freshness.initial_trusted_counter.value(),
                freshness.new_stream_max_age,
            ),
            authority_epoch: 0,
            freshness,
            epoch_now_us: None,
        }
    }

    /// The highest freshness counter local elapsed time can justify.
    ///
    /// Anchored at the first admission rather than at construction, so a
    /// gateway built long before the first command does not hand out a
    /// ceiling inflated by idle time. Saturating throughout: a monotonic
    /// clock cannot go backwards, but a wrapped or bogus `now_us` must
    /// tighten the bound, never widen it.
    fn plausible_ceiling(&mut self, now_us: u64) -> u64 {
        let anchor = *self.epoch_now_us.get_or_insert(now_us);
        let elapsed_us = now_us.saturating_sub(anchor);
        let elapsed_counts = elapsed_us / self.freshness.counter_tick_us.max(1);
        self.freshness
            .initial_trusted_counter
            .value()
            .saturating_add(elapsed_counts)
            .saturating_add(self.freshness.max_skew)
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
        let ceiling = self.plausible_ceiling(now_us);
        self.anti_replay
            .check_and_update(principal, command.counter(), ceiling)?;

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
