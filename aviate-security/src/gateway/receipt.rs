//! Provenance stamped onto a [`super::VerifiedSystemCommand`] the instant
//! the gateway admits it.
//!
//! The receipt is the trusted record of *where* a verified command came
//! from and *when the gateway received it* — never a value copied out of
//! an untrusted payload. Downstream liveness/recovery policy (the
//! independent RC / GCS / offboard timeouts) reads these fields; only a
//! newly admitted verified command carries a fresh `received_at_us`, so
//! telemetry, heartbeats, arm events, replay, and reconnection cannot
//! forge freshness.

/// Which external authority a command claims to come from.
///
/// These are the three independent liveness domains the failsafe policy
/// treats separately: an RC handset, a GCS / datalink, and an offboard
/// setpoint source. They are distinct link identities — GCS traffic must
/// never refresh RC or offboard freshness, and vice versa.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CommandSource {
    /// Manual RC / handset authority.
    Rc,
    /// Ground control station / datalink (mode, arm, mission).
    GcsDatalink,
    /// Offboard external setpoint source (companion / autonomy).
    Offboard,
}

/// Trusted provenance stamped by the gateway at admission time.
///
/// `Copy` and allocation-free: it is a fixed set of scalars, cheap to
/// carry alongside every verified command with no heap or clone cost.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct VerificationReceipt {
    source: CommandSource,
    authority_epoch: u32,
    sequence: u64,
    received_at_us: u64,
    expires_at_us: Option<u64>,
}

impl VerificationReceipt {
    /// Construct a receipt. `pub(crate)` on purpose: only the gateway,
    /// having actually verified a command, may stamp one — no external
    /// crate can fabricate provenance.
    pub(crate) fn new(
        source: CommandSource,
        authority_epoch: u32,
        sequence: u64,
        received_at_us: u64,
        expires_at_us: Option<u64>,
    ) -> Self {
        Self {
            source,
            authority_epoch,
            sequence,
            received_at_us,
            expires_at_us,
        }
    }

    /// Which authority the gateway verified this command against.
    pub fn source(&self) -> CommandSource {
        self.source
    }

    /// The authority epoch in force when the command was admitted.
    ///
    /// Bumped on recovery from a link loss: a command carrying a stale
    /// epoch is from before the recovery boundary and must not revive a
    /// lapsed authority (the fresh-epoch requirement lives in the
    /// liveness policy that consumes this).
    pub fn authority_epoch(&self) -> u32 {
        self.authority_epoch
    }

    /// The authenticated freshness counter that admitted this command: the
    /// signed frame's monotonic signature timestamp (`0` for an unsigned
    /// command under a development policy). It is taken from the verified
    /// signature, never from a payload-supplied claim.
    pub fn sequence(&self) -> u64 {
        self.sequence
    }

    /// Trusted receive time (µs, monotonic FC clock), stamped by the
    /// gateway at admission — never taken from the payload.
    pub fn received_at_us(&self) -> u64 {
        self.received_at_us
    }

    /// Optional hard expiry (µs, same clock). `None` means the command
    /// carries no intrinsic expiry and freshness is governed purely by
    /// the source's liveness policy.
    pub fn expires_at_us(&self) -> Option<u64> {
        self.expires_at_us
    }
}
