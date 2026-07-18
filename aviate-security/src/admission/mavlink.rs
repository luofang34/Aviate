//! MAVLink 2 signing admission adapter.
//!
//! [`MavlinkAdmission`] is the scheme-specific edge for MAVLink message
//! signing. It decodes a frame's command and signature from the *same*
//! bytes (via [`parse_system_command`]), verifies the `sha256_48`
//! signature over the canonical coverage, and seals the result into a
//! scheme-neutral [`AuthenticatedCommand`]. Everything the gateway does
//! afterward — authorization, anti-replay, receipts — is scheme-agnostic.
//!
//! Because the command and the signature come from one buffer that the
//! frame must occupy exactly, a valid signature can never vouch for a
//! command it did not sign.

use aviate_hal_io::security::{CryptoEngine, KeyStore};
use aviate_link::mavlink::parse_system_command;

use crate::auth::SignedAuth;
use crate::errors::{AuthError, GatewayError, GatewayResult};
use crate::gateway::AuthenticatedCommand;
use crate::principal::Principal;

/// Turns signed MAVLink 2 frames into authenticated commands.
///
/// This is the MAVLink signing profile's adapter. Construct it with a key
/// store and crypto engine; feed it whole frames; hand the resulting
/// [`AuthenticatedCommand`] to [`CommandGateway::admit`](crate::CommandGateway::admit).
///
/// The signing profile admits ONLY signed frames — an unsigned frame is
/// rejected ([`AuthError::MissingSignature`]). There is no automatic
/// unsigned fallback (that is the separate, feature-gated
/// `InsecureDevAdmission`).
pub struct MavlinkAdmission<K: KeyStore, C: CryptoEngine> {
    auth: SignedAuth<K, C>,
}

impl<K: KeyStore, C: CryptoEngine> MavlinkAdmission<K, C> {
    /// Build the adapter over a key store and crypto engine.
    pub fn new(keystore: K, crypto: C) -> Self {
        Self {
            auth: SignedAuth::new(keystore, crypto),
        }
    }

    /// Decode and authenticate one MAVLink 2 frame.
    ///
    /// Steps: decode `(command, signature)` from the frame bytes; require a
    /// signature; verify it over the canonical coverage; derive the
    /// principal from the *authenticated* header identity and key slot; seal
    /// the command with the signing timestamp as its freshness counter.
    ///
    /// Returns:
    /// - [`GatewayError::Link`] if the frame does not decode to a system
    ///   command (parse error, unmapped message, trailing bytes);
    /// - [`GatewayError::Auth`] with [`AuthError::MissingSignature`] for an
    ///   unsigned frame, or [`AuthError::InvalidSignature`] for a bad
    ///   signature;
    /// - a sealed [`AuthenticatedCommand`] on success. It is not yet
    ///   authorized or replay-checked — the gateway does that.
    pub fn authenticate(&mut self, frame: &[u8]) -> GatewayResult<AuthenticatedCommand> {
        let parsed = parse_system_command(frame).map_err(GatewayError::Link)?;
        let sig = parsed
            .signature
            .as_ref()
            .ok_or(AuthError::MissingSignature)?;

        self.auth.verify_frame(sig)?;

        let principal = Principal::mavlink(sig.system_id, sig.component_id, sig.link_id);
        let counter = sig.timestamp;
        Ok(AuthenticatedCommand::seal(
            principal,
            counter,
            parsed.command,
        ))
    }
}
