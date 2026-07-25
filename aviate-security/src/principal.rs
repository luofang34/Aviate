//! Scheme-neutral principal identity for the command boundary.
//!
//! The command gateway authorizes commands and tracks their freshness
//! against a [`Principal`] without knowing which cryptographic scheme
//! authenticated them. Each admission adapter (MAVLink signing today; an
//! AEAD or CCSDS SDLS envelope later) maps its own credential material to
//! a `Principal`, and the gateway treats every scheme uniformly from there
//! on. See the crate docs for the pipeline.

/// The cryptographic scheme that authenticated a command.
///
/// The tag is part of a [`Principal`]'s identity, so a principal minted by
/// one adapter can never satisfy an authorization binding or advance a
/// freshness counter intended for another scheme.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum SecurityScheme {
    /// MAVLink 2 message signing (`sha256_48`, shared secret). The
    /// interoperability baseline — authenticity and replay resistance
    /// only. See [`crate::admission::MavlinkAdmission`].
    MavlinkSigning,
    // Reserved for the crypto-agile envelope tracked in SEC-CMD; adapters
    // are not yet implemented:
    //   AviateAead,   // AES-GCM / ChaCha20-Poly1305 production envelope
    //   CcsdsSdls,    // CCSDS 355.0-B-2 space-link security
}

/// A scheme-neutral principal: which credential authenticated a command,
/// and what identity that credential asserts.
///
/// The gateway authorizes and de-duplicates freshness against this value
/// alone; it never inspects a scheme's wire format. The two coordinates
/// are scheme-scoped — for [`SecurityScheme::MavlinkSigning`], `credential`
/// is the signing `link_id` (key slot) and `identity` packs the sender
/// `(system_id, component_id)`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Principal {
    scheme: SecurityScheme,
    credential: u16,
    identity: u16,
}

impl Principal {
    /// Build a MAVLink signing principal from a signed frame's credential
    /// (`link_id`) and the sender identity it asserts.
    pub const fn mavlink(system_id: u8, component_id: u8, link_id: u8) -> Self {
        Self {
            scheme: SecurityScheme::MavlinkSigning,
            credential: link_id as u16,
            identity: ((system_id as u16) << 8) | component_id as u16,
        }
    }

    /// The scheme that authenticated this principal.
    pub const fn scheme(&self) -> SecurityScheme {
        self.scheme
    }

    /// The credential (key slot / security association) that authenticated
    /// the command — scheme-scoped; MAVLink: `link_id`. The authorization
    /// policy holds at most one binding per `(scheme, credential)`, because
    /// a credential is a single secret and cannot speak for two authorities.
    pub const fn credential(&self) -> u16 {
        self.credential
    }
}
