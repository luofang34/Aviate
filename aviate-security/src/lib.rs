#![deny(missing_docs)]
//! Security policy layer for Aviate flight control system
//!
//! This crate provides command authentication, authorization, and
//! anti-replay protection for external commands, behind a **scheme-neutral
//! admission boundary**: MAVLink 2 signing is one interoperable adapter,
//! not the permanent architecture (see issue SEC-CMD).
//!
//! ## The command pipeline
//!
//! ```text
//! transport bytes
//!   → admission adapter        (scheme-specific: decode + verify crypto)
//!   → AuthenticatedCommand      (sealed: principal + counter + command)
//!   → CommandGateway::admit      (authorize principal + anti-replay + stamp)
//!   → VerifiedSystemCommand      (sealed proof)
//!   → runtime / kernel           (security-agnostic)
//! ```
//!
//! Cryptographic verification and transport decoding live in an
//! [`admission`] adapter. Everything downstream of
//! [`AuthenticatedCommand`] — authorization ([`SourcePolicy`]), freshness
//! ([`AntiReplayWindow`]), receipts, the runtime — is identical for every
//! scheme. Adding an AEAD or CCSDS profile means adding an adapter, not
//! changing the gateway.
//!
//! ## Usage Example
//!
//! ```ignore
//! use aviate_security::{CommandGateway, FreshnessConfig, Principal, CommandSource, SourcePolicy};
//! use aviate_security::admission::MavlinkAdmission;
//! use aviate_security::NEW_STREAM_MAX_AGE_10US;
//! use aviate_hal_stm32h7::{Stm32h7KeyStore, Stm32h7CryptoEngine};
//!
//! // MAVLink signing adapter (scheme-specific edge).
//! let mut admission = MavlinkAdmission::new(Stm32h7KeyStore::new(), Stm32h7CryptoEngine::new());
//!
//! // Scheme-neutral gateway. `persisted_ts` is the signing-timestamp
//! // high-water mark restored from storage (reboot-replay defense).
//! let mut policy = SourcePolicy::new();
//! policy.bind(Principal::mavlink(1, 1, 5), CommandSource::GcsDatalink)?;
//! let freshness = FreshnessConfig {
//!     initial_trusted_counter: TrustedCounter::Trusted(persisted_ts),
//!     new_stream_max_age: NEW_STREAM_MAX_AGE_10US,
//! };
//! let mut gateway = CommandGateway::new(policy, freshness);
//!
//! // Runner: frame bytes → adapter → gateway → verified.
//! loop {
//!     if let Some(frame) = transport.try_recv_frame() {
//!         if let Ok(claim) = admission.authenticate(frame) {
//!             match gateway.admit(claim, now_us) {
//!                 Ok(verified) => ingress.receive(verified, now_us),
//!                 Err(_) => { /* rejected: logged, never executed */ }
//!             }
//!         }
//!     }
//! }
//! ```
//!
//! ## Security Model
//!
//! - **MAVLink signing profile** ([`admission::MavlinkAdmission`]):
//!   `sha256_48` (SHA-256 over `secret_key ‖ signed bytes`, truncated to
//!   48 bits; NOT HMAC), per-principal anti-replay with first-frame
//!   freshness, one credential (one authority) per `link_id`. Authenticity
//!   and replay resistance only — its threat model is deliberately narrow.
//! - **Insecure dev profile** (`admission::InsecureDevAdmission`): no
//!   cryptography; compiled only under the non-default `insecure-dev-auth`
//!   feature (SITL/bench).
//!
//! ## DO-178C Criticality
//!
//! - **DAL A/B**: Flight-critical security policy
//! - Commands MUST go through CommandGateway
//! - Bypass paths are prohibited (enforced by API design)

#![no_std]
#![forbid(unsafe_code)]

pub mod admission;
pub mod anti_replay;
pub mod auth;
pub mod errors;
pub mod gateway;
pub mod principal;

#[cfg(test)]
mod test_support;

// Re-export key types
pub use anti_replay::{AntiReplayWindow, NEW_STREAM_MAX_AGE_10US};
pub use auth::SignedAuth;
pub use errors::{AuthError, GatewayError};
pub use gateway::{
    AuthenticatedCommand, CommandGateway, CommandSource, CredentialError, FailsafeAuthority,
    FreshnessConfig, SourcePolicy, TrustedCounter, TrustedInternalCommand, VerificationReceipt,
    VerifiedSystemCommand, MAX_SOURCE_BINDINGS,
};
pub use principal::{Principal, SecurityScheme};
