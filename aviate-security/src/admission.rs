//! Admission adapters: the scheme-specific edge of the command boundary.
//!
//! An admission adapter is where a security scheme's particulars live —
//! transport decoding and cryptographic verification. Each adapter turns
//! raw transport bytes into a sealed, scheme-neutral
//! [`AuthenticatedCommand`](crate::AuthenticatedCommand) that the
//! [`CommandGateway`](crate::CommandGateway) authorizes and stamps. The
//! gateway and everything downstream are identical for every adapter, so
//! adding a scheme (an AEAD or CCSDS envelope; see issue SEC-CMD) means
//! adding an adapter here, not touching the gateway.
//!
//! - [`MavlinkAdmission`] — MAVLink 2 signing (`sha256_48`), the
//!   interoperability baseline.
//! - `InsecureDevAdmission` — no cryptography, for SITL/bench only,
//!   compiled solely under the non-default `insecure-dev-auth` feature.

mod mavlink;

#[cfg(any(test, feature = "insecure-dev-auth"))]
mod insecure_dev;

pub use mavlink::MavlinkAdmission;

#[cfg(any(test, feature = "insecure-dev-auth"))]
pub use insecure_dev::InsecureDevAdmission;
