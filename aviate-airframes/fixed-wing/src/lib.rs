//! Fixed-wing airframe definitions.
//!
//! Placeholder crate for fixed-wing vehicle airframe descriptors. The
//! control surfaces, mixer, and flight envelope land here as the
//! fixed-wing airframe family is implemented.
#![no_std]
#![forbid(unsafe_code)]
#![forbid(clippy::panic)]
#![forbid(clippy::unwrap_used)]
#![forbid(clippy::expect_used)]
#![deny(missing_docs)]

/// Placeholder fixed-wing airframe definitions.
pub fn airframe_id() -> &'static str {
    "fixed-wing"
}
