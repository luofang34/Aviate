#![no_std]
#![forbid(unsafe_code)]
#![deny(clippy::panic)]
#![deny(clippy::unwrap_used)]
#![deny(clippy::expect_used)]

/// Placeholder quadcopter airframe definitions.
pub fn airframe_id() -> &'static str {
    "quadcopter"
}
