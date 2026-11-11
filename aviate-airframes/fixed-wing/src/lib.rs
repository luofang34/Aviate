//! Fixed-wing airframe descriptions.
//!
//! Each description holds the numbers that the fixed-wing control law of
//! `aviate-core` needs for one vehicle: its airspeeds, its throttle
//! points, its envelope limits and its loop tuning. The law holds no
//! airframe number of its own.
#![no_std]
#![forbid(unsafe_code)]
#![forbid(clippy::panic)]
#![forbid(clippy::unwrap_used)]
#![forbid(clippy::expect_used)]
#![deny(missing_docs)]

pub mod description;
pub mod models;

pub use description::{FixedWingAirframe, FixedWingKernelError};
pub use models::{c172, rascal};
