//! Aviate — the public entry point to the Aviate flight-control kernel.
//!
//! This crate is a thin facade that re-exports [`aviate_core`], the minimal,
//! deterministic, hard-real-time inner-loop kernel: state estimation,
//! stabilization control, and actuator mixing. Depend on `aviate` as the
//! entry point; the layered implementation crates evolve behind it.
//!
//! The public API is **not stable**. This is experimental / work in progress
//! and re-exports `aviate-core` wholesale (`pub use aviate_core::*`); expect
//! breaking changes. A curated, stable surface may come later.
//!
//! ```
//! use aviate::math::Vector3;
//! use aviate::types::Meters;
//! let _ = Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0));
//! ```
//!
//! # Status
//!
//! **Experimental / work in progress — no warranty of any kind.** Not
//! qualified or certified for any use. Do not deploy on real hardware or rely
//! on it in any safety-critical context. See the project README.

#![no_std]
#![forbid(unsafe_code)]

#[doc(inline)]
pub use aviate_core::*;
