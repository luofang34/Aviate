//! The Earth's main magnetic field from the World Magnetic Model.
//!
//! [`wmm2025`] evaluates WMM2025 at a WGS-84 geodetic point and a decimal
//! year, and gives the field in local north-east-down components with its
//! declination, inclination and intensity. The model is valid from 2025.0
//! to 2030.0; a date outside that window is refused with a typed error,
//! not extrapolated.
//!
//! The crate is `no_std`, allocation-free and deterministic: the same
//! inputs give the same bits on every target. Its only dependency is
//! `libm`. The WMM coefficients are public domain (NOAA NCEI / BGS); see
//! the `coefficients` module for the source and checksum.

#![no_std]

mod coefficients;
mod date;
mod error;
mod field;
mod model;

pub use date::decimal_year_from_unix_seconds;
pub use error::GeomagError;
pub use field::{GeodeticPoint, MagneticField, ModelId, WMM2025};
pub use model::wmm2025;
