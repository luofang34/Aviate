//! Why an evaluation was refused.

use core::fmt;

/// Why the model refused to evaluate.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum GeomagError {
    /// The date is outside the model's validity window.
    OutsideValidity {
        /// The requested decimal year.
        year: f64,
        /// First valid decimal year.
        valid_from: f64,
        /// Last valid decimal year.
        valid_until: f64,
    },
    /// Latitude outside [-90, 90], height outside the model's domain, or a
    /// non-finite input.
    InvalidPoint,
}

impl fmt::Display for GeomagError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::OutsideValidity {
                year,
                valid_from,
                valid_until,
            } => write!(
                f,
                "decimal year {year} is outside the model validity {valid_from}..={valid_until}"
            ),
            Self::InvalidPoint => f.write_str("the point is outside the model domain"),
        }
    }
}

impl core::error::Error for GeomagError {}
