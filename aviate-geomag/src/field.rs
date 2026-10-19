//! Inputs and outputs of an evaluation.

/// A WGS-84 geodetic point.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GeodeticPoint {
    /// Geodetic latitude, degrees north, in [-90, 90].
    pub latitude_deg: f64,
    /// Longitude, degrees east.
    pub longitude_deg: f64,
    /// Height above the WGS-84 ellipsoid, meters.
    pub height_m: f64,
}

/// Which model produced a field, and the dates it is valid for.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ModelId {
    /// Model name, as published.
    pub name: &'static str,
    /// Model epoch, decimal year.
    pub epoch: f64,
    /// First valid decimal year.
    pub valid_from: f64,
    /// Last valid decimal year.
    pub valid_until: f64,
}

/// The World Magnetic Model 2025.
pub const WMM2025: ModelId = ModelId {
    name: "WMM2025",
    epoch: 2025.0,
    valid_from: 2025.0,
    valid_until: 2030.0,
};

/// The main field at one point and date.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MagneticField {
    /// North component (X), nT.
    pub north_nt: f64,
    /// East component (Y), nT.
    pub east_nt: f64,
    /// Down component (Z), nT.
    pub down_nt: f64,
    /// Horizontal intensity (H), nT.
    pub horizontal_nt: f64,
    /// Total intensity (F), nT.
    pub total_nt: f64,
    /// Declination (D): angle from true north to the horizontal field,
    /// positive east, degrees.
    pub declination_deg: f64,
    /// Inclination (I): angle from the horizontal to the field, positive
    /// down, degrees.
    pub inclination_deg: f64,
    /// The model that produced this field.
    pub model: ModelId,
}

impl MagneticField {
    /// Declination in radians, positive east.
    pub fn declination_rad(&self) -> f64 {
        self.declination_deg.to_radians()
    }

    /// Inclination in radians, positive down.
    pub fn inclination_rad(&self) -> f64 {
        self.inclination_deg.to_radians()
    }
}
