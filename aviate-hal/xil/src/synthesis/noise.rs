//! Deterministic SITL sensor noise model.
//!
//! Provides reproducible Gaussian additive noise on the synthesized
//! IMU / baro / mag / GNSS readings. Disabled by default; opt in by
//! exporting `AVIATE_SENSOR_NOISE=1` (or `=high`/`=mems`/`=tactical`
//! to select a tier).
//!
//! Cert-evidence rationale: the kernel must handle realistic sensor
//! signals, not just perfect ground-truth replays. Running the
//! existing SITL missions with noise enabled exercises the
//! estimator's noise rejection and the controller's tracking under
//! the conditions the airframe will see in flight. Disabled by
//! default so the existing "perfect-IMU" mission gate is preserved
//! while we ratchet up; the path is here when we want to assert
//! noise-aware behavior.
//!
//! Determinism: a fixed-seed LCG (numerical recipes constants)
//! drives Box-Muller normal sampling. Same `AVIATE_SENSOR_NOISE`
//! tier + same seed yields bit-identical sensor streams across
//! runs.

use world_core::{GeodeticPosition, LocalTangentPlane, Ned};

use crate::sim_types::{SimBaroData, SimGnssData, SimImuData, SimMagData};

/// Noise tier selector. Values are 1-σ standard deviations matched
/// to representative MEMS / tactical-grade specs for an airframe of
/// this class.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum NoiseTier {
    /// No noise. Default. Existing SITL missions retain perfect-IMU
    /// behavior.
    Off,
    /// Consumer MEMS (e.g. ICM-20649 class). Higher noise floor,
    /// representative of low-cost airframe sensor suites.
    Mems,
    /// Tactical-grade (e.g. ADIS16470 class). Lower noise floor,
    /// representative of DAL-B-targeted designs.
    Tactical,
}

impl NoiseTier {
    /// Parse the `AVIATE_SENSOR_NOISE` env var. Unset / "0" / "off" →
    /// `Off`. "1" / "mems" → `Mems`. "tactical" / "high" → `Tactical`.
    /// Anything else falls through to `Off` (fail-soft).
    pub fn from_env() -> Self {
        Self::parse(&std::env::var("AVIATE_SENSOR_NOISE").unwrap_or_default())
    }

    /// The tier a `AVIATE_SENSOR_NOISE` value selects (see
    /// [`Self::from_env`]); an unset variable is the empty string.
    pub fn parse(value: &str) -> Self {
        match value.to_ascii_lowercase().as_str() {
            "" | "0" | "off" | "false" => Self::Off,
            "1" | "mems" | "on" | "true" => Self::Mems,
            "tactical" | "high" | "2" => Self::Tactical,
            _ => Self::Off,
        }
    }

    /// Accelerometer 1-σ noise in m/s².
    fn accel_sigma(&self) -> f32 {
        match self {
            Self::Off => 0.0,
            Self::Mems => 0.08,
            Self::Tactical => 0.02,
        }
    }

    /// Gyro 1-σ noise in rad/s.
    fn gyro_sigma(&self) -> f32 {
        match self {
            Self::Off => 0.0,
            Self::Mems => 0.008,
            Self::Tactical => 0.002,
        }
    }

    /// Baro 1-σ noise in Pa (≈ 0.08 m altitude at sea level).
    fn baro_sigma_pa(&self) -> f32 {
        match self {
            Self::Off => 0.0,
            Self::Mems => 10.0,
            Self::Tactical => 2.5,
        }
    }

    /// Magnetometer 1-σ noise in microtesla per axis.
    fn mag_sigma_ut(&self) -> f32 {
        match self {
            Self::Off => 0.0,
            Self::Mems => 0.4,
            Self::Tactical => 0.1,
        }
    }

    /// GNSS horizontal 1-σ position noise in meters.
    fn gnss_h_sigma_m(&self) -> f32 {
        match self {
            Self::Off => 0.0,
            Self::Mems => 0.5,
            Self::Tactical => 0.2,
        }
    }

    /// GNSS vertical 1-σ position noise in meters.
    fn gnss_v_sigma_m(&self) -> f32 {
        match self {
            Self::Off => 0.0,
            Self::Mems => 0.8,
            Self::Tactical => 0.3,
        }
    }
}

/// Deterministic LCG-based Gaussian sampler.
///
/// Numerical-recipes LCG constants (`6364136223846793005`,
/// `1442695040888963407`) give a full 64-bit period; Box-Muller folds
/// two uniforms into one normal.
pub struct NoiseRng {
    state: u64,
    cached_normal: Option<f32>,
}

impl NoiseRng {
    pub fn new(seed: u64) -> Self {
        Self {
            state: seed.wrapping_add(0x9E37_79B9_7F4A_7C15),
            cached_normal: None,
        }
    }

    fn next_u64(&mut self) -> u64 {
        self.state = self
            .state
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        self.state
    }

    fn next_uniform(&mut self) -> f32 {
        // Top 24 bits give a uniformly-distributed f32 in [0, 1).
        let bits = (self.next_u64() >> 40) as u32;
        bits as f32 / (1u32 << 24) as f32
    }

    /// Sample one zero-mean unit-variance Gaussian via Box-Muller.
    /// One transform produces two normals; the second is cached.
    pub fn next_normal(&mut self) -> f32 {
        if let Some(z) = self.cached_normal.take() {
            return z;
        }
        // Box-Muller. Clamp u1 away from 0 to avoid log(0).
        let u1 = (self.next_uniform()).max(1e-7);
        let u2 = self.next_uniform();
        let r = (-2.0_f32 * u1.ln()).sqrt();
        let theta = 2.0_f32 * std::f32::consts::PI * u2;
        let z0 = r * theta.cos();
        let z1 = r * theta.sin();
        self.cached_normal = Some(z1);
        z0
    }
}

/// Mutate `imu` in place with tier-matched additive Gaussian noise.
/// No-op for `NoiseTier::Off`.
pub fn apply_imu_noise(imu: &mut SimImuData, tier: NoiseTier, rng: &mut NoiseRng) {
    if tier == NoiseTier::Off {
        return;
    }
    let acc_s = tier.accel_sigma();
    let gyro_s = tier.gyro_sigma();
    for v in imu.accel.iter_mut() {
        *v += rng.next_normal() * acc_s;
    }
    for v in imu.gyro.iter_mut() {
        *v += rng.next_normal() * gyro_s;
    }
}

/// Mutate `baro` in place with tier-matched additive pressure noise.
pub fn apply_baro_noise(baro: &mut SimBaroData, tier: NoiseTier, rng: &mut NoiseRng) {
    if tier == NoiseTier::Off {
        return;
    }
    baro.pressure_pa += rng.next_normal() * tier.baro_sigma_pa();
}

/// Mutate `mag` in place with tier-matched additive field noise.
pub fn apply_mag_noise(mag: &mut SimMagData, tier: NoiseTier, rng: &mut NoiseRng) {
    if tier == NoiseTier::Off {
        return;
    }
    let s = tier.mag_sigma_ut();
    for v in mag.field_ut.iter_mut() {
        *v += rng.next_normal() * s;
    }
}

/// Mutate `gnss` in place with tier-matched additive position noise:
/// a horizontal offset around the fix in its own local frame, and a
/// vertical offset on the altitude.
pub fn apply_gnss_noise(gnss: &mut SimGnssData, tier: NoiseTier, rng: &mut NoiseRng) {
    if tier == NoiseTier::Off {
        return;
    }
    let h_sigma = f64::from(tier.gnss_h_sigma_m());
    let v_sigma = tier.gnss_v_sigma_m();
    let fix = GeodeticPosition::wgs84(gnss.lat_deg, gnss.lon_deg, f64::from(gnss.alt_m));
    let offset = Ned {
        north_m: f64::from(rng.next_normal()) * h_sigma,
        east_m: f64::from(rng.next_normal()) * h_sigma,
        down_m: 0.0,
    };
    // A fix that is not a well-formed position has no frame to move in;
    // it is left as it is.
    if let Ok(frame) = LocalTangentPlane::new(fix) {
        let moved = frame.to_geodetic(offset);
        gnss.lat_deg = moved.latitude_deg;
        gnss.lon_deg = moved.longitude_deg;
    }
    gnss.alt_m += rng.next_normal() * v_sigma;
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::panic)]
mod tests;
