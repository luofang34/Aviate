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

use aviate_hal_xil::sim_types::{SimBaroData, SimGnssData, SimImuData, SimMagData};

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
        match std::env::var("AVIATE_SENSOR_NOISE")
            .unwrap_or_default()
            .to_ascii_lowercase()
            .as_str()
        {
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

/// Mutate `gnss` in place with tier-matched additive position noise.
/// Converts the meter-scale 1-σ to a lat/lon delta around the
/// supplied reference latitude.
pub fn apply_gnss_noise(
    gnss: &mut SimGnssData,
    tier: NoiseTier,
    rng: &mut NoiseRng,
    ref_lat_deg: f64,
) {
    if tier == NoiseTier::Off {
        return;
    }
    let h_sigma = tier.gnss_h_sigma_m() as f64;
    let v_sigma = tier.gnss_v_sigma_m();
    let lat_per_m = 1.0 / 111_111.0;
    let lon_per_m = 1.0 / (111_111.0 * ref_lat_deg.to_radians().cos());
    gnss.lat_deg += rng.next_normal() as f64 * h_sigma * lat_per_m;
    gnss.lon_deg += rng.next_normal() as f64 * h_sigma * lon_per_m;
    gnss.alt_m += rng.next_normal() * v_sigma;
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::panic)]
mod tests {
    use super::*;

    #[test]
    fn off_tier_is_passthrough() {
        let mut rng = NoiseRng::new(42);
        let mut imu = SimImuData {
            accel: [1.0, 2.0, -9.81],
            gyro: [0.1, 0.0, -0.1],
            temperature: Some(25.0),
        };
        apply_imu_noise(&mut imu, NoiseTier::Off, &mut rng);
        assert_eq!(imu.accel, [1.0, 2.0, -9.81]);
        assert_eq!(imu.gyro, [0.1, 0.0, -0.1]);
    }

    #[test]
    fn mems_tier_perturbs_values() {
        let mut rng = NoiseRng::new(42);
        let original = [1.0_f32, 2.0, -9.81];
        let mut imu = SimImuData {
            accel: original,
            gyro: [0.0; 3],
            temperature: None,
        };
        apply_imu_noise(&mut imu, NoiseTier::Mems, &mut rng);
        let any_changed = imu.accel.iter().zip(original.iter()).any(|(a, b)| a != b);
        assert!(any_changed, "MEMS tier should produce non-zero noise");
    }

    #[test]
    fn same_seed_yields_same_stream() {
        let mut a = NoiseRng::new(7);
        let mut b = NoiseRng::new(7);
        for _ in 0..100 {
            assert!((a.next_normal() - b.next_normal()).abs() < 1e-7);
        }
    }

    #[test]
    fn mean_close_to_zero_over_many_samples() {
        let mut rng = NoiseRng::new(123);
        let n = 10_000;
        let mut sum = 0.0;
        for _ in 0..n {
            sum += rng.next_normal();
        }
        let mean = sum / n as f32;
        assert!(
            mean.abs() < 0.05,
            "mean of {} samples was {}, expected near 0",
            n,
            mean
        );
    }

    #[test]
    fn variance_close_to_one_over_many_samples() {
        let mut rng = NoiseRng::new(456);
        let n = 10_000;
        let mut sum = 0.0;
        let mut sumsq = 0.0;
        for _ in 0..n {
            let x = rng.next_normal();
            sum += x;
            sumsq += x * x;
        }
        let mean = sum / n as f32;
        let variance = sumsq / n as f32 - mean * mean;
        assert!(
            (variance - 1.0).abs() < 0.1,
            "variance of {} samples was {}, expected near 1.0",
            n,
            variance
        );
    }

    // Env-var dispatch tests share a process-global resource
    // (`AVIATE_SENSOR_NOISE`). Cargo runs unit tests in parallel by
    // default, so splitting these into multiple `#[test]` functions
    // would let them race on the var. Folded into one body with
    // explicit cleanup so each assertion sees a known state.
    #[test]
    fn env_dispatch_table() {
        let cases = [
            ("", NoiseTier::Off),
            ("0", NoiseTier::Off),
            ("off", NoiseTier::Off),
            ("OFF", NoiseTier::Off),
            ("false", NoiseTier::Off),
            ("1", NoiseTier::Mems),
            ("mems", NoiseTier::Mems),
            ("MEMS", NoiseTier::Mems),
            ("on", NoiseTier::Mems),
            ("tactical", NoiseTier::Tactical),
            ("high", NoiseTier::Tactical),
            ("2", NoiseTier::Tactical),
            ("garbage", NoiseTier::Off),
        ];
        for (value, expected) in cases {
            if value.is_empty() {
                unsafe {
                    std::env::remove_var("AVIATE_SENSOR_NOISE");
                }
            } else {
                unsafe {
                    std::env::set_var("AVIATE_SENSOR_NOISE", value);
                }
            }
            assert_eq!(
                NoiseTier::from_env(),
                expected,
                "AVIATE_SENSOR_NOISE={:?} should map to {:?}",
                value,
                expected
            );
        }
        unsafe {
            std::env::remove_var("AVIATE_SENSOR_NOISE");
        }
    }
}
