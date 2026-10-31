//! Deterministic `no_std` Gaussian sensor noise tiers and generator.

use aviate_hal_io::{RawBaroReading, RawGnssReading, RawImuReading, RawMagReading};
use world_core::{GeodeticPosition, LocalTangentPlane, Ned};

/// Additive Gaussian sensor noise grade for [`super::DirectSynthesizer`].
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub enum DirectNoiseTier {
    /// Zero additive sensor noise.
    #[default]
    Off,
    /// Consumer MEMS grade (ICM-20649 class).
    Consumer,
    /// Industrial MEMS grade (ICM-42688-P / BMI088 class).
    Industrial,
    /// Tactical grade (ADIS16470 class).
    Tactical,
}

impl DirectNoiseTier {
    /// Return accelerometer 1-sigma noise in meters per second squared.
    #[must_use]
    pub const fn accel_sigma_mps2(self) -> f32 {
        match self {
            Self::Off => 0.0,
            Self::Consumer => 0.08,
            Self::Industrial => 0.04,
            Self::Tactical => 0.02,
        }
    }

    /// Return gyroscope 1-sigma noise in radians per second.
    #[must_use]
    pub const fn gyro_sigma_rps(self) -> f32 {
        match self {
            Self::Off => 0.0,
            Self::Consumer => 0.008,
            Self::Industrial => 0.004,
            Self::Tactical => 0.002,
        }
    }

    /// Return barometer 1-sigma pressure noise in Pascals.
    #[must_use]
    pub const fn baro_sigma_pa(self) -> f32 {
        match self {
            Self::Off => 0.0,
            Self::Consumer => 10.0,
            Self::Industrial => 5.0,
            Self::Tactical => 2.5,
        }
    }

    /// Return magnetometer 1-sigma field noise in microtesla per axis.
    #[must_use]
    pub const fn mag_sigma_ut(self) -> f32 {
        match self {
            Self::Off => 0.0,
            Self::Consumer => 0.4,
            Self::Industrial => 0.2,
            Self::Tactical => 0.1,
        }
    }

    /// Return GNSS horizontal 1-sigma position noise in meters.
    #[must_use]
    pub const fn gnss_h_sigma_m(self) -> f32 {
        match self {
            Self::Off => 0.0,
            Self::Consumer => 0.5,
            Self::Industrial => 0.35,
            Self::Tactical => 0.2,
        }
    }

    /// Return GNSS vertical 1-sigma position noise in meters.
    #[must_use]
    pub const fn gnss_v_sigma_m(self) -> f32 {
        match self {
            Self::Off => 0.0,
            Self::Consumer => 0.8,
            Self::Industrial => 0.5,
            Self::Tactical => 0.3,
        }
    }

    /// Return GNSS 1-sigma velocity noise in meters per second per axis.
    #[must_use]
    pub const fn gnss_vel_sigma_mps(self) -> f32 {
        match self {
            Self::Off => 0.0,
            Self::Consumer => 0.05,
            Self::Industrial => 0.03,
            Self::Tactical => 0.02,
        }
    }
}

/// Deterministic `no_std` pseudo-random Gaussian sampler using `splitmix64` and Box-Muller.
#[derive(Debug, Clone, PartialEq)]
pub struct DirectNoiseRng {
    state: u64,
    cached_normal: Option<f32>,
}

impl DirectNoiseRng {
    /// Create a [`DirectNoiseRng`] seeded with `seed`.
    #[must_use]
    pub const fn new(seed: u64) -> Self {
        Self {
            state: seed,
            cached_normal: None,
        }
    }

    /// Reset the generator to `seed` and discard any cached normal variate.
    pub fn reset(&mut self, seed: u64) {
        self.state = seed;
        self.cached_normal = None;
    }

    /// Advance the `splitmix64` state and return the next 64-bit word.
    pub fn next_u64(&mut self) -> u64 {
        self.state = self.state.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.state;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    /// Sample one uniform `f32` in `[0.0, 1.0)`.
    pub fn next_uniform(&mut self) -> f32 {
        let bits = (self.next_u64() >> 40) as u32;
        (bits as f32) * (1.0 / ((1_u32 << 24) as f32))
    }

    /// Sample one zero-mean unit-variance Gaussian variate via Box-Muller.
    pub fn next_normal(&mut self) -> f32 {
        if let Some(z) = self.cached_normal.take() {
            return z;
        }
        let u1 = self.next_uniform().max(1.0e-7);
        let u2 = self.next_uniform();
        let r = libm::sqrtf(-2.0 * libm::logf(u1));
        let theta = 2.0 * core::f32::consts::PI * u2;
        let z0 = r * libm::cosf(theta);
        let z1 = r * libm::sinf(theta);
        self.cached_normal = Some(z1);
        z0
    }
}

/// Add tier-matched Gaussian noise to `imu` in place.
pub fn apply_imu_noise(imu: &mut RawImuReading, tier: DirectNoiseTier, rng: &mut DirectNoiseRng) {
    if tier == DirectNoiseTier::Off {
        return;
    }
    let acc_sigma = tier.accel_sigma_mps2();
    let gyro_sigma = tier.gyro_sigma_rps();
    for axis in &mut imu.accel {
        *axis += rng.next_normal() * acc_sigma;
    }
    for axis in &mut imu.gyro {
        *axis += rng.next_normal() * gyro_sigma;
    }
}

/// Add tier-matched Gaussian pressure noise to `baro` in place.
pub fn apply_baro_noise(
    baro: &mut RawBaroReading,
    tier: DirectNoiseTier,
    rng: &mut DirectNoiseRng,
) {
    if tier == DirectNoiseTier::Off {
        return;
    }
    baro.pressure_pa += rng.next_normal() * tier.baro_sigma_pa();
}

/// Add tier-matched Gaussian magnetic field noise to `mag` in place.
pub fn apply_mag_noise(mag: &mut RawMagReading, tier: DirectNoiseTier, rng: &mut DirectNoiseRng) {
    if tier == DirectNoiseTier::Off {
        return;
    }
    let sigma = tier.mag_sigma_ut();
    for axis in &mut mag.field_ut {
        *axis += rng.next_normal() * sigma;
    }
}

/// Add tier-matched Gaussian position and velocity noise to `gnss` in place.
pub fn apply_gnss_noise(
    gnss: &mut RawGnssReading,
    tier: DirectNoiseTier,
    rng: &mut DirectNoiseRng,
) {
    if tier == DirectNoiseTier::Off {
        return;
    }
    let dn = rng.next_normal() * tier.gnss_h_sigma_m();
    let de = rng.next_normal() * tier.gnss_h_sigma_m();
    let d_alt = rng.next_normal() * tier.gnss_v_sigma_m();
    let vel_sigma = tier.gnss_vel_sigma_mps();

    gnss.position_ned[0] += dn;
    gnss.position_ned[1] += de;
    gnss.position_ned[2] -= d_alt;
    gnss.alt_m += d_alt;

    let fix = GeodeticPosition::wgs84(gnss.lat_deg, gnss.lon_deg, f64::from(gnss.alt_m));
    if let Ok(frame) = LocalTangentPlane::new(fix) {
        let moved = frame.to_geodetic(Ned {
            north_m: f64::from(dn),
            east_m: f64::from(de),
            down_m: 0.0,
        });
        gnss.lat_deg = moved.latitude_deg;
        gnss.lon_deg = moved.longitude_deg;
    }

    for vel in &mut gnss.vel_ned {
        *vel += rng.next_normal() * vel_sigma;
    }
}
