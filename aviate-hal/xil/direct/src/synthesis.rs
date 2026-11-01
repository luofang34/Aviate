//! Truth-to-sensor synthesis for in-process `no_std` XIL simulation.

mod environment;
mod noise;

pub use environment::{DirectEnvironment, DirectOrigin, DirectSynthesisError};
pub use noise::{
    apply_baro_noise, apply_gnss_noise, apply_imu_noise, apply_mag_noise, DirectNoiseRng,
    DirectNoiseTier,
};

use aviate_hal_io::{GnssFix, RawBaroReading, RawGnssReading, RawImuReading, RawMagReading};
use world_core::LocalTangentPlane;

/// Simulator-supplied velocity, angular rate, and specific force.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DirectMeasuredKinematics {
    /// Linear velocity in local NED meters per second.
    pub velocity_ned_mps: [f32; 3],
    /// Body angular velocity in FRD radians per second.
    pub body_rates_frd_rps: [f32; 3],
    /// Body specific force in FRD meters per second squared (`[0, 0, -g]` at level rest).
    pub specific_force_frd_mps2: [f32; 3],
}

/// One ground-truth vehicle state sample in Aviate NED and FRD conventions.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DirectTruthSample {
    /// Monotonic simulation time in microseconds within one `reset_generation`.
    pub time_us: u64,
    /// Simulator world epoch; a change signals a world reset.
    pub reset_generation: u32,
    /// Simulator physics step index.
    pub step: u64,
    /// Vehicle position relative to the local NED origin in meters.
    pub position_ned_m: [f32; 3],
    /// Unit quaternion `[w, x, y, z]` rotating body FRD vectors into world NED.
    pub attitude_wxyz: [f32; 4],
    /// Direct kinematic quantities when the simulator computes them natively.
    pub measured: Option<DirectMeasuredKinematics>,
}

/// Minimum simulation periods between synthesized aiding-sensor updates.
///
/// A period of `0` emits a fresh reading on every synthesized step.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DirectSensorRates {
    /// Minimum microseconds between barometer samples (`20_000` = 50 Hz by default).
    pub baro_period_us: u64,
    /// Minimum microseconds between magnetometer samples (`10_000` = 100 Hz by default).
    pub mag_period_us: u64,
    /// Minimum microseconds between GNSS samples (`100_000` = 10 Hz by default).
    pub gnss_period_us: u64,
}

impl DirectSensorRates {
    /// Emit barometer, magnetometer, and GNSS readings on every step.
    #[must_use]
    pub const fn every_step() -> Self {
        Self {
            baro_period_us: 0,
            mag_period_us: 0,
            gnss_period_us: 0,
        }
    }
}

impl Default for DirectSensorRates {
    fn default() -> Self {
        Self {
            baro_period_us: 20_000,
            mag_period_us: 10_000,
            gnss_period_us: 100_000,
        }
    }
}

/// Synthesized raw sensor readings for one advancing simulation step.
#[derive(Debug, Clone, Copy)]
pub struct DirectStepSample {
    /// Simulation timestamp in microseconds.
    pub timestamp_us: u64,
    /// Elapsed simulation microseconds since the previous truth state (`> 0`).
    pub delta_us: u64,
    /// Simulator step counter from [`DirectTruthSample::step`].
    pub sim_step: u64,
    /// World epoch from [`DirectTruthSample::reset_generation`].
    pub reset_generation: u32,
    /// Synthesized IMU sample (always emitted when simulation time advances).
    pub imu: RawImuReading,
    /// Synthesized barometer sample when the barometer period elapsed.
    pub baro: Option<RawBaroReading>,
    /// Synthesized magnetometer sample when the magnetometer period elapsed.
    pub mag: Option<RawMagReading>,
    /// Synthesized GNSS sample when the GNSS period elapsed.
    pub gnss: Option<RawGnssReading>,
}

/// Stateful `no_std` synthesizer converting [`DirectTruthSample`] streams into HAL readings.
#[derive(Debug, Clone)]
pub struct DirectSynthesizer {
    origin: DirectOrigin,
    plane: LocalTangentPlane,
    environment: DirectEnvironment,
    rates: DirectSensorRates,
    noise_tier: DirectNoiseTier,
    rng: DirectNoiseRng,
    prev: Option<DirectTruthSample>,
    prev_ned_vel: Option<[f32; 3]>,
    last_baro_us: Option<u64>,
    last_mag_us: Option<u64>,
    last_gnss_us: Option<u64>,
}

impl DirectSynthesizer {
    /// Construct a [`DirectSynthesizer`] with explicit sensor periods and noise settings.
    ///
    /// # Errors
    ///
    /// Returns [`DirectSynthesisError`] when `origin` fails world-model validation.
    pub fn new(
        origin: DirectOrigin,
        rates: DirectSensorRates,
        noise_tier: DirectNoiseTier,
        noise_seed: u64,
    ) -> Result<Self, DirectSynthesisError> {
        let plane = LocalTangentPlane::new(origin.position)?;
        let environment = DirectEnvironment::at(origin.position, origin.epoch)?;
        Ok(Self {
            origin,
            plane,
            environment,
            rates,
            noise_tier,
            rng: DirectNoiseRng::new(noise_seed),
            prev: None,
            prev_ned_vel: None,
            last_baro_us: None,
            last_mag_us: None,
            last_gnss_us: None,
        })
    }

    /// Construct a [`DirectSynthesizer`] with [`DirectSensorRates::default`].
    ///
    /// # Errors
    ///
    /// Returns [`DirectSynthesisError`] when `origin` fails world-model validation.
    pub fn with_default_rates(
        origin: DirectOrigin,
        noise_tier: DirectNoiseTier,
        noise_seed: u64,
    ) -> Result<Self, DirectSynthesisError> {
        Self::new(origin, DirectSensorRates::default(), noise_tier, noise_seed)
    }

    /// Clear step history and sensor decimation timestamps for a world reset.
    pub fn reset(&mut self) {
        self.prev = None;
        self.prev_ned_vel = None;
        self.last_baro_us = None;
        self.last_mag_us = None;
        self.last_gnss_us = None;
    }

    /// Reset step history and reseed the deterministic noise generator.
    pub fn reset_with_seed(&mut self, noise_seed: u64) {
        self.reset();
        self.rng.reset(noise_seed);
    }

    /// Return the configured [`DirectOrigin`].
    #[must_use]
    pub const fn origin(&self) -> DirectOrigin {
        self.origin
    }

    /// Return the latest [`DirectEnvironment`].
    #[must_use]
    pub const fn environment(&self) -> &DirectEnvironment {
        &self.environment
    }

    /// Return the active [`DirectSensorRates`].
    #[must_use]
    pub const fn rates(&self) -> DirectSensorRates {
        self.rates
    }

    /// Update the active [`DirectSensorRates`].
    pub fn set_rates(&mut self, rates: DirectSensorRates) {
        self.rates = rates;
    }

    /// Return the active [`DirectNoiseTier`].
    #[must_use]
    pub const fn noise_tier(&self) -> DirectNoiseTier {
        self.noise_tier
    }

    /// Update the active [`DirectNoiseTier`].
    pub fn set_noise_tier(&mut self, tier: DirectNoiseTier) {
        self.noise_tier = tier;
    }

    /// Synthesize sensor readings for `truth` when simulation time advances.
    ///
    /// Returns `None` on the initial anchor sample, on a `reset_generation` change,
    /// on a time regression, or when `truth.time_us` equals the previous timestamp.
    pub fn synthesize(&mut self, truth: &DirectTruthSample) -> Option<DirectStepSample> {
        let prev = match self.prev {
            Some(prev)
                if prev.reset_generation == truth.reset_generation
                    && truth.time_us >= prev.time_us =>
            {
                prev
            }
            _ => {
                self.reset();
                self.environment
                    .follow(&self.plane, truth.position_ned_m, self.origin.epoch);
                self.prev = Some(*truth);
                return None;
            }
        };

        let delta_us = truth.time_us.saturating_sub(prev.time_us);
        if delta_us == 0 {
            return None;
        }

        self.environment
            .follow(&self.plane, truth.position_ned_m, self.origin.epoch);
        let gravity = self.environment.gravity_mps2 as f32;
        let dt_s = (delta_us as f32) * 1.0e-6;
        let (ned_vel, gyro, accel) = match truth.measured {
            Some(m) => (
                m.velocity_ned_mps,
                m.body_rates_frd_rps,
                m.specific_force_frd_mps2,
            ),
            None => derived_kinematics(truth, &prev, dt_s, self.prev_ned_vel, gravity),
        };

        let mut imu = RawImuReading {
            accel,
            gyro,
            temperature: Some(25.0),
        };
        apply_imu_noise(&mut imu, self.noise_tier, &mut self.rng);

        let baro = self.synthesize_baro(truth.time_us);
        let mag = self.synthesize_mag(truth.time_us, truth.attitude_wxyz);
        let gnss = self.synthesize_gnss(truth.time_us, truth.position_ned_m, ned_vel);

        self.prev = Some(*truth);
        self.prev_ned_vel = Some(ned_vel);

        Some(DirectStepSample {
            timestamp_us: truth.time_us,
            delta_us,
            sim_step: truth.step,
            reset_generation: truth.reset_generation,
            imu,
            baro,
            mag,
            gnss,
        })
    }

    fn synthesize_baro(&mut self, time_us: u64) -> Option<RawBaroReading> {
        if !period_due(self.last_baro_us, time_us, self.rates.baro_period_us) {
            return None;
        }
        self.last_baro_us = Some(time_us);
        let mut baro = RawBaroReading {
            pressure_pa: self.environment.air.pressure_pa as f32,
            temperature_c: self.environment.air.temperature_c() as f32,
            ..RawBaroReading::default()
        };
        apply_baro_noise(&mut baro, self.noise_tier, &mut self.rng);
        Some(baro)
    }

    fn synthesize_mag(&mut self, time_us: u64, attitude_wxyz: [f32; 4]) -> Option<RawMagReading> {
        if !period_due(self.last_mag_us, time_us, self.rates.mag_period_us) {
            return None;
        }
        self.last_mag_us = Some(time_us);
        let mut mag = RawMagReading {
            field_ut: rotate_world_to_body(attitude_wxyz, self.environment.field_ned_ut),
        };
        apply_mag_noise(&mut mag, self.noise_tier, &mut self.rng);
        Some(mag)
    }

    fn synthesize_gnss(
        &mut self,
        time_us: u64,
        position_ned_m: [f32; 3],
        vel_ned: [f32; 3],
    ) -> Option<RawGnssReading> {
        if !period_due(self.last_gnss_us, time_us, self.rates.gnss_period_us) {
            return None;
        }
        self.last_gnss_us = Some(time_us);
        let pos = self.environment.position;
        let mut gnss = RawGnssReading {
            lat_deg: pos.latitude_deg,
            lon_deg: pos.longitude_deg,
            alt_m: pos.ellipsoidal_height_m as f32,
            position_ned: position_ned_m,
            vel_ned,
            fix: GnssFix::ThreeD,
            h_acc: 0.5,
            v_acc: 0.8,
            satellites: 14,
        };
        apply_gnss_noise(&mut gnss, self.noise_tier, &mut self.rng);
        Some(gnss)
    }
}

fn period_due(last_us: Option<u64>, now_us: u64, period_us: u64) -> bool {
    match last_us {
        None => true,
        Some(prev_us) => now_us.saturating_sub(prev_us) >= period_us,
    }
}

fn derived_kinematics(
    truth: &DirectTruthSample,
    prev: &DirectTruthSample,
    dt_s: f32,
    prev_ned_vel: Option<[f32; 3]>,
    gravity: f32,
) -> ([f32; 3], [f32; 3], [f32; 3]) {
    let now_vel = [
        (truth.position_ned_m[0] - prev.position_ned_m[0]) / dt_s,
        (truth.position_ned_m[1] - prev.position_ned_m[1]) / dt_s,
        (truth.position_ned_m[2] - prev.position_ned_m[2]) / dt_s,
    ];
    let gyro = body_rate_from_quats(truth.attitude_wxyz, prev.attitude_wxyz, dt_s);
    let accel_inertial_ned = match prev_ned_vel {
        Some(v_prev) => {
            let lim = 3.0 * gravity;
            [
                ((now_vel[0] - v_prev[0]) / dt_s).clamp(-lim, lim),
                ((now_vel[1] - v_prev[1]) / dt_s).clamp(-lim, lim),
                ((now_vel[2] - v_prev[2]) / dt_s).clamp(-lim, lim),
            ]
        }
        None => [0.0, 0.0, 0.0],
    };
    let specific_force_ned = [
        accel_inertial_ned[0],
        accel_inertial_ned[1],
        accel_inertial_ned[2] - gravity,
    ];
    (
        now_vel,
        gyro,
        rotate_world_to_body(truth.attitude_wxyz, specific_force_ned),
    )
}

fn body_rate_from_quats(now: [f32; 4], prev: [f32; 4], dt_s: f32) -> [f32; 3] {
    let qd_w = now[0] * prev[0] + now[1] * prev[1] + now[2] * prev[2] + now[3] * prev[3];
    let qd_x = -now[0] * prev[1] + now[1] * prev[0] - now[2] * prev[3] + now[3] * prev[2];
    let qd_y = -now[0] * prev[2] + now[1] * prev[3] + now[2] * prev[0] - now[3] * prev[1];
    let qd_z = -now[0] * prev[3] - now[1] * prev[2] + now[2] * prev[1] + now[3] * prev[0];
    let sign = if qd_w >= 0.0 { 1.0 } else { -1.0 };
    let omega_ned = [
        2.0 * sign * qd_x / dt_s,
        2.0 * sign * qd_y / dt_s,
        2.0 * sign * qd_z / dt_s,
    ];
    rotate_world_to_body(now, omega_ned)
}

/// Rotate a world-NED vector `v_world` into body-FRD coordinates using conjugate(`q_wxyz`).
#[must_use]
pub fn rotate_world_to_body(q_wxyz: [f32; 4], v_world: [f32; 3]) -> [f32; 3] {
    let [w, x, y, z] = q_wxyz;
    let r00 = 1.0 - 2.0 * (y * y + z * z);
    let r01 = 2.0 * (x * y + w * z);
    let r02 = 2.0 * (x * z - w * y);
    let r10 = 2.0 * (x * y - w * z);
    let r11 = 1.0 - 2.0 * (x * x + z * z);
    let r12 = 2.0 * (y * z + w * x);
    let r20 = 2.0 * (x * z + w * y);
    let r21 = 2.0 * (y * z - w * x);
    let r22 = 1.0 - 2.0 * (x * x + y * y);
    [
        r00 * v_world[0] + r01 * v_world[1] + r02 * v_world[2],
        r10 * v_world[0] + r11 * v_world[1] + r12 * v_world[2],
        r20 * v_world[0] + r21 * v_world[1] + r22 * v_world[2],
    ]
}
