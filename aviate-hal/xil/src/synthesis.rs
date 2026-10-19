//! Simulator truth → flight-controller sensor samples.
//!
//! A truth-driven SITL board reads the simulated vehicle's true state
//! (position, attitude, simulation time) and makes the IMU, barometer,
//! magnetometer and GNSS readings a real vehicle at that state would
//! give. [`StepSynthesizer`] does that for any simulator: the board
//! converts its simulator's state into a [`TruthSample`] in Aviate's
//! NED / FRD conventions and passes one per read. Tier-matched Gaussian
//! noise is applied on top when the board selects a [`NoiseTier`].

mod noise;
mod origin;

pub use noise::{
    apply_baro_noise, apply_gnss_noise, apply_imu_noise, apply_mag_noise, NoiseRng, NoiseTier,
};
pub use origin::{GeoOrigin, OriginError, ORIGIN_ENV};

mod environment;

pub use environment::{Environment, SynthesisError};

use world_core::{DecimalYear, LocalTangentPlane};

use crate::sim_types::{
    sensor_presence, SimBaroData, SimGnssData, SimGnssFix, SimImuData, SimMagData, SimSensorPacket,
    SimTimestampUs,
};

/// One read of the simulated vehicle's true state, in Aviate's
/// conventions.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TruthSample {
    /// Simulation time, microseconds. Monotonic within one world epoch.
    pub time_us: u64,
    /// The simulated world's epoch; a world reset gives a new value.
    pub reset_generation: u32,
    /// The simulator's step counter for this state.
    pub step: u64,
    /// Position relative to the world origin, NED, meters.
    pub position_ned_m: [f32; 3],
    /// Attitude as the body FRD → world NED rotation `[w, x, y, z]`.
    pub attitude: [f32; 4],
}

/// Apply tier-matched additive Gaussian noise to every populated
/// sensor channel in `packet` in place. No-op for `NoiseTier::Off`.
pub fn apply_packet_noise(packet: &mut SimSensorPacket, tier: NoiseTier, rng: &mut NoiseRng) {
    if tier == NoiseTier::Off {
        return;
    }
    if let Some(ref mut imu) = packet.imu {
        apply_imu_noise(imu, tier, rng);
    }
    if let Some(ref mut baro) = packet.baro {
        apply_baro_noise(baro, tier, rng);
    }
    if let Some(ref mut mag) = packet.mag {
        apply_mag_noise(mag, tier, rng);
    }
    if let Some(ref mut gnss) = packet.gnss {
        apply_gnss_noise(gnss, tier, rng);
    }
}

/// One sensor sample for one new simulator step.
#[derive(Debug, Clone)]
pub struct StepSample {
    /// The synthesized sensor packet, stamped with simulation time.
    pub packet: SimSensorPacket,
    /// Simulation time since the previous state, in microseconds.
    /// Always non-zero.
    pub delta_us: u64,
    /// The physics step the packet belongs to.
    pub sim_step: u64,
}

/// Turns successive truth reads into one sensor sample per new
/// simulator step.
///
/// A board reads the simulator on its own clock, and the simulator
/// does not run at exactly real time. A read can therefore see the
/// same state again, or skip a state. A repeated state has no motion
/// to difference: a sample made from it reports zero velocity and
/// zero body rate, and the acceleration then swings by the full clamp
/// in one direction and back on the next fresh state. The kernel
/// treats each such pair as a real jolt. So [`Self::next`] gives a
/// sample only when simulation time advanced, and the sample period
/// is the simulation time between the two states.
#[derive(Debug)]
pub struct StepSynthesizer {
    /// The local NED frame at the world origin.
    plane: LocalTangentPlane,
    /// The date the Earth field is evaluated on.
    epoch: DecimalYear,
    /// The environment at the latest state.
    environment: Environment,
    prev: Option<TruthSample>,
    /// Derived NED velocity of `prev`. `None` until two states of the
    /// same world epoch exist, because acceleration needs two
    /// velocities.
    prev_ned_vel: Option<[f32; 3]>,
}

impl StepSynthesizer {
    /// Forget all history, for a new simulator attachment.
    pub fn reset(&mut self) {
        self.prev = None;
        self.prev_ned_vel = None;
    }

    /// A synthesizer whose readings are made around `origin`, with the
    /// Earth field of `epoch`.
    ///
    /// # Errors
    ///
    /// A world model's refusal at the origin: a malformed origin, the
    /// date outside the geomagnetic model's validity, or an elevation
    /// outside the standard atmosphere. Each refuses startup, since a
    /// sensor made from a substituted environment flies the controller
    /// somewhere else.
    pub fn new(origin: GeoOrigin, epoch: DecimalYear) -> Result<Self, SynthesisError> {
        let plane = origin.plane()?;
        let environment = Environment::at(plane.origin(), epoch)?;
        Ok(Self {
            plane,
            epoch,
            environment,
            prev: None,
            prev_ned_vel: None,
        })
    }

    /// The environment at the latest state.
    pub fn environment(&self) -> &Environment {
        &self.environment
    }

    /// The sample for `truth`, or `None` when `truth` does not start a
    /// new step: the first state, a state from a new world epoch (a
    /// world reset rewinds the clock), or a repeated state.
    pub fn next(&mut self, truth: &TruthSample) -> Option<StepSample> {
        let prev = match self.prev {
            Some(prev)
                if prev.reset_generation == truth.reset_generation
                    && truth.time_us >= prev.time_us =>
            {
                prev
            }
            _ => {
                self.prev = Some(*truth);
                self.prev_ned_vel = None;
                return None;
            }
        };
        let delta_us = truth.time_us - prev.time_us;
        if delta_us == 0 {
            return None;
        }
        self.environment
            .follow(&self.plane, truth.position_ned_m, self.epoch);
        let (packet, ned_vel) =
            synthesize_packet(truth, &prev, delta_us, self.prev_ned_vel, &self.environment);
        self.prev = Some(*truth);
        self.prev_ned_vel = Some(ned_vel);
        Some(StepSample {
            packet,
            delta_us,
            sim_step: truth.step,
        })
    }
}

/// The body angular velocity that turns attitude `prev` into `now` over
/// `dt` seconds.
///
/// Derived from successive attitudes rather than taken from the
/// simulator, so the gyro has the shape a real gyro would deliver
/// whatever rate signal a simulator does or does not publish:
///
///   q_delta = q_new ⊗ q_old.conjugate
///   ω_world ≈ 2 · q_delta.imag / dt    (small-angle approx)
///
/// then ω_world is rotated into the body frame.
fn body_rate(now: [f32; 4], prev: [f32; 4], dt: f32) -> [f32; 3] {
    let qd_w = now[0] * prev[0] + now[1] * prev[1] + now[2] * prev[2] + now[3] * prev[3];
    let qd_x = -now[0] * prev[1] + now[1] * prev[0] - now[2] * prev[3] + now[3] * prev[2];
    let qd_y = -now[0] * prev[2] + now[1] * prev[3] + now[2] * prev[0] - now[3] * prev[1];
    let qd_z = -now[0] * prev[3] - now[1] * prev[2] + now[2] * prev[1] + now[3] * prev[0];
    // Force shortest-arc (qd and -qd are the same rotation; we want the
    // imaginary part in the +w hemisphere).
    let sign = if qd_w >= 0.0 { 1.0 } else { -1.0 };
    let omega_world_ned = [
        2.0 * sign * qd_x / dt,
        2.0 * sign * qd_y / dt,
        2.0 * sign * qd_z / dt,
    ];
    rotate_world_to_body(now, omega_world_ned)
}

/// Build a `SimSensorPacket` from two successive truth states
/// `delta_us` of simulation time apart (`delta_us > 0`).
///
/// `prev_ned_vel` is the NED-frame velocity this function returned for
/// `prev`, or `None` when `prev` has no derived velocity.
///
/// Velocity comes from a position difference and acceleration from a
/// velocity difference, so the controllers receive the same shape of
/// sensor data a real IMU and GNSS pair would produce rather than a
/// state vector the simulator happened to publish.
///
/// Returns `(packet, current_ned_vel)`.
fn synthesize_packet(
    truth: &TruthSample,
    prev: &TruthSample,
    delta_us: u64,
    prev_ned_vel: Option<[f32; 3]>,
    environment: &Environment,
) -> (SimSensorPacket, [f32; 3]) {
    let gravity = environment.gravity_mps2 as f32;
    let now_ned_pos = truth.position_ned_m;
    let dt = delta_us as f32 * 1e-6;

    let prev_ned_pos = prev.position_ned_m;
    let now_ned_vel = [
        (now_ned_pos[0] - prev_ned_pos[0]) / dt,
        (now_ned_pos[1] - prev_ned_pos[1]) / dt,
        (now_ned_pos[2] - prev_ned_pos[2]) / dt,
    ];

    let q_ned = truth.attitude;
    let gyro = body_rate(q_ned, prev.attitude, dt);

    // Inertial acceleration from the velocity finite difference — the
    // same baseline the velocity above is derived over. The ±3 g clamp
    // guards the spikes a physics-step time hiccup can produce. Feeding
    // zero here instead (gravity-only IMU) starves the EKF predict step
    // of every vertical transient: the velocity estimate then moves
    // only at the GNSS fusion gain, lags hard braking by seconds, and a
    // braked climb descends into the ground while the filter still
    // believes it is climbing.
    let accel_inertial_ned = match prev_ned_vel {
        Some(prev_ned_vel) => {
            let lim = 3.0 * gravity;
            [
                ((now_ned_vel[0] - prev_ned_vel[0]) / dt).clamp(-lim, lim),
                ((now_ned_vel[1] - prev_ned_vel[1]) / dt).clamp(-lim, lim),
                ((now_ned_vel[2] - prev_ned_vel[2]) / dt).clamp(-lim, lim),
            ]
        }
        None => [0.0, 0.0, 0.0],
    };
    let accel_ned = [
        accel_inertial_ned[0],
        accel_inertial_ned[1],
        accel_inertial_ned[2] - gravity,
    ];
    let accel_body = rotate_world_to_body(q_ned, accel_ned);

    let imu = SimImuData {
        accel: accel_body,
        gyro,
        temperature: Some(25.0),
    };

    let baro = SimBaroData {
        pressure_pa: environment.air.pressure_pa as f32,
        temperature_c: environment.air.temperature_c() as f32,
        ..SimBaroData::default()
    };

    let mag = SimMagData {
        field_ut: rotate_world_to_body(q_ned, environment.field_ned_ut),
    };

    let position = environment.position;
    let gnss = SimGnssData {
        lat_deg: position.latitude_deg,
        lon_deg: position.longitude_deg,
        alt_m: position.ellipsoidal_height_m as f32,
        // The kernel uses local NED, so the ground-truth NED position is
        // passed directly and the GNSS update agrees with the EKF frame;
        // lat/lon/alt are what a real receiver reports.
        position_ned: now_ned_pos,
        vel_ned: now_ned_vel,
        fix: SimGnssFix::ThreeD,
        satellites: 14,
        h_acc: 0.5,
        v_acc: 0.8,
    };

    let packet = SimSensorPacket {
        timestamp_us: truth.time_us as SimTimestampUs,
        presence_mask: sensor_presence::ACCELEROMETER
            | sensor_presence::GYROSCOPE
            | sensor_presence::MAGNETOMETER
            | sensor_presence::ABSOLUTE_PRESSURE,
        imu: Some(imu),
        baro: Some(baro),
        mag: Some(mag),
        gnss: Some(gnss),
        reset_generation: Some(truth.reset_generation),
    };
    (packet, now_ned_vel)
}

/// Rotate a world-frame vector into body frame via the inverse of `q`.
/// `q = [w, x, y, z]` rotates body→world; the conjugate rotates back.
pub fn rotate_world_to_body(q: [f32; 4], v: [f32; 3]) -> [f32; 3] {
    let [w, x, y, z] = q;
    // Inverse rotation = conjugate for a unit quaternion. The matrix
    // form below is the body-from-world DCM.
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
        r00 * v[0] + r01 * v[1] + r02 * v[2],
        r10 * v[0] + r11 * v[1] + r12 * v[2],
        r20 * v[0] + r21 * v[1] + r22 * v[2],
    ]
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::panic)]
mod tests;
