//! Per-tick stepping, sensor feeding, command dispatch, and reset handling for [`super::DirectXilBoard`].

use aviate_core::ekf::seed::SeedUncertainty;
use aviate_core::hal::{ActuatorHal, SensorHal};
use aviate_core::math::Vector3;
use aviate_core::mixer::ActuatorState;
use aviate_core::sensor::{
    GeomagneticReference, GeomagneticSource, GnssFix, SensorHealth, SensorReading,
};
use aviate_core::time::{TimeDelta, TimeSource as CoreTimeSource, Timestamp};
use aviate_core::types::{Meters, MetersPerSecond, Radians, Seconds};
use aviate_core::ChannelId;
use aviate_hal_io::{ActuatorDriver, BoardHal, FakeActuator, RawBaroReading, MAX_ACTUATOR_OUTPUTS};
use aviate_link::mavlink::MavState;

use crate::host_sensors::DirectHostSensors;
use crate::synthesis::DirectTruthSample;

use super::ingress::{
    default_failsafe_command, init_state_to_mav_state, DirectCommandIngress, DirectSensorCache,
};
use super::triad::triad_init_quat;
use super::{DirectActuatorCmd, DirectBoardHal, DirectTime, DirectXilBoard};

/// Longest wait for a first GNSS fix before the estimator is seeded at rest
/// at the origin: two periods of a 1 Hz receiver.
const SEED_FIX_WAIT_US: u64 = 2_000_000;
/// Variance of the wind that a seed without a fix does not know: 8 m/s
/// at one standard deviation.
const SEED_WIND_VAR_M2PS2: f32 = 64.0;
/// Variance of an air-velocity seed along the nose: angle of attack,
/// sideslip and probe error, 2 m/s at one standard deviation.
const SEED_AIR_VELOCITY_VAR_M2PS2: f32 = 4.0;
/// Variance of roll and pitch of a seed from the accelerometer of a
/// vehicle that may be in a turn: 5 degrees at one standard deviation.
const SEED_TILT_VAR_RAD2: f32 = 0.0076;
/// Variance of the heading of a seed from the magnetometer: 10 degrees
/// at one standard deviation.
const SEED_HEADING_VAR_RAD2: f32 = 0.03;
/// Variance of a velocity seed of zero: nothing says how fast the
/// vehicle flies, 30 m/s at one standard deviation.
const SEED_SPEED_VAR_M2PS2: f32 = 900.0;
/// Variance of a horizontal position seed at the origin: 5 km at one
/// standard deviation. The first fix must lie inside the innovation gate
/// of this variance.
const SEED_POSITION_VAR_M2: f32 = 2.5e7;

impl<C, M> DirectXilBoard<C, M>
where
    C: aviate_core::control::VehicleController,
    M: aviate_core::mixer::Mixer,
{
    /// Advance the flight controller by one ground-truth sample and return actuator outputs.
    ///
    /// The built-in [`crate::DirectSynthesizer`] makes the sensor samples from `truth`.
    /// Use [`Self::step_sensors`] when the host owns the sensor models.
    pub fn step_truth(&mut self, truth: &DirectTruthSample) -> DirectActuatorCmd {
        self.handle_generation_transition(truth.reset_generation);
        let anchor_us = *self.boot_anchor_us.get_or_insert(truth.time_us);
        advance_board_hal_time(&mut self.board_hal, truth.time_us);

        let Some(sample) = self.synthesizer.synthesize(truth) else {
            return self.idle_step(truth.time_us);
        };
        let geomag = GeomagneticReference {
            declination: Radians(self.synthesizer.environment().declination_rad()),
            source: GeomagneticSource::Model,
        };
        self.run_cycle(
            &sample.host_sensors(Some(geomag)),
            sample.delta_us,
            geomag,
            anchor_us,
        )
    }

    /// Hold the last actuator command on a step that runs no kernel cycle.
    pub(super) fn idle_step(&mut self, time_us: u64) -> DirectActuatorCmd {
        self.drain_inbound_commands(time_us);
        self.last_actuator_cmd.timestamp_us = time_us;
        self.last_actuator_cmd.armed = self.is_armed();
        if !self.is_armed() {
            self.last_actuator_cmd.outputs = [0.0; MAX_ACTUATOR_OUTPUTS];
        }
        self.last_actuator_cmd
    }

    /// Run one kernel cycle on `sensors` after `delta_us` of simulation time.
    ///
    /// Both step entries end here, so the kernel cannot tell which one
    /// supplied the samples.
    pub(super) fn run_cycle(
        &mut self,
        sensors_in: &DirectHostSensors,
        delta_us: u64,
        geomag: GeomagneticReference,
        anchor_us: u64,
    ) -> DirectActuatorCmd {
        let time_us = sensors_in.time_us;
        // A rebase comes first, so that this cycle's fix, estimate and
        // telemetry share one frame.
        self.maybe_rebase();
        self.feed_and_read_sensors(sensors_in);
        self.maybe_initialize_ekf(geomag.declination.0, time_us);
        let sensors = self.sensor_cache.to_sensor_set(Some(geomag));

        if !self.kernel.is_ready() {
            let ts = Timestamp {
                ticks: time_us,
                source: CoreTimeSource::Internal,
            };
            let prev = self.kernel.state.init_state;
            let _ = self.kernel.init_step(&sensors, ts);
            if self.kernel.state.init_state != prev {
                self.endpoint
                    .set_system_status(init_state_to_mav_state(self.kernel.state.init_state));
            }
        }

        self.drain_inbound_commands(time_us);

        let time_delta = TimeDelta {
            dt_sec: Seconds(((delta_us as f32) * 1.0e-6).clamp(0.0001, 0.1)),
            tick_delta: delta_us,
        };
        let command_age_ms = self.ingress.setpoint_age_ms(time_us);
        let mut flight_cmd = self.ingress.active_command();
        if flight_cmd.setpoint.airspeed.is_none() {
            flight_cmd.setpoint.airspeed = self.endpoint.airspeed_target().map(MetersPerSecond);
        }
        let result = self.kernel.update(
            ChannelId(0),
            time_delta,
            &sensors,
            &flight_cmd,
            command_age_ms,
            &ActuatorState::default(),
            None,
        );
        self.last_effective_command = result.effective_command;
        self.board_hal.write(&result.actuator);

        let cmd = self.take_actuator_cmd(time_us);
        self.last_actuator_cmd = cmd;

        let time_boot_ms =
            u32::try_from(time_us.saturating_sub(anchor_us) / 1_000).unwrap_or(u32::MAX);
        self.emit_cycle_telemetry(time_us, time_boot_ms, &result.estimate, geomag);
        cmd
    }

    fn take_actuator_cmd(&mut self, timestamp_us: u64) -> DirectActuatorCmd {
        let armed = self.is_armed();
        match self.board_hal.actuator_mut().take_cmd() {
            Some(raw) => DirectActuatorCmd {
                timestamp_us,
                outputs: if armed {
                    raw.outputs
                } else {
                    [0.0; MAX_ACTUATOR_OUTPUTS]
                },
                count: raw.count,
                armed,
            },
            None => DirectActuatorCmd {
                timestamp_us,
                outputs: [0.0; MAX_ACTUATOR_OUTPUTS],
                count: 0,
                armed,
            },
        }
    }

    pub(super) fn handle_generation_transition(&mut self, generation: u32) {
        if let Some(prev_gen) = self.active_generation {
            if prev_gen != generation {
                log::info!("DirectXilBoard reset_generation {prev_gen} -> {generation}");
                if self.is_armed() {
                    self.kernel.terminate();
                }
                self.kernel.ground_reset();
                self.kernel.state.checks.pre_arm.update_throttle(true);
                self.clear_runtime_state();
            }
        }
        self.active_generation = Some(generation);
    }

    pub(super) fn clear_runtime_state(&mut self) {
        self.board_hal.disarm();
        let (imu, baro, mag, gnss) = self.board_hal.sensors_mut();
        imu.clear();
        baro.clear();
        mag.clear();
        gnss.clear();
        self.synthesizer.reset();
        self.endpoint.clear_for_reset();
        self.endpoint.set_system_status(MavState::Boot);
        self.ingress = DirectCommandIngress::default();
        self.sensor_cache = DirectSensorCache::default();
        self.last_effective_command = default_failsafe_command();
        self.last_actuator_cmd = DirectActuatorCmd::default();
        self.last_gnss_raw = None;
        self.boot_anchor_us = None;
        self.ekf_initialized = false;
        self.host_clock_us = None;
        self.seed_wait_since_us = None;
        self.model_declination_rad = self.origin_declination_rad;
        self.frame_map = None;
        self.origin_announced = false;
    }

    fn feed_and_read_sensors(&mut self, sensors: &DirectHostSensors) {
        self.sensor_cache.begin_step();
        if let Some(imu) = sensors.imu {
            self.board_hal.imu_mut().feed(imu.reading);
        }
        if let Some(baro) = sensors.baro {
            self.board_hal.baro_mut().feed(baro.reading);
        }
        if let Some(mag) = sensors.mag {
            self.board_hal.mag_mut().feed(mag.reading);
        }
        if let Some(gnss) = sensors.gnss {
            self.last_gnss_raw = Some(gnss.reading);
            let mapped = self.map_gnss(gnss.reading);
            self.board_hal.gnss_mut().feed(mapped);
        }

        // The HAL stamps a reading with the board clock. The measurement
        // time that the source reported replaces that stamp.
        if let Some(mut imu) = self.board_hal.read_imu() {
            stamp(&mut imu, sensors.imu.map(|s| s.timestamp_us));
            self.sensor_cache.imu = Some(imu);
        }
        if let Some(mut baro) = self.board_hal.read_baro() {
            stamp(&mut baro, sensors.baro.map(|s| s.timestamp_us));
            self.sensor_cache.record_baro(baro);
        }
        if let Some(mut mag) = self.board_hal.read_mag() {
            stamp(&mut mag, sensors.mag.map(|s| s.timestamp_us));
            self.sensor_cache.record_mag(mag);
        }
        if let Some(mut gnss) = self.board_hal.read_gnss() {
            stamp(&mut gnss, sensors.gnss.map(|s| s.timestamp_us));
            self.sensor_cache.record_gnss(gnss);
        }
        if let Some(airspeed) = sensors.airspeed {
            self.sensor_cache.record_airspeed(SensorReading {
                value: airspeed.reading.air_data(),
                valid: true,
                source_id: 0,
                timestamp: Timestamp {
                    ticks: airspeed.timestamp_us,
                    source: CoreTimeSource::Internal,
                },
                health: SensorHealth::Good,
            });
        }
    }

    /// Seed the estimator from the first attitude sources and the first GNSS fix.
    ///
    /// A fix gives the position and the velocity. The seed waits
    /// [`SEED_FIX_WAIT_US`] for one. Without a fix the seed does not
    /// claim that the vehicle is at rest: see [`Self::seed_without_fix`].
    fn maybe_initialize_ekf(&mut self, decl: f32, time_us: u64) {
        if self.ekf_initialized {
            return;
        }
        let (Some(imu), Some(mag)) = (self.sensor_cache.imu, self.sensor_cache.mag) else {
            return;
        };
        let fix = self
            .sensor_cache
            .gnss
            .filter(|gnss| gnss.valid && gnss.value.fix != GnssFix::None);
        let waited_us = time_us.saturating_sub(*self.seed_wait_since_us.get_or_insert(time_us));
        if fix.is_none() && waited_us < SEED_FIX_WAIT_US {
            return;
        }
        let [ax, ay, az] = imu.value.accel;
        let [mx, my, mz] = mag.value.field_ut;
        let init_quat = triad_init_quat([ax.0, ay.0, az.0], [mx.0, my.0, mz.0], decl);
        match fix {
            Some(gnss) => {
                let [north, east, down] = gnss.value.position_ned;
                let [north_rate, east_rate, down_rate] = gnss.value.velocity_ned;
                self.kernel.state.estimator.init(
                    Vector3::new(north, east, down),
                    Vector3::new(north_rate, east_rate, down_rate),
                    init_quat,
                );
            }
            // A vehicle with air-data aiding can be in flight without a
            // fix. Any other vehicle that has no fix is on the ground.
            None if self.kernel.pipeline().estimator.has_air_aiding() => {
                self.seed_without_fix(init_quat);
            }
            None => {
                let rest = Vector3::new(
                    MetersPerSecond(0.0),
                    MetersPerSecond(0.0),
                    MetersPerSecond(0.0),
                );
                let origin = Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0));
                self.kernel.state.estimator.init(origin, rest, init_quat);
            }
        }
        // A seed is in the configured frame.
        self.frame_map = None;
        self.ekf_initialized = true;
    }

    /// Seed the estimator when no fix came.
    ///
    /// The velocity is the true airspeed along the nose when a probe
    /// gives one, with the variance of an unknown wind; without a probe
    /// it is zero with the variance of an unknown flight speed. The
    /// horizontal position is the origin with the variance of an unknown
    /// position. The height is the pressure height above the pressure of
    /// the origin.
    fn seed_without_fix(&mut self, init_quat: aviate_core::math::Quaternion) {
        let true_airspeed = self
            .sensor_cache
            .airspeed
            .and_then(|reading| reading.value.air.pitot_static())
            .map(|probe| probe.true_mps);
        let (velocity, velocity_var, wind_var) = match true_airspeed {
            Some(speed) => (
                init_quat.rotate_vector(Vector3::new(speed, 0.0, 0.0)),
                SEED_AIR_VELOCITY_VAR_M2PS2,
                Some(SEED_WIND_VAR_M2PS2),
            ),
            None => (Vector3::new(0.0, 0.0, 0.0), SEED_SPEED_VAR_M2PS2, None),
        };
        let origin_height = RawBaroReading {
            pressure_pa: self.origin_pressure_pa,
            ..RawBaroReading::default()
        }
        .altitude_m();
        let height = self
            .sensor_cache
            .baro
            .and_then(|reading| reading.value.altitude)
            .map_or(0.0, |altitude| altitude.0 - origin_height);
        self.kernel.state.estimator.init_uncertain(
            Vector3::new(Meters(0.0), Meters(0.0), Meters(-height)),
            Vector3::new(
                MetersPerSecond(velocity.x),
                MetersPerSecond(velocity.y),
                MetersPerSecond(velocity.z),
            ),
            init_quat,
            &SeedUncertainty {
                horizontal_position: SEED_POSITION_VAR_M2,
                velocity: velocity_var,
                tilt: SEED_TILT_VAR_RAD2,
                heading: SEED_HEADING_VAR_RAD2,
                wind: wind_var,
            },
        );
    }
}

/// Replace the HAL stamp of `reading` with the reported measurement time.
fn stamp<T>(reading: &mut SensorReading<T>, measured_us: Option<u64>) {
    if let Some(ticks) = measured_us {
        reading.timestamp.ticks = ticks;
    }
}

pub(super) fn advance_board_hal_time(hal: &mut DirectBoardHal, now_us: u64) {
    let was_armed = hal.is_armed();
    let pending_cmd = hal.actuator_mut().take_cmd();
    let pending_status = hal.actuator_mut().read_status();
    let imu = core::mem::take(hal.imu_mut());
    let baro = core::mem::take(hal.baro_mut());
    let mag = core::mem::take(hal.mag_mut());
    let gnss = core::mem::take(hal.gnss_mut());
    let mut actuator = FakeActuator::new();
    if was_armed {
        actuator.arm();
    }
    if let Some(cmd) = pending_cmd {
        let _ = actuator.write(&cmd);
    }
    if let Some(status) = pending_status {
        actuator.feed_status(status);
    }
    *hal = BoardHal::new(imu, baro, mag, gnss, DirectTime::from_us(now_us), actuator);
}
