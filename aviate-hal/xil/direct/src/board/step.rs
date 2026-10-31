//! Per-tick stepping, sensor feeding, command dispatch, and reset handling for [`super::DirectXilBoard`].

use aviate_core::hal::{ActuatorHal, SensorHal};
use aviate_core::math::Vector3;
use aviate_core::mixer::ActuatorState;
use aviate_core::sensor::{GeomagneticReference, GeomagneticSource};
use aviate_core::time::{TimeDelta, TimeSource as CoreTimeSource, Timestamp};
use aviate_core::types::{Meters, MetersPerSecond, Radians, Seconds};
use aviate_core::ChannelId;
use aviate_hal_io::{
    ActuatorDriver, BoardHal, CommandOutcome, FakeActuator, SystemCommand, MAX_ACTUATOR_OUTPUTS,
};
use aviate_link::mavlink::MavState;
use aviate_link::telemetry::GeodeticOrigin;

use crate::link::DirectTelemetryFrameInput;
use crate::synthesis::{DirectStepSample, DirectTruthSample};

use super::ingress::{
    default_failsafe_command, init_state_to_mav_state, DirectCommandIngress, DirectSensorCache,
};
use super::triad::triad_init_quat;
use super::{DirectActuatorCmd, DirectBoardHal, DirectTime, DirectXilBoard};

impl<C, M> DirectXilBoard<C, M>
where
    C: aviate_core::control::VehicleController,
    M: aviate_core::mixer::Mixer,
{
    /// Advance the flight controller by one ground-truth sample and return actuator outputs.
    pub fn step_truth(&mut self, truth: &DirectTruthSample) -> DirectActuatorCmd {
        self.handle_generation_transition(truth.reset_generation);
        let anchor_us = *self.boot_anchor_us.get_or_insert(truth.time_us);
        advance_board_hal_time(&mut self.board_hal, truth.time_us);

        let Some(sample) = self.synthesizer.synthesize(truth) else {
            self.drain_inbound_commands(truth.time_us);
            self.last_actuator_cmd.timestamp_us = truth.time_us;
            self.last_actuator_cmd.armed = self.is_armed();
            if !self.is_armed() {
                self.last_actuator_cmd.outputs = [0.0; MAX_ACTUATOR_OUTPUTS];
            }
            return self.last_actuator_cmd;
        };

        self.feed_and_read_sensors(&sample);
        self.maybe_initialize_ekf();
        let geomag = self.current_geomagnetic_reference();
        let sensors = self.sensor_cache.to_sensor_set(Some(geomag));

        if !self.kernel.is_ready() {
            let ts = Timestamp {
                ticks: truth.time_us,
                source: CoreTimeSource::Internal,
            };
            let prev = self.kernel.state.init_state;
            let _ = self.kernel.init_step(&sensors, ts);
            if self.kernel.state.init_state != prev {
                self.endpoint
                    .set_system_status(init_state_to_mav_state(self.kernel.state.init_state));
            }
        }

        self.drain_inbound_commands(truth.time_us);

        let time_delta = TimeDelta {
            dt_sec: Seconds(((sample.delta_us as f32) * 1.0e-6).clamp(0.0001, 0.1)),
            tick_delta: sample.delta_us,
        };
        let command_age_ms = self.ingress.setpoint_age_ms(truth.time_us);
        let flight_cmd = self.ingress.active_command();
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

        let cmd = self.take_actuator_cmd(truth.time_us);
        self.last_actuator_cmd = cmd;

        let time_boot_ms =
            u32::try_from(truth.time_us.saturating_sub(anchor_us) / 1_000).unwrap_or(u32::MAX);
        self.emit_cycle_telemetry(truth.time_us, time_boot_ms, &result.estimate, geomag);
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

    fn handle_generation_transition(&mut self, generation: u32) {
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
    }

    fn feed_and_read_sensors(&mut self, sample: &DirectStepSample) {
        self.sensor_cache.begin_step();
        self.board_hal.imu_mut().feed(sample.imu);
        if let Some(baro) = sample.baro {
            self.board_hal.baro_mut().feed(baro);
        }
        if let Some(mag) = sample.mag {
            self.board_hal.mag_mut().feed(mag);
        }
        if let Some(gnss) = sample.gnss {
            self.last_gnss_raw = Some(gnss);
            self.board_hal.gnss_mut().feed(gnss);
        }

        if let Some(imu) = self.board_hal.read_imu() {
            self.sensor_cache.imu = Some(imu);
        }
        if let Some(baro) = self.board_hal.read_baro() {
            self.sensor_cache.record_baro(baro);
        }
        if let Some(mag) = self.board_hal.read_mag() {
            self.sensor_cache.record_mag(mag);
        }
        if let Some(gnss) = self.board_hal.read_gnss() {
            self.sensor_cache.record_gnss(gnss);
        }
    }

    fn maybe_initialize_ekf(&mut self) {
        if self.ekf_initialized {
            return;
        }
        let (Some(imu), Some(mag)) = (self.sensor_cache.imu, self.sensor_cache.mag) else {
            return;
        };
        let decl = self.synthesizer.environment().declination_rad();
        let init_quat = triad_init_quat(
            [
                imu.value.accel[0].0,
                imu.value.accel[1].0,
                imu.value.accel[2].0,
            ],
            [
                mag.value.field_ut[0].0,
                mag.value.field_ut[1].0,
                mag.value.field_ut[2].0,
            ],
            decl,
        );
        let (pos, vel) = match self.sensor_cache.gnss {
            Some(gnss) => (
                Vector3::new(
                    gnss.value.position_ned[0],
                    gnss.value.position_ned[1],
                    gnss.value.position_ned[2],
                ),
                Vector3::new(
                    gnss.value.velocity_ned[0],
                    gnss.value.velocity_ned[1],
                    gnss.value.velocity_ned[2],
                ),
            ),
            None => (
                Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
                Vector3::new(
                    MetersPerSecond(0.0),
                    MetersPerSecond(0.0),
                    MetersPerSecond(0.0),
                ),
            ),
        };
        self.kernel.state.estimator.init(pos, vel, init_quat);
        self.ekf_initialized = true;
    }

    fn drain_inbound_commands(&mut self, now_us: u64) {
        while let Some(sys_cmd) = self.endpoint.poll_command() {
            if let SystemCommand::FlightControl(ref flight) = sys_cmd {
                self.kernel.state.checks.pre_arm.update_throttle(
                    flight.setpoint.collective_thrust
                        < aviate_core::kernel_types::THROTTLE_LOW_MAX_COLLECTIVE,
                );
            }
            if let Some(outcome) = self.enact_discrete(&sys_cmd) {
                self.endpoint.report_outcome(&sys_cmd, outcome);
            }
            self.ingress.receive(&sys_cmd, now_us);
        }
    }

    fn enact_discrete(&mut self, sys_cmd: &SystemCommand) -> Option<CommandOutcome> {
        match sys_cmd {
            SystemCommand::Arm => match self.kernel.arm() {
                Ok(()) => {
                    self.board_hal.arm();
                    self.endpoint.set_armed(true);
                    self.endpoint.set_system_status(MavState::Active);
                    Some(CommandOutcome::Accepted)
                }
                Err(error) => {
                    let missing = self.kernel.state.checks.pre_arm.missing();
                    Some(CommandOutcome::ArmRejected { error, missing })
                }
            },
            SystemCommand::Disarm => match self.kernel.disarm() {
                Ok(()) => {
                    self.board_hal.disarm();
                    self.endpoint.set_armed(false);
                    self.endpoint
                        .set_system_status(init_state_to_mav_state(self.kernel.state.init_state));
                    Some(CommandOutcome::Accepted)
                }
                Err(error) => Some(CommandOutcome::DisarmRejected(error)),
            },
            SystemCommand::EmergencyTerminate => {
                self.kernel.terminate();
                self.board_hal.disarm();
                self.endpoint.set_armed(false);
                self.endpoint.set_system_status(MavState::Critical);
                Some(CommandOutcome::Accepted)
            }
            SystemCommand::FlightControl(_) => None,
        }
    }

    fn current_geomagnetic_reference(&self) -> GeomagneticReference {
        GeomagneticReference {
            declination: Radians(self.synthesizer.environment().declination_rad()),
            source: GeomagneticSource::Model,
        }
    }

    fn emit_cycle_telemetry(
        &mut self,
        sim_time_us: u64,
        time_boot_ms: u32,
        estimate: &aviate_core::state::StateEstimate,
        geomag: GeomagneticReference,
    ) {
        let origin = GeodeticOrigin {
            latitude_deg: self.config.origin.latitude_deg,
            longitude_deg: self.config.origin.longitude_deg,
            altitude_msl_m: self.config.origin.ellipsoidal_height_m as f32,
        };
        let input = DirectTelemetryFrameInput {
            sim_time_us,
            time_boot_ms,
            faults: self.kernel.state.faults,
            state: estimate,
            baro_pressure_pa: self.sensor_cache.baro_pressure_pa,
            baro_temperature_c: self.sensor_cache.baro_temperature_c,
            geomagnetic: Some(geomag),
            geodetic_origin: origin,
            gnss_fix: self.last_gnss_raw.as_ref(),
        };
        self.endpoint.emit_telemetry(&input);
    }
}

fn advance_board_hal_time(hal: &mut DirectBoardHal, now_us: u64) {
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
