//! Closed-loop flight harness: a fixed-wing kernel on the direct board,
//! host-side sensor models, and the point-mass plant.

use aviate_airframe_fixed_wing::{c172, FixedWingAirframe};
use aviate_core::control::fixed_wing::{AirspeedSource, FixedWingController};
use aviate_core::control::{Command, CommandSource, ControlMode, Setpoint};
use aviate_core::ekf::Estimator;
use aviate_core::mixer::fixed_wing::FixedWingMixer;
use aviate_core::state::StateEstimate;
use aviate_core::time::{TimeSource, Timestamp};
use aviate_core::types::{Meters, MetersPerSecond, Radians};
use aviate_hal_io::SystemCommand;
use aviate_xil_direct::{
    DirectActuatorCmd, DirectBoardConfig, DirectHostSensors, DirectNoiseTier, DirectOrigin,
    DirectSensorRates, DirectSynthesizer, DirectXilBoard,
};
use world_core::{normal_gravity, DecimalYear, GeodeticPosition};

#[path = "earth.rs"]
mod earth;
#[path = "fixed_wing.rs"]
mod fixed_wing_plant;
use earth::Earth;
pub use fixed_wing_plant::{FixedWingPlant, PlantModel};

const DT_US: u64 = 4_000;
pub const DT_S: f32 = 0.004;
/// Steps between setpoint refreshes: 100 ms, inside the 500 ms command timeout.
const SETPOINT_REFRESH_STEPS: u64 = 25;
const START_HEIGHT_M: f32 = 300.0;

type Board = DirectXilBoard<FixedWingController, FixedWingMixer>;

fn ts() -> Timestamp {
    Timestamp {
        ticks: 0,
        source: TimeSource::Internal,
    }
}

/// One closed-loop flight: board, plant, sensor models and clock.
///
/// The sensor models run on the host side of the board, so a test can
/// take one sensor away as a failure does.
pub struct Flight {
    pub board: Board,
    pub plant: FixedWingPlant,
    sensors: DirectSynthesizer,
    /// The curved Earth that the host models report against.
    earth: Earth,
    /// The GNSS receiver supplies no sample.
    pub gnss_silent: bool,
    /// Constant bias of the gyroscope in body axes \[rad/s\].
    pub gyro_bias_rps: [f32; 3],
    /// Constant bias of the accelerometer in body axes \[m/s²\].
    pub accel_bias_mps2: [f32; 3],
    /// Pressure-sensor errors of the pitot-static probe, when modelled.
    pub probe_errors: Option<ProbeErrors>,
    step: u64,
    sequence: u32,
    pub last: DirectActuatorCmd,
}

/// Running mean and variance of one lane.
#[derive(Default)]
struct Spread {
    count: f32,
    sum: f32,
    sum_squares: f32,
}

impl Spread {
    fn add(&mut self, value: f32) {
        self.count += 1.0;
        self.sum += value;
        self.sum_squares += value * value;
    }

    fn deviation(&self) -> f32 {
        let mean = self.sum / self.count;
        (self.sum_squares / self.count - mean * mean)
            .max(0.0)
            .sqrt()
    }
}

/// Pressure-sensor errors of a pitot-static probe: white noise on each
/// sample and a slow drift, for the pitot port and for the static port.
pub struct ProbeErrors {
    state: u64,
}

impl ProbeErrors {
    /// White noise, one standard deviation, on each port \[Pa\].
    const WHITE_PA: f32 = 3.0;
    /// Amplitude of the slow drift of the pitot port \[Pa\].
    const PITOT_DRIFT_PA: f32 = 10.0;
    /// Amplitude of the slow drift of the static port \[Pa\].
    const STATIC_DRIFT_PA: f32 = 20.0;
    /// Period of the slow drift \[s\].
    const DRIFT_PERIOD_S: f32 = 120.0;

    pub fn new(seed: u64) -> Self {
        Self { state: seed | 1 }
    }

    /// One draw from an approximately normal distribution, unit variance.
    fn normal(&mut self) -> f32 {
        let mut sum = 0.0_f32;
        for _ in 0..12 {
            self.state ^= self.state << 13;
            self.state ^= self.state >> 7;
            self.state ^= self.state << 17;
            sum += (self.state >> 40) as f32 / (1_u64 << 24) as f32;
        }
        sum - 6.0
    }

    fn drift(time_s: f32, amplitude: f32) -> f32 {
        amplitude * (core::f32::consts::TAU * time_s / Self::DRIFT_PERIOD_S).sin()
    }

    fn pitot_pa(&mut self, time_s: f32) -> f32 {
        Self::WHITE_PA * self.normal() + Self::drift(time_s, Self::PITOT_DRIFT_PA)
    }

    fn static_pa(&mut self, time_s: f32) -> f32 {
        Self::WHITE_PA * self.normal() + Self::drift(time_s + 30.0, Self::STATIC_DRIFT_PA)
    }
}

/// What one flight segment saw.
#[derive(Debug, Clone, Copy)]
pub struct Extremes {
    pub min_indicated: f32,
    pub max_indicated: f32,
    pub max_bank: f32,
    pub max_pitch: f32,
    pub min_pitch: f32,
    pub max_load: f32,
    pub min_height: f32,
    pub max_height: f32,
    pub max_climb: f32,
    pub max_sink: f32,
    pub max_sideslip: f32,
    /// Mean rudder lane over the segment.
    pub mean_rudder: f32,
    /// Standard deviation of the throttle lane over the segment.
    pub throttle_deviation: f32,
    /// Standard deviation of the elevator lane over the segment.
    pub elevator_deviation: f32,
    /// Largest error of the estimated roll, pitch and heading \[rad\].
    pub max_attitude_error: [f32; 3],
}

/// Sensor faults that are present from the first step of a flight.
#[derive(Clone, Copy, Default)]
pub struct Faults {
    /// The GNSS receiver never supplies a sample.
    pub no_gnss: bool,
    /// Constant bias of the gyroscope in body axes \[rad/s\].
    pub gyro_bias_rps: [f32; 3],
    /// Constant bias of the accelerometer in body axes \[m/s²\].
    pub accel_bias_mps2: [f32; 3],
    /// Wind over the ground, north and east \[m/s\].
    pub wind_ne: [f32; 2],
}

impl Flight {
    /// `airframe` in level cruise on `model`, with the given sensor rates.
    pub fn cruising(
        airframe: &FixedWingAirframe,
        model: PlantModel,
        noise: DirectNoiseTier,
        rates: DirectSensorRates,
    ) -> Self {
        Self::cruising_with(airframe, model, noise, rates, Faults::default())
    }

    /// As [`Self::cruising`], before the kernel is ready or armed, so the
    /// test owns the arming path.
    pub fn cruising_disarmed(
        airframe: &FixedWingAirframe,
        model: PlantModel,
        noise: DirectNoiseTier,
        rates: DirectSensorRates,
    ) -> Self {
        Self::cruising_disarmed_with(airframe, model, noise, rates, Faults::default())
    }

    /// As [`Self::cruising`], with sensor faults from the first step.
    pub fn cruising_with(
        airframe: &FixedWingAirframe,
        model: PlantModel,
        noise: DirectNoiseTier,
        rates: DirectSensorRates,
        faults: Faults,
    ) -> Self {
        let mut flight = Self::cruising_disarmed_with(airframe, model, noise, rates, faults);
        flight.engage();
        flight
    }

    /// As [`Self::cruising_with`], before the kernel is ready or armed.
    pub fn cruising_disarmed_with(
        airframe: &FixedWingAirframe,
        model: PlantModel,
        noise: DirectNoiseTier,
        rates: DirectSensorRates,
        faults: Faults,
    ) -> Self {
        let kernel = airframe.kernel(ts).expect("build fixed-wing kernel");
        let config = DirectBoardConfig {
            origin: GeodeticPosition::wgs84(39.1582, -75.5244, 15.0),
            epoch: DecimalYear(2026.5),
            noise,
            noise_seed: 2024,
            sys_id: 1,
            comp_id: 1,
        };
        let gravity = normal_gravity(config.origin).expect("gravity") as f32;
        let sensors = DirectSynthesizer::new(
            DirectOrigin::new(config.origin, config.epoch),
            rates,
            noise,
            config.noise_seed,
        )
        .expect("host sensor models");
        let mut plant = FixedWingPlant::cruising(model, START_HEIGHT_M, gravity);
        plant.wind_ne = faults.wind_ne;
        Self {
            board: DirectXilBoard::new(kernel, config).expect("create board"),
            plant,
            sensors,
            earth: Earth::new(config.origin),
            gnss_silent: faults.no_gnss,
            gyro_bias_rps: faults.gyro_bias_rps,
            accel_bias_mps2: faults.accel_bias_mps2,
            probe_errors: None,
            step: 0,
            sequence: 0,
            last: DirectActuatorCmd::default(),
        }
    }

    /// A C172 with an airspeed probe at 20 Hz.
    pub fn c172_with_probe(noise: DirectNoiseTier) -> Self {
        Self::cruising(
            &c172(),
            PlantModel::C172,
            noise,
            DirectSensorRates::default().with_airspeed(50_000),
        )
    }

    pub fn advance(&mut self) {
        let truth = self.curved_truth();
        let mut sensors = match self.sensors.synthesize(&truth) {
            Some(sample) => sample.host_sensors(None),
            None => DirectHostSensors {
                time_us: truth.time_us,
                ..DirectHostSensors::default()
            },
        };
        if self.gnss_silent {
            sensors.gnss = None;
        }
        self.add_transport_rate(&mut sensors);
        if let Some(imu) = sensors.imu.as_mut() {
            for (rate, bias) in imu.reading.gyro.iter_mut().zip(self.gyro_bias_rps) {
                *rate += bias;
            }
            for (force, bias) in imu.reading.accel.iter_mut().zip(self.accel_bias_mps2) {
                *force += bias;
            }
        }
        // The host models know only the velocity over the ground; the
        // probe measures the velocity through the air.
        if let Some(airspeed) = sensors.airspeed.as_mut() {
            let squared = |v: [f32; 3]| v.iter().map(|c| c * c).sum::<f32>();
            let ground = squared(self.plant.ground_velocity_ned());
            if ground > 1.0 {
                airspeed.reading.differential_pressure_pa *=
                    squared(self.plant.velocity_ned()) / ground;
            }
        }
        if let (Some(errors), Some(airspeed)) =
            (self.probe_errors.as_mut(), sensors.airspeed.as_mut())
        {
            let time_s = truth.time_us as f32 * 1.0e-6;
            airspeed.reading.differential_pressure_pa += errors.pitot_pa(time_s);
            if let Some(static_pa) = airspeed.reading.static_pressure_pa.as_mut() {
                *static_pa += errors.static_pa(time_s);
            }
        }
        self.last = self.board.step_sensors(&sensors);
        self.plant.step(&self.last, DT_S);
        self.step += 1;
    }

    /// Fly `seconds` without sending a command.
    pub fn coast(&mut self, seconds: f32) {
        for _ in 0..(seconds / DT_S).round() as u64 {
            self.advance();
        }
    }

    /// Fly hands-off until the kernel is ready, then arm it.
    fn engage(&mut self) {
        while !self.board.is_ready() {
            assert!(
                self.step < 1_500,
                "kernel not ready after 6 s: {:?}",
                self.board.kernel().state.checks.pre_arm.missing()
            );
            self.advance();
        }
        self.board.send_command(SystemCommand::Arm);
        self.advance();
        assert!(self.board.is_armed(), "arm refused");
    }

    fn send(&mut self, mode: ControlMode, setpoint: Setpoint) {
        self.sequence += 1;
        self.board
            .send_command(SystemCommand::FlightControl(Command {
                mode,
                setpoint,
                config_mode_request: None,
                sensor_overrides: None,
                sequence: self.sequence,
                source: CommandSource::Autopilot,
            }));
    }

    /// Fly `seconds` under the setpoint that `command` gives for the present state.
    pub fn fly(
        &mut self,
        seconds: f32,
        mode: ControlMode,
        mut command: impl FnMut(&FixedWingPlant) -> Setpoint,
    ) -> Extremes {
        let mut seen = Extremes {
            min_indicated: f32::MAX,
            max_indicated: f32::MIN,
            max_bank: 0.0,
            max_pitch: f32::MIN,
            min_pitch: f32::MAX,
            max_load: f32::MIN,
            min_height: f32::MAX,
            max_height: f32::MIN,
            max_climb: f32::MIN,
            max_sink: f32::MIN,
            max_sideslip: 0.0,
            mean_rudder: 0.5,
            throttle_deviation: 0.0,
            elevator_deviation: 0.0,
            max_attitude_error: [0.0; 3],
        };
        let mut rudder_sum = 0.0_f32;
        let (mut throttle, mut elevator) = (Spread::default(), Spread::default());
        let steps = (seconds / DT_S).round() as u64;
        for local in 0..steps {
            if local % SETPOINT_REFRESH_STEPS == 0 {
                let setpoint = command(&self.plant);
                self.send(mode, setpoint);
            }
            self.advance();
            let climb = -self.plant.velocity_ned()[2];
            seen.min_indicated = seen.min_indicated.min(self.plant.indicated_airspeed());
            seen.max_indicated = seen.max_indicated.max(self.plant.indicated_airspeed());
            seen.max_bank = seen.max_bank.max(self.plant.roll.abs());
            seen.max_pitch = seen.max_pitch.max(self.plant.pitch);
            seen.min_pitch = seen.min_pitch.min(self.plant.pitch);
            seen.max_load = seen.max_load.max(self.plant.load_factor);
            seen.min_height = seen.min_height.min(self.plant.height());
            seen.max_height = seen.max_height.max(self.plant.height());
            seen.max_climb = seen.max_climb.max(climb);
            seen.max_sink = seen.max_sink.max(-climb);
            seen.max_sideslip = seen.max_sideslip.max(self.plant.sideslip().abs());
            for (worst, error) in seen
                .max_attitude_error
                .iter_mut()
                .zip(self.attitude_error())
            {
                *worst = worst.max(error.abs());
            }
            rudder_sum += self.last.outputs[3];
            throttle.add(self.last.outputs[0]);
            elevator.add(self.last.outputs[2]);
        }
        if steps > 0 {
            seen.mean_rudder = rudder_sum / steps as f32;
            seen.throttle_deviation = throttle.deviation();
            seen.elevator_deviation = elevator.deviation();
        }
        seen
    }

    /// The plant's truth on the curved Earth for the host models: the
    /// position and velocity in the configured plane, which the models
    /// read as plane coordinates (GNSS, and the barometer through the
    /// height above the ellipsoid). Attitude and body kinematics stay
    /// relative to the local level, as an IMU and a magnetometer see them.
    fn curved_truth(&self) -> aviate_xil_direct::DirectTruthSample {
        let plant = &self.plant;
        let mut truth = plant.truth(self.step * DT_US, self.step);
        let at = self
            .earth
            .position(plant.pos_ned[0], plant.pos_ned[1], plant.height());
        truth.position_ned_m = self.earth.plane_position(at);
        if let Some(measured) = truth.measured.as_mut() {
            measured.velocity_ned_mps = self.earth.plane_vector(at, plant.ground_velocity_ned());
        }
        truth
    }

    /// The gyroscope also measures the turn of the local level as the
    /// vehicle moves over the ellipsoid (the transport rate).
    fn add_transport_rate(&self, sensors: &mut DirectHostSensors) {
        let plant = &self.plant;
        let at = self
            .earth
            .position(plant.pos_ned[0], plant.pos_ned[1], plant.height());
        if let Some(imu) = sensors.imu.as_mut() {
            let transport = self.earth.transport_rate_body(
                at,
                plant.ground_velocity_ned(),
                plant.attitude_wxyz(),
            );
            for (rate, extra) in imu.reading.gyro.iter_mut().zip(transport) {
                *rate += extra;
            }
        }
    }

    /// The true position in the frame of the estimate.
    pub fn truth_in_estimate_frame(&self) -> [f32; 3] {
        let plant = &self.plant;
        let at = self
            .earth
            .position(plant.pos_ned[0], plant.pos_ned[1], plant.height());
        let frame = self.board.kernel().state.estimator.frame;
        let origin = frame.origin.map_or(
            (
                self.board.config().origin.latitude_deg,
                self.board.config().origin.longitude_deg,
            ),
            |o| (o.latitude_deg, o.longitude_deg),
        );
        self.earth.position_in_frame(at, origin)
    }

    /// The estimate that the kernel flies on.
    pub fn estimate(&self) -> StateEstimate {
        let kernel = self.board.kernel();
        kernel
            .pipeline()
            .estimator
            .estimate(&kernel.state.estimator)
    }

    /// Estimated minus true roll, pitch and heading \[rad\].
    pub fn attitude_error(&self) -> [f32; 3] {
        let (roll, pitch, yaw) = self.estimate().attitude.to_euler();
        [
            wrap(roll - self.plant.roll),
            wrap(pitch - self.plant.pitch),
            wrap(yaw - self.plant.heading),
        ]
    }

    pub fn airspeed_source(&self) -> AirspeedSource {
        self.board.kernel().state.controller.targets.airspeed_source
    }
}

/// Heading, airspeed and height: the command of a flight without a
/// position fix.
pub fn heading_speed_height(heading_rad: f32, airspeed: f32, height: f32) -> Setpoint {
    Setpoint {
        heading: Some(Radians(heading_rad)),
        altitude: Some(Meters(height)),
        velocity: Some([
            MetersPerSecond(airspeed),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ]),
        ..Setpoint::default()
    }
}

/// Course, airspeed and height as a velocity setpoint.
pub fn course_speed_height(course_rad: f32, airspeed: f32, height: f32) -> Setpoint {
    Setpoint {
        velocity: Some([
            MetersPerSecond(airspeed * course_rad.cos()),
            MetersPerSecond(airspeed * course_rad.sin()),
            MetersPerSecond(0.0),
        ]),
        altitude: Some(Meters(height)),
        ..Setpoint::default()
    }
}

pub fn wrap(angle: f32) -> f32 {
    angle.sin().atan2(angle.cos())
}
