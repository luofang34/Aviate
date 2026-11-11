//! Closed-loop flight harness: a fixed-wing kernel on the direct board,
//! host-side sensor models, and the point-mass plant.

use aviate_airframe_fixed_wing::{c172, FixedWingAirframe};
use aviate_core::control::fixed_wing::{AirspeedSource, FixedWingController};
use aviate_core::control::{Command, CommandSource, ControlMode, Setpoint};
use aviate_core::ekf::Ekf;
use aviate_core::kernel::builder::AviateKernelBuilder;
use aviate_core::mixer::fixed_wing::FixedWingMixer;
use aviate_core::mixer::Sanitizer;
use aviate_core::time::{TimeSource, Timestamp};
use aviate_core::types::{Meters, MetersPerSecond};
use aviate_hal_io::SystemCommand;
use aviate_xil_direct::{
    DirectActuatorCmd, DirectBoardConfig, DirectHostSensors, DirectNoiseTier, DirectOrigin,
    DirectSensorRates, DirectSynthesizer, DirectXilBoard,
};
use world_core::{normal_gravity, DecimalYear, GeodeticPosition};

#[path = "fixed_wing.rs"]
mod fixed_wing_plant;
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
    /// The GNSS receiver supplies no sample.
    pub gnss_silent: bool,
    step: u64,
    sequence: u32,
    pub last: DirectActuatorCmd,
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
}

impl Flight {
    /// `airframe` in level cruise on `model`, with the given sensor rates.
    pub fn cruising(
        airframe: &FixedWingAirframe,
        model: PlantModel,
        noise: DirectNoiseTier,
        rates: DirectSensorRates,
    ) -> Self {
        let kernel = AviateKernelBuilder::new()
            .estimator(Ekf::default())
            .controller(airframe.controller().expect("valid description"))
            .mixer(FixedWingAirframe::mixer(ts))
            .sanitizer(Sanitizer)
            .config(airframe.kernel_config())
            .build()
            .expect("build fixed-wing kernel");
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
        let mut flight = Self {
            board: DirectXilBoard::new(kernel, config).expect("create board"),
            plant: FixedWingPlant::cruising(model, START_HEIGHT_M, gravity),
            sensors,
            gnss_silent: false,
            step: 0,
            sequence: 0,
            last: DirectActuatorCmd::default(),
        };
        flight.engage();
        flight
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
        let truth = self.plant.truth(self.step * DT_US, self.step);
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
            assert!(self.step < 1_000, "kernel not ready after 4 s");
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
        };
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
        }
        seen
    }

    pub fn airspeed_source(&self) -> AirspeedSource {
        self.board.kernel().state.controller.targets.airspeed_source
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
