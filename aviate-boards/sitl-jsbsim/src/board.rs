//! The JSBSim board: one kernel cycle per truth state, one reply per
//! truth state.

use std::io;
use std::time::Duration;

use aviate_backend_mavlink_hil::{ActuatorSendReceipt, HilBackend, HilBackendConfig};
use aviate_core::mixer::ActuatorCmd;
use aviate_core::DefaultAviateKernel;
use aviate_hal_io::{BoardHal, FakeActuator, FakeBaro, FakeGnss, FakeImu, FakeMag};
use aviate_hal_xil::synthesis::{
    apply_packet_noise, NoiseRng, NoiseTier, StepSynthesizer, SynthesisError,
};
use aviate_hal_xil::{SimActuatorCmd, SimulatorLifecycle, SitlIO, VehicleState};
use aviate_runtime::telemetry::GeodeticOrigin;
use aviate_runtime::SitlRunner;
use log::warn;
use world_core::{LocalTangentPlane, PositionError};

use crate::config::JsbsimConfig;
use crate::truth::{truth_sample, vehicle_state, GenerationTracker};

mod session;

/// The airframe's map from the mixer's command to the simulator's
/// per-engine controls, given whether the vehicle is armed. It owns the
/// engine order and the actuator curve the simulated engines expect.
pub type ActuatorMap = fn(&ActuatorCmd, bool) -> SimActuatorCmd;

/// Why the board could not be built.
#[derive(Debug, thiserror::Error)]
pub enum JsbsimError {
    /// The runner's GCS link could not bind its port.
    #[error("binding the XIL GCS port: {0}")]
    Bind(#[source] io::Error),
    /// The world origin is not a usable position.
    #[error("world origin: {0}")]
    Origin(#[from] PositionError),
    /// A world model refused the origin or the date.
    #[error(transparent)]
    Synthesis(#[from] SynthesisError),
}

/// What one truth state did.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TruthStep {
    /// The state's simulation time, which the reply echoes.
    pub time_us: u64,
    /// The simulated world the state belongs to.
    pub generation: u32,
    /// The simulation time the kernel cycle covered, or `None` when the
    /// state started no new step (the first state of a world, or a
    /// repeated one) and the previous command was sent again.
    pub kernel_delta_us: Option<u64>,
    /// The reply as sent, or `None` when the link refused it.
    pub reply: Option<ActuatorSendReceipt>,
}

/// JSBSim SITL board, generic over the injected controller and mixer.
pub struct JsbsimBoard<C, M>
where
    C: aviate_core::control::VehicleController,
    M: aviate_core::mixer::Mixer,
{
    backend: HilBackend,
    runner: SitlRunner<C, M>,
    plane: LocalTangentPlane,
    synthesizer: StepSynthesizer,
    generations: GenerationTracker,
    generation: Option<u32>,
    actuator_map: ActuatorMap,
    noise: NoiseTier,
    noise_rng: NoiseRng,
    /// The command last sent, sent again for a state that starts no new
    /// step so the sidecar's lockstep never waits.
    last_reply: SimActuatorCmd,
    /// Where the session is in its lifecycle.
    lifecycle: SimulatorLifecycle,
    /// The vehicle the latest truth state states.
    vehicle: VehicleState,
    /// The latest truth state's step index within its generation.
    step_index: u64,
    /// The latest truth state's simulation time.
    time_us: u64,
}

impl<C, M> JsbsimBoard<C, M>
where
    C: aviate_core::control::VehicleController,
    M: aviate_core::mixer::Mixer,
{
    /// A board around an app-built kernel, dialing the sidecar.
    ///
    /// # Errors
    ///
    /// [`JsbsimError`] when the GCS port cannot bind, or the origin or
    /// date is outside the world models.
    pub fn new(
        kernel: DefaultAviateKernel<C, M>,
        config: JsbsimConfig,
        actuator_map: ActuatorMap,
    ) -> Result<Self, JsbsimError> {
        let plane = config.origin.plane()?;
        let synthesizer = StepSynthesizer::new(config.origin, config.epoch)?;
        let transport = SitlIO::new(config.xil).map_err(JsbsimError::Bind)?;
        let board_hal = BoardHal::new(
            FakeImu::new(),
            FakeBaro::new(),
            FakeMag::new(),
            FakeGnss::new(),
            aviate_runtime::sim::SitlTime::new(),
            FakeActuator::new(),
        );
        let mut runner = SitlRunner::new(transport, board_hal, kernel);
        let origin = config.origin.position;
        runner.set_geodetic_origin(GeodeticOrigin {
            latitude_deg: origin.latitude_deg,
            longitude_deg: origin.longitude_deg,
            altitude_msl_m: origin.ellipsoidal_height_m as f32,
        });
        let backend = HilBackend::connect_tcp(HilBackendConfig {
            simulator_addr: config.sidecar_addr,
            sys_id: config.sys_id,
            comp_id: config.comp_id,
            ..HilBackendConfig::default()
        });
        Ok(Self {
            backend,
            runner,
            plane,
            synthesizer,
            generations: GenerationTracker::default(),
            generation: None,
            actuator_map,
            noise: config.noise,
            noise_rng: NoiseRng::new(config.noise_seed),
            last_reply: SimActuatorCmd::default(),
            lifecycle: SimulatorLifecycle::Converging,
            vehicle: VehicleState::default(),
            step_index: 0,
            time_us: 0,
        })
    }

    /// Take the newest truth state, run the kernel cycle it starts, and
    /// answer it. `None` when no state has arrived.
    ///
    /// While the session is stopped, or waits for a new world, the state
    /// is answered idle and runs no kernel cycle.
    pub fn step(&mut self) -> Option<TruthStep> {
        let state = self.backend.poll_truth()?;
        let (generation, step) = self
            .generations
            .observe(self.backend.sessions(), state.time_usec);
        if self.generation != Some(generation) {
            // A new world starts disarmed; the old world's command must
            // not drive the new vehicle.
            self.last_reply = SimActuatorCmd::default();
            self.enter_new_world();
            self.generation = Some(generation);
        }
        self.step_index = step;
        self.time_us = state.time_usec;
        let truth = truth_sample(&state, &self.plane, generation, step);
        if let Some(truth) = truth {
            self.vehicle = vehicle_state(&state, &truth);
        }
        let running = !matches!(
            self.lifecycle,
            SimulatorLifecycle::Stopped | SimulatorLifecycle::Resetting
        );
        let sample = truth
            .filter(|_| running)
            .and_then(|truth| self.synthesizer.next(&truth));
        let kernel_delta_us = sample.map(|mut sample| {
            apply_packet_noise(&mut sample.packet, self.noise, &mut self.noise_rng);
            self.runner
                .transport_mut()
                .feed_sensor_packet(&sample.packet);
            let command = self.runner.step_sim(sample.delta_us);
            let armed = self.is_armed();
            self.last_reply = (self.actuator_map)(&command, armed);
            sample.delta_us
        });
        if !running {
            self.last_reply = SimActuatorCmd::default();
        }
        self.observe_health();
        let reply = match self
            .backend
            .send_actuators_at(&self.last_reply, state.time_usec)
        {
            Ok(receipt) => Some(receipt),
            Err(error) => {
                warn!("JSBSim reply at {} us not sent: {error}", state.time_usec);
                None
            }
        };
        Some(TruthStep {
            time_us: state.time_usec,
            generation,
            kernel_delta_us,
            reply,
        })
    }

    /// Wait until a truth state is readable or `timeout` passes.
    pub fn wait_for_truth(&mut self, timeout: Duration) -> bool {
        self.backend.wait_readable(timeout)
    }

    /// Whether the sidecar link is up.
    #[must_use]
    pub fn connected(&self) -> bool {
        self.backend.connected()
    }

    /// Whether the kernel is ready to arm.
    #[must_use]
    pub fn is_ready(&self) -> bool {
        self.runner.kernel.is_ready()
    }

    /// Whether the kernel is armed.
    #[must_use]
    pub fn is_armed(&self) -> bool {
        self.runner.kernel.state.init_state == aviate_core::InitState::Armed
    }

    /// The kernel.
    #[must_use]
    pub fn kernel(&self) -> &DefaultAviateKernel<C, M> {
        &self.runner.kernel
    }

    /// The kernel, mutably.
    pub fn kernel_mut(&mut self) -> &mut DefaultAviateKernel<C, M> {
        &mut self.runner.kernel
    }

    /// Start the configured telemetry stream at `loop_hz`.
    pub fn init_telemetry(&mut self, cfg: &aviate_config::AppConfig, loop_hz: u32) {
        self.runner.init_telemetry(cfg, loop_hz);
    }

    /// Whether the telemetry stream is enabled.
    #[must_use]
    pub fn telemetry_enabled(&self) -> bool {
        self.runner.telemetry_enabled()
    }
}
