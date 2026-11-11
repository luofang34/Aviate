//! In-process `no_std` XIL board driving [`aviate_core::DefaultAviateKernel`].

mod commands;
mod frame;
mod host;
mod ingress;
mod step;
mod triad;

use aviate_core::control::Command;
use aviate_core::{DefaultAviateKernel, InitState};
use aviate_hal_io::{
    BoardHal, FakeActuator, FakeBaro, FakeGnss, FakeImu, FakeMag, RawGnssReading, SystemCommand,
    TimeHal, TimeSource, MAX_ACTUATOR_OUTPUTS,
};
use world_core::{DecimalYear, GeodeticPosition};

use crate::link::{
    DirectMavlinkEndpoint, DirectTelemetryRates, DEFAULT_RX_RING_BYTES, DEFAULT_TX_RING_BYTES,
};
use crate::synthesis::{
    DirectNoiseTier, DirectOrigin, DirectSensorRates, DirectSynthesisError, DirectSynthesizer,
};

pub use frame::{DirectNavFrame, DEFAULT_REBASE_RADIUS_M};
pub use ingress::DirectCommandRefusal;
use ingress::{default_failsafe_command, DirectCommandIngress, DirectSensorCache};

/// Simulation clock in microseconds implementing [`TimeSource`] and [`TimeHal`].
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct DirectTime {
    now_us: u64,
}

impl DirectTime {
    /// Construct a [`DirectTime`] starting at `0` microseconds.
    #[must_use]
    pub const fn new() -> Self {
        Self { now_us: 0 }
    }

    /// Construct a [`DirectTime`] stamped at `now_us` microseconds.
    #[must_use]
    pub const fn from_us(now_us: u64) -> Self {
        Self { now_us }
    }

    /// Set the simulation timestamp in microseconds.
    pub fn set_us(&mut self, now_us: u64) {
        self.now_us = now_us;
    }
}

impl TimeSource for DirectTime {
    fn now_us(&self) -> u64 {
        self.now_us
    }
}

impl TimeHal for DirectTime {
    fn now_us(&mut self) -> u64 {
        self.now_us
    }

    fn sleep_until_us(&mut self, target_us: u64) {
        if target_us > self.now_us {
            self.now_us = target_us;
        }
    }
}

/// Concrete [`BoardHal`] composition used by [`DirectXilBoard`].
pub type DirectBoardHal = BoardHal<FakeImu, FakeBaro, FakeMag, FakeGnss, DirectTime, FakeActuator>;

/// Normalized actuator command returned by [`DirectXilBoard::step_truth`] and
/// [`DirectXilBoard::step_sensors`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DirectActuatorCmd {
    /// Simulation timestamp in microseconds when this command was produced.
    pub timestamp_us: u64,
    /// Normalized actuator outputs (`[0.0, 1.0]` for rotors; zeroed when disarmed).
    pub outputs: [f32; MAX_ACTUATOR_OUTPUTS],
    /// Number of active channels populated in `outputs`.
    pub count: u8,
    /// `true` when the kernel and board HAL are in [`InitState::Armed`].
    pub armed: bool,
}

impl Default for DirectActuatorCmd {
    fn default() -> Self {
        Self {
            timestamp_us: 0,
            outputs: [0.0; MAX_ACTUATOR_OUTPUTS],
            count: 0,
            armed: false,
        }
    }
}

/// Configuration for [`DirectXilBoard`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DirectBoardConfig {
    /// WGS-84 geodetic origin for the local NED frame.
    pub origin: GeodeticPosition,
    /// Decimal year used to evaluate the WMM2025 Earth magnetic field.
    pub epoch: DecimalYear,
    /// Sensor noise tier applied during synthesis.
    pub noise: DirectNoiseTier,
    /// Seed for the deterministic `splitmix64` Gaussian noise generator.
    pub noise_seed: u64,
    /// MAVLink system ID (`1..=255`).
    pub sys_id: u8,
    /// MAVLink component ID (`1..=255`).
    pub comp_id: u8,
}

/// Zero-overhead in-process XIL board generic over controller `C` and mixer `M`.
pub struct DirectXilBoard<C, M>
where
    C: aviate_core::control::VehicleController,
    M: aviate_core::mixer::Mixer,
{
    config: DirectBoardConfig,
    kernel: DefaultAviateKernel<C, M>,
    board_hal: DirectBoardHal,
    synthesizer: DirectSynthesizer,
    endpoint: DirectMavlinkEndpoint<DEFAULT_RX_RING_BYTES, DEFAULT_TX_RING_BYTES>,
    ingress: DirectCommandIngress,
    sensor_cache: DirectSensorCache,
    last_effective_command: Command,
    last_actuator_cmd: DirectActuatorCmd,
    last_gnss_raw: Option<RawGnssReading>,
    active_generation: Option<u32>,
    boot_anchor_us: Option<u64>,
    ekf_initialized: bool,
    /// Time at which the attitude sources for the estimator seed were first complete.
    seed_wait_since_us: Option<u64>,
    /// Time of the last [`Self::step_sensors`] call in this generation.
    host_clock_us: Option<u64>,
    /// Static pressure at the origin: the datum of the pressure height
    /// of a seed without a fix.
    origin_pressure_pa: f32,
    /// Declination from the geomagnetic model at the origin.
    origin_declination_rad: f32,
    /// Declination from the geomagnetic model at the last GNSS fix.
    model_declination_rad: f32,
    /// Distance from the frame origin at which the origin moves to the
    /// vehicle \[m\].
    rebase_radius_m: f64,
    /// Map from the configured frame to the frame of the estimate, once
    /// the origin has moved.
    frame_map: Option<aviate_core::ekf::rebase::FrameShift>,
    /// Why the last refused link command was dropped.
    last_refusal: Option<DirectCommandRefusal>,
    /// Height of the geoid above the WGS-84 ellipsoid at the origin \[m\],
    /// when the host declares it.
    geoid_separation_m: Option<f64>,
    /// Whether `GPS_GLOBAL_ORIGIN` went out for the present origin.
    origin_announced: bool,
}

impl<C, M> DirectXilBoard<C, M>
where
    C: aviate_core::control::VehicleController,
    M: aviate_core::mixer::Mixer,
{
    /// Create a [`DirectXilBoard`] around an app-built `kernel` and `config`.
    ///
    /// # Errors
    ///
    /// Returns [`DirectSynthesisError`] when `config.origin` or `config.epoch` fails world-model checks.
    pub fn new(
        mut kernel: DefaultAviateKernel<C, M>,
        config: DirectBoardConfig,
    ) -> Result<Self, DirectSynthesisError> {
        kernel.state.checks.pre_arm.update_throttle(true);
        let origin = DirectOrigin::new(config.origin, config.epoch);
        let synthesizer =
            DirectSynthesizer::with_default_rates(origin, config.noise, config.noise_seed)?;
        let board_hal = BoardHal::new(
            FakeImu::new(),
            FakeBaro::new(),
            FakeMag::new(),
            FakeGnss::new(),
            DirectTime::new(),
            FakeActuator::new(),
        );
        let mut endpoint = DirectMavlinkEndpoint::new(config.sys_id, config.comp_id);
        endpoint.set_airspeed_commandable(kernel.pipeline().controller.flies_airspeed_target());
        let origin_declination_rad = synthesizer.environment().declination_rad();
        let origin_pressure_pa = synthesizer.environment().air.pressure_pa as f32;
        Ok(Self {
            config,
            kernel,
            board_hal,
            synthesizer,
            endpoint,
            ingress: DirectCommandIngress::default(),
            sensor_cache: DirectSensorCache::default(),
            last_effective_command: default_failsafe_command(),
            last_actuator_cmd: DirectActuatorCmd::default(),
            last_gnss_raw: None,
            active_generation: None,
            boot_anchor_us: None,
            ekf_initialized: false,
            seed_wait_since_us: None,
            host_clock_us: None,
            origin_pressure_pa,
            origin_declination_rad,
            model_declination_rad: origin_declination_rad,
            rebase_radius_m: frame::DEFAULT_REBASE_RADIUS_M,
            frame_map: None,
            last_refusal: None,
            geoid_separation_m: None,
            origin_announced: false,
        })
    }

    /// Replace the active kernel and reset all sensor, link, and lifecycle state.
    pub fn reset_with_kernel(&mut self, mut kernel: DefaultAviateKernel<C, M>) {
        kernel.state.checks.pre_arm.update_throttle(true);
        self.endpoint
            .set_airspeed_commandable(kernel.pipeline().controller.flies_airspeed_target());
        self.kernel = kernel;
        self.clear_runtime_state();
        self.active_generation = None;
    }

    /// Return an immutable reference to the flight-control kernel.
    #[must_use]
    pub const fn kernel(&self) -> &DefaultAviateKernel<C, M> {
        &self.kernel
    }

    /// Return a mutable reference to the flight-control kernel.
    pub fn kernel_mut(&mut self) -> &mut DefaultAviateKernel<C, M> {
        &mut self.kernel
    }

    /// Return the immutable [`DirectBoardConfig`].
    #[must_use]
    pub const fn config(&self) -> &DirectBoardConfig {
        &self.config
    }

    /// Return an immutable reference to [`DirectBoardHal`].
    #[must_use]
    pub const fn board_hal(&self) -> &DirectBoardHal {
        &self.board_hal
    }

    /// Return a mutable reference to [`DirectBoardHal`].
    pub fn board_hal_mut(&mut self) -> &mut DirectBoardHal {
        &mut self.board_hal
    }

    /// Return an immutable reference to [`DirectSynthesizer`].
    #[must_use]
    pub const fn synthesizer(&self) -> &DirectSynthesizer {
        &self.synthesizer
    }

    /// Return a mutable reference to [`DirectSynthesizer`].
    pub fn synthesizer_mut(&mut self) -> &mut DirectSynthesizer {
        &mut self.synthesizer
    }

    /// Return an immutable reference to [`DirectMavlinkEndpoint`].
    #[must_use]
    pub const fn endpoint(&self) -> &DirectMavlinkEndpoint {
        &self.endpoint
    }

    /// Return a mutable reference to [`DirectMavlinkEndpoint`].
    pub fn endpoint_mut(&mut self) -> &mut DirectMavlinkEndpoint {
        &mut self.endpoint
    }

    /// Configure aiding-sensor synthesis periods.
    pub fn set_sensor_rates(&mut self, rates: DirectSensorRates) {
        self.synthesizer.set_rates(rates);
    }

    /// Declare the static pressure at the origin.
    ///
    /// A board that starts without a position fix takes its height from
    /// the barometer, as the pressure height above this pressure. The
    /// default is the pressure of the standard atmosphere at the height
    /// of the origin.
    pub fn set_origin_pressure(&mut self, pressure_pa: f32) {
        self.origin_pressure_pa = pressure_pa;
    }

    /// Declare the height of the geoid above the WGS-84 ellipsoid at the
    /// origin \[m\].
    ///
    /// MAVLink altitudes in `GPS_GLOBAL_ORIGIN` and `GLOBAL_POSITION_INT`
    /// are above mean sea level, but the origin is configured with an
    /// ellipsoidal height. With the separation the board gives mean sea
    /// level and announces the origin. Without it the board does not send
    /// `GPS_GLOBAL_ORIGIN`, because it cannot state its altitude, and the
    /// altitude of `GLOBAL_POSITION_INT` is the ellipsoidal height.
    pub fn set_geoid_separation(&mut self, separation_m: f64) {
        self.geoid_separation_m = Some(separation_m).filter(|value| value.is_finite());
        self.origin_announced = false;
    }

    /// Send `GPS_GLOBAL_ORIGIN` again with the next telemetry.
    pub fn announce_origin(&mut self) {
        self.origin_announced = false;
    }

    /// Configure outbound MAVLink telemetry stream rates.
    pub fn set_telemetry_rates(&mut self, rates: DirectTelemetryRates) {
        self.endpoint.set_rates(rates);
    }

    /// Return the effective [`Command`] executed in the most recent kernel update.
    #[must_use]
    pub const fn last_effective_command(&self) -> &Command {
        &self.last_effective_command
    }

    /// Why the board dropped the last refused flight command from the
    /// link, if it dropped one.
    #[must_use]
    pub const fn last_command_refusal(&self) -> Option<DirectCommandRefusal> {
        self.last_refusal
    }

    /// Return `true` when the kernel has reached [`InitState::Ready`].
    #[must_use]
    pub fn is_ready(&self) -> bool {
        self.kernel.is_ready()
    }

    /// Return `true` when the kernel is in [`InitState::Armed`].
    #[must_use]
    pub fn is_armed(&self) -> bool {
        self.kernel.state.init_state == InitState::Armed
    }

    /// Push raw MAVLink v2 bytes into the inbound RX ring and return bytes accepted.
    pub fn push_mavlink_rx(&mut self, bytes: &[u8]) -> usize {
        self.endpoint.push_rx(bytes)
    }

    /// Pop encoded MAVLink v2 telemetry and acknowledgement bytes into `buf`.
    pub fn pop_mavlink_tx(&mut self, buf: &mut [u8]) -> usize {
        self.endpoint.pop_tx(buf)
    }

    /// Enqueue a typed [`SystemCommand`] for execution on the next step call.
    pub fn send_command(&mut self, cmd: SystemCommand) {
        let _ = self.endpoint.enqueue_command(cmd);
    }
}
