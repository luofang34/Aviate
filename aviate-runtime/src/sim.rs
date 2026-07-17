//! Simulation runtime for SITL/HITL
//!
//! Extracted from sitl-gazebo board to eliminate ~165 lines of stepping logic duplication.
//!
//! The `SitlRunner` struct encapsulates the control loop stepping logic.
//!
//! ## Shared Components
//!
//! This module provides factory functions and types that are shared across all SITL boards:
//! - `default_command()` - Creates a safe failsafe command with zero thrust
//! - `sitl_timestamp()` - Returns a SITL timestamp
//! - `BoardInfo` - Common board information structure
//! - `run_control_loop()` - Configurable control loop with specified period
//!
//! ## Backend Support
//!
//! - **Gazebo (SitlIO)**: Shared memory FFI + MAVLink commands
//! - **jMAVSim (HilBackend)**: Full MAVLink HIL (sensor + actuator)
//!
//! Both backends use `SitlRunner` for the core stepping logic but have different
//! transport layer integrations.
//!
//! This module is only available when env-sitl or env-hitl features are enabled.

#![cfg(any(feature = "env-sitl", feature = "env-hitl"))]

mod step;

use log::{error, info, warn};

use crate::sensor_cache::SensorCache;
use crate::telemetry::{FrameTx, TelemetryTask};

use aviate_link::mavlink::{MavState, MavlinkCycleFormatter};

use aviate_config::AppConfig;
use aviate_core::control::Command;
use aviate_core::hal::SystemHal;
use aviate_core::{DefaultAviateKernel, InitState};
use aviate_hal_io::{BoardHal, FakeActuator, FakeBaro, FakeGnss, FakeImu, FakeMag};
use aviate_hal_xil::SitlIO;

/// Time source for SITL (re-exported for convenience)
///
/// Implements both `aviate_hal_io::TimeSource` (legacy) and `aviate_hal_io::TimeHal` (new).
pub struct SitlTime {
    start: std::time::Instant,
}

impl SitlTime {
    /// Create a SITL clock anchored at the current instant.
    pub fn new() -> Self {
        Self {
            start: std::time::Instant::now(),
        }
    }

    /// Get elapsed time in microseconds (internal helper)
    fn elapsed_us(&self) -> u64 {
        self.start.elapsed().as_micros() as u64
    }
}

impl Default for SitlTime {
    fn default() -> Self {
        Self::new()
    }
}

// Legacy TimeSource impl (used by existing BoardHal)
impl aviate_hal_io::TimeSource for SitlTime {
    fn now_us(&self) -> u64 {
        self.elapsed_us()
    }
}

// New TimeHal impl (for FlightRunner)
impl aviate_hal_io::TimeHal for SitlTime {
    fn now_us(&mut self) -> u64 {
        self.elapsed_us()
    }

    fn sleep_until_us(&mut self, target_us: u64) {
        let now = self.elapsed_us();
        if target_us > now {
            std::thread::sleep(std::time::Duration::from_micros(target_us - now));
        }
    }
}

/// SITL Board HAL type (Gazebo/SitlIO)
pub type SitlBoardHal = BoardHal<FakeImu, FakeBaro, FakeMag, FakeGnss, SitlTime, FakeActuator>;

// ============================================================================
// UDP Telemetry Transport (SITL-only)
// ============================================================================

use std::net::{SocketAddr, UdpSocket};

/// UDP frame transmitter for telemetry (SITL-only).
///
/// The target address is fixed at construction — deliberately no
/// mutator. The telemetry stream belongs to the CONFIGURED consumer;
/// a settable address is exactly the hook that once let whichever
/// peer commanded last steal the stream from a fixed consumer.
pub struct UdpFrameTx {
    socket: UdpSocket,
    addr: SocketAddr,
}

impl UdpFrameTx {
    /// Create a new UDP transmitter
    pub fn new(socket: UdpSocket, addr: SocketAddr) -> Self {
        Self { socket, addr }
    }
}

impl FrameTx for UdpFrameTx {
    fn try_send(&mut self, frame: &[u8]) -> Result<(), ()> {
        let _ = self.socket.send_to(frame, self.addr);
        Ok(())
    }
}

// ============================================================================
// SITL Runner
// ============================================================================

/// SITL runner encapsulating the stepping logic (Gazebo/SitlIO-specific for Phase 1)
///
/// This struct wraps SitlIO transport, BoardHal, and kernel, providing the control loop
/// stepping logic that was previously duplicated in GazeboSitlBoard.
///
/// **Future**: Make this generic over transport types to support both SitlIO and HilBackend.
pub struct SitlRunner<C, M>
where
    C: aviate_core::control::VehicleController,
    M: aviate_core::mixer::Mixer,
{
    /// Simulator transport (SitlIO for Gazebo)
    pub transport: SitlIO,

    /// Per-instance fault command listener. None when the
    /// `xil-fault` feature path didn't bind a socket (e.g. port in
    /// use). When Some, `step()` polls it each cycle and routes
    /// inbound `FaultCommand`s to the BoardHal's fake sensors.
    pub fault_ctrl: Option<aviate_hal_xil::FaultController>,

    /// Board HAL with fake sensors (same interface as real hardware)
    pub board_hal: SitlBoardHal,

    /// Flight controller kernel
    pub kernel: DefaultAviateKernel<C, M>,

    /// Last command received
    /// Shared command-ingress state machine (#133) — the same
    /// freshness implementation the hardware FlightRunner uses, so
    /// the two environments cannot drift: setpoints are retained
    /// with their OWN receive timestamp; discrete Arm/Disarm are
    /// one-shot and never refresh setpoint age.
    pub ingress: crate::command_ingress::CommandIngress<aviate_hal_io::SystemCommand>,

    /// Last IMU timestamp for dt calculation
    pub last_imu_time: Option<u64>,

    /// Cached sensor readings for kernel initialization
    pub sensor_cache: SensorCache,

    /// EKF initialization flag
    pub ekf_initialized: bool,

    /// Telemetry task (optional, config-driven)
    /// Uses MavlinkCycleFormatter for MAVLink protocol
    pub(crate) telemetry: Option<TelemetryTask<UdpFrameTx, MavlinkCycleFormatter>>,

    /// Iteration counter for rate dividers
    pub(crate) iteration: u32,
}

impl<C, M> SitlRunner<C, M>
where
    C: aviate_core::control::VehicleController,
    M: aviate_core::mixer::Mixer,
{
    /// Create a new SITL runner
    pub fn new(
        transport: SitlIO,
        board_hal: SitlBoardHal,
        kernel: DefaultAviateKernel<C, M>,
    ) -> Self {
        // Best-effort: bind the per-instance fault command port. If
        // the bind fails (port in use, instance mismatch), continue
        // without a fault controller — missions that don't inject
        // faults are unaffected, and fault-injecting missions log a
        // visible warning when the ack never arrives.
        let cfg = transport.config().clone();
        let fault_ctrl = aviate_hal_xil::FaultController::new(&cfg).ok();
        Self {
            transport,
            fault_ctrl,
            board_hal,
            kernel,
            ingress: crate::command_ingress::CommandIngress::default(),
            last_imu_time: None,
            sensor_cache: SensorCache::new(),
            ekf_initialized: false,
            telemetry: None,
            iteration: 0,
        }
    }

    /// Initialize telemetry from config (called in AppRuntime::run)
    ///
    /// Looks for a transport with "telemetry" role and UDP endpoint.
    /// If found, creates a TelemetryTask for GCS communication.
    pub fn init_telemetry(&mut self, cfg: &AppConfig, loop_hz: u32) {
        let Some(telem_cfg) = &cfg.telemetry else {
            warn!("Telemetry disabled: no [telemetry] section in the app config");
            return;
        };
        // Refuse an invalid config loudly; running with silently
        // reinterpreted rates would ship a telemetry contract the
        // config never requested. Every refusal below disables
        // telemetry and changes nothing else — the control loop
        // must not care whether the stream exists.
        if let Err(e) = crate::validation::validate_telemetry_config(telem_cfg) {
            error!("Telemetry disabled: {}", e);
            return;
        }
        // The stream's home is the transport with the "telemetry"
        // role; "gcs" is accepted as a fallback for configs that
        // predate the dedicated role. Whichever matches, the
        // endpoint is PERMANENT for the process: telemetry goes to
        // the configured consumer, never to whichever peer
        // commanded last (see `UdpFrameTx`).
        let transport = cfg
            .transports
            .iter()
            .find(|t| t.roles.iter().any(|r| r == "telemetry") && t.endpoint.is_some())
            .or_else(|| {
                cfg.transports
                    .iter()
                    .find(|t| t.roles.iter().any(|r| r == "gcs") && t.endpoint.is_some())
            });
        let Some(t) = transport else {
            warn!("Telemetry disabled: no transport with a \"telemetry\"/\"gcs\" role and an endpoint");
            return;
        };
        let Some(endpoint) = &t.endpoint else {
            return; // unreachable: the find above requires an endpoint
        };
        let addr = match endpoint.parse::<SocketAddr>() {
            Ok(addr) => addr,
            Err(e) => {
                error!("Telemetry disabled: endpoint {endpoint:?} is not host:port ({e})");
                return;
            }
        };
        // Bind an ephemeral local port for sending.
        let sock = match UdpSocket::bind("0.0.0.0:0") {
            Ok(sock) => sock,
            Err(e) => {
                error!("Telemetry disabled: cannot bind a UDP socket ({e})");
                return;
            }
        };
        if let Err(e) = sock.set_nonblocking(true) {
            error!("Telemetry disabled: cannot set the socket non-blocking ({e})");
            return;
        }
        let tx = UdpFrameTx::new(sock, addr);

        // Create protocol-specific formatter (from aviate-link)
        let formatter = match MavlinkCycleFormatter::new(telem_cfg, loop_hz) {
            Ok(formatter) => formatter,
            Err(e) => {
                error!("Telemetry disabled: {}", e);
                return;
            }
        };
        // Create protocol-agnostic task (from aviate-runtime)
        self.telemetry = Some(TelemetryTask::new(tx, formatter));
        info!("Telemetry enabled: {} via {}", endpoint, t.protocol);
    }

    /// Whether `init_telemetry` accepted a config and the stream is
    /// live. `false` after any loud refusal — callers that require
    /// telemetry can turn that into their own failure.
    pub fn telemetry_enabled(&self) -> bool {
        self.telemetry.is_some()
    }

    /// Check if system is armed
    pub fn is_armed(&self) -> bool {
        self.kernel.state.init_state == InitState::Armed
    }

    /// Get access to transport
    pub fn transport_mut(&mut self) -> &mut SitlIO {
        &mut self.transport
    }

    /// Get system uptime in microseconds
    pub fn now_us(&self) -> u64 {
        self.transport.now_us()
    }
}

// ============================================================================
// Shared Factory Functions (used by all SITL boards)
// ============================================================================

use aviate_core::control::{CommandSource, ControlMode, Setpoint};
use aviate_core::time::{TimeSource, Timestamp};
use aviate_core::types::NormalizedThrust;

/// Create a safe default/failsafe command with zero thrust
///
/// This is shared by all SITL boards to ensure consistent failsafe behavior.
pub fn default_command() -> Command {
    Command {
        mode: ControlMode::Attitude,
        setpoint: Setpoint {
            collective_thrust: NormalizedThrust(0.0),
            ..Default::default()
        },
        config_mode_request: None,
        sensor_overrides: None,
        sequence: 0,
        source: CommandSource::Failsafe,
    }
}

/// Map FC InitState to MAVLink MAV_STATE for heartbeat system_status
///
/// - `Boot` (1): PowerOn, ConfigLoading
/// - `Calibrating` (2): SensorInit, EstimatorConverging, PreArm
/// - `Standby` (3): Ready (disarmed, can be armed)
/// - `Active` (4): Armed
fn init_state_to_mav_state(state: InitState) -> MavState {
    match state {
        InitState::PowerOn | InitState::ConfigLoading => MavState::Boot,
        InitState::SensorInit | InitState::EstimatorConverging | InitState::PreArm => {
            MavState::Calibrating
        }
        InitState::Ready => MavState::Standby,
        InitState::Armed => MavState::Active,
        InitState::Disarmed => MavState::Standby,
        InitState::Fault => MavState::Critical,
    }
}

/// SITL timestamp function for mixer
pub fn sitl_timestamp() -> Timestamp {
    Timestamp {
        ticks: 0,
        source: TimeSource::Internal,
    }
}

/// Board information structure (shared across SITL boards)
#[derive(Clone, Debug)]
pub struct SitlBoardInfo {
    /// Short board identifier.
    pub name: &'static str,
    /// Human-readable board description.
    pub description: &'static str,
}

/// Run a control loop with configurable period
///
/// This is the shared control loop implementation used by all SITL boards.
/// The only difference between boards is the `loop_period_us` parameter:
/// - Gazebo: 1000us (1kHz)
/// - jMAVSim: 2500us (400Hz)
///
/// # Arguments
/// * `runner` - The SitlRunner to step
/// * `loop_period_us` - Control loop period in microseconds
pub fn run_control_loop<C, M>(runner: &mut SitlRunner<C, M>, loop_period_us: u64) -> !
where
    C: aviate_core::control::VehicleController,
    M: aviate_core::mixer::Mixer,
{
    let mut last_tick = runner.now_us();

    loop {
        let now = runner.now_us();
        let elapsed = now.saturating_sub(last_tick);

        if elapsed >= loop_period_us {
            last_tick = now;
            runner.step();
        } else {
            let remaining_us = loop_period_us - elapsed;
            if remaining_us > 100 {
                std::thread::sleep(std::time::Duration::from_micros(remaining_us - 100));
            }
        }
    }
}

/// Default loop periods for different simulators
pub mod loop_periods {
    /// Gazebo SITL loop period (1kHz)
    pub const GAZEBO_US: u64 = 1000;
    /// jMAVSim SITL loop period (400Hz to match jMAVSim default rate)
    pub const JMAVSIM_US: u64 = 2500;
}

// Re-export for convenience (Phase 1 stub - Phase 2+ will use this for AppRuntime)

/// Application runtime for SITL/HITL (Phase 1 stub)
///
/// Phase 2+: This will wrap SitlRunner and provide AppRuntime::run(config)
pub struct AppRuntime<Board, Airframe> {
    _board: core::marker::PhantomData<Board>,
    _airframe: core::marker::PhantomData<Airframe>,
}

impl<Board, Airframe> AppRuntime<Board, Airframe> {
    /// Run the simulation application (never returns)
    ///
    /// Phase 1: Stub (boards use SitlRunner directly)
    /// Phase 2+: Full implementation using AppConfig
    pub fn run(_config: &AppConfig) -> ! {
        unimplemented!("Phase 1: Use SitlRunner directly. Phase 2+: Full AppRuntime::run()")
    }
}
