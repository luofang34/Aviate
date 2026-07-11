//! Simulation runtime for SITL/HITL
//!
//! Extracted from sitl-gazebo board to eliminate ~165 lines of stepping logic duplication.
//!
//! The `SitlRunner` struct encapsulates the control loop stepping logic.
//!
//! ## Shared Components
//!
//! This module provides factory functions and types that are shared across all SITL boards:
//! - `create_kernel()` - Creates an AviateKernel with MultirotorController + QuadXMixerX500
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

use log::info;

use crate::sensor_cache::SensorCache;
use crate::telemetry::{FrameTx, TelemetryTask};

use aviate_link::mavlink::{MavState, MavlinkCycleFormatter};

use aviate_config::AppConfig;
use aviate_core::control::multirotor::MultirotorController;
use aviate_core::control::Command;
use aviate_core::ekf::Ekf;
use aviate_core::hal::SystemHal;
use aviate_core::mixer::QuadXMixerX500;
use aviate_core::mixer::Sanitizer;
use aviate_core::{AviateKernel, DefaultAviateKernel, InitState};
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

/// SITL Kernel type
pub type SitlKernel = DefaultAviateKernel<MultirotorController, QuadXMixerX500>;

// ============================================================================
// UDP Telemetry Transport (SITL-only)
// ============================================================================

use std::net::{SocketAddr, UdpSocket};

/// UDP frame transmitter for telemetry (SITL-only)
pub struct UdpFrameTx {
    socket: UdpSocket,
    addr: SocketAddr,
}

impl UdpFrameTx {
    /// Create a new UDP transmitter
    pub fn new(socket: UdpSocket, addr: SocketAddr) -> Self {
        Self { socket, addr }
    }

    /// Update target address (e.g. after receiving from GCS)
    pub fn set_addr(&mut self, addr: SocketAddr) {
        self.addr = addr;
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
pub struct SitlRunner {
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
    pub kernel: SitlKernel,

    /// Last command received
    pub last_cmd: Command,

    /// Microsecond tick when `last_cmd` was last refreshed by an
    /// uplink `SystemCommand::FlightControl` frame. `None` until
    /// the first command lands; `command_age_ms` clamps to
    /// `u32::MAX` while None so `update_command_status` enforces
    /// the timeout immediately.
    pub last_cmd_rx_ticks: Option<u64>,

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

impl SitlRunner {
    /// Create a new SITL runner
    pub fn new(
        transport: SitlIO,
        board_hal: SitlBoardHal,
        kernel: SitlKernel,
        default_command: Command,
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
            last_cmd: default_command,
            last_cmd_rx_ticks: None,
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
        if let Some(telem_cfg) = &cfg.telemetry {
            // Find transport with "telemetry" role and endpoint
            if let Some(t) = cfg.transports.iter().find(|t| {
                t.roles.iter().any(|r| r == "telemetry" || r == "gcs") && t.endpoint.is_some()
            }) {
                if let Some(ref endpoint) = t.endpoint {
                    // Bind to ephemeral port for sending (target address is updated dynamically)
                    let sock = UdpSocket::bind("0.0.0.0:0").expect("bind telemetry socket");
                    sock.set_nonblocking(true).expect("set nonblocking");

                    // Create frame transmitter with initial endpoint from config
                    let addr = endpoint.parse::<SocketAddr>().expect("parse endpoint");
                    let tx = UdpFrameTx::new(sock, addr);

                    // Create protocol-specific formatter (from aviate-link)
                    let formatter = MavlinkCycleFormatter::new(telem_cfg, loop_hz);
                    // Create protocol-agnostic task (from aviate-runtime)
                    self.telemetry = Some(TelemetryTask::new(tx, formatter));
                    info!("Telemetry enabled: {} via {}", endpoint, t.protocol);
                }
            }
        }
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

use aviate_core::control::{CommandSource, ConfigMode, ControlMode, Setpoint};
use aviate_core::mixer::ModeConfig;
use aviate_core::time::{TimeSource, Timestamp};
use aviate_core::types::Normalized;

/// Build the SITL kernel wired for the x500 airframe.
pub fn create_kernel() -> SitlKernel {
    // Single tuning source (#114): the same CascadeGains value and
    // hover trim construct the flying controller AND land in the
    // lockstep-hashed ResolvedKernelConfig — two independently
    // initialized copies can drift apart silently, with the hash
    // vouching for tuning the cascade isn't actually flying. The
    // runtime deliberately does NOT depend on a concrete airframe
    // crate; airframe selection moves to the app layer with #120's
    // preset loading (this factory retires then).
    let gains = aviate_core::control::cascade_gains::CascadeGains::x500_defaults();
    let hover: f32 = 0.77;
    let controller = MultirotorController::from_gains(gains, hover);
    let mixer = QuadXMixerX500 {
        timestamp_source: sitl_timestamp,
    };
    let mode_config = ModeConfig {
        mode: ConfigMode::Hover,
        groups: &[],
    };

    let mut kernel = AviateKernel::new(Ekf::default(), controller, mixer, Sanitizer, mode_config);
    kernel.cfg.cascade_gains = gains;
    kernel.cfg.hover_thrust_norm = aviate_core::types::Normalized(hover);

    // Initialize throttle check as satisfied (default command has low throttle)
    kernel.state.checks.pre_arm.update_throttle(true);

    kernel
}

/// Create a safe default/failsafe command with zero thrust
///
/// This is shared by all SITL boards to ensure consistent failsafe behavior.
pub fn default_command() -> Command {
    Command {
        mode: ControlMode::Attitude,
        setpoint: Setpoint {
            collective_thrust: Normalized(0.0),
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
pub fn run_control_loop(runner: &mut SitlRunner, loop_period_us: u64) -> ! {
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
