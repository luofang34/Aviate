//! The X-Plane SITL board: a TCP HIL link to the simulator's bridge
//! plugin, feeding the kernel through the simulator-neutral sensor
//! seam.
//!
//! Two properties distinguish it from the UDP HIL board:
//!
//! - The FLIGHT CONTROLLER dials the simulator, which listens. The link
//!   therefore comes and goes, and the board reports its state rather
//!   than assuming a peer.
//! - The bridge paces its sensor stream on actuator feedback: it holds
//!   the next sample until the command answering the previous one
//!   arrives. The actuator command is therefore sent on SENSOR RECEIPT,
//!   not on a wall-clock cadence — a timer-driven send would leave the
//!   bridge waiting and stall the simulation.

use std::io;

use aviate_backend_mavlink_hil::{HilBackend, HilBackendConfig};
use aviate_core::control::Command;
use aviate_core::hal::{ActuatorHal, SystemHal};
use aviate_core::mixer::ActuatorCmd;
use aviate_core::{ArmError, DefaultAviateKernel, DisarmError, InitState};
use aviate_hal_io::{BoardHal, FakeActuator, FakeBaro, FakeGnss, FakeImu, FakeMag};
use aviate_hal_xil::{SitlConfig, SitlIO};
use aviate_runtime::{SitlBoardInfo, SitlRunner, SitlTime};
use log::info;

/// Samples answered in one control iteration before yielding, so a
/// flooded link cannot monopolize the loop.
const MAX_SAMPLES_PER_ITERATION: usize = 32;

/// Board configuration.
#[derive(Debug, Clone)]
pub struct XPlaneConfig {
    /// The bridge plugin's listening address the board dials.
    pub simulator_addr: std::net::SocketAddr,
    /// System ID for outgoing messages.
    pub sys_id: u8,
    /// Component ID for outgoing messages.
    pub comp_id: u8,
    /// Reorders mixer lanes into the simulated airframe's rotor order,
    /// applied once just before the command reaches the wire.
    ///
    /// Motor NUMBERING is airframe knowledge, which belongs to the
    /// application, but the send belongs to the board — so the app
    /// injects the mapping here rather than the board guessing it. An
    /// absent mapping sends the mixer's own order.
    pub lane_order: Option<fn(&mut [f32; 16], u8)>,
}

impl Default for XPlaneConfig {
    fn default() -> Self {
        Self {
            simulator_addr: std::net::SocketAddr::from(([127, 0, 0, 1], 4560)),
            sys_id: 1,
            comp_id: 1,
            lane_order: None,
        }
    }
}

/// X-Plane SITL board, generic over the injected controller and mixer.
pub struct XPlaneBoard<C, M>
where
    C: aviate_core::control::VehicleController,
    M: aviate_core::mixer::Mixer,
{
    hil_backend: HilBackend,
    runner: SitlRunner<C, M>,
    lane_order: Option<fn(&mut [f32; 16], u8)>,
    armed: bool,
}

impl<C, M> XPlaneBoard<C, M>
where
    C: aviate_core::control::VehicleController,
    M: aviate_core::mixer::Mixer,
{
    /// Builds the board around an injected kernel and dials the bridge.
    ///
    /// # Errors
    ///
    /// Returns the failure of binding the local simulator transport. A
    /// bridge that is not listening yet is NOT an error: the HIL link
    /// retries, because the simulator commonly starts after the flight
    /// controller.
    pub fn with_config(
        kernel: DefaultAviateKernel<C, M>,
        config: XPlaneConfig,
    ) -> io::Result<Self> {
        let hil_backend = HilBackend::connect_tcp(HilBackendConfig {
            local_port: 0,
            simulator_addr: config.simulator_addr,
            sys_id: config.sys_id,
            comp_id: config.comp_id,
        });
        let transport = SitlIO::new(SitlConfig::default())?;
        let board_hal = BoardHal::new(
            FakeImu::new(),
            FakeBaro::new(),
            FakeMag::new(),
            FakeGnss::new(),
            SitlTime::new(),
            FakeActuator::new(),
        );
        Ok(Self {
            hil_backend,
            runner: SitlRunner::new(transport, board_hal, kernel),
            lane_order: config.lane_order,
            armed: false,
        })
    }

    /// Runs one control iteration, answering EVERY sensor sample the
    /// bridge delivered.
    ///
    /// The bridge paces its stream on actuator feedback: it holds its
    /// next sample until the previous one is answered. Answering only
    /// the newest sample of a batch would leave the earlier ones
    /// unanswered and let the bridge's own queue grow until it gives up
    /// on the flight controller, so each sample gets its own kernel step
    /// and its own command.
    pub fn step(&mut self) -> ActuatorCmd {
        let mut last = ActuatorCmd::default();
        let mut answered = 0_usize;
        while let Some(packet) = self.hil_backend.poll() {
            self.runner.transport.feed_sensor_packet(&packet);
            last = self.runner.step();
            self.answer_sample();
            answered += 1;
            // A bounded drain keeps one iteration from monopolizing the
            // loop when the link floods.
            if answered >= MAX_SAMPLES_PER_ITERATION {
                break;
            }
        }
        if answered == 0 {
            // No sample this iteration: still advance the kernel so
            // command ingress and timers run.
            last = self.runner.step();
        }
        last
    }

    /// Sends the command the kernel produced for the sample just fed.
    fn answer_sample(&mut self) {
        let Some(mut sim_cmd) = self.runner.transport.take_actuator_cmd() else {
            return;
        };
        // Mixer outputs are force-domain per-motor thrust; the resolved
        // actuator curve converts them to the boundary command here,
        // exactly once, before it reaches the wire.
        apply_actuator_curve(self.runner.kernel.cfg().actuator_curve, &mut sim_cmd);
        // Lane order is applied AFTER the curve and only here: the
        // mixer, the controller and the curve all reason in the mixer's
        // own numbering.
        if let Some(reorder) = self.lane_order {
            reorder(&mut sim_cmd.outputs, sim_cmd.count);
        }
        if let Err(error) = self.hil_backend.send_actuators(&sim_cmd) {
            // A dropped command is the bridge's cue to stall; it must be
            // visible, not swallowed.
            log::debug!("actuator command not sent: {error}");
        }
    }

    /// Whether the HIL link to the bridge is up.
    pub fn connected(&self) -> bool {
        self.hil_backend.connected()
    }

    /// Arms the flight controller.
    ///
    /// # Errors
    ///
    /// Returns the kernel's refusal, so a harness sees a refused arm
    /// rather than a silent no-op.
    pub fn arm(&mut self) -> Result<(), ArmError> {
        info!(
            "Arm command (state={:?})",
            self.runner.kernel.state.init_state
        );
        self.runner.kernel.arm()?;
        self.runner.board_hal.arm();
        self.runner.transport.set_armed(true);
        self.armed = true;
        Ok(())
    }

    /// Disarms the flight controller.
    ///
    /// # Errors
    ///
    /// Returns the kernel's refusal (an in-flight disarm, for example).
    pub fn disarm(&mut self) -> Result<(), DisarmError> {
        self.runner.kernel.disarm()?;
        self.runner.board_hal.disarm();
        self.runner.transport.set_armed(false);
        self.armed = false;
        Ok(())
    }

    /// Cuts outputs immediately, in any flight phase.
    pub fn terminate(&mut self) {
        info!("Emergency terminate");
        self.runner.kernel.terminate();
        self.runner.board_hal.disarm();
        self.runner.transport.set_armed(false);
        self.armed = false;
    }

    /// Sets the flight command, routed through the shared ingress so
    /// the setpoint carries a real receive timestamp.
    pub fn set_command(&mut self, cmd: Command) {
        self.runner.kernel.state.checks.pre_arm.update_throttle(
            cmd.setpoint.collective_thrust < aviate_core::kernel_types::THROTTLE_LOW_MAX_COLLECTIVE,
        );
        let now_ticks = self.runner.transport.now().ticks;
        self.runner
            .ingress
            .receive(aviate_hal_io::SystemCommand::FlightControl(cmd), now_ticks);
    }

    /// Whether the kernel is ready for flight.
    pub fn is_ready(&self) -> bool {
        self.runner.kernel.is_ready()
    }

    /// Whether the kernel is armed.
    pub fn is_armed(&self) -> bool {
        self.runner.kernel.state.init_state == InitState::Armed
    }

    /// Starts the estimate telemetry stream this app's config declares.
    pub fn init_telemetry(&mut self, cfg: &aviate_config::AppConfig, loop_hz: u32) {
        self.runner.init_telemetry(cfg, loop_hz);
    }

    /// Whether the telemetry stream is running.
    pub fn telemetry_enabled(&self) -> bool {
        self.runner.telemetry_enabled()
    }

    /// The kernel this board drives.
    pub fn kernel(&self) -> &DefaultAviateKernel<C, M> {
        &self.runner.kernel
    }

    /// Microseconds on the simulation clock.
    pub fn now_us(&self) -> u64 {
        self.runner.now_us()
    }

    /// Received frames, sent frames, CRC failures, unsent commands, and
    /// successful connections.
    pub fn stats(&self) -> (u64, u64, u64, u64, u64) {
        self.hil_backend.tcp_stats()
    }

    /// Sends one heartbeat to the bridge.
    pub fn send_heartbeat(&mut self) {
        let _ = self.hil_backend.send_heartbeat(self.armed);
    }
}

/// Converts force-domain mixer outputs into boundary actuator commands
/// in place — the single curve application point for this path.
fn apply_actuator_curve(
    curve: aviate_core::kernel::config::ActuatorCurveKind,
    cmd: &mut aviate_hal_xil::sim_types::SimActuatorCmd,
) {
    let lanes = usize::from(cmd.count).min(cmd.outputs.len());
    for lane in &mut cmd.outputs[..lanes] {
        *lane = curve
            .boundary_command(aviate_core::types::NormalizedThrust(*lane))
            .0;
    }
}

/// Board info for the X-Plane SITL board.
pub const BOARD_INFO: SitlBoardInfo = SitlBoardInfo {
    name: "sitl-xplane",
    description: "X-Plane SITL via the MAVLink HIL bridge over TCP",
};

#[cfg(test)]
mod tests;
