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
    last_fix: Option<aviate_hal_xil::sim_types::SimGnssData>,
    last_imu: Option<aviate_hal_xil::sim_types::SimImuData>,
    /// Force-domain per-lane offsets added to every outgoing command,
    /// in mixer lane order. Zero in flight; the identification
    /// experiment injects its excitation here so the probe reaches the
    /// plant regardless of what the closed loop is doing.
    lane_injection: [f32; 4],
    /// The last collective mean this board let onto the wire, for the
    /// spool-rate constraint. See `answer_sample`.
    last_collective: f32,
    last_answer_at: Option<std::time::Instant>,
    /// GPS altitude captured at arming, and whether the vehicle has
    /// climbed clear of it since. See `limit_collective_spool`.
    ground_alt: Option<f32>,
    airborne: bool,
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
            last_fix: None,
            last_imu: None,
            lane_injection: [0.0; 4],
            last_collective: 0.0,
            last_answer_at: None,
            ground_alt: None,
            airborne: false,
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
            if let Some(gnss) = packet.gnss {
                self.last_fix = Some(gnss);
            }
            if let Some(imu) = packet.imu {
                self.last_imu = Some(imu);
            }
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

    /// Sets the per-lane force-domain injection (identification only).
    pub fn set_lane_injection(&mut self, lanes: [f32; 4]) {
        self.lane_injection = lanes;
    }

    /// Sends the command the kernel produced for the sample just fed.
    fn answer_sample(&mut self) {
        let Some(mut sim_cmd) = self.runner.transport.take_actuator_cmd() else {
            return;
        };
        for (lane, inj) in sim_cmd.outputs.iter_mut().zip(self.lane_injection) {
            *lane = (*lane + inj).clamp(0.0, 1.0);
        }
        self.limit_collective_spool(&mut sim_cmd);
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

    /// Rate-limits the RISE of the collective mean, preserving the
    /// differential content untouched.
    ///
    /// A rotor commanded from idle to high thrust faster than its RPM
    /// can follow stalls its blades, and the simulator models the
    /// stall as a latched state: thrust stays collapsed (measured
    /// NEGATIVE prop force at full command) for as long as the demand
    /// is held. The constraint is therefore a property of the PLANT,
    /// enforced at the last exit before the wire so no commander —
    /// cascade, harness, or failsafe — can slam the collective. The
    /// downward direction is deliberately unlimited: cutting thrust is
    /// always safe and a disarm must not ramp.
    fn limit_collective_spool(&mut self, sim_cmd: &mut aviate_hal_xil::sim_types::SimActuatorCmd) {
        /// Maximum collective rise, force-domain units per second —
        /// paced to the rotors' RPM inertia so blade angle of attack
        /// never outruns rotor speed. The bracketing is empirical and
        /// consistent across every flight of the night: staircase
        /// ramps at 0.036/s always spool cleanly, ramps at 0.08/s and
        /// above always leave the props partially stalled and the
        /// vehicle perched at "full" thrust. Fifteen seconds from idle
        /// to hover is a turbine-class spool-up, which a rotor this
        /// size honestly is.
        const RISE_PER_S: f32 = 0.035;
        /// Collective mean ceiling, and the more important half of the
        /// spool constraint: the prop model's thrust curve COLLAPSES
        /// under a sustained high command (measured: force 1.0 held on
        /// the ground reads near-zero prop force — blade stall latched
        /// by RPM that can no longer catch up). The healthy regime
        /// tops out well below that, hover sits near 0.43, and the
        /// ceiling keeps every commander out of the latch while
        /// reserving differential headroom for attitude authority.
        const MEAN_CEILING: f32 = 0.55;
        let now = std::time::Instant::now();
        let dt = self
            .last_answer_at
            .map_or(0.01, |at| now.duration_since(at).as_secs_f32())
            .clamp(0.0, 0.05);
        self.last_answer_at = Some(now);
        /// Collective the rotors idle at while armed. An eVTOL spools
        /// its rotors ONCE and modulates around the operating point;
        /// dropping to zero between demands would pay the full spool
        /// time again on every climb — and this idle sits safely below
        /// liftoff thrust while keeping the blades unstalled.
        const ARMED_IDLE: f32 = 0.35;
        let lanes = usize::from(sim_cmd.count).min(4).max(1);
        let mean: f32 =
            sim_cmd.outputs[..lanes].iter().sum::<f32>() / lanes as f32;
        let floor = if sim_cmd.armed { ARMED_IDLE } else { 0.0 };
        let target = mean.max(floor);
        let allowed = if target > self.last_collective {
            (self.last_collective + RISE_PER_S * dt).min(target)
        } else {
            target
        }
        .min(MEAN_CEILING);
        let shift = allowed - mean;
        if shift < 0.0 {
            for lane in &mut sim_cmd.outputs[..lanes] {
                *lane = (*lane + shift).clamp(0.0, 1.0);
            }
        }
        self.last_collective = allowed;

        // Until the vehicle has climbed clear of its arming altitude,
        // squeeze the differential toward the mean: on its gear the
        // attitude is held by the ground, and the cascade's per-lane
        // dither keeps re-tripping blades into stall exactly the way a
        // symmetric ramp — which spools cleanly every time — does not.
        // One-way: full authority from the moment it is airborne.
        if !self.airborne {
            let clear = match (self.ground_alt, self.last_fix.as_ref()) {
                (Some(ground), Some(fix)) => fix.alt_m > ground + 1.0,
                _ => false,
            };
            if clear {
                self.airborne = true;
            } else {
                let new_mean: f32 =
                    sim_cmd.outputs[..lanes].iter().sum::<f32>() / lanes as f32;
                for lane in &mut sim_cmd.outputs[..lanes] {
                    *lane = (new_mean + (*lane - new_mean) * 0.15).clamp(0.0, 1.0);
                }
            }
        }
    }

    /// Whether the HIL link to the bridge is up.
    pub fn connected(&self) -> bool {
        self.hil_backend.connected()
    }

    /// The most recent GNSS fix the bridge delivered.
    ///
    /// The simulated receiver is the one measurement of where the
    /// vehicle actually is that does not pass through the estimator, so
    /// a flight can be read from it rather than inferred.
    pub fn last_fix(&self) -> Option<&aviate_hal_xil::sim_types::SimGnssData> {
        self.last_fix.as_ref()
    }

    /// The most recent IMU sample the bridge delivered — the body
    /// rates an identification experiment fits its model against.
    pub fn last_imu(&self) -> Option<&aviate_hal_xil::sim_types::SimImuData> {
        self.last_imu.as_ref()
    }

    /// Waits for the bridge's next sample, up to `timeout`.
    ///
    /// This is how the control loop paces itself. The bridge holds its
    /// next sample until the previous one is answered, and it allows
    /// itself only a millisecond or two per simulator frame to drain
    /// the samples that frame produced — so a loop paced by a sleep
    /// answers a fraction of them and the bridge's queue overflows.
    pub fn wait_for_sample(&mut self, timeout: std::time::Duration) -> bool {
        self.hil_backend.wait_readable(timeout)
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
        self.ground_alt = self.last_fix.map(|fix| fix.alt_m);
        self.airborne = false;
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
