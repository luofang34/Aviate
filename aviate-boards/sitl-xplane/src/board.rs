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

mod arm_authorization;
mod config;
mod handshake;
mod observation;
mod packet;
mod tuning_trace;
mod wire;

pub use config::{
    XPlaneConfig, XPlaneHoverInitialization, XPlanePerturbationBindingError, BOARD_INFO,
};
pub use handshake::{RuntimeHandshakeError, XPlaneRuntimeHandshake};
pub use observation::{XPlaneConstraintFlags, XPlaneControlObservation, XPlaneSendEvidence};
pub use tuning_trace::{
    TuningActuatorApplication, TuningActuatorBypassReason, TuningActuatorEligibility,
    TuningCommand, TuningCommandSource, TuningConfigMode, TuningConstraintFlags, TuningControlMode,
    TuningControlObservation, TuningEstimate, TuningEstimateQuality, TuningEstimateValidity,
    TuningFrameType, TuningHandshake, TuningHoverEstimatorMode, TuningHoverInitialization,
    TuningImu, TuningObservationAck, TuningPerturbationCapability, TuningReady, TuningSendEvidence,
    TuningSensorApplication, TuningSetpoint, TuningTraceError, XPlaneTuningTraceConfig,
    XPlaneTuningTraceIdentity,
};

use std::io;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

use aviate_backend_mavlink_hil::{HilBackend, HilBackendConfig};
use aviate_config::xplane_model::XPlaneModelDigest;
use aviate_core::control::Command;
use aviate_core::hal::{ActuatorHal, SystemHal};
use aviate_core::mixer::ActuatorCmd;
use aviate_core::{ArmError, DefaultAviateKernel, DisarmError, InitState};
use aviate_hal_io::{BoardHal, FakeActuator, FakeBaro, FakeGnss, FakeImu, FakeMag};
use aviate_hal_xil::{SitlConfig, SitlIO};
use aviate_runtime::{SitlRunner, SitlTime};
use log::{info, warn};

use config::validate_hover_initialization;
use handshake::{validate_kernel_model, RuntimeIdentityGate};

/// X-Plane SITL board, generic over the injected controller and mixer.
pub struct XPlaneBoard<C, M>
where
    C: aviate_core::control::VehicleController,
    M: aviate_core::mixer::Mixer,
{
    hil_backend: HilBackend,
    runner: SitlRunner<C, M>,
    lane_order: [u8; 4],
    motor_count: u8,
    max_samples_per_iteration: usize,
    model_digest: XPlaneModelDigest,
    armed: bool,
    last_fix: Option<aviate_hal_xil::sim_types::SimGnssData>,
    last_imu: Option<aviate_hal_xil::sim_types::SimImuData>,
    /// Force-domain per-lane offsets added to every outgoing command,
    /// in mixer lane order. Zero in flight; the identification
    /// experiment injects its excitation here so the probe reaches the
    /// plant regardless of what the closed loop is doing.
    lane_injection: [f32; 4],
    /// The wire constraints between the mixer and the bridge — the
    /// plant-protection state machine, pure and tested on its own.
    wire: wire::WireConstraints,
    /// Wall-clock arrival of the newest bridge packet, for gap probes.
    last_packet_at: Option<std::time::Instant>,
    /// The newest sample's clock, and the dt it implied: the wire
    /// constraints pace the SIMULATED plant, so their dt is sample
    /// time, which a dilated simulator stretches along with the
    /// physics a wall clock would outrun.
    last_sample_time_us: Option<u64>,
    sample_dt_sec: f32,
    runtime_identity: RuntimeIdentityGate,
    runtime_failure: Option<RuntimeHandshakeError>,
    control_observations: Vec<XPlaneControlObservation>,
    tuning_trace: Option<tuning_trace::TuningTracePublisher>,
    perturbation: Option<aviate_hal_xil::perturbation::PerturbationEngine>,
    perturbation_failure: Option<aviate_hal_xil::perturbation::PerturbationError>,
    perturbation_guard: Option<aviate_hal_xil::perturbation::LiveArtifactGuard>,
    perturbation_identity_bound: bool,
    artifact_failure: Arc<AtomicBool>,
    hover_initialization: XPlaneHoverInitialization,
    sample_sequence: u64,
    last_answer_armed: Option<bool>,
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
        config
            .model
            .validate()
            .map_err(|error| io::Error::new(io::ErrorKind::InvalidInput, error))?;
        let model_digest = config
            .model
            .canonical_digest()
            .map_err(|error| io::Error::new(io::ErrorKind::InvalidInput, error))?;
        validate_kernel_model(&kernel, &config.model)?;
        let effective_hover_bits = kernel.cfg().hover_thrust_norm.0.to_bits();
        let hover_initialization =
            config
                .hover_initialization
                .unwrap_or(XPlaneHoverInitialization {
                    baseline_force_bits: effective_hover_bits,
                    effective_force_bits: effective_hover_bits,
                    scale_basis_points: 10_000,
                    kernel_config_hash: kernel.cfg().canonical_hash(),
                });
        validate_hover_initialization(&kernel, hover_initialization)?;
        let runtime_identity = RuntimeIdentityGate::new(config.model.clone());
        let tuning_trace = config
            .tuning_trace
            .map(tuning_trace::TuningTracePublisher::connect)
            .transpose()
            .map_err(io::Error::other)?;
        let perturbation = config
            .perturbation
            .clone()
            .map(aviate_hal_xil::perturbation::PerturbationEngine::new)
            .transpose()
            .map_err(|error| io::Error::new(io::ErrorKind::InvalidInput, error))?;
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
            lane_order: config.model.lane_order(),
            motor_count: config.model.motor_count(),
            max_samples_per_iteration: usize::from(config.model.max_samples_per_iteration()),
            model_digest,
            armed: false,
            last_fix: None,
            last_imu: None,
            lane_injection: [0.0; 4],
            wire: wire::WireConstraints::new(config.model.wire()),
            last_packet_at: None,
            last_sample_time_us: None,
            sample_dt_sec: 1.0 / f32::from(config.model.sample_rate_hz()),
            runtime_identity,
            runtime_failure: None,
            control_observations: Vec::with_capacity(
                config.model.max_samples_per_iteration().into(),
            ),
            tuning_trace,
            perturbation,
            perturbation_failure: None,
            perturbation_guard: config.perturbation_guard,
            perturbation_identity_bound: config.perturbation_identity_bound,
            artifact_failure: Arc::new(AtomicBool::new(false)),
            hover_initialization,
            sample_sequence: 0,
            last_answer_armed: None,
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
        self.control_observations.clear();
        if self.artifact_failure.load(Ordering::Acquire) {
            self.terminate();
        }
        let mut last = ActuatorCmd::default();
        let mut answered = 0_usize;
        while let Some(mut packet) = self.hil_backend.poll() {
            let sample_sequence = self.sample_sequence;
            if let Err(error) = self.runtime_identity.observe_timestamp(packet.timestamp_us) {
                warn!("runtime HIL clock rejected: {error}");
                self.runtime_failure = Some(error);
                self.terminate();
            }
            // A gap in the bridge's sample stream is the first half of
            // every lockstep wedge post-mortem; record it at the
            // moment it closes, with its width.
            let now = std::time::Instant::now();
            if let Some(at) = self.last_packet_at {
                let gap = now.duration_since(at);
                if gap > std::time::Duration::from_millis(300) {
                    warn!(
                        "sensor stream resumed after a {:.2}s gap",
                        gap.as_secs_f32()
                    );
                }
            }
            self.last_packet_at = Some(now);
            if let Some(prev) = self.last_sample_time_us {
                self.sample_dt_sec =
                    (packet.timestamp_us.saturating_sub(prev) as f32) / 1_000_000.0;
            }
            self.last_sample_time_us = Some(packet.timestamp_us);
            if let Some(gnss) = packet.gnss {
                self.last_fix = Some(gnss);
            }
            if let Some(imu) = packet.imu {
                self.last_imu = Some(imu);
            }
            let sensor_application = match self.perturbation.as_mut() {
                Some(engine) => match engine.apply_sensor(sample_sequence, &mut packet) {
                    Ok(application) => Some(application),
                    Err(error) => {
                        warn!("sensor perturbation failed: {error}");
                        self.perturbation_failure = Some(error);
                        self.terminate();
                        None
                    }
                },
                None => None,
            };
            self.runner.transport.feed_sensor_packet(&packet);
            let arm_authorizer = self.arm_authorizer();
            let was_armed = self.runner.is_armed();
            last = self.runner.step_with_arm_authorizer(&arm_authorizer);
            if self.artifact_failure.load(Ordering::Acquire) {
                self.terminate();
            }
            let is_armed = self.runner.is_armed();
            if !was_armed && is_armed {
                self.wire.arm(self.last_fix.map(|fix| fix.alt_m));
            }
            self.armed = is_armed;
            self.answer_sample(
                sample_sequence,
                packet.timestamp_us,
                packet.imu,
                &last,
                sensor_application,
            );
            self.sample_sequence = self.sample_sequence.wrapping_add(1);
            answered = answered.wrapping_add(1);
            // A bounded drain keeps one iteration from monopolizing the
            // loop when the link floods.
            if answered >= self.max_samples_per_iteration {
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

    /// Return the identity of the plant-protection boundary in use.
    #[must_use]
    pub fn model_digest(&self) -> XPlaneModelDigest {
        self.model_digest
    }

    /// Verify one runtime bridge and aircraft identity.
    ///
    /// # Errors
    ///
    /// Returns an error when any runtime field differs from the model.
    pub fn accept_runtime_handshake(
        &mut self,
        handshake: XPlaneRuntimeHandshake,
    ) -> Result<(), RuntimeHandshakeError> {
        self.runtime_identity.accept(handshake)
    }

    /// Return the verified runtime identity.
    #[must_use]
    pub fn runtime_handshake(&self) -> Option<&XPlaneRuntimeHandshake> {
        self.runtime_identity.verified()
    }

    /// Return a permanent HIL clock failure for this run.
    #[must_use]
    pub fn runtime_handshake_failure(&self) -> Option<&RuntimeHandshakeError> {
        self.runtime_failure.as_ref()
    }

    /// Return a permanent tuning trace failure for this run.
    #[must_use]
    pub fn tuning_trace_failure(&self) -> Option<&TuningTraceError> {
        self.tuning_trace
            .as_ref()
            .and_then(tuning_trace::TuningTracePublisher::failure)
    }

    /// Return a permanent perturbation failure for this run.
    #[must_use]
    pub fn perturbation_failure(&self) -> Option<&aviate_hal_xil::perturbation::PerturbationError> {
        self.perturbation_failure.as_ref()
    }

    /// Return true when Arm-time verification found a changed artifact.
    #[must_use]
    pub fn perturbation_artifact_failed(&self) -> bool {
        self.artifact_failure.load(Ordering::Acquire)
    }

    /// Take all causal observations produced by the most recent step.
    #[must_use]
    pub fn take_control_observations(&mut self) -> Vec<XPlaneControlObservation> {
        core::mem::take(&mut self.control_observations)
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

    /// Takes the bridge's latest ground-truth state, when it sent one.
    /// Simulation truth exists so an estimate can be judged against
    /// it; the SITL app forwards it on the estimate stream under its
    /// own message id, and flight builds have no such stream to carry.
    pub fn take_truth(
        &mut self,
    ) -> Option<aviate_backend_mavlink_hil::messages::HilStateQuaternion> {
        self.hil_backend.take_state_quaternion()
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
        aviate_runtime::ArmAuthorizer::authorize_arm(&self.arm_authorizer())?;
        info!(
            "Arm command (state={:?})",
            self.runner.kernel.state.init_state
        );
        self.runner.kernel.arm()?;
        self.runner.board_hal.arm();
        self.runner.transport.set_armed(true);
        self.armed = true;
        self.wire.arm(self.last_fix.map(|fix| fix.alt_m));
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
        self.runtime_identity.is_verified()
            && self
                .tuning_trace
                .as_ref()
                .is_none_or(tuning_trace::TuningTracePublisher::is_ready)
            && self.runner.kernel.is_ready()
            && !self.perturbation_artifact_failed()
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
        self.hil_backend.send_heartbeat(self.armed).ok();
    }
}

#[cfg(test)]
mod tests;
