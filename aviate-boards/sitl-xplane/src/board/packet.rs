//! One sensor packet to one observed actuator answer.

use aviate_core::control::CommandSource;
use aviate_core::ekf::Estimator as _;
use aviate_core::mixer::ActuatorCmd;
use aviate_hal_io::SystemCommand;
use aviate_hal_xil::perturbation::{
    ActuatorApplication, ActuatorBypassReason, ActuatorEligibility, SensorApplication,
};
use aviate_hal_xil::sim_types::{SimActuatorCmd, SimImuData, SimSensorPacket};
use log::warn;

use super::observation::{prepare_actuator_command, PreparedActuatorCommand};
use super::{XPlaneBoard, XPlaneControlObservation, XPlaneSendEvidence};

/// What the sensor seam decided about one converted sample.
struct SensorOutcome {
    /// The recorded application, when the seam produced one.
    application: Option<SensorApplication>,
    /// Whether the flight controller may consume the sample.
    accepted: bool,
}

impl SensorOutcome {
    /// No condition artifact is loaded, so the sample passes unchanged.
    const fn unperturbed() -> Self {
        Self {
            application: None,
            accepted: true,
        }
    }

    /// The sample carries the recorded perturbation.
    const fn applied(application: SensorApplication) -> Self {
        Self {
            application: Some(application),
            accepted: true,
        }
    }

    /// The sample failed the sensor contract and never reaches the controller.
    const fn refused() -> Self {
        Self {
            application: None,
            accepted: false,
        }
    }
}

impl<C, M> XPlaneBoard<C, M>
where
    C: aviate_core::control::VehicleController,
    M: aviate_core::mixer::Mixer,
{
    pub(super) fn process_sample(&mut self, mut packet: SimSensorPacket) -> ActuatorCmd {
        let sample_sequence = self.sample_sequence;
        self.observe_sample_time(packet.timestamp_us);
        if let Some(gnss) = packet.gnss {
            self.last_fix = Some(gnss);
        }
        if let Some(imu) = packet.imu {
            self.last_imu = Some(imu);
        }
        let sensor_outcome = self.apply_sensor_perturbation(sample_sequence, &mut packet);
        if sensor_outcome.accepted {
            self.runner.transport.feed_sensor_packet(&packet);
        }
        let sensor_application = sensor_outcome.application;
        let arm_authorizer = self.arm_authorizer();
        let was_armed = self.runner.is_armed();
        let command = self.runner.step_with_arm_authorizer(&arm_authorizer);
        if self
            .artifact_failure
            .load(std::sync::atomic::Ordering::Acquire)
        {
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
            &command,
            sensor_application,
        );
        command
    }

    fn observe_sample_time(&mut self, timestamp_us: u64) {
        if let Err(error) = self.runtime_identity.observe_timestamp(timestamp_us) {
            warn!("runtime HIL clock rejected: {error}");
            self.runtime_failure = Some(error);
            self.terminate();
        }
        let now = std::time::Instant::now();
        if let Some(previous) = self.last_packet_at {
            let gap = now.duration_since(previous);
            if gap > std::time::Duration::from_millis(300) {
                warn!(
                    "sensor stream resumed after a {:.2}s gap",
                    gap.as_secs_f32()
                );
            }
        }
        self.last_packet_at = Some(now);
        if let Some(previous) = self.last_sample_time_us {
            self.sample_dt_sec = (timestamp_us.saturating_sub(previous) as f32) / 1_000_000.0;
        }
        self.last_sample_time_us = Some(timestamp_us);
    }

    fn apply_sensor_perturbation(
        &mut self,
        sample_sequence: u64,
        packet: &mut SimSensorPacket,
    ) -> SensorOutcome {
        let Some(engine) = self.perturbation.as_mut() else {
            return SensorOutcome::unperturbed();
        };
        match engine.apply_sensor(sample_sequence, packet) {
            Ok(application) => SensorOutcome::applied(application),
            Err(error) => {
                warn!("sensor perturbation failed: {error}");
                self.perturbation_failure = Some(error);
                self.terminate();
                SensorOutcome::refused()
            }
        }
    }

    /// Send and record the command caused by one simulator sample.
    pub(super) fn answer_sample(
        &mut self,
        sample_sequence: u64,
        timestamp_us: u64,
        imu: Option<SimImuData>,
        kernel_command: &ActuatorCmd,
        sensor_application: Option<SensorApplication>,
    ) {
        let (mut sim_cmd, prepared, actuator_application) =
            self.prepare_answer(sample_sequence, kernel_command);
        if self.tuning_trace_failure().is_some() {
            sim_cmd.outputs.fill(0.0);
            sim_cmd.armed = false;
            self.terminate();
        }
        if !prepared.valid_count {
            warn!(
                "actuator count {} does not match simulator model {}; sending safe output",
                sim_cmd.count, self.motor_count
            );
        }
        let sent_lanes = first_four(&sim_cmd.outputs);
        let send_result = self.hil_backend.send_actuators_with_receipt(&sim_cmd);
        if let Err(error) = &send_result {
            warn!("actuator command not sent: {error}");
        }
        if actuator_application.is_some() {
            self.complete_perturbation_send(sample_sequence, send_result.is_ok());
        }
        let mut observation = XPlaneControlObservation {
            sample_sequence,
            timestamp_us,
            imu,
            sensor_application,
            actuator_application,
            lane_injection: self.lane_injection,
            fix_altitude_m: self.last_fix.map(|fix| fix.alt_m),
            sample_dt_sec: self.sample_dt_sec,
            pre_wire_force_lanes: prepared.pre_wire_force_lanes,
            applied_force_lanes: prepared.applied_force_lanes,
            sent_lanes,
            send: XPlaneSendEvidence {
                reply_attempted: true,
                reply_succeeded: send_result.is_ok(),
                echoed_timestamp_us: send_result
                    .as_ref()
                    .map_or(timestamp_us, |receipt| receipt.echoed_timestamp_us),
                lockstep: send_result.as_ref().is_ok_and(|receipt| receipt.lockstep),
            },
            hover_initialization: self.hover_initialization,
            constraint_flags: prepared.constraint_flags,
        };
        self.publish_tuning_observation(observation);
        if send_result.is_err() || self.tuning_trace_failure().is_some() {
            observation.constraint_flags.tuning_trace_failure =
                self.tuning_trace_failure().is_some();
            self.terminate();
        }
        self.control_observations.push(observation);
    }

    fn prepare_answer(
        &mut self,
        sample_sequence: u64,
        kernel_command: &ActuatorCmd,
    ) -> (
        SimActuatorCmd,
        PreparedActuatorCommand,
        Option<ActuatorApplication>,
    ) {
        let (mut command, missing_answer) = self.take_actuator_answer();
        let eligibility = actuator_eligibility(
            &command,
            missing_answer,
            self.last_answer_armed,
            self.motor_count,
            kernel_command,
            self.runner.last_effective_command().source,
            self.runner.last_command_provenance().is_some(),
        );
        self.last_answer_armed = Some(command.armed);
        let application = self.apply_actuator_perturbation(
            sample_sequence,
            &mut command,
            eligibility,
            kernel_command.fallback_mask,
        );
        let mut prepared = prepare_actuator_command(
            &mut command,
            self.lane_injection,
            &mut self.wire,
            self.runner.kernel.cfg().actuator_curve,
            self.lane_order,
            self.motor_count,
            self.last_fix.map(|fix| fix.alt_m),
            self.sample_dt_sec,
        );
        prepared.constraint_flags.missing_actuator_answer = missing_answer;
        (command, prepared, application)
    }

    fn take_actuator_answer(&mut self) -> (SimActuatorCmd, bool) {
        if let Some(command) = self.runner.transport.take_actuator_cmd() {
            return (command, false);
        }
        warn!("kernel produced no actuator answer; sending safe output");
        let safe = SimActuatorCmd {
            count: self.motor_count,
            armed: false,
            ..SimActuatorCmd::default()
        };
        (safe, true)
    }

    fn apply_actuator_perturbation(
        &mut self,
        sample_sequence: u64,
        command: &mut SimActuatorCmd,
        eligibility: ActuatorEligibility,
        kernel_fallback_mask: u8,
    ) -> Option<ActuatorApplication> {
        if self.perturbation_failure.is_some() {
            return None;
        }
        let engine = self.perturbation.as_mut()?;
        match engine.apply_actuator(sample_sequence, command, eligibility, kernel_fallback_mask) {
            Ok(application) => Some(application),
            Err(error) => {
                warn!("actuator perturbation failed: {error}");
                self.perturbation_failure = Some(error);
                command.outputs.fill(0.0);
                command.armed = false;
                self.terminate();
                None
            }
        }
    }

    fn complete_perturbation_send(&mut self, sample_sequence: u64, succeeded: bool) {
        let result = self
            .perturbation
            .as_mut()
            .map(|engine| engine.complete_actuator_send(sample_sequence, succeeded));
        if let Some(Err(error)) = result {
            warn!("actuator send evidence failed: {error}");
            self.perturbation_failure = Some(error);
            self.terminate();
        }
    }

    fn publish_tuning_observation(&mut self, observation: XPlaneControlObservation) {
        let requested = match self.runner.ingress.setpoint() {
            Some(SystemCommand::FlightControl(command)) => Some(command.clone()),
            _ => None,
        };
        let effective = self.runner.last_effective_command().clone();
        let command_provenance = self.runner.last_command_provenance();
        let estimate = self
            .runner
            .kernel
            .pipeline()
            .estimator
            .estimate(&self.runner.kernel.state.estimator);
        let armed = self.is_armed();
        if let Some(trace) = self.tuning_trace.as_mut() {
            trace.publish(
                observation,
                requested.as_ref(),
                command_provenance,
                &effective,
                &estimate,
                armed,
            );
            if let Some(error) = trace.failure() {
                warn!("tuning trace failed: {error}");
            }
        }
    }
}

fn actuator_eligibility(
    command: &SimActuatorCmd,
    missing_answer: bool,
    last_armed: Option<bool>,
    motor_count: u8,
    kernel_command: &ActuatorCmd,
    source: CommandSource,
    has_external_provenance: bool,
) -> ActuatorEligibility {
    let reason = if missing_answer {
        Some(ActuatorBypassReason::MissingAnswer)
    } else if command.count != motor_count {
        Some(ActuatorBypassReason::InvalidActuatorCount)
    } else if kernel_command.fallback_mask != 0 {
        Some(ActuatorBypassReason::FallbackMask)
    } else if last_armed.unwrap_or(false) != command.armed {
        Some(ActuatorBypassReason::ArmTransition)
    } else if !command.armed {
        Some(ActuatorBypassReason::Disarmed)
    } else if source == CommandSource::Failsafe {
        Some(ActuatorBypassReason::Failsafe)
    } else if !has_external_provenance {
        Some(ActuatorBypassReason::Direct)
    } else {
        None
    };
    reason.map_or(ActuatorEligibility::Eligible, ActuatorEligibility::Bypass)
}

fn first_four(outputs: &[f32; 16]) -> [f32; 4] {
    [outputs[0], outputs[1], outputs[2], outputs[3]]
}
