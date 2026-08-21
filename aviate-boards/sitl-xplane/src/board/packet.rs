//! One sensor packet to one observed actuator answer.

use aviate_core::ekf::Estimator as _;
use aviate_hal_io::SystemCommand;
use aviate_hal_xil::sim_types::{SimActuatorCmd, SimImuData};
use log::warn;

use super::observation::prepare_actuator_command;
use super::{XPlaneBoard, XPlaneControlObservation};

impl<C, M> XPlaneBoard<C, M>
where
    C: aviate_core::control::VehicleController,
    M: aviate_core::mixer::Mixer,
{
    /// Send and record the command caused by one simulator sample.
    pub(super) fn answer_sample(&mut self, timestamp_us: u64, imu: Option<SimImuData>) {
        let (mut sim_cmd, missing_actuator_answer) =
            if let Some(command) = self.runner.transport.take_actuator_cmd() {
                (command, false)
            } else {
                warn!("kernel produced no actuator answer; sending safe output");
                let safe = SimActuatorCmd {
                    count: self.motor_count,
                    armed: false,
                    ..SimActuatorCmd::default()
                };
                (safe, true)
            };
        let mut prepared = prepare_actuator_command(
            &mut sim_cmd,
            self.lane_injection,
            &mut self.wire,
            self.runner.kernel.cfg().actuator_curve,
            self.lane_order,
            self.motor_count,
            self.last_fix.map(|fix| fix.alt_m),
            self.sample_dt_sec,
        );
        if missing_actuator_answer {
            prepared.constraint_flags.missing_actuator_answer = true;
        }
        let mut observation = XPlaneControlObservation {
            timestamp_us,
            imu,
            pre_wire_force_lanes: prepared.pre_wire_force_lanes,
            applied_force_lanes: prepared.applied_force_lanes,
            constraint_flags: prepared.constraint_flags,
        };
        self.publish_tuning_observation(observation);
        if self.tuning_trace_failure().is_some() {
            observation.constraint_flags.tuning_trace_failure = true;
            sim_cmd.outputs.fill(0.0);
            sim_cmd.armed = false;
            self.terminate();
        }
        self.control_observations.push(observation);
        if !prepared.valid_count {
            warn!(
                "actuator count {} does not match simulator model {}; sending safe output",
                sim_cmd.count, self.motor_count
            );
            if let Err(error) = self.hil_backend.send_actuators(&sim_cmd) {
                warn!("safe actuator command not sent: {error}");
            }
            return;
        }
        if let Err(error) = self.hil_backend.send_actuators(&sim_cmd) {
            warn!("actuator command not sent: {error}");
        }
    }

    fn publish_tuning_observation(&mut self, observation: XPlaneControlObservation) {
        let requested = match self.runner.ingress.setpoint() {
            Some(SystemCommand::FlightControl(command)) => Some(command.clone()),
            _ => None,
        };
        let effective = self.runner.last_effective_command().clone();
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
