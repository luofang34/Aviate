//! One multirotor controller cycle and its diagnostic witness.

use crate::control::law_invariants::DISARMED_COLLECTIVE_THRESHOLD;
use crate::control::velocity::AccelFeedforward;
use crate::control::{
    AxisCommand, Command, ControllerLoopObservation, ControllerStep, ControllerStepObservation,
    EffectiveControlTopology, EffectiveSetpointObservation, IntegratorAction, Limits,
    MultirotorControllerObservation, OuterLoopSelection, VehicleControlMode,
};
use crate::math::{Quaternion, Vector3};
use crate::state::StateEstimate;
use crate::types::{MetersPerSecond, MetersPerSecondSquared, NormalizedThrust, Scalar};

use super::{attitude_with_heading, MultirotorController, MultirotorRuntimeState};

struct PreparedCascade {
    topology: EffectiveControlTopology,
    setpoints: EffectiveSetpointObservation,
    velocity_loop: ControllerLoopObservation,
}

impl MultirotorController {
    pub(super) fn run_step(
        &self,
        runtime: &mut MultirotorRuntimeState,
        state: &StateEstimate,
        command: &Command,
        flags: &VehicleControlMode,
        limits: &Limits,
    ) -> ControllerStep {
        let previous_mode = runtime.previous_effective_mode;
        let previous_topology = runtime.previous_topology;
        let topology = flags.effective_topology(command.mode, &command.setpoint);
        if topology == EffectiveControlTopology::Unsupported {
            return self.reject_unsupported(runtime, command, previous_mode, previous_topology);
        }

        align_topology_state(runtime, state, topology);
        let mut prepared = self.prepare_cascade(runtime, state, command, flags, limits, topology);
        apply_automatic_fallback(self, runtime, state, flags, &mut prepared);
        if prepared.setpoints.collective.0 < DISARMED_COLLECTIVE_THRESHOLD {
            return self.reset_for_zero_thrust(
                runtime,
                command,
                previous_mode,
                previous_topology,
                prepared,
            );
        }

        let (axis_command, rate_loop) = self.run_inner_loops(runtime, state, &mut prepared);
        record_step(
            runtime,
            command,
            previous_mode,
            previous_topology,
            prepared,
            rate_loop,
            axis_command,
        )
    }

    fn prepare_cascade(
        &self,
        runtime: &mut MultirotorRuntimeState,
        state: &StateEstimate,
        command: &Command,
        flags: &VehicleControlMode,
        limits: &Limits,
        topology: EffectiveControlTopology,
    ) -> PreparedCascade {
        let mut prepared = PreparedCascade {
            topology,
            setpoints: EffectiveSetpointObservation {
                attitude: command.setpoint.attitude.unwrap_or(Quaternion::IDENTITY),
                collective: command.setpoint.collective_thrust,
                ..Default::default()
            },
            velocity_loop: inactive_velocity_observation(runtime),
        };
        match flags.outer_loop(&command.setpoint) {
            OuterLoopSelection::Position(position) => {
                prepared.setpoints.position_ned = Some(position);
                let target = Vector3::new(position[0], position[1], position[2]);
                let current = Vector3::new(
                    state.position_ned[0],
                    state.position_ned[1],
                    state.position_ned[2],
                );
                let velocity = self.pos_ctrl.step(target, current);
                self.run_velocity_loop(runtime, state, command, velocity, &mut prepared);
            }
            OuterLoopSelection::Velocity(velocity) => {
                let velocity = Vector3::new(velocity[0], velocity[1], velocity[2]);
                self.run_velocity_loop(runtime, state, command, velocity, &mut prepared);
            }
            OuterLoopSelection::None if flags.flag_control_altitude_enabled => {
                self.run_vertical_loop(runtime, state, command, limits, &mut prepared);
            }
            OuterLoopSelection::None => runtime.vel_sp_primed = false,
        }
        prepared
    }

    fn run_velocity_loop(
        &self,
        runtime: &mut MultirotorRuntimeState,
        state: &StateEstimate,
        command: &Command,
        velocity: Vector3<MetersPerSecond>,
        prepared: &mut PreparedCascade,
    ) {
        let acceleration = acceleration_feedforward(runtime, velocity);
        runtime.last_vel_sp_ned = velocity;
        runtime.vel_sp_primed = true;
        let current = Vector3::new(
            state.velocity_ned[0],
            state.velocity_ned[1],
            state.velocity_ned[2],
        );
        let (output, observation) = self.vel_ctrl.step_with_observation(
            &mut runtime.velocity_loop,
            velocity,
            current,
            AccelFeedforward {
                accel_ned: acceleration,
            },
            &state.attitude,
            command.setpoint.heading,
            runtime.dt_sec,
        );
        prepared.setpoints.velocity_ned = Some([velocity.x, velocity.y, velocity.z]);
        prepared.setpoints.acceleration_ned = [acceleration.x, acceleration.y, acceleration.z];
        prepared.setpoints.collective = output.collective;
        prepared.setpoints.attitude = output.attitude;
        prepared.velocity_loop = observation;
    }

    fn run_vertical_loop(
        &self,
        runtime: &mut MultirotorRuntimeState,
        state: &StateEstimate,
        command: &Command,
        limits: &Limits,
        prepared: &mut PreparedCascade,
    ) {
        if let Some(vertical) = self.vertical_velocity_setpoint(&command.setpoint, state, limits) {
            let zero = MetersPerSecond(0.0);
            let target = Vector3::new(zero, zero, vertical);
            let current = Vector3::new(zero, zero, state.velocity_ned[2]);
            let (output, observation) = self.vel_ctrl.step_with_observation(
                &mut runtime.velocity_loop,
                target,
                current,
                AccelFeedforward::default(),
                &state.attitude,
                None,
                runtime.dt_sec,
            );
            prepared.setpoints.velocity_ned = Some([zero, zero, vertical]);
            prepared.setpoints.collective = output.collective;
            prepared.velocity_loop = observation;
        }
        if let Some(heading) = command.setpoint.heading {
            prepared.setpoints.attitude =
                attitude_with_heading(&prepared.setpoints.attitude, heading);
        }
        runtime.vel_sp_primed = false;
    }

    fn run_inner_loops(
        &self,
        runtime: &mut MultirotorRuntimeState,
        state: &StateEstimate,
        prepared: &mut PreparedCascade,
    ) -> (AxisCommand, ControllerLoopObservation) {
        let rate_setpoint = self
            .att_ctrl
            .step(&prepared.setpoints.attitude, &state.attitude);
        let current = [
            state.angular_velocity[0],
            state.angular_velocity[1],
            state.angular_velocity[2],
        ];
        let (torque, observation) = self.rate_ctrl.step_with_observation(
            &mut runtime.rate_loop,
            rate_setpoint,
            current,
            runtime.dt_sec,
        );
        prepared.setpoints.angular_rate = rate_setpoint;
        (
            AxisCommand {
                roll: torque[0],
                pitch: torque[1],
                yaw: torque[2],
                collective: prepared.setpoints.collective,
            },
            observation,
        )
    }

    fn reset_for_zero_thrust(
        &self,
        runtime: &mut MultirotorRuntimeState,
        command: &Command,
        previous_mode: Option<crate::control::ControlMode>,
        previous_topology: Option<EffectiveControlTopology>,
        mut prepared: PreparedCascade,
    ) -> ControllerStep {
        let rate_before = runtime.rate_loop.integral;
        runtime.velocity_loop.reset();
        runtime.rate_loop.reset();
        runtime.vel_sp_primed = false;
        prepared.topology = EffectiveControlTopology::ZeroThrust;
        prepared.velocity_loop.integrator_after = [0.0; 3];
        prepared.velocity_loop.integrator_action = [IntegratorAction::Reset; 3];
        let axis_command = AxisCommand {
            collective: prepared.setpoints.collective,
            ..Default::default()
        };
        record_step(
            runtime,
            command,
            previous_mode,
            previous_topology,
            prepared,
            reset_observation(rate_before),
            axis_command,
        )
    }

    fn reject_unsupported(
        &self,
        runtime: &mut MultirotorRuntimeState,
        command: &Command,
        previous_mode: Option<crate::control::ControlMode>,
        previous_topology: Option<EffectiveControlTopology>,
    ) -> ControllerStep {
        let axis_command = AxisCommand::default();
        let velocity_loop = inactive_velocity_observation(runtime);
        let rate_loop = inactive_observation(runtime.rate_loop.integral);
        runtime.previous_effective_mode = Some(command.mode);
        runtime.previous_topology = Some(EffectiveControlTopology::Unsupported);
        runtime.last_axis_command = axis_command;
        runtime.axis_command_primed = true;
        ControllerStep {
            axis_command,
            observation: ControllerStepObservation::from_multirotor(
                MultirotorControllerObservation {
                    previous_mode,
                    current_mode: command.mode,
                    previous_topology,
                    current_topology: EffectiveControlTopology::Unsupported,
                    setpoints: EffectiveSetpointObservation::default(),
                    velocity_loop,
                    rate_loop,
                    axis_command,
                },
            ),
        }
    }
}

fn acceleration_feedforward(
    runtime: &MultirotorRuntimeState,
    velocity: Vector3<MetersPerSecond>,
) -> Vector3<MetersPerSecondSquared> {
    if !runtime.vel_sp_primed || runtime.dt_sec <= 0.0 {
        return Vector3::new(
            MetersPerSecondSquared(0.0),
            MetersPerSecondSquared(0.0),
            MetersPerSecondSquared(0.0),
        );
    }
    Vector3::new(
        MetersPerSecondSquared((velocity.x.0 - runtime.last_vel_sp_ned.x.0) / runtime.dt_sec),
        MetersPerSecondSquared((velocity.y.0 - runtime.last_vel_sp_ned.y.0) / runtime.dt_sec),
        MetersPerSecondSquared((velocity.z.0 - runtime.last_vel_sp_ned.z.0) / runtime.dt_sec),
    )
}

fn align_topology_state(
    runtime: &mut MultirotorRuntimeState,
    state: &StateEstimate,
    topology: EffectiveControlTopology,
) {
    if runtime.previous_topology == Some(topology) {
        return;
    }
    if runtime.previous_topology == Some(EffectiveControlTopology::Unsupported) {
        runtime.rate_loop.meas_filtered_prev = Vector3::new(
            state.angular_velocity[0],
            state.angular_velocity[1],
            state.angular_velocity[2],
        );
        runtime.rate_loop.primed = false;
    }
    runtime.vel_sp_primed = false;
    if matches!(
        topology,
        EffectiveControlTopology::Position
            | EffectiveControlTopology::Velocity
            | EffectiveControlTopology::Vertical
    ) {
        runtime.velocity_loop.last_vel_filt_ned = Vector3::new(
            state.velocity_ned[0],
            state.velocity_ned[1],
            state.velocity_ned[2],
        );
        runtime.velocity_loop.d_primed = false;
    }
}

fn apply_automatic_fallback(
    controller: &MultirotorController,
    runtime: &MultirotorRuntimeState,
    state: &StateEstimate,
    flags: &VehicleControlMode,
    prepared: &mut PreparedCascade,
) {
    let from_outer_loop = matches!(
        flags.automatic_fallback_from,
        Some(
            crate::control::ControlMode::AltitudeHold
                | crate::control::ControlMode::PositionHold
                | crate::control::ControlMode::VelocityControl
                | crate::control::ControlMode::DeviationTracking
        )
    );
    if prepared.topology != EffectiveControlTopology::Attitude || !from_outer_loop {
        return;
    }
    prepared.setpoints.attitude = state.attitude;
    prepared.setpoints.collective = if runtime.axis_command_primed {
        runtime.last_axis_command.collective
    } else {
        NormalizedThrust(controller.hover_thrust_norm())
    };
}

fn record_step(
    runtime: &mut MultirotorRuntimeState,
    command: &Command,
    previous_mode: Option<crate::control::ControlMode>,
    previous_topology: Option<EffectiveControlTopology>,
    prepared: PreparedCascade,
    rate_loop: ControllerLoopObservation,
    axis_command: AxisCommand,
) -> ControllerStep {
    runtime.previous_effective_mode = Some(command.mode);
    runtime.previous_topology = Some(prepared.topology);
    runtime.last_axis_command = axis_command;
    runtime.axis_command_primed = true;
    ControllerStep {
        axis_command,
        observation: ControllerStepObservation::from_multirotor(MultirotorControllerObservation {
            previous_mode,
            current_mode: command.mode,
            previous_topology,
            current_topology: prepared.topology,
            setpoints: prepared.setpoints,
            velocity_loop: prepared.velocity_loop,
            rate_loop,
            axis_command,
        }),
    }
}

fn inactive_velocity_observation(runtime: &MultirotorRuntimeState) -> ControllerLoopObservation {
    inactive_observation(velocity_integrators(runtime))
}

fn inactive_observation(integrators: [Scalar; 3]) -> ControllerLoopObservation {
    ControllerLoopObservation {
        integrator_before: integrators,
        integrator_after: integrators,
        ..Default::default()
    }
}

fn reset_observation(before: [Scalar; 3]) -> ControllerLoopObservation {
    ControllerLoopObservation {
        integrator_before: before,
        integrator_after: [0.0; 3],
        integrator_action: [IntegratorAction::Reset; 3],
        ..Default::default()
    }
}

fn velocity_integrators(runtime: &MultirotorRuntimeState) -> [Scalar; 3] {
    [
        runtime.velocity_loop.integrator_ned.x.0,
        runtime.velocity_loop.integrator_ned.y.0,
        runtime.velocity_loop.integrator_ned.z.0,
    ]
}
