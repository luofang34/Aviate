use crate::control::attitude::AttitudeController;
use crate::control::position::PositionController;
use crate::control::rate::RateController;
use crate::control::velocity::VelocityController;
use crate::control::{AxisCommand, Command, ConfigMode, Limits, VehicleController};
use crate::math::{Quaternion, Vector3};
use crate::state::StateEstimate;

pub struct MultirotorController {
    pub pos_ctrl: PositionController,
    pub vel_ctrl: VelocityController,
    pub rate_ctrl: RateController,
    pub att_ctrl: AttitudeController,
}

impl Default for MultirotorController {
    fn default() -> Self {
        Self {
            pos_ctrl: PositionController::new([0.2, 0.2, 0.5]), // Default P gains for pos (X,Y,Z)
            vel_ctrl: VelocityController::new([0.1, 0.1, 0.2], 0.349), // Default P gains for vel (X,Y,Z), max 20deg roll/pitch
            rate_ctrl: RateController::new([0.15, 0.15, 0.2]), // Default P gains for rate (R,P,Y)
            att_ctrl: AttitudeController::new([6.0, 6.0, 2.0]), // Default P gains for att (R,P,Y)
        }
    }
}

impl VehicleController for MultirotorController {
    fn step(
        &mut self,
        state: &StateEstimate,
        command: &Command,
        _mode: ConfigMode,
        _limits: &Limits, // Limits can be applied here for safety or in mixer
    ) -> AxisCommand {
        // Assume Command priority: Position > Velocity > Attitude (or internal control)

        let mut collective_sp = command.setpoint.collective_thrust;
        let mut att_sp = command.setpoint.attitude.unwrap_or(Quaternion::IDENTITY); // Default to level if no attitude commanded

        // 1. Position Control (if active)
        if let Some(pos_sp_arr) = command.setpoint.position {
            let pos_sp = Vector3::new(pos_sp_arr[0], pos_sp_arr[1], pos_sp_arr[2]);
            let vel_sp = self.pos_ctrl.step(
                pos_sp,
                Vector3 {
                    x: state.position_ned[0],
                    y: state.position_ned[1],
                    z: state.position_ned[2],
                },
            );
            // 2. Velocity Control (from position control)
            let (col, att) = self.vel_ctrl.step(
                vel_sp,
                Vector3 {
                    x: state.velocity_ned[0],
                    y: state.velocity_ned[1],
                    z: state.velocity_ned[2],
                },
                &state.attitude,
            );
            collective_sp = col;
            att_sp = att;
        } else if let Some(vel_sp_arr) = command.setpoint.velocity {
            let vel_sp = Vector3::new(vel_sp_arr[0], vel_sp_arr[1], vel_sp_arr[2]);
            // 2. Velocity Control (if active)
            let (col, att) = self.vel_ctrl.step(
                vel_sp,
                Vector3 {
                    x: state.velocity_ned[0],
                    y: state.velocity_ned[1],
                    z: state.velocity_ned[2],
                },
                &state.attitude,
            );
            collective_sp = col;
            att_sp = att;
        }

        // 3. Attitude Control
        let rate_sp = self.att_ctrl.step(&att_sp, &state.attitude);

        // 4. Rate Control
        let torque_norm = self.rate_ctrl.step(rate_sp, state.angular_velocity);

        // 5. Output
        AxisCommand {
            roll: torque_norm[0],
            pitch: torque_norm[1],
            yaw: torque_norm[2],
            collective: collective_sp,
        }
    }
}
