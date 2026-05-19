use crate::control::attitude::AttitudeController;
use crate::control::position::PositionController;
use crate::control::rate::RateController;
use crate::control::runtime::NoControllerState;
use crate::control::velocity::VelocityController;
use crate::control::{AxisCommand, Command, ConfigMode, Limits, Scalar, VehicleController};
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
        Self::with_hover_thrust(0.5)
    }
}

impl MultirotorController {
    /// Construct a multirotor controller with the airframe's hover-trim
    /// value. The vertical velocity loop commands collective-thrust
    /// corrections around this point; setting it correctly is the
    /// difference between a vehicle that holds altitude and one that
    /// sinks under closed-loop control. See `ResolvedKernelConfig.
    /// hover_thrust_norm` for the canonical record.
    pub fn with_hover_thrust(hover_thrust_norm: Scalar) -> Self {
        // Default gains. These are the same as the pre-Phase-X
        // values that hover_trim_check passed against; closed-loop
        // position-target stability needs further tuning and is
        // tracked separately.
        Self {
            pos_ctrl: PositionController::new([0.2, 0.2, 0.5]),
            vel_ctrl: VelocityController::new([0.1, 0.1, 0.2], 0.349, hover_thrust_norm),
            rate_ctrl: RateController::new([0.15, 0.15, 0.2]),
            att_ctrl: AttitudeController::new([6.0, 6.0, 2.0]),
        }
    }
}

impl VehicleController for MultirotorController {
    type RuntimeState = NoControllerState;

    // Registered in cert/algorithm_id_registry.toml as
    // "controller.multirotor.v1".
    const ALGORITHM_ID: u64 = 0x4354_4C4D_5552_5631; // "CTLMURV1"

    fn step(
        &self,
        _runtime: &mut NoControllerState,
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

        // Below the minimum-thrust gate the axis loops are suppressed.
        // With near-zero collective the vehicle is on the ground; running
        // the attitude/rate loop on small EKF-attitude errors with no
        // lift would yaw the chassis against the ground (one diagonal
        // of motors firing while the other is idle is a pure torque
        // pair — no lift, just spin). The closed loop only engages
        // once the operator has commanded enough thrust to imply
        // "we want to fly".
        // Axis control gate.
        //
        // **Currently set above [0, 1]** so the closed-loop
        // attitude / rate cascade is never invoked — the X500
        // tumbles within ~1.5 s of takeoff under the current
        // att/rate gain set + EKF behavior (tracked on
        // DRQ-CTL-003). With the cascade out of the picture the
        // X500 is passively stable: symmetric thrust on a balanced
        // airframe with no torque inputs means no body rotation,
        // and the vehicle flies straight up / down.
        //
        // To re-engage the cascade once DRQ-CTL-003 closes, lower
        // this constant to 0.1 (or whatever the closed loop's
        // hover-trim minimum is). The cascaded path below is
        // wired and unit-tested — it is simply not driven from
        // here today.
        const MIN_THRUST_FOR_AXIS_CONTROL: Scalar = 2.0;
        if collective_sp.0 < MIN_THRUST_FOR_AXIS_CONTROL {
            return AxisCommand {
                roll: crate::types::NormalizedSigned(0.0),
                pitch: crate::types::NormalizedSigned(0.0),
                yaw: crate::types::NormalizedSigned(0.0),
                collective: collective_sp,
            };
        }

        // 3. Attitude Control
        let rate_sp = self.att_ctrl.step(&att_sp, &state.attitude);

        // 4. Rate Control
        let torque_norm = self.rate_ctrl.step(rate_sp, state.angular_velocity);

        // 5. Output. Yaw is bounded so it cannot overwhelm collective
        // thrust in the mixer; see DRQ-CTL-003 for the workaround
        // rationale.
        const YAW_PRIORITY_CAP: f32 = 0.2;
        AxisCommand {
            roll: torque_norm[0],
            pitch: torque_norm[1],
            yaw: crate::types::NormalizedSigned(
                torque_norm[2].0.clamp(-YAW_PRIORITY_CAP, YAW_PRIORITY_CAP),
            ),
            collective: collective_sp,
        }
    }
}
