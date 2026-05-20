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
        // Tuned for X500 + gz + quaternion-derived gyro. Two
        // properties matter most for closed-loop stability here:
        //
        //   1. **Rate inner loop fast vs attitude outer loop.**
        //      Att gain 0.5 + rate gain 0.5 means a 1 rad
        //      attitude error commands at most a 0.5 rad/s rate
        //      setpoint, and the rate loop produces at most a
        //      0.5 motor command per rad/s of rate error. Both
        //      are well below saturation.
        //
        //   2. **Bounded attitude setpoint from velocity loop.**
        //      `max_roll_pitch: 0.175 rad` (≈10°) caps how
        //      aggressively the velocity controller can tilt the
        //      body in response to a horizontal position error.
        //      Without this cap, a large drift commanded a 60°
        //      tilt that the rate loop could not track without
        //      saturating the mixer.
        Self {
            pos_ctrl: PositionController::new([0.3, 0.3, 0.6]),
            vel_ctrl: VelocityController::new([0.25, 0.25, 0.3], 0.175, hover_thrust_norm),
            rate_ctrl: RateController::new([0.5, 0.5, 0.3]),
            att_ctrl: AttitudeController::new([0.5, 0.5, 0.2]),
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
        // **Set above [0, 1]** so the closed-loop attitude / rate
        // cascade is never invoked from this controller today —
        // even with the gyro frame conversion fixed and the X500
        // mixer correct, the cascade tumbles the vehicle in
        // closed-loop operation (DRQ-CTL-003 carries the
        // remaining EKF / controller debugging work). With the
        // cascade gated off, the X500 flies passively stable
        // for vertical mission profiles.
        //
        // Lowering this to 0.1 re-engages the cascade.
        const MIN_THRUST_FOR_AXIS_CONTROL: Scalar = 0.1;
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
