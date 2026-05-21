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

        // Above this collective the cascaded axis controller
        // engages; below it the kernel falls back to open-loop
        // thrust with zero axis input. The bar is set just above
        // the disarmed/on-ground regime: at near-zero collective
        // the mixer cannot add corrective torque without saturating
        // individual motors, and running the loop while the
        // chassis is on the ground spins one diagonal of motors
        // pure-torque against the surface. Once the operator has
        // commanded enough thrust to imply "we want to fly",
        // engage the cascade — including during sub-hover descent,
        // where active rate damping is what keeps the vehicle from
        // diverging (passive stability is not a property of a
        // multirotor in free fall).
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

        // 5. Output. Roll/pitch bounded by a collective-aware cap
        // so a saturated cascade cannot drive the mixer into a
        // pure-torque-pair regime (two motors at zero, two at max
        // — attitude corrects at the cost of all thrust, then the
        // vehicle falls). The cap leaves a floor of headroom so
        // the mixer never asks a motor for negative thrust.
        //
        // Yaw gets a separate, larger cap. The yaw axis enters the
        // mixer with the *same* sign on the two motors of a
        // diagonal (m0+m1 both gain +y, m2+m3 both lose +y), so
        // saturation here clips two motors at once and the body
        // still gets a useful net yaw torque even past the mixer
        // limit. Constraining yaw to the collective-aware cap
        // starves it of authority during hover (cap shrinks to
        // ~0.05 at 0.85 thrust) — the vehicle picks up small yaw
        // disturbances and rotates without ever catching up.
        const AXIS_PRIORITY_FLOOR: f32 = 0.1;
        const YAW_AUTHORITY_CAP: f32 = 0.3;
        let rp_cap = (collective_sp.0 - AXIS_PRIORITY_FLOOR)
            .min(1.0 - collective_sp.0 - AXIS_PRIORITY_FLOOR)
            .max(0.05);
        let yaw_axis = torque_norm[2].0.clamp(-YAW_AUTHORITY_CAP, YAW_AUTHORITY_CAP);
        AxisCommand {
            roll: crate::types::NormalizedSigned(torque_norm[0].0.clamp(-rp_cap, rp_cap)),
            pitch: crate::types::NormalizedSigned(torque_norm[1].0.clamp(-rp_cap, rp_cap)),
            yaw: crate::types::NormalizedSigned(yaw_axis),
            collective: collective_sp,
        }
    }
}
