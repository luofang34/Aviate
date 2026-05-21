use crate::math::Vector3;
use crate::types::{Meters, MetersPerSecond, Scalar};

#[derive(Clone, Debug)]
pub struct PositionController {
    pub gains: [Scalar; 3], // P gains for X, Y, Z position
}

impl PositionController {
    pub fn new(gains: [Scalar; 3]) -> Self {
        Self { gains }
    }

    pub fn step(
        &self,
        setpoint: Vector3<Meters>,
        current: Vector3<Meters>,
    ) -> Vector3<MetersPerSecond> {
        let error = Vector3 {
            x: setpoint.x.0 - current.x.0,
            y: setpoint.y.0 - current.y.0,
            z: setpoint.z.0 - current.z.0,
        };

        // Per-axis velocity cap. A P-only position controller (no
        // I-term yet — DRQ-CTL-002) overshoots when allowed to
        // command a large velocity, because residual velocity at
        // the moment the position error hits zero carries the
        // vehicle past the target. The caps below are chosen for
        // multirotor airframes; switch to a sqrt-shaped velocity
        // setpoint (PX4 / ArduPilot `sqrt_controller`) for
        // proper energy management once the velocity feedback
        // fidelity question is settled.
        const VEL_CAP_HORIZONTAL: f32 = 2.0;
        const VEL_CAP_VERTICAL: f32 = 1.5;
        Vector3 {
            x: MetersPerSecond(
                (error.x * self.gains[0]).clamp(-VEL_CAP_HORIZONTAL, VEL_CAP_HORIZONTAL),
            ),
            y: MetersPerSecond(
                (error.y * self.gains[1]).clamp(-VEL_CAP_HORIZONTAL, VEL_CAP_HORIZONTAL),
            ),
            z: MetersPerSecond(
                (error.z * self.gains[2]).clamp(-VEL_CAP_VERTICAL, VEL_CAP_VERTICAL),
            ),
        }
    }
}
