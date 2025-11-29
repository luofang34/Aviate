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

        Vector3 {
            x: MetersPerSecond((error.x * self.gains[0]).clamp(-10.0, 10.0)), // Clamp to some reasonable velocity
            y: MetersPerSecond((error.y * self.gains[1]).clamp(-10.0, 10.0)),
            z: MetersPerSecond((error.z * self.gains[2]).clamp(-10.0, 10.0)),
        }
    }
}
