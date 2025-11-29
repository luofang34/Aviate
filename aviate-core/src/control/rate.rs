use crate::types::{NormalizedSigned, RadiansPerSecond, Scalar};

#[derive(Clone, Debug)]
pub struct RateController {
    pub gains: [Scalar; 3], // P gains for Roll, Pitch, Yaw
}

impl RateController {
    pub fn new(gains: [Scalar; 3]) -> Self {
        Self { gains }
    }

    pub fn step(
        &self,
        setpoint: [RadiansPerSecond; 3],
        current: [RadiansPerSecond; 3],
    ) -> [NormalizedSigned; 3] {
        let mut output = [NormalizedSigned(0.0); 3];
        for i in 0..3 {
            let error = setpoint[i].0 - current[i].0;
            let cmd = (error * self.gains[i]).clamp(-1.0, 1.0);
            output[i] = NormalizedSigned(cmd);
        }
        output
    }
}
