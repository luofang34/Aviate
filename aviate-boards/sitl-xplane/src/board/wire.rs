//! The wire constraints: everything this board enforces on a command
//! between the mixer and the bridge, as a pure state machine so the
//! plant-protection math is testable without a TCP link.

use aviate_config::xplane_model::XPlaneWireModel;

/// The constraint state that survives between samples.
#[derive(Debug, Clone)]
pub(crate) struct WireConstraints {
    config: XPlaneWireModel,
    /// The collective mean this state last let onto the wire — the
    /// ACTUAL wire mean, so the spool bookkeeping can never believe a
    /// ramp happened that the rotors did not see.
    last_collective: f32,
    /// GPS altitude captured at arming (or at the first fix after, if
    /// arming preceded the receiver's first solution — a vehicle that
    /// can never satisfy the climb-clear test would otherwise fly its
    /// whole flight at half attitude authority).
    ground_alt: Option<f32>,
    airborne: bool,
}

impl WireConstraints {
    pub(crate) fn new(config: XPlaneWireModel) -> Self {
        Self {
            config,
            last_collective: 0.0,
            ground_alt: None,
            airborne: false,
        }
    }

    /// Marks an arming event: the current fix (when one exists) is the
    /// ground reference and the vehicle is on its gear.
    pub(crate) fn arm(&mut self, fix_alt_m: Option<f32>) {
        self.ground_alt = fix_alt_m;
        self.airborne = false;
    }

    /// Constrains one mixed command in place. `dt_sec` is SAMPLE time,
    /// not wall time: the stall latch is a property of the simulated
    /// plant, and under simulator time dilation a wall clock would let
    /// the sim-time rise rate scale past the measured margin.
    pub(crate) fn constrain(
        &mut self,
        outputs: &mut [f32; 16],
        count: u8,
        armed: bool,
        fix_alt_m: Option<f32>,
        dt_sec: f32,
    ) {
        let lanes = usize::from(count).clamp(1, 4);
        let dt = dt_sec.clamp(0.0, 0.05);
        let mean: f32 = outputs[..lanes].iter().sum::<f32>() / lanes as f32;
        let rise = if self.last_collective < self.config.band_boundary {
            self.config.low_band_rise_per_s
        } else {
            self.config.rise_per_s
        };
        let allowed = if mean > self.last_collective {
            (self.last_collective + rise * dt).min(mean)
        } else if armed {
            (self.last_collective - self.config.fall_per_s * dt).max(mean)
        } else {
            mean
        }
        .min(self.config.mean_ceiling);
        let shift = allowed - mean;
        if shift != 0.0 {
            // Both directions: the rise limit pulls a too-fast climb
            // down, and the fall limit props a too-fast cut up.
            for lane in &mut outputs[..lanes] {
                *lane = (*lane + shift).clamp(0.0, 1.0);
            }
        }
        // After the shift the wire mean is `allowed`, up to per-lane
        // clamping — whose error is conservative for the rise limit
        // (the bookkeeping never believes a faster rise than the wire
        // saw).
        self.last_collective = allowed;

        // Per-lane stall ceiling: scale every deviation by one factor,
        // so the commanded moment keeps its direction and the mix its
        // shape; only the magnitude yields.
        let mean_now: f32 = outputs[..lanes].iter().sum::<f32>() / lanes as f32;
        let mut squeeze = 1.0f32;
        for lane in &outputs[..lanes] {
            let dev = *lane - mean_now;
            if dev > 0.0 && mean_now + dev > self.config.lane_ceiling {
                squeeze = squeeze.min((self.config.lane_ceiling - mean_now) / dev);
            }
            if dev < 0.0 && mean_now + dev < 0.0 {
                squeeze = squeeze.min(mean_now / -dev);
            }
        }
        if squeeze < 1.0 {
            for lane in &mut outputs[..lanes] {
                *lane = mean_now + (*lane - mean_now) * squeeze;
            }
        }

        // Until the vehicle has climbed clear of its arming altitude,
        // the differential squeezes toward the mean: on its gear the
        // attitude is held by the ground, and per-lane dither re-trips
        // blades into stall the way a symmetric ramp does not.
        // One-way: full authority from the moment it is airborne.
        if !self.airborne {
            if self.ground_alt.is_none() {
                // Armed before the first fix: the first solution seen
                // afterward is the ground.
                self.ground_alt = fix_alt_m;
            }
            let clear = match (self.ground_alt, fix_alt_m) {
                (Some(ground), Some(alt)) => alt > ground + self.config.airborne_clearance_m,
                _ => false,
            };
            if clear {
                self.airborne = true;
            } else {
                let new_mean: f32 = outputs[..lanes].iter().sum::<f32>() / lanes as f32;
                for lane in &mut outputs[..lanes] {
                    *lane = (new_mean + (*lane - new_mean) * self.config.ground_squeeze)
                        .clamp(0.0, 1.0);
                }
            }
        }
    }
}

#[cfg(test)]
mod tests;
