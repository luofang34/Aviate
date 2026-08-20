//! The wire constraints: everything this board enforces on a command
//! between the mixer and the bridge, as a pure state machine so the
//! plant-protection math is testable without a TCP link.

/// Maximum collective rise in the working band, force-domain units per
/// second — paced to the rotors' RPM inertia so blade angle of attack
/// never outruns rotor speed. The bracketing is empirical and
/// consistent across every flight of the night: staircase ramps at
/// 0.036/s always spool cleanly, ramps at 0.08/s and above always
/// leave the props partially stalled and the vehicle perched at
/// "full" thrust.
const RISE_PER_S: f32 = 0.035;
/// Below this collective the blades are lightly loaded and the stall
/// latch has never been observed; that band rises faster so an armed
/// vehicle answers the stick in seconds rather than reading as dead.
const BAND_BOUNDARY: f32 = 0.40;
/// Rise rate inside the lightly-loaded band.
const LOW_BAND_RISE_PER_S: f32 = 0.15;
/// Collective mean ceiling, the more important half of the spool
/// constraint: the prop model's thrust COLLAPSES under a sustained
/// high command (measured: force 1.0 held on the ground reads
/// near-zero prop force — blade stall latched by RPM that can no
/// longer catch up). Hover sits near 0.43; the ceiling keeps every
/// commander out of the latch while reserving differential headroom.
const MEAN_CEILING: f32 = 0.55;
/// Per-lane ceiling, below the ~0.9 latch threshold with margin. The
/// mean ceiling alone cannot keep a lane healthy: differentials stack
/// on top of the mean, and a railed lane latches its prop — a single
/// latched prop is an asymmetric-thrust departure, not a soft limit.
const LANE_CEILING: f32 = 0.85;
/// How far above the arming altitude the vehicle must climb before
/// the on-gear differential squeeze releases.
const AIRBORNE_CLEARANCE_M: f32 = 0.5;
/// The on-gear differential retention: half authority, not near-zero
/// — wind grabs the vehicle the moment it unloads its gear, and a
/// liftoff with no attitude authority is swept downwind faster than
/// any later correction can recover. Half keeps the anti-stall dither
/// suppression while leaving the wind something to fight it with.
const GROUND_SQUEEZE: f32 = 0.5;

/// The constraint state that survives between samples.
#[derive(Debug, Clone)]
pub(crate) struct WireConstraints {
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
    pub(crate) fn new() -> Self {
        Self {
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
        fix_alt_m: Option<f32>,
        dt_sec: f32,
    ) {
        let lanes = usize::from(count).clamp(1, 4);
        let dt = dt_sec.clamp(0.0, 0.05);
        let mean: f32 = outputs[..lanes].iter().sum::<f32>() / lanes as f32;
        let rise = if self.last_collective < BAND_BOUNDARY {
            LOW_BAND_RISE_PER_S
        } else {
            RISE_PER_S
        };
        let allowed = if mean > self.last_collective {
            (self.last_collective + rise * dt).min(mean)
        } else {
            mean
        }
        .min(MEAN_CEILING);
        let shift = allowed - mean;
        if shift < 0.0 {
            for lane in &mut outputs[..lanes] {
                *lane = (*lane + shift).clamp(0.0, 1.0);
            }
        }
        // `allowed <= mean` always (the ramp only limits rises), so
        // after the shift the wire mean IS `allowed`.
        self.last_collective = allowed;

        // Per-lane stall ceiling: scale every deviation by one factor,
        // so the commanded moment keeps its direction and the mix its
        // shape; only the magnitude yields.
        let mean_now: f32 = outputs[..lanes].iter().sum::<f32>() / lanes as f32;
        let mut squeeze = 1.0f32;
        for lane in &outputs[..lanes] {
            let dev = *lane - mean_now;
            if dev > 0.0 && mean_now + dev > LANE_CEILING {
                squeeze = squeeze.min((LANE_CEILING - mean_now) / dev);
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
                (Some(ground), Some(alt)) => alt > ground + AIRBORNE_CLEARANCE_M,
                _ => false,
            };
            if clear {
                self.airborne = true;
            } else {
                let new_mean: f32 = outputs[..lanes].iter().sum::<f32>() / lanes as f32;
                for lane in &mut outputs[..lanes] {
                    *lane = (new_mean + (*lane - new_mean) * GROUND_SQUEEZE).clamp(0.0, 1.0);
                }
            }
        }
    }
}

#[cfg(test)]
mod tests;
