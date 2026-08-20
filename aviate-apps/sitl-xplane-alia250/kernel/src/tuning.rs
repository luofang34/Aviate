//! The airframe's measured identity and the cascade derived from
//! it: hover trim, per-axis plant authority, and the gains the
//! LLR-CTL-202/205 formulation yields for a heavy payload vehicle.

use aviate_core::control::cascade_gains::CascadeGains;

/// Force-domain hover trim, MEASURED by the grounded collective sweep
/// (`--sweep`) against the simulator's own prop-force dataref in the
/// HEALTHY spool regime: thrust crosses the vehicle's weight near
/// force 0.43 and the vehicle lifts by 0.5. An earlier 0.78 reading
/// was taken with the rotors in the latched-stall state the ceiling in
/// the board now guards against — a trim measured there drives every
/// takeoff back INTO the stall. The trim drifts with battery state
/// (the airframe lightens as it discharges); the vertical loop's
/// integrator absorbs the drift.
const HOVER_TRIM: f32 = 0.43;

/// Reads one plant number from the environment, so a tuning session on
/// a different simulator aircraft can override the baked identity
/// without a rebuild. Flight builds carry no environment plumbing —
/// this app is SITL by definition.
fn env_f32(name: &str, fallback: f32) -> f32 {
    std::env::var(name)
        .ok()
        .and_then(|value| value.parse().ok())
        .unwrap_or(fallback)
}

pub(crate) fn hover_trim() -> f32 {
    env_f32("AVIATE_HOVER_TRIM", HOVER_TRIM)
}

fn plant_k() -> [f32; 3] {
    [
        env_f32("AVIATE_PLANT_K_ROLL", PLANT_K[0]),
        env_f32("AVIATE_PLANT_K_PITCH", PLANT_K[1]),
        env_f32("AVIATE_PLANT_K_YAW", PLANT_K[2]),
    ]
}

/// Measured plant authority per axis, rad/s^2 per unit normalized
/// force-domain torque: the `--identify` flight's output (correlation
/// at the 2.5 rad/s probe, virtual-test-stand run). Roll and yaw are
/// read in the HEALTHY spool regime (hover collective 0.43), where the
/// two probe frequencies agree on each axis's magnitude. These
/// are the ONLY
/// airframe-specific inputs to the attitude cascade; re-run the
/// experiment and update them when the airframe (or the simulator's
/// model of it) changes.
const PLANT_K: [f32; 3] = [5.3, 3.1, 1.0];

/// Attitude-cascade gains derived from the measured plant.
///
/// Design targets, per the LLR-CTL-202/205 formulation the X500
/// derivation uses (closed-loop wn = sqrt(att_p * K * rate_p), zeta =
/// 0.5 * sqrt(K * rate_p / att_p), separation S = K * rate_p / att_p):
///
/// - zeta = 1.25 (overdamped, no overshoot), hence S = 4 * zeta^2 =
///   6.25 — above the 5x separation floor with margin.
/// - wn = 2 rad/s roll/pitch, 1.2 rad/s yaw: a heavy payload vehicle's
///   response (settle ~1.2 s / ~2 s). Deliberately below the probe
///   frequency where the measured phase already shows the rotors'
///   spool lag — crossing over into that lag is what a hotter target
///   would do, and what the first hand-scaled guess died of.
///
/// Which gives, per axis: att_p = wn / sqrt(S), rate_p = S * att_p / K.
/// The rate command cap keeps a six-ton vehicle from being asked to
/// servo fighter rates; the tilt cap bounds horizontal acceleration to
/// what a payload flight needs.
pub(crate) fn alia250_gains() -> CascadeGains {
    const ZETA: f32 = 1.25;
    const SEPARATION: f32 = 4.0 * ZETA * ZETA;
    // Yaw runs far slower than roll/pitch on purpose: its actuator is
    // rotor drag, whose measured lag is already -63 degrees beyond the
    // integrator at 2.5 rad/s — a yaw crossover anywhere near that
    // frequency has no phase margin left. A slow yaw loop also cannot
    // chase the mag heading's tilt-compensation wobble fast enough to
    // destabilize the frame.
    const WN: [f32; 3] = [2.0, 2.0, 0.9];
    let k = plant_k();
    let mut att_p = [0.0_f32; 3];
    let mut rate_p = [0.0_f32; 3];
    for axis in 0..3 {
        att_p[axis] = WN[axis] / SEPARATION.sqrt();
        rate_p[axis] = SEPARATION * att_p[axis] / k[axis];
    }
    CascadeGains {
        att_p,
        att_max_rate_cmd: 1.0,
        rate_p,
        // This airframe hovers TILTED: its center of mass sits well off
        // the rotor centroid (measured as a sustained ~0.2-normalized
        // pitch torque in level hover), and the velocity loop's
        // integrator is what finds that trim attitude. At the X500's
        // 0.05 it takes tens of seconds, during which the vehicle
        // drifts at whatever velocity error the un-trimmed attitude
        // sustains; the conditional anti-windup makes the faster
        // integrator safe.
        vel_i: [0.3, 0.3, 0.1],
        // Damping the long arms would ring with; kept small relative
        // to P so derivative noise cannot dominate the torque demand.
        rate_d: [0.05, 0.05, 0.05],
        // The measured standing yaw torque (rotor reaction imbalance)
        // walks the heading without an integrator; roll and pitch get
        // gentler trims for their own steady asymmetries.
        rate_i: [0.1, 0.1, 0.3],
        vel_max_roll_pitch: 0.45,
        ..CascadeGains::x500_defaults()
    }
}
