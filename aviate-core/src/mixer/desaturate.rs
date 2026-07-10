//! Priority-preserving desaturation for quad mixers.
//!
//! Plain per-motor clamping lets one axis silently steal authority
//! from another: a hot yaw demand pushes a motor past its limit and
//! the clamp truncates whatever else rode on that motor — collective
//! included — so the airframe loses lift while the mixer still
//! nominally "works". Desaturation resolves saturation by explicit
//! priority instead:
//!
//! 1. **Roll/pitch** keep their differential authority, scaled only
//!    if they alone exceed the full actuator span.
//! 2. **Collective** shifts into the interval that keeps every
//!    motor's roll/pitch contribution realizable.
//! 3. **Yaw** is clipped to the headroom that remains.
//!
//! Yaw goes last because a quad's yaw torque comes from rotor drag,
//! which is far weaker than thrust — the controller legitimately
//! commands large yaw outputs, and giving them priority starves
//! lift. A clipped yaw degrades heading tracking for a cycle; a
//! starved collective drops the airframe.

use crate::types::Scalar;

/// Per-motor sign of each axis's contribution for a 4-motor mixer.
pub(super) struct QuadSigns {
    pub roll: [Scalar; 4],
    pub pitch: [Scalar; 4],
    pub yaw: [Scalar; 4],
}

/// Mixes `(collective, roll, pitch, yaw)` through `signs` with
/// roll/pitch > collective > yaw saturation priority. Every output
/// lands in `[0, 1]` by construction; the final clamp only absorbs
/// float rounding.
pub(super) fn mix_desaturated(
    collective: Scalar,
    roll: Scalar,
    pitch: Scalar,
    yaw: Scalar,
    signs: &QuadSigns,
) -> [Scalar; 4] {
    let mut rp = [0.0; 4];
    for (i, v) in rp.iter_mut().enumerate() {
        *v = signs.roll[i] * roll + signs.pitch[i] * pitch;
    }
    let mut rp_min = rp[0];
    let mut rp_max = rp[0];
    for &v in &rp[1..] {
        rp_min = rp_min.min(v);
        rp_max = rp_max.max(v);
    }
    if rp_max - rp_min > 1.0 {
        let s = 1.0 / (rp_max - rp_min);
        for v in &mut rp {
            *v *= s;
        }
        rp_min *= s;
        rp_max *= s;
    }

    // Collective moves inside the interval that keeps every motor's
    // roll/pitch contribution feasible; non-empty because the span
    // is ≤ 1 after scaling. This can raise collective above the
    // commanded value — safe because the controller's thrust gate
    // zeroes roll/pitch/yaw (making this a no-op) whenever the
    // commanded collective is essentially zero.
    let t = collective.max(-rp_min).min(1.0 - rp_max);

    // Yaw takes what headroom remains. Each motor's pre-yaw output
    // `t + rp[i]` is in [0, 1], so every per-motor bound interval
    // contains zero and the max/min chain cannot invert. When the
    // request exceeds that headroom, collective may shift by up to
    // `YAW_COLLECTIVE_MARGIN` toward the span midpoint that widens
    // it: rotor-drag yaw torque is the airframe's only brake against
    // yaw momentum, and refusing it any collective at all leaves the
    // vehicle unable to stop a commanded spin (it coasts through the
    // target by whole revolutions). The margin bounds the thrust sag
    // a hard yaw stop can cause, keeping the collective > roll/pitch
    // ordering intact in the large.
    let t = shift_for_yaw(t, yaw, &rp, signs);
    let (y_lo, y_hi) = yaw_headroom(t, &rp, signs);
    let y = yaw.max(y_lo).min(y_hi);

    let mut out = [0.0; 4];
    for (i, o) in out.iter_mut().enumerate() {
        *o = (t + rp[i] + signs.yaw[i] * y).clamp(0.0, 1.0);
    }
    out
}

/// How much collective a clipped yaw request may pull toward the
/// span midpoint. At the X500's hover trim (~0.77) the full margin
/// raises symmetric yaw headroom from ~0.23 to ~0.38 per motor — a
/// ~65 % stronger brake for a bounded ~20 % thrust dip.
const YAW_COLLECTIVE_MARGIN: Scalar = 0.15;

/// The `[y_lo, y_hi]` yaw contribution interval that keeps every
/// motor's `t + rp[i] + signs.yaw[i] · y` inside `[0, 1]`.
fn yaw_headroom(t: Scalar, rp: &[Scalar; 4], signs: &QuadSigns) -> (Scalar, Scalar) {
    let mut y_lo: Scalar = -1.0;
    let mut y_hi: Scalar = 1.0;
    for (i, &v) in rp.iter().enumerate() {
        let base = t + v;
        if signs.yaw[i] > 0.0 {
            y_lo = y_lo.max(-base);
            y_hi = y_hi.min(1.0 - base);
        } else {
            y_lo = y_lo.max(base - 1.0);
            y_hi = y_hi.min(base);
        }
    }
    (y_lo, y_hi)
}

/// Shifts collective by up to [`YAW_COLLECTIVE_MARGIN`] when the yaw
/// request exceeds its headroom at `t`. The target is the collective
/// that equalizes the tightest pair of per-motor constraints in the
/// requested direction (the concave maximum of the headroom as a
/// function of collective); shifting past it buys nothing.
fn shift_for_yaw(t: Scalar, yaw: Scalar, rp: &[Scalar; 4], signs: &QuadSigns) -> Scalar {
    let (y_lo, y_hi) = yaw_headroom(t, rp, signs);
    if yaw >= y_lo && yaw <= y_hi {
        return t;
    }
    // Tightest roll/pitch offsets among the motors a positive /
    // negative yaw pushes toward each rail.
    let mut rising_max: Scalar = -1.0;
    let mut falling_min: Scalar = 1.0;
    for (i, &v) in rp.iter().enumerate() {
        let rises_for_request = (signs.yaw[i] > 0.0) == (yaw > y_hi);
        if rises_for_request {
            rising_max = rising_max.max(v);
        } else {
            falling_min = falling_min.min(v);
        }
    }
    // `1 − t' − rising_max = t' + falling_min` at the concave peak.
    let ideal = (1.0 - rising_max - falling_min) / 2.0;
    ideal
        .max(t - YAW_COLLECTIVE_MARGIN)
        .min(t + YAW_COLLECTIVE_MARGIN)
        // Never leave the interval that keeps roll/pitch feasible.
        .max(-rp_bound(rp).0)
        .min(1.0 - rp_bound(rp).1)
}

/// `(min, max)` of the four roll/pitch offsets.
fn rp_bound(rp: &[Scalar; 4]) -> (Scalar, Scalar) {
    let mut lo = rp[0];
    let mut hi = rp[0];
    for &v in &rp[1..] {
        lo = lo.min(v);
        hi = hi.max(v);
    }
    (lo, hi)
}

#[cfg(test)]
#[path = "desaturate_tests.rs"]
mod tests;
