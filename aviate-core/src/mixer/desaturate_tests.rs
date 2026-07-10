#![allow(clippy::expect_used, clippy::panic)]

use super::{mix_desaturated, QuadSigns};
use crate::types::Scalar;

/// Sign table matching `QuadXMixer`'s per-motor formulas.
const X_SIGNS: QuadSigns = QuadSigns {
    roll: [-1.0, 1.0, 1.0, -1.0],
    pitch: [1.0, 1.0, -1.0, -1.0],
    yaw: [-1.0, 1.0, 1.0, -1.0],
};

fn assert_close(actual: Scalar, expected: Scalar, label: &str) {
    assert!(
        (actual - expected).abs() < 1e-5,
        "{label}: {actual} != {expected}"
    );
}

#[test]
fn unsaturated_input_matches_the_plain_formula() {
    let (t, r, p, y) = (0.5, 0.1, 0.05, 0.02);
    let out = mix_desaturated(t, r, p, y, &X_SIGNS);
    for (i, &m) in out.iter().enumerate() {
        let plain = t + X_SIGNS.roll[i] * r + X_SIGNS.pitch[i] * p + X_SIGNS.yaw[i] * y;
        assert_close(m, plain, "motor");
    }
}

#[test]
fn hover_with_zero_axes_passes_collective_through() {
    let out = mix_desaturated(0.77, 0.0, 0.0, 0.0, &X_SIGNS);
    for m in out {
        assert_close(m, 0.77, "hover motor");
    }
}

#[test]
fn roll_pitch_span_scales_down_when_it_exceeds_the_actuator_range() {
    // rp = [-0.3, 0.9, 0.3, -0.9]: span 1.8 → scaled by 1/1.8, so the
    // differential *shape* survives while fitting in [0, 1].
    let out = mix_desaturated(0.5, 0.6, 0.3, 0.0, &X_SIGNS);
    assert_close(out[0], 1.0 / 3.0, "m0");
    assert_close(out[1], 1.0, "m1");
    assert_close(out[2], 2.0 / 3.0, "m2");
    assert_close(out[3], 0.0, "m3");
}

#[test]
fn collective_yields_to_roll_at_high_throttle() {
    // Plain clamping would truncate m1/m2 at 1.0 and keep m0/m3 at
    // 0.4, halving the delivered roll moment. Desaturation lowers
    // collective to 0.5 so the full differential of 1.0 survives.
    let out = mix_desaturated(0.9, 0.5, 0.0, 0.0, &X_SIGNS);
    assert_close(out[0], 0.0, "m0");
    assert_close(out[1], 1.0, "m1");
    assert_close(out[2], 1.0, "m2");
    assert_close(out[3], 0.0, "m3");
}

#[test]
fn collective_boosts_to_preserve_roll_at_low_throttle() {
    // Plain clamping would floor m0/m3 at 0 and deliver a weaker
    // moment than commanded; boosting collective to 0.3 keeps the
    // commanded differential intact.
    let out = mix_desaturated(0.1, 0.3, 0.0, 0.0, &X_SIGNS);
    assert_close(out[0], 0.0, "m0");
    assert_close(out[1], 0.6, "m1");
    assert_close(out[2], 0.6, "m2");
    assert_close(out[3], 0.0, "m3");
}

#[test]
fn yaw_is_clipped_to_the_headroom_left_by_roll_pitch_and_collective() {
    // rp span 1.2 scales to 1.0, collective centers at 0.5, and the
    // pre-yaw outputs [0.5, 1.0, 0.5, 0.0] leave zero yaw headroom:
    // the +0.2 yaw command is dropped entirely rather than being
    // allowed to eat collective off motors 1 and 3.
    let out = mix_desaturated(0.8, 0.3, 0.3, 0.2, &X_SIGNS);
    assert_close(out[0], 0.5, "m0");
    assert_close(out[1], 1.0, "m1");
    assert_close(out[2], 0.5, "m2");
    assert_close(out[3], 0.0, "m3");
}

#[test]
fn negative_yaw_flows_when_headroom_allows() {
    // Same rp/collective as above but yaw in the direction that has
    // headroom: it passes through unclipped.
    let out = mix_desaturated(0.8, 0.3, 0.3, -0.2, &X_SIGNS);
    assert_close(out[0], 0.7, "m0");
    assert_close(out[1], 0.8, "m1");
    assert_close(out[2], 0.3, "m2");
    assert_close(out[3], 0.2, "m3");
}

#[test]
fn oversized_negative_yaw_borrows_margin_then_clips() {
    // Bases sit at 0.2, giving ±0.2 native yaw headroom. The −0.8
    // request pulls collective up by the full margin (0.2 → 0.35),
    // widening headroom to ±0.35, and the rest of the request is
    // clipped without pushing any motor below zero.
    let out = mix_desaturated(0.2, 0.0, 0.0, -0.8, &X_SIGNS);
    assert_close(out[0], 0.70, "m0");
    assert_close(out[1], 0.0, "m1");
    assert_close(out[2], 0.0, "m2");
    assert_close(out[3], 0.70, "m3");
}

#[test]
fn yaw_brake_borrows_bounded_collective_at_hover() {
    // Hover trim 0.77 leaves only 0.23 of up-headroom for the
    // motors a full −1.0 yaw brake must raise. Collective sags by
    // the margin (0.77 → 0.62), buying a ±0.38 yaw differential —
    // the brake authority that stops a commanded spin — while the
    // thrust dip stays bounded.
    let out = mix_desaturated(0.77, 0.0, 0.0, -1.0, &X_SIGNS);
    assert_close(out[0], 1.0, "m0");
    assert_close(out[1], 0.24, "m1");
    assert_close(out[2], 0.24, "m2");
    assert_close(out[3], 1.0, "m3");
}

#[test]
fn yaw_borrow_stops_at_the_concave_peak_within_margin() {
    // From t = 0.55 the headroom-maximizing collective (0.5) lies
    // inside the margin, so the shift stops there instead of using
    // the full ±0.15: shifting past the peak would shrink headroom
    // again.
    let out = mix_desaturated(0.55, 0.0, 0.0, -0.9, &X_SIGNS);
    assert_close(out[0], 1.0, "m0");
    assert_close(out[1], 0.0, "m1");
    assert_close(out[2], 0.0, "m2");
    assert_close(out[3], 1.0, "m3");
}

#[test]
fn roll_keeps_priority_over_the_yaw_borrow() {
    // A full-span roll differential pins collective at 0.5; the yaw
    // request finds zero headroom and the borrow cannot move
    // collective anywhere that helps, so yaw stays fully clipped
    // and the roll moment is delivered intact.
    let out = mix_desaturated(0.5, 0.5, 0.0, 0.3, &X_SIGNS);
    assert_close(out[0], 0.0, "m0");
    assert_close(out[1], 1.0, "m1");
    assert_close(out[2], 1.0, "m2");
    assert_close(out[3], 0.0, "m3");
}

#[test]
fn every_output_stays_in_bounds_across_a_command_grid() {
    let steps = |lo: Scalar, hi: Scalar| (0..=8).map(move |i| lo + (hi - lo) * (i as Scalar) / 8.0);
    for t in steps(0.0, 1.0) {
        for r in steps(-1.0, 1.0) {
            for p in steps(-1.0, 1.0) {
                for y in steps(-1.0, 1.0) {
                    let out = mix_desaturated(t, r, p, y, &X_SIGNS);
                    for (i, m) in out.iter().enumerate() {
                        assert!(
                            (0.0..=1.0).contains(m),
                            "motor {i} out of bounds: {m} for t={t} r={r} p={p} y={y}"
                        );
                    }
                }
            }
        }
    }
}
