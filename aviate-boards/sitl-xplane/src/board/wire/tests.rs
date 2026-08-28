//! The wire constraints against the measured plant regime: the spool
//! ramp, the two ceilings, and the on-gear squeeze, each driven the
//! way a flight drives them.
#![allow(clippy::expect_used, clippy::panic)]

use aviate_config::xplane_model::XPlaneWireModel;

use super::WireConstraints;

fn constraints() -> WireConstraints {
    WireConstraints::new(XPlaneWireModel {
        rise_per_s: 0.035,
        band_boundary: 0.40,
        low_band_rise_per_s: 0.15,
        fall_per_s: 0.30,
        mean_ceiling: 0.55,
        lane_ceiling: 0.85,
        airborne_clearance_m: 0.5,
        ground_squeeze: 0.5,
        max_sample_dt_s: 0.05,
    })
}

fn mean(outputs: &[f32; 16]) -> f32 {
    outputs[..4].iter().sum::<f32>() / 4.0
}

fn airborne(wire: &mut WireConstraints) {
    wire.arm(Some(10.0));
    let mut outputs = [0.0f32; 16];
    // One sample well above the clearance flips the airborne latch.
    wire.constrain(&mut outputs, 4, true, Some(20.0), 0.01);
}

#[test]
fn the_collective_mean_rises_no_faster_than_the_band_allows() {
    let mut wire = constraints();
    airborne(&mut wire);
    let mut outputs = [0.5f32; 16];
    wire.constrain(&mut outputs, 4, true, Some(20.0), 0.01);
    // From zero, one 10 ms sample in the fast band credits 0.0015.
    assert!(mean(&outputs) <= 0.15 * 0.01 + 1e-6);
    // Sustained demand keeps ramping, never jumping.
    let mut last = mean(&outputs);
    for _ in 0..50 {
        let mut step = [0.5f32; 16];
        wire.constrain(&mut step, 4, true, Some(20.0), 0.01);
        let now = mean(&step);
        assert!(now >= last - 1e-6);
        assert!(now - last <= 0.15 * 0.01 + 1e-6, "ramp outran the band");
        last = now;
    }
}

#[test]
fn the_mean_ceiling_holds_and_the_bookkeeping_is_the_wire() {
    let mut wire = constraints();
    airborne(&mut wire);
    let mut outputs = [1.0f32; 16];
    for _ in 0..2000 {
        outputs = [1.0f32; 16];
        wire.constrain(&mut outputs, 4, true, Some(20.0), 0.05);
    }
    assert!(mean(&outputs) <= 0.55 + 1e-6, "mean ceiling breached");
}

#[test]
fn a_railed_lane_stays_under_the_stall_ceiling_with_its_moment_direction_kept() {
    let mut wire = constraints();
    airborne(&mut wire);
    // Spool up to hover first so the differential has a real mean.
    for _ in 0..2000 {
        let mut warm = [0.43f32; 16];
        wire.constrain(&mut warm, 4, true, Some(20.0), 0.05);
    }
    let mut outputs = [0.0f32; 16];
    outputs[..4].copy_from_slice(&[0.0, 1.0, 0.3, 0.43]);
    wire.constrain(&mut outputs, 4, true, Some(20.0), 0.01);
    for lane in &outputs[..4] {
        assert!(*lane <= 0.85 + 1e-6, "a lane crossed the latch ceiling");
        assert!(*lane >= -1e-6);
    }
    // The squeeze preserves the ORDER of the lanes: the moment keeps
    // its direction even when its magnitude yields.
    assert!(outputs[1] > outputs[3]);
    assert!(outputs[3] > outputs[2]);
    assert!(outputs[2] > outputs[0]);
}

#[test]
fn the_gear_squeezes_differentials_until_the_climb_clears_and_then_lets_go() {
    let mut wire = constraints();
    wire.arm(Some(10.0));
    let mut on_gear = [0.0f32; 16];
    on_gear[..4].copy_from_slice(&[0.2, 0.4, 0.2, 0.4]);
    wire.constrain(&mut on_gear, 4, true, Some(10.0), 0.05);
    let spread_on_gear = on_gear[1] - on_gear[0];
    let mut aloft = [0.0f32; 16];
    aloft[..4].copy_from_slice(&[0.2, 0.4, 0.2, 0.4]);
    wire.constrain(&mut aloft, 4, true, Some(10.6), 0.05);
    let mut aloft2 = [0.0f32; 16];
    aloft2[..4].copy_from_slice(&[0.2, 0.4, 0.2, 0.4]);
    wire.constrain(&mut aloft2, 4, true, Some(10.6), 0.05);
    let spread_aloft = aloft2[1] - aloft2[0];
    assert!(
        spread_on_gear < spread_aloft,
        "the gear squeeze must retain less differential than free air"
    );
}

#[test]
fn arming_before_the_first_fix_still_finds_its_ground() {
    let mut wire = constraints();
    wire.arm(None);
    // First fix after arm becomes the ground reference; spool the
    // collective to the demand so the ramp cannot eat the
    // differential this test measures.
    for _ in 0..80 {
        let mut warm = [0.3f32; 16];
        wire.constrain(&mut warm, 4, true, Some(7.0), 0.05);
    }
    // A climb past the clearance releases the squeeze: the
    // differential survives once airborne.
    let mut climb = [0.0f32; 16];
    climb[..4].copy_from_slice(&[0.2, 0.4, 0.2, 0.4]);
    wire.constrain(&mut climb, 4, true, Some(8.0), 0.05);
    let mut aloft = [0.0f32; 16];
    aloft[..4].copy_from_slice(&[0.2, 0.4, 0.2, 0.4]);
    wire.constrain(&mut aloft, 4, true, Some(8.0), 0.05);
    assert!((aloft[1] - aloft[0]) > 0.19, "full differential once clear");
}

#[test]
fn an_armed_fall_is_paced_and_a_disarm_cuts_instantly() {
    let mut wire = constraints();
    airborne(&mut wire);
    for _ in 0..2000 {
        let mut warm = [0.5f32; 16];
        wire.constrain(&mut warm, 4, true, Some(20.0), 0.05);
    }
    // An armed cut steps down at the fall limit, not to the floor: the
    // rise limit can catch a paced dip, and an uncaught one rings the
    // vertical loop into the ground.
    let mut cut = [0.1f32; 16];
    wire.constrain(&mut cut, 4, true, Some(20.0), 0.05);
    let paced = mean(&cut);
    assert!(paced > 0.4, "one sample must not collapse the collective");
    // A disarm is not a maneuver: it cuts without a ramp.
    let mut off = [0.0f32; 16];
    wire.constrain(&mut off, 4, false, Some(20.0), 0.05);
    assert!(mean(&off) < 1e-6);
}
