//! Behaviour tests for the airborne latch.
//!
//! Every case drives `update()` with estimates, never by writing the
//! private phase field: a test that sets the latch directly would keep
//! passing if the determination were deleted.

#![allow(clippy::expect_used, clippy::panic)]

use super::*;
use crate::state::EstimateQuality;

/// Estimate at `altitude` metres up with `vz` metres per second up.
fn est(altitude: f32, vz: f32, valid: StateValidFlags, quality: EstimateQuality) -> StateEstimate {
    StateEstimate {
        position_ned: [Meters(0.0), Meters(0.0), Meters(-altitude)],
        velocity_ned: [
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(-vz),
        ],
        quality,
        valid_flags: valid,
        ..StateEstimate::default()
    }
}

/// The shape a GNSS-denied multirotor actually produces: no POSITION
/// fix, a usable vertical solution, and a velocity estimate.
fn usable(altitude: f32, vz: f32) -> StateEstimate {
    est(
        altitude,
        vz,
        StateValidFlags::VELOCITY,
        EstimateQuality::Degraded,
    )
}

/// An estimate the filter itself declares unusable.
fn unusable(altitude: f32, vz: f32) -> StateEstimate {
    est(
        altitude,
        vz,
        StateValidFlags::VELOCITY,
        EstimateQuality::Unusable,
    )
}

/// Armed at ground level with a usable estimate.
fn armed_on_ground() -> (FlightPhaseState, FlightPhaseLimits) {
    let mut s = FlightPhaseState::default();
    s.begin_flight_period(&usable(0.0, 0.0));
    (s, FlightPhaseLimits::default())
}

/// Hold the landed condition for `n` cycles.
fn settle(s: &mut FlightPhaseState, limits: &FlightPhaseLimits, n: u16) {
    for _ in 0..n {
        s.update(&usable(0.0, 0.0), true, limits);
    }
}

#[test]
fn default_limits_are_hysteretic() {
    assert!(FlightPhaseLimits::default().is_hysteretic());
}

#[test]
fn starts_on_ground() {
    let (s, _) = armed_on_ground();
    assert_eq!(s.phase(), FlightPhase::OnGround);
    assert!(!s.is_airborne());
}

#[test]
fn climbing_past_the_takeoff_height_latches_airborne() {
    let (mut s, limits) = armed_on_ground();
    s.update(&usable(0.4, 1.0), true, &limits);
    assert_eq!(s.phase(), FlightPhase::OnGround, "below threshold");
    s.update(&usable(0.6, 1.0), true, &limits);
    assert!(s.is_airborne(), "above threshold latches");
}

#[test]
fn descending_below_landed_height_does_not_immediately_clear() {
    let (mut s, limits) = armed_on_ground();
    s.update(&usable(5.0, 0.0), true, &limits);
    assert!(s.is_airborne());

    // One cycle of the landed condition is not a landing.
    s.update(&usable(0.0, 0.0), true, &limits);
    assert!(s.is_airborne(), "debounce not yet satisfied");

    settle(&mut s, &limits, limits.landed_debounce_cycles);
    assert_eq!(s.phase(), FlightPhase::OnGround, "debounce satisfied");
}

#[test]
fn fast_descent_through_the_landed_height_stays_airborne() {
    let (mut s, limits) = armed_on_ground();
    s.update(&usable(5.0, 0.0), true, &limits);

    // Plenty of cycles at low height, but descending fast: mid-approach,
    // not landed.
    for _ in 0..(limits.landed_debounce_cycles * 3) {
        s.update(&usable(0.1, -4.0), true, &limits);
    }
    assert!(s.is_airborne(), "vertical speed gate holds the latch");
}

#[test]
fn interrupted_landed_condition_restarts_the_debounce() {
    let (mut s, limits) = armed_on_ground();
    s.update(&usable(5.0, 0.0), true, &limits);

    settle(&mut s, &limits, limits.landed_debounce_cycles - 1);
    assert!(s.is_airborne());
    // A single cycle that breaks the condition must cost the whole run.
    s.update(&usable(2.0, 1.0), true, &limits);
    settle(&mut s, &limits, limits.landed_debounce_cycles - 1);
    assert!(s.is_airborne(), "debounce restarted, not resumed");
    s.update(&usable(0.0, 0.0), true, &limits);
    assert_eq!(s.phase(), FlightPhase::OnGround);
}

#[test]
fn hovering_at_the_threshold_does_not_chatter() {
    let (mut s, limits) = armed_on_ground();
    // Between landed_height and takeoff_height: neither latch fires.
    for _ in 0..(limits.landed_debounce_cycles * 2) {
        s.update(&usable(0.4, 0.0), true, &limits);
    }
    assert_eq!(s.phase(), FlightPhase::OnGround);

    s.update(&usable(5.0, 0.0), true, &limits);
    assert!(s.is_airborne());
    // Back into the hysteresis band: still airborne, no clear.
    for _ in 0..(limits.landed_debounce_cycles * 2) {
        s.update(&usable(0.4, 0.0), true, &limits);
    }
    assert!(s.is_airborne(), "hysteresis band does not clear the latch");
}

#[test]
fn a_position_fix_is_not_required_to_latch() {
    // The decisive case: a GNSS-denied multirotor raises no POSITION
    // flag for an entire flight. Requiring one would leave the latch on
    // the ground the whole time and let an in-air disarm through.
    let (mut s, limits) = armed_on_ground();
    assert!(!usable(0.0, 0.0)
        .valid_flags
        .contains(StateValidFlags::POSITION));
    s.update(&usable(5.0, 0.0), true, &limits);
    assert!(s.is_airborne());
}

#[test]
fn unusable_quality_freezes_an_airborne_latch() {
    let (mut s, limits) = armed_on_ground();
    s.update(&usable(5.0, 0.0), true, &limits);
    assert!(s.is_airborne());

    // An unusable estimate at ground level must not be read as a landing.
    for _ in 0..(limits.landed_debounce_cycles * 3) {
        s.update(&unusable(0.0, 0.0), true, &limits);
    }
    assert!(
        s.is_airborne(),
        "unusable estimate is not evidence of landing"
    );
}

#[test]
fn unusable_quality_freezes_a_ground_latch() {
    let (mut s, limits) = armed_on_ground();
    // A wild altitude the filter itself disowns must not latch airborne.
    for _ in 0..10 {
        s.update(&unusable(500.0, 0.0), true, &limits);
    }
    assert_eq!(s.phase(), FlightPhase::OnGround);
}

#[test]
fn invalid_velocity_blocks_the_landed_debounce() {
    let (mut s, limits) = armed_on_ground();
    s.update(&usable(5.0, 0.0), true, &limits);

    // Height usable and at ground level, but no usable vertical speed:
    // not enough to call it a landing.
    for _ in 0..(limits.landed_debounce_cycles * 3) {
        s.update(
            &est(
                0.0,
                0.0,
                StateValidFlags::empty(),
                EstimateQuality::Degraded,
            ),
            true,
            &limits,
        );
    }
    assert!(s.is_airborne());
}

#[test]
fn a_datum_captured_from_an_unusable_estimate_never_latches() {
    let mut s = FlightPhaseState::default();
    let limits = FlightPhaseLimits::default();
    s.begin_flight_period(&unusable(0.0, 0.0));
    for _ in 0..10 {
        s.update(&usable(50.0, 0.0), true, &limits);
    }
    assert_eq!(
        s.phase(),
        FlightPhase::OnGround,
        "no trustworthy datum means no trustworthy height"
    );
}

#[test]
fn disarmed_cycles_do_not_advance_the_latch() {
    let (mut s, limits) = armed_on_ground();
    for _ in 0..10 {
        s.update(&usable(50.0, 0.0), false, &limits);
    }
    assert_eq!(s.phase(), FlightPhase::OnGround);
}

#[test]
fn datum_is_relative_not_absolute() {
    // Armed on a 1000 m plateau: height is measured from where it armed,
    // so an absolute-altitude test would latch airborne on the pad.
    let mut s = FlightPhaseState::default();
    let limits = FlightPhaseLimits::default();
    s.begin_flight_period(&usable(1000.0, 0.0));
    s.update(&usable(1000.2, 0.0), true, &limits);
    assert_eq!(s.phase(), FlightPhase::OnGround);
    s.update(&usable(1000.8, 0.0), true, &limits);
    assert!(s.is_airborne());
}

#[test]
fn reset_clears_the_latch_and_the_datum() {
    let (mut s, limits) = armed_on_ground();
    s.update(&usable(5.0, 0.0), true, &limits);
    assert!(s.is_airborne());
    s.reset();
    assert_eq!(s, FlightPhaseState::default());
    // With no datum captured, nothing latches until the next arm.
    s.update(&usable(50.0, 0.0), true, &limits);
    assert_eq!(s.phase(), FlightPhase::OnGround);
}

#[test]
fn re_arming_recaptures_the_datum() {
    let (mut s, limits) = armed_on_ground();
    s.update(&usable(5.0, 0.0), true, &limits);
    s.reset();
    // Landed somewhere 20 m lower than the first arm point.
    s.begin_flight_period(&usable(-20.0, 0.0));
    s.update(&usable(-19.4, 0.0), true, &limits);
    assert!(s.is_airborne(), "datum tracks the new arm point");
}

#[test]
fn encoding_is_fixed_width_and_covers_every_field() {
    let (mut a, limits) = armed_on_ground();
    let mut buf = [0u8; FlightPhaseState::ENCODED_LEN];
    assert_eq!(a.encode_canonical(&mut buf), FlightPhaseState::ENCODED_LEN);

    let before = buf;
    a.update(&usable(5.0, 0.0), true, &limits);
    let mut after = [0u8; FlightPhaseState::ENCODED_LEN];
    a.encode_canonical(&mut after);
    assert_ne!(before, after, "a phase change must change the encoding");
}

#[test]
fn encoding_clamps_to_a_short_buffer() {
    let (s, _) = armed_on_ground();
    let mut buf = [0u8; 3];
    assert_eq!(s.encode_canonical(&mut buf), 3);
}
