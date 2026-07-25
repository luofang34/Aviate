//! Behaviour tests for the airborne latch.
//!
//! Every case drives `update()` with estimates, never by writing the
//! private phase field: a test that sets the latch directly would keep
//! passing if the determination were deleted.

#![allow(clippy::expect_used, clippy::panic)]

use super::*;
use crate::state::EstimateQuality;
use crate::types::Seconds;

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

/// One step at the 1 kHz runner rate.
const DT: Seconds = Seconds(0.001);

/// Steps that comfortably satisfy the debounce at `DT`.
///
/// The margin keeps these cases off the exact float boundary: summing
/// `DT` repeatedly lands a hair either side of the threshold depending
/// on rounding, and the properties under test are "an interruption
/// costs the whole run" and "one low cycle is not a landing" — not the
/// precise step at which the sum tips over.
fn enough_steps(limits: &FlightPhaseLimits) -> u32 {
    (limits.landed_debounce.0 / DT.0).ceil() as u32 + 2
}

/// Steps that definitely do not satisfy the debounce at `DT`.
fn not_enough_steps(limits: &FlightPhaseLimits) -> u32 {
    (limits.landed_debounce.0 / DT.0).ceil() as u32 - 2
}

/// Armed at ground level with a usable estimate.
fn armed_on_ground() -> (FlightPhaseState, FlightPhaseLimits) {
    let mut s = FlightPhaseState::default();
    s.begin_flight_period(&usable(0.0, 0.0));
    (s, FlightPhaseLimits::default())
}

/// Hold the landed condition for `n` steps.
fn settle(s: &mut FlightPhaseState, limits: &FlightPhaseLimits, n: u32) {
    for _ in 0..n {
        s.update(&usable(0.0, 0.0), true, DT, limits);
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
    s.update(&usable(0.4, 1.0), true, DT, &limits);
    assert_eq!(s.phase(), FlightPhase::OnGround, "below threshold");
    s.update(&usable(0.6, 1.0), true, DT, &limits);
    assert!(s.is_airborne(), "above threshold latches");
}

#[test]
fn descending_below_landed_height_does_not_immediately_clear() {
    let (mut s, limits) = armed_on_ground();
    s.update(&usable(5.0, 0.0), true, DT, &limits);
    assert!(s.is_airborne());

    // One cycle of the landed condition is not a landing.
    s.update(&usable(0.0, 0.0), true, DT, &limits);
    assert!(s.is_airborne(), "debounce not yet satisfied");

    settle(&mut s, &limits, enough_steps(&limits));
    assert_eq!(s.phase(), FlightPhase::OnGround, "debounce satisfied");
}

#[test]
fn fast_descent_through_the_landed_height_stays_airborne() {
    let (mut s, limits) = armed_on_ground();
    s.update(&usable(5.0, 0.0), true, DT, &limits);

    // Plenty of cycles at low height, but descending fast: mid-approach,
    // not landed.
    for _ in 0..(enough_steps(&limits) * 3) {
        s.update(&usable(0.1, -4.0), true, DT, &limits);
    }
    assert!(s.is_airborne(), "vertical speed gate holds the latch");
}

#[test]
fn interrupted_landed_condition_restarts_the_debounce() {
    let (mut s, limits) = armed_on_ground();
    s.update(&usable(5.0, 0.0), true, DT, &limits);

    settle(&mut s, &limits, not_enough_steps(&limits));
    assert!(s.is_airborne());
    // A single cycle that breaks the condition must cost the whole run,
    // so the same number of steps again must still not complete it.
    s.update(&usable(2.0, 1.0), true, DT, &limits);
    settle(&mut s, &limits, not_enough_steps(&limits));
    assert!(s.is_airborne(), "debounce restarted, not resumed");
    // Only a full uninterrupted run finishes it.
    settle(&mut s, &limits, enough_steps(&limits));
    assert_eq!(s.phase(), FlightPhase::OnGround);
}

#[test]
fn hovering_at_the_threshold_does_not_chatter() {
    let (mut s, limits) = armed_on_ground();
    // Between landed_height and takeoff_height: neither latch fires.
    for _ in 0..(enough_steps(&limits) * 2) {
        s.update(&usable(0.4, 0.0), true, DT, &limits);
    }
    assert_eq!(s.phase(), FlightPhase::OnGround);

    s.update(&usable(5.0, 0.0), true, DT, &limits);
    assert!(s.is_airborne());
    // Back into the hysteresis band: still airborne, no clear.
    for _ in 0..(enough_steps(&limits) * 2) {
        s.update(&usable(0.4, 0.0), true, DT, &limits);
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
    s.update(&usable(5.0, 0.0), true, DT, &limits);
    assert!(s.is_airborne());
}

#[test]
fn unusable_quality_freezes_an_airborne_latch() {
    let (mut s, limits) = armed_on_ground();
    s.update(&usable(5.0, 0.0), true, DT, &limits);
    assert!(s.is_airborne());

    // An unusable estimate at ground level must not be read as a landing.
    for _ in 0..(enough_steps(&limits) * 3) {
        s.update(&unusable(0.0, 0.0), true, DT, &limits);
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
        s.update(&unusable(500.0, 0.0), true, DT, &limits);
    }
    assert_eq!(s.phase(), FlightPhase::OnGround);
}

#[test]
fn invalid_velocity_blocks_the_landed_debounce() {
    let (mut s, limits) = armed_on_ground();
    s.update(&usable(5.0, 0.0), true, DT, &limits);

    // Height usable and at ground level, but no usable vertical speed:
    // not enough to call it a landing.
    for _ in 0..(enough_steps(&limits) * 3) {
        s.update(
            &est(
                0.0,
                0.0,
                StateValidFlags::empty(),
                EstimateQuality::Degraded,
            ),
            true,
            DT,
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
        s.update(&usable(50.0, 0.0), true, DT, &limits);
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
        s.update(&usable(50.0, 0.0), false, DT, &limits);
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
    s.update(&usable(1000.2, 0.0), true, DT, &limits);
    assert_eq!(s.phase(), FlightPhase::OnGround);
    s.update(&usable(1000.8, 0.0), true, DT, &limits);
    assert!(s.is_airborne());
}

#[test]
fn reset_clears_the_latch_and_the_datum() {
    let (mut s, limits) = armed_on_ground();
    s.update(&usable(5.0, 0.0), true, DT, &limits);
    assert!(s.is_airborne());
    s.reset();
    assert_eq!(s, FlightPhaseState::default());
    // With no datum captured, nothing latches until the next arm.
    s.update(&usable(50.0, 0.0), true, DT, &limits);
    assert_eq!(s.phase(), FlightPhase::OnGround);
}

#[test]
fn re_arming_recaptures_the_datum() {
    let (mut s, limits) = armed_on_ground();
    s.update(&usable(5.0, 0.0), true, DT, &limits);
    s.reset();
    // Landed somewhere 20 m lower than the first arm point.
    s.begin_flight_period(&usable(-20.0, 0.0));
    s.update(&usable(-19.4, 0.0), true, DT, &limits);
    assert!(s.is_airborne(), "datum tracks the new arm point");
}

#[test]
fn encoding_is_fixed_width_and_covers_every_field() {
    let (mut a, limits) = armed_on_ground();
    let mut buf = [0u8; FlightPhaseState::ENCODED_LEN];
    assert_eq!(a.encode_canonical(&mut buf), FlightPhaseState::ENCODED_LEN);

    let before = buf;
    a.update(&usable(5.0, 0.0), true, DT, &limits);
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

#[test]
fn the_debounce_is_the_same_duration_at_any_loop_rate() {
    // The runners do not share a rate: Gazebo and hardware step at
    // 1 kHz, jMAVSim at 400 Hz. A cycle-count debounce would mean
    // 0.5 s on one vehicle and 0.2 s on another; this pins that the
    // landing takes the same wall-clock time on both.
    let limits = FlightPhaseLimits::default();
    let mut elapsed = [0.0f32; 2];

    for (slot, dt) in [Seconds(0.001), Seconds(0.0025)].iter().enumerate() {
        let mut s = FlightPhaseState::default();
        s.begin_flight_period(&usable(0.0, 0.0));
        s.update(&usable(5.0, 0.0), true, *dt, &limits);
        assert!(s.is_airborne());

        let mut t = 0.0;
        while s.is_airborne() && t < limits.landed_debounce.0 * 4.0 {
            s.update(&usable(0.0, 0.0), true, *dt, &limits);
            t += dt.0;
        }
        assert_eq!(s.phase(), FlightPhase::OnGround, "landing must complete");
        elapsed[slot] = t;
    }

    let difference = (elapsed[0] - elapsed[1]).abs();
    assert!(
        difference < 0.01,
        "landing took {} s at 1 kHz and {} s at 400 Hz",
        elapsed[0],
        elapsed[1]
    );
}

#[test]
fn a_non_finite_timestep_cannot_complete_a_landing() {
    let (mut s, limits) = armed_on_ground();
    s.update(&usable(5.0, 0.0), true, DT, &limits);
    assert!(s.is_airborne());

    for dt in [Seconds(f32::NAN), Seconds(f32::INFINITY), Seconds(-1.0)] {
        for _ in 0..100 {
            s.update(&usable(0.0, 0.0), true, dt, &limits);
        }
    }
    assert!(s.is_airborne(), "a bad clock must not decide a landing");
}

#[test]
fn one_enormous_timestep_cannot_complete_a_landing() {
    let (mut s, limits) = armed_on_ground();
    s.update(&usable(5.0, 0.0), true, DT, &limits);
    assert!(s.is_airborne());

    // A cycle that took an hour means the loop stalled: there was no
    // continuous observation across it, so it must not by itself decide
    // that the vehicle is down.
    s.update(&usable(0.0, 0.0), true, Seconds(3600.0), &limits);
    assert!(
        s.is_airborne(),
        "one post-stall sample is not an observed landing"
    );

    // A second settled observation does complete it.
    s.update(&usable(0.0, 0.0), true, Seconds(3600.0), &limits);
    assert_eq!(s.phase(), FlightPhase::OnGround);
}
