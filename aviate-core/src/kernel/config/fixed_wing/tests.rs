//! Validation and canonical identity of `FixedWingParams`.

use super::{FixedWingParams, FixedWingParamsError};
use crate::kernel::config::ResolvedKernelConfig;
use crate::types::{MetersPerSecond, NormalizedThrust, Radians, Seconds};

/// A light single-engine aeroplane: 25 m/s stall, 55 m/s cruise.
pub(crate) fn reference_params() -> FixedWingParams {
    FixedWingParams {
        stall_airspeed: MetersPerSecond(25.0),
        min_airspeed: MetersPerSecond(33.0),
        cruise_airspeed: MetersPerSecond(55.0),
        max_airspeed: MetersPerSecond(65.0),
        min_throttle: NormalizedThrust(0.0),
        cruise_throttle: NormalizedThrust(0.65),
        max_throttle: NormalizedThrust(1.0),
        max_climb_rate: MetersPerSecond(2.5),
        max_sink_rate: MetersPerSecond(5.0),
        trim_pitch: Radians(0.02),
        height_time_constant: Seconds(5.0),
        speed_time_constant: Seconds(5.0),
        energy_rate_p: 0.05,
        energy_rate_i: 0.02,
        balance_rate_p: 0.5,
        balance_rate_i: 0.2,
        course_time_constant: Seconds(5.0),
        track_lookahead_time: Seconds(8.0),
        max_intercept_angle: Radians(0.785),
        roll_time_constant: Seconds(0.5),
        pitch_time_constant: Seconds(0.5),
        rate_p: [0.4, 0.6, 0.5],
        rate_i: [0.2, 0.4, 0.1],
        rate_ff: [0.8, 1.0, 0.5],
        rate_integrator_limit: 0.3,
        airspeed_scale_min: 0.5,
        airspeed_scale_max: 2.0,
        airspeed_filter_time_constant: Seconds(0.5),
    }
}

/// One mutation of each stored value, in declaration order.
fn mutations() -> [fn(&mut FixedWingParams); 34] {
    [
        |p| p.stall_airspeed.0 += 0.5,
        |p| p.min_airspeed.0 += 0.5,
        |p| p.cruise_airspeed.0 += 0.5,
        |p| p.max_airspeed.0 += 0.5,
        |p| p.min_throttle.0 += 0.05,
        |p| p.cruise_throttle.0 += 0.05,
        |p| p.max_throttle.0 -= 0.05,
        |p| p.max_climb_rate.0 += 0.5,
        |p| p.max_sink_rate.0 += 0.5,
        |p| p.trim_pitch.0 += 0.01,
        |p| p.height_time_constant.0 += 0.5,
        |p| p.speed_time_constant.0 += 0.5,
        |p| p.energy_rate_p += 0.01,
        |p| p.energy_rate_i += 0.01,
        |p| p.balance_rate_p += 0.1,
        |p| p.balance_rate_i += 0.1,
        |p| p.course_time_constant.0 += 0.5,
        |p| p.track_lookahead_time.0 += 0.5,
        |p| p.max_intercept_angle.0 += 0.1,
        |p| p.roll_time_constant.0 += 0.1,
        |p| p.pitch_time_constant.0 += 0.1,
        |p| p.rate_p[0] += 0.1,
        |p| p.rate_p[1] += 0.1,
        |p| p.rate_p[2] += 0.1,
        |p| p.rate_i[0] += 0.1,
        |p| p.rate_i[1] += 0.1,
        |p| p.rate_i[2] += 0.1,
        |p| p.rate_ff[0] += 0.1,
        |p| p.rate_ff[1] += 0.1,
        |p| p.rate_ff[2] += 0.1,
        |p| p.rate_integrator_limit += 0.1,
        |p| p.airspeed_scale_min -= 0.1,
        |p| p.airspeed_scale_max += 0.1,
        |p| p.airspeed_filter_time_constant.0 += 0.1,
    ]
}

fn config_with(params: FixedWingParams) -> ResolvedKernelConfig {
    ResolvedKernelConfig {
        fixed_wing: Some(params),
        ..ResolvedKernelConfig::default()
    }
}

#[test]
fn reference_block_is_valid() {
    assert_eq!(reference_params().validate(), Ok(()));
}

#[test]
fn every_value_moves_the_tuning_identity_and_the_canonical_hash() {
    let reference = config_with(reference_params());
    let mut identities = [reference.fixed_wing_tuning_identity(); 35];
    for (index, mutate) in mutations().into_iter().enumerate() {
        let mut params = reference_params();
        mutate(&mut params);
        assert_eq!(
            params.validate(),
            Ok(()),
            "mutation {index} must stay valid"
        );
        let mutated = config_with(params);
        assert_ne!(
            mutated.canonical_hash(),
            reference.canonical_hash(),
            "value {index} is not in the canonical hash"
        );
        // Slot 0 holds the reference; slots 1..=index hold earlier mutations.
        let identity = mutated.fixed_wing_tuning_identity();
        assert!(
            !identities[..=index].contains(&identity),
            "value {index} does not move the tuning identity"
        );
        identities[index + 1] = identity;
    }
}

#[test]
fn a_configuration_without_the_block_has_no_identity_and_its_own_hash() {
    let without = ResolvedKernelConfig::default();
    assert_eq!(without.fixed_wing_tuning_identity(), None);
    assert_ne!(
        without.canonical_hash(),
        config_with(reference_params()).canonical_hash()
    );
}

fn rejected(mutate: fn(&mut FixedWingParams)) -> FixedWingParamsError {
    let mut params = reference_params();
    mutate(&mut params);
    params.validate().expect_err("the block must be refused")
}

#[test]
fn a_block_that_cannot_fly_is_refused_with_its_reason() {
    use FixedWingParamsError as E;
    assert_eq!(rejected(|p| p.trim_pitch.0 = f32::NAN), E::NotFinite);
    assert_eq!(rejected(|p| p.rate_ff[1] = f32::INFINITY), E::NotFinite);
    assert_eq!(rejected(|p| p.roll_time_constant.0 = 0.0), E::NotPositive);
    assert_eq!(rejected(|p| p.max_sink_rate.0 = -1.0), E::NotPositive);
    assert_eq!(rejected(|p| p.rate_p[2] = -0.1), E::NegativeGain);
    assert_eq!(rejected(|p| p.min_airspeed.0 = 20.0), E::AirspeedOrder);
    assert_eq!(rejected(|p| p.max_airspeed.0 = 55.0), E::AirspeedOrder);
    assert_eq!(rejected(|p| p.min_throttle.0 = -0.1), E::ThrottleOrder);
    assert_eq!(rejected(|p| p.max_throttle.0 = 1.1), E::ThrottleOrder);
    assert_eq!(rejected(|p| p.cruise_throttle.0 = 1.0), E::ThrottleOrder);
    assert_eq!(rejected(|p| p.airspeed_scale_min = 1.2), E::ScaleOrder);
    assert_eq!(rejected(|p| p.airspeed_scale_max = 0.9), E::ScaleOrder);
}

/// Fixed-capacity text sink: the crate has no allocator in its tests.
struct Text {
    bytes: [u8; 96],
    len: usize,
}

impl core::fmt::Write for Text {
    fn write_str(&mut self, text: &str) -> core::fmt::Result {
        let end = self.len + text.len();
        self.bytes
            .get_mut(self.len..end)
            .ok_or(core::fmt::Error)?
            .copy_from_slice(text.as_bytes());
        self.len = end;
        Ok(())
    }
}

fn text_of(reason: FixedWingParamsError) -> Text {
    use core::fmt::Write;
    let mut text = Text {
        bytes: [0; 96],
        len: 0,
    };
    write!(text, "{reason}").expect("the reason fits the buffer");
    text
}

#[test]
fn each_refusal_reason_has_its_own_text() {
    use FixedWingParamsError as E;
    let reasons = [
        E::NotFinite,
        E::NotPositive,
        E::NegativeGain,
        E::AirspeedOrder,
        E::ThrottleOrder,
        E::ScaleOrder,
    ];
    for (index, reason) in reasons.iter().enumerate() {
        let text = text_of(*reason);
        let rendered = &text.bytes[..text.len];
        assert!(rendered.starts_with(b"fixed-wing") || rendered.starts_with(b"a fixed-wing"));
        for earlier in &reasons[..index] {
            let other = text_of(*earlier);
            assert_ne!(rendered, &other.bytes[..other.len]);
        }
    }
}
