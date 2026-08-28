#![allow(clippy::expect_used, clippy::panic)]

use crate::replicable::Replicable;

use super::MultirotorRuntimeState;

fn encoded(state: &MultirotorRuntimeState) -> [u8; 95] {
    let mut buf = [0u8; 95];
    let written = state.encode_canonical(&mut buf);
    assert_eq!(written, MultirotorRuntimeState::ENCODED_LEN);
    buf
}

/// The guardrail the encoding comment promises: every persistent
/// field, mutated alone, must change the canonical bytes. A field
/// added without an encoding lane fails here instead of surfacing
/// as a silent lockstep divergence one cycle after it matters.
#[test]
fn every_persistent_field_reaches_the_canonical_encoding() {
    type Mutator = fn(&mut MultirotorRuntimeState);
    let mutators: [(&str, Mutator); 26] = [
        ("vel integrator x", |s| {
            s.velocity_loop.integrator_ned.x.0 = 9.0
        }),
        ("vel integrator y", |s| {
            s.velocity_loop.integrator_ned.y.0 = 9.0
        }),
        ("vel integrator z", |s| {
            s.velocity_loop.integrator_ned.z.0 = 9.0
        }),
        ("vel filt x", |s| {
            s.velocity_loop.last_vel_filt_ned.x.0 = 9.0
        }),
        ("vel filt y", |s| {
            s.velocity_loop.last_vel_filt_ned.y.0 = 9.0
        }),
        ("vel filt z", |s| {
            s.velocity_loop.last_vel_filt_ned.z.0 = 9.0
        }),
        ("rate filt x", |s| s.rate_loop.meas_filtered_prev.x.0 = 9.0),
        ("rate filt y", |s| s.rate_loop.meas_filtered_prev.y.0 = 9.0),
        ("rate filt z", |s| s.rate_loop.meas_filtered_prev.z.0 = 9.0),
        ("rate integral 0", |s| s.rate_loop.integral[0] = 9.0),
        ("rate integral 1", |s| s.rate_loop.integral[1] = 9.0),
        ("rate integral 2", |s| s.rate_loop.integral[2] = 9.0),
        ("vel sp x", |s| s.last_vel_sp_ned.x.0 = 9.0),
        ("vel sp y", |s| s.last_vel_sp_ned.y.0 = 9.0),
        ("vel sp z", |s| s.last_vel_sp_ned.z.0 = 9.0),
        ("vel sp primed", |s| s.vel_sp_primed = true),
        ("d primed", |s| s.velocity_loop.d_primed = true),
        ("rate primed", |s| s.rate_loop.primed = true),
        ("dt", |s| s.dt_sec = 9.0),
        ("previous mode", |s| {
            s.previous_effective_mode = Some(crate::control::ControlMode::Attitude)
        }),
        ("previous topology", |s| {
            s.previous_topology = Some(crate::control::EffectiveControlTopology::Attitude)
        }),
        ("last axis roll", |s| s.last_axis_command.roll.0 = 9.0),
        ("last axis pitch", |s| s.last_axis_command.pitch.0 = 9.0),
        ("last axis yaw", |s| s.last_axis_command.yaw.0 = 9.0),
        ("last axis collective", |s| {
            s.last_axis_command.collective.0 = 9.0
        }),
        ("axis command primed", |s| s.axis_command_primed = true),
    ];
    let baseline = encoded(&MultirotorRuntimeState::default());
    for (name, mutate) in mutators {
        let mut state = MultirotorRuntimeState::default();
        mutate(&mut state);
        assert_ne!(
            encoded(&state),
            baseline,
            "field {name} does not reach the canonical encoding"
        );
    }
}

#[test]
fn every_mode_and_topology_has_a_distinct_stable_tag() {
    use crate::control::{ControlMode, EffectiveControlTopology};

    let modes = [
        None,
        Some(ControlMode::Attitude),
        Some(ControlMode::AltitudeHold),
        Some(ControlMode::PositionHold),
        Some(ControlMode::VelocityControl),
        Some(ControlMode::Rate),
        Some(ControlMode::DeviationTracking),
    ];
    let topologies = [
        None,
        Some(EffectiveControlTopology::Attitude),
        Some(EffectiveControlTopology::Vertical),
        Some(EffectiveControlTopology::Velocity),
        Some(EffectiveControlTopology::Position),
        Some(EffectiveControlTopology::ZeroThrust),
        Some(EffectiveControlTopology::Unsupported),
    ];
    for (index, mode) in modes.iter().enumerate() {
        for prior in &modes[..index] {
            assert_ne!(
                crate::control::transfer::mode_option_tag(*mode),
                crate::control::transfer::mode_option_tag(*prior)
            );
        }
    }
    for (index, topology) in topologies.iter().enumerate() {
        for prior in &topologies[..index] {
            assert_ne!(
                crate::control::transfer::topology_option_tag(*topology),
                crate::control::transfer::topology_option_tag(*prior)
            );
        }
    }
}
