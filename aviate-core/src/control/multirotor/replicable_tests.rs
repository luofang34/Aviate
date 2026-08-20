#![allow(clippy::expect_used, clippy::panic)]

use crate::replicable::Replicable;

use super::MultirotorRuntimeState;

fn encoded(state: &MultirotorRuntimeState) -> [u8; 76] {
    let mut buf = [0u8; 76];
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
    let mutators: [(&str, Mutator); 19] = [
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
