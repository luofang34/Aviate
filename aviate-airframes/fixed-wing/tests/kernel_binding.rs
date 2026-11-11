//! Each shipped airframe builds a kernel whose controller matches its configuration.

#![allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]

use aviate_core::ekf::Ekf;
use aviate_core::kernel::builder::AviateKernelBuilder;
use aviate_core::kernel::config::MixerGeometry;
use aviate_core::mixer::fixed_wing::neutral_outputs;
use aviate_core::mixer::{GroupKind, Sanitizer};
use aviate_core::time::{TimeSource, Timestamp};

use aviate_airframe_fixed_wing::{c172, rascal, FixedWingAirframe};

fn ts() -> Timestamp {
    Timestamp {
        ticks: 0,
        source: TimeSource::Internal,
    }
}

#[test]
fn shipped_airframes_build_a_bound_kernel() {
    for airframe in [c172(), rascal()] {
        assert_eq!(airframe.params.validate(), Ok(()), "{}", airframe.id);
        let config = airframe.kernel_config();
        assert_eq!(config.mixer_geometry, MixerGeometry::FixedWing);
        assert_eq!(config.safe_output, neutral_outputs());
        assert_eq!(config.fixed_wing, Some(airframe.params));
        let kernel = AviateKernelBuilder::new()
            .estimator(Ekf::default())
            .controller(airframe.controller().expect("valid description"))
            .mixer(FixedWingAirframe::mixer(ts))
            .sanitizer(Sanitizer)
            .config(config)
            .build();
        assert!(kernel.is_ok(), "{} kernel refused", airframe.id);
    }
}

#[test]
fn a_controller_for_one_airframe_is_refused_with_the_configuration_of_another() {
    let kernel = AviateKernelBuilder::new()
        .estimator(Ekf::default())
        .controller(c172().controller().expect("valid description"))
        .mixer(FixedWingAirframe::mixer(ts))
        .sanitizer(Sanitizer)
        .config(rascal().kernel_config())
        .build();
    assert!(kernel.is_err());
}

#[test]
fn airframes_differ_and_keep_their_identifiers() {
    assert_eq!(c172().id, "c172");
    assert_eq!(rascal().id, "rascal");
    assert_ne!(
        c172().kernel_config().canonical_hash(),
        rascal().kernel_config().canonical_hash()
    );
}

#[test]
fn actuator_groups_fall_back_to_idle_throttle_and_neutral_surfaces() {
    let config = c172().kernel_config();
    let [throttle, surfaces] = config.mode_config.groups else {
        panic!("two actuator groups expected");
    };
    assert_eq!(throttle.kind, GroupKind::DistributedThrust);
    assert_eq!(throttle.members, &[0]);
    assert_eq!(throttle.safe_pattern.outputs[0].0, 0.0);
    assert_eq!(surfaces.kind, GroupKind::ControlSurfaces);
    assert_eq!(surfaces.members, &[1, 2, 3]);
    for lane in [1, 2, 3] {
        assert_eq!(surfaces.safe_pattern.outputs[lane].0, 0.5);
    }
    assert!(surfaces.safe_pattern.valid && throttle.safe_pattern.valid);
}
