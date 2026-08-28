//! Exact source identity for the Alia X-Plane executable.

use aviate_config::airframe_preset::ContentDigest;

pub(super) fn application_source_identity() -> ContentDigest {
    application_source_identity_from(application_source_inputs())
}

fn application_source_inputs() -> &'static [(&'static str, &'static [u8])] {
    &[
        ("kernel/construct.rs", include_bytes!("../construct.rs")),
        ("kernel/tuning.rs", include_bytes!("../tuning.rs")),
        ("kernel/manifest.rs", include_bytes!("../manifest.rs")),
        (
            "kernel/manifest/source_identity.rs",
            include_bytes!("source_identity.rs"),
        ),
        ("kernel/build.rs", include_bytes!("../../build.rs")),
        (
            "workspace/Cargo.toml",
            include_bytes!("../../../../../Cargo.toml"),
        ),
        ("kernel/Cargo.toml", include_bytes!("../../Cargo.toml")),
        ("app/Cargo.toml", include_bytes!("../../../Cargo.toml")),
        (
            "app/AviateApp.toml",
            include_bytes!("../../../AviateApp.toml"),
        ),
        (
            "board/Cargo.toml",
            include_bytes!("../../../../../aviate-boards/sitl-xplane/Cargo.toml"),
        ),
        (
            "board/lib.rs",
            include_bytes!("../../../../../aviate-boards/sitl-xplane/src/lib.rs"),
        ),
        ("app/main.rs", include_bytes!("../../../src/main.rs")),
        ("app/cli.rs", include_bytes!("../../../src/cli.rs")),
        (
            "app/cli/error.rs",
            include_bytes!("../../../src/cli/error.rs"),
        ),
        (
            "app/cli/runtime_binding.rs",
            include_bytes!("../../../src/cli/runtime_binding.rs"),
        ),
        (
            "app/artifact.rs",
            include_bytes!("../../../src/artifact.rs"),
        ),
        (
            "app/tuning_trace.rs",
            include_bytes!("../../../src/tuning_trace.rs"),
        ),
        (
            "app/identify.rs",
            include_bytes!("../../../src/identify.rs"),
        ),
        (
            "app/report.rs",
            include_bytes!("../../../src/identify/report.rs"),
        ),
        (
            "app/stand.rs",
            include_bytes!("../../../src/identify/stand.rs"),
        ),
        (
            "app/identify/trace.rs",
            include_bytes!("../../../src/identify/trace.rs"),
        ),
        (
            "app/identify/sweep.rs",
            include_bytes!("../../../src/identify/sweep.rs"),
        ),
        (
            "app/identify/yaw_sign.rs",
            include_bytes!("../../../src/identify/yaw_sign.rs"),
        ),
        (
            "board/board.rs",
            include_bytes!("../../../../../aviate-boards/sitl-xplane/src/board.rs"),
        ),
        (
            "board/handshake.rs",
            include_bytes!("../../../../../aviate-boards/sitl-xplane/src/board/handshake.rs"),
        ),
        (
            "board/observation.rs",
            include_bytes!("../../../../../aviate-boards/sitl-xplane/src/board/observation.rs"),
        ),
        (
            "board/packet.rs",
            include_bytes!("../../../../../aviate-boards/sitl-xplane/src/board/packet.rs"),
        ),
        (
            "board/tuning_trace.rs",
            include_bytes!("../../../../../aviate-boards/sitl-xplane/src/board/tuning_trace.rs"),
        ),
        (
            "board/tuning_trace/protocol.rs",
            include_bytes!(
                "../../../../../aviate-boards/sitl-xplane/src/board/tuning_trace/protocol.rs"
            ),
        ),
        (
            "board/wire.rs",
            include_bytes!("../../../../../aviate-boards/sitl-xplane/src/board/wire.rs"),
        ),
        (
            "config/candidate.rs",
            include_bytes!("../../../../../aviate-config/src/airframe_preset/candidate.rs"),
        ),
        (
            "config/candidate/design.rs",
            include_bytes!("../../../../../aviate-config/src/airframe_preset/candidate/design.rs"),
        ),
        (
            "config/candidate/lineage.rs",
            include_bytes!("../../../../../aviate-config/src/airframe_preset/candidate/lineage.rs"),
        ),
        (
            "config/candidate/plant.rs",
            include_bytes!("../../../../../aviate-config/src/airframe_preset/candidate/plant.rs"),
        ),
        (
            "config/model.rs",
            include_bytes!("../../../../../aviate-config/src/xplane_model.rs"),
        ),
        (
            "config/runtime.rs",
            include_bytes!("../../../../../aviate-config/src/xplane_runtime.rs"),
        ),
        (
            "hal-xil/Cargo.toml",
            include_bytes!("../../../../../aviate-hal/xil/Cargo.toml"),
        ),
        (
            "hal-xil/lib.rs",
            include_bytes!("../../../../../aviate-hal/xil/src/lib.rs"),
        ),
        (
            "hal-xil/bridge.rs",
            include_bytes!("../../../../../aviate-hal/xil/src/bridge.rs"),
        ),
        (
            "hal-xil/command_provenance.rs",
            include_bytes!("../../../../../aviate-hal/xil/src/command_provenance.rs"),
        ),
        (
            "hal-xil/sitl_io.rs",
            include_bytes!("../../../../../aviate-hal/xil/src/sitl_io.rs"),
        ),
        (
            "hal-xil/sitl_io/command_link.rs",
            include_bytes!("../../../../../aviate-hal/xil/src/sitl_io/command_link.rs"),
        ),
        (
            "link/Cargo.toml",
            include_bytes!("../../../../../aviate-link/Cargo.toml"),
        ),
        (
            "link/lib.rs",
            include_bytes!("../../../../../aviate-link/src/lib.rs"),
        ),
        (
            "link/mavlink.rs",
            include_bytes!("../../../../../aviate-link/src/mavlink.rs"),
        ),
        (
            "link/mavlink/protocol.rs",
            include_bytes!("../../../../../aviate-link/src/mavlink/protocol.rs"),
        ),
        (
            "runtime/Cargo.toml",
            include_bytes!("../../../../../aviate-runtime/Cargo.toml"),
        ),
        (
            "runtime/lib.rs",
            include_bytes!("../../../../../aviate-runtime/src/lib.rs"),
        ),
        (
            "runtime/command_ingress.rs",
            include_bytes!("../../../../../aviate-runtime/src/command_ingress.rs"),
        ),
        (
            "runtime/sim.rs",
            include_bytes!("../../../../../aviate-runtime/src/sim.rs"),
        ),
        (
            "runtime/sim/step.rs",
            include_bytes!("../../../../../aviate-runtime/src/sim/step.rs"),
        ),
        (
            "core/kernel_update.rs",
            include_bytes!("../../../../../aviate-core/src/kernel_update.rs"),
        ),
        (
            "core/kernel_types.rs",
            include_bytes!("../../../../../aviate-core/src/kernel_types.rs"),
        ),
        (
            "core/control.rs",
            include_bytes!("../../../../../aviate-core/src/control.rs"),
        ),
        (
            "core/control/mode_gate.rs",
            include_bytes!("../../../../../aviate-core/src/control/mode_gate.rs"),
        ),
        (
            "core/control/runtime.rs",
            include_bytes!("../../../../../aviate-core/src/control/runtime.rs"),
        ),
        (
            "core/control/observation.rs",
            include_bytes!("../../../../../aviate-core/src/control/observation.rs"),
        ),
        (
            "core/control/transfer.rs",
            include_bytes!("../../../../../aviate-core/src/control/transfer.rs"),
        ),
        (
            "core/control/vehicle_control_mode.rs",
            include_bytes!("../../../../../aviate-core/src/control/vehicle_control_mode.rs"),
        ),
        (
            "core/control/multirotor.rs",
            include_bytes!("../../../../../aviate-core/src/control/multirotor.rs"),
        ),
        (
            "core/control/multirotor/step.rs",
            include_bytes!("../../../../../aviate-core/src/control/multirotor/step.rs"),
        ),
        (
            "core/control/velocity.rs",
            include_bytes!("../../../../../aviate-core/src/control/velocity.rs"),
        ),
        (
            "core/control/rate.rs",
            include_bytes!("../../../../../aviate-core/src/control/rate.rs"),
        ),
    ]
}

fn application_source_identity_from(sources: &[(&str, &[u8])]) -> ContentDigest {
    let mut bytes = Vec::new();
    for (name, source) in sources {
        bytes.extend_from_slice(&(name.len() as u64).to_le_bytes());
        bytes.extend_from_slice(name.as_bytes());
        bytes.extend_from_slice(&(source.len() as u64).to_le_bytes());
        bytes.extend_from_slice(source);
    }
    ContentDigest::calculate(&bytes)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn application_identity_commits_the_production_control_boundary() {
        let required = [
            "hal-xil/Cargo.toml",
            "hal-xil/lib.rs",
            "hal-xil/bridge.rs",
            "hal-xil/command_provenance.rs",
            "hal-xil/sitl_io.rs",
            "hal-xil/sitl_io/command_link.rs",
            "link/Cargo.toml",
            "link/mavlink.rs",
            "link/mavlink/protocol.rs",
            "runtime/Cargo.toml",
            "runtime/lib.rs",
            "runtime/command_ingress.rs",
            "runtime/sim.rs",
            "runtime/sim/step.rs",
            "board/lib.rs",
            "board/packet.rs",
            "board/tuning_trace.rs",
            "board/tuning_trace/protocol.rs",
            "link/lib.rs",
            "core/kernel_update.rs",
            "core/kernel_types.rs",
            "core/control.rs",
            "core/control/mode_gate.rs",
            "core/control/runtime.rs",
            "core/control/observation.rs",
            "core/control/transfer.rs",
            "core/control/vehicle_control_mode.rs",
            "core/control/multirotor.rs",
            "core/control/multirotor/step.rs",
            "core/control/velocity.rs",
            "core/control/rate.rs",
        ];
        let inputs = application_source_inputs();
        let base = application_source_identity_from(inputs);
        for target in required {
            assert!(inputs.iter().any(|(name, _)| *name == target));
            let changed = inputs
                .iter()
                .map(|(name, source)| {
                    if *name == target {
                        (*name, b"mutation".as_slice())
                    } else {
                        (*name, *source)
                    }
                })
                .collect::<Vec<_>>();
            assert_ne!(application_source_identity_from(&changed), base, "{target}");
        }
    }
}
