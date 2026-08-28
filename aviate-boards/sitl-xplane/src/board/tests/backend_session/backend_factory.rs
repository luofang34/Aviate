//! Build the Alia backend for TCP contract tests.

use std::net::SocketAddr;

use aviate_config::xplane_model::XPlaneSimulatorModel;
use aviate_hal_xil::VehicleState;

use super::AliaBackend;
use crate::{XPlaneBoard, XPlaneConfig, XPlaneRuntimeHandshake, XPlaneSimulatorBackend};

const MODEL: &str = include_str!("../../../../../../presets/alia250-xplane.toml");

pub(super) fn make_backend(address: SocketAddr) -> AliaBackend {
    let model = XPlaneSimulatorModel::from_toml_str(MODEL).expect("valid model");
    let kernel =
        aviate_app_sitl_xplane_alia250_kernel::build_alia250_kernel().expect("Alia kernel builds");
    let mut board = XPlaneBoard::with_config(kernel, XPlaneConfig::new(address, model.clone()))
        .expect("board builds");
    board
        .accept_runtime_handshake(runtime_handshake(&model, address))
        .expect("runtime handshake matches");
    XPlaneSimulatorBackend::new(board, 0, initial_state())
}

fn runtime_handshake(model: &XPlaneSimulatorModel, address: SocketAddr) -> XPlaneRuntimeHandshake {
    XPlaneRuntimeHandshake {
        schema_version: 1,
        verifier_id: "pilotage-xplane-trial-v1".to_owned(),
        session_binding_digest: "f".repeat(64),
        bridge_endpoint: address.to_string(),
        bridge_protocol: model.bridge_protocol(),
        bridge_build_digest: "a".repeat(64),
        bridge_config_digest: "c".repeat(64),
        simulator_id: model.simulator_id().to_owned(),
        aircraft_id: model.aircraft_id().to_owned(),
        aircraft_file_digest: model.aircraft_file_digest().to_owned(),
        sample_rate_hz: model.sample_rate_hz(),
        motor_count: model.motor_count(),
        lane_order: model.lane_order(),
    }
}

fn initial_state() -> VehicleState {
    VehicleState {
        orientation: [1.0, 0.0, 0.0, 0.0],
        valid: true,
        ..VehicleState::default()
    }
}
