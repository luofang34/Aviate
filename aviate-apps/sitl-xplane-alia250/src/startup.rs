//! Fail-closed construction of one Alia X-Plane run.

use std::process::ExitCode;

use aviate_app_sitl_xplane_alia250_kernel::{
    AliaKernel, AliaRunManifest, CalibrationRunManifest, HoverInitializationEvidence,
    RunExecutionIdentity, RunPurpose,
};
use aviate_board_sitl_xplane::{XPlaneBoard, XPlaneConfig, XPlaneTuningTraceConfig};
use aviate_config::airframe_preset::ContentDigest;
use aviate_core::control::multirotor::MultirotorController;
use aviate_core::mixer::QuadXMixerReversedSpin;
use aviate_hal_xil::perturbation::LoadedPerturbationArtifact;

use crate::cli::{CalibrationInputs, ClaimedRuntimeHandshake, Cli};

const APP_CONFIG_TOML: &str = include_str!("../AviateApp.toml");
const AIRFRAME_PRESET_TOML: &str = include_str!("../../../presets/alia250.toml");
const XPLANE_MODEL_TOML: &str = include_str!("../../../presets/alia250-xplane.toml");

type AliaBoard = XPlaneBoard<MultirotorController, QuadXMixerReversedSpin>;

pub(super) fn start(cli: Cli) -> Result<ExitCode, String> {
    let environment = load_environment(&cli)?;
    let execution = load_execution(&cli)?;
    let built = build_kernel(&environment.xplane_model, &execution)?;
    let manifest = build_manifest(&environment, &execution, &built)?;
    let manifest_text = manifest.to_toml();
    let manifest_digest = ContentDigest::calculate(manifest_text.as_bytes());
    log::info!("run manifest:\n{manifest_text}");
    crate::artifact::publish_optional(cli.run_manifest.as_deref(), &manifest_text)
        .map_err(|error| error.to_string())?;
    let trace = crate::tuning_trace::config(cli.tuning_trace_endpoint, &manifest, manifest_digest)
        .map_err(|error| format!("tuning trace configuration failed: {error}"))?;
    let (mut board, app_config, sensor_rate_hz) = construct_board(
        environment,
        execution.condition.as_ref(),
        built,
        &manifest,
        trace,
    )?;
    dispatch(
        &cli,
        &mut board,
        &app_config,
        sensor_rate_hz,
        manifest_digest,
    )
}

struct Environment {
    app_config: aviate_config::AppConfig,
    xplane_model: aviate_config::xplane_model::XPlaneSimulatorModel,
    model_identity: ContentDigest,
    runtime_binding: ClaimedRuntimeHandshake,
    sensor_rate_hz: u32,
}

struct ExecutionInputs {
    experiment: Option<RunPurpose>,
    calibration: Option<CalibrationInputs>,
    condition: Option<LoadedPerturbationArtifact>,
}

struct BuiltExecution {
    kernel: AliaKernel,
    calibration_manifest: Option<CalibrationRunManifest>,
    hover: HoverInitializationEvidence,
    purpose: RunPurpose,
}

fn load_environment(cli: &Cli) -> Result<Environment, String> {
    let app_config = load_app_config()?;
    let xplane_model = load_xplane_model(&app_config.app.airframe)?;
    let model_digest = xplane_model
        .canonical_digest()
        .map_err(|error| format!("X-Plane model identity failed: {error}"))?;
    let model_identity = ContentDigest::from_hex(&model_digest.to_string())
        .map_err(|error| format!("X-Plane model identity is incompatible: {error}"))?;
    let runtime_binding = cli
        .claim_runtime_handshake()
        .map_err(|error| format!("runtime handshake failed: {error}"))?;
    let sensor_rate_hz = u32::from(xplane_model.sample_rate_hz());
    log::info!("X-Plane model identity={model_digest}");
    Ok(Environment {
        app_config,
        xplane_model,
        model_identity,
        runtime_binding,
        sensor_rate_hz,
    })
}

fn load_execution(cli: &Cli) -> Result<ExecutionInputs, String> {
    let calibration = cli
        .calibration_inputs()
        .map_err(|error| error.to_string())?;
    let condition = cli
        .load_condition_artifact()
        .map_err(|error| format!("condition artifact failed: {error}"))?;
    Ok(ExecutionInputs {
        experiment: cli.experiment,
        calibration,
        condition,
    })
}

fn build_kernel(
    model: &aviate_config::xplane_model::XPlaneSimulatorModel,
    inputs: &ExecutionInputs,
) -> Result<BuiltExecution, String> {
    let scale = inputs
        .condition
        .as_ref()
        .map_or(10_000, LoadedPerturbationArtifact::hover_scale_basis_points);
    if inputs.experiment.is_some() {
        let built = aviate_app_sitl_xplane_alia250_kernel::build_alia250_identification_kernel_with_hover_scale(scale)
            .map_err(|error| error.to_string())?;
        return Ok(BuiltExecution {
            kernel: built.kernel,
            calibration_manifest: None,
            hover: built.hover_initialization,
            purpose: inputs.experiment.unwrap_or(RunPurpose::Identify),
        });
    }
    if let Some(calibration) = &inputs.calibration {
        let built = aviate_app_sitl_xplane_alia250_kernel::build_alia250_kernel_with_candidate_and_hover_scale(
            &calibration.candidate,
            &calibration.plant_artifact,
            model,
            scale,
        )
        .map_err(|error| error.to_string())?;
        return Ok(BuiltExecution {
            kernel: built.kernel,
            calibration_manifest: Some(built.manifest),
            hover: built.hover_initialization,
            purpose: RunPurpose::Candidate,
        });
    }
    let built = aviate_app_sitl_xplane_alia250_kernel::build_alia250_kernel_with_hover_scale(scale)
        .map_err(|error| error.to_string())?;
    Ok(BuiltExecution {
        kernel: built.kernel,
        calibration_manifest: None,
        hover: built.hover_initialization,
        purpose: RunPurpose::Normal,
    })
}

fn build_manifest(
    environment: &Environment,
    inputs: &ExecutionInputs,
    built: &BuiltExecution,
) -> Result<AliaRunManifest, String> {
    let build = aviate_app_sitl_xplane_alia250_kernel::BuildIdentity::current()
        .map_err(|error| format!("build identity failed: {error}"))?;
    AliaRunManifest::new(
        &built.kernel,
        built.purpose,
        built.calibration_manifest.as_ref(),
        build,
        RunExecutionIdentity {
            simulator_model: environment.model_identity,
            runtime_handshake: environment.runtime_binding.content_digest,
            hover_initialization: built.hover,
            perturbation: inputs
                .condition
                .as_ref()
                .map(|artifact| artifact.identity().clone()),
        },
    )
    .map_err(|error| format!("run manifest failed: {error}"))
}

fn construct_board(
    environment: Environment,
    condition: Option<&LoadedPerturbationArtifact>,
    built: BuiltExecution,
    manifest: &AliaRunManifest,
    trace: Option<XPlaneTuningTraceConfig>,
) -> Result<(AliaBoard, aviate_config::AppConfig, u32), String> {
    let mut config = XPlaneConfig::new(
        environment
            .runtime_binding
            .handshake
            .bridge_endpoint
            .parse()
            .map_err(|error| format!("runtime bridge endpoint failed: {error}"))?,
        environment.xplane_model,
    )
    .with_hover_initialization(aviate_board_sitl_xplane::XPlaneHoverInitialization {
        baseline_force_bits: built.hover.baseline_force_bits,
        effective_force_bits: built.hover.effective_force_bits,
        scale_basis_points: built.hover.scale_basis_points,
        kernel_config_hash: built.hover.effective_kernel_config_hash,
    });
    if let Some(trace) = trace {
        config = config.with_tuning_trace(trace);
    }
    if let Some(artifact) = condition {
        let identity = manifest
            .perturbation_artifact_identity()
            .ok_or_else(|| "run manifest did not bind the condition artifact".to_owned())?;
        config = config
            .with_verified_perturbation(artifact, identity)
            .map_err(|error| format!("condition artifact binding failed: {error}"))?;
    }
    let mut board = XPlaneBoard::with_config(built.kernel, config)
        .map_err(|error| format!("board construction failed: {error}"))?;
    board
        .accept_runtime_handshake(environment.runtime_binding.handshake)
        .map_err(|error| format!("runtime handshake rejected: {error}"))?;
    Ok((board, environment.app_config, environment.sensor_rate_hz))
}

fn dispatch(
    cli: &Cli,
    board: &mut AliaBoard,
    app_config: &aviate_config::AppConfig,
    sensor_rate_hz: u32,
    manifest_digest: ContentDigest,
) -> Result<ExitCode, String> {
    match cli.experiment {
        Some(RunPurpose::Identify) => run_identification(cli, board, manifest_digest),
        Some(RunPurpose::YawSign) => {
            log::info!(
                "dialing the X-Plane bridge at {} (yaw-sign probe)",
                cli.bridge
            );
            experiment_exit(crate::identify::run_yaw_sign(board))
        }
        Some(RunPurpose::Sweep) => {
            log::info!(
                "dialing the X-Plane bridge at {} (collective sweep)",
                cli.bridge
            );
            experiment_exit(crate::identify::run_sweep(board))
        }
        Some(RunPurpose::Normal | RunPurpose::Candidate) | None => {
            run_normal(cli, board, app_config, sensor_rate_hz)
        }
    }
}

fn run_identification(
    cli: &Cli,
    board: &mut AliaBoard,
    manifest_digest: ContentDigest,
) -> Result<ExitCode, String> {
    log::info!(
        "dialing the X-Plane bridge at {} (identification flight)",
        cli.bridge
    );
    let (plant, trace_text) = crate::identify::run(board, manifest_digest.to_string())
        .map_err(|error| format!("identification failed: {error}"))?;
    let plant_text = plant
        .to_toml()
        .map_err(|error| format!("plant artifact encoding failed: {error}"))?;
    log::info!("plant artifact:\n{plant_text}");
    crate::artifact::publish_optional(cli.trace_output.as_deref(), &trace_text)
        .map_err(|error| error.to_string())?;
    crate::artifact::publish_optional(cli.plant_output.as_deref(), &plant_text)
        .map_err(|error| error.to_string())?;
    Ok(ExitCode::SUCCESS)
}

fn run_normal(
    cli: &Cli,
    board: &mut AliaBoard,
    app_config: &aviate_config::AppConfig,
    sensor_rate_hz: u32,
) -> Result<ExitCode, String> {
    let truth_tx = truth_socket(app_config);
    if truth_tx.is_none() {
        log::warn!("no telemetry endpoint; sim truth will not be forwarded");
    }
    board.init_telemetry(app_config, sensor_rate_hz);
    if !board.telemetry_enabled() {
        log::warn!("running without an estimate stream (see errors above)");
    }
    log::info!("dialing the X-Plane bridge at {}", cli.bridge);
    Ok(crate::flight_loop::run(board, cli.auto_arm, truth_tx))
}

fn truth_socket(config: &aviate_config::AppConfig) -> Option<std::net::UdpSocket> {
    config
        .transports
        .iter()
        .find(|transport| transport.roles.iter().any(|role| role == "telemetry"))
        .and_then(|transport| transport.endpoint.clone())
        .and_then(|endpoint| {
            let socket = std::net::UdpSocket::bind("0.0.0.0:0").ok()?;
            socket.connect(&endpoint).ok()?;
            log::info!("sim-truth forwarding to {endpoint}");
            Some(socket)
        })
}

fn load_app_config() -> Result<aviate_config::AppConfig, String> {
    let mut config = aviate_config::from_toml_str(APP_CONFIG_TOML)
        .map_err(|error| format!("AviateApp.toml failed to parse: {error:?}"))?;
    aviate_config::validate(&config)
        .map_err(|error| format!("AviateApp.toml failed validation: {error:?}"))?;
    if let Ok(endpoint) = std::env::var("AVIATE_TELEMETRY_ENDPOINT") {
        for transport in &mut config.transports {
            if transport.roles.iter().any(|role| role == "telemetry") {
                log::info!("telemetry endpoint override: {endpoint}");
                transport.endpoint = Some(endpoint);
                break;
            }
        }
    }
    Ok(config)
}

fn load_xplane_model(
    expected_airframe: &str,
) -> Result<aviate_config::xplane_model::XPlaneSimulatorModel, String> {
    let model = aviate_config::xplane_model::XPlaneSimulatorModel::from_toml_str(XPLANE_MODEL_TOML)
        .map_err(|error| format!("Alia X-Plane model failed validation: {error}"))?;
    if model.airframe_id() != expected_airframe {
        return Err(format!(
            "X-Plane model airframe {:?} does not match app airframe {expected_airframe:?}",
            model.airframe_id()
        ));
    }
    let preset_digest = ContentDigest::calculate(AIRFRAME_PRESET_TOML.as_bytes());
    if model.airframe_preset_digest() != preset_digest.to_string() {
        return Err("X-Plane model does not match the embedded airframe preset".to_owned());
    }
    Ok(model)
}

fn experiment_exit(
    result: Result<(), crate::identify::ExperimentError>,
) -> Result<ExitCode, String> {
    result
        .map(|()| ExitCode::SUCCESS)
        .map_err(|error| format!("experiment failed: {error}"))
}
