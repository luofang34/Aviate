//! Alia-250 lift rotors on X-Plane SITL.
//!
//! The simulator's bridge plugin listens on TCP 4560; this binary dials
//! it, feeds the HIL sensor stream into the kernel, and answers each
//! sample with the mixer's command. The simulator is NOT launched from
//! here: it is an operator-owned desktop application whose lifetime
//! outlives any one flight-controller run, so the link simply retries
//! until the bridge is listening.
//!
//! Usage:
//!   sitl-xplane-alia250 [--bridge HOST:PORT] [--auto-arm SECONDS]
//!       --runtime-handshake FILE
//!       [--run-manifest FILE] [--candidate FILE --plant-artifact FILE]
//!       [--identify --plant-output FILE --trace-output FILE]
//!       [--tuning-trace-endpoint 127.0.0.1:PORT]
//!
//! `--identify` flies the plant-identification experiment instead of
//! serving a session: a short hop, per-axis attitude square waves, and
//! a printed measurement of each axis's angular authority K. It runs
//! under the same kernel the app flies, so the numbers it prints are
//! the numbers that kernel's derivation should be fed.

mod artifact;
mod cli;
mod identify;
mod tuning_trace;

use aviate_core::ekf::Estimator as _;

use std::process::ExitCode;
use std::time::{Duration, Instant};

use aviate_board_sitl_xplane::{XPlaneBoard, XPlaneConfig};

/// How long one iteration waits for the bridge's next sample before
/// looking around anyway.
///
/// The loop is paced by SAMPLE ARRIVAL, not by this period: the bridge
/// holds its next sample until the previous one is answered, so waiting
/// out a fixed period after each answer would leave the simulation
/// waiting on the flight controller. The wait exists only so a loop with
/// no link still runs its timers and retries.
const IDLE_WAIT: Duration = Duration::from_micros(2_500);

/// The app configuration, embedded so a deployment cannot drift from
/// the binary it runs.
const APP_CONFIG_TOML: &str = include_str!("../AviateApp.toml");
const AIRFRAME_PRESET_TOML: &str = include_str!("../../../presets/alia250.toml");
/// The simulator boundary, embedded with the app binary.
const XPLANE_MODEL_TOML: &str = include_str!("../../../presets/alia250-xplane.toml");

fn main() -> ExitCode {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();

    let cli = match cli::Cli::parse(std::env::args().skip(1)) {
        Ok(cli) => cli,
        Err(message) => {
            log::error!("{message}");
            return ExitCode::FAILURE;
        }
    };
    let bridge = cli.bridge;
    let auto_arm = cli.auto_arm;

    let config = match load_app_config() {
        Ok(config) => config,
        Err(message) => {
            log::error!("{message}");
            return ExitCode::FAILURE;
        }
    };
    let xplane_model = match load_xplane_model(&config.app.airframe) {
        Ok(model) => model,
        Err(message) => {
            log::error!("{message}");
            return ExitCode::FAILURE;
        }
    };
    let model_digest = match xplane_model.canonical_digest() {
        Ok(digest) => digest,
        Err(error) => {
            log::error!("X-Plane model identity failed: {error}");
            return ExitCode::FAILURE;
        }
    };
    let sensor_rate_hz = u32::from(xplane_model.sample_rate_hz());
    log::info!("X-Plane model identity={model_digest}");
    let expected_model =
        match aviate_config::airframe_preset::ContentDigest::from_hex(&model_digest.to_string()) {
            Ok(digest) => digest,
            Err(error) => {
                log::error!("X-Plane model identity is incompatible: {error}");
                return ExitCode::FAILURE;
            }
        };
    let runtime_binding = match cli.claim_runtime_handshake() {
        Ok(binding) => binding,
        Err(error) => {
            log::error!("runtime handshake failed: {error}");
            return ExitCode::FAILURE;
        }
    };

    let experiment = cli.experiment;
    let experiment_flight = experiment.is_some();
    let calibration_inputs = match cli.calibration_inputs() {
        Ok(inputs) => inputs,
        Err(message) => {
            log::error!("{message}");
            return ExitCode::FAILURE;
        }
    };
    // The identification flight must not fly the gains it exists to
    // derive; it uses the known-flyable default cascade instead.
    let kernel_result = if experiment_flight {
        aviate_app_sitl_xplane_alia250_kernel::build_alia250_identification_kernel()
            .map(|kernel| (kernel, None))
    } else if let Some(inputs) = calibration_inputs.as_ref() {
        match aviate_app_sitl_xplane_alia250_kernel::build_alia250_kernel_with_candidate(
            &inputs.candidate,
            &inputs.plant_artifact,
            &xplane_model,
        ) {
            Ok(built) => Ok((built.kernel, Some(built.manifest))),
            Err(error) => Err(error),
        }
    } else {
        aviate_app_sitl_xplane_alia250_kernel::build_alia250_kernel().map(|kernel| (kernel, None))
    };
    let (kernel, calibration_manifest) = match kernel_result {
        Ok(built) => built,
        Err(error) => {
            log::error!("{error}");
            return ExitCode::FAILURE;
        }
    };
    let purpose = experiment.unwrap_or(if calibration_manifest.is_some() {
        aviate_app_sitl_xplane_alia250_kernel::RunPurpose::Candidate
    } else {
        aviate_app_sitl_xplane_alia250_kernel::RunPurpose::Normal
    });
    let build_identity = match aviate_app_sitl_xplane_alia250_kernel::BuildIdentity::current() {
        Ok(identity) => identity,
        Err(error) => {
            log::error!("build identity failed: {error}");
            return ExitCode::FAILURE;
        }
    };
    let run_manifest = match aviate_app_sitl_xplane_alia250_kernel::AliaRunManifest::new(
        &kernel,
        expected_model,
        purpose,
        calibration_manifest.as_ref(),
        build_identity,
        runtime_binding.content_digest,
    ) {
        Ok(manifest) => manifest,
        Err(error) => {
            log::error!("run manifest failed: {error}");
            return ExitCode::FAILURE;
        }
    };
    let run_manifest_text = run_manifest.to_toml();
    let run_manifest_digest =
        aviate_config::airframe_preset::ContentDigest::calculate(run_manifest_text.as_bytes());
    log::info!("run manifest:\n{run_manifest_text}");
    if let Err(error) = artifact::publish_optional(cli.run_manifest.as_deref(), &run_manifest_text)
    {
        log::error!("{error}");
        return ExitCode::FAILURE;
    }
    let trace_config = match tuning_trace::config(
        cli.tuning_trace_endpoint,
        &run_manifest,
        run_manifest_digest,
    ) {
        Ok(config) => config,
        Err(error) => {
            log::error!("tuning trace configuration failed: {error}");
            return ExitCode::FAILURE;
        }
    };
    let mut board_config = XPlaneConfig::new(bridge, xplane_model);
    if let Some(trace) = trace_config {
        board_config = board_config.with_tuning_trace(trace);
    }
    let mut board = match XPlaneBoard::with_config(kernel, board_config) {
        Ok(board) => board,
        Err(error) => {
            log::error!("board construction failed: {error}");
            return ExitCode::FAILURE;
        }
    };
    if let Err(error) = board.accept_runtime_handshake(runtime_binding.handshake) {
        log::error!("runtime handshake rejected: {error}");
        return ExitCode::FAILURE;
    }

    if experiment == Some(aviate_app_sitl_xplane_alia250_kernel::RunPurpose::Identify) {
        log::info!("dialing the X-Plane bridge at {bridge} (identification flight)");
        let (artifact, trace_text) =
            match identify::run(&mut board, run_manifest_digest.to_string()) {
                Ok(output) => output,
                Err(error) => {
                    log::error!("identification failed: {error}");
                    return ExitCode::FAILURE;
                }
            };
        let artifact_text = match artifact.to_toml() {
            Ok(text) => text,
            Err(error) => {
                log::error!("plant artifact encoding failed: {error}");
                return ExitCode::FAILURE;
            }
        };
        log::info!("plant artifact:\n{artifact_text}");
        if let Err(error) = artifact::publish_optional(cli.trace_output.as_deref(), &trace_text) {
            log::error!("{error}");
            return ExitCode::FAILURE;
        }
        if let Err(error) = artifact::publish_optional(cli.plant_output.as_deref(), &artifact_text)
        {
            log::error!("{error}");
            return ExitCode::FAILURE;
        }
        return ExitCode::SUCCESS;
    }
    // The sweep flies the identification kernel for the same reason
    // --identify does: it must not depend on the gains it informs.
    if experiment == Some(aviate_app_sitl_xplane_alia250_kernel::RunPurpose::YawSign) {
        log::info!("dialing the X-Plane bridge at {bridge} (yaw-sign probe)");
        return experiment_exit(identify::run_yaw_sign(&mut board));
    }
    if experiment == Some(aviate_app_sitl_xplane_alia250_kernel::RunPurpose::Sweep) {
        log::info!("dialing the X-Plane bridge at {bridge} (collective sweep)");
        return experiment_exit(identify::run_sweep(&mut board));
    }

    // Simulation truth rides the SAME estimate stream, sent by this
    // app from beside the runner's own telemetry: the bridge's
    // HIL_STATE_QUATERNION is the simulator's oracle, and recording an
    // estimate without the truth it should be judged against wastes
    // the whole point of simulating. Flight builds have no simulator
    // and no such stream to carry.
    let truth_tx = config
        .transports
        .iter()
        .find(|t| t.roles.iter().any(|role| role == "telemetry"))
        .and_then(|t| t.endpoint.clone())
        .and_then(|endpoint| {
            let socket = std::net::UdpSocket::bind("0.0.0.0:0").ok()?;
            socket.connect(&endpoint).ok()?;
            log::info!("sim-truth forwarding to {endpoint}");
            Some(socket)
        });
    if truth_tx.is_none() {
        log::warn!("no telemetry endpoint; sim truth will not be forwarded");
    }

    board.init_telemetry(&config, sensor_rate_hz);
    if !board.telemetry_enabled() {
        log::warn!("running without an estimate stream (see errors above)");
    }
    log::info!("dialing the X-Plane bridge at {bridge}");

    run(&mut board, auto_arm, truth_tx)
}

/// The control loop. A required identity or trace failure ends the run.
fn run<C, M>(
    board: &mut XPlaneBoard<C, M>,
    auto_arm: Option<Duration>,
    truth_tx: Option<std::net::UdpSocket>,
) -> ExitCode
where
    C: aviate_core::control::VehicleController,
    M: aviate_core::mixer::Mixer,
{
    let started = Instant::now();
    let mut last_heartbeat = Instant::now();
    let mut last_report = Instant::now();
    let mut last_truth = Instant::now();
    let mut truth_seq: u8 = 0;
    let mut truth_forwarded = false;
    let mut was_connected = false;
    let mut armed = false;

    loop {
        let cycle_start = Instant::now();
        let command = board.step();
        if let Some(error) = board.runtime_handshake_failure() {
            log::error!("runtime handshake failed during run: {error}");
            board.terminate();
            return ExitCode::FAILURE;
        }
        if let Some(error) = board.tuning_trace_failure() {
            log::error!("tuning trace failed during run: {error}");
            board.terminate();
            return ExitCode::FAILURE;
        }

        // Forward the simulator's ground truth onto the estimate
        // stream at 10 Hz, so every estimate arrives beside the truth
        // it can be judged against.
        if let Some(truth) = board.take_truth() {
            if last_truth.elapsed() >= Duration::from_millis(100) {
                if let Some(socket) = truth_tx.as_ref() {
                    let mut buf = [0u8; 256];
                    if let Some(len) = aviate_backend_mavlink_hil::serialize_frame(
                        &aviate_backend_mavlink_hil::messages::HilMessage::StateQuaternion(truth),
                        truth_seq,
                        1,
                        1,
                        &mut buf,
                    ) {
                        if !truth_forwarded {
                            truth_forwarded = true;
                            log::info!("first sim-truth frame forwarded");
                        }
                        truth_seq = truth_seq.wrapping_add(1);
                        if let Err(error) = socket.send(&buf[..len]) {
                            log::debug!("sim-truth send failed: {error}");
                        }
                    }
                }
                last_truth = Instant::now();
            }
        }

        let connected = board.connected();
        if connected != was_connected {
            log::info!(
                "{}",
                if connected {
                    "HIL link up"
                } else {
                    "HIL link down; retrying"
                }
            );
            was_connected = connected;
        }

        if last_heartbeat.elapsed() >= Duration::from_secs(1) {
            board.send_heartbeat();
            last_heartbeat = Instant::now();
        }

        if !armed && board.is_ready() {
            if let Some(delay) = auto_arm {
                if started.elapsed() >= delay {
                    match board.arm() {
                        Ok(()) => {
                            log::info!("auto-armed");
                            armed = true;
                        }
                        Err(error) => log::warn!("auto-arm refused: {error:?}"),
                    }
                }
            }
        }

        if last_report.elapsed() >= Duration::from_secs(5) {
            let (rx, tx, crc, unsent, connects) = board.stats();
            // The simulated receiver's own fix, so a flight is read from
            // a measurement rather than inferred from the link counters.
            let fix = board.last_fix().map_or_else(
                || "fix=none".to_owned(),
                |fix| {
                    format!(
                        "fix={:?} sats={} n={:.1}m e={:.1}m d={:.1}m alt={:.1}m",
                        fix.fix,
                        fix.satellites,
                        fix.position_ned[0],
                        fix.position_ned[1],
                        fix.position_ned[2],
                        fix.alt_m
                    )
                },
            );
            // The commanded motor lanes, so a vehicle that will not fly
            // separates a command that never arrived from thrust that
            // was never enough.
            let outputs = command.outputs[..4]
                .iter()
                .map(|lane| format!("{:.2}", lane.0))
                .collect::<Vec<_>>()
                .join(",");
            // The estimator's vertical state beside the receiver's:
            // when they disagree, the vertical loop is flying the
            // estimate, and THAT is the number to read.
            let est = board
                .kernel()
                .pipeline()
                .estimator
                .estimate(&board.kernel().state.estimator);
            let vz_est = est.velocity_ned[2].0;
            let dz_est = est.position_ned[2].0;
            // One lifecycle phase, not two booleans: `ready` means
            // ready TO ARM, so an armed kernel is legitimately not
            // "ready" and printing both reads as a fault.
            let phase = if board.is_armed() {
                "armed"
            } else if board.is_ready() {
                "ready"
            } else {
                "init"
            };
            log::info!(
                "link rx={rx} tx={tx} crc_errors={crc} unsent={unsent} connects={connects} \
                 phase={phase} {fix} est_d={dz_est:.1}m est_vz={vz_est:.2} motors=[{outputs}]"
            );
            last_report = Instant::now();
        }

        // Block on the link rather than on the clock, so the answer to
        // each sample leaves within microseconds of its arrival. With
        // no link there is nothing to block on, so the loop falls back
        // to the idle wait rather than spinning a core.
        if !board.wait_for_sample(IDLE_WAIT) && !board.connected() {
            if let Some(remaining) = IDLE_WAIT.checked_sub(cycle_start.elapsed()) {
                std::thread::sleep(remaining);
            }
        }
    }
}

/// Loads and validates the embedded configuration, honoring the
/// telemetry endpoint override a multi-instance deployment needs.
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
    let preset_digest =
        aviate_config::airframe_preset::ContentDigest::calculate(AIRFRAME_PRESET_TOML.as_bytes());
    if model.airframe_preset_digest() != preset_digest.to_string() {
        return Err("X-Plane model does not match the embedded airframe preset".to_owned());
    }
    Ok(model)
}

fn experiment_exit(result: Result<(), identify::ExperimentError>) -> ExitCode {
    match result {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            log::error!("experiment failed: {error}");
            ExitCode::FAILURE
        }
    }
}
