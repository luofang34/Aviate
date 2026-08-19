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
//!   sitl-xplane-alia250 [--bridge HOST:PORT] [--auto-arm SECONDS] [--identify]
//!
//! `--identify` flies the plant-identification experiment instead of
//! serving a session: a short hop, per-axis attitude square waves, and
//! a printed measurement of each axis's angular authority K. It runs
//! under the same kernel the app flies, so the numbers it prints are
//! the numbers that kernel's derivation should be fed.

mod identify;
mod motors;

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

/// The rate the bridge delivers sensor samples at, and therefore the
/// rate the kernel steps at. Telemetry divisors are derived from it, so
/// it must match the bridge's configured sensor period.
const SENSOR_RATE_HZ: u32 = 100;

/// The app configuration, embedded so a deployment cannot drift from
/// the binary it runs.
const APP_CONFIG_TOML: &str = include_str!("../AviateApp.toml");

fn main() -> ExitCode {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();

    let args: Vec<String> = std::env::args().skip(1).collect();
    let bridge = match bridge_address(&args) {
        Ok(bridge) => bridge,
        Err(message) => {
            log::error!("{message}");
            return ExitCode::FAILURE;
        }
    };
    let auto_arm = auto_arm_delay(&args);

    let config = match load_app_config() {
        Ok(config) => config,
        Err(message) => {
            log::error!("{message}");
            return ExitCode::FAILURE;
        }
    };

    let experiment_flight = args
        .iter()
        .any(|arg| arg == "--identify" || arg == "--sweep");
    // The identification flight must not fly the gains it exists to
    // derive; it uses the known-flyable default cascade instead.
    let kernel = match if experiment_flight {
        aviate_app_sitl_xplane_alia250_kernel::build_alia250_identification_kernel()
    } else {
        aviate_app_sitl_xplane_alia250_kernel::build_alia250_kernel()
    } {
        Ok(kernel) => kernel,
        Err(error) => {
            log::error!("kernel construction refused: {error:?}");
            return ExitCode::FAILURE;
        }
    };
    // The wire lane order is airframe knowledge: the Alia's channel
    // map wants the [0,2,1,3] permutation; the qtailsitter's channels
    // are the PX4 quad-x order the mixer already emits, and applying
    // the Alia permutation there cross-feeds roll into pitch and flips
    // the vehicle on the ground at 9 % thrust.
    let lane_order = if std::env::var("AVIATE_CASCADE").as_deref() == Ok("x500") {
        None
    } else {
        Some(motors::to_airframe_order as fn(&mut [f32; 16], u8))
    };
    let mut board = match XPlaneBoard::with_config(
        kernel,
        XPlaneConfig {
            simulator_addr: bridge,
            lane_order,
            ..XPlaneConfig::default()
        },
    ) {
        Ok(board) => board,
        Err(error) => {
            log::error!("board construction failed: {error}");
            return ExitCode::FAILURE;
        }
    };

    if args.iter().any(|arg| arg == "--identify") {
        log::info!("dialing the X-Plane bridge at {bridge} (identification flight)");
        identify::run(&mut board);
        return ExitCode::SUCCESS;
    }
    // The sweep flies the identification kernel for the same reason
    // --identify does: it must not depend on the gains it informs.
    if args.iter().any(|arg| arg == "--sweep") {
        log::info!("dialing the X-Plane bridge at {bridge} (collective sweep)");
        identify::run_sweep(&mut board);
        return ExitCode::SUCCESS;
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

    board.init_telemetry(&config, SENSOR_RATE_HZ);
    if !board.telemetry_enabled() {
        log::warn!("running without an estimate stream (see errors above)");
    }
    log::info!("dialing the X-Plane bridge at {bridge}");

    run(&mut board, auto_arm, truth_tx)
}

/// The control loop. Never returns: a link that is down is not a fatal
/// condition — the simulator is allowed to restart underneath.
fn run<C, M>(
    board: &mut XPlaneBoard<C, M>,
    auto_arm: Option<Duration>,
    truth_tx: Option<std::net::UdpSocket>,
) -> !
where
    C: aviate_core::control::VehicleController,
    M: aviate_core::mixer::Mixer,
{
    let started = Instant::now();
    let mut last_heartbeat = Instant::now();
    let mut last_report = Instant::now();
    let mut last_truth = Instant::now();
    let mut truth_seq: u8 = 0;
    let mut was_connected = false;
    let mut armed = false;

    loop {
        let cycle_start = Instant::now();
        let command = board.step();

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
                        truth_seq = truth_seq.wrapping_add(1);
                        if truth_seq == 1 {
                            log::info!("first sim-truth frame forwarded");
                        }
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
                        fix.fix, fix.satellites, fix.position_ned[0], fix.position_ned[1],
                        fix.position_ned[2], fix.alt_m
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

/// Reads `--bridge HOST:PORT`, defaulting to the bridge's own port on
/// loopback. A malformed address is refused, never silently defaulted.
fn bridge_address(args: &[String]) -> Result<std::net::SocketAddr, String> {
    let default = std::net::SocketAddr::from(([127, 0, 0, 1], 4560));
    let Some(index) = args.iter().position(|arg| arg == "--bridge") else {
        return Ok(default);
    };
    let value = args
        .get(index + 1)
        .ok_or_else(|| "--bridge requires HOST:PORT".to_owned())?;
    value
        .parse()
        .map_err(|error| format!("--bridge {value:?} is not HOST:PORT: {error}"))
}

/// Reads `--auto-arm SECONDS`, absent when the flag is not given.
fn auto_arm_delay(args: &[String]) -> Option<Duration> {
    let index = args.iter().position(|arg| arg == "--auto-arm")?;
    let seconds = args.get(index + 1)?.parse().ok()?;
    Some(Duration::from_secs(seconds))
}
