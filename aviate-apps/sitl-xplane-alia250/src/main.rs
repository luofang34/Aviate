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

    let identifying = args.iter().any(|arg| arg == "--identify");
    // The identification flight must not fly the gains it exists to
    // derive; it uses the known-flyable default cascade instead.
    let kernel = match if identifying {
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
    let mut board = match XPlaneBoard::with_config(
        kernel,
        XPlaneConfig {
            simulator_addr: bridge,
            lane_order: Some(motors::to_airframe_order),
            ..XPlaneConfig::default()
        },
    ) {
        Ok(board) => board,
        Err(error) => {
            log::error!("board construction failed: {error}");
            return ExitCode::FAILURE;
        }
    };

    if identifying {
        log::info!("dialing the X-Plane bridge at {bridge} (identification flight)");
        identify::run(&mut board);
        return ExitCode::SUCCESS;
    }
    if args.iter().any(|arg| arg == "--sweep") {
        log::info!("dialing the X-Plane bridge at {bridge} (collective sweep)");
        identify::run_sweep(&mut board);
        return ExitCode::SUCCESS;
    }

    board.init_telemetry(&config, SENSOR_RATE_HZ);
    if !board.telemetry_enabled() {
        log::warn!("running without an estimate stream (see errors above)");
    }
    log::info!("dialing the X-Plane bridge at {bridge}");

    run(&mut board, auto_arm)
}

/// The control loop. Never returns: a link that is down is not a fatal
/// condition — the simulator is allowed to restart underneath.
fn run<C, M>(board: &mut XPlaneBoard<C, M>, auto_arm: Option<Duration>) -> !
where
    C: aviate_core::control::VehicleController,
    M: aviate_core::mixer::Mixer,
{
    let started = Instant::now();
    let mut last_heartbeat = Instant::now();
    let mut last_report = Instant::now();
    let mut was_connected = false;
    let mut armed = false;

    loop {
        let cycle_start = Instant::now();
        let command = board.step();

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
                 phase={phase} {fix} motors=[{outputs}]"
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
