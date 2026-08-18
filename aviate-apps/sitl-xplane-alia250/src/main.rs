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

mod motors;

use std::process::ExitCode;
use std::time::{Duration, Instant};

use aviate_board_sitl_xplane::{XPlaneBoard, XPlaneConfig};

/// The control-loop period. The bridge paces sensor samples on actuator
/// feedback, so this is an upper bound on how often the loop looks for
/// work, not a rate the simulation is held to.
const CYCLE_PERIOD: Duration = Duration::from_micros(2_500);

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

    let kernel = match aviate_app_sitl_xplane_alia250_kernel::build_alia250_kernel() {
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

    let loop_hz = u32::try_from(1_000_000 / CYCLE_PERIOD.as_micros()).unwrap_or(400);
    board.init_telemetry(&config, loop_hz);
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
        board.step();

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
            log::info!(
                "link rx={rx} tx={tx} crc_errors={crc} unsent={unsent} connects={connects} \
                 ready={} armed={}",
                board.is_ready(),
                board.is_armed()
            );
            last_report = Instant::now();
        }

        if let Some(remaining) = CYCLE_PERIOD.checked_sub(cycle_start.elapsed()) {
            std::thread::sleep(remaining);
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
