//! X500 JSBSim SITL FC binary.
//!
//! ```text
//! JSBSim sidecar ── TCP 4560+instance ── this binary
//!   HIL_STATE_QUATERNION (truth, one per 4 ms step) ──▶ JsbsimBoard
//!                                                       │ synthesize sensors
//!                                                       │ one kernel cycle
//!   HIL_ACTUATOR_CONTROLS (one per state) ◀──────────── X500 rotor map
//! ```
//!
//! The sidecar owns the simulation clock and the pacing: this binary
//! runs one kernel cycle per truth state and never paces against wall
//! time. The airframe is the app's: the X500 kernel and its rotor map
//! are injected into the airframe-neutral board.

mod ground_station;

use std::time::Duration;

use aviate_app_sitl_jsbsim_x500::x500_controls;
use aviate_board_sitl_jsbsim::{JsbsimBoard, JsbsimConfig};
use aviate_hal_xil::synthesis::{GeoOrigin, NoiseTier};
use world_core::DecimalYear;

/// The app configuration, embedded at build time (low-DAL parse
/// happens once at startup; the control loop never touches TOML).
const APP_CONFIG_TOML: &str = include_str!("../AviateApp.toml");

/// The sidecar's physics rate, which is this binary's kernel rate.
const SIDECAR_STEP_HZ: u32 = 250;

/// How long to wait for a truth state before polling again. Only the
/// link-state log uses the wake-up; a state is taken the moment it
/// arrives.
const TRUTH_WAIT: Duration = Duration::from_millis(100);

/// `AVIATE_SITL_INSTANCE` selects the instance: its sidecar port and
/// XIL ports. Unset is instance 0.
const INSTANCE_ENV: &str = "AVIATE_SITL_INSTANCE";

/// Parse + validate the embedded app config. An unparseable or
/// invalid config refuses startup.
fn load_app_config() -> std::io::Result<aviate_config::AppConfig> {
    let mut cfg = aviate_config::from_toml_str(APP_CONFIG_TOML)
        .map_err(|e| std::io::Error::other(format!("AviateApp.toml failed to parse: {e:?}")))?;
    aviate_config::validate(&cfg)
        .map_err(|e| std::io::Error::other(format!("AviateApp.toml failed validation: {e:?}")))?;
    apply_telemetry_endpoint_override(&mut cfg);
    Ok(cfg)
}

/// `AVIATE_TELEMETRY_ENDPOINT=host:port` re-points the telemetry
/// stream for multi-instance setups.
fn apply_telemetry_endpoint_override(cfg: &mut aviate_config::AppConfig) {
    let Ok(endpoint) = std::env::var("AVIATE_TELEMETRY_ENDPOINT") else {
        return;
    };
    for transport in &mut cfg.transports {
        if transport.roles.iter().any(|r| r == "telemetry") {
            log::info!("telemetry endpoint override: {endpoint}");
            transport.endpoint = Some(endpoint);
            return;
        }
    }
    log::warn!("AVIATE_TELEMETRY_ENDPOINT set but no telemetry-role transport in AviateApp.toml");
}

/// The instance `AVIATE_SITL_INSTANCE` states; a malformed value
/// refuses startup rather than flying on another instance's ports.
fn instance_from_env() -> std::io::Result<u8> {
    match std::env::var(INSTANCE_ENV) {
        Ok(text) => text.trim().parse().map_err(|_| {
            std::io::Error::other(format!(
                "{INSTANCE_ENV}={text:?} is not an instance 0..=255"
            ))
        }),
        Err(_) => Ok(0),
    }
}

/// Today's date on the host clock, for the simulated Earth field.
fn decimal_year_now() -> DecimalYear {
    let seconds = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_or(0, |d| i64::try_from(d.as_secs()).unwrap_or(i64::MAX));
    DecimalYear::from_unix_seconds(seconds)
}

fn main() -> std::io::Result<()> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();
    log::info!("aviate sitl-jsbsim-x500 starting");

    let app_config = load_app_config()?;
    let instance = instance_from_env()?;
    // The origin must be the sidecar's spawn point: the launcher
    // states the same point to both.
    let origin = GeoOrigin::from_env().map_err(std::io::Error::other)?;
    let mut config = JsbsimConfig::for_instance(instance, origin, decimal_year_now());
    config.noise = NoiseTier::from_env();
    if let Some(endpoint) = ground_station::telemetry_endpoint(&app_config) {
        config.xil.gcs_addr = endpoint;
    }
    log::info!(
        "instance {instance}: sidecar {}, origin {:.7}, {:.7}, {} m, noise {:?}",
        config.sidecar_addr,
        origin.position.latitude_deg,
        origin.position.longitude_deg,
        origin.position.ellipsoidal_height_m,
        config.noise
    );

    let kernel = aviate_app_sitl_gazebo_x500_kernel::build_x500_kernel()
        .map_err(|e| std::io::Error::other(format!("X500 kernel: {e:?}")))?;
    let mut board =
        JsbsimBoard::new(kernel, config, x500_controls).map_err(std::io::Error::other)?;
    board.init_telemetry(&app_config, SIDECAR_STEP_HZ);
    if !board.telemetry_enabled() {
        log::warn!("running without a telemetry stream (see errors above)");
    }

    let mut connected = false;
    loop {
        board.wait_for_truth(TRUTH_WAIT);
        while board.step().is_some() {}
        if board.connected() != connected {
            connected = board.connected();
            if connected {
                log::info!("JSBSim sidecar connected");
            } else {
                log::warn!("JSBSim sidecar link down; redialing");
            }
        }
    }
}
