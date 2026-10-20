//! `sitl-jsbsim-missions`: run the SITL missions on JSBSim.
//!
//! ```text
//! sitl-jsbsim-missions [--missions DIR] SELECTION...
//! ```
//!
//! Each SELECTION is `blocking` (every mission the manifest gates on), a
//! shard of `DIR/manifest.json` (every mission it puts in that shard), or
//! one mission by name. A mission runs
//! the manifest's `runs` times and must pass `pass_threshold` of them; a
//! mission named directly that the manifest does not shard runs once and
//! must pass. Each run flies the X500 against the JSBSim sidecar of the
//! mission vehicle's instance (`127.0.0.1:4560+instance`), which starts a
//! new world for the run; the world origin is `AVIATE_GNSS_ORIGIN`, the
//! point the sidecar was started at. Exits non-zero when any mission
//! misses its bar.

use std::path::{Path, PathBuf};
use std::process::ExitCode;
use std::time::{Instant, SystemTime, UNIX_EPOCH};

use aviate_app_sitl_jsbsim_x500::x500_controls;
use aviate_board_sitl_jsbsim::{JsbsimBoard, JsbsimConfig, JsbsimSimulatorBackend};
use aviate_core::control::multirotor::MultirotorController;
use aviate_core::mixer::QuadXMixerX500;
use aviate_hal_xil::synthesis::GeoOrigin;
use aviate_hal_xil::{parse_test_config, run_test_config, SimulatorError, TestResult};
use tracing::{error, info};
use world_core::DecimalYear;

type Backend = JsbsimSimulatorBackend<MultirotorController, QuadXMixerX500>;

/// One mission and the reliability bar it must meet.
struct Selected {
    name: String,
    runs: u32,
    pass_threshold: u32,
}

fn main() -> ExitCode {
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new("info")),
        )
        .init();
    match run() {
        Ok(true) => ExitCode::SUCCESS,
        Ok(false) => ExitCode::FAILURE,
        Err(message) => {
            error!("{message}");
            ExitCode::from(2)
        }
    }
}

/// Run every selected mission; `Ok(false)` when one missed its bar.
fn run() -> Result<bool, String> {
    let (dir, selection) = parse_args(std::env::args().skip(1).collect())?;
    let manifest = read_manifest(&dir)?;
    let missions = select(&manifest, &selection)?;
    let origin = GeoOrigin::from_env().map_err(|e| e.to_string())?;
    let mut all_passed = true;
    for mission in &missions {
        all_passed &= fly(&dir, mission, origin)?;
    }
    Ok(all_passed)
}

fn parse_args(args: Vec<String>) -> Result<(PathBuf, Vec<String>), String> {
    let mut dir = PathBuf::from("tests/missions");
    let mut selection = Vec::new();
    let mut args = args.into_iter();
    while let Some(arg) = args.next() {
        if arg == "--missions" {
            dir = PathBuf::from(args.next().ok_or("--missions needs a directory")?);
        } else {
            selection.push(arg);
        }
    }
    if selection.is_empty() {
        return Err(
            "usage: sitl-jsbsim-missions [--missions DIR] blocking|SHARD|MISSION...".to_owned(),
        );
    }
    Ok((dir, selection))
}

fn read_manifest(dir: &Path) -> Result<serde_json::Value, String> {
    let path = dir.join("manifest.json");
    let text =
        std::fs::read_to_string(&path).map_err(|e| format!("reading {}: {e}", path.display()))?;
    serde_json::from_str(&text).map_err(|e| format!("parsing {}: {e}", path.display()))
}

/// The missions a selection names, with their bars from the manifest.
fn select(manifest: &serde_json::Value, selection: &[String]) -> Result<Vec<Selected>, String> {
    let entries = manifest["missions"]
        .as_object()
        .ok_or("manifest.json has no missions table")?;
    let bar = |name: &str, entry: &serde_json::Value| {
        let count = |key: &str| entry[key].as_u64().and_then(|n| u32::try_from(n).ok());
        Selected {
            name: name.to_owned(),
            runs: count("runs").unwrap_or(1),
            pass_threshold: count("pass_threshold").unwrap_or(1),
        }
    };
    let mut selected = Vec::new();
    for wanted in selection {
        let member = |entry: &serde_json::Value| {
            if wanted == "blocking" {
                entry["gate"].as_str() == Some("blocking")
            } else {
                entry["shard"].as_str() == Some(wanted.as_str())
            }
        };
        let shard: Vec<Selected> = entries
            .iter()
            .filter(|(_, entry)| member(entry))
            .map(|(name, entry)| bar(name, entry))
            .collect();
        if !shard.is_empty() {
            selected.extend(shard);
        } else if let Some(entry) = entries.get(wanted) {
            selected.push(bar(wanted, entry));
        } else {
            return Err(format!("{wanted:?} is neither a shard nor a mission"));
        }
    }
    Ok(selected)
}

/// Fly one mission up to its run count; true when it met its bar.
fn fly(dir: &Path, mission: &Selected, origin: GeoOrigin) -> Result<bool, String> {
    let path = dir.join(format!("{}.toml", mission.name));
    let config = parse_test_config(&path).map_err(|e| format!("{}: {e}", path.display()))?;
    let started = Instant::now();
    let mut passes = 0;
    for run in 1..=mission.runs {
        let result = run_test_config(&config, move |instance| backend(instance, origin));
        report_run(&mission.name, run, &result);
        passes += u32::from(result.passed);
        let remaining = mission.runs - run;
        if passes >= mission.pass_threshold || passes + remaining < mission.pass_threshold {
            break;
        }
    }
    let met = passes >= mission.pass_threshold;
    let verdict = if met { "PASS" } else { "FAIL" };
    info!(
        "{verdict} {}: {passes} of {} needed passes, {:.1} s wall clock",
        mission.name,
        mission.pass_threshold,
        started.elapsed().as_secs_f64()
    );
    Ok(met)
}

fn report_run(name: &str, run: u32, result: &TestResult) {
    info!(
        "{name} run {run}: {} in {:.1} s",
        if result.passed { "passed" } else { "failed" },
        result.duration.as_secs_f64()
    );
    for vehicle in &result.vehicle_results {
        for phase in &vehicle.phases {
            for criterion in phase.criteria_results.iter().filter(|c| !c.passed) {
                error!(
                    "{name} run {run} phase {}: {} expected {}, got {}",
                    phase.name, criterion.criterion, criterion.expected, criterion.actual_value
                );
            }
        }
    }
}

/// A backend for the X500 of `instance`, on that instance's sidecar.
fn backend(instance: u8, origin: GeoOrigin) -> Result<Backend, SimulatorError> {
    let config = JsbsimConfig::for_instance(instance, origin, decimal_year_now());
    let kernel = aviate_app_sitl_gazebo_x500_kernel::build_x500_kernel().map_err(|e| {
        SimulatorError::ConnectionFailed {
            backend: "jsbsim".to_owned(),
            detail: format!("X500 kernel: {e:?}"),
        }
    })?;
    let board = JsbsimBoard::new(kernel, config, x500_controls).map_err(|e| {
        SimulatorError::ConnectionFailed {
            backend: "jsbsim".to_owned(),
            detail: e.to_string(),
        }
    })?;
    Ok(JsbsimSimulatorBackend::new(board, instance))
}

/// Today's date on the host clock, for the simulated Earth field.
fn decimal_year_now() -> DecimalYear {
    let seconds = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_or(0, |d| i64::try_from(d.as_secs()).unwrap_or(i64::MAX));
    DecimalYear::from_unix_seconds(seconds)
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::panic)]
#[path = "missions/tests.rs"]
mod tests;
