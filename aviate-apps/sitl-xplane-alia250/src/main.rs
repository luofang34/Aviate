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
mod flight_loop;
mod identify;
mod startup;
mod tuning_trace;

use std::process::ExitCode;

fn main() -> ExitCode {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();

    let cli = match cli::Cli::parse(std::env::args().skip(1)) {
        Ok(cli) => cli,
        Err(error) => {
            log::error!("{error}");
            return ExitCode::FAILURE;
        }
    };
    match startup::start(cli) {
        Ok(exit_code) => exit_code,
        Err(error) => {
            log::error!("{error}");
            ExitCode::FAILURE
        }
    }
}
