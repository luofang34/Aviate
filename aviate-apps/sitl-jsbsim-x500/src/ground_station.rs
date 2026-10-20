//! Where the FC's command link sends its own heartbeats.
//!
//! The command link heartbeats a fixed ground-station address besides
//! any peer that commanded it. That address must be this vehicle's own
//! consumer: an instance beside others streams telemetry to its own
//! endpoint, and a heartbeat sent to the shared default would reach
//! another session's ground station as this system.

use std::net::SocketAddr;

/// The endpoint of the app config's telemetry-role transport, if it
/// names one that parses.
pub(crate) fn telemetry_endpoint(cfg: &aviate_config::AppConfig) -> Option<SocketAddr> {
    cfg.transports
        .iter()
        .find(|transport| transport.roles.iter().any(|role| role == "telemetry"))
        .and_then(|transport| transport.endpoint.as_deref())
        .and_then(|endpoint| endpoint.parse().ok())
}

#[cfg(test)]
mod tests;
