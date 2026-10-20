//! The command link's ground station follows the telemetry endpoint.

#![allow(clippy::expect_used)]

use super::*;

fn config(endpoint: &str) -> aviate_config::AppConfig {
    let text = include_str!("../../AviateApp.toml").replace("127.0.0.1:14550", endpoint);
    aviate_config::from_toml_str(&text).expect("the app config parses")
}

#[test]
fn the_ground_station_is_the_telemetry_endpoint() {
    assert_eq!(
        telemetry_endpoint(&config("127.0.0.1:14550")),
        Some(SocketAddr::from(([127, 0, 0, 1], 14_550)))
    );
    assert_eq!(
        telemetry_endpoint(&config("127.0.0.1:14728")),
        Some(SocketAddr::from(([127, 0, 0, 1], 14_728)))
    );
}

#[test]
fn an_endpoint_that_does_not_parse_names_none() {
    assert_eq!(telemetry_endpoint(&config("nowhere")), None);
}
