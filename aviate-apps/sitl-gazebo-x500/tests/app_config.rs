//! The embedded AviateApp.toml is the telemetry contract's source:
//! if it stops parsing, validating, or carrying the telemetry-role
//! endpoint transport, the FC silently reverts to publishing no
//! estimate stream — exactly the regression this file pins.

#![allow(clippy::expect_used, clippy::panic)]

const APP_CONFIG_TOML: &str = include_str!("../AviateApp.toml");

#[test]
fn embedded_config_parses_validates_and_names_a_telemetry_endpoint() {
    let cfg = aviate_config::from_toml_str(APP_CONFIG_TOML).expect("AviateApp.toml parses");
    aviate_config::validate(&cfg).expect("AviateApp.toml validates");

    let telem = cfg.telemetry.as_ref().expect("[telemetry] section");
    assert!(telem.heartbeat_hz > 0);
    // Responsiveness floors: this stream drives a glass panel over
    // loopback. Radio-class rates render as a visibly stepping
    // PFD/HSI (#280) — refuse a silent regression below a smooth
    // attitude cadence and a usable position cadence.
    assert!(
        telem.attitude_hz >= 25,
        "attitude below panel-responsiveness floor"
    );
    assert!(
        telem.position_hz >= 10,
        "position below panel-responsiveness floor"
    );
    assert!(telem.estimator_status_hz > 0);

    let transport = cfg
        .transports
        .iter()
        .find(|t| t.roles.iter().any(|r| r == "telemetry"))
        .expect("a telemetry-role transport");
    let endpoint = transport
        .endpoint
        .as_ref()
        .expect("telemetry transport carries an endpoint");
    endpoint
        .parse::<std::net::SocketAddr>()
        .expect("endpoint is host:port");
}
