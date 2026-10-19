//! Heading-reference telemetry tests, through the cycle formatter.

use aviate_core::sensor::{GeomagneticReference, GeomagneticSource};
use aviate_core::types::Radians;

use crate::mavlink::protocol::{
    aviate_geomagnetic_source, parse_mavlink, AviateGeomagneticReference, MavMessage,
};
use crate::mavlink::telemetry::{MavlinkCycleFormatter, TelemetryConfig};
use crate::queue::DefaultTelemetryQueue;
use crate::telemetry::{TelemetryCycleFormatter, TelemetrySnapshot};

/// The reference frames of one heartbeat cycle.
fn reference_frame(snapshot: &TelemetrySnapshot) -> Option<AviateGeomagneticReference> {
    let mut formatter = MavlinkCycleFormatter::new(&TelemetryConfig::default(), 4).ok()?;
    let mut queue = DefaultTelemetryQueue::new();
    formatter.format_cycle(snapshot, &mut queue);
    let mut found = None;
    while queue.pop_with(|frame| {
        if let Ok((MavMessage::AviateGeomagneticReference(r), _, _)) = parse_mavlink(frame) {
            found = Some(r);
        }
    }) {}
    found
}

#[test]
fn the_applied_declination_and_its_source_are_published() {
    for (source, wire) in [
        (GeomagneticSource::Model, aviate_geomagnetic_source::MODEL),
        (
            GeomagneticSource::Configured,
            aviate_geomagnetic_source::CONFIGURED,
        ),
    ] {
        let snapshot = TelemetrySnapshot {
            time_ms: 2_000,
            geomagnetic: Some(GeomagneticReference {
                declination: Radians(-0.2113),
                source,
            }),
            ..TelemetrySnapshot::default()
        };
        assert_eq!(
            reference_frame(&snapshot),
            Some(AviateGeomagneticReference {
                time_usec: 2_000_000,
                declination: -0.2113,
                source: wire,
            })
        );
    }
}

#[test]
fn a_magnetic_heading_is_published_as_no_reference() {
    let frame = reference_frame(&TelemetrySnapshot::default());
    assert!(frame
        .is_some_and(|f| f.source == aviate_geomagnetic_source::NONE && f.declination.is_nan()));
}
