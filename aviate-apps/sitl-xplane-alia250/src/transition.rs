//! Structure for the Alia's full flight envelope, fail-closed.
//!
//! The Alia-250 is a winged eVTOL: four lift rotors, one pusher, and
//! aerodynamic surfaces. This application flies the lift rotors as a
//! quad and nothing else, because the production multirotor controller
//! has no wing-borne modes — `multirotor_mode_capability` names every
//! mode it can run, and none of them is a transition or a fixed-wing
//! law. This module declares the shape a transition capability must
//! fill so the envelope can be discussed, planned, and refused in one
//! typed place, and so a later implementation replaces a refusal rather
//! than inventing a vocabulary.
//!
//! Every wing-regime request refuses. Nothing here reaches an
//! actuator.

/// One regime of the Alia's flight envelope.
///
/// The order is the physical sequence of a full sortie: hover flight,
/// acceleration onto the wing, wing-borne flight, deceleration back
/// onto the rotors. A landing is flown from whichever regime the
/// aircraft is in — hover lands vertically, wing-borne lands on a
/// runway — so landing is a maneuver within a regime, not a regime.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum FlightRegime {
    /// Lift-rotor-borne flight: the regime this application flies.
    Hover,
    /// Accelerating onto the wing: lift rotors unloading, pusher and
    /// surfaces active, airspeed-scheduled.
    TransitionToWing,
    /// Wing-borne flight: lift rotors stopped, pusher and surfaces
    /// only.
    WingBorne,
    /// Decelerating off the wing back onto the lift rotors.
    TransitionToHover,
}

/// The wing-side actuators a transition needs, by bridge channel.
///
/// The numbers are the px4xplane channel map for the Alia airframe:
/// channels zero through three are the lift rotors this application
/// already commands; the wing set starts above them. Declared here so
/// a later implementation and the bridge configuration are held to one
/// vocabulary, and so a mismatch is a compile-site diff rather than a
/// silent cross-mapping.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum WingChannel {
    /// Left aileron group.
    AileronLeft = 4,
    /// Right aileron group.
    AileronRight = 5,
    /// Elevator group.
    Elevator = 6,
    /// Rudder group.
    Rudder = 7,
    /// Tail pusher rotor.
    Pusher = 8,
}

impl WingChannel {
    /// Every wing-side actuator, in bridge channel order.
    pub(crate) const ALL: [Self; 5] = [
        Self::AileronLeft,
        Self::AileronRight,
        Self::Elevator,
        Self::Rudder,
        Self::Pusher,
    ];
}

/// Why a regime request was refused.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum RegimeRefusal {
    /// The production controller has no law for this regime; the
    /// request names the regime so the refusal is auditable.
    NoLawForRegime(FlightRegime),
}

impl core::fmt::Display for RegimeRefusal {
    fn fmt(&self, formatter: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::NoLawForRegime(regime) => {
                write!(formatter, "no production control law for {regime:?}")
            }
        }
    }
}

/// Requests a regime change. Refuses everything except staying in
/// hover, which needs no change.
///
/// A later transition capability replaces this body; its callers keep
/// this signature and this refusal type.
///
/// # Errors
///
/// Returns [`RegimeRefusal::NoLawForRegime`] for every regime the
/// production controller has no law for — today, everything but
/// [`FlightRegime::Hover`].
pub(crate) fn request_regime(target: FlightRegime) -> Result<FlightRegime, RegimeRefusal> {
    match target {
        FlightRegime::Hover => Ok(FlightRegime::Hover),
        other => Err(RegimeRefusal::NoLawForRegime(other)),
    }
}

/// Logs the envelope boundary once, at session start: the regime this
/// session flies, each wing regime's refusal, and the bridge channels
/// left un-commanded. The operator reading the log learns the
/// fail-closed boundary before the first setpoint arrives.
pub(crate) fn announce_envelope() {
    if let Ok(flown) = request_regime(FlightRegime::Hover) {
        log::info!("flight regime: {flown:?}");
    }
    for regime in [
        FlightRegime::TransitionToWing,
        FlightRegime::WingBorne,
        FlightRegime::TransitionToHover,
    ] {
        if let Err(refusal) = request_regime(regime) {
            log::info!("{refusal}");
        }
    }
    let channels = WingChannel::ALL.map(|channel| channel as u8);
    log::info!("wing channels {channels:?} are not commanded; lift rotors only");
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::panic)]
mod tests {
    use super::*;

    /// The fail-closed contract: every regime without a law refuses,
    /// and the refusal names the regime it refused.
    #[test]
    fn every_unimplemented_regime_refuses_by_name() {
        for regime in [
            FlightRegime::TransitionToWing,
            FlightRegime::WingBorne,
            FlightRegime::TransitionToHover,
        ] {
            match request_regime(regime) {
                Err(RegimeRefusal::NoLawForRegime(named)) => assert_eq!(named, regime),
                Ok(granted) => panic!("{regime:?} must refuse, granted {granted:?}"),
            }
        }
    }

    /// Hover is the regime this application flies; asking for it is a
    /// no-op grant, never a refusal.
    #[test]
    fn hover_is_granted() {
        assert_eq!(request_regime(FlightRegime::Hover), Ok(FlightRegime::Hover));
    }

    /// The wing channel numbers are the bridge's channel map for this
    /// airframe. A change on either side must meet this test.
    #[test]
    fn wing_channels_match_the_bridge_channel_map() {
        assert_eq!(WingChannel::AileronLeft as u8, 4);
        assert_eq!(WingChannel::AileronRight as u8, 5);
        assert_eq!(WingChannel::Elevator as u8, 6);
        assert_eq!(WingChannel::Rudder as u8, 7);
        assert_eq!(WingChannel::Pusher as u8, 8);
    }
}
