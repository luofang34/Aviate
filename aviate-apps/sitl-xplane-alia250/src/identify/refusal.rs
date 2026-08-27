//! The identification experiment's refusal vocabulary: every way one
//! run can fail, each carrying what diagnosing it needs.

use super::stand;

/// Failure of one identification experiment.
#[derive(Debug)]
pub(crate) enum ExperimentError {
    /// The experiment made no safe progress before its wall-clock guard expired.
    Timeout(&'static str),
    /// The kernel refused to arm.
    Arm(aviate_core::ArmError),
    /// The kernel refused to disarm.
    Disarm(aviate_core::DisarmError),
    /// The climb phase ended with the vehicle still on its gear.
    NeverLifted {
        /// The fix altitude when the climb budget expired.
        alt_m: f32,
    },
    /// The vehicle contacted the ground inside an excitation window.
    GroundContact {
        /// Which window was flying when contact happened.
        window: &'static str,
        /// The fix altitude at contact.
        alt_m: f32,
        /// The encoded sample trace up to the contact.
        trace_text: String,
    },
    /// The X-Plane test stand did not confirm an operation.
    Stand(stand::StandError),
    /// The process could not open the X-Plane test-stand socket.
    StandSocket(std::io::Error),
    /// Simulator sample time did not increase.
    ClockRegression { previous_us: u64, next_us: u64 },
    /// Runtime identity or sample-clock evidence failed.
    RuntimeHandshake(aviate_board_sitl_xplane::RuntimeHandshakeError),
    /// The external tuning trace did not accept a packet.
    TuningTrace(String),
    /// Plant fitting rejected the trace; the trace itself rides along
    /// so a refused experiment still leaves its evidence behind.
    Report {
        /// The refusing gate's own words.
        reason: String,
        /// The encoded sample trace of the refused run; empty when the
        /// refusing experiment records no sample trace.
        trace_text: String,
    },
}

impl core::fmt::Display for ExperimentError {
    fn fmt(&self, formatter: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Timeout(name) => write!(formatter, "{name} timed out"),
            Self::Arm(error) => write!(formatter, "arm failed: {error:?}"),
            Self::Disarm(error) => write!(formatter, "disarm failed: {error:?}"),
            Self::NeverLifted { alt_m } => {
                if alt_m.is_finite() {
                    write!(
                        formatter,
                        "the climb budget expired with the vehicle still on its \
                         gear (fix altitude {alt_m:.1} m); rotor spool or thrust \
                         never overcame weight"
                    )
                } else {
                    write!(
                        formatter,
                        "the climb budget expired without a GNSS fix ever \
                         arriving; the achieved-height gate could not observe \
                         the vehicle at all"
                    )
                }
            }
            Self::GroundContact { window, alt_m, .. } => write!(
                formatter,
                "the vehicle contacted the ground during the {window} window \
                 (fix altitude {alt_m:.1} m); a grounded trace fits gear \
                 friction, not the plant"
            ),
            Self::Stand(error) => write!(formatter, "test stand failed: {error}"),
            Self::StandSocket(error) => write!(formatter, "test stand socket failed: {error}"),
            Self::ClockRegression {
                previous_us,
                next_us,
            } => write!(
                formatter,
                "simulator clock did not increase: {previous_us} then {next_us}"
            ),
            Self::RuntimeHandshake(error) => write!(formatter, "runtime handshake failed: {error}"),
            Self::TuningTrace(error) => write!(formatter, "tuning trace failed: {error}"),
            Self::Report { reason, .. } => write!(formatter, "plant report failed: {reason}"),
        }
    }
}

impl std::error::Error for ExperimentError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Stand(error) => Some(error),
            Self::StandSocket(error) => Some(error),
            Self::RuntimeHandshake(error) => Some(error),
            Self::Timeout(_)
            | Self::GroundContact { .. }
            | Self::NeverLifted { .. }
            | Self::Arm(_)
            | Self::Disarm(_)
            | Self::ClockRegression { .. }
            | Self::TuningTrace(_)
            | Self::Report { .. } => None,
        }
    }
}
