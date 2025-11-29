use crate::control::ControlLaw;

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum FaultCategory {
    // Sensor faults
    ImuFailed,
    ImuAllFailed,
    GnssLost,
    GnssAllLost,
    BaroFailed,
    MagFailed,
    AirspeedFailed,

    // Actuator faults
    ActuatorFailed,
    ActuatorSaturated,
    ActuatorDisagreement,
    ActuatorNumericError,
    ActuatorFallbackPersistent,

    // Estimation faults
    EstimatorDiverged,
    AttitudeUncertain,
    PositionUncertain,
    NumericError,

    // Command/timing faults
    CommandTimeout,
    CommandInvalid,
    TimingViolation,
    TimingViolationPersistent,
    ConfigInvalid,
    ConfigTransitionFailed,
}

bitflags::bitflags! {
    #[derive(Copy, Clone, Debug, PartialEq, Eq)]
    pub struct FaultFlags: u64 {
        const IMU0_FAILED = 1 << 0;
        const IMU1_FAILED = 1 << 1;
        const IMU2_FAILED = 1 << 2;
        const ALL_IMU_FAILED = 1 << 3;
        const GNSS0_LOST = 1 << 4;
        const GNSS1_LOST = 1 << 5;
        const ALL_GNSS_LOST = 1 << 6;
        const BARO_FAILED = 1 << 7;
        const MAG_FAILED = 1 << 8;
        const AIRSPEED_FAILED = 1 << 9;

        const ACTUATOR_FAULT = 1 << 16;
        const ACTUATOR_NUMERIC = 1 << 17;
        const ACTUATOR_FALLBACK = 1 << 18;

        const ESTIMATOR_DIVERGED = 1 << 24;
        const ATTITUDE_UNCERTAIN = 1 << 25;
        const POSITION_UNCERTAIN = 1 << 26;
        const NUMERIC_ERROR = 1 << 27;

        const COMMAND_TIMEOUT = 1 << 32;
        const COMMAND_INVALID = 1 << 33;
        const TIMING_VIOLATION = 1 << 40;
        const TIMING_PERSISTENT = 1 << 41;
        const CONFIG_INVALID = 1 << 48;
        const CONFIG_TRANSITION_FAILED = 1 << 49;
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum FaultAction {
    Monitor,
    Isolate,
    Degrade,
    Emergency,
}

#[derive(Copy, Clone, Debug)]
pub struct FaultResponse {
    pub fault: FaultCategory,
    pub action: FaultAction,
    pub degrade_to: Option<ControlLaw>,
    pub max_response_time_ms: u32,
}

#[derive(Clone, Debug)]
pub struct FaultHandlingTable {
    pub entries: &'static [FaultResponse],
}

impl FaultHandlingTable {
    pub const DEFAULT: Self = Self {
        entries: &[
            FaultResponse {
                fault: FaultCategory::ImuFailed,
                action: FaultAction::Isolate,
                degrade_to: None,
                max_response_time_ms: 10,
            },
            FaultResponse {
                fault: FaultCategory::ImuAllFailed,
                action: FaultAction::Emergency,
                degrade_to: Some(ControlLaw::Frozen),
                max_response_time_ms: 0,
            },
            FaultResponse {
                fault: FaultCategory::GnssAllLost,
                action: FaultAction::Degrade,
                degrade_to: Some(ControlLaw::Alternate1),
                max_response_time_ms: 100,
            },
            FaultResponse {
                fault: FaultCategory::EstimatorDiverged,
                action: FaultAction::Degrade,
                degrade_to: Some(ControlLaw::Alternate2),
                max_response_time_ms: 10,
            },
            FaultResponse {
                fault: FaultCategory::NumericError,
                action: FaultAction::Emergency,
                degrade_to: Some(ControlLaw::Frozen),
                max_response_time_ms: 0,
            },
            FaultResponse {
                fault: FaultCategory::CommandTimeout,
                action: FaultAction::Degrade,
                degrade_to: Some(ControlLaw::Alternate1),
                max_response_time_ms: 100,
            },
            FaultResponse {
                fault: FaultCategory::ActuatorNumericError,
                action: FaultAction::Monitor,
                degrade_to: None,
                max_response_time_ms: 0,
            },
            FaultResponse {
                fault: FaultCategory::ActuatorFallbackPersistent,
                action: FaultAction::Degrade,
                degrade_to: Some(ControlLaw::Alternate1),
                max_response_time_ms: 10,
            },
            FaultResponse {
                fault: FaultCategory::ConfigTransitionFailed,
                action: FaultAction::Degrade,
                degrade_to: Some(ControlLaw::Alternate1),
                max_response_time_ms: 0,
            },
            FaultResponse {
                fault: FaultCategory::TimingViolationPersistent,
                action: FaultAction::Degrade,
                degrade_to: Some(ControlLaw::Alternate2),
                max_response_time_ms: 50,
            },
        ],
    };
}
