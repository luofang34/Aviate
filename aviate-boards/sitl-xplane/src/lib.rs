//! X-Plane SITL board.
//!
//! The simulator's bridge plugin listens; this board dials it, feeds
//! the received HIL sensor stream into the kernel through the
//! simulator-neutral `SimSensorPacket` seam, and answers each sample
//! with the mixer's actuator command. Airframe selection stays with
//! the application: the board takes the kernel by injection.

mod backend;
mod board;

pub use backend::XPlaneSimulatorBackend;

pub use board::{
    RuntimeHandshakeError, TuningActuatorApplication, TuningActuatorBypassReason,
    TuningActuatorEligibility, TuningCommand, TuningCommandSource, TuningConfigMode,
    TuningConstraintFlags, TuningControlMode, TuningControlObservation, TuningEstimate,
    TuningEstimateQuality, TuningEstimateValidity, TuningFrameType, TuningHandshake,
    TuningHoverEstimatorMode, TuningHoverInitialization, TuningImu, TuningObservationAck,
    TuningPerturbationCapability, TuningReady, TuningSendEvidence, TuningSensorApplication,
    TuningSetpoint, TuningTraceError, XPlaneBoard, XPlaneConfig, XPlaneConstraintFlags,
    XPlaneControlObservation, XPlaneHoverInitialization, XPlanePerturbationBindingError,
    XPlaneResetError, XPlaneRuntimeHandshake, XPlaneSendEvidence, XPlaneTuningTraceConfig,
    XPlaneTuningTraceIdentity, BOARD_INFO,
};
