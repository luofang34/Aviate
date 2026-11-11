//! In-process `no_std` XIL board, sensor synthesizer, and MAVLink byte link.
//!
//! `aviate-xil-direct` runs an injected [`aviate_core::DefaultAviateKernel`]
//! directly inside a host or embedded simulation loop without OS sockets,
//! shared memory, threads, or heap allocation.

#![no_std]
#![deny(missing_docs)]

#[cfg(test)]
extern crate std;

pub mod board;
pub mod host_sensors;
pub mod link;
pub mod synthesis;

pub use board::{
    DirectActuatorCmd, DirectBoardConfig, DirectBoardHal, DirectCommandRefusal, DirectTime,
    DirectXilBoard, DEFAULT_REBASE_RADIUS_M,
};
pub use host_sensors::{DirectHostSensors, DirectSensorSample};
pub use link::{
    DirectByteRing, DirectMavlinkEndpoint, DirectTelemetryRates, GeodeticTarget,
    DEFAULT_RX_RING_BYTES, DEFAULT_TX_RING_BYTES,
};
pub use synthesis::{
    DirectEnvironment, DirectMeasuredKinematics, DirectNoiseRng, DirectNoiseTier, DirectOrigin,
    DirectSensorRates, DirectStepSample, DirectSynthesisError, DirectSynthesizer,
    DirectTruthSample,
};
