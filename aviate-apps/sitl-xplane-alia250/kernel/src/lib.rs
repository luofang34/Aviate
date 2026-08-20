//! App-owned kernel construction for the Alia-250 lift rotors.
//!
//! Airframe selection is an application decision: this app states that
//! it flies the four lift rotors of the simulator's Alia-250 as a
//! quad-X, builds the resolved configuration, and constructs through
//! the checked builder. The board receives the kernel by injection and
//! never chooses an airframe.
//!
//! The rotor arrangement comes from the simulated airframe: lift rotors
//! at ±3.0 m longitudinally and ±2.5 m laterally, with the front-right
//! and rear-left pair spinning CLOCKWISE — the X500's diagonals with
//! every spin direction reversed, which is why the kernel mixes through
//! `MixerGeometry::QuadXX500ReversedSpin`. On the X500 mixer this
//! airframe's yaw loop is positive feedback, measured in flight as a
//! spin that winds up to the attitude loop's rate command limit.
//!
//! The embedded `alia250.toml` preset is the only source for the normal
//! kernel gains, limits, hover seed, mixer, and actuator curve.

mod construct;
mod tuning;

pub use construct::{
    build_alia250_identification_kernel, build_alia250_kernel, build_alia250_kernel_with_candidate,
    AliaKernel, AliaKernelBuildError, CalibratedAliaKernel, CalibrationRunManifest,
};
