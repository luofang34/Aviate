//! Flight runtime for real hardware (DO-178C path)
//!
//! Phase 1: Stub implementation (unimplemented!())
//! Phase 4+: Full implementation migrated from micoair-h743-v2-test app
//!
//! # Safety Requirements
//!
//! - NO simulator dependencies (enforced by features)
//! - NO debugging/testing-only paths
//! - Only real board HALs allowed
//! - Task separation: control (high-DAL), telemetry (low-DAL), command (low-DAL)

use aviate_config::AppConfig;

/// Flight-safe runtime for real hardware
///
/// Generic over Board and Airframe types for compile-time optimization.
///
/// # Type Parameters
///
/// - `Board`: Board HAL providing sensor/actuator access (e.g., `micoair_h743_v2::Board`)
/// - `Airframe`: Airframe dynamics (e.g., `multirotor::QuadX`)
pub struct AppRuntime<Board, Airframe> {
    _board: core::marker::PhantomData<Board>,
    _airframe: core::marker::PhantomData<Airframe>,
}

impl<Board, Airframe> AppRuntime<Board, Airframe> {
    /// Run the flight application (never returns)
    ///
    /// Phase 1: Stub (unimplemented!())
    /// Phase 4+: Implement task loop with:
    /// - control_task() (high-DAL): sensors → EKF → controller → actuators
    /// - telemetry_task() (low-DAL): queue → transport
    /// - command_task() (low-DAL): gateway → execution
    pub fn run(_config: &AppConfig) -> ! {
        unimplemented!("Phase 1: flight.rs stub - full implementation in Phase 4+")
    }
}
