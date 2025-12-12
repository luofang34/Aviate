//! Aviate application runtime
//!
//! This crate provides the control loop, task scheduling, and wiring infrastructure
//! for Aviate flight control applications.
//!
//! # Environment Separation (DO-178C Safety)
//!
//! **Exactly ONE** environment feature must be enabled:
//! - `env-flight`: Real hardware (DO-178C clean, NO simulator code)
//! - `env-sitl`: Software-in-the-loop simulation
//! - `env-hitl`: Hardware-in-the-loop simulation
//!
//! These are **mutually exclusive** and enforced at compile time.
//!
//! # Usage
//!
//! ```toml
//! # Flight build (explicit env required)
//! cargo build --features env-flight --target thumbv7em-none-eabihf
//!
//! # SITL build (explicit env required)
//! cargo build --features env-sitl
//! ```

// ============================================================================
// Environment Feature Guards (DO-178C Safety)
// ============================================================================

// Guard 1: Exactly one environment feature required
#[cfg(not(any(feature = "env-flight", feature = "env-sitl", feature = "env-hitl")))]
compile_error!("Exactly one of env-flight/env-sitl/env-hitl must be enabled");

// Guard 2: env-flight cannot be combined with sim environments
#[cfg(all(
    feature = "env-flight",
    any(feature = "env-sitl", feature = "env-hitl")
))]
compile_error!("env-flight cannot be combined with env-sitl/env-hitl");

// Guard 3: env-sitl and env-hitl are mutually exclusive
#[cfg(all(feature = "env-sitl", feature = "env-hitl"))]
compile_error!("env-sitl and env-hitl are mutually exclusive");

// ============================================================================
// Module Structure
// ============================================================================

pub mod flight;
pub mod sensor_cache;
pub mod sim;
pub mod telemetry;
pub mod validation;

// Re-export telemetry types (available in all environments)
pub use telemetry::{FrameTx, TelemetrySnapshot, TelemetryTask};

// Re-export AppRuntime based on environment
#[cfg(feature = "env-flight")]
pub use flight::AppRuntime;

#[cfg(any(feature = "env-sitl", feature = "env-hitl"))]
pub use sim::{
    // Shared factory functions for SITL boards
    create_kernel,
    default_command,
    loop_periods,
    // Control loop utilities
    run_control_loop,
    sitl_timestamp,
    AppRuntime,
    SitlBoardHal,
    // Shared types
    SitlBoardInfo,
    SitlKernel,
    SitlRunner,
    SitlTime,
};
