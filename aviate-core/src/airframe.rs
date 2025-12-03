//! Airframe trait for compile-time vehicle configuration.
//!
//! This module defines the `Airframe` trait which provides a compile-time
//! description of a vehicle type, binding a specific controller and mixer
//! implementation to a fixed motor count and airframe identifier.
//!
//! # Architecture
//!
//! Boards and applications never construct controllers/mixers directly;
//! they go through this trait. This enables:
//!
//! - Zero-cost abstraction via monomorphization
//! - Same board code works with any airframe
//! - Same airframe code works on SITL and real hardware
//!
//! # Example
//!
//! ```ignore
//! // In aviate-airframes/multirotor/src/lib.rs
//! pub struct X500Airframe;
//!
//! impl Airframe for X500Airframe {
//!     type Controller = MultirotorController;
//!     type Mixer = QuadXMixer;
//!
//!     const MOTOR_COUNT: u8 = 4;
//!     const AIRFRAME_ID: &'static str = "x500";
//!     const CATEGORY: &'static str = "multirotor";
//!
//!     fn create_controller() -> Self::Controller { ... }
//!     fn create_mixer() -> Self::Mixer { ... }
//!     fn mode_config() -> ModeConfig { ... }
//! }
//!
//! // In board code
//! type Board = SitlGazeboBoard<X500Airframe>;
//! ```

#![forbid(unsafe_code)]

use crate::control::VehicleController;
use crate::mixer::{Mixer, ModeConfig};

/// Airframe is a compile-time description of a vehicle type.
///
/// It binds a specific controller and mixer implementation to a
/// fixed motor count and airframe identifier.
///
/// Boards and applications never construct controllers/mixers
/// directly; they go through this trait.
///
/// # Design Notes
///
/// - `MotorLayout` is NOT in this trait - it's defined per-category
///   (multirotor, fixed-wing, etc.) since different categories have
///   completely different layout concepts.
///
/// - `timestamp_source` is NOT passed to `create_mixer()` - the
///   kernel/board owns the clock and injects time during `Mixer::mix()`.
///
/// - `CATEGORY` is `&'static str` for now. Future versions may use
///   an enum for type safety.
///
/// - `mode_config()` must be **deterministic and pure** (no side effects).
///   It declares supported modes, setpoint types, limits, and failsafe defaults.
pub trait Airframe {
    /// Closed-loop controller used for this airframe.
    ///
    /// For multirotors: typically `MultirotorController`
    /// For fixed-wing: typically `FixedWingController`
    /// For VTOL: a hybrid controller that blends both
    type Controller: VehicleController;

    /// Mixer mapping high-level axis commands to individual motors/actuators.
    ///
    /// For quad-x: `QuadXMixer`
    /// For hex: `HexXMixer`
    /// etc.
    type Mixer: Mixer;

    /// Number of motors/actuators driven by this mixer.
    ///
    /// This is the only motor-related information exposed to the board.
    /// Layout details are category-specific (e.g., `MultirotorAirframe::LAYOUT`).
    const MOTOR_COUNT: u8;

    /// Stable identifier for this airframe.
    ///
    /// Naming convention: lowercase, hyphen-separated.
    /// Examples: `"generic-quad-x"`, `"x500"`, `"tailsitter-v1"`
    ///
    /// Used in logs, black box recordings, and telemetry.
    const AIRFRAME_ID: &'static str;

    /// Category string for this airframe.
    ///
    /// Examples: `"multirotor"`, `"fixed-wing"`, `"vtol"`
    ///
    /// TODO: Consider replacing with an `AirframeCategory` enum for type safety.
    const CATEGORY: &'static str;

    /// Construct the controller with airframe-specific gains/limits.
    ///
    /// Called once during board initialization. The returned controller
    /// should be configured with gains appropriate for this specific
    /// airframe (e.g., X500-specific PID tuning).
    fn create_controller() -> Self::Controller;

    /// Construct the mixer.
    ///
    /// The kernel/board is responsible for providing the time source
    /// to `Mixer::mix()`; the airframe does not own the clock.
    fn create_mixer() -> Self::Mixer;

    /// Declarative description of supported modes and their defaults.
    ///
    /// This must be **deterministic and pure** - no side effects.
    /// It declares:
    /// - Supported flight modes (MANUAL, ALTCTL, POSCTL, AUTO, etc.)
    /// - Setpoint types and limits per mode
    /// - Failsafe/default mode configuration
    /// - Actuator group configurations
    fn mode_config() -> ModeConfig;
}
