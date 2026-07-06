#![deny(missing_docs)]
//! Multirotor airframe definitions.
//!
//! This crate provides airframe implementations for multirotor vehicles
//! (quadcopters, hexacopters, octocopters, etc.).
//!
//! # Architecture
//!
//! All multirotors share the `MultirotorController` from aviate-core,
//! but use different mixers based on motor count and layout.
//!
//! # Features
//!
//! - `quad-x` (default) - Quadcopter X configuration
//! - `quad-plus` - Quadcopter + configuration
//! - `hex-x` - Hexacopter X configuration
//! - `x500` - PX4 X500 quadcopter model (uses quad-x)
//!
//! # Example
//!
//! ```ignore
//! use aviate_airframe_multirotor::GenericQuadX;
//! use aviate_board_sitl_gazebo::SitlGazeboBoard;
//!
//! type Board = SitlGazeboBoard<GenericQuadX>;
//! ```

#![no_std]
#![forbid(unsafe_code)]
#![forbid(clippy::panic)]
#![forbid(clippy::unwrap_used)]
#![forbid(clippy::expect_used)]

// Re-export Airframe trait so apps can use it without depending on aviate-core directly
pub use aviate_core::airframe::Airframe;

use aviate_core::control::multirotor::MultirotorController;
use aviate_core::control::ConfigMode;
use aviate_core::mixer::{ModeConfig, QuadXMixer};
use aviate_core::time::{TimeSource, Timestamp};

/// Motor layout for multirotor category.
///
/// Defines the physical arrangement of motors on a multirotor vehicle.
/// Different layouts require different mixer implementations.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MotorLayout {
    /// 4 motors, X configuration (standard quadcopter)
    QuadX,
    /// 4 motors, + configuration
    QuadPlus,
    /// 6 motors, X configuration
    HexX,
    /// 6 motors, + configuration
    HexPlus,
    /// 8 motors, X configuration
    OctoX,
    /// 8 motors, + configuration
    OctoPlus,
}

/// Category-specific trait for multirotors (extends core Airframe).
///
/// Provides the motor layout which is specific to multirotor vehicles.
/// Boards don't need to know the layout - only motor count is exposed
/// through the base `Airframe` trait.
pub trait MultirotorAirframe: Airframe {
    /// Physical motor layout of this airframe
    const LAYOUT: MotorLayout;
}

/// Generic quad-x airframe (default, works out of box).
///
/// AIRFRAME_ID = "generic-quad-x" (not tuned for any specific model)
///
/// This is a generic quadcopter with reasonable default gains that
/// should work for most X-configuration quadcopters. For better
/// performance, use a model-specific airframe like `X500Airframe`.
pub struct GenericQuadX;

/// Default timestamp function for GenericQuadX mixer
fn generic_timestamp() -> Timestamp {
    Timestamp {
        ticks: 0,
        source: TimeSource::Internal,
    }
}

impl Airframe for GenericQuadX {
    type Controller = MultirotorController;
    type Mixer = QuadXMixer;

    const MOTOR_COUNT: u8 = 4;
    const AIRFRAME_ID: &'static str = "generic-quad-x";
    const CATEGORY: &'static str = "multirotor";

    fn create_controller() -> Self::Controller {
        MultirotorController::default()
    }

    fn create_mixer() -> Self::Mixer {
        QuadXMixer {
            timestamp_source: generic_timestamp,
        }
    }

    fn mode_config() -> ModeConfig {
        ModeConfig {
            mode: ConfigMode::Hover,
            groups: &[],
        }
    }
}

impl MultirotorAirframe for GenericQuadX {
    const LAYOUT: MotorLayout = MotorLayout::QuadX;
}

// X500 model (feature-gated)
#[cfg(feature = "x500")]
pub mod models;

#[cfg(feature = "x500")]
pub use models::x500::X500Airframe;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_generic_quad_x_constants() {
        assert_eq!(GenericQuadX::MOTOR_COUNT, 4);
        assert_eq!(GenericQuadX::AIRFRAME_ID, "generic-quad-x");
        assert_eq!(GenericQuadX::CATEGORY, "multirotor");
        assert_eq!(GenericQuadX::LAYOUT, MotorLayout::QuadX);
    }

    #[test]
    fn test_generic_quad_x_create_controller() {
        let _controller = GenericQuadX::create_controller();
    }

    #[test]
    fn test_generic_quad_x_create_mixer() {
        let _mixer = GenericQuadX::create_mixer();
    }

    #[test]
    fn test_generic_quad_x_mode_config() {
        let config = GenericQuadX::mode_config();
        assert_eq!(config.mode, ConfigMode::Hover);
    }

    #[test]
    fn test_motor_layout_clone() {
        let layout = MotorLayout::QuadX;
        let cloned = layout;
        assert_eq!(layout, cloned);
    }

    #[test]
    fn test_motor_layout_all_variants() {
        // Verify all variants exist and are distinct
        let layouts = [
            MotorLayout::QuadX,
            MotorLayout::QuadPlus,
            MotorLayout::HexX,
            MotorLayout::HexPlus,
            MotorLayout::OctoX,
            MotorLayout::OctoPlus,
        ];
        for i in 0..layouts.len() {
            for j in (i + 1)..layouts.len() {
                assert_ne!(layouts[i], layouts[j]);
            }
        }
    }
}
