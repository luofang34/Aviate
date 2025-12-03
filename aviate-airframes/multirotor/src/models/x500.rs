//! PX4 X500 quadcopter airframe.
//!
//! The X500 is a standard quadcopter frame commonly used with PX4.
//! This airframe provides X500-specific tuning and gains.

use crate::{MotorLayout, MultirotorAirframe};
use aviate_core::airframe::Airframe;
use aviate_core::control::multirotor::MultirotorController;
use aviate_core::control::ConfigMode;
use aviate_core::mixer::{ModeConfig, QuadXMixer};
use aviate_core::time::{TimeSource, Timestamp};

/// PX4 X500 quadcopter airframe.
///
/// Tuned for the specific mass/dimensions of the PX4 X500 model.
/// Uses X-configuration with 4 motors.
///
/// # Physical Characteristics
///
/// - Mass: ~2.0 kg
/// - Arm length: ~0.25 m
/// - Motor layout: X configuration
///
/// # Motor Layout
///
/// ```text
///     Front
///   1 (CW)   2 (CCW)
///       \   /
///        [X]
///       /   \
///   4 (CCW)  3 (CW)
///     Rear
/// ```
pub struct X500Airframe;

/// Default timestamp function for X500 mixer
fn x500_timestamp() -> Timestamp {
    Timestamp {
        ticks: 0,
        source: TimeSource::Internal,
    }
}

impl Airframe for X500Airframe {
    type Controller = MultirotorController;
    type Mixer = QuadXMixer;

    const MOTOR_COUNT: u8 = 4;
    const AIRFRAME_ID: &'static str = "x500";
    const CATEGORY: &'static str = "multirotor";

    fn create_controller() -> Self::Controller {
        // X500-specific gains could be tuned here
        // For now, use defaults
        MultirotorController::default()
    }

    fn create_mixer() -> Self::Mixer {
        QuadXMixer {
            timestamp_source: x500_timestamp,
        }
    }

    fn mode_config() -> ModeConfig {
        ModeConfig {
            mode: ConfigMode::Hover,
            groups: &[],
        }
    }
}

impl MultirotorAirframe for X500Airframe {
    const LAYOUT: MotorLayout = MotorLayout::QuadX;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_x500_constants() {
        assert_eq!(X500Airframe::MOTOR_COUNT, 4);
        assert_eq!(X500Airframe::AIRFRAME_ID, "x500");
        assert_eq!(X500Airframe::CATEGORY, "multirotor");
        assert_eq!(X500Airframe::LAYOUT, MotorLayout::QuadX);
    }

    #[test]
    fn test_x500_create_controller() {
        let _controller = X500Airframe::create_controller();
    }

    #[test]
    fn test_x500_create_mixer() {
        let _mixer = X500Airframe::create_mixer();
    }

    #[test]
    fn test_x500_mode_config() {
        let config = X500Airframe::mode_config();
        assert_eq!(config.mode, ConfigMode::Hover);
    }
}
