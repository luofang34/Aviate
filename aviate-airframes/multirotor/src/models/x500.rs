//! PX4 X500 quadcopter airframe.
//!
//! The X500 is a standard quadcopter frame commonly used with PX4.
//! This airframe provides X500-specific tuning and gains.

use crate::{MotorLayout, MultirotorAirframe};
use aviate_core::airframe::Airframe;
use aviate_core::control::cascade_gains::CascadeGains;
use aviate_core::control::multirotor::MultirotorController;
use aviate_core::control::ConfigMode;
use aviate_core::mixer::{ModeConfig, QuadXMixerX500};
use aviate_core::time::{TimeSource, Timestamp};
use aviate_core::types::Scalar;

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

impl X500Airframe {
    /// Hover thrust trim. Newtonian estimate from base + rotor
    /// masses; SITL rig evidence puts the true trim slightly lower
    /// (~0.72–0.75 — the attitude-mode rig climbs at this setting).
    /// The domain question (speed vs thrust) is #140; an online
    /// estimator eventually supersedes the constant.
    pub const HOVER_THRUST_NORM: Scalar = 0.77;

    /// The measured x500 cascade tuning — inherent, not a generic
    /// trait method: multirotor gains have no meaning on the shared
    /// `Airframe` surface (#114).
    pub fn cascade_gains() -> CascadeGains {
        CascadeGains::x500_defaults()
    }
}

/// Default timestamp function for X500 mixer
fn x500_timestamp() -> Timestamp {
    Timestamp {
        ticks: 0,
        source: TimeSource::Internal,
    }
}

impl Airframe for X500Airframe {
    type Controller = MultirotorController;
    // The PX4-gazebo-models X500 spins CW on the FL+RR diagonal —
    // the opposite pattern from the generic QuadXMixer. Using the
    // generic mixer here closes the yaw loop in the wrong direction
    // (positive feedback -> tumble); see the mixer docs.
    type Mixer = QuadXMixerX500;

    const MOTOR_COUNT: u8 = 4;
    const AIRFRAME_ID: &'static str = "x500";
    const CATEGORY: &'static str = "multirotor";

    fn create_controller() -> Self::Controller {
        // Single tuning source: the same values the kernel config
        // hashes (#114).
        MultirotorController::from_gains(
            X500Airframe::cascade_gains(),
            X500Airframe::HOVER_THRUST_NORM,
        )
    }

    fn create_mixer() -> Self::Mixer {
        QuadXMixerX500 {
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
