//! Perturbation configuration validation tests.

use super::*;

#[test]
fn invalid_factor_bounds_fail_before_execution() {
    let mut config = actuator_config();
    config.actuator.authority_scale_basis_points = 4_999;
    assert!(matches!(
        PerturbationEngine::new(config),
        Err(PerturbationError::InvalidAuthorityScale(4_999))
    ));

    let mut config = sensor_config(7);
    config.sensor_noise[0].peak_amplitude = f32::NAN;
    assert!(matches!(
        PerturbationEngine::new(config),
        Err(PerturbationError::InvalidSensorAmplitude(
            SensorLane::AccelerometerX
        ))
    ));
}
