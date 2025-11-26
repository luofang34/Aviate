#![forbid(unsafe_code)]

#[cfg(test)]
mod tests {
    use aviate_core::ekf::Ekf;
    use aviate_core::math::{Vector3, Quaternion};
    use aviate_core::types::{MetersPerSecondSquared, RadiansPerSecond};
    use aviate_core::sensor::ImuData;

    #[test]
    fn test_ekf_init_predict() {
        let mut ekf = Ekf::new();
        assert!(!ekf.is_initialized());

        ekf.init(Vector3::zero(), Vector3::zero(), Quaternion::IDENTITY);
        assert!(ekf.is_initialized());

        // Test prediction with zero input (stationary)
        let imu_zero = ImuData {
            accel: [MetersPerSecondSquared(0.0); 3],
            gyro: [RadiansPerSecond(0.0); 3],
        };
        
        // dt = 0.01s
        ekf.predict(&imu_zero, 0.01);
        
        let est = ekf.get_estimate();
        // Should barely move (biases are zero initially)
        assert!(est.position_ned[0].0.abs() < 1e-5);
    }
    
    #[test]
    fn test_ekf_accel_integration() {
        let mut ekf = Ekf::new();
        ekf.init(Vector3::zero(), Vector3::zero(), Quaternion::IDENTITY);

        // Accel x = 1.0 m/s^2
        let imu_accel = ImuData {
            accel: [MetersPerSecondSquared(1.0), MetersPerSecondSquared(0.0), MetersPerSecondSquared(0.0)],
            gyro: [RadiansPerSecond(0.0); 3],
        };

        let dt = 0.1;
                // Run 10 steps
                for _ in 0..10 {
                    ekf.predict(&imu_accel, dt);
                }
                
                let est = ekf.get_estimate();
                
                // Expected vel = a * t = 1.0 * 1.0 = 1.0
                let vel_x = est.velocity_ned[0].0;
                assert!((vel_x - 1.0).abs() < 0.1, "Velocity X should be ~1.0, got {}", vel_x);
                
                // Expected pos = 0.5 * a * t^2 = 0.5 * 1.0 * 1.0 = 0.5
                let pos_x = est.position_ned[0].0;
                assert!((pos_x - 0.5).abs() < 0.1, "Position X should be ~0.5, got {}", pos_x);
            }
        
            #[test]
            fn test_ekf_gnss_update() {
                use aviate_core::sensor::{GnssData, GnssFix};
                use aviate_core::types::{Meters, MetersPerSecond};
                
                let mut ekf = Ekf::new();
                ekf.init(Vector3::zero(), Vector3::zero(), Quaternion::IDENTITY);
        
                // Predict once to grow uncertainty
                let imu_zero = ImuData {
                    accel: [MetersPerSecondSquared(0.0); 3],
                    gyro: [RadiansPerSecond(0.0); 3],
                };
                ekf.predict(&imu_zero, 1.0);
                
                // Current pos is near 0. P is large.
                
                let gnss = GnssData {
                    position_ned: [Meters(10.0), Meters(0.0), Meters(0.0)], // Measure 10m
                    velocity_ned: [MetersPerSecond(0.0); 3],
                    fix: GnssFix::ThreeD,
                };
                
                ekf.update_gnss(&gnss);
                
                let est = ekf.get_estimate();
                // Estimate should move significantly towards 10.0
                assert!(est.position_ned[0].0 > 1.0, "Position should move towards measurement");
                assert!(est.position_ned[0].0 < 10.0, "Position should not overshoot measurement");
            }
        
        }
        