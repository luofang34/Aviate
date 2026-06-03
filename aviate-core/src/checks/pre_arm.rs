//! Pre-arm checks (spec §17 InitState transitions).
//!
//! Conditions the kernel must observe before permitting
//! `InitState::PreArm → Ready → Armed`.

use crate::fault::FaultFlags;
use crate::sensor::{SensorHealth, SensorSet};

bitflags::bitflags! {
    /// Pre-arm checks required before InitState::PreArm → Ready → Armed
    ///
    /// Each bit is traceable to a spec requirement.
    /// Vehicle configs specify which flags are required via `CheckStatus.required`.
    #[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
    pub struct PreArmFlags: u32 {
        // === Sensor Health (from SensorReading.health) ===
        // Ref: §15.1 FaultCategory::ImuFailed, BaroFailed, MagFailed, GnssLost

        /// At least one IMU reporting Good health
        const IMU_HEALTHY       = 1 << 0;
        /// At least one barometer reporting Good health
        const BARO_HEALTHY      = 1 << 1;
        /// At least one magnetometer reporting Good health
        const MAG_HEALTHY       = 1 << 2;
        /// GNSS available with valid fix (optional for some vehicles)
        const GNSS_AVAILABLE    = 1 << 3;

        // === Sensor Convergence (sample counts for EKF) ===
        // Ref: §17 InitState::EstimatorConverging

        /// Sufficient IMU samples received for EKF initialization
        const IMU_CONVERGED     = 1 << 4;
        /// Sufficient baro samples received
        const BARO_CONVERGED    = 1 << 5;
        /// Sufficient mag samples received
        const MAG_CONVERGED     = 1 << 6;
        /// EKF state estimate has converged
        const EKF_CONVERGED     = 1 << 7;

        // === Safety Conditions ===

        /// Throttle/collective at low position (stick safety)
        const THROTTLE_LOW      = 1 << 8;
        /// Configuration loaded and valid
        /// Ref: §15.1 FaultCategory::ConfigInvalid
        const CONFIG_VALID      = 1 << 9;
        /// No active faults (FaultFlags::is_empty())
        const NO_FAULTS         = 1 << 10;
        /// Physical hardware safety switch armed (if equipped)
        const HARDWARE_ARMED    = 1 << 11;
        /// Gyro bias calibration complete
        const GYRO_CALIBRATED   = 1 << 12;
        /// Accelerometer calibration valid
        const ACCEL_CALIBRATED  = 1 << 13;

        // === Composite Requirements ===

        /// Minimum checks for basic quadrotor (no GPS required)
        const QUAD_MINIMUM = Self::IMU_HEALTHY.bits()
                           | Self::BARO_HEALTHY.bits()
                           | Self::IMU_CONVERGED.bits()
                           | Self::BARO_CONVERGED.bits()
                           | Self::EKF_CONVERGED.bits()
                           | Self::THROTTLE_LOW.bits()
                           | Self::CONFIG_VALID.bits()
                           | Self::NO_FAULTS.bits();

        /// Full checks for GPS-enabled vehicle
        const QUAD_WITH_GPS = Self::QUAD_MINIMUM.bits()
                            | Self::MAG_HEALTHY.bits()
                            | Self::MAG_CONVERGED.bits()
                            | Self::GNSS_AVAILABLE.bits();
    }
}

/// Sample counts for sensor convergence checks
#[derive(Copy, Clone, Debug, Default)]
pub struct SampleCounts {
    /// Valid IMU samples received
    pub imu: u32,
    /// Valid baro samples received
    pub baro: u32,
    /// Valid mag samples received
    pub mag: u32,
    /// Valid GNSS samples received
    pub gnss: u32,
    /// Minimum samples required for convergence (default: 100 @ 1kHz = 100ms)
    pub min_required: u32,
}

impl SampleCounts {
    /// Default minimum samples for EKF convergence (~100ms at 1kHz)
    pub const DEFAULT_MIN_SAMPLES: u32 = 100;

    pub fn new() -> Self {
        Self {
            min_required: Self::DEFAULT_MIN_SAMPLES,
            ..Default::default()
        }
    }

    /// Reset all counts (e.g., after disarm or fault)
    pub fn reset(&mut self) {
        self.imu = 0;
        self.baro = 0;
        self.mag = 0;
        self.gnss = 0;
    }

    /// Check if IMU has converged
    pub fn imu_converged(&self) -> bool {
        self.imu >= self.min_required
    }

    /// Check if baro has converged
    pub fn baro_converged(&self) -> bool {
        self.baro >= self.min_required
    }

    /// Check if mag has converged
    pub fn mag_converged(&self) -> bool {
        self.mag >= self.min_required
    }
}

/// Check status for PreArmFlags
#[derive(Copy, Clone, Debug)]
pub struct PreArmStatus {
    /// Checks required to pass (configurable per vehicle)
    pub required: PreArmFlags,
    /// Checks currently passing
    pub current: PreArmFlags,
    /// Sample counts for convergence
    pub samples: SampleCounts,
}

impl crate::replicable::Replicable for PreArmFlags {
    const ENCODED_LEN: usize = 4;
    fn encode_canonical(&self, buf: &mut [u8]) -> usize {
        crate::replicable::copy_into(buf, 0, &self.bits().to_le_bytes())
    }
}

impl crate::replicable::Replicable for SampleCounts {
    // 5 × u32 = 20 bytes.
    const ENCODED_LEN: usize = 5 * 4;
    fn encode_canonical(&self, buf: &mut [u8]) -> usize {
        let mut w = 0usize;
        w += crate::replicable::copy_into(buf, w, &self.imu.to_le_bytes());
        w += crate::replicable::copy_into(buf, w, &self.baro.to_le_bytes());
        w += crate::replicable::copy_into(buf, w, &self.mag.to_le_bytes());
        w += crate::replicable::copy_into(buf, w, &self.gnss.to_le_bytes());
        w += crate::replicable::copy_into(buf, w, &self.min_required.to_le_bytes());
        w
    }
}

impl crate::replicable::Replicable for PreArmStatus {
    const ENCODED_LEN: usize =
        PreArmFlags::ENCODED_LEN + PreArmFlags::ENCODED_LEN + SampleCounts::ENCODED_LEN;
    fn encode_canonical(&self, buf: &mut [u8]) -> usize {
        let mut written = self.required.encode_canonical(buf);
        if written < buf.len() {
            written += self.current.encode_canonical(&mut buf[written..]);
        }
        if written < buf.len() {
            written += self.samples.encode_canonical(&mut buf[written..]);
        }
        written
    }
}

impl Default for PreArmStatus {
    fn default() -> Self {
        Self {
            required: PreArmFlags::QUAD_MINIMUM,
            current: PreArmFlags::empty(),
            samples: SampleCounts::new(),
        }
    }
}

impl PreArmStatus {
    /// Create with custom required flags
    pub fn with_required(required: PreArmFlags) -> Self {
        Self {
            required,
            current: PreArmFlags::empty(),
            samples: SampleCounts::new(),
        }
    }

    /// Check if all required checks pass
    pub fn is_satisfied(&self) -> bool {
        self.current.contains(self.required)
    }

    /// Get flags that are required but not passing
    pub fn missing(&self) -> PreArmFlags {
        self.required - self.current
    }

    /// Update checks from sensor data
    pub fn update_from_sensors(&mut self, sensors: &SensorSet) {
        // Update sensor health flags
        let imu_healthy = sensors
            .imus
            .iter()
            .any(|s| s.valid && s.health == SensorHealth::Good);
        let baro_healthy = sensors
            .baros
            .iter()
            .any(|s| s.valid && s.health == SensorHealth::Good);
        let mag_healthy = sensors
            .mags
            .iter()
            .any(|s| s.valid && s.health == SensorHealth::Good);
        let gnss_available = sensors
            .gnss
            .iter()
            .any(|s| s.valid && s.health == SensorHealth::Good);

        self.current.set(PreArmFlags::IMU_HEALTHY, imu_healthy);
        self.current.set(PreArmFlags::BARO_HEALTHY, baro_healthy);
        self.current.set(PreArmFlags::MAG_HEALTHY, mag_healthy);
        self.current
            .set(PreArmFlags::GNSS_AVAILABLE, gnss_available);

        // Update sample counts
        if imu_healthy {
            self.samples.imu = self.samples.imu.saturating_add(1);
        }
        if baro_healthy {
            self.samples.baro = self.samples.baro.saturating_add(1);
        }
        if mag_healthy {
            self.samples.mag = self.samples.mag.saturating_add(1);
        }
        if gnss_available {
            self.samples.gnss = self.samples.gnss.saturating_add(1);
        }

        // Update convergence flags
        self.current
            .set(PreArmFlags::IMU_CONVERGED, self.samples.imu_converged());
        self.current
            .set(PreArmFlags::BARO_CONVERGED, self.samples.baro_converged());
        self.current
            .set(PreArmFlags::MAG_CONVERGED, self.samples.mag_converged());
    }

    /// Update from fault flags
    pub fn update_from_faults(&mut self, faults: FaultFlags) {
        self.current.set(PreArmFlags::NO_FAULTS, faults.is_empty());
        self.current.set(
            PreArmFlags::CONFIG_VALID,
            !faults.contains(FaultFlags::CONFIG_INVALID),
        );
    }

    /// Update throttle check
    pub fn update_throttle(&mut self, throttle_low: bool) {
        self.current.set(PreArmFlags::THROTTLE_LOW, throttle_low);
    }

    /// Update EKF convergence
    pub fn update_ekf(&mut self, converged: bool) {
        self.current.set(PreArmFlags::EKF_CONVERGED, converged);
    }

    /// Reset for re-arming attempt
    pub fn reset(&mut self) {
        self.current = PreArmFlags::empty();
        self.samples.reset();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_pre_arm_flags_quad_minimum() {
        let required = PreArmFlags::QUAD_MINIMUM;
        assert!(required.contains(PreArmFlags::IMU_HEALTHY));
        assert!(required.contains(PreArmFlags::BARO_HEALTHY));
        assert!(required.contains(PreArmFlags::THROTTLE_LOW));
        assert!(!required.contains(PreArmFlags::GNSS_AVAILABLE)); // Not required for basic quad
    }

    #[test]
    fn test_pre_arm_status_missing() {
        let status = PreArmStatus {
            current: PreArmFlags::IMU_HEALTHY | PreArmFlags::BARO_HEALTHY,
            ..Default::default()
        };

        let missing = status.missing();
        assert!(missing.contains(PreArmFlags::THROTTLE_LOW));
        assert!(missing.contains(PreArmFlags::EKF_CONVERGED));
        assert!(!missing.contains(PreArmFlags::IMU_HEALTHY)); // Already passing
    }

    #[test]
    fn test_sample_counts_convergence() {
        let mut counts = SampleCounts::new();
        assert!(!counts.imu_converged());

        for _ in 0..100 {
            counts.imu = counts.imu.saturating_add(1);
        }
        assert!(counts.imu_converged());
    }

    #[test]
    fn test_pre_arm_satisfied() {
        let mut status =
            PreArmStatus::with_required(PreArmFlags::IMU_HEALTHY | PreArmFlags::THROTTLE_LOW);

        assert!(!status.is_satisfied());

        status.current = PreArmFlags::IMU_HEALTHY | PreArmFlags::THROTTLE_LOW;
        assert!(status.is_satisfied());
    }
}
