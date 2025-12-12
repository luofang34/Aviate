//! Sensor caching for EKF initialization (SITL/HITL)
//!
//! Extracted from SITL board implementations (sitl-gazebo, sitl-jmavsim).
//! Used to accumulate initial sensor readings before kernel initialization.

use aviate_core::sensor::{BaroData, GnssData, ImuData, MagData, SensorReading, SensorSet};

/// Cached sensor readings for kernel initialization
///
/// Used in SITL/HITL to collect initial sensor data before starting the
/// EKF. Once we have at least one reading from each required sensor,
/// we can initialize the kernel with a valid `SensorSet`.
pub struct SensorCache {
    pub imu: Option<SensorReading<ImuData>>,
    pub gnss: Option<SensorReading<GnssData>>,
    pub baro: Option<SensorReading<BaroData>>,
    pub mag: Option<SensorReading<MagData>>,
}

impl SensorCache {
    /// Create a new empty sensor cache
    pub fn new() -> Self {
        Self {
            imu: None,
            gnss: None,
            baro: None,
            mag: None,
        }
    }

    /// Convert cached sensor readings to a SensorSet
    ///
    /// This is used to initialize the kernel once we have collected
    /// initial sensor data. Missing sensors will use default values.
    pub fn to_sensor_set(&self) -> SensorSet {
        SensorSet {
            imus: [
                self.imu.unwrap_or_default(),
                SensorReading::default(),
                SensorReading::default(),
            ],
            gnss: [self.gnss.unwrap_or_default(), SensorReading::default()],
            mags: [self.mag.unwrap_or_default(), SensorReading::default()],
            baros: [self.baro.unwrap_or_default(), SensorReading::default()],
            airspeeds: [SensorReading::default(), SensorReading::default()],
            geometry: None,
        }
    }
}

impl Default for SensorCache {
    fn default() -> Self {
        Self::new()
    }
}
