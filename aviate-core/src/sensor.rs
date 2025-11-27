use crate::types::{
    Meters, MetersPerSecond, MetersPerSecondSquared, 
    RadiansPerSecond, Pascals, Celsius, Microtesla
};
use crate::time::Timestamp;

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum SensorHealth { Good, Degraded, Failed, NotAvailable }

impl Default for SensorHealth {
    fn default() -> Self { Self::NotAvailable }
}

#[derive(Copy, Clone, Debug)]
pub struct SensorReading<T> {
    pub value: T,
    pub valid: bool,
    pub source_id: u8,
    pub timestamp: Timestamp,
    pub health: SensorHealth,
}

impl<T: Default> Default for SensorReading<T> {
    fn default() -> Self {
        Self {
            value: T::default(),
            valid: false,
            source_id: 0,
            timestamp: Timestamp::default(),
            health: SensorHealth::default(),
        }
    }
}

pub const MAX_IMU: usize = 3;
pub const MAX_GNSS: usize = 2;
pub const MAX_MAG: usize = 2;
pub const MAX_BARO: usize = 2;
pub const MAX_AIRSPEED: usize = 2;

#[derive(Copy, Clone, Debug, Default)]
pub struct ImuData {
    pub accel: [MetersPerSecondSquared; 3],
    pub gyro: [RadiansPerSecond; 3],
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Default)]
pub enum GnssFix {
    #[default]
    None,
    TwoD,
    ThreeD,
    RtkFloat,
    RtkFixed,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Default)]
pub enum GnssHealth {
    Good,
    Suspect, // propagated for diagnostics only; not fused for control/estimation
    #[default]
    Lost,
}

#[derive(Copy, Clone, Debug, Default)]
pub struct GnssData {
    pub position_ned: [Meters; 3],
    pub velocity_ned: [MetersPerSecond; 3],
    pub fix: GnssFix,
    pub health: GnssHealth,
}

#[derive(Copy, Clone, Debug, Default)]
pub struct MagData {
    pub field_ut: [Microtesla; 3],
}

#[derive(Copy, Clone, Debug, Default)]
pub struct AirData {
    pub static_pressure: Option<Pascals>,
    pub dynamic_pressure: Option<Pascals>,
    pub total_pressure: Option<Pascals>,
    pub temperature: Option<Celsius>,
    pub indicated_airspeed: Option<MetersPerSecond>,
    pub true_airspeed: Option<MetersPerSecond>,
}

#[derive(Copy, Clone, Debug, Default)]
pub struct BaroData {
    pub altitude: Option<Meters>,
    pub air: AirData,
}

#[derive(Copy, Clone, Debug, Default)]
pub struct AirspeedData {
    pub air: AirData,
}

// Stub for geometry state needed by SensorSet
#[derive(Clone, Debug)]
pub struct GeometryState {
    // simplified for now
    pub valid: bool,
}

#[derive(Clone, Debug)]
pub struct SensorSet {
    pub imus: [SensorReading<ImuData>; MAX_IMU],
    pub gnss: [SensorReading<GnssData>; MAX_GNSS],
    pub mags: [SensorReading<MagData>; MAX_MAG],
    pub baros: [SensorReading<BaroData>; MAX_BARO],
    pub airspeeds: [SensorReading<AirspeedData>; MAX_AIRSPEED],
    pub geometry: Option<GeometryState>,
}