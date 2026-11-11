use crate::time::Timestamp;
use crate::types::{
    Celsius, Meters, MetersPerSecond, MetersPerSecondSquared, Microtesla, Pascals,
    RadiansPerSecond, Scalar,
};

#[derive(Copy, Clone, Debug, PartialEq, Eq, Default)]
pub enum SensorHealth {
    Good,
    Degraded,
    Failed,
    #[default]
    NotAvailable,
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

/// Where a [`GeomagneticReference`] came from.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GeomagneticSource {
    /// A geomagnetic model evaluated at the vehicle's position and date.
    Model,
    /// A declination configured for the operating area.
    Configured,
}

/// The local magnetic declination: the angle from true north to
/// magnetic north, positive east. The estimator adds it to the
/// magnetic heading it measures so that yaw is referenced to true
/// north.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GeomagneticReference {
    /// Declination, positive east.
    pub declination: crate::types::Radians,
    /// Where the declination came from.
    pub source: GeomagneticSource,
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
    /// The local declination, `None` when neither a model nor a
    /// configured value is available; yaw is then magnetic.
    pub geomagnetic: Option<GeomagneticReference>,
}

/// Air density at sea level in the standard atmosphere \[kg/m³\].
/// Indicated airspeed is defined against this density.
pub const SEA_LEVEL_AIR_DENSITY_KGPM3: Scalar = 1.225;

/// Specific gas constant of dry air \[J/(kg·K)\].
pub const DRY_AIR_GAS_CONSTANT_JPKGK: Scalar = 287.05;

/// Offset from degrees Celsius to kelvin.
pub const CELSIUS_TO_KELVIN_OFFSET: Scalar = 273.15;

/// Air temperature at sea level in the standard atmosphere \[K\]. Stands
/// in for the temperature of a probe that reports none.
pub const STANDARD_TEMPERATURE_K: Scalar = 288.15;

/// Airspeeds and static port of one pitot-static sample.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PitotStatic {
    /// Indicated airspeed \[m/s\]: the dynamic pressure at sea-level density.
    pub indicated_mps: Scalar,
    /// True airspeed \[m/s\]. Equal to the indicated airspeed when the
    /// probe gives no static pressure with a temperature.
    pub true_mps: Scalar,
    /// Static pressure \[Pa\] and air temperature \[K\], when the probe has
    /// a static port. The temperature is the standard one when the probe
    /// reports none.
    pub static_port: Option<(Scalar, Scalar)>,
}

impl AirData {
    /// Airspeeds from the pressures of this sample, or `None` when it
    /// carries no usable dynamic pressure.
    ///
    /// The pressures are the authoritative source: the derived airspeed
    /// fields of the sample are not read.
    pub fn pitot_static(&self) -> Option<PitotStatic> {
        let dynamic_pa = self.dynamic_pressure.map(|pressure| pressure.0)?;
        if !dynamic_pa.is_finite() {
            return None;
        }
        // A negative reading is sensor noise about zero airspeed.
        let indicated = libm::sqrtf(2.0 * dynamic_pa.max(0.0) / SEA_LEVEL_AIR_DENSITY_KGPM3);
        let static_port = self
            .static_pressure
            .map(|pressure| pressure.0)
            .filter(|pressure| pressure.is_finite() && *pressure > 0.0)
            .map(|pressure| {
                let kelvin = self
                    .temperature
                    .map(|temperature| temperature.0 + CELSIUS_TO_KELVIN_OFFSET)
                    .filter(|kelvin| kelvin.is_finite() && *kelvin > 0.0)
                    .unwrap_or(STANDARD_TEMPERATURE_K);
                (pressure, kelvin)
            });
        // Density from the static port, when the probe reports a
        // temperature with it.
        let density = static_port
            .filter(|_| self.temperature.is_some())
            .map_or(SEA_LEVEL_AIR_DENSITY_KGPM3, |(pressure, kelvin)| {
                pressure / (DRY_AIR_GAS_CONSTANT_JPKGK * kelvin)
            });
        Some(PitotStatic {
            indicated_mps: indicated,
            true_mps: indicated * libm::sqrtf(SEA_LEVEL_AIR_DENSITY_KGPM3 / density),
            static_port,
        })
    }
}

impl SensorSet {
    /// Air data of the first airspeed sensor that is valid and healthy.
    pub fn primary_air_data(&self) -> Option<&AirData> {
        self.airspeeds
            .iter()
            .find(|reading| reading.valid && reading.health == SensorHealth::Good)
            .map(|reading| &reading.value.air)
    }
}
