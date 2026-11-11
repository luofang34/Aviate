//! Airspeed for the fixed-wing law: the pitot-static latch and the
//! fallback when no measurement is usable.

use crate::control::law_invariants::STANDARD_GRAVITY_MPS2;
use crate::kernel::config::FixedWingParams;
use crate::sensor::AirData;
use crate::state::{StateEstimate, StateValidFlags};
#[allow(unused_imports)] // FloatExt supplies the no_std math methods
use crate::types::FloatExt;
use crate::types::Scalar;

use super::runtime_state::AirSample;

/// Air density at sea level in the standard atmosphere \[kg/m³\].
/// Indicated airspeed is defined against this density, so it converts
/// a dynamic pressure to an indicated airspeed for every airframe.
pub const SEA_LEVEL_AIR_DENSITY_KGPM3: Scalar = 1.225;

/// Specific gas constant of dry air \[J/(kg·K)\]. Gives the air density
/// at the probe from its static pressure and temperature.
pub const DRY_AIR_GAS_CONSTANT_JPKGK: Scalar = 287.05;

/// Offset from degrees Celsius to kelvin.
pub const CELSIUS_TO_KELVIN_OFFSET: Scalar = 273.15;

/// Age \[s\] at which the law stops flying on the last airspeed sample.
/// A pitot-static sensor supplies a sample at least each 100 ms, so ten
/// missed samples are a failed sensor and not a late one. An aged-out
/// airspeed moves the law to its fallback; a longer hold would keep the
/// stall protection on a number that no longer describes the airflow.
pub const AIRSPEED_STALE_AFTER_S: Scalar = 1.0;

const _: () = assert!(SEA_LEVEL_AIR_DENSITY_KGPM3 > 0.0);
const _: () = assert!(DRY_AIR_GAS_CONSTANT_JPKGK > 0.0);
const _: () = assert!(AIRSPEED_STALE_AFTER_S > 0.0);

/// Air temperature at sea level in the standard atmosphere \[K\]. Stands
/// in for the temperature of a probe that reports none: the pressure
/// height is used only as a difference over minutes, where an error of
/// 30 K in this value changes the height scale by a tenth.
pub const STANDARD_TEMPERATURE_K: Scalar = 288.15;

const _: () = assert!(STANDARD_TEMPERATURE_K > 0.0);

/// Static port of one pitot-static sample.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct StaticAir {
    /// Static pressure \[Pa\].
    pub pressure_pa: Scalar,
    /// Height that the pressure falls by a factor of e over \[m\], from
    /// the temperature of the sample.
    pub scale_height_m: Scalar,
}

/// Last airspeed measurement and its age.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct AirDataLatch {
    /// Filtered indicated airspeed \[m/s\].
    pub indicated_mps: Scalar,
    /// Filtered true airspeed \[m/s\].
    pub true_mps: Scalar,
    /// Filtered rate of the true airspeed \[m/s²\].
    pub true_rate_mps2: Scalar,
    /// Filtered static pressure \[Pa\]; zero when the probe has no static port.
    pub static_pa: Scalar,
    /// Filtered rate of the static pressure \[Pa/s\].
    pub static_rate_paps: Scalar,
    /// Scale height of the last sample \[m\].
    pub scale_height_m: Scalar,
    /// Time since the last sample \[s\].
    pub age_s: Scalar,
    /// Whether a sample has arrived since the last reset.
    pub primed: bool,
    /// Whether the static-pressure filter holds a sample.
    pub static_primed: bool,
}

/// Static port of one sample, when it gives a usable pressure.
fn static_of(air: &AirData) -> Option<StaticAir> {
    let pressure_pa = air
        .static_pressure
        .map(|pressure| pressure.0)
        .filter(|pressure| pressure.is_finite() && *pressure > 0.0)?;
    let kelvin = air
        .temperature
        .map(|temperature| temperature.0 + CELSIUS_TO_KELVIN_OFFSET)
        .filter(|kelvin| kelvin.is_finite() && *kelvin > 0.0)
        .unwrap_or(STANDARD_TEMPERATURE_K);
    Some(StaticAir {
        pressure_pa,
        scale_height_m: DRY_AIR_GAS_CONSTANT_JPKGK * kelvin / STANDARD_GRAVITY_MPS2,
    })
}

/// Indicated and true airspeed from one pitot-static sample, or `None`
/// when the sample carries no usable dynamic pressure.
pub(super) fn airspeeds_of(air: &AirData) -> Option<AirSample> {
    let dynamic_pa = air.dynamic_pressure.map(|pressure| pressure.0)?;
    if !dynamic_pa.is_finite() {
        return None;
    }
    // A negative reading is sensor noise about zero airspeed.
    let indicated = (2.0 * dynamic_pa.max(0.0) / SEA_LEVEL_AIR_DENSITY_KGPM3).sqrt();
    let static_air = static_of(air);
    // Density from the static port, when the probe reports a temperature
    // with it. Without both, true airspeed is taken as indicated airspeed.
    let density = static_air
        .filter(|_| air.temperature.is_some())
        .map_or(SEA_LEVEL_AIR_DENSITY_KGPM3, |port| {
            port.pressure_pa / (STANDARD_GRAVITY_MPS2 * port.scale_height_m)
        });
    Some(AirSample {
        indicated_mps: indicated,
        true_mps: indicated * (SEA_LEVEL_AIR_DENSITY_KGPM3 / density).sqrt(),
        static_air,
    })
}

/// One low-pass step: move `value` toward `sample` and return the move.
fn low_pass(value: &mut Scalar, sample: Scalar, blend: Scalar) -> Scalar {
    let step = blend * (sample - *value);
    *value += step;
    step
}

impl AirDataLatch {
    /// Whether the last sample is recent enough to fly on.
    pub fn is_fresh(&self) -> bool {
        self.primed && self.age_s <= AIRSPEED_STALE_AFTER_S
    }

    /// Height above the level of `reference_pa` \[m\] and climb rate
    /// \[m/s\] from the static port, when the probe has one and is fresh.
    pub(super) fn pressure_height(&self, reference_pa: Scalar) -> Option<(Scalar, Scalar)> {
        (self.is_fresh() && self.static_primed).then(|| {
            (
                self.scale_height_m * libm::logf(reference_pa / self.static_pa),
                -self.scale_height_m * self.static_rate_paps / self.static_pa,
            )
        })
    }

    /// Age the latch by one cycle and take the sample of this cycle.
    ///
    /// The value filter and the rate filter have the same time constant.
    /// The rate of a pressure sample is its noise divided by the sample
    /// interval, so it needs the second stage before a loop can use it.
    pub(super) fn observe(&mut self, sample: Option<AirSample>, dt: Scalar, filter_tau: Scalar) {
        let was_fresh = self.is_fresh();
        // The age stops at twice the limit: a latch that stays silent
        // for hours must not lose resolution in its own counter.
        self.age_s = (self.age_s + dt).min(2.0 * AIRSPEED_STALE_AFTER_S);
        let Some(sample) = sample else {
            return;
        };
        let since_last = self.age_s;
        let filtering = was_fresh && since_last > 0.0;
        let blend = since_last / (filter_tau + since_last);
        if filtering {
            low_pass(&mut self.indicated_mps, sample.indicated_mps, blend);
            let step = low_pass(&mut self.true_mps, sample.true_mps, blend);
            low_pass(&mut self.true_rate_mps2, step / since_last, blend);
        } else {
            // The first sample, or the first after an outage, has no
            // history to filter against.
            self.indicated_mps = sample.indicated_mps;
            self.true_mps = sample.true_mps;
            self.true_rate_mps2 = 0.0;
        }
        match sample.static_air {
            Some(port) if filtering && self.static_primed => {
                let step = low_pass(&mut self.static_pa, port.pressure_pa, blend);
                low_pass(&mut self.static_rate_paps, step / since_last, blend);
                self.scale_height_m = port.scale_height_m;
            }
            Some(port) => {
                self.static_pa = port.pressure_pa;
                self.static_rate_paps = 0.0;
                self.scale_height_m = port.scale_height_m;
                self.static_primed = true;
            }
            None => self.static_primed = false,
        }
        self.age_s = 0.0;
        self.primed = true;
    }
}

/// Where the airspeed of one cycle came from.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum AirspeedSource {
    /// A fresh pitot-static sample.
    Measured,
    /// The estimated velocity over the ground: correct in still air.
    GroundSpeed,
    /// The cruise airspeed of the airframe description.
    #[default]
    Assumed,
}

/// Airspeed that the law uses in one cycle.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) struct Airspeed {
    /// Indicated airspeed \[m/s\].
    pub indicated: Scalar,
    /// True airspeed \[m/s\].
    pub true_speed: Scalar,
    /// Rate of the true airspeed \[m/s²\].
    pub rate: Scalar,
    /// Origin of the values.
    pub source: AirspeedSource,
}

impl Airspeed {
    /// Select the best available airspeed for this cycle.
    pub(super) fn resolve(
        latch: &AirDataLatch,
        state: &StateEstimate,
        params: &FixedWingParams,
    ) -> Self {
        if latch.is_fresh() {
            return Self {
                indicated: latch.indicated_mps,
                true_speed: latch.true_mps,
                rate: latch.true_rate_mps2,
                source: AirspeedSource::Measured,
            };
        }
        if state.valid_flags.contains(StateValidFlags::VELOCITY) {
            let [north, east, down] = state.velocity_ned;
            let speed = (north.0 * north.0 + east.0 * east.0 + down.0 * down.0).sqrt();
            return Self {
                indicated: speed,
                true_speed: speed,
                rate: 0.0,
                source: AirspeedSource::GroundSpeed,
            };
        }
        Self {
            indicated: params.cruise_airspeed.0,
            true_speed: params.cruise_airspeed.0,
            rate: 0.0,
            source: AirspeedSource::Assumed,
        }
    }

    /// Whether the value is a measurement of the airflow.
    pub(super) fn is_measured(&self) -> bool {
        self.source == AirspeedSource::Measured
    }

    /// True airspeed for use as a divisor: never below the stall speed,
    /// so a slow or grounded vehicle cannot produce an unbounded rate.
    pub(super) fn divisor(&self, params: &FixedWingParams) -> Scalar {
        self.true_speed.max(params.stall_airspeed.0)
    }
}
