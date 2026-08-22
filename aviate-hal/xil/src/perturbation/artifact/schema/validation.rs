//! Semantic checks for strict condition decoding.

use super::{
    ActuatorCondition, CommandLossPolicy, ConditionSet, DelayJitter, HoverThrustExpectation,
    SensorAxis, SensorCondition, SensorNoiseLane, TurbulenceModel,
};
use crate::perturbation::ArtifactError;

const MAX_SENSOR_NOISE_LANES: usize = 12;
const MAX_UPDATE_INTERVAL_SAMPLES: u32 = 100_000;

pub(super) fn condition(value: &ConditionSet) -> Result<(), ArtifactError> {
    if value.schema_version != 4 {
        return invalid("condition schema version must be 4");
    }
    if value.id.trim().is_empty() || value.id.len() > 256 {
        return invalid("condition id is empty or too long");
    }
    validate_wind(value)?;
    validate_timing(value)?;
    validate_sensor(&value.sensor)?;
    validate_actuator(value.actuator)?;
    let hover_scale = value.hover_scale_basis_points();
    if !(8_000..=12_000).contains(&hover_scale) {
        return invalid("hover-force scale is outside 8000 through 12000");
    }
    validate_plant(value)
}

fn validate_wind(value: &ConditionSet) -> Result<(), ArtifactError> {
    if value.wind.gusts.len() > 64 {
        return invalid("wind has more than 64 gust events");
    }
    validate_range(value.wind.steady.speed_mps, 0.0, 100.0, "steady wind speed")?;
    validate_range(
        value.wind.steady.direction_deg,
        0.0,
        360.0,
        "steady wind direction",
    )?;
    let mut maximum_speed = value.wind.steady.speed_mps;
    for gust in &value.wind.gusts {
        validate_range(gust.speed_mps, 0.0, 100.0, "gust speed")?;
        validate_range(gust.direction_deg, 0.0, 360.0, "gust direction")?;
        if gust.rise_ns == 0 && gust.hold_ns == 0 && gust.fall_ns == 0 {
            return invalid("gust duration is zero");
        }
        maximum_speed += gust.speed_mps;
    }
    if let TurbulenceModel::BandLimitedNoise {
        amplitude_mps,
        knot_interval_ns,
    } = value.wind.turbulence
    {
        validate_range(amplitude_mps, 0.0, 20.0, "turbulence amplitude")?;
        if knot_interval_ns == 0 {
            return invalid("turbulence knot interval is zero");
        }
        maximum_speed += amplitude_mps;
    }
    validate_range(maximum_speed, 0.0, 100.0, "maximum wind speed")
}

fn validate_timing(value: &ConditionSet) -> Result<(), ArtifactError> {
    let maximum_jitter = match value.timing.update_jitter {
        DelayJitter::None => 0,
        DelayJitter::SampleAndHold {
            maximum_delay_ns,
            interval_ns,
        } => {
            if maximum_delay_ns == 0 || interval_ns == 0 {
                return invalid("timing jitter duration is zero");
            }
            maximum_delay_ns
        }
    };
    if value
        .timing
        .estimate_delay_ns
        .saturating_add(maximum_jitter)
        > 100_000_000
    {
        invalid("maximum source delay exceeds 100 milliseconds")
    } else {
        Ok(())
    }
}

fn validate_sensor(value: &SensorCondition) -> Result<(), ArtifactError> {
    let SensorCondition::BoundedNoise { lanes } = value else {
        return Ok(());
    };
    if lanes.is_empty() || lanes.len() > MAX_SENSOR_NOISE_LANES {
        return invalid("bounded sensor noise must have 1 through 12 lanes");
    }
    for (index, lane) in lanes.iter().copied().enumerate() {
        validate_sensor_lane(lane)?;
        if lanes[..index]
            .iter()
            .copied()
            .any(|prior| sensor_identity(prior) == sensor_identity(lane))
        {
            return invalid("bounded sensor noise repeats one lane");
        }
    }
    Ok(())
}

fn validate_sensor_lane(value: SensorNoiseLane) -> Result<(), ArtifactError> {
    let (amplitude, maximum, interval) = match value {
        SensorNoiseLane::Accelerometer {
            peak_amplitude_mps2,
            update_interval_samples,
            ..
        } => (peak_amplitude_mps2, 20.0, update_interval_samples),
        SensorNoiseLane::Gyroscope {
            peak_amplitude_rad_s,
            update_interval_samples,
            ..
        } => (peak_amplitude_rad_s, 10.0, update_interval_samples),
        SensorNoiseLane::Magnetometer {
            peak_amplitude_gauss,
            update_interval_samples,
            ..
        } => (peak_amplitude_gauss, 2.0, update_interval_samples),
        SensorNoiseLane::AbsolutePressure {
            peak_amplitude_hpa,
            update_interval_samples,
        }
        | SensorNoiseLane::DifferentialPressure {
            peak_amplitude_hpa,
            update_interval_samples,
        } => (peak_amplitude_hpa, 200.0, update_interval_samples),
        SensorNoiseLane::PressureAltitude {
            peak_amplitude_m,
            update_interval_samples,
        } => (peak_amplitude_m, 2_000.0, update_interval_samples),
    };
    if !amplitude.is_finite() || amplitude <= 0.0 || amplitude > maximum {
        return invalid("sensor amplitude is outside its physical bound");
    }
    if !(1..=MAX_UPDATE_INTERVAL_SAMPLES).contains(&interval) {
        return invalid("sensor update interval is outside 1 through 100000");
    }
    Ok(())
}

fn sensor_identity(value: SensorNoiseLane) -> (u8, Option<SensorAxis>) {
    match value {
        SensorNoiseLane::Accelerometer { axis, .. } => (0, Some(axis)),
        SensorNoiseLane::Gyroscope { axis, .. } => (1, Some(axis)),
        SensorNoiseLane::Magnetometer { axis, .. } => (2, Some(axis)),
        SensorNoiseLane::AbsolutePressure { .. } => (3, None),
        SensorNoiseLane::DifferentialPressure { .. } => (4, None),
        SensorNoiseLane::PressureAltitude { .. } => (5, None),
    }
}

fn validate_actuator(value: ActuatorCondition) -> Result<(), ArtifactError> {
    if !(5_000..=15_000).contains(&value.authority_scale_basis_points) {
        return invalid("actuator authority scale is outside 5000 through 15000");
    }
    let CommandLossPolicy::SeededZeroOrderHold {
        fraction_basis_points,
        decision_interval_samples,
    } = value.command_loss
    else {
        return Ok(());
    };
    let product = u64::from(fraction_basis_points) * u64::from(decision_interval_samples);
    if !(1..=1_000).contains(&fraction_basis_points)
        || !(1..=10_000).contains(&decision_interval_samples)
        || product % 10_000 != 0
    {
        invalid("command hold does not select an exact permitted count")
    } else {
        Ok(())
    }
}

fn validate_plant(value: &ConditionSet) -> Result<(), ArtifactError> {
    validate_range(
        value.plant.payload_mass_delta_kg,
        0.0,
        2_000.0,
        "payload mass delta",
    )?;
    validate_range(
        value.plant.longitudinal_cg_offset_m,
        -2.0,
        2.0,
        "longitudinal center-of-gravity offset",
    )?;
    validate_range(
        value.plant.lateral_cg_offset_m,
        -2.0,
        2.0,
        "lateral center-of-gravity offset",
    )?;
    if let HoverThrustExpectation::ExplicitRatio {
        ratio,
        maximum_error,
    } = value.plant.hover_thrust_expectation
    {
        validate_range(ratio, 0.5, 1.5, "hover thrust ratio")?;
        validate_range(maximum_error, 0.0, 0.1, "hover thrust ratio error")?;
    }
    Ok(())
}

fn validate_range(
    value: f64,
    minimum: f64,
    maximum: f64,
    field: &'static str,
) -> Result<(), ArtifactError> {
    if value.is_finite() && (minimum..=maximum).contains(&value) {
        Ok(())
    } else {
        invalid(field)
    }
}

fn invalid<T>(message: &'static str) -> Result<T, ArtifactError> {
    Err(ArtifactError::Invalid(message))
}
