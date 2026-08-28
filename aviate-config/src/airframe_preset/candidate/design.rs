//! Bounded stage application and dynamic candidate checks.

use crate::xplane_model::XPlaneSimulatorModel;

use super::{
    AirframePreset, CalibrationStage, CandidateError, CandidateLayer, GainOverrides,
    InnerLoopDesign, PlantIdentificationArtifact,
};

const MAX_GAIN_STEP: f32 = 0.25;
const MAX_HOVER_STEP: f32 = 0.15;
const ZERO_BASE_STEP: f32 = 0.1;
const MIN_LOOP_SEPARATION: f32 = 5.0;
const MAX_LOOP_SEPARATION: f32 = 10.0;
const MAX_DELAY_PRODUCT: f32 = 0.35;
const WIRE_COLLECTIVE_HEADROOM: f32 = 0.05;

pub(super) fn apply_validated_layer(
    preset: &mut AirframePreset,
    layer: CandidateLayer,
    plant: &PlantIdentificationArtifact,
    simulator_model: &XPlaneSimulatorModel,
) -> Result<(), CandidateError> {
    let champion = preset.clone();
    validate_stage_fields(layer)?;
    if let Some(value) = layer.hover_thrust_seed {
        preset.hover_thrust_seed = value;
    }
    apply_overrides(&mut preset.gains, layer.gains);
    if let Some(design) = layer.inner_loop {
        apply_inner_loop_design(preset, design, plant)?;
    }
    let declared_rate = f32::from(simulator_model.sample_rate_hz());
    // The same sanity band the runtime handshake enforces: the
    // simulator's display-synced frame clock steps across a wide range
    // with instance, load, and warm-up state, and the artifact records
    // the rate its run actually delivered. The declaration is the
    // band's center, not a promise of one number.
    if relative_delta(declared_rate, plant.sample_rate_hz) > 0.25 {
        return Err(CandidateError::InvalidRelation(
            "plant sample rate must match the simulator model",
        ));
    }
    validate_candidate_bounds(preset)?;
    validate_relative_steps(preset, &champion)?;
    validate_hover(preset, &champion, layer, plant, simulator_model)?;
    validate_envelope_relations(preset)?;
    validate_inner_loop(preset, layer.inner_loop, plant)?;
    preset.validate().map_err(CandidateError::ResolvedPreset)
}

fn validate_stage_fields(layer: CandidateLayer) -> Result<(), CandidateError> {
    let gains = layer.gains;
    let valid = match layer.stage {
        CalibrationStage::InnerLoop => {
            layer.inner_loop.is_some()
                && layer.hover_thrust_seed.is_none()
                && gains == GainOverrides::default()
        }
        CalibrationStage::Hover => {
            layer.inner_loop.is_none()
                && layer.hover_thrust_seed.is_some()
                && gains == GainOverrides::default()
        }
        CalibrationStage::RateIntegralDerivative => {
            layer.inner_loop.is_none() && layer.hover_thrust_seed.is_none() && only_rate_id(gains)
        }
        CalibrationStage::OuterLoop => {
            layer.inner_loop.is_none()
                && layer.hover_thrust_seed.is_none()
                && only_outer_loop(gains)
        }
        CalibrationStage::CommandEnvelope => {
            layer.inner_loop.is_none()
                && layer.hover_thrust_seed.is_none()
                && only_command_envelope(gains)
        }
    };
    if !valid {
        return Err(CandidateError::InvalidRelation(
            "candidate fields must belong to one calibration stage",
        ));
    }
    Ok(())
}

fn only_rate_id(gains: GainOverrides) -> bool {
    (gains.rate_i.is_some() || gains.rate_d.is_some() || gains.rate_d_lpf_alpha.is_some())
        && gains.pos_p.is_none()
        && gains.pos_accel_limits.is_none()
        && gains.pos_vel_caps.is_none()
        && gains.vel_p.is_none()
        && gains.vel_i.is_none()
        && gains.vel_d.is_none()
        && gains.vel_max_roll_pitch.is_none()
        && gains.vel_max_yaw_step.is_none()
        && gains.vel_accel_ff.is_none()
        && gains.att_max_rate_cmd.is_none()
}

fn only_outer_loop(gains: GainOverrides) -> bool {
    (gains.pos_p.is_some()
        || gains.pos_accel_limits.is_some()
        || gains.pos_vel_caps.is_some()
        || gains.vel_p.is_some()
        || gains.vel_i.is_some()
        || gains.vel_d.is_some())
        && gains.vel_max_roll_pitch.is_none()
        && gains.vel_max_yaw_step.is_none()
        && gains.vel_accel_ff.is_none()
        && gains.att_max_rate_cmd.is_none()
        && gains.rate_i.is_none()
        && gains.rate_d.is_none()
        && gains.rate_d_lpf_alpha.is_none()
}

fn only_command_envelope(gains: GainOverrides) -> bool {
    (gains.vel_max_roll_pitch.is_some()
        || gains.vel_max_yaw_step.is_some()
        || gains.vel_accel_ff.is_some()
        || gains.att_max_rate_cmd.is_some())
        && gains.pos_p.is_none()
        && gains.pos_accel_limits.is_none()
        && gains.pos_vel_caps.is_none()
        && gains.vel_p.is_none()
        && gains.vel_i.is_none()
        && gains.vel_d.is_none()
        && gains.rate_i.is_none()
        && gains.rate_d.is_none()
        && gains.rate_d_lpf_alpha.is_none()
}

fn apply_overrides(target: &mut super::super::GainsPreset, source: GainOverrides) {
    assign(&mut target.pos_p, source.pos_p);
    assign(&mut target.pos_accel_limits, source.pos_accel_limits);
    assign(&mut target.pos_vel_caps, source.pos_vel_caps);
    assign(&mut target.vel_p, source.vel_p);
    assign(&mut target.vel_i, source.vel_i);
    assign(&mut target.vel_d, source.vel_d);
    assign(&mut target.vel_max_roll_pitch, source.vel_max_roll_pitch);
    assign(&mut target.vel_max_yaw_step, source.vel_max_yaw_step);
    assign(&mut target.vel_accel_ff, source.vel_accel_ff);
    assign(&mut target.att_max_rate_cmd, source.att_max_rate_cmd);
    assign(&mut target.rate_i, source.rate_i);
    assign(&mut target.rate_d, source.rate_d);
    assign(&mut target.rate_d_lpf_alpha, source.rate_d_lpf_alpha);
}

fn assign<T: Copy>(target: &mut T, source: Option<T>) {
    if let Some(value) = source {
        *target = value;
    }
}

fn apply_inner_loop_design(
    preset: &mut AirframePreset,
    design: InnerLoopDesign,
    plant: &PlantIdentificationArtifact,
) -> Result<(), CandidateError> {
    for axis in 0..3 {
        let frequency = design.natural_frequency_rad_s[axis];
        let separation = design.loop_separation[axis];
        bounded("inner_loop.natural_frequency_rad_s", frequency, 0.2, 4.0)?;
        bounded(
            "inner_loop.loop_separation",
            separation,
            MIN_LOOP_SEPARATION,
            MAX_LOOP_SEPARATION,
        )?;
        let separation_root = libm::sqrtf(separation);
        preset.gains.att_p[axis] = frequency / separation_root;
        preset.gains.rate_p[axis] = frequency * separation_root / plant.authority_k[axis];
    }
    Ok(())
}

fn validate_hover(
    preset: &AirframePreset,
    champion: &AirframePreset,
    layer: CandidateLayer,
    plant: &PlantIdentificationArtifact,
    model: &XPlaneSimulatorModel,
) -> Result<(), CandidateError> {
    let hover = preset.hover_thrust_force_seed();
    let ceiling = model.wire().mean_ceiling - WIRE_COLLECTIVE_HEADROOM;
    if hover >= ceiling {
        return Err(CandidateError::InvalidRelation(
            "hover force must keep wire collective headroom",
        ));
    }
    if relative_delta(plant.operating_hover_force, hover) > MAX_HOVER_STEP {
        return Err(CandidateError::InvalidRelation(
            "hover force must stay inside the identified operating envelope",
        ));
    }
    if layer.stage == CalibrationStage::Hover
        && relative_delta(champion.hover_thrust_force_seed(), hover) > MAX_HOVER_STEP
    {
        return Err(CandidateError::InvalidRelation(
            "hover candidate step must be adjacent to the champion",
        ));
    }
    Ok(())
}

fn validate_inner_loop(
    preset: &AirframePreset,
    design: Option<InnerLoopDesign>,
    plant: &PlantIdentificationArtifact,
) -> Result<(), CandidateError> {
    for axis in 0..3 {
        if plant.response_sign[axis] != 1 {
            return Err(CandidateError::InvalidRelation(
                "plant response signs must match the compiled mixer",
            ));
        }
        let k_low = plant.authority_k[axis] - plant.authority_ci95[axis];
        let k_high = plant.authority_k[axis] + plant.authority_ci95[axis];
        if k_low <= 0.0 {
            return Err(CandidateError::InvalidRelation(
                "plant authority confidence interval must be positive",
            ));
        }
        let att_p = preset.gains.att_p[axis];
        let rate_p = preset.gains.rate_p[axis];
        let separation_low = k_low * rate_p / att_p;
        let separation_high = k_high * rate_p / att_p;
        if separation_low < MIN_LOOP_SEPARATION || separation_high > MAX_LOOP_SEPARATION {
            return Err(CandidateError::InvalidRelation(
                "loop separation must hold across the authority interval",
            ));
        }
        let frequency_high = libm::sqrtf(att_p * k_high * rate_p);
        let delay_high = plant.delay_s[axis] + plant.delay_ci95_s[axis];
        if frequency_high * delay_high > MAX_DELAY_PRODUCT {
            return Err(CandidateError::InvalidRelation(
                "bandwidth must hold across the delay interval",
            ));
        }
        validate_rate_id(preset, axis, frequency_high)?;
        if let Some(target) = design {
            validate_nominal_design(preset, plant, target, axis)?;
        }
    }
    Ok(())
}

fn validate_rate_id(
    preset: &AirframePreset,
    axis: usize,
    frequency: f32,
) -> Result<(), CandidateError> {
    let rate_p = preset.gains.rate_p[axis];
    if preset.gains.rate_i[axis] > rate_p * frequency * 0.25
        || preset.gains.rate_d[axis] * frequency > rate_p * 0.25
    {
        return Err(CandidateError::InvalidRelation(
            "rate I and D terms must stay below the inner-loop bandwidth",
        ));
    }
    Ok(())
}

fn validate_nominal_design(
    preset: &AirframePreset,
    plant: &PlantIdentificationArtifact,
    design: InnerLoopDesign,
    axis: usize,
) -> Result<(), CandidateError> {
    let separation = plant.authority_k[axis] * preset.gains.rate_p[axis] / preset.gains.att_p[axis];
    let frequency =
        libm::sqrtf(preset.gains.att_p[axis] * plant.authority_k[axis] * preset.gains.rate_p[axis]);
    if relative_delta(separation, design.loop_separation[axis]) > 1.0e-4
        || relative_delta(frequency, design.natural_frequency_rad_s[axis]) > 1.0e-4
    {
        return Err(CandidateError::InvalidRelation(
            "derived inner-loop values must match the design",
        ));
    }
    Ok(())
}

fn validate_envelope_relations(preset: &AirframePreset) -> Result<(), CandidateError> {
    let rate_limit = preset
        .limits
        .max_roll_rate
        .min(preset.limits.max_pitch_rate)
        .min(preset.limits.max_yaw_rate);
    if preset.gains.att_max_rate_cmd > rate_limit {
        return Err(CandidateError::InvalidRelation(
            "att_max_rate_cmd <= airframe rate limits",
        ));
    }
    if preset.gains.vel_max_roll_pitch > preset.limits.max_roll.min(preset.limits.max_pitch) {
        return Err(CandidateError::InvalidRelation(
            "vel_max_roll_pitch <= airframe tilt limits",
        ));
    }
    let vertical_limit = preset
        .limits
        .max_climb_rate
        .min(preset.limits.max_descent_rate);
    if preset.gains.pos_vel_caps[0] > preset.limits.max_horizontal_speed
        || preset.gains.pos_vel_caps[1] > preset.limits.max_horizontal_speed
        || preset.gains.pos_vel_caps[2] > vertical_limit
    {
        return Err(CandidateError::InvalidRelation(
            "position velocity caps <= airframe speed limits",
        ));
    }
    Ok(())
}

fn validate_relative_steps(
    candidate: &AirframePreset,
    champion: &AirframePreset,
) -> Result<(), CandidateError> {
    let new = candidate.gains;
    let old = champion.gains;
    for (next, base) in [
        (new.pos_p, old.pos_p),
        (new.pos_accel_limits, old.pos_accel_limits),
        (new.pos_vel_caps, old.pos_vel_caps),
        (new.vel_p, old.vel_p),
        (new.vel_i, old.vel_i),
        (new.vel_d, old.vel_d),
        (new.att_p, old.att_p),
        (new.rate_p, old.rate_p),
        (new.rate_i, old.rate_i),
        (new.rate_d, old.rate_d),
    ] {
        for axis in 0..3 {
            adjacent(base[axis], next[axis])?;
        }
    }
    for (next, base) in [
        (new.vel_max_roll_pitch, old.vel_max_roll_pitch),
        (new.vel_max_yaw_step, old.vel_max_yaw_step),
        (new.vel_accel_ff, old.vel_accel_ff),
        (new.att_max_rate_cmd, old.att_max_rate_cmd),
        (new.rate_d_lpf_alpha, old.rate_d_lpf_alpha),
    ] {
        adjacent(base, next)?;
    }
    Ok(())
}

fn adjacent(base: f32, next: f32) -> Result<(), CandidateError> {
    let permitted = (base.abs() * MAX_GAIN_STEP).max(ZERO_BASE_STEP);
    if (next - base).abs() > permitted {
        return Err(CandidateError::InvalidRelation(
            "candidate gain step must be adjacent to the champion",
        ));
    }
    Ok(())
}

fn validate_candidate_bounds(preset: &AirframePreset) -> Result<(), CandidateError> {
    bounded("hover_thrust_seed", preset.hover_thrust_seed, 0.1, 0.9)?;
    triples("gains.pos_p", preset.gains.pos_p, 0.0, 10.0)?;
    triples(
        "gains.pos_accel_limits",
        preset.gains.pos_accel_limits,
        0.05,
        20.0,
    )?;
    triples("gains.pos_vel_caps", preset.gains.pos_vel_caps, 0.05, 30.0)?;
    triples("gains.vel_p", preset.gains.vel_p, 0.0, 20.0)?;
    triples("gains.vel_i", preset.gains.vel_i, 0.0, 20.0)?;
    triples("gains.vel_d", preset.gains.vel_d, 0.0, 20.0)?;
    bounded(
        "gains.vel_max_roll_pitch",
        preset.gains.vel_max_roll_pitch,
        0.05,
        1.2,
    )?;
    bounded(
        "gains.vel_max_yaw_step",
        preset.gains.vel_max_yaw_step,
        0.0,
        3.2,
    )?;
    bounded("gains.vel_accel_ff", preset.gains.vel_accel_ff, 0.0, 1.0)?;
    triples("gains.att_p", preset.gains.att_p, 0.01, 20.0)?;
    bounded(
        "gains.att_max_rate_cmd",
        preset.gains.att_max_rate_cmd,
        0.1,
        10.0,
    )?;
    triples("gains.rate_p", preset.gains.rate_p, 0.01, 20.0)?;
    triples("gains.rate_i", preset.gains.rate_i, 0.0, 20.0)?;
    triples("gains.rate_d", preset.gains.rate_d, 0.0, 20.0)?;
    bounded(
        "gains.rate_d_lpf_alpha",
        preset.gains.rate_d_lpf_alpha,
        0.05,
        0.95,
    )
}

fn triples(
    field: &'static str,
    values: [f32; 3],
    lower: f32,
    upper: f32,
) -> Result<(), CandidateError> {
    for value in values {
        bounded(field, value, lower, upper)?;
    }
    Ok(())
}

fn bounded(field: &'static str, value: f32, lower: f32, upper: f32) -> Result<(), CandidateError> {
    if !value.is_finite() || !(lower..=upper).contains(&value) {
        return Err(CandidateError::FieldOutOfRange(field));
    }
    Ok(())
}

fn relative_delta(base: f32, value: f32) -> f32 {
    (value - base).abs() / base.abs().max(f32::EPSILON)
}
