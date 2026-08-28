//! Machine-readable plant-identification report.

use aviate_config::airframe_preset::{
    ContentDigest, PlantIdentificationArtifact, PlantSampleClock,
};

mod fit;
use fit::fit_point;

use super::excitation::{AXIS_NAMES, PROBE_RAD_S};
use super::stand::Sample;
use super::trace;

const MIN_SAMPLES: usize = 200;
const MIN_BLOCKS: usize = 3;
// The two probes straddle the rotor's spool pole, and the plant model
// is K plus delay only: the upper reading carries the pole's magnitude
// attenuation as well as K, so a real vehicle disagrees with itself by
// the pole's rolloff even in a perfect measurement. The bar admits
// that physics; a channel whose readings differ beyond it is still
// refused as polluted.
const MAX_K_DISAGREEMENT: f32 = 0.4;
// The saturation bar is the artifact validator's own — one authority,
// so the report cannot admit a window the artifact then refuses.
const MAX_WINDOW_SATURATION: f32 = aviate_config::airframe_preset::MAX_SATURATION_FRACTION;
// One authority with the artifact validator, so the report cannot
// admit a point the artifact then refuses.
const MIN_COHERENCE: f32 = aviate_config::airframe_preset::MIN_COHERENCE;
const MAX_DELAY_S: f32 = 0.5;
const MIN_DELAY_UNCERTAINTY_S: f32 = 0.01;

pub(super) struct ReportContext {
    pub(super) simulator_model_digest: String,
    pub(super) run_manifest_digest: String,
    pub(super) hover_force: f32,
}

#[derive(Clone, Copy, Debug)]
struct FitPoint {
    authority_k: f32,
    phase_deg: f32,
    r_squared: f32,
    authority_ci95: f32,
    delay_ci95_s: f32,
    coherence: f32,
    applied_input_max: f32,
    sample_count: u32,
    saturation_fraction: f32,
    response_sign: i8,
}

#[derive(Clone, Copy)]
struct TransferEstimate {
    h_re: f32,
    h_im: f32,
    authority_k: f32,
    phase_deg: f32,
    rate_cos: f32,
    rate_sin: f32,
}

/// Fit all excitation windows and return one validated artifact.
pub(super) fn report(
    samples: &[Sample],
    windows: &[[(usize, usize); 2]; 3],
    context: ReportContext,
) -> Result<(PlantIdentificationArtifact, String), ReportRefusal> {
    // The trace is the experiment's evidence whether the fit accepts it
    // or not: a refused run's trace is exactly what diagnosing the
    // refusal needs, so it is encoded first and travels with either
    // outcome.
    let trace_text = trace::encode(
        samples,
        windows,
        &context.simulator_model_digest,
        &context.run_manifest_digest,
    );
    match fit_artifact(samples, windows, context, &trace_text) {
        Ok(artifact) => Ok((artifact, trace_text)),
        Err(reason) => Err(ReportRefusal { reason, trace_text }),
    }
}

/// Why the fit refused the experiment, with the evidence that shows it.
#[derive(Debug)]
pub(super) struct ReportRefusal {
    pub(super) reason: String,
    pub(super) trace_text: String,
}

fn fit_artifact(
    samples: &[Sample],
    windows: &[[(usize, usize); 2]; 3],
    context: ReportContext,
    trace_text: &str,
) -> Result<PlantIdentificationArtifact, String> {
    let mut points = [[None; 2]; 3];
    for axis in 0..3 {
        for frequency in 0..2 {
            let window = window(samples, windows[axis][frequency], axis)?;
            let point = fit_point(window, axis, PROBE_RAD_S[frequency])?;
            validate_fit_point(window, point, axis, frequency)?;
            points[axis][frequency] = Some(point);
        }
    }
    let trace_digest = ContentDigest::calculate(trace_text.as_bytes());
    let mut artifact = empty_artifact(context, trace_digest, samples, windows)?;
    for (axis, values) in points.into_iter().enumerate() {
        let low = values[0].ok_or_else(|| "missing low-frequency fit".to_owned())?;
        let high = values[1].ok_or_else(|| "missing high-frequency fit".to_owned())?;
        combine_axis(&mut artifact, axis, low, high)?;
    }
    artifact.validate().map_err(|error| error.to_string())?;
    Ok(artifact)
}

fn window(samples: &[Sample], bounds: (usize, usize), axis: usize) -> Result<&[Sample], String> {
    samples
        .get(bounds.0..bounds.1)
        .ok_or_else(|| format!("{} window is outside the sample trace", AXIS_NAMES[axis]))
}

fn validate_fit_point(
    window: &[Sample],
    point: FitPoint,
    axis: usize,
    frequency: usize,
) -> Result<(), String> {
    if point.saturation_fraction > MAX_WINDOW_SATURATION {
        // Name the constraints so the refusal is diagnosable from the
        // log alone: a clipped probe and a jittering bridge need
        // different fixes.
        let mut counts = [0_usize; 5];
        for sample in window {
            for (count, hit) in counts.iter_mut().zip(sample.constraints) {
                if hit {
                    *count += 1;
                }
            }
        }
        let breakdown = super::stand::CONSTRAINT_NAMES
            .iter()
            .zip(counts)
            .filter(|(_, count)| *count > 0)
            .map(|(name, count)| {
                format!(
                    "{name} {:.1}%",
                    count as f32 * 100.0 / window.len().max(1) as f32
                )
            })
            .collect::<Vec<_>>()
            .join(", ");
        return Err(format!(
            "{} @{} rad/s has {:.1}% constrained samples ({breakdown})",
            AXIS_NAMES[axis],
            PROBE_RAD_S[frequency],
            point.saturation_fraction * 100.0
        ));
    }
    if point.coherence < MIN_COHERENCE {
        return Err(format!(
            "{} @{} rad/s coherence is {:.3}",
            AXIS_NAMES[axis], PROBE_RAD_S[frequency], point.coherence
        ));
    }
    Ok(())
}

fn empty_artifact(
    context: ReportContext,
    trace_digest: ContentDigest,
    samples: &[Sample],
    windows: &[[(usize, usize); 2]; 3],
) -> Result<PlantIdentificationArtifact, String> {
    let trace = trace_digest.to_string();
    Ok(PlantIdentificationArtifact {
        schema_version: 1,
        artifact_id: format!("alia250-xplane-{}", &trace[..12]),
        airframe_id: "alia250".to_owned(),
        simulator_model_digest: context.simulator_model_digest,
        run_manifest_digest: context.run_manifest_digest,
        trace_digest: trace,
        sample_clock: PlantSampleClock::SimulatorMicroseconds,
        operating_hover_force: operating_collective(samples, windows, context.hover_force),
        probe_rad_s: PROBE_RAD_S,
        sample_rate_hz: observed_sample_rate(samples, windows)?,
        authority_k: [0.0; 3],
        delay_s: [0.0; 3],
        delay_ci95_s: [0.0; 3],
        r_squared: [0.0; 3],
        authority_ci95: [0.0; 3],
        coherence: [0.0; 3],
        applied_input_max: [0.0; 3],
        sample_count: [0; 3],
        saturation_fraction: [0.0; 3],
        response_sign: [0; 3],
    })
}

fn combine_axis(
    artifact: &mut PlantIdentificationArtifact,
    axis: usize,
    low: FitPoint,
    high: FitPoint,
) -> Result<(), String> {
    let mean_k = (low.authority_k + high.authority_k) * 0.5;
    let disagreement = (low.authority_k - high.authority_k).abs() / mean_k;
    if disagreement > MAX_K_DISAGREEMENT {
        return Err(format!(
            "{} authority differs by {:.1}% between frequencies",
            AXIS_NAMES[axis],
            disagreement * 100.0
        ));
    }
    if low.response_sign != high.response_sign || low.response_sign != 1 {
        return Err(format!(
            "{} response sign is not positive",
            AXIS_NAMES[axis]
        ));
    }
    let (delay_s, delay_ci95_s) = fit_delay(low, high, axis)?;
    artifact.authority_k[axis] = mean_k;
    artifact.delay_s[axis] = delay_s;
    artifact.delay_ci95_s[axis] = delay_ci95_s;
    artifact.r_squared[axis] = low.r_squared.min(high.r_squared);
    artifact.authority_ci95[axis] = low
        .authority_ci95
        .max(high.authority_ci95)
        .max((low.authority_k - high.authority_k).abs() * 0.5);
    artifact.coherence[axis] = low.coherence.min(high.coherence);
    artifact.applied_input_max[axis] = low.applied_input_max.max(high.applied_input_max);
    artifact.sample_count[axis] = low
        .sample_count
        .checked_add(high.sample_count)
        .ok_or_else(|| format!("{} sample count overflowed", AXIS_NAMES[axis]))?;
    artifact.saturation_fraction[axis] = low.saturation_fraction.max(high.saturation_fraction);
    artifact.response_sign[axis] = 1;
    Ok(())
}

fn fit_delay(low: FitPoint, high: FitPoint, axis: usize) -> Result<(f32, f32), String> {
    let mut best: Option<(f32, f32)> = None;
    for low_turn in -2..=1 {
        for high_turn in -2..=1 {
            let low_phase = (low.phase_deg + 360.0 * low_turn as f32).to_radians();
            let high_phase = (high.phase_deg + 360.0 * high_turn as f32).to_radians();
            let low_delay = (-core::f32::consts::FRAC_PI_2 - low_phase) / PROBE_RAD_S[0];
            let high_delay = (-core::f32::consts::FRAC_PI_2 - high_phase) / PROBE_RAD_S[1];
            if !(-low.delay_ci95_s..=MAX_DELAY_S).contains(&low_delay)
                || !(-high.delay_ci95_s..=MAX_DELAY_S).contains(&high_delay)
            {
                continue;
            }
            let low_delay = low_delay.max(0.0);
            let high_delay = high_delay.max(0.0);
            let disagreement = (low_delay - high_delay).abs();
            if best.is_none_or(|current| disagreement < current.0) {
                best = Some((disagreement, low_delay.max(high_delay)));
            }
        }
    }
    let Some((disagreement, delay)) = best else {
        return Err(format!(
            "{} phase is not a delayed integrator",
            AXIS_NAMES[axis]
        ));
    };
    let uncertainty = low.delay_ci95_s.max(high.delay_ci95_s);
    if disagreement > (2.0 * uncertainty).max(0.03) {
        return Err(format!(
            "{} delay differs between probe frequencies",
            AXIS_NAMES[axis]
        ));
    }
    Ok((delay, uncertainty + disagreement * 0.5))
}

fn observed_sample_rate(
    samples: &[Sample],
    windows: &[[(usize, usize); 2]; 3],
) -> Result<f32, String> {
    let mut intervals = 0_u64;
    let mut duration_s = 0.0;
    for axis in windows {
        for bounds in axis {
            let selected = samples
                .get(bounds.0..bounds.1)
                .ok_or_else(|| "sample-rate window is invalid".to_owned())?;
            if selected.len() >= 2 {
                intervals = intervals.wrapping_add((selected.len() - 1) as u64);
                duration_s += elapsed_s(
                    selected[0].timestamp_us,
                    selected[selected.len() - 1].timestamp_us,
                )?;
            }
        }
    }
    if duration_s <= 0.0 {
        return Err("sample-rate duration is zero".to_owned());
    }
    Ok(intervals as f32 / duration_s)
}

fn operating_collective(
    samples: &[Sample],
    windows: &[[(usize, usize); 2]; 3],
    fallback: f32,
) -> f32 {
    let mut sum = 0.0;
    let mut count = 0_u64;
    for axis in windows {
        for bounds in axis {
            if let Some(selected) = samples.get(bounds.0..bounds.1) {
                sum += selected
                    .iter()
                    .map(|sample| sample.collective_force)
                    .sum::<f32>();
                count = count.wrapping_add(selected.len() as u64);
            }
        }
    }
    if count == 0 {
        fallback
    } else {
        sum / count as f32
    }
}

fn elapsed_s(start_us: u64, end_us: u64) -> Result<f32, String> {
    let elapsed = end_us
        .checked_sub(start_us)
        .ok_or_else(|| "simulator sample time regressed".to_owned())?;
    Ok(elapsed as f32 / 1_000_000.0)
}

fn unwrap_near(value: f32, reference: f32) -> f32 {
    let mut unwrapped = value;
    while unwrapped - reference > 180.0 {
        unwrapped -= 360.0;
    }
    while unwrapped - reference < -180.0 {
        unwrapped += 360.0;
    }
    unwrapped
}

#[cfg(test)]
mod tests;
