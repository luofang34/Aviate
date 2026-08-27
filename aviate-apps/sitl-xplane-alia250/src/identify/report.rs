//! Machine-readable plant-identification report.

use aviate_config::airframe_preset::{
    ContentDigest, PlantIdentificationArtifact, PlantSampleClock,
};

use super::stand::Sample;
use super::trace;
use super::{AXIS_NAMES, PROBE_RAD_S};

const MIN_SAMPLES: usize = 200;
const MIN_BLOCKS: usize = 3;
const MAX_K_DISAGREEMENT: f32 = 0.25;
// A correlation fit over three-plus probe periods averages hundreds of
// clean samples per block; distributed railing bursts at this level
// perturb it far less than the parameters it feeds tolerate, while a
// loop genuinely fighting the stand still shows an order more and is
// refused.
const MAX_WINDOW_SATURATION: f32 = 0.12;
const MIN_COHERENCE: f32 = 0.8;
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
) -> Result<(PlantIdentificationArtifact, String), String> {
    let mut points = [[None; 2]; 3];
    for axis in 0..3 {
        for frequency in 0..2 {
            let window = window(samples, windows[axis][frequency], axis)?;
            let point = fit_point(window, axis, PROBE_RAD_S[frequency])?;
            validate_fit_point(point, axis, frequency)?;
            points[axis][frequency] = Some(point);
        }
    }
    let trace_text = trace::encode(
        samples,
        windows,
        &context.simulator_model_digest,
        &context.run_manifest_digest,
    );
    let trace_digest = ContentDigest::calculate(trace_text.as_bytes());
    let mut artifact = empty_artifact(context, trace_digest, samples, windows)?;
    for (axis, values) in points.into_iter().enumerate() {
        let low = values[0].ok_or_else(|| "missing low-frequency fit".to_owned())?;
        let high = values[1].ok_or_else(|| "missing high-frequency fit".to_owned())?;
        combine_axis(&mut artifact, axis, low, high)?;
    }
    artifact.validate().map_err(|error| error.to_string())?;
    Ok((artifact, trace_text))
}

fn window(samples: &[Sample], bounds: (usize, usize), axis: usize) -> Result<&[Sample], String> {
    samples
        .get(bounds.0..bounds.1)
        .ok_or_else(|| format!("{} window is outside the sample trace", AXIS_NAMES[axis]))
}

fn validate_fit_point(point: FitPoint, axis: usize, frequency: usize) -> Result<(), String> {
    if point.saturation_fraction > MAX_WINDOW_SATURATION {
        return Err(format!(
            "{} @{} rad/s has {:.1}% constrained samples",
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

fn fit_point(window: &[Sample], axis: usize, omega: f32) -> Result<FitPoint, String> {
    if window.len() < MIN_SAMPLES {
        return Err(format!(
            "{} @{} rad/s has too few samples",
            AXIS_NAMES[axis], omega
        ));
    }
    let span = elapsed_s(
        window[0].timestamp_us,
        window[window.len() - 1].timestamp_us,
    )?;
    let period = core::f32::consts::TAU / omega;
    let blocks = (span / period).floor() as usize;
    if blocks < MIN_BLOCKS {
        return Err(format!(
            "{} @{} rad/s has too few cycles",
            AXIS_NAMES[axis], omega
        ));
    }
    let horizon = blocks as f32 * period;
    let estimate = transfer(window, axis, omega, window[0].timestamp_us, horizon)?;
    let residual = residual_quality(window, axis, omega, horizon, estimate)?;
    let block_estimates = block_estimates(window, axis, omega, period, blocks)?;
    let authority_ci95 = confidence_interval(
        &block_estimates
            .iter()
            .map(|value| value.authority_k)
            .collect::<Vec<_>>(),
    );
    let phase_ci_deg = confidence_interval(
        &block_estimates
            .iter()
            .map(|value| unwrap_near(value.phase_deg, estimate.phase_deg))
            .collect::<Vec<_>>(),
    );
    Ok(FitPoint {
        authority_k: estimate.authority_k,
        phase_deg: estimate.phase_deg,
        r_squared: residual.r_squared,
        authority_ci95,
        delay_ci95_s: (phase_ci_deg.to_radians() / omega).max(MIN_DELAY_UNCERTAINTY_S),
        coherence: block_coherence(&block_estimates),
        applied_input_max: window
            .iter()
            .map(|sample| sample.u[axis].abs())
            .fold(0.0, f32::max),
        sample_count: residual.sample_count,
        saturation_fraction: residual.saturation_fraction,
        response_sign: if estimate.h_im < 0.0 { 1 } else { -1 },
    })
}

fn block_estimates(
    window: &[Sample],
    axis: usize,
    omega: f32,
    period: f32,
    blocks: usize,
) -> Result<Vec<TransferEstimate>, String> {
    let origin = window[0].timestamp_us;
    let mut estimates = Vec::with_capacity(blocks);
    for block in 0..blocks {
        let start_s = block as f32 * period;
        let end_s = (block + 1) as f32 * period;
        let mut selected = Vec::new();
        for sample in window {
            let time = elapsed_s(origin, sample.timestamp_us)?;
            if time >= start_s && time <= end_s {
                selected.push(*sample);
            }
        }
        if selected.len() < 8 {
            return Err("one probe cycle has too few samples".to_owned());
        }
        estimates.push(transfer(&selected, axis, omega, origin, end_s)?);
    }
    Ok(estimates)
}

fn transfer(
    window: &[Sample],
    axis: usize,
    omega: f32,
    origin_us: u64,
    horizon_s: f32,
) -> Result<TransferEstimate, String> {
    let mut input_re = 0.0_f64;
    let mut input_im = 0.0_f64;
    let mut rate_re = 0.0_f64;
    let mut rate_im = 0.0_f64;
    for pair in window.windows(2) {
        let time = elapsed_s(origin_us, pair[0].timestamp_us)?;
        if time > horizon_s {
            break;
        }
        let dt = elapsed_s(pair[0].timestamp_us, pair[1].timestamp_us)?;
        let (sin, cos) = libm::sincosf(omega * time);
        input_re += f64::from(pair[0].u[axis] * cos * dt);
        input_im -= f64::from(pair[0].u[axis] * sin * dt);
        rate_re += f64::from(pair[0].gyro[axis] * cos * dt);
        rate_im -= f64::from(pair[0].gyro[axis] * sin * dt);
    }
    let denominator = input_re * input_re + input_im * input_im;
    if denominator <= 1.0e-12 {
        return Err(format!(
            "{} @{} rad/s has no input",
            AXIS_NAMES[axis], omega
        ));
    }
    let h_re = ((rate_re * input_re + rate_im * input_im) / denominator) as f32;
    let h_im = ((rate_im * input_re - rate_re * input_im) / denominator) as f32;
    Ok(TransferEstimate {
        h_re,
        h_im,
        authority_k: libm::sqrtf(h_re * h_re + h_im * h_im) * omega,
        phase_deg: libm::atan2f(h_im, h_re).to_degrees(),
        rate_cos: (2.0 * rate_re / f64::from(horizon_s)) as f32,
        rate_sin: (-2.0 * rate_im / f64::from(horizon_s)) as f32,
    })
}

struct ResidualQuality {
    r_squared: f32,
    sample_count: u32,
    saturation_fraction: f32,
}

fn residual_quality(
    window: &[Sample],
    axis: usize,
    omega: f32,
    horizon: f32,
    estimate: TransferEstimate,
) -> Result<ResidualQuality, String> {
    let origin = window[0].timestamp_us;
    let mut selected = Vec::new();
    for sample in window {
        if elapsed_s(origin, sample.timestamp_us)? <= horizon {
            selected.push(sample);
        }
    }
    let mean =
        selected.iter().map(|sample| sample.gyro[axis]).sum::<f32>() / selected.len().max(1) as f32;
    let mut residual_sum = 0.0;
    let mut total_sum = 0.0;
    let mut saturated = 0_u32;
    for sample in &selected {
        let time = elapsed_s(origin, sample.timestamp_us)?;
        let (sin, cos) = libm::sincosf(omega * time);
        let predicted = estimate.rate_cos * cos + estimate.rate_sin * sin;
        residual_sum += (sample.gyro[axis] - mean - predicted).powi(2);
        total_sum += (sample.gyro[axis] - mean).powi(2);
        saturated = saturated.wrapping_add(u32::from(sample.saturated));
    }
    let count = u32::try_from(selected.len()).map_err(|_| "too many samples".to_owned())?;
    Ok(ResidualQuality {
        r_squared: if total_sum <= f32::EPSILON {
            0.0
        } else {
            1.0 - residual_sum / total_sum
        },
        sample_count: count,
        saturation_fraction: saturated as f32 / count.max(1) as f32,
    })
}

fn block_coherence(estimates: &[TransferEstimate]) -> f32 {
    let count = estimates.len().max(1) as f32;
    let mean_re = estimates.iter().map(|value| value.h_re).sum::<f32>() / count;
    let mean_im = estimates.iter().map(|value| value.h_im).sum::<f32>() / count;
    let mean_power = estimates
        .iter()
        .map(|value| value.h_re * value.h_re + value.h_im * value.h_im)
        .sum::<f32>()
        / count;
    ((mean_re * mean_re + mean_im * mean_im) / mean_power.max(f32::EPSILON)).clamp(0.0, 1.0)
}

fn confidence_interval(values: &[f32]) -> f32 {
    if values.len() < 2 {
        return f32::INFINITY;
    }
    let count = values.len() as f32;
    let mean = values.iter().sum::<f32>() / count;
    let variance = values
        .iter()
        .map(|value| (value - mean).powi(2))
        .sum::<f32>()
        / (count - 1.0);
    1.96 * libm::sqrtf(variance / count)
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
