//! The correlation fit: per-block transfer estimates and the quality
//! numbers one probe point carries.

use super::super::excitation::AXIS_NAMES;
use super::super::stand::Sample;
use super::{
    elapsed_s, unwrap_near, FitPoint, TransferEstimate, MIN_BLOCKS, MIN_DELAY_UNCERTAINTY_S,
    MIN_SAMPLES,
};

pub(super) fn fit_point(window: &[Sample], axis: usize, omega: f32) -> Result<FitPoint, String> {
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
        saturated = saturated.wrapping_add(u32::from(sample.saturated()));
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
