#![allow(clippy::expect_used)]

use super::*;

fn synthetic_trace(authority: [f32; 3]) -> (Vec<Sample>, [[(usize, usize); 2]; 3]) {
    let mut samples = Vec::new();
    let mut windows = [[(0, 0); 2]; 3];
    for axis in 0..3 {
        for frequency in 0..2 {
            let omega = PROBE_RAD_S[frequency];
            let start = samples.len();
            for index in 0..2_200 {
                let t = index as f32 * 0.01;
                let mut u = [0.0; 3];
                let mut gyro = [0.0; 3];
                u[axis] = libm::sinf(omega * t) * 0.2;
                gyro[axis] = -authority[axis] / omega * libm::cosf(omega * t) * 0.2;
                samples.push(Sample {
                    timestamp_us: samples.len() as u64 * 10_000,
                    u,
                    gyro,
                    collective_force: 0.43,
                    saturated: false,
                    constraints: [false; 5],
                });
            }
            windows[axis][frequency] = (start, samples.len());
        }
    }
    (samples, windows)
}

fn context() -> ReportContext {
    ReportContext {
        simulator_model_digest: "a".repeat(64),
        run_manifest_digest: "b".repeat(64),
        hover_force: 0.43,
    }
}

#[test]
fn analytic_integrator_trace_recovers_authority_and_quality() {
    let (samples, windows) = synthetic_trace([5.3, 3.1, 1.0]);
    let (artifact, trace) = report(&samples, &windows, context()).expect("valid fit");
    assert!((artifact.authority_k[0] - 5.3).abs() < 0.03);
    assert!((artifact.authority_k[1] - 3.1).abs() < 0.03);
    assert!((artifact.authority_k[2] - 1.0).abs() < 0.03);
    assert!(artifact.r_squared.iter().all(|value| *value > 0.99));
    assert_eq!(artifact.response_sign, [1, 1, 1]);
    assert_eq!(
        artifact.trace_digest,
        ContentDigest::calculate(trace.as_bytes()).to_string()
    );
}

#[test]
fn frequency_inconsistent_authority_is_rejected() {
    let (mut samples, windows) = synthetic_trace([5.3, 3.1, 1.0]);
    let (start, end) = windows[0][1];
    for sample in &mut samples[start..end] {
        sample.gyro[0] *= 0.5;
    }
    assert!(report(&samples, &windows, context()).is_err());
}

#[test]
fn clipped_evidence_is_rejected() {
    let (mut samples, windows) = synthetic_trace([5.3, 3.1, 1.0]);
    for sample in samples.iter_mut().take(300) {
        sample.saturated = true;
    }
    assert!(report(&samples, &windows, context()).is_err());
}

#[test]
fn one_clipped_frequency_is_rejected_before_aggregation() {
    let (mut samples, windows) = synthetic_trace([5.3, 3.1, 1.0]);
    let (start, end) = windows[0][1];
    let clipped = ((end - start) as f32 * 0.15) as usize;
    for sample in &mut samples[start..start + clipped] {
        sample.saturated = true;
    }
    let refusal = report(&samples, &windows, context()).expect_err("must refuse");
    // The per-window gate must be the refusing authority, not the
    // artifact validator after aggregation.
    assert!(
        refusal.reason.contains("constrained samples"),
        "{}",
        refusal.reason
    );
}

#[test]
fn phase_that_is_not_a_delayed_integrator_is_rejected() {
    let (mut samples, windows) = synthetic_trace([5.3, 3.1, 1.0]);
    for (axis, axis_windows) in windows.iter().enumerate() {
        for (frequency, bounds) in axis_windows.iter().copied().enumerate() {
            let omega = PROBE_RAD_S[frequency];
            let (start, end) = bounds;
            let origin = samples[start].timestamp_us;
            for sample in &mut samples[start..end] {
                let time = (sample.timestamp_us - origin) as f32 / 1_000_000.0;
                sample.gyro[axis] = 0.2 * libm::sinf(omega * time);
            }
        }
    }
    assert!(report(&samples, &windows, context()).is_err());
}
