//! The plant-identity report: correlates each excitation window
//! against its own probe tone and prints the measured transfer
//! ratio per axis and frequency.

use super::stand::Sample;
use super::{AXIS_NAMES, PROBE_RAD_S};

/// Fits the plant per axis by correlation at each probe frequency and
/// prints the plant identity.
///
/// The product per (axis, frequency) is the complex transfer ratio
/// H(jΩ) from normalized axis torque to body rate. |H|·Ω is the
/// effective angular authority there; phase beyond −90° is
/// actuator/spool lag the design must respect. Two frequencies
/// cross-check each other: a channel whose K disagrees between them is
/// polluted, not measured. Correlation at a known frequency stays
/// valid inside a closed loop, because every signal at Ω descends from
/// the injected probe.
pub(super) fn report(samples: &[Sample], windows: &[[(usize, usize); 2]; 3]) {
    println!("=== plant identity (measured) ===");
    for axis in 0..3 {
        for freq in 0..2 {
            let omega = PROBE_RAD_S[freq];
            let (start, end) = windows[axis][freq];
            if end.saturating_sub(start) < 200 {
                println!(
                    "{} @{omega} rad/s: too few samples ({})",
                    AXIS_NAMES[axis],
                    end - start
                );
                continue;
            }
            let window = &samples[start..end];
            let t0 = window[0].at;
            // Whole cycles only, so spectral leakage cannot bias the
            // correlation.
            let period = 2.0 * core::f32::consts::PI / omega;
            let span = window[window.len() - 1].at.duration_since(t0).as_secs_f32();
            let cycles = (span / period).floor();
            if cycles < 2.0 {
                println!("{} @{omega} rad/s: fewer than two cycles", AXIS_NAMES[axis]);
                continue;
            }
            let horizon = cycles * period;

            let (mut u_re, mut u_im) = (0.0_f64, 0.0_f64);
            let (mut w_re, mut w_im) = (0.0_f64, 0.0_f64);
            for pair in window.windows(2) {
                let t = pair[0].at.duration_since(t0).as_secs_f32();
                if t > horizon {
                    break;
                }
                let dt = pair[1].at.duration_since(pair[0].at).as_secs_f32();
                let (sin, cos) = (omega * t).sin_cos();
                u_re += f64::from(pair[0].u[axis] * cos * dt);
                u_im -= f64::from(pair[0].u[axis] * sin * dt);
                w_re += f64::from(pair[0].gyro[axis] * cos * dt);
                w_im -= f64::from(pair[0].gyro[axis] * sin * dt);
            }
            let u_mag = (u_re * u_re + u_im * u_im).sqrt();
            if u_mag < 1e-6 {
                println!("{} @{omega} rad/s: no torque content", AXIS_NAMES[axis]);
                continue;
            }
            let h_re = (w_re * u_re + w_im * u_im) / (u_mag * u_mag);
            let h_im = (w_im * u_re - w_re * u_im) / (u_mag * u_mag);
            let h_mag = (h_re * h_re + h_im * h_im).sqrt() as f32;
            let h_phase_deg = (h_im.atan2(h_re) as f32).to_degrees();
            println!(
                "{} @{omega} rad/s: |H| = {:.2}, phase = {:6.1} deg, K_eff = {:5.1} rad/s^2/torque  ({} cycles, u_amp {:.3})",
                AXIS_NAMES[axis],
                h_mag,
                h_phase_deg,
                h_mag * omega,
                cycles as u32,
                (2.0 * u_mag / f64::from(horizon)) as f32,
            );
        }
    }
    println!("=================================");
}

#[allow(dead_code)]
fn mean_dt(window: &[Sample]) -> f32 {
    if window.len() < 2 {
        return 0.01;
    }
    let span = window[window.len() - 1]
        .at
        .duration_since(window[0].at)
        .as_secs_f32();
    span / (window.len() - 1) as f32
}
