//! Canonical machine trace for one plant-identification report.

use super::stand::Sample;
use super::PROBE_RAD_S;

pub(super) fn encode(
    samples: &[Sample],
    windows: &[[(usize, usize); 2]; 3],
    simulator_model_digest: &str,
    run_manifest_digest: &str,
) -> String {
    let mut text = format!(
        "schema_version = 1\nsimulator_model_digest = {simulator_model_digest:?}\nrun_manifest_digest = {run_manifest_digest:?}\nsample_clock = \"simulator-microseconds\"\nprobe_rad_s = {PROBE_RAD_S:?}\n"
    );
    text.push_str("windows = [");
    for (index, bounds) in windows.iter().flatten().enumerate() {
        if index != 0 {
            text.push_str(", ");
        }
        text.push_str(&format!("[{}, {}]", bounds.0, bounds.1));
    }
    text.push_str("]\n");
    for sample in samples {
        text.push_str(&format!(
            "\n[[samples]]\ntimestamp_us = {}\nu = {:?}\ngyro = {:?}\ncollective_force = {:?}\nsaturated = {}\n",
            sample.timestamp_us,
            sample.u,
            sample.gyro,
            sample.collective_force,
            sample.saturated,
        ));
    }
    text
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn trace_identity_changes_with_windows_and_samples() {
        let sample = Sample {
            timestamp_us: 10,
            u: [0.1, 0.0, 0.0],
            gyro: [0.2, 0.0, 0.0],
            collective_force: 0.43,
            saturated: false,
            constraints: [false; 5],
        };
        let first = encode(&[sample], &[[(0, 1); 2]; 3], "model", "run");
        let second = encode(&[sample], &[[(0, 0); 2]; 3], "model", "run");
        assert_ne!(first, second);
    }
}
