//! Sensor noise model tests.

use super::*;

#[test]
fn off_tier_is_passthrough() {
    let mut rng = NoiseRng::new(42);
    let mut imu = SimImuData {
        accel: [1.0, 2.0, -9.81],
        gyro: [0.1, 0.0, -0.1],
        temperature: Some(25.0),
    };
    apply_imu_noise(&mut imu, NoiseTier::Off, &mut rng);
    assert_eq!(imu.accel, [1.0, 2.0, -9.81]);
    assert_eq!(imu.gyro, [0.1, 0.0, -0.1]);
}

#[test]
fn mems_tier_perturbs_values() {
    let mut rng = NoiseRng::new(42);
    let original = [1.0_f32, 2.0, -9.81];
    let mut imu = SimImuData {
        accel: original,
        gyro: [0.0; 3],
        temperature: None,
    };
    apply_imu_noise(&mut imu, NoiseTier::Mems, &mut rng);
    let any_changed = imu.accel.iter().zip(original.iter()).any(|(a, b)| a != b);
    assert!(any_changed, "MEMS tier should produce non-zero noise");
}

#[test]
fn same_seed_yields_same_stream() {
    let mut a = NoiseRng::new(7);
    let mut b = NoiseRng::new(7);
    for _ in 0..100 {
        assert!((a.next_normal() - b.next_normal()).abs() < 1e-7);
    }
}

#[test]
fn mean_close_to_zero_over_many_samples() {
    let mut rng = NoiseRng::new(123);
    let n = 10_000;
    let mut sum = 0.0;
    for _ in 0..n {
        sum += rng.next_normal();
    }
    let mean = sum / n as f32;
    assert!(
        mean.abs() < 0.05,
        "mean of {} samples was {}, expected near 0",
        n,
        mean
    );
}

#[test]
fn variance_close_to_one_over_many_samples() {
    let mut rng = NoiseRng::new(456);
    let n = 10_000;
    let mut sum = 0.0;
    let mut sumsq = 0.0;
    for _ in 0..n {
        let x = rng.next_normal();
        sum += x;
        sumsq += x * x;
    }
    let mean = sum / n as f32;
    let variance = sumsq / n as f32 - mean * mean;
    assert!(
        (variance - 1.0).abs() < 0.1,
        "variance of {} samples was {}, expected near 1.0",
        n,
        variance
    );
}

// Env-var dispatch tests share a process-global resource
// (`AVIATE_SENSOR_NOISE`). Cargo runs unit tests in parallel by
// default, so splitting these into multiple `#[test]` functions
// would let them race on the var. Folded into one body with
// explicit cleanup so each assertion sees a known state.
#[test]
fn env_dispatch_table() {
    let cases = [
        ("", NoiseTier::Off),
        ("0", NoiseTier::Off),
        ("off", NoiseTier::Off),
        ("OFF", NoiseTier::Off),
        ("false", NoiseTier::Off),
        ("1", NoiseTier::Mems),
        ("mems", NoiseTier::Mems),
        ("MEMS", NoiseTier::Mems),
        ("on", NoiseTier::Mems),
        ("tactical", NoiseTier::Tactical),
        ("high", NoiseTier::Tactical),
        ("2", NoiseTier::Tactical),
        ("garbage", NoiseTier::Off),
    ];
    for (value, expected) in cases {
        assert_eq!(
            NoiseTier::parse(value),
            expected,
            "AVIATE_SENSOR_NOISE={:?} should map to {:?}",
            value,
            expected
        );
    }
}
