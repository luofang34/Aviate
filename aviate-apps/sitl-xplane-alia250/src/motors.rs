//! Motor-order mapping between this kernel's mixer and the simulated
//! airframe's lift rotors.
//!
//! The mixer numbers its rotors front-right, rear-left, front-left,
//! rear-right. The simulated airframe numbers its lift rotors
//! front-right, front-left, rear-left, rear-right. The two agree on
//! geometry and on which diagonal spins which way, so only the INDEX
//! order differs — and an unmapped index would put the rear-left
//! command on the front-left rotor, inverting pitch and rolling the
//! vehicle over on its first correction.

/// Mixer lane index for each airframe rotor position, in the airframe's
/// own order: front-right, front-left, rear-left, rear-right.
pub const AIRFRAME_FROM_MIXER: [usize; 4] = [0, 2, 1, 3];

/// Reorders mixer lanes into the airframe's rotor order in place.
pub fn to_airframe_order(outputs: &mut [f32; 16], count: u8) {
    if usize::from(count) < AIRFRAME_FROM_MIXER.len() {
        return;
    }
    let mixer = *outputs;
    for (airframe_index, mixer_index) in AIRFRAME_FROM_MIXER.iter().enumerate() {
        outputs[airframe_index] = mixer[*mixer_index];
    }
}

#[cfg(test)]
mod tests {
    use super::{to_airframe_order, AIRFRAME_FROM_MIXER};

    #[test]
    fn the_mapping_is_a_permutation() {
        let mut seen = AIRFRAME_FROM_MIXER;
        seen.sort_unstable();
        assert_eq!(seen, [0, 1, 2, 3], "every rotor is driven exactly once");
    }

    #[test]
    fn rear_left_and_front_left_swap() {
        let mut outputs = [0.0_f32; 16];
        // Distinct values so a wrong lane is visible, not plausible.
        outputs[..4].copy_from_slice(&[0.1, 0.2, 0.3, 0.4]);
        to_airframe_order(&mut outputs, 4);
        assert_eq!(
            &outputs[..4],
            &[0.1, 0.3, 0.2, 0.4],
            "the mixer's rear-left lane must reach the airframe's rear-left rotor"
        );
    }

    #[test]
    fn a_short_command_is_left_alone() {
        let mut outputs = [0.0_f32; 16];
        outputs[..2].copy_from_slice(&[0.7, 0.8]);
        to_airframe_order(&mut outputs, 2);
        assert_eq!(&outputs[..2], &[0.7, 0.8]);
    }
}
