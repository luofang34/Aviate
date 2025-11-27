#![forbid(unsafe_code)]

#[cfg(test)]
mod tests {
    use aviate_core::mixer::{Sanitizer, ActuatorSanitizer, ActuatorCmd, ModeConfig, ActuatorGroupConfig, GroupKind, CouplingKind, FallbackPolicy, GroupVector, MAX_ACTUATORS, GroupSanitizeResult};
    use aviate_core::control::ConfigMode;
    use aviate_core::types::{Normalized, Scalar};
    use aviate_core::time::{Timestamp, TimeSource};

    const TEST_GROUP_MEMBERS: &[u8] = &[0, 1, 2, 3];
    const TEST_SAFE_PATTERN: GroupVector = GroupVector {
        outputs: [Normalized(0.1); MAX_ACTUATORS],
        mask: 0x000F,
        valid: true,
    };

    fn make_cmd() -> ActuatorCmd {
        ActuatorCmd {
            outputs: [Normalized(0.5); MAX_ACTUATORS],
            active_mask: 0xFFFF,
            sequence: 0,
            timestamp: Timestamp { ticks: 0, source: TimeSource::Internal },
            fallback_mask: 0,
            sanitized: false,
        }
    }

    // Workaround for static slice in tests: use leak? Or define multiple statics.
    static STRONG_GROUP: [ActuatorGroupConfig; 1] = [
        ActuatorGroupConfig {
            kind: GroupKind::Multirotor,
            coupling: CouplingKind::Strong,
            fallback: FallbackPolicy::HoldLastGood,
            members: TEST_GROUP_MEMBERS,
            safe_pattern: TEST_SAFE_PATTERN,
        }
    ];

    static WEAK_GROUP: [ActuatorGroupConfig; 1] = [
        ActuatorGroupConfig {
            kind: GroupKind::DistributedThrust,
            coupling: CouplingKind::Weak,
            fallback: FallbackPolicy::SafePattern,
            members: TEST_GROUP_MEMBERS,
            safe_pattern: TEST_SAFE_PATTERN,
        }
    ];

    #[test]
    fn test_sanitizer_all_valid() {
        let mut sanitizer = Sanitizer::default();
        let mut cmd = make_cmd();
        let mode = ModeConfig { mode: ConfigMode::Hover, groups: &STRONG_GROUP };

        let report = sanitizer.sanitize(&mut cmd, &mode);

        assert!(!report.any_fallback);
        assert_eq!(report.group_results[0], GroupSanitizeResult::AllValid);
        assert_eq!(cmd.outputs[0].0, 0.5);
    }

    #[test]
    fn test_sanitizer_nan_rejection_strong() {
        let mut sanitizer = Sanitizer::default();
        let mut cmd = make_cmd();
        let mode = ModeConfig { mode: ConfigMode::Hover, groups: &STRONG_GROUP };

        // Inject NaN
        cmd.outputs[1] = Normalized(Scalar::NAN);

        // First run: no last good, should fallback to safe
        let report = sanitizer.sanitize(&mut cmd, &mode);

        assert!(report.any_fallback);
        assert_eq!(report.group_results[0], GroupSanitizeResult::FallbackSafe);
        // Entire group should be replaced by safe pattern (0.1)
        assert_eq!(cmd.outputs[0].0, 0.1); // Channel 0 was 0.5, now 0.1
        assert_eq!(cmd.outputs[1].0, 0.1);
    }

    #[test]
    fn test_sanitizer_last_good_fallback() {
        let mut sanitizer = Sanitizer::default();
        let mode = ModeConfig { mode: ConfigMode::Hover, groups: &STRONG_GROUP };

        // 1. Valid cycle to establish last_good
        let mut cmd1 = make_cmd();
        cmd1.outputs[0] = Normalized(0.8);
        sanitizer.sanitize(&mut cmd1, &mode);

        // 2. Invalid cycle
        let mut cmd2 = make_cmd();
        cmd2.outputs[0] = Normalized(Scalar::NAN);
        
        let report = sanitizer.sanitize(&mut cmd2, &mode);
        
        assert!(report.any_fallback);
        assert_eq!(report.group_results[0], GroupSanitizeResult::FallbackLastGood);
        // Should use last good (0.8)
        assert_eq!(cmd2.outputs[0].0, 0.8);
        assert_eq!(cmd2.outputs[1].0, 0.5); // cmd1[1] was 0.5
    }
    
    #[test]
    fn test_sanitizer_weak_coupling() {
        let mut sanitizer = Sanitizer::default();
        let mode = ModeConfig { mode: ConfigMode::Cruise, groups: &WEAK_GROUP };
        
        let mut cmd = make_cmd();
        cmd.outputs[0] = Normalized(0.5);
        cmd.outputs[1] = Normalized(Scalar::NAN); // Bad
        
        let _report = sanitizer.sanitize(&mut cmd, &mode);
        
        // Weak coupling: bad channel falls back, good channel stays?
        // Current implementation: Clamped/Fallback logic for weak
        
        // Channel 0 should be preserved
        assert_eq!(cmd.outputs[0].0, 0.5);
        // Channel 1 should be safe pattern (0.1)
        assert_eq!(cmd.outputs[1].0, 0.1);
    }

    use aviate_core::mixer::{QuadXMixer, Mixer};
    use aviate_core::control::AxisCommand;
    use aviate_core::types::NormalizedSigned;

    fn dummy_timestamp() -> Timestamp {
        Timestamp { ticks: 123, source: TimeSource::Internal }
    }

    #[test]
    fn test_quad_mixer_hover() {
        let mixer = QuadXMixer { timestamp_source: dummy_timestamp };
        let axis = AxisCommand {
            roll: NormalizedSigned(0.0),
            pitch: NormalizedSigned(0.0),
            yaw: NormalizedSigned(0.0),
            collective: Normalized(0.5),
        };
        
        let cmd = mixer.mix(&axis);
        
        // All motors should be 0.5
        for i in 0..4 {
            assert!((cmd.outputs[i].0 - 0.5).abs() < 1e-5);
        }
    }

    #[test]
    fn test_quad_mixer_roll() {
        let mixer = QuadXMixer { timestamp_source: dummy_timestamp };
        // Roll right (+0.1)
        // M0(FR, CW): -0.1 -> 0.4
        // M1(FL, CCW): +0.1 -> 0.6
        // M2(RL, CCW): +0.1 -> 0.6
        // M3(RR, CW): -0.1 -> 0.4
        let axis = AxisCommand {
            roll: NormalizedSigned(0.1),
            pitch: NormalizedSigned(0.0),
            yaw: NormalizedSigned(0.0),
            collective: Normalized(0.5),
        };
        
        let cmd = mixer.mix(&axis);
        
        assert!((cmd.outputs[0].0 - 0.4).abs() < 1e-5);
        assert!((cmd.outputs[1].0 - 0.6).abs() < 1e-5);
        assert!((cmd.outputs[2].0 - 0.6).abs() < 1e-5);
        assert!((cmd.outputs[3].0 - 0.4).abs() < 1e-5);
    }

    #[test]
    fn test_quad_mixer_saturation() {
        let mixer = QuadXMixer { timestamp_source: dummy_timestamp };
        // High collective + roll
        // t=0.9, r=0.2
        // M1 = 0.9 + 0.2 = 1.1 -> clamped to 1.0
        let axis = AxisCommand {
            roll: NormalizedSigned(0.2),
            pitch: NormalizedSigned(0.0),
            yaw: NormalizedSigned(0.0),
            collective: Normalized(0.9),
        };
        
        let cmd = mixer.mix(&axis);
        
        assert!((cmd.outputs[1].0 - 1.0).abs() < 1e-5);
    }

}
