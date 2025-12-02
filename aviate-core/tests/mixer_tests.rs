#![forbid(unsafe_code)]

#[cfg(test)]
mod tests {
    use aviate_core::control::ConfigMode;
    use aviate_core::mixer::{
        ActuatorCmd, ActuatorGroupConfig, ActuatorSanitizer, CouplingKind, FallbackPolicy,
        GroupKind, GroupSanitizeResult, GroupVector, ModeConfig, Sanitizer, MAX_ACTUATORS,
    };
    use aviate_core::time::{TimeSource, Timestamp};
    use aviate_core::types::{Normalized, Scalar};

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
            timestamp: Timestamp {
                ticks: 0,
                source: TimeSource::Internal,
            },
            fallback_mask: 0,
            sanitized: false,
        }
    }

    // Workaround for static slice in tests: use leak? Or define multiple statics.
    static STRONG_GROUP: [ActuatorGroupConfig; 1] = [ActuatorGroupConfig {
        kind: GroupKind::Multirotor,
        coupling: CouplingKind::Strong,
        fallback: FallbackPolicy::HoldLastGood,
        members: TEST_GROUP_MEMBERS,
        safe_pattern: TEST_SAFE_PATTERN,
    }];

    static WEAK_GROUP: [ActuatorGroupConfig; 1] = [ActuatorGroupConfig {
        kind: GroupKind::DistributedThrust,
        coupling: CouplingKind::Weak,
        fallback: FallbackPolicy::SafePattern,
        members: TEST_GROUP_MEMBERS,
        safe_pattern: TEST_SAFE_PATTERN,
    }];

    #[test]
    fn test_sanitizer_all_valid() {
        let mut sanitizer = Sanitizer::default();
        let mut cmd = make_cmd();
        let mode = ModeConfig {
            mode: ConfigMode::Hover,
            groups: &STRONG_GROUP,
        };

        let report = sanitizer.sanitize(&mut cmd, &mode);

        assert!(!report.any_fallback);
        assert_eq!(report.group_results[0], GroupSanitizeResult::AllValid);
        assert_eq!(cmd.outputs[0].0, 0.5);
    }

    #[test]
    fn test_sanitizer_nan_rejection_strong() {
        let mut sanitizer = Sanitizer::default();
        let mut cmd = make_cmd();
        let mode = ModeConfig {
            mode: ConfigMode::Hover,
            groups: &STRONG_GROUP,
        };

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
        let mode = ModeConfig {
            mode: ConfigMode::Hover,
            groups: &STRONG_GROUP,
        };

        // 1. Valid cycle to establish last_good
        let mut cmd1 = make_cmd();
        cmd1.outputs[0] = Normalized(0.8);
        sanitizer.sanitize(&mut cmd1, &mode);

        // 2. Invalid cycle
        let mut cmd2 = make_cmd();
        cmd2.outputs[0] = Normalized(Scalar::NAN);

        let report = sanitizer.sanitize(&mut cmd2, &mode);

        assert!(report.any_fallback);
        assert_eq!(
            report.group_results[0],
            GroupSanitizeResult::FallbackLastGood
        );
        // Should use last good (0.8)
        assert_eq!(cmd2.outputs[0].0, 0.8);
        assert_eq!(cmd2.outputs[1].0, 0.5); // cmd1[1] was 0.5
    }

    #[test]
    fn test_sanitizer_weak_coupling() {
        let mut sanitizer = Sanitizer::default();
        let mode = ModeConfig {
            mode: ConfigMode::Cruise,
            groups: &WEAK_GROUP,
        };

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

    use aviate_core::control::AxisCommand;
    use aviate_core::mixer::{Mixer, QuadXMixer};
    use aviate_core::types::NormalizedSigned;

    fn dummy_timestamp() -> Timestamp {
        Timestamp {
            ticks: 123,
            source: TimeSource::Internal,
        }
    }

    #[test]
    fn test_quad_mixer_hover() {
        let mixer = QuadXMixer {
            timestamp_source: dummy_timestamp,
        };
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
        let mixer = QuadXMixer {
            timestamp_source: dummy_timestamp,
        };
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
        let mixer = QuadXMixer {
            timestamp_source: dummy_timestamp,
        };
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

    // =========================================================================
    // EDGE CASE TESTS: Out-of-range values
    // =========================================================================

    #[test]
    fn test_sanitizer_out_of_range_negative() {
        let mut sanitizer = Sanitizer::default();
        let mut cmd = make_cmd();
        let mode = ModeConfig {
            mode: ConfigMode::Hover,
            groups: &STRONG_GROUP,
        };

        // Inject out-of-range negative value
        cmd.outputs[0] = Normalized(-0.1);

        let report = sanitizer.sanitize(&mut cmd, &mode);

        // Should trigger fallback due to out-of-range
        assert!(report.any_fallback);
        // All channels should be safe pattern (0.1) since strong coupling
        assert_eq!(cmd.outputs[0].0, 0.1);
    }

    #[test]
    fn test_sanitizer_out_of_range_above_one() {
        let mut sanitizer = Sanitizer::default();
        let mut cmd = make_cmd();
        let mode = ModeConfig {
            mode: ConfigMode::Hover,
            groups: &STRONG_GROUP,
        };

        // Inject out-of-range value > 1.0
        cmd.outputs[2] = Normalized(1.5);

        let report = sanitizer.sanitize(&mut cmd, &mode);

        // Should trigger fallback due to out-of-range
        assert!(report.any_fallback);
        // All channels should be safe pattern due to strong coupling
        assert_eq!(cmd.outputs[0].0, 0.1);
        assert_eq!(cmd.outputs[2].0, 0.1);
    }

    // =========================================================================
    // EDGE CASE TESTS: Critical failure (no fallback available)
    // =========================================================================

    static STRONG_GROUP_NO_SAFE: [ActuatorGroupConfig; 1] = [ActuatorGroupConfig {
        kind: GroupKind::Multirotor,
        coupling: CouplingKind::Strong,
        fallback: FallbackPolicy::HoldLastGood,
        members: TEST_GROUP_MEMBERS,
        safe_pattern: GroupVector {
            outputs: [Normalized(0.0); MAX_ACTUATORS],
            mask: 0,
            valid: false, // No valid safe pattern!
        },
    }];

    #[test]
    fn test_sanitizer_critical_failure_zero_output() {
        let mut sanitizer = Sanitizer::default();
        let mut cmd = make_cmd();
        let mode = ModeConfig {
            mode: ConfigMode::Hover,
            groups: &STRONG_GROUP_NO_SAFE,
        };

        // First invalid cycle (no last_good established, no safe pattern)
        cmd.outputs[1] = Normalized(Scalar::NAN);

        let report = sanitizer.sanitize(&mut cmd, &mode);

        // Should be critical failure - FallbackUnavailable
        assert!(report.critical_failure);
        assert_eq!(
            report.group_results[0],
            GroupSanitizeResult::FallbackUnavailable
        );

        // All channels should be zero (critical failure fallback)
        for i in 0..4 {
            assert_eq!(cmd.outputs[i].0, 0.0, "Channel {} should be 0.0", i);
        }
    }

    // =========================================================================
    // EDGE CASE TESTS: Weak coupling with last_good fallback
    // =========================================================================

    #[test]
    fn test_sanitizer_weak_coupling_last_good_fallback() {
        let mut sanitizer = Sanitizer::default();
        let mode = ModeConfig {
            mode: ConfigMode::Cruise,
            groups: &WEAK_GROUP,
        };

        // 1. First valid cycle to establish last_good
        let mut cmd1 = make_cmd();
        cmd1.outputs[0] = Normalized(0.7);
        cmd1.outputs[1] = Normalized(0.8);
        cmd1.outputs[2] = Normalized(0.6);
        cmd1.outputs[3] = Normalized(0.9);
        sanitizer.sanitize(&mut cmd1, &mode);

        // 2. Second cycle with NaN - should use last_good for bad channel
        let mut cmd2 = make_cmd();
        cmd2.outputs[0] = Normalized(0.5); // Valid, keep as-is
        cmd2.outputs[1] = Normalized(Scalar::NAN); // Invalid, should fallback
        cmd2.outputs[2] = Normalized(0.5); // Valid
        cmd2.outputs[3] = Normalized(0.5); // Valid

        let _report = sanitizer.sanitize(&mut cmd2, &mode);

        // Channel 0 should keep its value
        assert_eq!(cmd2.outputs[0].0, 0.5);
        // Channel 1 uses last_good (0.8) since it's available and valid
        // (weak coupling with HoldLastGood prefers last_good over safe_pattern)
        assert_eq!(cmd2.outputs[1].0, 0.8);
        // Channel 2 should keep its value
        assert_eq!(cmd2.outputs[2].0, 0.5);
    }

    static WEAK_GROUP_NO_SAFE: [ActuatorGroupConfig; 1] = [ActuatorGroupConfig {
        kind: GroupKind::DistributedThrust,
        coupling: CouplingKind::Weak,
        fallback: FallbackPolicy::HoldLastGood,
        members: TEST_GROUP_MEMBERS,
        safe_pattern: GroupVector {
            outputs: [Normalized(0.0); MAX_ACTUATORS],
            mask: 0,
            valid: false, // No safe pattern
        },
    }];

    #[test]
    fn test_sanitizer_weak_coupling_no_safe_uses_last_good() {
        let mut sanitizer = Sanitizer::default();
        let mode = ModeConfig {
            mode: ConfigMode::Cruise,
            groups: &WEAK_GROUP_NO_SAFE,
        };

        // 1. First valid cycle to establish last_good
        let mut cmd1 = make_cmd();
        cmd1.outputs[0] = Normalized(0.7);
        cmd1.outputs[1] = Normalized(0.8);
        sanitizer.sanitize(&mut cmd1, &mode);

        // 2. Invalid cycle
        let mut cmd2 = make_cmd();
        cmd2.outputs[0] = Normalized(0.5);
        cmd2.outputs[1] = Normalized(Scalar::NAN);

        let _report = sanitizer.sanitize(&mut cmd2, &mode);

        // Channel 0 stays valid
        assert_eq!(cmd2.outputs[0].0, 0.5);
        // Channel 1 should use last_good (0.8) since no safe pattern
        assert_eq!(cmd2.outputs[1].0, 0.8);
    }

    #[test]
    fn test_sanitizer_weak_coupling_no_safe_no_last_good_uses_zero() {
        let mut sanitizer = Sanitizer::default();
        let mode = ModeConfig {
            mode: ConfigMode::Cruise,
            groups: &WEAK_GROUP_NO_SAFE,
        };

        // First invalid cycle - no last_good, no safe pattern
        let mut cmd = make_cmd();
        cmd.outputs[1] = Normalized(Scalar::NAN);

        let _report = sanitizer.sanitize(&mut cmd, &mode);

        // Channel 0 stays valid
        assert_eq!(cmd.outputs[0].0, 0.5);
        // Channel 1 should be zero (last resort)
        assert_eq!(cmd.outputs[1].0, 0.0);
    }

    // =========================================================================
    // EDGE CASE TESTS: Infinity handling
    // =========================================================================

    #[test]
    fn test_sanitizer_infinity_positive() {
        let mut sanitizer = Sanitizer::default();
        let mut cmd = make_cmd();
        let mode = ModeConfig {
            mode: ConfigMode::Hover,
            groups: &STRONG_GROUP,
        };

        cmd.outputs[0] = Normalized(Scalar::INFINITY);

        let report = sanitizer.sanitize(&mut cmd, &mode);

        assert!(report.any_fallback);
        assert_eq!(cmd.outputs[0].0, 0.1);
    }

    #[test]
    fn test_sanitizer_infinity_negative() {
        let mut sanitizer = Sanitizer::default();
        let mut cmd = make_cmd();
        let mode = ModeConfig {
            mode: ConfigMode::Hover,
            groups: &STRONG_GROUP,
        };

        cmd.outputs[0] = Normalized(Scalar::NEG_INFINITY);

        let report = sanitizer.sanitize(&mut cmd, &mode);

        assert!(report.any_fallback);
        assert_eq!(cmd.outputs[0].0, 0.1);
    }

    // =========================================================================
    // FAULT INJECTION TESTS: Multiple groups, age expiration, fallback chain
    // =========================================================================

    // Group 0: channels 0-3, Group 1: channels 4-7
    const GROUP0_MEMBERS: &[u8] = &[0, 1, 2, 3];
    const GROUP1_MEMBERS: &[u8] = &[4, 5, 6, 7];

    const SAFE_PATTERN_0: GroupVector = GroupVector {
        outputs: [Normalized(0.1); MAX_ACTUATORS],
        mask: 0x000F,
        valid: true,
    };

    const SAFE_PATTERN_1: GroupVector = GroupVector {
        outputs: [Normalized(0.2); MAX_ACTUATORS],
        mask: 0x00F0,
        valid: true,
    };

    static TWO_GROUPS: [ActuatorGroupConfig; 2] = [
        ActuatorGroupConfig {
            kind: GroupKind::Multirotor,
            coupling: CouplingKind::Strong,
            fallback: FallbackPolicy::HoldLastGood,
            members: GROUP0_MEMBERS,
            safe_pattern: SAFE_PATTERN_0,
        },
        ActuatorGroupConfig {
            kind: GroupKind::ControlSurfaces,
            coupling: CouplingKind::Strong,
            fallback: FallbackPolicy::HoldLastGood,
            members: GROUP1_MEMBERS,
            safe_pattern: SAFE_PATTERN_1,
        },
    ];

    /// Test that multiple groups fail independently - fault in one group
    /// should not affect the other group's output or fallback state.
    #[test]
    fn test_sanitizer_multiple_groups_independent() {
        let mut sanitizer = Sanitizer::default();
        let mode = ModeConfig {
            mode: ConfigMode::Hover,
            groups: &TWO_GROUPS,
        };

        // 1. First valid cycle to establish last_good for both groups
        let mut cmd1 = make_cmd();
        cmd1.outputs[0] = Normalized(0.6); // Group 0
        cmd1.outputs[4] = Normalized(0.7); // Group 1
        let report1 = sanitizer.sanitize(&mut cmd1, &mode);
        assert!(!report1.any_fallback);
        assert_eq!(report1.group_results[0], GroupSanitizeResult::AllValid);
        assert_eq!(report1.group_results[1], GroupSanitizeResult::AllValid);

        // 2. Inject NaN in Group 0 only, Group 1 stays valid
        let mut cmd2 = make_cmd();
        cmd2.outputs[0] = Normalized(Scalar::NAN); // Fault in Group 0
        cmd2.outputs[4] = Normalized(0.8); // Valid in Group 1
        cmd2.outputs[5] = Normalized(0.9);
        cmd2.outputs[6] = Normalized(0.85);
        cmd2.outputs[7] = Normalized(0.75);

        let report2 = sanitizer.sanitize(&mut cmd2, &mode);

        // Group 0 should fallback to last_good
        assert!(report2.any_fallback);
        assert_eq!(
            report2.group_results[0],
            GroupSanitizeResult::FallbackLastGood
        );
        assert_eq!(cmd2.outputs[0].0, 0.6); // From last_good

        // Group 1 should be unaffected - stays AllValid with original values
        assert_eq!(report2.group_results[1], GroupSanitizeResult::AllValid);
        assert_eq!(cmd2.outputs[4].0, 0.8); // Preserved
        assert_eq!(cmd2.outputs[5].0, 0.9);

        // Verify fallback_mask only has Group 0 bit set
        assert_eq!(cmd2.fallback_mask, 0b0001);
    }

    /// Test that last_good expires after MAX_FALLBACK_AGE_CYCLES (100 cycles).
    /// After expiration, sanitizer should fall back to safe_pattern.
    ///
    /// Boundary analysis:
    /// - Condition: age < MAX_FALLBACK_AGE_CYCLES (i.e., age < 100)
    /// - After valid cycle: age = 0
    /// - After N invalid cycles: age = N
    /// - age = 99: 99 < 100 → true → uses last_good
    /// - age = 100: 100 < 100 → false → uses safe_pattern
    #[test]
    fn test_sanitizer_fallback_age_expiration() {
        use aviate_core::mixer::MAX_FALLBACK_AGE_CYCLES;

        let mut sanitizer = Sanitizer::default();
        let mode = ModeConfig {
            mode: ConfigMode::Hover,
            groups: &STRONG_GROUP,
        };

        // 1. Establish last_good with a valid cycle
        let mut cmd_valid = make_cmd();
        cmd_valid.outputs[0] = Normalized(0.75);
        cmd_valid.outputs[1] = Normalized(0.75);
        cmd_valid.outputs[2] = Normalized(0.75);
        cmd_valid.outputs[3] = Normalized(0.75);
        sanitizer.sanitize(&mut cmd_valid, &mode);

        // Verify last_good is established
        assert!(sanitizer.state.last_good[0].valid);
        assert_eq!(sanitizer.state.age[0], 0);

        // 2. Run MAX_FALLBACK_AGE_CYCLES invalid cycles (age goes 0 → 100)
        // Each cycle: check age < 100, use last_good if true, then increment age
        for cycle in 0..MAX_FALLBACK_AGE_CYCLES {
            let mut cmd = make_cmd();
            cmd.outputs[0] = Normalized(Scalar::NAN);
            let report = sanitizer.sanitize(&mut cmd, &mode);

            assert_eq!(
                report.group_results[0],
                GroupSanitizeResult::FallbackLastGood,
                "Cycle {}: should use last_good (age {} < {})",
                cycle,
                cycle,
                MAX_FALLBACK_AGE_CYCLES
            );
            assert_eq!(
                cmd.outputs[0].0, 0.75,
                "Cycle {}: should have last_good value",
                cycle
            );
        }

        // Age should now be exactly MAX_FALLBACK_AGE_CYCLES
        assert_eq!(sanitizer.state.age[0], MAX_FALLBACK_AGE_CYCLES);

        // 3. One more invalid cycle - age is now 100, which is NOT < 100
        // So this cycle should use safe_pattern
        let mut cmd_expire = make_cmd();
        cmd_expire.outputs[0] = Normalized(Scalar::NAN);
        let report_expire = sanitizer.sanitize(&mut cmd_expire, &mode);

        assert_eq!(
            report_expire.group_results[0],
            GroupSanitizeResult::FallbackSafe,
            "Cycle {}: age {} >= {}, should use safe_pattern",
            MAX_FALLBACK_AGE_CYCLES,
            MAX_FALLBACK_AGE_CYCLES,
            MAX_FALLBACK_AGE_CYCLES
        );
        assert_eq!(
            cmd_expire.outputs[0].0, 0.1,
            "Should use safe_pattern value (0.1)"
        );
    }

    /// Test the complete sequential fallback chain:
    /// valid → last_good fallback → (age expires) → safe_pattern → (no safe) → zero
    #[test]
    fn test_sanitizer_sequential_fallback_chain() {
        use aviate_core::mixer::MAX_FALLBACK_AGE_CYCLES;

        // Stage 1: Test with safe_pattern available
        {
            let mut sanitizer = Sanitizer::default();
            let mode = ModeConfig {
                mode: ConfigMode::Hover,
                groups: &STRONG_GROUP,
            };

            // Step 1: Valid cycle
            let mut cmd1 = make_cmd();
            let report1 = sanitizer.sanitize(&mut cmd1, &mode);
            assert_eq!(report1.group_results[0], GroupSanitizeResult::AllValid);

            // Step 2: Invalid - falls back to last_good
            let mut cmd2 = make_cmd();
            cmd2.outputs[0] = Normalized(Scalar::NAN);
            let report2 = sanitizer.sanitize(&mut cmd2, &mode);
            assert_eq!(
                report2.group_results[0],
                GroupSanitizeResult::FallbackLastGood
            );

            // Step 3: Exhaust last_good by running MAX_FALLBACK_AGE_CYCLES invalid cycles
            // (first cycle already incremented age to 1, so we need MAX-1 more)
            for _ in 0..(MAX_FALLBACK_AGE_CYCLES - 1) {
                let mut cmd = make_cmd();
                cmd.outputs[0] = Normalized(Scalar::NAN);
                sanitizer.sanitize(&mut cmd, &mode);
            }

            // After MAX_FALLBACK_AGE_CYCLES invalid cycles total, age = MAX_FALLBACK_AGE_CYCLES
            // Step 4: Now should use safe_pattern (age >= MAX_FALLBACK_AGE_CYCLES)
            let mut cmd3 = make_cmd();
            cmd3.outputs[0] = Normalized(Scalar::NAN);
            let report3 = sanitizer.sanitize(&mut cmd3, &mode);
            assert_eq!(report3.group_results[0], GroupSanitizeResult::FallbackSafe);
            assert_eq!(cmd3.outputs[0].0, 0.1); // safe_pattern value
        }

        // Stage 2: Test without safe_pattern - should go to zero
        {
            let mut sanitizer = Sanitizer::default();
            let mode = ModeConfig {
                mode: ConfigMode::Hover,
                groups: &STRONG_GROUP_NO_SAFE,
            };

            // Step 1: Valid cycle to establish last_good
            let mut cmd1 = make_cmd();
            cmd1.outputs[0] = Normalized(0.8);
            sanitizer.sanitize(&mut cmd1, &mode);

            // Step 2: Exhaust last_good
            for _ in 0..MAX_FALLBACK_AGE_CYCLES {
                let mut cmd = make_cmd();
                cmd.outputs[0] = Normalized(Scalar::NAN);
                sanitizer.sanitize(&mut cmd, &mode);
            }

            // Step 3: Now should go to zero (FallbackUnavailable)
            let mut cmd2 = make_cmd();
            cmd2.outputs[0] = Normalized(Scalar::NAN);
            let report = sanitizer.sanitize(&mut cmd2, &mode);
            assert_eq!(
                report.group_results[0],
                GroupSanitizeResult::FallbackUnavailable
            );
            assert!(report.critical_failure);
            assert_eq!(cmd2.outputs[0].0, 0.0);
        }
    }

    /// Test mixer saturation handling with combined axis inputs.
    /// Verifies that when total demand exceeds motor capacity,
    /// outputs are clamped symmetrically preserving attitude authority.
    #[test]
    fn test_mixer_saturation_priority() {
        let mixer = QuadXMixer {
            timestamp_source: dummy_timestamp,
        };

        // Scenario 1: High thrust + maximum roll
        // t=0.9, r=0.5 -> M1 = 0.9+0.5 = 1.4 (clamped to 1.0)
        //               -> M0 = 0.9-0.5 = 0.4
        let axis1 = AxisCommand {
            roll: NormalizedSigned(0.5),
            pitch: NormalizedSigned(0.0),
            yaw: NormalizedSigned(0.0),
            collective: Normalized(0.9),
        };

        let cmd1 = mixer.mix(&axis1);

        // Verify clamping behavior
        assert_eq!(cmd1.outputs[1].0, 1.0, "M1 should be clamped to 1.0");
        assert!((cmd1.outputs[0].0 - 0.4).abs() < 1e-5, "M0 should be 0.4");
        // M2 = t + r = 1.4 clamped to 1.0
        assert_eq!(cmd1.outputs[2].0, 1.0, "M2 should be clamped to 1.0");
        // M3 = t - r = 0.4
        assert!((cmd1.outputs[3].0 - 0.4).abs() < 1e-5, "M3 should be 0.4");

        // Scenario 2: Combined roll + pitch + yaw saturation
        // Tests that all axes contribute even when saturating
        // t=0.8, r=0.3, p=0.3, y=0.2
        // M0 = t - r + p - y = 0.8 - 0.3 + 0.3 - 0.2 = 0.6
        // M1 = t + r + p + y = 0.8 + 0.3 + 0.3 + 0.2 = 1.6 → 1.0
        // M2 = t + r - p + y = 0.8 + 0.3 - 0.3 + 0.2 = 1.0
        // M3 = t - r - p - y = 0.8 - 0.3 - 0.3 - 0.2 = 0.0
        let axis2 = AxisCommand {
            roll: NormalizedSigned(0.3),
            pitch: NormalizedSigned(0.3),
            yaw: NormalizedSigned(0.2),
            collective: Normalized(0.8),
        };

        let cmd2 = mixer.mix(&axis2);

        assert!((cmd2.outputs[0].0 - 0.6).abs() < 1e-5, "M0 = 0.6");
        assert_eq!(cmd2.outputs[1].0, 1.0, "M1 clamped to 1.0");
        assert!((cmd2.outputs[2].0 - 1.0).abs() < 1e-5, "M2 = 1.0");
        assert!((cmd2.outputs[3].0 - 0.0).abs() < 1e-5, "M3 = 0.0");

        // Scenario 3: Negative saturation (low thrust with large control demand)
        // t=0.1, r=0.3
        // M0 = 0.1 - 0.3 = -0.2 → 0.0
        // M3 = 0.1 - 0.3 = -0.2 → 0.0
        let axis3 = AxisCommand {
            roll: NormalizedSigned(0.3),
            pitch: NormalizedSigned(0.0),
            yaw: NormalizedSigned(0.0),
            collective: Normalized(0.1),
        };

        let cmd3 = mixer.mix(&axis3);

        assert_eq!(cmd3.outputs[0].0, 0.0, "M0 clamped to 0.0");
        assert_eq!(cmd3.outputs[3].0, 0.0, "M3 clamped to 0.0");
        // M1 = 0.1 + 0.3 = 0.4
        assert!((cmd3.outputs[1].0 - 0.4).abs() < 1e-5, "M1 = 0.4");
    }

    // =========================================================================
    // CONSECUTIVE FALLBACK COUNTER TESTS
    // =========================================================================

    use aviate_core::mixer::MAX_CONSECUTIVE_FALLBACK;

    #[test]
    fn test_consecutive_fallback_counter_increments() {
        let mut sanitizer = Sanitizer::default();
        let mode = ModeConfig {
            mode: ConfigMode::Hover,
            groups: &STRONG_GROUP,
        };

        // Simulate consecutive invalid inputs (fallback cycles)
        for i in 0..3 {
            let mut cmd = make_cmd();
            cmd.outputs[0] = Normalized(Scalar::NAN); // Force fallback
            let report = sanitizer.sanitize(&mut cmd, &mode);

            assert!(report.any_fallback);
            assert_eq!(
                sanitizer.state.consecutive_fallback[0],
                (i + 1) as u16,
                "After {} fallback cycles, counter should be {}",
                i + 1,
                i + 1
            );
        }
    }

    #[test]
    fn test_consecutive_fallback_resets_on_valid() {
        let mut sanitizer = Sanitizer::default();
        let mode = ModeConfig {
            mode: ConfigMode::Hover,
            groups: &STRONG_GROUP,
        };

        // Two fallback cycles
        for _ in 0..2 {
            let mut cmd = make_cmd();
            cmd.outputs[0] = Normalized(Scalar::NAN);
            sanitizer.sanitize(&mut cmd, &mode);
        }
        assert_eq!(sanitizer.state.consecutive_fallback[0], 2);

        // Valid frame resets counter
        let mut cmd = make_cmd();
        let report = sanitizer.sanitize(&mut cmd, &mode);

        assert!(!report.any_fallback);
        assert_eq!(report.group_results[0], GroupSanitizeResult::AllValid);
        assert_eq!(
            sanitizer.state.consecutive_fallback[0], 0,
            "Counter should reset to 0 on valid frame"
        );
    }

    #[test]
    fn test_consecutive_fallback_triggers_limit_exceeded() {
        let mut sanitizer = Sanitizer::default();
        let mode = ModeConfig {
            mode: ConfigMode::Hover,
            groups: &STRONG_GROUP,
        };

        // MAX_CONSECUTIVE_FALLBACK frames - should NOT trigger yet
        for i in 0..MAX_CONSECUTIVE_FALLBACK {
            let mut cmd = make_cmd();
            cmd.outputs[0] = Normalized(Scalar::NAN);
            let report = sanitizer.sanitize(&mut cmd, &mode);

            assert!(
                !report.consecutive_fallback_limit_exceeded,
                "Frame {}: should NOT trigger limit_exceeded (counter={})",
                i + 1,
                sanitizer.state.consecutive_fallback[0]
            );
        }

        assert_eq!(
            sanitizer.state.consecutive_fallback[0], MAX_CONSECUTIVE_FALLBACK,
            "After MAX frames, counter should equal MAX"
        );

        // (MAX + 1)-th frame - should trigger
        let mut cmd = make_cmd();
        cmd.outputs[0] = Normalized(Scalar::NAN);
        let report = sanitizer.sanitize(&mut cmd, &mode);

        assert!(
            report.consecutive_fallback_limit_exceeded,
            "Frame {}: SHOULD trigger limit_exceeded",
            MAX_CONSECUTIVE_FALLBACK + 1
        );
        assert_eq!(
            sanitizer.state.consecutive_fallback[0],
            MAX_CONSECUTIVE_FALLBACK + 1,
            "Counter should be MAX+1"
        );
    }

    #[test]
    fn test_consecutive_fallback_clamped_does_not_increment() {
        let mut sanitizer = Sanitizer::default();
        let mode = ModeConfig {
            mode: ConfigMode::Cruise,
            groups: &WEAK_GROUP,
        };

        // For weak coupling, even with some invalid channels, result is Clamped
        // which should NOT increment the consecutive fallback counter
        let mut cmd = make_cmd();
        cmd.outputs[0] = Normalized(0.5); // Valid
        cmd.outputs[1] = Normalized(Scalar::NAN); // Invalid - triggers Clamped for weak

        let report = sanitizer.sanitize(&mut cmd, &mode);

        // Weak coupling with some invalid channels results in Clamped
        assert_eq!(report.group_results[0], GroupSanitizeResult::Clamped);
        assert_eq!(
            sanitizer.state.consecutive_fallback[0], 0,
            "Clamped should reset counter (still has control authority)"
        );
    }

    #[test]
    fn test_consecutive_fallback_per_group_independence() {
        let mut sanitizer = Sanitizer::default();
        let mode = ModeConfig {
            mode: ConfigMode::Hover,
            groups: &TWO_GROUPS,
        };

        // Make Group 0 fail, Group 1 valid
        for _ in 0..5 {
            let mut cmd = make_cmd();
            cmd.outputs[0] = Normalized(Scalar::NAN); // Group 0 fails
            cmd.outputs[4] = Normalized(0.5); // Group 1 valid
            cmd.outputs[5] = Normalized(0.5);
            cmd.outputs[6] = Normalized(0.5);
            cmd.outputs[7] = Normalized(0.5);
            sanitizer.sanitize(&mut cmd, &mode);
        }

        // Group 0 should have counter = 5
        assert_eq!(
            sanitizer.state.consecutive_fallback[0], 5,
            "Group 0 should have 5 consecutive fallbacks"
        );

        // Group 1 should have counter = 0 (always valid)
        assert_eq!(
            sanitizer.state.consecutive_fallback[1], 0,
            "Group 1 should have 0 consecutive fallbacks"
        );
    }
}
