//! The `update()` method on `AviateKernelImpl`.
//!
//! Isolated in its own file so the per-cycle control loop stays within
//! the 500-line per-.rs limit and is easy to locate at review time.

use crate::checks::DegradationReason;
use crate::control::{
    AuthorityProfile, Command, ControlLawV1, ControlMode, ModeEntryDecision, VehicleControlMode,
    VehicleController,
};
use crate::ekf::Estimator;
use crate::fault::FaultFlags;
use crate::kernel::AviateKernelImpl;
use crate::kernel_types::{
    ChannelHealthV1, ChannelId, ChannelStatus, ConfigTransitionState, CrossChannelData,
    CycleTiming, EnvelopeMargin, TerminalCause, UpdateResult, TIMING_VIOLATION_THRESHOLD,
};
use crate::mixer::{ActuatorCmd, ActuatorSanitizer, ActuatorState, Mixer, SanitizeReport};
use crate::sensor::SensorSet;
use crate::time::TimeDelta;

impl<E: Estimator, V: VehicleController, M: Mixer, S: ActuatorSanitizer>
    AviateKernelImpl<E, V, M, S>
{
    /// Main control update with in-flight monitoring (Spec §20)
    ///
    /// # Arguments
    /// * `channel` - Channel ID (primary/secondary/etc.)
    /// * `time` - Time delta since last update
    /// * `sensors` - Current sensor readings
    /// * `command` - The command to execute
    /// * `actuator_state` - Feedback from actuators
    /// * `cross_channel` - Data from other redundant channels (optional)
    // The 8-arg surface mirrors `AviateKernelTrait::update` and is
    // dictated by spec §20 — adding a struct wrapper here would just
    // shift the parameter count one level up. Allowed locally.
    #[allow(clippy::too_many_arguments)]
    pub fn update(
        &mut self,
        _channel: ChannelId,
        time: TimeDelta,
        sensors: &SensorSet,
        command: &Command,
        command_age_ms: u32,
        _actuator_state: &ActuatorState,
        _cross_channel: Option<&CrossChannelData>,
    ) -> UpdateResult {
        // Spec §18: Track timing statistics
        // NOTE: dt (time since last call) is tracked for statistics, but deadline violations
        // require external monitoring of actual update() execution time by the caller.
        // The deadline (800us) refers to how long update() should take, not the call interval.
        let dt_us = time.as_micros() as u32;

        // Update timing statistics (for monitoring, not degradation triggering)
        self.state.timing_stats.last_cycle_us = dt_us;
        self.state.timing_stats.total_cycles =
            self.state.timing_stats.total_cycles.saturating_add(1);

        if dt_us > self.state.timing_stats.max_cycle_us {
            self.state.timing_stats.max_cycle_us = dt_us;
        }
        if dt_us < self.state.timing_stats.min_cycle_us || self.state.timing_stats.min_cycle_us == 0
        {
            self.state.timing_stats.min_cycle_us = dt_us;
        }

        // Deadline violations are tracked by the caller via report_timing_violation()
        // which has access to actual execution time measurements

        // Basic timestamp for now
        let timestamp = crate::time::Timestamp {
            ticks: time.tick_delta,
            source: crate::time::TimeSource::Internal,
        };

        // COV:EXCL_START(DEFENSIVE: SEU/memory corruption detection - cannot trigger in unit tests)
        // SEU Resilience: Validate command enum fields (Spec §15.3)
        // This detects memory corruption in control-plane enums.
        // On failure: set ENUM_INVALID fault and force safe output.
        if !command.validate_enums() {
            self.state.faults.insert(FaultFlags::ENUM_INVALID);
            return UpdateResult {
                actuator: ActuatorCmd {
                    outputs: self.cfg.safe_output,
                    active_mask: 0,
                    sequence: command.sequence,
                    timestamp,
                    fallback_mask: 0,
                    sanitized: true,
                },
                status: ChannelStatus {
                    mode: ControlMode::Attitude, // Safe default
                    config_mode: self.state.mode,
                    transition_state: ConfigTransitionState::Stable(self.state.mode),
                    law: ControlLawV1::Backup,
                    health: ChannelHealthV1::Operative,
                    faults: self.state.faults,
                    confidence: self
                        .pipeline
                        .estimator
                        .estimate(&self.state.estimator)
                        .quality,
                    envelope_margin: EnvelopeMargin::default(),
                    sequence: command.sequence,
                    protection: Default::default(),
                    sanitize_report: SanitizeReport::default(),
                    mode_entry: ModeEntryDecision::Granted(ControlMode::Attitude),
                },
                estimate: self.pipeline.estimator.estimate(&self.state.estimator),
                timing: CycleTiming::default(),
                degradation: None,
            };
        }
        // COV:EXCL_STOP

        // 0. Update sensor fault flags (always, regardless of armed state)
        //    This allows continuous monitoring of sensor health.
        self.update_sensor_faults(sensors);

        // 1. Safety Gate: If not armed, force safe output immediately.
        //
        //    The ESTIMATOR still observes: a vehicle on the pad must
        //    converge — gyro predict, GNSS/baro aiding, POSITION/
        //    VELOCITY authorization — BEFORE arming, both because
        //    pre-arm nav checks are meaningless against a filter that
        //    has never fused, and because a pose-gated GCS cannot even
        //    send the first ARM while position is unauthorized (the
        //    #277 cold-start deadlock). Only actuator authority is
        //    gated on arming; state estimation runs from boot. The
        //    critical-fault guard is read-only here — the Fault-state
        //    transition in `check_critical_faults` is armed-flow
        //    semantics — and matches the armed path, which never
        //    observes under a critical fault either. The estimator's
        //    own health gates (per-sensor, plus the unseeded-filter
        //    gate in each update) decide what actually fuses.
        if !self.state.init_state.allows_active_control() {
            if !self
                .state
                .faults
                .intersects(crate::kernel_types::CRITICAL_FAULTS)
            {
                self.pipeline.estimator.observe(
                    &mut self.state.estimator,
                    sensors,
                    command.sensor_overrides.as_ref(),
                    time.dt_sec.0,
                );
            }
            return UpdateResult {
                actuator: ActuatorCmd {
                    outputs: self.cfg.safe_output,
                    active_mask: 0,
                    sequence: command.sequence,
                    timestamp,
                    fallback_mask: 0,
                    sanitized: true,
                },
                status: ChannelStatus {
                    mode: command.mode,
                    config_mode: self.state.mode,
                    transition_state: ConfigTransitionState::Stable(self.state.mode),
                    law: ControlLawV1::Backup, // Force Backup reporting when not armed
                    health: ChannelHealthV1::Operative,
                    faults: self.state.faults,
                    confidence: self
                        .pipeline
                        .estimator
                        .estimate(&self.state.estimator)
                        .quality,
                    envelope_margin: EnvelopeMargin::default(),
                    sequence: command.sequence,
                    protection: Default::default(),
                    sanitize_report: SanitizeReport::default(),
                    mode_entry: ModeEntryDecision::Granted(command.mode),
                },
                estimate: self.pipeline.estimator.estimate(&self.state.estimator),
                timing: CycleTiming::default(),
                degradation: None,
            };
        }

        // 2. Check for critical faults (if we got here, we're armed). These
        //    are the unrecoverable cases (total sensor loss, numeric /
        //    estimator divergence) HLR-FLT-203 requires the safe pattern
        //    for — not the Descend/Land terminal, which needs a state
        //    estimate this cycle hasn't computed yet. The ActuatorCmd and
        //    ChannelStatus mirror the Backup branch built in step 6/11 so
        //    telemetry reports the real fault state instead of defaults.
        if self.check_critical_faults() {
            return UpdateResult {
                actuator: ActuatorCmd {
                    outputs: self.cfg.safe_output,
                    active_mask: 0b1111,
                    sequence: command.sequence,
                    timestamp,
                    fallback_mask: 0xFF,
                    sanitized: true,
                },
                status: ChannelStatus {
                    mode: command.mode,
                    config_mode: self.state.mode,
                    transition_state: ConfigTransitionState::Stable(self.state.mode),
                    law: ControlLawV1::Backup,
                    health: ChannelHealthV1::Operative,
                    faults: self.state.faults,
                    confidence: self
                        .pipeline
                        .estimator
                        .estimate(&self.state.estimator)
                        .quality,
                    envelope_margin: EnvelopeMargin::default(),
                    sequence: command.sequence,
                    protection: Default::default(),
                    sanitize_report: SanitizeReport::default(),
                    mode_entry: ModeEntryDecision::Granted(command.mode),
                },
                estimate: self.pipeline.estimator.estimate(&self.state.estimator),
                timing: CycleTiming::default(),
                degradation: None,
            };
        }

        // 3. Estimator observation. Pipeline carries the algorithm
        //    identity (`&self`); KernelState owns the filter state
        //    (`&mut self.state.estimator`). Disjoint-borrow rule
        //    lets us read the algorithm and write the state in the
        //    same call. The estimator decides which sensor channels
        //    it consumes — the kernel does not pre-process or
        //    pre-select.
        self.pipeline.estimator.observe(
            &mut self.state.estimator,
            sensors,
            command.sensor_overrides.as_ref(),
            time.dt_sec.0,
        );

        // Get updated estimate
        let state = self.pipeline.estimator.estimate(&self.state.estimator);

        // 3. Update in-flight checks
        self.state.checks.in_flight.update_from_state(&state);
        self.state.checks.in_flight.update_from_sensors(sensors);
        // Geofence: flag whether the vehicle's measured altitude sits
        // within the configured band. NED z is down-positive; altitude
        // is up-positive.
        let altitude_m = -state.position_ned[2].0;
        let altitude_ok =
            (self.cfg.limits.min_altitude.0..=self.cfg.limits.max_altitude.0).contains(&altitude_m);
        self.state.checks.in_flight.update_altitude(altitude_ok);
        // Spec §12: Command staleness gate. The caller supplies
        // `command_age_ms` measured against its own timebase
        // (typically the time elapsed since the last RC/GCS frame
        // arrived); the in-flight check clears COMMAND_RECENT once
        // the age meets or exceeds `cfg.command_timeout_ms`.
        self.state
            .checks
            .in_flight
            .update_command_status(command_age_ms, self.cfg.command_timeout_ms);

        // 4a. Terminal release (LLR-FLT-209): command staleness is
        //     recoverable and SHALL NOT be latched. A Direct terminal
        //     that engaged for command loss releases in the same cycle
        //     command recency is restored; the cascade resumes flying
        //     the live command. A commanded land (TerminalCause::
        //     Commanded) stays latched, and Backup only releases via
        //     ground_reset.
        if self.state.control_law == ControlLawV1::Direct
            && self.state.terminal_cause == TerminalCause::CommandLoss
            && self
                .state
                .checks
                .in_flight
                .current
                .contains(crate::checks::InFlightFlags::COMMAND_RECENT)
        {
            self.state.control_law = ControlLawV1::Primary;
            self.state.terminal_cause = TerminalCause::None;
            // Restored authority means restored control objective; the
            // terminal's accumulated integrators must not leak into the
            // resumed law (LLR-CTL-101).
            self.pipeline.controller.reset(&mut self.state.controller);
        }

        // 4. Handle degradation
        // Timing violations are reported externally via report_timing_violation()
        let degradation =
            if self.state.timing_stats.consecutive_violations >= TIMING_VIOLATION_THRESHOLD {
                // Persistent timing violation → degrade to Alternate
                self.handle_degradation(DegradationReason::TimingViolation, timestamp)
            } else if let Some(reason) = self.state.checks.in_flight.get_degradation_trigger() {
                self.handle_degradation(reason, timestamp)
            } else {
                None
            };

        // 5. Envelope Protection
        use crate::control::envelope::EnvelopeProtector;
        let (constrained_sp, protection_status) = self.pipeline.protector.constrain(
            &command.setpoint,
            &state,
            &self.cfg.limits,
            AuthorityProfile::HardEnvelope,
        );

        self.state
            .checks
            .in_flight
            .update_from_envelope(&protection_status);

        let constrained_cmd = Command {
            setpoint: constrained_sp,
            ..command.clone()
        };

        // 6a. Mode-entry gate: a commanded Position/Velocity-family
        //    mode whose estimator requirement isn't met must not run
        //    those loops. Refuse entry and fall back to the highest
        //    mode current validity still supports — the kernel-side
        //    analog of PX4's `mode_requirements.cpp` — surfaced
        //    honestly below instead of echoing the raw request.
        //    Scoped to the uplinked command only: `Backup` already
        //    forces safe output regardless of mode, and the
        //    synthesized Descend/Land terminal command (`Direct`) is
        //    the existing terminal's own mode with its own validity
        //    story, not re-gated here.
        let mode_entry = match self.state.control_law {
            ControlLawV1::Backup => ModeEntryDecision::Granted(command.mode),
            ControlLawV1::Direct => ModeEntryDecision::Granted(ControlMode::AltitudeHold),
            ControlLawV1::Primary | ControlLawV1::Alternate => {
                crate::control::gate_mode_entry(constrained_cmd.mode, state.valid_flags)
            }
        };

        // 6b. Control Step
        // `Backup` is the motors-off last resort: reserved for cases
        // where a controlled descent is impossible (on the ground,
        // total loss of attitude, unrecoverable numeric/estimator
        // divergence). Every other law flies the cascade — including
        // the Descend/Land terminal (`Direct`), which rides a
        // kernel-synthesized level-descent setpoint down instead of
        // cutting thrust mid-air.
        let mut actuator_cmd = if self.state.control_law == ControlLawV1::Backup {
            ActuatorCmd {
                outputs: self.cfg.safe_output,
                active_mask: 0b1111,
                sequence: command.sequence,
                timestamp,
                fallback_mask: 0xFF,
                sanitized: true,
            }
        } else {
            // Terminal descent flies the synthesized level-descent
            // setpoint; all other laws fly the uplinked command,
            // gated to the mode-entry decision above. Loop selection
            // is driven by the control-mode flags derived from the
            // effective command's mode.
            let effective_cmd = if self.state.control_law == ControlLawV1::Direct {
                crate::kernel::descend::descend_command(
                    &state,
                    &self.cfg.limits,
                    constrained_cmd.sequence,
                )
            } else {
                crate::control::apply_mode_entry(constrained_cmd, mode_entry)
            };
            let control_flags = VehicleControlMode::from_control_mode(effective_cmd.mode);
            let axis_cmd = self.pipeline.controller.step(
                &mut self.state.controller,
                &state,
                &effective_cmd,
                &control_flags,
                self.state.mode,
                &self.cfg.limits,
            );
            self.pipeline.mixer.mix(&axis_cmd)
        };

        // 7. Snapshot previous-cycle commanded outputs for the slew
        //    limiter (DRQ-FLT-001 / DRQ-MORPH-001). Capture BEFORE
        //    update_commanded — `state.actuator_state.commanded` still
        //    holds last cycle's value at this point.
        let previous_outputs = self.state.actuator_state.commanded;

        // 8. Sanitization. Pipeline holds the algorithm (`&self`);
        //    KernelState owns the per-group fallback memory
        //    (`&mut self.state.fallback`).
        let sanitize_report = if self.state.control_law == ControlLawV1::Backup {
            SanitizeReport::default()
        } else {
            self.pipeline.sanitizer.sanitize(
                &mut actuator_cmd,
                &self.cfg.mode_config,
                &mut self.state.fallback,
            )
        };

        // 9. Slew limit (DRQ-FLT-001 / DRQ-MORPH-001). Applied after
        //    sanitization so both controller-bumps and sanitizer-
        //    induced bumps respect the per-cycle delta limit. Disabled
        //    per-channel when `slew_limit_per_cycle[i] <= 0`, so
        //    airframes that don't opt in see no behavior change.
        crate::kernel::slew::apply_slew_limit(
            &mut actuator_cmd,
            &previous_outputs,
            &self.cfg.slew_limit_per_cycle,
        );

        // 10. Record the final commanded outputs (post-slew). This is
        //     what next cycle's `previous_outputs` will read.
        self.state
            .actuator_state
            .update_commanded(&actuator_cmd, timestamp);

        // 11. Construct Result
        UpdateResult {
            actuator: actuator_cmd,
            status: ChannelStatus {
                mode: mode_entry.effective(),
                config_mode: self.state.mode,
                transition_state: ConfigTransitionState::Stable(self.state.mode),
                law: self.state.control_law,
                health: ChannelHealthV1::Operative,
                faults: self.state.faults,
                confidence: state.quality,
                envelope_margin: EnvelopeMargin::default(), // TODO calculate
                sequence: command.sequence,
                protection: protection_status,
                sanitize_report,
                mode_entry,
            },
            estimate: state,
            timing: CycleTiming {
                cycle_start_us: 0, // Caller provides absolute timing if needed
                cycle_end_us: dt_us,
                duration_us: dt_us, // dt since last call (actual exec time tracked externally)
                deadline_met: self.state.timing_stats.consecutive_violations == 0,
            },
            degradation,
        }
    }
}
