//! The `update()` method on `AviateKernelImpl`.
//!
//! Isolated in its own file so the per-cycle control loop stays within
//! the 500-line per-.rs limit and is easy to locate at review time.

use crate::checks::DegradationReason;
use crate::control::{AuthorityProfile, Command, ControlLawV1, ControlMode, VehicleController};
use crate::ekf::Estimator;
use crate::fault::FaultFlags;
use crate::kernel::AviateKernelImpl;
use crate::kernel_types::{
    ChannelHealthV1, ChannelId, ChannelStatus, ConfigTransitionState, CrossChannelData,
    CycleTiming, EnvelopeMargin, UpdateResult, TIMING_VIOLATION_THRESHOLD,
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
                    confidence: self.state.estimator.get_estimate().quality,
                    envelope_margin: EnvelopeMargin::default(),
                    sequence: command.sequence,
                    protection: Default::default(),
                    sanitize_report: SanitizeReport::default(),
                },
                estimate: self.state.estimator.get_estimate(),
                timing: CycleTiming::default(),
                degradation: None,
            };
        }
        // COV:EXCL_STOP

        // 0. Update sensor fault flags (always, regardless of armed state)
        //    This allows continuous monitoring of sensor health.
        self.update_sensor_faults(sensors);

        // 1. Safety Gate: If not armed, force safe output immediately
        if !self.state.init_state.allows_active_control() {
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
                    confidence: self.state.estimator.get_estimate().quality,
                    envelope_margin: EnvelopeMargin::default(),
                    sequence: command.sequence,
                    protection: Default::default(),
                    sanitize_report: SanitizeReport::default(),
                },
                estimate: self.state.estimator.get_estimate(),
                timing: CycleTiming::default(),
                degradation: None,
            };
        }

        // 2. Check for critical faults (if we got here, we're armed)
        if self.check_critical_faults() {
            // If critical fault, force Backup/Frozen behavior
            return UpdateResult {
                actuator: ActuatorCmd {
                    outputs: self.cfg.safe_output,
                    active_mask: 0,
                    sequence: command.sequence,
                    timestamp,
                    fallback_mask: 0,
                    sanitized: true,
                },
                status: ChannelStatus::default(), // TODO: Populate with fault info
                estimate: self.state.estimator.get_estimate(),
                timing: CycleTiming::default(),
                degradation: None,
            };
        }

        // 3. EKF Update (predict and update). Pipeline carries the
        //    algorithm identity (`&self`); KernelState owns the filter
        //    state (`&mut self.state.estimator`). The disjoint-borrow
        //    rule lets us read the algorithm and write the state in
        //    the same call.
        let primary_imu = &sensors.imus[0];
        if primary_imu.valid && primary_imu.health == crate::sensor::SensorHealth::Good {
            self.pipeline.estimator.predict(
                &mut self.state.estimator,
                &primary_imu.value,
                time.dt_sec.0,
            );
        }

        // Apply sensor overrides from command
        if let Some(overrides) = &command.sensor_overrides {
            if let Some(gnss_health) = overrides.gnss_force_state {
                let mut primary_gnss_reading = sensors.gnss[0];
                primary_gnss_reading.health = match gnss_health {
                    crate::sensor::GnssHealth::Good => crate::sensor::SensorHealth::Good,
                    crate::sensor::GnssHealth::Suspect => crate::sensor::SensorHealth::Degraded,
                    crate::sensor::GnssHealth::Lost => crate::sensor::SensorHealth::Failed,
                };
                self.pipeline
                    .estimator
                    .update_gnss(&mut self.state.estimator, &primary_gnss_reading);
            }
        } else {
            // Normal sensor updates
            let primary_gnss = &sensors.gnss[0];
            if primary_gnss.valid && primary_gnss.health == crate::sensor::SensorHealth::Good {
                self.pipeline
                    .estimator
                    .update_gnss(&mut self.state.estimator, primary_gnss);
            }
        }

        let primary_baro = &sensors.baros[0];
        if primary_baro.valid && primary_baro.health == crate::sensor::SensorHealth::Good {
            self.pipeline
                .estimator
                .update_baro(&mut self.state.estimator, primary_baro);
        }

        let primary_mag = &sensors.mags[0];
        if primary_mag.valid && primary_mag.health == crate::sensor::SensorHealth::Good {
            self.pipeline
                .estimator
                .update_mag(&mut self.state.estimator, primary_mag);
        }

        // Get updated estimate
        let state = self.state.estimator.get_estimate();

        // 3. Update in-flight checks
        self.state.checks.in_flight.update_from_state(&state);
        self.state.checks.in_flight.update_from_sensors(sensors);
        // Spec §12: Command staleness gate. The caller supplies
        // `command_age_ms` measured against its own timebase
        // (typically the time elapsed since the last RC/GCS frame
        // arrived); the in-flight check clears COMMAND_RECENT once
        // the age meets or exceeds `cfg.command_timeout_ms`.
        self.state
            .checks
            .in_flight
            .update_command_status(command_age_ms, self.cfg.command_timeout_ms);

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

        // 6. Control Step
        // If Backup, we might want to use safe outputs or a simplified controller.
        // For now, if Backup, force safe output (as per spec "Non-Armed states → Backup → safe").
        // But Backup during flight might mean "Last-ditch stability".
        // The spec says "Last-ditch stability only".
        // For this minimal impl, if Backup, we output safe_output (effectively shutting down/idle).
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
            let axis_cmd = self.pipeline.controller.step(
                &state,
                &constrained_cmd,
                self.state.mode,
                &self.cfg.limits,
            );
            self.pipeline.mixer.mix(&axis_cmd)
        };

        // 7. Update actuator state
        self.state
            .actuator_state
            .update_commanded(&actuator_cmd, timestamp);

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

        // 9. Construct Result
        UpdateResult {
            actuator: actuator_cmd,
            status: ChannelStatus {
                mode: command.mode,
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
