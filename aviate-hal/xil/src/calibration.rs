//! Calibration action primitives (SIM / NOT FOR FLIGHT).
//!
//! Three typed simulator-only actions for identification experiments:
//! lane injection with a per-axis waveform, test-stand control, and
//! hold-current-attitude. Each action has one confirm-or-error
//! receipt.
//!
//! The module owns the vocabulary only. Board implementations stay in
//! `aviate-boards`. An execution path reuses the board functions:
//! `XPlaneBoard::set_lane_injection` applies a lane injection, and the
//! stand API drives the test stand. A receipt is complete when the
//! simulator confirms the action, not when the command is sent.

use std::time::Duration;

/// One body axis that a lane-injection excitation drives.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InjectionAxis {
    /// Roll axis.
    Roll,
    /// Pitch axis.
    Pitch,
    /// Yaw axis.
    Yaw,
}

/// The per-axis waveform of a lane injection.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ExcitationWaveform {
    /// A sine at `frequency_rad_s` with a force-domain `amplitude`.
    Sine {
        amplitude: f32,
        frequency_rad_s: f32,
    },
}

impl ExcitationWaveform {
    /// A stable digest of the waveform specification.
    ///
    /// The digest covers a domain prefix, the waveform kind, and every
    /// parameter, in a canonical little-endian encoding. Two windows
    /// that applied the same waveform carry the same digest.
    #[must_use]
    pub fn digest(&self) -> [u8; 32] {
        use sha2::Digest as _;
        let mut hasher = sha2::Sha256::new();
        hasher.update(b"aviate.excitation-waveform.v1\0");
        match self {
            Self::Sine {
                amplitude,
                frequency_rad_s,
            } => {
                hasher.update([1_u8]);
                hasher.update(amplitude.to_le_bytes());
                hasher.update(frequency_rad_s.to_le_bytes());
            }
        }
        hasher.finalize().into()
    }
}

/// A lane injection with a per-axis waveform.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LaneInjection {
    /// The body axis that the waveform drives.
    pub axis: InjectionAxis,
    /// The per-axis waveform.
    pub waveform: ExcitationWaveform,
    /// The excitation window length, in simulator time.
    pub window: Duration,
}

impl LaneInjection {
    /// Validate the injection parameters.
    ///
    /// A zero, negative, or non-finite amplitude or frequency, or a
    /// zero-length window, makes the injection a silent no-op. The
    /// validator refuses it.
    ///
    /// # Errors
    ///
    /// Returns [`CalibrationError::InvalidParameter`] for the first
    /// invalid parameter.
    pub fn validate(&self) -> Result<(), CalibrationError> {
        match self.waveform {
            ExcitationWaveform::Sine {
                amplitude,
                frequency_rad_s,
            } => {
                if !amplitude.is_finite() || amplitude <= 0.0 {
                    return Err(CalibrationError::InvalidParameter {
                        action: CalibrationActionKind::LaneInjection,
                        field: "amplitude",
                        detail: format!(
                            "expected a finite value greater than zero, got {amplitude}"
                        ),
                    });
                }
                if !frequency_rad_s.is_finite() || frequency_rad_s <= 0.0 {
                    return Err(CalibrationError::InvalidParameter {
                        action: CalibrationActionKind::LaneInjection,
                        field: "frequency_rad_s",
                        detail: format!(
                            "expected a finite value greater than zero, got {frequency_rad_s}"
                        ),
                    });
                }
            }
        }
        if self.window.is_zero() {
            return Err(CalibrationError::InvalidParameter {
                action: CalibrationActionKind::LaneInjection,
                field: "window",
                detail: "expected a window longer than zero".to_owned(),
            });
        }
        Ok(())
    }
}

/// One test-stand directive. Mirrors the stand API: engage, pin, zero
/// rates, release.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TestStandCommand {
    /// Engage the stand at the current altitude.
    Engage,
    /// Pin translation: zero the linear velocity, hold the altitude.
    Pin,
    /// Zero the body rotation rates.
    ZeroRates,
    /// Release the stand.
    Release,
}

/// The three typed calibration actions. Each action is simulator-only.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CalibrationAction {
    /// Inject a per-axis waveform on the actuator lanes.
    LaneInjection(LaneInjection),
    /// Drive the virtual test stand.
    TestStand(TestStandCommand),
    /// Freeze the current attitude estimate as the attitude reference.
    /// `Action::AttitudeTarget` takes a literal quaternion and cannot
    /// express this.
    HoldCurrentAttitude,
}

/// The kind of a calibration action, for typed errors.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CalibrationActionKind {
    /// Lane injection.
    LaneInjection,
    /// Test-stand control.
    TestStand,
    /// Hold-current-attitude.
    HoldCurrentAttitude,
}

/// The kind of target that a sequencer addresses.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TargetKind {
    /// A simulator target.
    Simulator,
    /// A non-simulator target (hardware).
    Hardware,
}

impl CalibrationAction {
    /// Every calibration action is simulator-only (SIM / NOT FOR
    /// FLIGHT).
    #[must_use]
    pub const fn simulator_only(&self) -> bool {
        true
    }

    /// The kind of this action.
    #[must_use]
    pub const fn kind(&self) -> CalibrationActionKind {
        match self {
            Self::LaneInjection(_) => CalibrationActionKind::LaneInjection,
            Self::TestStand(_) => CalibrationActionKind::TestStand,
            Self::HoldCurrentAttitude => CalibrationActionKind::HoldCurrentAttitude,
        }
    }

    /// Admit this action on `target`.
    ///
    /// # Errors
    ///
    /// A non-simulator target refuses the action with
    /// [`CalibrationError::SimulatorOnly`].
    pub fn admit(&self, target: TargetKind) -> Result<(), CalibrationError> {
        match target {
            TargetKind::Simulator => Ok(()),
            TargetKind::Hardware => Err(CalibrationError::SimulatorOnly {
                action: self.kind(),
                target,
            }),
        }
    }
}

/// A typed calibration-action failure.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum CalibrationError {
    /// A non-simulator target refuses a simulator-only action.
    #[error("{action:?} is simulator-only; the {target:?} target refuses it")]
    SimulatorOnly {
        /// The refused action.
        action: CalibrationActionKind,
        /// The refusing target.
        target: TargetKind,
    },
    /// An action parameter is not valid.
    #[error("{action:?} parameter {field} is not valid: {detail}")]
    InvalidParameter {
        /// The action with the invalid parameter.
        action: CalibrationActionKind,
        /// The invalid parameter.
        field: &'static str,
        /// Why the parameter is not valid.
        detail: String,
    },
    /// The simulator readback does not confirm the expected value.
    #[error(
        "{action:?} readback of {field} does not confirm: expected {expected}, actual {actual}"
    )]
    Readback {
        /// The action that the readback confirms.
        action: CalibrationActionKind,
        /// The readback field.
        field: &'static str,
        /// The expected value.
        expected: f32,
        /// The actual value.
        actual: f32,
    },
}

/// The excitation window that a lane injection applied, in simulator
/// time.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ExcitationWindowReceipt {
    /// The window start, in simulator time.
    pub window_start: Duration,
    /// The window end, in simulator time.
    pub window_end: Duration,
    /// The digest of the applied waveform.
    pub waveform_digest: [u8; 32],
}

/// The receipt for one lane injection. Follows the stand readback
/// pattern: the receipt carries the expected value and the actual
/// value.
#[derive(Debug, Clone, PartialEq)]
pub struct LaneInjectionReceipt {
    /// The per-lane injection that the action requested.
    pub expected_lanes: [f32; 4],
    /// The per-lane injection that the simulator confirms applied.
    pub actual_lanes: [f32; 4],
    /// The excitation window, in simulator time.
    pub window: ExcitationWindowReceipt,
    /// The simulator time of the confirmation.
    pub simulation_time: Duration,
}

impl LaneInjectionReceipt {
    /// Confirm that the simulator applied the expected lanes.
    ///
    /// # Errors
    ///
    /// Returns [`CalibrationError::Readback`] when any actual lane
    /// differs from its expected lane by more than `tolerance`.
    pub fn confirm(
        expected_lanes: [f32; 4],
        actual_lanes: [f32; 4],
        window: ExcitationWindowReceipt,
        simulation_time: Duration,
        tolerance: f32,
    ) -> Result<Self, CalibrationError> {
        for (lane, (expected, actual)) in expected_lanes.iter().zip(actual_lanes.iter()).enumerate()
        {
            const LANE_FIELDS: [&str; 4] = ["lane0", "lane1", "lane2", "lane3"];
            confirm_readback(
                CalibrationActionKind::LaneInjection,
                LANE_FIELDS[lane],
                *expected,
                *actual,
                tolerance,
            )?;
        }
        Ok(Self {
            expected_lanes,
            actual_lanes,
            window,
            simulation_time,
        })
    }
}

/// The receipt for one test-stand directive. Follows the stand
/// readback pattern: the receipt carries the expected value and the
/// actual value.
#[derive(Debug, Clone, PartialEq)]
pub struct TestStandReceipt {
    /// The directive that the action requested.
    pub command: TestStandCommand,
    /// The expected stand value (for example the held altitude).
    pub expected: f32,
    /// The actual stand value that the simulator confirms.
    pub actual: f32,
    /// The simulator time of the confirmation.
    pub simulation_time: Duration,
}

impl TestStandReceipt {
    /// Confirm that the simulator applied the stand directive.
    ///
    /// # Errors
    ///
    /// Returns [`CalibrationError::Readback`] when `actual` differs
    /// from `expected` by more than `tolerance`.
    pub fn confirm(
        command: TestStandCommand,
        field: &'static str,
        expected: f32,
        actual: f32,
        simulation_time: Duration,
        tolerance: f32,
    ) -> Result<Self, CalibrationError> {
        confirm_readback(
            CalibrationActionKind::TestStand,
            field,
            expected,
            actual,
            tolerance,
        )?;
        Ok(Self {
            command,
            expected,
            actual,
            simulation_time,
        })
    }
}

/// The receipt for one hold-current-attitude action. Follows the stand
/// readback pattern: the receipt carries the expected value and the
/// actual value.
#[derive(Debug, Clone, PartialEq)]
pub struct AttitudeHoldReceipt {
    /// The attitude estimate that the action captured at the window
    /// start, quaternion `[w, x, y, z]`.
    pub expected: [f32; 4],
    /// The attitude reference that the simulator confirms held.
    pub actual: [f32; 4],
    /// The simulator time of the confirmation.
    pub simulation_time: Duration,
}

impl AttitudeHoldReceipt {
    /// Confirm that the simulator holds the captured attitude
    /// reference.
    ///
    /// # Errors
    ///
    /// Returns [`CalibrationError::Readback`] when any actual
    /// quaternion component differs from its expected component by
    /// more than `tolerance`.
    pub fn confirm(
        expected: [f32; 4],
        actual: [f32; 4],
        simulation_time: Duration,
        tolerance: f32,
    ) -> Result<Self, CalibrationError> {
        for (expected_component, actual_component) in expected.iter().zip(actual.iter()) {
            confirm_readback(
                CalibrationActionKind::HoldCurrentAttitude,
                "attitude",
                *expected_component,
                *actual_component,
                tolerance,
            )?;
        }
        Ok(Self {
            expected,
            actual,
            simulation_time,
        })
    }
}

/// One receipt for one calibration action.
#[derive(Debug, Clone, PartialEq)]
pub enum CalibrationReceipt {
    /// The receipt for a lane injection.
    LaneInjection(LaneInjectionReceipt),
    /// The receipt for a test-stand directive.
    TestStand(TestStandReceipt),
    /// The receipt for a hold-current-attitude action.
    AttitudeHold(AttitudeHoldReceipt),
}

impl CalibrationReceipt {
    /// The simulator time of the confirmation.
    #[must_use]
    pub const fn simulation_time(&self) -> Duration {
        match self {
            Self::LaneInjection(receipt) => receipt.simulation_time,
            Self::TestStand(receipt) => receipt.simulation_time,
            Self::AttitudeHold(receipt) => receipt.simulation_time,
        }
    }
}

/// One readback check, shared by the three receipts. The same shape
/// as the stand API's readback verification.
fn confirm_readback(
    action: CalibrationActionKind,
    field: &'static str,
    expected: f32,
    actual: f32,
    tolerance: f32,
) -> Result<(), CalibrationError> {
    if (expected - actual).abs() <= tolerance {
        Ok(())
    } else {
        Err(CalibrationError::Readback {
            action,
            field,
            expected,
            actual,
        })
    }
}

#[cfg(test)]
mod tests;
