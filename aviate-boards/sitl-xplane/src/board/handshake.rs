//! Runtime identity gate for the X-Plane bridge and aircraft.

use std::fmt;

use aviate_config::xplane_model::{XPlaneMixerGeometry, XPlaneSimulatorModel};
pub use aviate_config::xplane_runtime::XPlaneRuntimeHandshake;
use aviate_config::xplane_runtime::XPlaneRuntimeHandshakeError as BindingError;
use aviate_core::kernel::config::{ActuatorCurveKind, MixerGeometry};
use aviate_core::DefaultAviateKernel;

/// Runtime identity mismatch.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum RuntimeHandshakeError {
    /// The verified-session document is malformed.
    InvalidBinding(BindingError),
    /// A second runtime identity was supplied.
    AlreadyAccepted,
    /// A reported runtime field does not match the model.
    Mismatch(&'static str),
    /// The HIL sample clock did not increase.
    SampleClockRegression {
        /// Previous accepted simulator timestamp.
        previous_us: u64,
        /// Rejected simulator timestamp.
        next_us: u64,
    },
    /// HIL timestamps do not prove the declared sample rate.
    SampleRateMismatch {
        /// Sample rate declared by the verified model.
        expected_hz: u16,
        /// Mean period measured from the first HIL timestamp batch.
        observed_period_us: u64,
    },
}

impl fmt::Display for RuntimeHandshakeError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidBinding(error) => write!(formatter, "invalid runtime binding: {error}"),
            Self::AlreadyAccepted => {
                formatter.write_str("X-Plane runtime handshake already accepted")
            }
            Self::Mismatch(field) => {
                write!(formatter, "X-Plane runtime field {field} does not match")
            }
            Self::SampleClockRegression {
                previous_us,
                next_us,
            } => write!(
                formatter,
                "X-Plane HIL sample clock regressed from {previous_us} to {next_us}"
            ),
            Self::SampleRateMismatch {
                expected_hz,
                observed_period_us,
            } => write!(
                formatter,
                "X-Plane HIL sample period {observed_period_us} us does not match {expected_hz} Hz"
            ),
        }
    }
}

impl std::error::Error for RuntimeHandshakeError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::InvalidBinding(error) => Some(error),
            Self::AlreadyAccepted
            | Self::Mismatch(_)
            | Self::SampleClockRegression { .. }
            | Self::SampleRateMismatch { .. } => None,
        }
    }
}

const SAMPLE_RATE_EVIDENCE_INTERVALS: u16 = 8;
const SAMPLE_RATE_TOLERANCE_PERCENT: u128 = 5;

pub(crate) struct RuntimeIdentityGate {
    expected: XPlaneSimulatorModel,
    accepted: Option<XPlaneRuntimeHandshake>,
    first_timestamp_us: Option<u64>,
    last_timestamp_us: Option<u64>,
    intervals: u16,
    verified: bool,
    failed: bool,
}

impl RuntimeIdentityGate {
    pub(crate) fn new(expected: XPlaneSimulatorModel) -> Self {
        Self {
            expected,
            accepted: None,
            first_timestamp_us: None,
            last_timestamp_us: None,
            intervals: 0,
            verified: false,
            failed: false,
        }
    }

    pub(crate) fn accept(
        &mut self,
        handshake: XPlaneRuntimeHandshake,
    ) -> Result<(), RuntimeHandshakeError> {
        if self.accepted.is_some() {
            return Err(RuntimeHandshakeError::AlreadyAccepted);
        }
        verify(&self.expected, &handshake)?;
        self.accepted = Some(handshake);
        Ok(())
    }

    pub(crate) fn observe_timestamp(
        &mut self,
        timestamp_us: u64,
    ) -> Result<(), RuntimeHandshakeError> {
        if self.failed || self.accepted.is_none() {
            return Ok(());
        }
        if let Some(previous_us) = self.last_timestamp_us {
            if timestamp_us <= previous_us {
                self.failed = true;
                self.verified = false;
                return Err(RuntimeHandshakeError::SampleClockRegression {
                    previous_us,
                    next_us: timestamp_us,
                });
            }
            self.intervals = self.intervals.wrapping_add(1);
        } else {
            self.first_timestamp_us = Some(timestamp_us);
        }
        self.last_timestamp_us = Some(timestamp_us);
        if !self.verified && self.intervals >= SAMPLE_RATE_EVIDENCE_INTERVALS {
            self.verify_sample_rate()?;
            self.verified = true;
        }
        Ok(())
    }

    fn verify_sample_rate(&mut self) -> Result<(), RuntimeHandshakeError> {
        let Some(first_us) = self.first_timestamp_us else {
            return Ok(());
        };
        let Some(last_us) = self.last_timestamp_us else {
            return Ok(());
        };
        let elapsed_us = last_us.saturating_sub(first_us);
        let expected_hz = self.expected.sample_rate_hz();
        let measured_scaled = u128::from(elapsed_us) * u128::from(expected_hz);
        let expected_scaled = u128::from(self.intervals) * 1_000_000_u128;
        let difference = measured_scaled.abs_diff(expected_scaled);
        if difference * 100 > expected_scaled * SAMPLE_RATE_TOLERANCE_PERCENT {
            self.failed = true;
            let observed_period_us = elapsed_us / u64::from(self.intervals.max(1));
            return Err(RuntimeHandshakeError::SampleRateMismatch {
                expected_hz,
                observed_period_us,
            });
        }
        Ok(())
    }

    pub(crate) fn is_verified(&self) -> bool {
        self.verified && !self.failed
    }

    pub(crate) fn verified(&self) -> Option<&XPlaneRuntimeHandshake> {
        self.is_verified()
            .then_some(self.accepted.as_ref())
            .flatten()
    }
}

fn verify(
    expected: &XPlaneSimulatorModel,
    actual: &XPlaneRuntimeHandshake,
) -> Result<(), RuntimeHandshakeError> {
    actual
        .validate()
        .map_err(RuntimeHandshakeError::InvalidBinding)?;
    check(
        expected.bridge_protocol() == actual.bridge_protocol,
        "bridge_protocol",
    )?;
    check(
        expected.simulator_id() == actual.simulator_id,
        "simulator_id",
    )?;
    check(expected.aircraft_id() == actual.aircraft_id, "aircraft_id")?;
    check(
        expected
            .aircraft_file_digest()
            .eq_ignore_ascii_case(&actual.aircraft_file_digest),
        "aircraft_file_digest",
    )?;
    check(
        expected.sample_rate_hz() == actual.sample_rate_hz,
        "sample_rate_hz",
    )?;
    check(expected.motor_count() == actual.motor_count, "motor_count")?;
    check(expected.lane_order() == actual.lane_order, "lane_order")
}

fn check(condition: bool, field: &'static str) -> Result<(), RuntimeHandshakeError> {
    if condition {
        Ok(())
    } else {
        Err(RuntimeHandshakeError::Mismatch(field))
    }
}

pub(crate) fn validate_kernel_model<C, M>(
    kernel: &DefaultAviateKernel<C, M>,
    model: &XPlaneSimulatorModel,
) -> std::io::Result<()>
where
    C: aviate_core::control::VehicleController,
    M: aviate_core::mixer::Mixer,
{
    let cfg = kernel.cfg();
    let geometry_matches = matches!(
        (model.mixer_geometry(), cfg.mixer_geometry),
        (XPlaneMixerGeometry::QuadX, MixerGeometry::QuadX)
            | (XPlaneMixerGeometry::QuadXX500, MixerGeometry::QuadXX500)
            | (
                XPlaneMixerGeometry::QuadXX500ReversedSpin,
                MixerGeometry::QuadXX500ReversedSpin
            )
    );
    let curve_matches = matches!(
        (model.actuator_curve(), cfg.actuator_curve),
        (
            aviate_config::xplane_model::XPlaneActuatorCurve::Linear,
            ActuatorCurveKind::Linear
        ) | (
            aviate_config::xplane_model::XPlaneActuatorCurve::QuadraticRotor,
            ActuatorCurveKind::QuadraticRotor
        )
    );
    if geometry_matches && curve_matches && model.motor_count() == cfg.mixer_geometry.motor_count()
    {
        Ok(())
    } else {
        Err(std::io::Error::new(
            std::io::ErrorKind::InvalidInput,
            "kernel actuator layout does not match the X-Plane model",
        ))
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::expect_used)]

    use super::*;

    const MODEL: &str = include_str!("../../../../presets/alia250-xplane.toml");

    fn handshake(model: &XPlaneSimulatorModel) -> XPlaneRuntimeHandshake {
        XPlaneRuntimeHandshake {
            schema_version: 1,
            verifier_id: "pilotage-xplane-trial-v1".to_owned(),
            session_binding_digest: "f".repeat(64),
            bridge_endpoint: "127.0.0.1:4560".to_owned(),
            bridge_protocol: model.bridge_protocol(),
            bridge_build_digest: "a".repeat(64),
            bridge_config_digest: "c".repeat(64),
            simulator_id: model.simulator_id().to_owned(),
            aircraft_id: model.aircraft_id().to_owned(),
            aircraft_file_digest: model.aircraft_file_digest().to_owned(),
            sample_rate_hz: model.sample_rate_hz(),
            motor_count: model.motor_count(),
            lane_order: model.lane_order(),
        }
    }

    #[test]
    fn exact_runtime_identity_needs_hil_clock_evidence() {
        let model = XPlaneSimulatorModel::from_toml_str(MODEL).expect("valid model");
        let mut gate = RuntimeIdentityGate::new(model.clone());
        gate.accept(handshake(&model)).expect("matching handshake");
        assert!(!gate.is_verified());
        // The period comes from the rate the MODEL declares, not from a
        // number typed here. The gate exists to refuse a clock that disagrees
        // with the declaration, so a fixture with its own period tests the
        // gate against a rate nobody declared — and pins the declaration in
        // place, since changing it then fails a test that has nothing to say
        // about the change.
        let period_us = 1_000_000 / u64::from(model.sample_rate_hz());
        for index in 0..=SAMPLE_RATE_EVIDENCE_INTERVALS {
            gate.observe_timestamp(u64::from(index) * period_us)
                .expect("valid sample clock");
        }
        assert!(gate.is_verified());
    }

    #[test]
    fn declared_rate_without_matching_hil_timestamps_stays_closed() {
        let model = XPlaneSimulatorModel::from_toml_str(MODEL).expect("valid model");
        let mut gate = RuntimeIdentityGate::new(model.clone());
        gate.accept(handshake(&model)).expect("matching handshake");
        let mut failure = None;
        for index in 0..=SAMPLE_RATE_EVIDENCE_INTERVALS {
            failure = gate
                .observe_timestamp(u64::from(index) * 20_000)
                .err()
                .or(failure);
        }
        assert!(matches!(
            failure,
            Some(RuntimeHandshakeError::SampleRateMismatch { .. })
        ));
        assert!(!gate.is_verified());
    }

    #[test]
    fn each_plant_identity_mismatch_keeps_the_gate_closed() {
        let model = XPlaneSimulatorModel::from_toml_str(MODEL).expect("valid model");
        let mutations: [fn(&mut XPlaneRuntimeHandshake); 5] = [
            |value| value.aircraft_file_digest = "b".repeat(64),
            |value| value.aircraft_id.push('b'),
            |value| value.simulator_id.push('b'),
            |value| value.sample_rate_hz = value.sample_rate_hz.saturating_add(1),
            |value| value.lane_order.swap(0, 1),
        ];
        for mutate in mutations {
            let mut gate = RuntimeIdentityGate::new(model.clone());
            let mut actual = handshake(&model);
            mutate(&mut actual);
            assert!(gate.accept(actual).is_err());
            assert!(!gate.is_verified());
        }
    }
}
