//! Pilotage condition schema fields used by the Aviate executor.

mod validation;

use serde::{Deserialize, Serialize};

use super::super::{
    ActuatorPerturbation, CommandHoldPerturbation, PerturbationConfig, PerturbationIdentity,
    SensorLane, SensorNoise,
};
use super::{ArtifactError, PerturbationCapability};

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub(super) struct ConditionSet {
    pub(super) schema_version: u16,
    pub(super) id: String,
    pub(super) revision: u32,
    pub(super) seed: u64,
    pub(super) wind: WindCondition,
    pub(super) timing: TimingCondition,
    pub(super) sensor: SensorCondition,
    pub(super) actuator: ActuatorCondition,
    pub(super) controller_initialization: ControllerInitializationCondition,
    pub(super) plant: PlantCondition,
}

impl ConditionSet {
    pub(super) fn validate(&self) -> Result<(), ArtifactError> {
        validation::condition(self)
    }

    pub(super) fn required_capabilities(&self) -> Vec<PerturbationCapability> {
        let mut required = Vec::new();
        if matches!(self.sensor, SensorCondition::BoundedNoise { .. }) {
            required.push(PerturbationCapability::SensorPerturbation);
        }
        if self.actuator.authority_scale_basis_points != 10_000 {
            required.push(PerturbationCapability::ActuatorAuthority);
        }
        if matches!(
            self.actuator.command_loss,
            CommandLossPolicy::SeededZeroOrderHold { .. }
        ) {
            required.push(PerturbationCapability::CommandHold);
        }
        if self.hover_scale_basis_points() != 10_000 {
            required.push(PerturbationCapability::HoverTrimUncertainty);
        }
        required
    }

    pub(super) fn perturbation_config(
        &self,
        condition_digest: [u8; 32],
        run_seed: u64,
    ) -> PerturbationConfig {
        let sensor_noise = match &self.sensor {
            SensorCondition::None {} => Vec::new(),
            SensorCondition::BoundedNoise { lanes } => lanes
                .iter()
                .copied()
                .map(SensorNoiseLane::request)
                .collect(),
        };
        let command_hold = match self.actuator.command_loss {
            CommandLossPolicy::None {} => None,
            CommandLossPolicy::SeededZeroOrderHold {
                fraction_basis_points,
                decision_interval_samples,
            } => Some(CommandHoldPerturbation {
                fraction_basis_points,
                decision_interval_samples,
            }),
        };
        PerturbationConfig {
            identity: PerturbationIdentity {
                condition_digest,
                run_seed,
            },
            sensor_noise,
            actuator: ActuatorPerturbation {
                authority_scale_basis_points: self.actuator.authority_scale_basis_points,
                command_hold,
            },
        }
    }

    pub(super) const fn hover_scale_basis_points(&self) -> u16 {
        match self.controller_initialization.hover_thrust_force {
            HoverThrustForceInitialization::ScaleBaseline { scale_basis_points } => {
                scale_basis_points
            }
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub(super) struct WindCondition {
    pub(super) steady: HorizontalWind,
    pub(super) gusts: Vec<GustEvent>,
    pub(super) turbulence: TurbulenceModel,
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub(super) struct HorizontalWind {
    pub(super) speed_mps: f64,
    pub(super) direction_deg: f64,
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub(super) struct GustEvent {
    pub(super) start_ns: u64,
    pub(super) rise_ns: u64,
    pub(super) hold_ns: u64,
    pub(super) fall_ns: u64,
    pub(super) speed_mps: f64,
    pub(super) direction_deg: f64,
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub(super) enum TurbulenceModel {
    None,
    BandLimitedNoise {
        amplitude_mps: f64,
        knot_interval_ns: u64,
    },
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub(super) struct TimingCondition {
    pub(super) estimate_delay_ns: u64,
    pub(super) update_jitter: DelayJitter,
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub(super) enum DelayJitter {
    None,
    SampleAndHold {
        maximum_delay_ns: u64,
        interval_ns: u64,
    },
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub(super) enum SensorCondition {
    None {},
    BoundedNoise { lanes: Vec<SensorNoiseLane> },
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub(super) enum SensorAxis {
    X,
    Y,
    Z,
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
#[serde(tag = "sensor", rename_all = "snake_case", deny_unknown_fields)]
pub(super) enum SensorNoiseLane {
    Accelerometer {
        axis: SensorAxis,
        peak_amplitude_mps2: f64,
        update_interval_samples: u32,
    },
    Gyroscope {
        axis: SensorAxis,
        peak_amplitude_rad_s: f64,
        update_interval_samples: u32,
    },
    Magnetometer {
        axis: SensorAxis,
        peak_amplitude_gauss: f64,
        update_interval_samples: u32,
    },
    AbsolutePressure {
        peak_amplitude_hpa: f64,
        update_interval_samples: u32,
    },
    DifferentialPressure {
        peak_amplitude_hpa: f64,
        update_interval_samples: u32,
    },
    PressureAltitude {
        peak_amplitude_m: f64,
        update_interval_samples: u32,
    },
}

impl SensorNoiseLane {
    fn request(self) -> SensorNoise {
        let (lane, peak_amplitude, update_interval_samples) = match self {
            Self::Accelerometer {
                axis,
                peak_amplitude_mps2,
                update_interval_samples,
            } => (
                vector_lane(
                    axis,
                    SensorLane::AccelerometerX,
                    SensorLane::AccelerometerY,
                    SensorLane::AccelerometerZ,
                ),
                peak_amplitude_mps2,
                update_interval_samples,
            ),
            Self::Gyroscope {
                axis,
                peak_amplitude_rad_s,
                update_interval_samples,
            } => (
                vector_lane(
                    axis,
                    SensorLane::GyroscopeX,
                    SensorLane::GyroscopeY,
                    SensorLane::GyroscopeZ,
                ),
                peak_amplitude_rad_s,
                update_interval_samples,
            ),
            Self::Magnetometer {
                axis,
                peak_amplitude_gauss,
                update_interval_samples,
            } => (
                vector_lane(
                    axis,
                    SensorLane::MagnetometerX,
                    SensorLane::MagnetometerY,
                    SensorLane::MagnetometerZ,
                ),
                peak_amplitude_gauss * 100.0,
                update_interval_samples,
            ),
            Self::AbsolutePressure {
                peak_amplitude_hpa,
                update_interval_samples,
            } => (
                SensorLane::AbsolutePressure,
                peak_amplitude_hpa * 100.0,
                update_interval_samples,
            ),
            Self::DifferentialPressure {
                peak_amplitude_hpa,
                update_interval_samples,
            } => (
                SensorLane::DifferentialPressure,
                peak_amplitude_hpa * 100.0,
                update_interval_samples,
            ),
            Self::PressureAltitude {
                peak_amplitude_m,
                update_interval_samples,
            } => (
                SensorLane::PressureAltitude,
                peak_amplitude_m,
                update_interval_samples,
            ),
        };
        SensorNoise {
            lane,
            peak_amplitude: peak_amplitude as f32,
            update_interval_samples,
        }
    }
}

fn vector_lane(axis: SensorAxis, x: SensorLane, y: SensorLane, z: SensorLane) -> SensorLane {
    match axis {
        SensorAxis::X => x,
        SensorAxis::Y => y,
        SensorAxis::Z => z,
    }
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub(super) struct ActuatorCondition {
    pub(super) authority_scale_basis_points: u16,
    pub(super) command_loss: CommandLossPolicy,
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub(super) enum CommandLossPolicy {
    None {},
    SeededZeroOrderHold {
        fraction_basis_points: u16,
        decision_interval_samples: u32,
    },
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub(super) struct ControllerInitializationCondition {
    pub(super) hover_thrust_force: HoverThrustForceInitialization,
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub(super) enum HoverThrustForceInitialization {
    ScaleBaseline { scale_basis_points: u16 },
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub(super) struct PlantCondition {
    pub(super) payload_mass_delta_kg: f64,
    pub(super) longitudinal_cg_offset_m: f64,
    pub(super) lateral_cg_offset_m: f64,
    pub(super) hover_thrust_expectation: HoverThrustExpectation,
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub(super) enum HoverThrustExpectation {
    MeasuredWeightRatio,
    ExplicitRatio { ratio: f64, maximum_error: f64 },
}
