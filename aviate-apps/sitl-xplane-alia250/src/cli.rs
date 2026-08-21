//! Fail-closed command-line contract.

use std::path::PathBuf;
use std::time::Duration;

use aviate_app_sitl_xplane_alia250_kernel::RunPurpose;

mod error;
mod runtime_binding;

pub(super) use error::CliError;
pub(super) use runtime_binding::ClaimedRuntimeHandshake;

pub(super) struct Cli {
    pub(super) bridge: std::net::SocketAddr,
    pub(super) auto_arm: Option<Duration>,
    pub(super) run_manifest: Option<PathBuf>,
    pub(super) plant_output: Option<PathBuf>,
    pub(super) trace_output: Option<PathBuf>,
    pub(super) tuning_trace_endpoint: Option<std::net::SocketAddr>,
    pub(super) experiment: Option<RunPurpose>,
    runtime_handshake: Option<PathBuf>,
    candidate: Option<PathBuf>,
    plant_artifact: Option<PathBuf>,
    bridge_set: bool,
}

pub(super) struct CalibrationInputs {
    pub(super) candidate: String,
    pub(super) plant_artifact: String,
}

impl Cli {
    pub(super) fn parse<I>(args: I) -> Result<Self, CliError>
    where
        I: IntoIterator<Item = String>,
    {
        let mut values = args.into_iter();
        let mut parsed = Self {
            bridge: std::net::SocketAddr::from(([127, 0, 0, 1], 4560)),
            auto_arm: None,
            run_manifest: None,
            plant_output: None,
            trace_output: None,
            tuning_trace_endpoint: None,
            experiment: None,
            runtime_handshake: None,
            candidate: None,
            plant_artifact: None,
            bridge_set: false,
        };
        while let Some(flag) = values.next() {
            match flag.as_str() {
                "--bridge" => {
                    let value = next_value(&mut values, "--bridge")?;
                    set_once_socket(
                        &mut parsed.bridge,
                        &mut parsed.bridge_set,
                        value,
                        "--bridge",
                    )?;
                }
                "--auto-arm" => {
                    let value = next_value(&mut values, "--auto-arm")?;
                    let seconds =
                        value
                            .parse::<u64>()
                            .map_err(|source| CliError::InvalidDuration {
                                value: value.clone(),
                                source,
                            })?;
                    set_once(
                        &mut parsed.auto_arm,
                        Duration::from_secs(seconds),
                        "--auto-arm",
                    )?;
                }
                "--run-manifest" => set_once(
                    &mut parsed.run_manifest,
                    PathBuf::from(next_value(&mut values, "--run-manifest")?),
                    "--run-manifest",
                )?,
                "--plant-output" => set_once(
                    &mut parsed.plant_output,
                    PathBuf::from(next_value(&mut values, "--plant-output")?),
                    "--plant-output",
                )?,
                "--trace-output" => set_once(
                    &mut parsed.trace_output,
                    PathBuf::from(next_value(&mut values, "--trace-output")?),
                    "--trace-output",
                )?,
                "--tuning-trace-endpoint" => {
                    let value = next_value(&mut values, "--tuning-trace-endpoint")?;
                    let endpoint = parse_socket(value, "--tuning-trace-endpoint")?;
                    set_once(
                        &mut parsed.tuning_trace_endpoint,
                        endpoint,
                        "--tuning-trace-endpoint",
                    )?;
                }
                "--candidate" => set_once(
                    &mut parsed.candidate,
                    PathBuf::from(next_value(&mut values, "--candidate")?),
                    "--candidate",
                )?,
                "--plant-artifact" => set_once(
                    &mut parsed.plant_artifact,
                    PathBuf::from(next_value(&mut values, "--plant-artifact")?),
                    "--plant-artifact",
                )?,
                "--runtime-handshake" => set_once(
                    &mut parsed.runtime_handshake,
                    PathBuf::from(next_value(&mut values, "--runtime-handshake")?),
                    "--runtime-handshake",
                )?,
                "--identify" => parsed.select_experiment(RunPurpose::Identify)?,
                "--sweep" => parsed.select_experiment(RunPurpose::Sweep)?,
                "--yaw-sign" => parsed.select_experiment(RunPurpose::YawSign)?,
                _ => return Err(CliError::UnknownArgument(flag)),
            }
        }
        parsed.validate()?;
        Ok(parsed)
    }

    pub(super) fn calibration_inputs(&self) -> Result<Option<CalibrationInputs>, CliError> {
        let (candidate, plant) = match (&self.candidate, &self.plant_artifact) {
            (None, None) => return Ok(None),
            (Some(_), None) => {
                return Err(CliError::InvalidCombination(
                    "--candidate requires --plant-artifact",
                ));
            }
            (None, Some(_)) => {
                return Err(CliError::InvalidCombination(
                    "--plant-artifact requires --candidate",
                ));
            }
            (Some(candidate), Some(plant)) => (candidate, plant),
        };
        let candidate_text = read_artifact(candidate, "calibration candidate")?;
        let plant_text = read_artifact(plant, "plant artifact")?;
        Ok(Some(CalibrationInputs {
            candidate: candidate_text,
            plant_artifact: plant_text,
        }))
    }

    pub(super) fn claim_runtime_handshake(&self) -> Result<ClaimedRuntimeHandshake, CliError> {
        let path = self
            .runtime_handshake
            .as_deref()
            .ok_or(CliError::InvalidCombination(
                "--runtime-handshake is required",
            ))?;
        runtime_binding::claim(path, self.bridge)
    }

    fn select_experiment(&mut self, purpose: RunPurpose) -> Result<(), CliError> {
        if self.experiment.is_some() {
            return Err(CliError::InvalidCombination(
                "select only one identification experiment",
            ));
        }
        self.experiment = Some(purpose);
        Ok(())
    }

    fn validate(&self) -> Result<(), CliError> {
        if self.runtime_handshake.is_none() {
            return Err(CliError::InvalidCombination(
                "--runtime-handshake is required",
            ));
        }
        let has_candidate = self.candidate.is_some() || self.plant_artifact.is_some();
        if let Some(endpoint) = self.tuning_trace_endpoint {
            if !endpoint.ip().is_loopback() {
                return Err(CliError::NonLoopbackTrace(endpoint));
            }
        }
        if (has_candidate || self.experiment.is_some()) && self.tuning_trace_endpoint.is_none() {
            return Err(CliError::InvalidCombination(
                "candidate and experiment runs require --tuning-trace-endpoint",
            ));
        }
        if (has_candidate || self.experiment.is_some()) && self.run_manifest.is_none() {
            return Err(CliError::InvalidCombination(
                "candidate and experiment runs require --run-manifest",
            ));
        }
        if has_candidate && self.experiment.is_some() {
            return Err(CliError::InvalidCombination(
                "a candidate cannot be combined with an identification experiment",
            ));
        }
        if self.plant_output.is_some() && self.experiment != Some(RunPurpose::Identify) {
            return Err(CliError::InvalidCombination(
                "--plant-output requires --identify",
            ));
        }
        if self.trace_output.is_some() && self.experiment != Some(RunPurpose::Identify) {
            return Err(CliError::InvalidCombination(
                "--trace-output requires --identify",
            ));
        }
        if self.experiment == Some(RunPurpose::Identify)
            && (self.run_manifest.is_none()
                || self.plant_output.is_none()
                || self.trace_output.is_none())
        {
            return Err(CliError::InvalidCombination(
                "--identify requires --run-manifest, --plant-output, and --trace-output",
            ));
        }
        if self.candidate.is_some() != self.plant_artifact.is_some() {
            return Err(CliError::InvalidCombination(
                "--candidate and --plant-artifact must be supplied together",
            ));
        }
        Ok(())
    }
}

fn next_value<I>(values: &mut I, flag: &'static str) -> Result<String, CliError>
where
    I: Iterator<Item = String>,
{
    let value = values.next().ok_or(CliError::MissingValue(flag))?;
    if value.starts_with("--") {
        return Err(CliError::MissingValue(flag));
    }
    Ok(value)
}

fn set_once<T>(target: &mut Option<T>, value: T, flag: &'static str) -> Result<(), CliError> {
    if target.is_some() {
        return Err(CliError::Duplicate(flag));
    }
    *target = Some(value);
    Ok(())
}

fn set_once_socket(
    target: &mut std::net::SocketAddr,
    seen: &mut bool,
    value: String,
    flag: &'static str,
) -> Result<(), CliError> {
    if *seen {
        return Err(CliError::Duplicate(flag));
    }
    *target = value.parse().map_err(|source| CliError::InvalidSocket {
        flag,
        value: value.clone(),
        source,
    })?;
    *seen = true;
    Ok(())
}

fn parse_socket(value: String, flag: &'static str) -> Result<std::net::SocketAddr, CliError> {
    value.parse().map_err(|source| CliError::InvalidSocket {
        flag,
        value,
        source,
    })
}

fn read_artifact(path: &std::path::Path, kind: &'static str) -> Result<String, CliError> {
    std::fs::read_to_string(path).map_err(|source| CliError::ReadArtifact {
        kind,
        path: path.to_owned(),
        source,
    })
}

#[cfg(test)]
mod tests {
    #![allow(clippy::expect_used)]

    use super::*;

    fn args(values: &[&str]) -> Vec<String> {
        values.iter().map(|value| (*value).to_owned()).collect()
    }

    #[test]
    fn unknown_duplicate_and_missing_arguments_fail_closed() {
        for values in [
            args(&["--unknown"]),
            args(&["--candidate"]),
            args(&["--auto-arm", "bad"]),
            args(&["--bridge", "127.0.0.1:1", "--bridge", "127.0.0.1:2"]),
        ] {
            assert!(Cli::parse(values).is_err());
        }
    }

    #[test]
    fn candidate_and_experiment_contracts_fail_closed() {
        for values in [
            args(&["--candidate", "a"]),
            args(&["--plant-artifact", "b"]),
            args(&["--candidate", "a", "--plant-artifact", "b", "--identify"]),
            args(&["--identify", "--sweep"]),
            args(&["--plant-output", "out.toml"]),
            args(&[
                "--runtime-handshake",
                "runtime.toml",
                "--identify",
                "--plant-output",
                "plant.toml",
            ]),
        ] {
            assert!(Cli::parse(values).is_err());
        }
    }

    #[test]
    fn complete_candidate_arguments_are_accepted() {
        let parsed = Cli::parse(args(&[
            "--candidate",
            "candidate.toml",
            "--plant-artifact",
            "plant.toml",
            "--run-manifest",
            "run.toml",
            "--runtime-handshake",
            "runtime.toml",
            "--tuning-trace-endpoint",
            "127.0.0.1:9000",
        ]))
        .expect("valid candidate arguments");
        assert_eq!(parsed.experiment, None);
    }

    #[test]
    fn complete_identification_artifacts_are_required() {
        let parsed = Cli::parse(args(&[
            "--runtime-handshake",
            "runtime.toml",
            "--identify",
            "--run-manifest",
            "run.toml",
            "--plant-output",
            "plant.toml",
            "--trace-output",
            "trace.toml",
            "--tuning-trace-endpoint",
            "127.0.0.1:9000",
        ]))
        .expect("valid identify arguments");
        assert_eq!(parsed.experiment, Some(RunPurpose::Identify));
    }

    #[test]
    fn tuning_runs_require_a_manifest_and_loopback_trace() {
        for values in [
            args(&[
                "--runtime-handshake",
                "runtime.toml",
                "--candidate",
                "candidate.toml",
                "--plant-artifact",
                "plant.toml",
                "--run-manifest",
                "run.toml",
            ]),
            args(&[
                "--runtime-handshake",
                "runtime.toml",
                "--sweep",
                "--tuning-trace-endpoint",
                "127.0.0.1:9000",
            ]),
            args(&[
                "--runtime-handshake",
                "runtime.toml",
                "--sweep",
                "--run-manifest",
                "run.toml",
                "--tuning-trace-endpoint",
                "192.0.2.1:9000",
            ]),
        ] {
            assert!(Cli::parse(values).is_err());
        }
    }

    #[test]
    fn normal_run_can_omit_the_optional_trace() {
        let parsed =
            Cli::parse(args(&["--runtime-handshake", "runtime.toml"])).expect("valid normal run");
        assert!(parsed.tuning_trace_endpoint.is_none());
        assert_eq!(parsed.experiment, None);
    }
}
