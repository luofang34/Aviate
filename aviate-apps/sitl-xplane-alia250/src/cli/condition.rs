//! All-or-none command-line binding for one condition artifact.

use std::path::PathBuf;

use aviate_hal_xil::perturbation::{LoadedPerturbationArtifact, PerturbationCapability};

use super::CliError;

#[derive(Default)]
pub(super) struct ConditionArguments {
    artifact: Option<PathBuf>,
    artifact_sha256: Option<String>,
    condition_digest: Option<String>,
    run_seed: Option<String>,
    required_capabilities: Option<String>,
}

impl ConditionArguments {
    pub(super) fn set(&mut self, flag: &str, value: String) -> Result<(), CliError> {
        match flag {
            "--condition-artifact" => set_once(
                &mut self.artifact,
                PathBuf::from(value),
                "--condition-artifact",
            ),
            "--condition-artifact-sha256" => set_once(
                &mut self.artifact_sha256,
                value,
                "--condition-artifact-sha256",
            ),
            "--condition-digest" => {
                set_once(&mut self.condition_digest, value, "--condition-digest")
            }
            "--run-seed" => set_once(&mut self.run_seed, value, "--run-seed"),
            "--required-perturbation-capabilities" => set_once(
                &mut self.required_capabilities,
                value,
                "--required-perturbation-capabilities",
            ),
            _ => Err(CliError::UnknownArgument(flag.to_owned())),
        }
    }

    pub(super) fn validate(&self) -> Result<(), CliError> {
        let present = [
            self.artifact.is_some(),
            self.artifact_sha256.is_some(),
            self.condition_digest.is_some(),
            self.run_seed.is_some(),
            self.required_capabilities.is_some(),
        ];
        let count = present.into_iter().filter(|value| *value).count();
        if count == 0 {
            return Ok(());
        }
        if count != present.len() {
            Err(CliError::InvalidCombination(
                "condition artifact identity flags must be supplied together",
            ))
        } else {
            parse_digest(
                required(&self.artifact_sha256, "--condition-artifact-sha256")?,
                "--condition-artifact-sha256",
            )?;
            parse_digest(
                required(&self.condition_digest, "--condition-digest")?,
                "--condition-digest",
            )?;
            let run_seed = required(&self.run_seed, "--run-seed")?;
            run_seed
                .parse::<u64>()
                .map_err(|source| CliError::InvalidRunSeed {
                    value: run_seed.to_owned(),
                    source,
                })?;
            parse_capabilities(required(
                &self.required_capabilities,
                "--required-perturbation-capabilities",
            )?)?;
            Ok(())
        }
    }

    pub(super) const fn is_some(&self) -> bool {
        self.artifact.is_some()
    }

    pub(super) fn load(&self) -> Result<Option<LoadedPerturbationArtifact>, CliError> {
        let Some(path) = self.artifact.as_deref() else {
            return Ok(None);
        };
        let artifact_sha256 = parse_digest(
            required(&self.artifact_sha256, "--condition-artifact-sha256")?,
            "--condition-artifact-sha256",
        )?;
        let condition_digest = parse_digest(
            required(&self.condition_digest, "--condition-digest")?,
            "--condition-digest",
        )?;
        let run_seed_text = required(&self.run_seed, "--run-seed")?;
        let run_seed = run_seed_text
            .parse::<u64>()
            .map_err(|source| CliError::InvalidRunSeed {
                value: run_seed_text.to_owned(),
                source,
            })?;
        let capabilities = parse_capabilities(required(
            &self.required_capabilities,
            "--required-perturbation-capabilities",
        )?)?;
        LoadedPerturbationArtifact::load(
            path,
            artifact_sha256,
            condition_digest,
            run_seed,
            &capabilities,
        )
        .map(Some)
        .map_err(|source| CliError::ConditionArtifact {
            path: path.to_owned(),
            source,
        })
    }
}

fn required<'a>(value: &'a Option<String>, flag: &'static str) -> Result<&'a str, CliError> {
    value.as_deref().ok_or(CliError::MissingValue(flag))
}

fn parse_digest(value: &str, flag: &'static str) -> Result<[u8; 32], CliError> {
    if value.len() != 64
        || !value
            .as_bytes()
            .iter()
            .all(|byte| byte.is_ascii_digit() || matches!(*byte, b'a'..=b'f'))
    {
        return Err(CliError::InvalidDigest {
            flag,
            value: value.to_owned(),
        });
    }
    let mut digest = [0_u8; 32];
    for (index, output) in digest.iter_mut().enumerate() {
        let offset = index * 2;
        *output = u8::from_str_radix(&value[offset..offset + 2], 16).map_err(|_| {
            CliError::InvalidDigest {
                flag,
                value: value.to_owned(),
            }
        })?;
    }
    Ok(digest)
}

fn parse_capabilities(value: &str) -> Result<Vec<PerturbationCapability>, CliError> {
    if value == "none" {
        return Ok(Vec::new());
    }
    if value.is_empty() {
        return Err(CliError::InvalidCapabilitySet(value.to_owned()));
    }
    value
        .split(',')
        .map(|name| {
            PerturbationCapability::parse(name)
                .map_err(|_| CliError::InvalidCapabilitySet(value.to_owned()))
        })
        .collect()
}

fn set_once<T>(target: &mut Option<T>, value: T, flag: &'static str) -> Result<(), CliError> {
    if target.is_some() {
        return Err(CliError::Duplicate(flag));
    }
    *target = Some(value);
    Ok(())
}
