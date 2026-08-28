//! No-clobber publication of machine-readable artifacts.

use std::fs::OpenOptions;
use std::io::Write as _;
use std::path::{Path, PathBuf};

#[derive(Debug)]
pub(super) enum ArtifactError {
    InvalidTarget(PathBuf),
    Exists(PathBuf),
    Create {
        path: PathBuf,
        source: std::io::Error,
    },
    Write {
        path: PathBuf,
        source: std::io::Error,
    },
    Publish {
        path: PathBuf,
        source: std::io::Error,
    },
}

impl core::fmt::Display for ArtifactError {
    fn fmt(&self, formatter: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::InvalidTarget(path) => write!(formatter, "invalid artifact target {path:?}"),
            Self::Exists(path) => write!(formatter, "artifact target already exists: {path:?}"),
            Self::Create { path, source } => {
                write!(
                    formatter,
                    "cannot create artifact temporary file {path:?}: {source}"
                )
            }
            Self::Write { path, source } => {
                write!(
                    formatter,
                    "cannot write artifact temporary file {path:?}: {source}"
                )
            }
            Self::Publish { path, source } => {
                write!(formatter, "cannot publish artifact {path:?}: {source}")
            }
        }
    }
}

impl std::error::Error for ArtifactError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Create { source, .. }
            | Self::Write { source, .. }
            | Self::Publish { source, .. } => Some(source),
            Self::InvalidTarget(_) | Self::Exists(_) => None,
        }
    }
}

pub(super) fn publish_optional(path: Option<&Path>, text: &str) -> Result<(), ArtifactError> {
    if let Some(path) = path {
        publish(path, text)
    } else {
        Ok(())
    }
}

fn publish(target: &Path, text: &str) -> Result<(), ArtifactError> {
    let parent = target.parent().unwrap_or_else(|| Path::new("."));
    let file_name = target
        .file_name()
        .ok_or_else(|| ArtifactError::InvalidTarget(target.to_owned()))?;
    let (temporary, file) = create_temporary(parent, file_name)?;
    let result = write_and_publish(file, &temporary, target, text);
    std::fs::remove_file(&temporary).ok();
    result
}

fn create_temporary(
    parent: &Path,
    file_name: &std::ffi::OsStr,
) -> Result<(PathBuf, std::fs::File), ArtifactError> {
    for attempt in 0..100_u32 {
        let name = format!(
            ".{}.{}.{}.tmp",
            file_name.to_string_lossy(),
            std::process::id(),
            attempt
        );
        let path = parent.join(name);
        match OpenOptions::new().write(true).create_new(true).open(&path) {
            Ok(file) => return Ok((path, file)),
            Err(error) if error.kind() == std::io::ErrorKind::AlreadyExists => continue,
            Err(source) => return Err(ArtifactError::Create { path, source }),
        }
    }
    Err(ArtifactError::InvalidTarget(parent.join(file_name)))
}

fn write_and_publish(
    mut file: std::fs::File,
    temporary: &Path,
    target: &Path,
    text: &str,
) -> Result<(), ArtifactError> {
    file.write_all(text.as_bytes())
        .and_then(|()| file.sync_all())
        .map_err(|source| ArtifactError::Write {
            path: temporary.to_owned(),
            source,
        })?;
    match std::fs::hard_link(temporary, target) {
        Ok(()) => Ok(()),
        Err(error) if error.kind() == std::io::ErrorKind::AlreadyExists => {
            Err(ArtifactError::Exists(target.to_owned()))
        }
        Err(source) => Err(ArtifactError::Publish {
            path: target.to_owned(),
            source,
        }),
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::expect_used)]

    use super::*;

    #[test]
    fn concurrent_publish_never_overwrites_the_winner() {
        let directory =
            std::env::temp_dir().join(format!("aviate-artifact-test-{}", std::process::id()));
        std::fs::create_dir_all(&directory).expect("test directory");
        let target = directory.join("artifact.toml");
        std::fs::remove_file(&target).ok();
        let mut threads = Vec::new();
        for index in 0..8 {
            let target = target.clone();
            threads.push(std::thread::spawn(move || {
                publish(&target, &index.to_string())
            }));
        }
        let successes = threads
            .into_iter()
            .map(|thread| thread.join().expect("writer did not panic"))
            .filter(Result::is_ok)
            .count();
        assert_eq!(successes, 1);
        let content = std::fs::read_to_string(&target).expect("published content");
        assert!(content.parse::<usize>().is_ok());
        std::fs::remove_file(target).ok();
        std::fs::remove_dir(directory).ok();
    }
}
