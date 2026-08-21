//! Typed command-line and calibration input failures.

use std::path::PathBuf;

#[derive(Debug)]
pub(crate) enum CliError {
    UnknownArgument(String),
    MissingValue(&'static str),
    Duplicate(&'static str),
    InvalidSocket {
        flag: &'static str,
        value: String,
        source: std::net::AddrParseError,
    },
    NonLoopbackTrace(std::net::SocketAddr),
    InvalidDuration {
        value: String,
        source: std::num::ParseIntError,
    },
    InvalidCombination(&'static str),
    ClaimRuntimeBinding {
        path: PathBuf,
        source: std::io::Error,
    },
    InsecureRuntimeBinding {
        path: PathBuf,
        mode: u32,
    },
    RuntimeBindingChanged(PathBuf),
    InvalidRuntimeBinding(aviate_config::xplane_runtime::XPlaneRuntimeHandshakeError),
    RuntimeBridgeMismatch {
        declared: String,
        selected: std::net::SocketAddr,
    },
    ConsumeRuntimeBinding {
        path: PathBuf,
        source: std::io::Error,
    },
    ReadArtifact {
        kind: &'static str,
        path: PathBuf,
        source: std::io::Error,
    },
}

impl core::fmt::Display for CliError {
    fn fmt(&self, formatter: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::UnknownArgument(value) => write!(formatter, "unknown argument {value:?}"),
            Self::MissingValue(flag) => write!(formatter, "{flag} requires a value"),
            Self::Duplicate(flag) => write!(formatter, "{flag} can be specified only once"),
            Self::InvalidSocket {
                flag,
                value,
                source,
            } => {
                write!(formatter, "{flag} {value:?} is not HOST:PORT: {source}")
            }
            Self::NonLoopbackTrace(endpoint) => {
                write!(
                    formatter,
                    "tuning trace endpoint {endpoint} is not loopback"
                )
            }
            Self::InvalidDuration { value, source } => {
                write!(formatter, "--auto-arm {value:?} is not seconds: {source}")
            }
            Self::InvalidCombination(message) => formatter.write_str(message),
            Self::ClaimRuntimeBinding { path, source } => {
                write!(
                    formatter,
                    "cannot claim runtime handshake {path:?}: {source}"
                )
            }
            Self::InsecureRuntimeBinding { path, mode } => write!(
                formatter,
                "runtime handshake {path:?} has insecure permissions {mode:o}"
            ),
            Self::RuntimeBindingChanged(path) => {
                write!(
                    formatter,
                    "runtime handshake changed while claimed: {path:?}"
                )
            }
            Self::InvalidRuntimeBinding(error) => write!(formatter, "{error}"),
            Self::RuntimeBridgeMismatch { declared, selected } => write!(
                formatter,
                "runtime handshake bridge {declared:?} does not match selected bridge {selected}"
            ),
            Self::ConsumeRuntimeBinding { path, source } => {
                write!(
                    formatter,
                    "cannot consume runtime handshake {path:?}: {source}"
                )
            }
            Self::ReadArtifact { kind, path, source } => {
                write!(formatter, "cannot read {kind} {path:?}: {source}")
            }
        }
    }
}

impl std::error::Error for CliError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::InvalidSocket { source, .. } => Some(source),
            Self::InvalidDuration { source, .. } => Some(source),
            Self::ClaimRuntimeBinding { source, .. }
            | Self::ConsumeRuntimeBinding { source, .. } => Some(source),
            Self::InvalidRuntimeBinding(error) => Some(error),
            Self::ReadArtifact { source, .. } => Some(source),
            Self::UnknownArgument(_)
            | Self::MissingValue(_)
            | Self::Duplicate(_)
            | Self::NonLoopbackTrace(_)
            | Self::InvalidCombination(_)
            | Self::InsecureRuntimeBinding { .. }
            | Self::RuntimeBindingChanged(_)
            | Self::RuntimeBridgeMismatch { .. } => None,
        }
    }
}
