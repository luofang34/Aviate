//! Fail-closed, packet-synchronous tuning observation transport.

mod protocol;

pub use protocol::{
    TuningCommand, TuningCommandSource, TuningConfigMode, TuningConstraintFlags, TuningControlMode,
    TuningControlObservation, TuningEstimate, TuningEstimateQuality, TuningEstimateValidity,
    TuningFrameType, TuningHandshake, TuningImu, TuningObservationAck, TuningReady, TuningSetpoint,
};

use std::fmt;
use std::io::{Read, Write};
use std::net::{Shutdown, SocketAddr, TcpStream};
use std::time::Duration;

use aviate_core::control::Command;
use aviate_core::state::StateEstimate;
use serde::de::DeserializeOwned;
use serde::Serialize;

use super::XPlaneControlObservation;

pub(super) const TUNING_TRACE_SCHEMA_VERSION: u16 = 2;
const HANDSHAKE_TIMEOUT: Duration = Duration::from_secs(2);
const OBSERVATION_TIMEOUT: Duration = Duration::from_millis(20);
const MAX_FRAME_BYTES: usize = 65_536;

/// Immutable identities for one tuning trace.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct XPlaneTuningTraceIdentity {
    /// SHA-256 of the exact run manifest text.
    pub run_manifest_digest: String,
    /// SHA-256 of the running executable.
    pub build_identity: String,
    /// SHA-256 of the embedded source bundle.
    pub source_identity: String,
    /// SHA-256 of the dependency lock file.
    pub lock_identity: String,
    /// SHA-256 of the simulator model.
    pub simulator_model_digest: String,
    /// SHA-256 of the consumed runtime handshake.
    pub runtime_handshake_digest: String,
    /// Candidate document identity for a candidate run.
    pub candidate_digest: Option<String>,
    /// Final candidate lineage identity for a candidate run.
    pub candidate_lineage_digest: Option<String>,
    /// Plant artifact identity for a candidate run.
    pub plant_artifact_digest: Option<String>,
    /// Flight algorithm identity as 16 lowercase hexadecimal digits.
    pub algorithm_identity_hash: String,
    /// Resolved kernel identity as 16 lowercase hexadecimal digits.
    pub kernel_config_hash: String,
}

/// Configuration for the packet-synchronous tuning trace.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct XPlaneTuningTraceConfig {
    endpoint: SocketAddr,
    identity: XPlaneTuningTraceIdentity,
}

impl XPlaneTuningTraceConfig {
    /// Validate one loopback endpoint and its immutable identity.
    ///
    /// # Errors
    ///
    /// Returns an error for a non-loopback address, malformed digest, or
    /// incomplete candidate identity.
    pub fn new(
        endpoint: SocketAddr,
        identity: XPlaneTuningTraceIdentity,
    ) -> Result<Self, TuningTraceError> {
        if !endpoint.ip().is_loopback() {
            return Err(TuningTraceError::NonLoopbackEndpoint(endpoint));
        }
        validate_identity(&identity)?;
        Ok(Self { endpoint, identity })
    }
}

/// Tuning trace transport failure.
#[derive(Debug)]
pub enum TuningTraceError {
    /// The trace endpoint is not on the local host.
    NonLoopbackEndpoint(SocketAddr),
    /// One immutable identity is malformed or incomplete.
    InvalidIdentity(&'static str),
    /// The trace runner did not accept a TCP connection.
    Connect {
        /// Selected local endpoint.
        endpoint: SocketAddr,
        /// Socket failure.
        source: std::io::Error,
    },
    /// A socket bound could not be configured.
    Configure {
        /// Configuration operation.
        operation: &'static str,
        /// Socket failure.
        source: std::io::Error,
    },
    /// A frame could not be encoded.
    Encode(serde_json::Error),
    /// An encoded or received frame exceeded the protocol bound.
    FrameTooLarge(usize),
    /// A frame could not be written in its deadline.
    Write(std::io::Error),
    /// An acknowledgement could not be read in its deadline.
    Read(std::io::Error),
    /// An acknowledgement could not be decoded.
    Decode(serde_json::Error),
    /// The runner returned the wrong ready acknowledgement.
    ReadyMismatch,
    /// The runner returned an acknowledgement for another run or schema.
    ObservationAckIdentityMismatch,
    /// The runner returned the wrong observation sequence.
    ObservationAckSequenceMismatch {
        /// Sequence Aviate sent.
        expected: u64,
        /// Sequence the runner accepted.
        received: u64,
    },
    /// The transport failed earlier and cannot resume this run.
    Failed,
}

impl fmt::Display for TuningTraceError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NonLoopbackEndpoint(endpoint) => {
                write!(
                    formatter,
                    "tuning trace endpoint {endpoint} is not loopback"
                )
            }
            Self::InvalidIdentity(field) => write!(formatter, "invalid tuning identity {field}"),
            Self::Connect { endpoint, source } => {
                write!(
                    formatter,
                    "cannot connect tuning trace {endpoint}: {source}"
                )
            }
            Self::Configure { operation, source } => {
                write!(
                    formatter,
                    "cannot {operation} tuning trace socket: {source}"
                )
            }
            Self::Encode(error) => write!(formatter, "cannot encode tuning trace: {error}"),
            Self::FrameTooLarge(size) => write!(formatter, "tuning trace frame has {size} bytes"),
            Self::Write(error) => write!(formatter, "cannot write tuning trace: {error}"),
            Self::Read(error) => write!(formatter, "cannot read tuning trace ack: {error}"),
            Self::Decode(error) => write!(formatter, "cannot decode tuning trace ack: {error}"),
            Self::ReadyMismatch => {
                formatter.write_str("tuning trace ready identity does not match")
            }
            Self::ObservationAckIdentityMismatch => {
                formatter.write_str("tuning trace observation ack identity does not match")
            }
            Self::ObservationAckSequenceMismatch { expected, received } => write!(
                formatter,
                "tuning trace ack sequence {received} does not match {expected}"
            ),
            Self::Failed => formatter.write_str("tuning trace transport already failed"),
        }
    }
}

impl std::error::Error for TuningTraceError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Connect { source, .. } | Self::Configure { source, .. } => Some(source),
            Self::Encode(error) | Self::Decode(error) => Some(error),
            Self::Write(error) | Self::Read(error) => Some(error),
            Self::NonLoopbackEndpoint(_)
            | Self::InvalidIdentity(_)
            | Self::FrameTooLarge(_)
            | Self::ReadyMismatch
            | Self::ObservationAckIdentityMismatch
            | Self::ObservationAckSequenceMismatch { .. }
            | Self::Failed => None,
        }
    }
}

pub(super) struct TuningTracePublisher {
    stream: Option<TcpStream>,
    run_manifest_digest: String,
    sequence: u64,
    failure: Option<TuningTraceError>,
}

impl TuningTracePublisher {
    pub(super) fn connect(config: XPlaneTuningTraceConfig) -> Result<Self, TuningTraceError> {
        let mut stream =
            TcpStream::connect_timeout(&config.endpoint, HANDSHAKE_TIMEOUT).map_err(|source| {
                TuningTraceError::Connect {
                    endpoint: config.endpoint,
                    source,
                }
            })?;
        configure(&stream, HANDSHAKE_TIMEOUT)?;
        let handshake = handshake_from_identity(config.identity);
        write_frame(&mut stream, &handshake)?;
        let ready: TuningReady = read_frame(&mut stream)?;
        if ready.frame_type != TuningFrameType::AviateTuningReady
            || ready.schema_version != TUNING_TRACE_SCHEMA_VERSION
            || ready.run_manifest_digest != handshake.run_manifest_digest
        {
            return Err(TuningTraceError::ReadyMismatch);
        }
        configure(&stream, OBSERVATION_TIMEOUT)?;
        Ok(Self {
            stream: Some(stream),
            run_manifest_digest: handshake.run_manifest_digest,
            sequence: 0,
            failure: None,
        })
    }

    pub(super) fn publish(
        &mut self,
        observation: XPlaneControlObservation,
        requested: Option<&Command>,
        command_provenance: Option<aviate_hal_xil::MavlinkCommandProvenance>,
        effective: &Command,
        estimate: &StateEstimate,
        armed: bool,
    ) {
        if self.failure.is_some() {
            return;
        }
        let sequence = self.sequence;
        let frame = TuningControlObservation::from_packet(
            sequence,
            observation,
            requested,
            command_provenance,
            effective,
            estimate,
            armed,
        );
        if let Err(error) = self.publish_frame(&frame) {
            self.fail(error);
            return;
        }
        self.sequence = self.sequence.wrapping_add(1);
    }

    fn publish_frame(&mut self, frame: &TuningControlObservation) -> Result<(), TuningTraceError> {
        let stream = self.stream.as_mut().ok_or(TuningTraceError::Failed)?;
        write_frame(stream, frame)?;
        let ack: TuningObservationAck = read_frame(stream)?;
        if ack.frame_type != TuningFrameType::AviateTuningObservationAck
            || ack.schema_version != TUNING_TRACE_SCHEMA_VERSION
            || ack.run_manifest_digest != self.run_manifest_digest
        {
            return Err(TuningTraceError::ObservationAckIdentityMismatch);
        }
        if ack.sequence != frame.sequence {
            return Err(TuningTraceError::ObservationAckSequenceMismatch {
                expected: frame.sequence,
                received: ack.sequence,
            });
        }
        Ok(())
    }

    fn fail(&mut self, error: TuningTraceError) {
        if let Some(stream) = self.stream.take() {
            stream.shutdown(Shutdown::Both).ok();
        }
        self.failure = Some(error);
    }

    pub(super) fn is_ready(&self) -> bool {
        self.stream.is_some() && self.failure.is_none()
    }

    pub(super) fn failure(&self) -> Option<&TuningTraceError> {
        self.failure.as_ref()
    }
}

fn handshake_from_identity(identity: XPlaneTuningTraceIdentity) -> TuningHandshake {
    TuningHandshake {
        frame_type: TuningFrameType::AviateTuningHandshake,
        schema_version: TUNING_TRACE_SCHEMA_VERSION,
        run_manifest_digest: identity.run_manifest_digest,
        build_identity: identity.build_identity,
        source_identity: identity.source_identity,
        lock_identity: identity.lock_identity,
        simulator_model_digest: identity.simulator_model_digest,
        runtime_handshake_digest: identity.runtime_handshake_digest,
        candidate_digest: identity.candidate_digest,
        candidate_lineage_digest: identity.candidate_lineage_digest,
        plant_artifact_digest: identity.plant_artifact_digest,
        algorithm_identity_hash: identity.algorithm_identity_hash,
        kernel_config_hash: identity.kernel_config_hash,
    }
}

fn validate_identity(identity: &XPlaneTuningTraceIdentity) -> Result<(), TuningTraceError> {
    for (field, value) in [
        ("run_manifest_digest", identity.run_manifest_digest.as_str()),
        ("build_identity", identity.build_identity.as_str()),
        ("source_identity", identity.source_identity.as_str()),
        ("lock_identity", identity.lock_identity.as_str()),
        (
            "simulator_model_digest",
            identity.simulator_model_digest.as_str(),
        ),
        (
            "runtime_handshake_digest",
            identity.runtime_handshake_digest.as_str(),
        ),
    ] {
        validate_hex(field, value, 64)?;
    }
    validate_hex(
        "algorithm_identity_hash",
        &identity.algorithm_identity_hash,
        16,
    )?;
    validate_hex("kernel_config_hash", &identity.kernel_config_hash, 16)?;
    let candidate_fields = [
        identity.candidate_digest.as_deref(),
        identity.candidate_lineage_digest.as_deref(),
        identity.plant_artifact_digest.as_deref(),
    ];
    if candidate_fields.iter().any(Option::is_some) && !candidate_fields.iter().all(Option::is_some)
    {
        return Err(TuningTraceError::InvalidIdentity("candidate identity"));
    }
    for value in candidate_fields.into_iter().flatten() {
        validate_hex("candidate identity", value, 64)?;
    }
    Ok(())
}

fn validate_hex(field: &'static str, value: &str, length: usize) -> Result<(), TuningTraceError> {
    if value.len() != length
        || !value
            .as_bytes()
            .iter()
            .all(|byte| byte.is_ascii_digit() || matches!(*byte, b'a'..=b'f'))
    {
        return Err(TuningTraceError::InvalidIdentity(field));
    }
    Ok(())
}

fn configure(stream: &TcpStream, timeout: Duration) -> Result<(), TuningTraceError> {
    stream
        .set_nodelay(true)
        .map_err(|source| TuningTraceError::Configure {
            operation: "set no-delay on",
            source,
        })?;
    stream
        .set_read_timeout(Some(timeout))
        .map_err(|source| TuningTraceError::Configure {
            operation: "set read timeout on",
            source,
        })?;
    stream
        .set_write_timeout(Some(timeout))
        .map_err(|source| TuningTraceError::Configure {
            operation: "set write timeout on",
            source,
        })
}

fn write_frame<T: Serialize>(stream: &mut TcpStream, frame: &T) -> Result<(), TuningTraceError> {
    let payload = serde_json::to_vec(frame).map_err(TuningTraceError::Encode)?;
    if payload.len() > MAX_FRAME_BYTES {
        return Err(TuningTraceError::FrameTooLarge(payload.len()));
    }
    let length =
        u32::try_from(payload.len()).map_err(|_| TuningTraceError::FrameTooLarge(payload.len()))?;
    stream
        .write_all(&length.to_be_bytes())
        .and_then(|()| stream.write_all(&payload))
        .map_err(TuningTraceError::Write)
}

fn read_frame<T: DeserializeOwned>(stream: &mut TcpStream) -> Result<T, TuningTraceError> {
    let mut length = [0_u8; 4];
    stream
        .read_exact(&mut length)
        .map_err(TuningTraceError::Read)?;
    let length = u32::from_be_bytes(length) as usize;
    if length > MAX_FRAME_BYTES {
        return Err(TuningTraceError::FrameTooLarge(length));
    }
    let mut payload = vec![0_u8; length];
    stream
        .read_exact(&mut payload)
        .map_err(TuningTraceError::Read)?;
    serde_json::from_slice(&payload).map_err(TuningTraceError::Decode)
}

#[cfg(test)]
mod tests;
