//! Fault Injection Protocol for XIL Testing
//!
//! Defines the UDP-based protocol for injecting faults during SITL/HITL testing.
//! This enables deterministic, reproducible testing of fault handling logic.
//!
//! ## Protocol Overview
//!
//! ```text
//! Test Runner                    Flight Controller
//!     |                                |
//!     |--- FaultCommand (UDP) -------->|
//!     |                                |-- inject fault
//!     |<-- FaultAck (UDP) -------------|
//!     |                                |
//! ```
//!
//! ## Message Format
//!
//! All messages are simple byte sequences for minimal overhead:
//!
//! **FaultCommand** (12 bytes):
//! - magic: u16 = 0xFA17 ("FALT")
//! - sequence: u16 (for matching acks)
//! - target: u8 (SensorTarget)
//! - action: u8 (FaultAction)
//! - param1: i16 (fault-specific, e.g., dropout cycles)
//! - param2: i16 (fault-specific)
//! - param3: i16 (fault-specific)
//!
//! **FaultAck** (8 bytes):
//! - magic: u16 = 0xAC17 ("ACKT")
//! - sequence: u16 (echoed from command)
//! - status: u8 (AckStatus)
//! - target: u8 (echoed from command)
//! - reserved: u16

#![forbid(unsafe_code)]

use crate::mission::{FaultSpec, SensorTarget};

/// Protocol magic number for FaultCommand
pub const FAULT_CMD_MAGIC: u16 = 0xFA17;

/// Protocol magic number for FaultAck
pub const FAULT_ACK_MAGIC: u16 = 0xAC17;

/// Size of FaultCommand message in bytes
pub const FAULT_CMD_SIZE: usize = 12;

/// Size of FaultAck message in bytes
pub const FAULT_ACK_SIZE: usize = 8;

/// Wire-format action code (maps to FaultSpec)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum FaultAction {
    /// Clear all faults on target
    Clear = 0,
    /// Set sensor health to degraded
    HealthDegraded = 1,
    /// Set sensor health to failed
    HealthFailed = 2,
    /// Inject NaN values
    InjectNaN = 3,
    /// Drop sensor readings for N cycles (param1 = cycle count)
    Dropout = 4,
    /// Add bias offset (param1/2/3 = offset * 100 for 3-axis, param1 * 100 for scalar)
    BiasShift = 5,
    /// Scalar bias (param1 = offset * 100)
    BiasScalar = 6,
}

impl FaultAction {
    /// Convert from u8, returns None for invalid values
    pub fn from_u8(value: u8) -> Option<Self> {
        match value {
            0 => Some(Self::Clear),
            1 => Some(Self::HealthDegraded),
            2 => Some(Self::HealthFailed),
            3 => Some(Self::InjectNaN),
            4 => Some(Self::Dropout),
            5 => Some(Self::BiasShift),
            6 => Some(Self::BiasScalar),
            _ => None,
        }
    }
}

/// Acknowledgment status
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum AckStatus {
    /// Command executed successfully
    Ok = 0,
    /// Unknown/invalid target
    UnknownTarget = 1,
    /// Unknown/invalid action
    UnknownAction = 2,
    /// Invalid parameters
    InvalidParams = 3,
    /// Fault injection not enabled (feature disabled)
    NotEnabled = 4,
}

impl AckStatus {
    /// Convert from u8, returns None for invalid values
    pub fn from_u8(value: u8) -> Option<Self> {
        match value {
            0 => Some(Self::Ok),
            1 => Some(Self::UnknownTarget),
            2 => Some(Self::UnknownAction),
            3 => Some(Self::InvalidParams),
            4 => Some(Self::NotEnabled),
            _ => None,
        }
    }
}

/// Convert SensorTarget to wire format (u8)
fn target_to_u8(target: SensorTarget) -> u8 {
    match target {
        SensorTarget::Imu => 0,
        SensorTarget::Baro => 1,
        SensorTarget::Mag => 2,
        SensorTarget::Gnss => 3,
    }
}

/// Convert wire format (u8) to SensorTarget
fn target_from_u8(value: u8) -> Option<SensorTarget> {
    match value {
        0 => Some(SensorTarget::Imu),
        1 => Some(SensorTarget::Baro),
        2 => Some(SensorTarget::Mag),
        3 => Some(SensorTarget::Gnss),
        _ => None,
    }
}

/// Fault command sent from test runner to FC
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FaultCommand {
    /// Sequence number for matching acknowledgments
    pub sequence: u16,
    /// Target sensor/subsystem (None = all sensors for clear)
    pub target: Option<SensorTarget>,
    /// Fault specification (None = clear)
    pub fault: Option<FaultSpec>,
}

impl FaultCommand {
    /// Create a new fault command
    pub fn new(target: SensorTarget, fault: FaultSpec) -> Self {
        Self {
            sequence: 0,
            target: Some(target),
            fault: Some(fault),
        }
    }

    /// Create a clear command for specified target
    pub fn clear(target: SensorTarget) -> Self {
        Self {
            sequence: 0,
            target: Some(target),
            fault: None,
        }
    }

    /// Create a clear-all command
    pub fn clear_all() -> Self {
        Self {
            sequence: 0,
            target: None,
            fault: None,
        }
    }

    /// Set sequence number
    pub fn with_sequence(mut self, seq: u16) -> Self {
        self.sequence = seq;
        self
    }

    /// Serialize to bytes (little-endian)
    pub fn to_bytes(&self) -> [u8; FAULT_CMD_SIZE] {
        let mut buf = [0u8; FAULT_CMD_SIZE];
        buf[0..2].copy_from_slice(&FAULT_CMD_MAGIC.to_le_bytes());
        buf[2..4].copy_from_slice(&self.sequence.to_le_bytes());

        // Target: 255 = all, otherwise sensor index
        buf[4] = self.target.map(target_to_u8).unwrap_or(255);

        // Encode action and params
        let (action, param1, param2, param3) = match &self.fault {
            None => (FaultAction::Clear, 0i16, 0i16, 0i16),
            Some(FaultSpec::HealthDegraded) => (FaultAction::HealthDegraded, 0, 0, 0),
            Some(FaultSpec::HealthFailed) => (FaultAction::HealthFailed, 0, 0, 0),
            Some(FaultSpec::NaN) => (FaultAction::InjectNaN, 0, 0, 0),
            Some(FaultSpec::Dropout { cycles }) => (
                FaultAction::Dropout,
                (*cycles).min(i16::MAX as u32) as i16,
                0,
                0,
            ),
            Some(FaultSpec::BiasShift { offset }) => {
                let p1 = (offset[0] * 100.0).clamp(-32768.0, 32767.0) as i16;
                let p2 = (offset[1] * 100.0).clamp(-32768.0, 32767.0) as i16;
                let p3 = (offset[2] * 100.0).clamp(-32768.0, 32767.0) as i16;
                (FaultAction::BiasShift, p1, p2, p3)
            }
            Some(FaultSpec::BiasScalar { offset }) => {
                let p1 = (*offset * 100.0).clamp(-32768.0, 32767.0) as i16;
                (FaultAction::BiasScalar, p1, 0, 0)
            }
        };

        buf[5] = action as u8;
        buf[6..8].copy_from_slice(&param1.to_le_bytes());
        buf[8..10].copy_from_slice(&param2.to_le_bytes());
        buf[10..12].copy_from_slice(&param3.to_le_bytes());
        buf
    }

    /// Deserialize from bytes (little-endian)
    pub fn from_bytes(buf: &[u8]) -> Option<Self> {
        if buf.len() < FAULT_CMD_SIZE {
            return None;
        }

        let magic = u16::from_le_bytes([buf[0], buf[1]]);
        if magic != FAULT_CMD_MAGIC {
            return None;
        }

        let sequence = u16::from_le_bytes([buf[2], buf[3]]);
        let target = if buf[4] == 255 {
            None
        } else {
            Some(target_from_u8(buf[4])?)
        };
        let action = FaultAction::from_u8(buf[5])?;
        let param1 = i16::from_le_bytes([buf[6], buf[7]]);
        let param2 = i16::from_le_bytes([buf[8], buf[9]]);
        let param3 = i16::from_le_bytes([buf[10], buf[11]]);

        let fault = match action {
            FaultAction::Clear => None,
            FaultAction::HealthDegraded => Some(FaultSpec::HealthDegraded),
            FaultAction::HealthFailed => Some(FaultSpec::HealthFailed),
            FaultAction::InjectNaN => Some(FaultSpec::NaN),
            FaultAction::Dropout => Some(FaultSpec::Dropout {
                cycles: param1.max(0) as u32,
            }),
            FaultAction::BiasShift => Some(FaultSpec::BiasShift {
                offset: [
                    param1 as f32 / 100.0,
                    param2 as f32 / 100.0,
                    param3 as f32 / 100.0,
                ],
            }),
            FaultAction::BiasScalar => Some(FaultSpec::BiasScalar {
                offset: param1 as f32 / 100.0,
            }),
        };

        Some(Self {
            sequence,
            target,
            fault,
        })
    }
}

/// Fault acknowledgment sent from FC to test runner
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FaultAck {
    /// Echoed sequence number
    pub sequence: u16,
    /// Result status
    pub status: AckStatus,
    /// Echoed target (None = all)
    pub target: Option<SensorTarget>,
}

impl FaultAck {
    /// Create a successful acknowledgment
    pub fn ok(cmd: &FaultCommand) -> Self {
        Self {
            sequence: cmd.sequence,
            status: AckStatus::Ok,
            target: cmd.target,
        }
    }

    /// Create an error acknowledgment
    pub fn error(cmd: &FaultCommand, status: AckStatus) -> Self {
        Self {
            sequence: cmd.sequence,
            status,
            target: cmd.target,
        }
    }

    /// Serialize to bytes (little-endian)
    pub fn to_bytes(&self) -> [u8; FAULT_ACK_SIZE] {
        let mut buf = [0u8; FAULT_ACK_SIZE];
        buf[0..2].copy_from_slice(&FAULT_ACK_MAGIC.to_le_bytes());
        buf[2..4].copy_from_slice(&self.sequence.to_le_bytes());
        buf[4] = self.status as u8;
        buf[5] = self.target.map(target_to_u8).unwrap_or(255);
        // buf[6..8] reserved
        buf
    }

    /// Deserialize from bytes (little-endian)
    pub fn from_bytes(buf: &[u8]) -> Option<Self> {
        if buf.len() < FAULT_ACK_SIZE {
            return None;
        }

        let magic = u16::from_le_bytes([buf[0], buf[1]]);
        if magic != FAULT_ACK_MAGIC {
            return None;
        }

        let sequence = u16::from_le_bytes([buf[2], buf[3]]);
        let status = AckStatus::from_u8(buf[4])?;
        let target = if buf[5] == 255 {
            None
        } else {
            Some(target_from_u8(buf[5])?)
        };

        Some(Self {
            sequence,
            status,
            target,
        })
    }

    /// Check if acknowledgment indicates success
    pub fn is_ok(&self) -> bool {
        self.status == AckStatus::Ok
    }
}

/// Fault injection client for test runners
///
/// Sends fault commands to a flight controller and waits for acknowledgment.
///
/// ## Usage
///
/// ```ignore
/// use aviate_hal_xil::{FaultClient, FaultSpec, SensorTarget, XilConfig};
///
/// // Create client for instance 0
/// let config = XilConfig::for_instance(0);
/// let mut client = FaultClient::new(&config)?;
///
/// // Inject a fault
/// let ack = client.inject(SensorTarget::Imu, FaultSpec::HealthDegraded)?;
/// assert!(ack.is_ok());
///
/// // Clear all faults
/// client.clear_all()?;
/// ```
pub struct FaultClient {
    /// UDP socket for sending commands
    socket: std::net::UdpSocket,
    /// Target address (FC fault command port)
    target_addr: std::net::SocketAddr,
    /// Sequence counter
    sequence: u16,
    /// Receive buffer
    buf: [u8; 64],
}

impl FaultClient {
    /// Create a new fault client for the given instance
    ///
    /// # Arguments
    /// * `config` - XIL configuration with instance and port settings
    ///
    /// # Errors
    /// Returns error if socket binding fails
    pub fn new(config: &crate::XilConfig) -> std::io::Result<Self> {
        // Bind to ephemeral port
        let socket = std::net::UdpSocket::bind("127.0.0.1:0")?;
        socket.set_read_timeout(Some(std::time::Duration::from_millis(100)))?;

        // Calculate target port
        let port = config
            .net
            .port(config.instance as u16, crate::PortSlot::FaultCmd);
        let target_addr = std::net::SocketAddr::from(([127, 0, 0, 1], port));

        Ok(Self {
            socket,
            target_addr,
            sequence: 0,
            buf: [0u8; 64],
        })
    }

    /// Inject a fault to a specific sensor
    ///
    /// Sends the command and waits for acknowledgment.
    pub fn inject(&mut self, target: SensorTarget, fault: FaultSpec) -> std::io::Result<FaultAck> {
        let cmd = FaultCommand::new(target, fault).with_sequence(self.next_sequence());
        self.send_and_wait(&cmd)
    }

    /// Clear faults on a specific sensor
    pub fn clear(&mut self, target: SensorTarget) -> std::io::Result<FaultAck> {
        let cmd = FaultCommand::clear(target).with_sequence(self.next_sequence());
        self.send_and_wait(&cmd)
    }

    /// Clear all sensor faults
    pub fn clear_all(&mut self) -> std::io::Result<FaultAck> {
        let cmd = FaultCommand::clear_all().with_sequence(self.next_sequence());
        self.send_and_wait(&cmd)
    }

    /// Send a command and wait for acknowledgment
    fn send_and_wait(&mut self, cmd: &FaultCommand) -> std::io::Result<FaultAck> {
        let bytes = cmd.to_bytes();
        self.socket.send_to(&bytes, self.target_addr)?;

        // Wait for ack (with timeout)
        match self.socket.recv_from(&mut self.buf) {
            Ok((len, _)) => {
                if let Some(ack) = FaultAck::from_bytes(&self.buf[..len]) {
                    if ack.sequence == cmd.sequence {
                        return Ok(ack);
                    }
                }
                // Wrong sequence or parse error, return timeout-like error
                Err(std::io::Error::new(
                    std::io::ErrorKind::InvalidData,
                    "Invalid or mismatched acknowledgment",
                ))
            }
            Err(e) => Err(e),
        }
    }

    /// Get next sequence number
    fn next_sequence(&mut self) -> u16 {
        let seq = self.sequence;
        self.sequence = self.sequence.wrapping_add(1);
        seq
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_fault_command_roundtrip_degraded() {
        let cmd = FaultCommand::new(SensorTarget::Imu, FaultSpec::HealthDegraded).with_sequence(42);

        let bytes = cmd.to_bytes();
        let parsed = FaultCommand::from_bytes(&bytes).expect("should parse");

        assert_eq!(parsed.sequence, 42);
        assert_eq!(parsed.target, Some(SensorTarget::Imu));
        assert_eq!(parsed.fault, Some(FaultSpec::HealthDegraded));
    }

    #[test]
    fn test_fault_command_roundtrip_dropout() {
        let cmd = FaultCommand::new(SensorTarget::Gnss, FaultSpec::Dropout { cycles: 100 });

        let bytes = cmd.to_bytes();
        let parsed = FaultCommand::from_bytes(&bytes).expect("should parse");

        assert_eq!(parsed.target, Some(SensorTarget::Gnss));
        assert_eq!(parsed.fault, Some(FaultSpec::Dropout { cycles: 100 }));
    }

    #[test]
    fn test_fault_command_roundtrip_bias_3axis() {
        let cmd = FaultCommand::new(
            SensorTarget::Imu,
            FaultSpec::BiasShift {
                offset: [1.5, -2.25, 0.5],
            },
        );

        let bytes = cmd.to_bytes();
        let parsed = FaultCommand::from_bytes(&bytes).expect("should parse");

        if let Some(FaultSpec::BiasShift { offset }) = parsed.fault {
            assert!((offset[0] - 1.5).abs() < 0.01);
            assert!((offset[1] - (-2.25)).abs() < 0.01);
            assert!((offset[2] - 0.5).abs() < 0.01);
        } else {
            panic!("Expected BiasShift");
        }
    }

    #[test]
    fn test_fault_command_roundtrip_bias_scalar() {
        let cmd = FaultCommand::new(SensorTarget::Baro, FaultSpec::BiasScalar { offset: 150.5 });

        let bytes = cmd.to_bytes();
        let parsed = FaultCommand::from_bytes(&bytes).expect("should parse");

        if let Some(FaultSpec::BiasScalar { offset }) = parsed.fault {
            assert!((offset - 150.5).abs() < 0.01);
        } else {
            panic!("Expected BiasScalar");
        }
    }

    #[test]
    fn test_fault_command_clear() {
        let cmd = FaultCommand::clear(SensorTarget::Mag);

        let bytes = cmd.to_bytes();
        let parsed = FaultCommand::from_bytes(&bytes).expect("should parse");

        assert_eq!(parsed.target, Some(SensorTarget::Mag));
        assert_eq!(parsed.fault, None);
    }

    #[test]
    fn test_fault_command_clear_all() {
        let cmd = FaultCommand::clear_all();

        let bytes = cmd.to_bytes();
        let parsed = FaultCommand::from_bytes(&bytes).expect("should parse");

        assert_eq!(parsed.target, None);
        assert_eq!(parsed.fault, None);
    }

    #[test]
    fn test_fault_ack_roundtrip() {
        let cmd = FaultCommand::new(SensorTarget::Mag, FaultSpec::HealthFailed).with_sequence(123);
        let ack = FaultAck::ok(&cmd);

        let bytes = ack.to_bytes();
        let parsed = FaultAck::from_bytes(&bytes).expect("should parse");

        assert_eq!(parsed.sequence, 123);
        assert_eq!(parsed.status, AckStatus::Ok);
        assert_eq!(parsed.target, Some(SensorTarget::Mag));
        assert!(parsed.is_ok());
    }

    #[test]
    fn test_fault_ack_error() {
        let cmd = FaultCommand::new(SensorTarget::Baro, FaultSpec::NaN);
        let ack = FaultAck::error(&cmd, AckStatus::NotEnabled);

        assert_eq!(ack.status, AckStatus::NotEnabled);
        assert!(!ack.is_ok());
    }

    #[test]
    fn test_invalid_magic_rejected() {
        let mut bytes = FaultCommand::clear_all().to_bytes();
        bytes[0] = 0xFF; // corrupt magic

        assert!(FaultCommand::from_bytes(&bytes).is_none());
    }

    #[test]
    fn test_invalid_target_rejected() {
        let mut bytes = FaultCommand::clear(SensorTarget::Imu).to_bytes();
        bytes[4] = 100; // invalid target (not 0-3 or 255)

        assert!(FaultCommand::from_bytes(&bytes).is_none());
    }

    #[test]
    fn test_invalid_action_rejected() {
        let mut bytes = FaultCommand::clear_all().to_bytes();
        bytes[5] = 200; // invalid action

        assert!(FaultCommand::from_bytes(&bytes).is_none());
    }

    #[test]
    fn test_short_buffer_rejected() {
        let bytes = [0u8; 4]; // too short

        assert!(FaultCommand::from_bytes(&bytes).is_none());
        assert!(FaultAck::from_bytes(&bytes).is_none());
    }

    #[test]
    fn test_fault_action_from_u8() {
        assert_eq!(FaultAction::from_u8(0), Some(FaultAction::Clear));
        assert_eq!(FaultAction::from_u8(1), Some(FaultAction::HealthDegraded));
        assert_eq!(FaultAction::from_u8(2), Some(FaultAction::HealthFailed));
        assert_eq!(FaultAction::from_u8(3), Some(FaultAction::InjectNaN));
        assert_eq!(FaultAction::from_u8(4), Some(FaultAction::Dropout));
        assert_eq!(FaultAction::from_u8(5), Some(FaultAction::BiasShift));
        assert_eq!(FaultAction::from_u8(6), Some(FaultAction::BiasScalar));
        assert_eq!(FaultAction::from_u8(100), None);
    }

    #[test]
    fn test_ack_status_from_u8() {
        assert_eq!(AckStatus::from_u8(0), Some(AckStatus::Ok));
        assert_eq!(AckStatus::from_u8(1), Some(AckStatus::UnknownTarget));
        assert_eq!(AckStatus::from_u8(2), Some(AckStatus::UnknownAction));
        assert_eq!(AckStatus::from_u8(3), Some(AckStatus::InvalidParams));
        assert_eq!(AckStatus::from_u8(4), Some(AckStatus::NotEnabled));
        assert_eq!(AckStatus::from_u8(100), None);
    }

    #[test]
    fn test_all_targets_roundtrip() {
        for target in [
            SensorTarget::Imu,
            SensorTarget::Baro,
            SensorTarget::Mag,
            SensorTarget::Gnss,
        ] {
            let cmd = FaultCommand::clear(target);
            let bytes = cmd.to_bytes();
            let parsed = FaultCommand::from_bytes(&bytes).expect("should parse");
            assert_eq!(parsed.target, Some(target));
        }
    }

    #[test]
    fn test_all_faults_roundtrip() {
        let faults = [
            FaultSpec::HealthDegraded,
            FaultSpec::HealthFailed,
            FaultSpec::NaN,
            FaultSpec::Dropout { cycles: 50 },
            FaultSpec::BiasShift {
                offset: [1.0, 2.0, 3.0],
            },
            FaultSpec::BiasScalar { offset: -100.0 },
        ];

        for fault in faults {
            let cmd = FaultCommand::new(SensorTarget::Imu, fault.clone());
            let bytes = cmd.to_bytes();
            let parsed = FaultCommand::from_bytes(&bytes).expect("should parse");

            match (&fault, &parsed.fault) {
                (
                    FaultSpec::BiasShift { offset: o1 },
                    Some(FaultSpec::BiasShift { offset: o2 }),
                ) => {
                    // Compare with precision loss tolerance
                    for i in 0..3 {
                        assert!((o1[i] - o2[i]).abs() < 0.01);
                    }
                }
                (
                    FaultSpec::BiasScalar { offset: o1 },
                    Some(FaultSpec::BiasScalar { offset: o2 }),
                ) => {
                    assert!((o1 - o2).abs() < 0.01);
                }
                _ => {
                    assert_eq!(parsed.fault, Some(fault));
                }
            }
        }
    }

    #[test]
    fn test_fault_client_creation() {
        let config = crate::XilConfig::for_instance(95);
        let client = FaultClient::new(&config);
        assert!(client.is_ok());
    }
}
