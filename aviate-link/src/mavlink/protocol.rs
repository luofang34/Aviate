//! MAVLink 2.0 Protocol Implementation
//!
//! This module contains the complete MAVLink protocol specification:
//! - Message type definitions
//! - Binary codec (parser + serializer)
//! - Protocol constants and enums
//!
//! ## DO-178C Separation of Concerns
//!
//! This module is **protocol-only** (pure translation between bytes and structs).
//! It does NOT contain:
//! - ❌ Aviate domain types (StateEstimate, ChannelStatus, etc.)
//! - ❌ Link-layer logic (TelemetryBackend, CommandLink)
//! - ❌ Security (signatures, authentication, anti-replay)
//!
//! ## Audit Checklist
//!
//! When auditing this file, verify:
//! - ✅ All functions are pure translation (no side effects)
//! - ✅ No imports from aviate-core, aviate-security
//! - ✅ No I/O operations (only byte buffer manipulation)
//! - ✅ All parsing has explicit error handling
//!
//! ## Usage
//!
//! This module is used by `telemetry.rs` and `command.rs` for protocol translation.
//! Applications should NOT use this module directly - use the link layer instead.

// ============================================================================
// CONSTANTS
// ============================================================================

/// MAVLink 2.0 start byte
pub const MAVLINK_STX_V2: u8 = 0xFD;

/// MAVLink 1.0 start byte (for compatibility detection)
pub const MAVLINK_STX_V1: u8 = 0xFE;

/// Maximum MAVLink message payload size
pub const MAX_PAYLOAD_LEN: usize = 255;

/// System ID for Aviate autopilot
pub const AVIATE_SYSTEM_ID: u8 = 1;

/// Component ID for Aviate autopilot
pub const AVIATE_COMPONENT_ID: u8 = 1;

/// MAVLink 2.0 signature length (13 bytes)
pub const MAVLINK_SIGNATURE_LEN: usize = 13;

/// MAVLink 2.0 incompatibility flag: MAVLINK_IFLAG_SIGNED
///
/// When bit 0 of the incompatibility flags byte is set, the frame
/// includes a 13-byte signature extension after the CRC.
pub const MAVLINK_IFLAG_SIGNED: u8 = 0x01;

// ============================================================================
// ENUMS AND TYPES
// ============================================================================

/// MAVLink component types
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum MavComponent {
    Autopilot = 1,
    Camera = 100,
    Gimbal = 154,
    Gcs = 190,
}

/// MAVLink autopilot types
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum MavAutopilot {
    Generic = 0,
    Px4 = 12,
    Ardupilot = 3,
    Aviate = 18, // Custom ID for Aviate
}

/// MAVLink system type
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum MavType {
    Generic = 0,
    FixedWing = 1,
    Quadrotor = 2,
    Coaxial = 3,
    Helicopter = 4,
    Hexarotor = 13,
    Octorotor = 14,
    Vtol = 19,
    Gcs = 6,
}

/// MAVLink system state
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum MavState {
    Uninit = 0,
    Boot = 1,
    Calibrating = 2,
    Standby = 3,
    Active = 4,
    Critical = 5,
    Emergency = 6,
    Poweroff = 7,
    FlightTermination = 8,
}

/// MAVLink mode flags
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct MavModeFlag(pub u8);

impl MavModeFlag {
    pub const CUSTOM_MODE_ENABLED: Self = Self(1);
    pub const TEST_ENABLED: Self = Self(2);
    pub const AUTO_ENABLED: Self = Self(4);
    pub const GUIDED_ENABLED: Self = Self(8);
    pub const STABILIZE_ENABLED: Self = Self(16);
    pub const HIL_ENABLED: Self = Self(32);
    pub const MANUAL_INPUT_ENABLED: Self = Self(64);
    pub const SAFETY_ARMED: Self = Self(128);
}

/// MAV_CMD constants
pub mod mav_cmd {
    pub const NAV_LAND: u16 = 21;
    pub const NAV_TAKEOFF: u16 = 22;
    pub const DO_SET_MODE: u16 = 176;
    pub const COMPONENT_ARM_DISARM: u16 = 400;
}

/// MAV_RESULT constants
pub mod mav_result {
    pub const ACCEPTED: u8 = 0;
    pub const TEMPORARILY_REJECTED: u8 = 1;
    pub const DENIED: u8 = 2;
    pub const UNSUPPORTED: u8 = 3;
    pub const FAILED: u8 = 4;
    pub const IN_PROGRESS: u8 = 5;
    pub const CANCELLED: u8 = 6;
}

/// MAVLink 2.0 signature block
///
/// Appears after the CRC when the MAVLINK_IFLAG_SIGNED flag is set.
/// Contains link_id, timestamp, and truncated HMAC-SHA256 signature.
///
/// ## Structure (13 bytes total)
///
/// ```text
/// [link_id(1)][timestamp(6)][signature(6)]
/// ```
///
/// - `link_id`: Identifies which key to use (0-255)
/// - `timestamp`: 48-bit timestamp in 10 microsecond units (monotonic counter)
/// - `signature`: First 6 bytes of HMAC-SHA256(secret_key, message)
///
/// ## Security Model
///
/// Per MAVLink message signing spec:
/// - Signature covers: [header + payload + CRC + link_id + timestamp]
/// - Timestamp must be monotonically increasing per link_id
/// - 48-bit timestamp wraps every ~89 years at 10μs resolution
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct MavSignature {
    /// Link identifier (maps to key in KeyStore)
    pub link_id: u8,

    /// 48-bit timestamp (10 microsecond resolution)
    ///
    /// This is a remote monotonic counter, NOT a wall clock time.
    /// Receiver must track per-link_id and reject if counter <= last_seen.
    pub timestamp: u64,

    /// Truncated HMAC-SHA256 signature (first 6 bytes)
    pub signature: [u8; 6],
}

// ============================================================================
// MESSAGE DEFINITIONS
// ============================================================================

// MAVLink message definitions for Aviate autopilot

/// HEARTBEAT (MAVLink #0)
#[derive(Copy, Clone, Debug, Default)]
pub struct Heartbeat {
    pub mav_type: u8,
    pub autopilot: u8,
    pub base_mode: u8,
    pub custom_mode: u32,
    pub system_status: u8,
    pub mavlink_version: u8,
}

impl Heartbeat {
    pub const MSG_ID: u32 = 0;
    pub const PAYLOAD_LEN: usize = 9;
}

/// SYSTEM_TIME (MAVLink #2)
#[derive(Copy, Clone, Debug, Default)]
pub struct SystemTime {
    pub time_unix_usec: u64,
    pub time_boot_ms: u32,
}

impl SystemTime {
    pub const MSG_ID: u32 = 2;
    pub const PAYLOAD_LEN: usize = 12;
}

/// ATTITUDE_QUATERNION (MAVLink #31)
#[derive(Copy, Clone, Debug, Default)]
pub struct AttitudeQuaternion {
    pub time_boot_ms: u32,
    pub q1: f32,
    pub q2: f32,
    pub q3: f32,
    pub q4: f32,
    pub rollspeed: f32,
    pub pitchspeed: f32,
    pub yawspeed: f32,
    pub repr_offset_q: [f32; 4],
}

impl AttitudeQuaternion {
    pub const MSG_ID: u32 = 31;
    pub const PAYLOAD_LEN: usize = 48;
}

/// LOCAL_POSITION_NED (MAVLink #32)
#[derive(Copy, Clone, Debug, Default)]
pub struct LocalPositionNed {
    pub time_boot_ms: u32,
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub vx: f32,
    pub vy: f32,
    pub vz: f32,
}

impl LocalPositionNed {
    pub const MSG_ID: u32 = 32;
    pub const PAYLOAD_LEN: usize = 28;
}

/// SET_ATTITUDE_TARGET (MAVLink #82)
#[derive(Copy, Clone, Debug, Default)]
pub struct SetAttitudeTarget {
    pub time_boot_ms: u32,
    pub target_system: u8,
    pub target_component: u8,
    pub type_mask: u8,
    pub q: [f32; 4],
    pub body_roll_rate: f32,
    pub body_pitch_rate: f32,
    pub body_yaw_rate: f32,
    pub thrust: f32,
    pub thrust_body: [f32; 3],
}

impl SetAttitudeTarget {
    pub const MSG_ID: u32 = 82;
    pub const PAYLOAD_LEN: usize = 51;
}

pub mod attitude_target_typemask {
    pub const BODY_ROLL_RATE_IGNORE: u8 = 1;
    pub const BODY_PITCH_RATE_IGNORE: u8 = 2;
    pub const BODY_YAW_RATE_IGNORE: u8 = 4;
    pub const THRUST_BODY_SET: u8 = 32;
    pub const THROTTLE_IGNORE: u8 = 64;
    pub const ATTITUDE_IGNORE: u8 = 128;
}

/// SET_POSITION_TARGET_LOCAL_NED (MAVLink #84)
/// Sets position/velocity/acceleration setpoints in local NED frame
#[derive(Copy, Clone, Debug, Default)]
pub struct SetPositionTargetLocalNed {
    pub time_boot_ms: u32,
    pub target_system: u8,
    pub target_component: u8,
    pub coordinate_frame: u8,
    pub type_mask: u16,
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub vx: f32,
    pub vy: f32,
    pub vz: f32,
    pub afx: f32,
    pub afy: f32,
    pub afz: f32,
    pub yaw: f32,
    pub yaw_rate: f32,
}

impl SetPositionTargetLocalNed {
    pub const MSG_ID: u32 = 84;
    pub const PAYLOAD_LEN: usize = 53;
}

/// Type mask bits for SET_POSITION_TARGET_LOCAL_NED
pub mod position_target_typemask {
    pub const X_IGNORE: u16 = 1;
    pub const Y_IGNORE: u16 = 2;
    pub const Z_IGNORE: u16 = 4;
    pub const VX_IGNORE: u16 = 8;
    pub const VY_IGNORE: u16 = 16;
    pub const VZ_IGNORE: u16 = 32;
    pub const AX_IGNORE: u16 = 64;
    pub const AY_IGNORE: u16 = 128;
    pub const AZ_IGNORE: u16 = 256;
    pub const FORCE_SET: u16 = 512;
    pub const YAW_IGNORE: u16 = 1024;
    pub const YAW_RATE_IGNORE: u16 = 2048;
}

/// COMMAND_LONG (MAVLink #76)
#[derive(Copy, Clone, Debug, Default)]
pub struct CommandLong {
    pub param1: f32,
    pub param2: f32,
    pub param3: f32,
    pub param4: f32,
    pub param5: f32,
    pub param6: f32,
    pub param7: f32,
    pub command: u16,
    pub target_system: u8,
    pub target_component: u8,
    pub confirmation: u8,
}

impl CommandLong {
    pub const MSG_ID: u32 = 76;
    pub const PAYLOAD_LEN: usize = 33;
}

/// COMMAND_ACK (MAVLink #77)
#[derive(Copy, Clone, Debug, Default)]
pub struct CommandAck {
    pub command: u16,
    pub result: u8,
    pub progress: u8,
    pub result_param2: i32,
    pub target_system: u8,
    pub target_component: u8,
}

impl CommandAck {
    pub const MSG_ID: u32 = 77;
    pub const PAYLOAD_LEN: usize = 10;
}

// --- Additional Messages ---

/// RC_CHANNELS_OVERRIDE (MAVLink #70)
#[derive(Copy, Clone, Debug, Default)]
pub struct RcChannelsOverride {
    pub chan1_raw: u16,
    pub chan2_raw: u16,
    pub chan3_raw: u16,
    pub chan4_raw: u16,
    pub chan5_raw: u16,
    pub chan6_raw: u16,
    pub chan7_raw: u16,
    pub chan8_raw: u16,
    pub target_system: u8,
    pub target_component: u8,
    pub chan9_raw: u16,
    pub chan10_raw: u16,
    pub chan11_raw: u16,
    pub chan12_raw: u16,
    pub chan13_raw: u16,
    pub chan14_raw: u16,
    pub chan15_raw: u16,
    pub chan16_raw: u16,
    pub chan17_raw: u16,
    pub chan18_raw: u16,
}

impl RcChannelsOverride {
    pub const MSG_ID: u32 = 70;
    pub const PAYLOAD_LEN: usize = 38; // 18 basic + 20 extension
}

/// MANUAL_CONTROL (MAVLink #69)
#[derive(Copy, Clone, Debug, Default)]
pub struct ManualControl {
    pub x: i16,
    pub y: i16,
    pub z: i16,
    pub r: i16,
    pub buttons: u16,
    pub target: u8,
    pub buttons2: u16,
    pub enabled_extensions: u8,
    pub s: i16,
    pub t: i16,
    pub aux1: i16,
    pub aux2: i16,
    pub aux3: i16,
    pub aux4: i16,
    pub aux5: i16,
    pub aux6: i16,
}

impl ManualControl {
    pub const MSG_ID: u32 = 69;
    pub const PAYLOAD_LEN: usize = 30; // 11 basic + 19 extension
}

/// SYS_STATUS (MAVLink #1)
#[derive(Copy, Clone, Debug, Default)]
pub struct SysStatus {
    pub onboard_control_sensors_present: u32,
    pub onboard_control_sensors_enabled: u32,
    pub onboard_control_sensors_health: u32,
    pub load: u16,
    pub voltage_battery: u16,
    pub current_battery: i16,
    pub drop_rate_comm: u16,
    pub errors_comm: u16,
    pub errors_count1: u16,
    pub errors_count2: u16,
    pub errors_count3: u16,
    pub errors_count4: u16,
    pub battery_remaining: i8,
    pub onboard_control_sensors_present_extended: u32,
    pub onboard_control_sensors_enabled_extended: u32,
    pub onboard_control_sensors_health_extended: u32,
}

impl SysStatus {
    pub const MSG_ID: u32 = 1;
    pub const PAYLOAD_LEN: usize = 43; // 31 basic + 12 extension
}

/// STATUSTEXT (MAVLink #253)
#[derive(Copy, Clone, Debug)]
pub struct Statustext {
    pub severity: u8,
    pub text: [u8; 50],
    pub id: u16,
    pub chunk_seq: u8,
}

impl Default for Statustext {
    fn default() -> Self {
        Self {
            severity: 0,
            text: [0; 50],
            id: 0,
            chunk_seq: 0,
        }
    }
}

impl Statustext {
    pub const MSG_ID: u32 = 253;
    pub const PAYLOAD_LEN: usize = 54; // 51 basic + 3 extension
}

/// Enum of all supported MAVLink messages
#[derive(Copy, Clone, Debug)]
pub enum MavMessage {
    Heartbeat(Heartbeat),
    SystemTime(SystemTime),
    AttitudeQuaternion(AttitudeQuaternion),
    LocalPositionNed(LocalPositionNed),
    SetAttitudeTarget(SetAttitudeTarget),
    SetPositionTargetLocalNed(SetPositionTargetLocalNed),
    CommandLong(CommandLong),
    CommandAck(CommandAck),
    RcChannelsOverride(RcChannelsOverride),
    ManualControl(ManualControl),
    SysStatus(SysStatus),
    Statustext(Statustext),

    Unknown { msg_id: u32 },
}

impl MavMessage {
    /// Get the message ID
    pub fn msg_id(&self) -> u32 {
        match self {
            MavMessage::Heartbeat(_) => Heartbeat::MSG_ID,
            MavMessage::SystemTime(_) => SystemTime::MSG_ID,
            MavMessage::AttitudeQuaternion(_) => AttitudeQuaternion::MSG_ID,
            MavMessage::LocalPositionNed(_) => LocalPositionNed::MSG_ID,
            MavMessage::SetAttitudeTarget(_) => SetAttitudeTarget::MSG_ID,
            MavMessage::SetPositionTargetLocalNed(_) => SetPositionTargetLocalNed::MSG_ID,
            MavMessage::CommandLong(_) => CommandLong::MSG_ID,
            MavMessage::CommandAck(_) => CommandAck::MSG_ID,
            MavMessage::RcChannelsOverride(_) => RcChannelsOverride::MSG_ID,
            MavMessage::ManualControl(_) => ManualControl::MSG_ID,
            MavMessage::SysStatus(_) => SysStatus::MSG_ID,
            MavMessage::Statustext(_) => Statustext::MSG_ID,
            MavMessage::Unknown { msg_id } => *msg_id,
        }
    }
}

// ============================================================================
// PARSER (Bytes → MAVLink Messages)
// ============================================================================

// MAVLink message parser
//
// Parses MAVLink 2.0 frames from byte buffers.

/// Parse error types
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum ParseError {
    /// Buffer too short
    BufferTooShort,
    /// Invalid start byte
    InvalidStartByte,
    /// Payload length exceeds maximum
    PayloadTooLong,
    /// CRC mismatch
    CrcMismatch,
    /// Unsupported message ID
    UnsupportedMessage(u32),
    /// Invalid payload for message type
    InvalidPayload,
}

/// MAVLink 2.0 frame header
#[derive(Copy, Clone, Debug)]
pub struct MavHeader {
    pub payload_len: u8,
    pub incompat_flags: u8,
    pub compat_flags: u8,
    pub seq: u8,
    pub sysid: u8,
    pub compid: u8,
    pub msgid: u32, // 24-bit in wire format
}

impl MavHeader {
    pub const SIZE: usize = 10; // STX + 9 header bytes
}

/// Parse a MAVLink message from a byte buffer
///
/// Returns the parsed message, optional signature, and the number of bytes consumed.
///
/// ## MAVLink 2.0 Payload Truncation
///
/// Compliant senders strip trailing zero bytes from the payload. The CRC
/// is verified against the on-wire (possibly truncated) payload; the
/// omitted tail bytes then decode as zeros. Payload bytes beyond the
/// declared layout of a known message are unknown extensions and are
/// ignored.
///
/// ## MAVLink 2.0 Signature Support
///
/// When the MAVLINK_IFLAG_SIGNED flag is set, the frame includes a 13-byte
/// signature extension after the CRC. This function extracts the signature
/// metadata but does NOT verify it - verification happens in `aviate-security`.
pub fn parse_mavlink(buf: &[u8]) -> Result<(MavMessage, Option<MavSignature>, usize), ParseError> {
    // Minimum frame: STX(1) + header(9) + payload(0) + checksum(2) = 12
    if buf.len() < 12 {
        return Err(ParseError::BufferTooShort);
    }

    // Check start byte
    if buf[0] != MAVLINK_STX_V2 {
        return Err(ParseError::InvalidStartByte);
    }

    // Parse header
    let payload_len = buf[1] as usize;
    if payload_len > MAX_PAYLOAD_LEN {
        return Err(ParseError::PayloadTooLong);
    }

    let header = MavHeader {
        payload_len: buf[1],
        incompat_flags: buf[2],
        compat_flags: buf[3],
        seq: buf[4],
        sysid: buf[5],
        compid: buf[6],
        msgid: (buf[7] as u32) | ((buf[8] as u32) << 8) | ((buf[9] as u32) << 16),
    };

    // Check if frame is signed
    let is_signed = (header.incompat_flags & MAVLINK_IFLAG_SIGNED) != 0;

    // Calculate total frame size (including signature if present)
    let mut frame_size = MavHeader::SIZE + payload_len + 2; // +2 for CRC
    if is_signed {
        frame_size += MAVLINK_SIGNATURE_LEN;
    }

    if buf.len() < frame_size {
        return Err(ParseError::BufferTooShort);
    }

    // Extract payload slice
    let payload = &buf[MavHeader::SIZE..MavHeader::SIZE + payload_len];

    // Verify CRC (X.25)
    let crc_offset = MavHeader::SIZE + payload_len;
    let received_crc = (buf[crc_offset] as u16) | ((buf[crc_offset + 1] as u16) << 8);
    let computed_crc = compute_crc(&buf[1..crc_offset], get_crc_extra(header.msgid));

    if received_crc != computed_crc {
        return Err(ParseError::CrcMismatch);
    }

    // Extract signature if present
    let signature = if is_signed {
        let sig_offset = crc_offset + 2; // After CRC
        Some(parse_signature(
            &buf[sig_offset..sig_offset + MAVLINK_SIGNATURE_LEN],
        ))
    } else {
        None
    };

    // MAVLink 2 removes trailing zero payload bytes on the wire; restore
    // them before field decoding. Bytes beyond the declared layout belong
    // to extensions this dialect does not know and are ignored.
    let msg = if let Some(declared) = declared_payload_len(header.msgid) {
        // The truncation rule always retains at least one payload byte.
        if payload.is_empty() {
            return Err(ParseError::InvalidPayload);
        }
        let mut full = [0u8; MAX_PAYLOAD_LEN];
        let copy_len = payload.len().min(declared);
        full[..copy_len].copy_from_slice(&payload[..copy_len]);
        parse_message_payload(header.msgid, &full[..declared])?
    } else {
        parse_message_payload(header.msgid, payload)?
    };

    Ok((msg, signature, frame_size))
}

/// Parse MAVLink 2.0 signature block (13 bytes)
///
/// ## Format
///
/// ```text
/// [link_id(1)][timestamp(6)][signature(6)]
/// ```
///
/// - timestamp: 48-bit little-endian (10 microsecond resolution)
/// - signature: First 6 bytes of HMAC-SHA256
fn parse_signature(buf: &[u8]) -> MavSignature {
    debug_assert!(buf.len() >= MAVLINK_SIGNATURE_LEN);

    let link_id = buf[0];

    // Parse 48-bit timestamp (6 bytes, little-endian)
    let timestamp = (buf[1] as u64)
        | ((buf[2] as u64) << 8)
        | ((buf[3] as u64) << 16)
        | ((buf[4] as u64) << 24)
        | ((buf[5] as u64) << 32)
        | ((buf[6] as u64) << 40);

    // Extract 6-byte signature
    let mut signature = [0u8; 6];
    signature.copy_from_slice(&buf[7..13]);

    MavSignature {
        link_id,
        timestamp,
        signature,
    }
}

/// Parse message payload based on message ID
fn parse_message_payload(msg_id: u32, payload: &[u8]) -> Result<MavMessage, ParseError> {
    match msg_id {
        Heartbeat::MSG_ID => parse_heartbeat(payload),
        SystemTime::MSG_ID => parse_system_time(payload),
        AttitudeQuaternion::MSG_ID => parse_attitude_quaternion(payload),
        LocalPositionNed::MSG_ID => parse_local_position_ned(payload),
        SetAttitudeTarget::MSG_ID => parse_set_attitude_target(payload),
        SetPositionTargetLocalNed::MSG_ID => parse_set_position_target_local_ned(payload),
        CommandLong::MSG_ID => parse_command_long(payload),
        CommandAck::MSG_ID => parse_command_ack(payload),
        RcChannelsOverride::MSG_ID => parse_rc_channels_override(payload),
        ManualControl::MSG_ID => parse_manual_control(payload),
        SysStatus::MSG_ID => parse_sys_status(payload),
        Statustext::MSG_ID => parse_statustext(payload),
        _ => Ok(MavMessage::Unknown { msg_id }),
    }
}

fn parse_heartbeat(payload: &[u8]) -> Result<MavMessage, ParseError> {
    if payload.len() < Heartbeat::PAYLOAD_LEN {
        return Err(ParseError::InvalidPayload);
    }

    Ok(MavMessage::Heartbeat(Heartbeat {
        custom_mode: read_u32_le(payload, 0),
        mav_type: payload[4],
        autopilot: payload[5],
        base_mode: payload[6],
        system_status: payload[7],
        mavlink_version: payload[8],
    }))
}

fn parse_system_time(payload: &[u8]) -> Result<MavMessage, ParseError> {
    if payload.len() < SystemTime::PAYLOAD_LEN {
        return Err(ParseError::InvalidPayload);
    }

    Ok(MavMessage::SystemTime(SystemTime {
        time_unix_usec: read_u64_le(payload, 0),
        time_boot_ms: read_u32_le(payload, 8),
    }))
}

fn parse_attitude_quaternion(payload: &[u8]) -> Result<MavMessage, ParseError> {
    if payload.len() < 32 {
        // Basic payload
        return Err(ParseError::InvalidPayload);
    }

    let mut offset_q = [0.0; 4];
    if payload.len() >= 48 {
        offset_q[0] = read_f32_le(payload, 32);
        offset_q[1] = read_f32_le(payload, 36);
        offset_q[2] = read_f32_le(payload, 40);
        offset_q[3] = read_f32_le(payload, 44);
    }

    Ok(MavMessage::AttitudeQuaternion(AttitudeQuaternion {
        time_boot_ms: read_u32_le(payload, 0),
        q1: read_f32_le(payload, 4),
        q2: read_f32_le(payload, 8),
        q3: read_f32_le(payload, 12),
        q4: read_f32_le(payload, 16),
        rollspeed: read_f32_le(payload, 20),
        pitchspeed: read_f32_le(payload, 24),
        yawspeed: read_f32_le(payload, 28),
        repr_offset_q: offset_q,
    }))
}

fn parse_local_position_ned(payload: &[u8]) -> Result<MavMessage, ParseError> {
    if payload.len() < LocalPositionNed::PAYLOAD_LEN {
        return Err(ParseError::InvalidPayload);
    }

    Ok(MavMessage::LocalPositionNed(LocalPositionNed {
        time_boot_ms: read_u32_le(payload, 0),
        x: read_f32_le(payload, 4),
        y: read_f32_le(payload, 8),
        z: read_f32_le(payload, 12),
        vx: read_f32_le(payload, 16),
        vy: read_f32_le(payload, 20),
        vz: read_f32_le(payload, 24),
    }))
}

fn parse_set_attitude_target(payload: &[u8]) -> Result<MavMessage, ParseError> {
    if payload.len() < 39 {
        // Basic payload
        return Err(ParseError::InvalidPayload);
    }

    let mut thrust_body = [0.0; 3];
    if payload.len() >= 51 {
        thrust_body[0] = read_f32_le(payload, 39);
        thrust_body[1] = read_f32_le(payload, 43);
        thrust_body[2] = read_f32_le(payload, 47);
    }

    Ok(MavMessage::SetAttitudeTarget(SetAttitudeTarget {
        time_boot_ms: read_u32_le(payload, 0),
        q: [
            read_f32_le(payload, 4),
            read_f32_le(payload, 8),
            read_f32_le(payload, 12),
            read_f32_le(payload, 16),
        ],
        body_roll_rate: read_f32_le(payload, 20),
        body_pitch_rate: read_f32_le(payload, 24),
        body_yaw_rate: read_f32_le(payload, 28),
        thrust: read_f32_le(payload, 32),
        target_system: payload[36],
        target_component: payload[37],
        type_mask: payload[38],
        thrust_body,
    }))
}

fn parse_set_position_target_local_ned(payload: &[u8]) -> Result<MavMessage, ParseError> {
    // MAVLink wire format: 14 floats (56 bytes) + u32 (4) + u16 (2) + 3 u8s (3) = 51 bytes minimum
    // Actual layout: time_boot_ms(4) + x/y/z/vx/vy/vz/afx/afy/afz/yaw/yaw_rate(44) + type_mask(2) + target_system(1) + target_component(1) + coordinate_frame(1) = 53
    if payload.len() < 51 {
        return Err(ParseError::InvalidPayload);
    }

    Ok(MavMessage::SetPositionTargetLocalNed(
        SetPositionTargetLocalNed {
            time_boot_ms: read_u32_le(payload, 0),
            x: read_f32_le(payload, 4),
            y: read_f32_le(payload, 8),
            z: read_f32_le(payload, 12),
            vx: read_f32_le(payload, 16),
            vy: read_f32_le(payload, 20),
            vz: read_f32_le(payload, 24),
            afx: read_f32_le(payload, 28),
            afy: read_f32_le(payload, 32),
            afz: read_f32_le(payload, 36),
            yaw: read_f32_le(payload, 40),
            yaw_rate: read_f32_le(payload, 44),
            type_mask: read_u16_le(payload, 48),
            target_system: payload[50],
            target_component: if payload.len() > 51 { payload[51] } else { 0 },
            coordinate_frame: if payload.len() > 52 { payload[52] } else { 1 }, // Default to LOCAL_NED
        },
    ))
}

fn parse_command_long(payload: &[u8]) -> Result<MavMessage, ParseError> {
    // MAVLink 2.0 trims trailing zeros - minimum is 32 bytes (without confirmation)
    if payload.len() < 32 {
        return Err(ParseError::InvalidPayload);
    }

    Ok(MavMessage::CommandLong(CommandLong {
        param1: read_f32_le(payload, 0),
        param2: read_f32_le(payload, 4),
        param3: read_f32_le(payload, 8),
        param4: read_f32_le(payload, 12),
        param5: read_f32_le(payload, 16),
        param6: read_f32_le(payload, 20),
        param7: read_f32_le(payload, 24),
        command: read_u16_le(payload, 28),
        target_system: payload[30],
        target_component: payload[31],
        confirmation: if payload.len() > 32 { payload[32] } else { 0 },
    }))
}

fn parse_command_ack(payload: &[u8]) -> Result<MavMessage, ParseError> {
    if payload.len() < 3 {
        // Basic: command(2) + result(1)
        return Err(ParseError::InvalidPayload);
    }

    let progress = if payload.len() > 3 { payload[3] } else { 0 };
    let result_param2 = if payload.len() > 7 {
        read_i32_le(payload, 4)
    } else {
        0
    };
    let target_system = if payload.len() > 8 { payload[8] } else { 0 };
    let target_component = if payload.len() > 9 { payload[9] } else { 0 };

    Ok(MavMessage::CommandAck(CommandAck {
        command: read_u16_le(payload, 0),
        result: payload[2],
        progress,
        result_param2,
        target_system,
        target_component,
    }))
}

fn parse_rc_channels_override(payload: &[u8]) -> Result<MavMessage, ParseError> {
    if payload.len() < 18 {
        // Basic
        return Err(ParseError::InvalidPayload);
    }

    let chan9_18_start = 18;
    let mut msg = RcChannelsOverride {
        chan1_raw: read_u16_le(payload, 0),
        chan2_raw: read_u16_le(payload, 2),
        chan3_raw: read_u16_le(payload, 4),
        chan4_raw: read_u16_le(payload, 6),
        chan5_raw: read_u16_le(payload, 8),
        chan6_raw: read_u16_le(payload, 10),
        chan7_raw: read_u16_le(payload, 12),
        chan8_raw: read_u16_le(payload, 14),
        target_system: payload[16],
        target_component: payload[17],
        ..Default::default()
    };

    if payload.len() >= 38 {
        msg.chan9_raw = read_u16_le(payload, chan9_18_start);
        msg.chan10_raw = read_u16_le(payload, chan9_18_start + 2);
        msg.chan11_raw = read_u16_le(payload, chan9_18_start + 4);
        msg.chan12_raw = read_u16_le(payload, chan9_18_start + 6);
        msg.chan13_raw = read_u16_le(payload, chan9_18_start + 8);
        msg.chan14_raw = read_u16_le(payload, chan9_18_start + 10);
        msg.chan15_raw = read_u16_le(payload, chan9_18_start + 12);
        msg.chan16_raw = read_u16_le(payload, chan9_18_start + 14);
        msg.chan17_raw = read_u16_le(payload, chan9_18_start + 16);
        msg.chan18_raw = read_u16_le(payload, chan9_18_start + 18);
    }

    Ok(MavMessage::RcChannelsOverride(msg))
}

fn parse_manual_control(payload: &[u8]) -> Result<MavMessage, ParseError> {
    if payload.len() < 11 {
        // Basic
        return Err(ParseError::InvalidPayload);
    }

    let mut msg = ManualControl {
        x: read_i16_le(payload, 0),
        y: read_i16_le(payload, 2),
        z: read_i16_le(payload, 4),
        r: read_i16_le(payload, 6),
        buttons: read_u16_le(payload, 8),
        target: payload[10],
        ..Default::default()
    };

    // Extensions follow in common.xml declaration order.
    if payload.len() >= 13 {
        msg.buttons2 = read_u16_le(payload, 11);
    }
    if payload.len() >= 14 {
        msg.enabled_extensions = payload[13];
    }
    if payload.len() >= 16 {
        msg.s = read_i16_le(payload, 14);
    }
    if payload.len() >= 18 {
        msg.t = read_i16_le(payload, 16);
    }
    if payload.len() >= 20 {
        msg.aux1 = read_i16_le(payload, 18);
    }
    if payload.len() >= 22 {
        msg.aux2 = read_i16_le(payload, 20);
    }
    if payload.len() >= 24 {
        msg.aux3 = read_i16_le(payload, 22);
    }
    if payload.len() >= 26 {
        msg.aux4 = read_i16_le(payload, 24);
    }
    if payload.len() >= 28 {
        msg.aux5 = read_i16_le(payload, 26);
    }
    if payload.len() >= 30 {
        msg.aux6 = read_i16_le(payload, 28);
    }

    Ok(MavMessage::ManualControl(msg))
}

fn parse_sys_status(payload: &[u8]) -> Result<MavMessage, ParseError> {
    if payload.len() < 31 {
        // Basic (43 in messages.rs but check wire size)
        return Err(ParseError::InvalidPayload);
    }

    let mut msg = SysStatus {
        onboard_control_sensors_present: read_u32_le(payload, 0),
        onboard_control_sensors_enabled: read_u32_le(payload, 4),
        onboard_control_sensors_health: read_u32_le(payload, 8),
        load: read_u16_le(payload, 12),
        voltage_battery: read_u16_le(payload, 14),
        current_battery: read_i16_le(payload, 16),
        drop_rate_comm: read_u16_le(payload, 18),
        errors_comm: read_u16_le(payload, 20),
        errors_count1: read_u16_le(payload, 22),
        errors_count2: read_u16_le(payload, 24),
        errors_count3: read_u16_le(payload, 26),
        errors_count4: read_u16_le(payload, 28),
        battery_remaining: payload[30] as i8,
        ..Default::default()
    };

    if payload.len() >= 43 {
        msg.onboard_control_sensors_present_extended = read_u32_le(payload, 31);
        msg.onboard_control_sensors_enabled_extended = read_u32_le(payload, 35);
        msg.onboard_control_sensors_health_extended = read_u32_le(payload, 39);
    }

    Ok(MavMessage::SysStatus(msg))
}

fn parse_statustext(payload: &[u8]) -> Result<MavMessage, ParseError> {
    if payload.len() < 51 {
        // Basic
        return Err(ParseError::InvalidPayload);
    }

    let mut text = [0u8; 50];
    text.copy_from_slice(&payload[1..51]);

    let mut msg = Statustext {
        severity: payload[0],
        text,
        id: 0,
        chunk_seq: 0,
    };

    if payload.len() >= 54 {
        msg.id = read_u16_le(payload, 51);
        msg.chunk_seq = payload[53];
    }

    Ok(MavMessage::Statustext(msg))
}

// Byte reading helpers (little-endian)
fn read_u16_le(buf: &[u8], offset: usize) -> u16 {
    (buf[offset] as u16) | ((buf[offset + 1] as u16) << 8)
}

fn read_i16_le(buf: &[u8], offset: usize) -> i16 {
    read_u16_le(buf, offset) as i16
}

fn read_u32_le(buf: &[u8], offset: usize) -> u32 {
    (buf[offset] as u32)
        | ((buf[offset + 1] as u32) << 8)
        | ((buf[offset + 2] as u32) << 16)
        | ((buf[offset + 3] as u32) << 24)
}

fn read_i32_le(buf: &[u8], offset: usize) -> i32 {
    read_u32_le(buf, offset) as i32
}

fn read_u64_le(buf: &[u8], offset: usize) -> u64 {
    (buf[offset] as u64)
        | ((buf[offset + 1] as u64) << 8)
        | ((buf[offset + 2] as u64) << 16)
        | ((buf[offset + 3] as u64) << 24)
        | ((buf[offset + 4] as u64) << 32)
        | ((buf[offset + 5] as u64) << 40)
        | ((buf[offset + 6] as u64) << 48)
        | ((buf[offset + 7] as u64) << 56)
}

fn read_f32_le(buf: &[u8], offset: usize) -> f32 {
    f32::from_bits(read_u32_le(buf, offset))
}

/// X.25 CRC calculation for MAVLink
fn compute_crc(data: &[u8], crc_extra: u8) -> u16 {
    let mut crc: u16 = 0xFFFF;

    for &byte in data {
        crc = crc_accumulate(byte, crc);
    }

    // Include CRC extra byte
    crc = crc_accumulate(crc_extra, crc);

    crc
}

fn crc_accumulate(byte: u8, crc: u16) -> u16 {
    let tmp = (byte ^ (crc as u8)) as u16;
    let tmp = tmp ^ ((tmp << 4) & 0xFF); // Mask to 8 bits per X.25 CRC spec
    (crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)
}

/// Get CRC extra byte for message ID (from MAVLink XML definitions)
fn get_crc_extra(msg_id: u32) -> u8 {
    match msg_id {
        0 => 50,   // HEARTBEAT
        1 => 124,  // SYS_STATUS
        2 => 137,  // SYSTEM_TIME
        31 => 246, // ATTITUDE_QUATERNION
        32 => 185, // LOCAL_POSITION_NED
        69 => 243, // MANUAL_CONTROL
        70 => 124, // RC_CHANNELS_OVERRIDE
        76 => 152, // COMMAND_LONG
        77 => 143, // COMMAND_ACK
        82 => 49,  // SET_ATTITUDE_TARGET
        84 => 143, // SET_POSITION_TARGET_LOCAL_NED
        253 => 83, // STATUSTEXT
        _ => 0,    // Unknown message
    }
}

/// Declared payload length (base fields plus extensions) for known messages.
///
/// This is the length a MAVLink 2 truncated payload is zero-extended to
/// before field decoding.
fn declared_payload_len(msg_id: u32) -> Option<usize> {
    match msg_id {
        Heartbeat::MSG_ID => Some(Heartbeat::PAYLOAD_LEN),
        SysStatus::MSG_ID => Some(SysStatus::PAYLOAD_LEN),
        SystemTime::MSG_ID => Some(SystemTime::PAYLOAD_LEN),
        AttitudeQuaternion::MSG_ID => Some(AttitudeQuaternion::PAYLOAD_LEN),
        LocalPositionNed::MSG_ID => Some(LocalPositionNed::PAYLOAD_LEN),
        ManualControl::MSG_ID => Some(ManualControl::PAYLOAD_LEN),
        RcChannelsOverride::MSG_ID => Some(RcChannelsOverride::PAYLOAD_LEN),
        CommandLong::MSG_ID => Some(CommandLong::PAYLOAD_LEN),
        CommandAck::MSG_ID => Some(CommandAck::PAYLOAD_LEN),
        SetAttitudeTarget::MSG_ID => Some(SetAttitudeTarget::PAYLOAD_LEN),
        SetPositionTargetLocalNed::MSG_ID => Some(SetPositionTargetLocalNed::PAYLOAD_LEN),
        Statustext::MSG_ID => Some(Statustext::PAYLOAD_LEN),
        _ => None,
    }
}

// ============================================================================
// SERIALIZER (MAVLink Messages → Bytes)
// ============================================================================

// MAVLink message serialization
//
// Serializes MAVLink 2.0 frames to byte buffers.

/// Serialize a MAVLink message to a byte buffer
///
/// Returns the number of bytes written, or None if buffer is too small.
/// Serialize a MAVLink message to a byte buffer
///
/// Returns the number of bytes written, or None if buffer is too small.
pub fn serialize_mavlink(
    msg: &MavMessage,
    seq: u8,
    sys_id: u8,
    comp_id: u8,
    buf: &mut [u8],
) -> Option<usize> {
    match msg {
        MavMessage::Heartbeat(m) => serialize_heartbeat(m, seq, sys_id, comp_id, buf),
        MavMessage::SystemTime(m) => serialize_system_time(m, seq, sys_id, comp_id, buf),
        MavMessage::AttitudeQuaternion(m) => {
            serialize_attitude_quaternion(m, seq, sys_id, comp_id, buf)
        }
        MavMessage::LocalPositionNed(m) => {
            serialize_local_position_ned(m, seq, sys_id, comp_id, buf)
        }
        MavMessage::SetAttitudeTarget(m) => {
            serialize_set_attitude_target(m, seq, sys_id, comp_id, buf)
        }
        MavMessage::SetPositionTargetLocalNed(m) => {
            serialize_set_position_target_local_ned(m, seq, sys_id, comp_id, buf)
        }
        MavMessage::CommandLong(m) => serialize_command_long(m, seq, sys_id, comp_id, buf),
        MavMessage::CommandAck(m) => serialize_command_ack(m, seq, sys_id, comp_id, buf),
        MavMessage::RcChannelsOverride(m) => {
            serialize_rc_channels_override(m, seq, sys_id, comp_id, buf)
        }
        MavMessage::ManualControl(m) => serialize_manual_control(m, seq, sys_id, comp_id, buf),
        MavMessage::SysStatus(m) => serialize_sys_status(m, seq, sys_id, comp_id, buf),
        MavMessage::Statustext(m) => serialize_statustext(m, seq, sys_id, comp_id, buf),
        MavMessage::Unknown { .. } => None,
    }
}

fn write_header(
    buf: &mut [u8],
    payload_len: u8,
    seq: u8,
    sys_id: u8,
    comp_id: u8,
    msgid: u32,
) -> usize {
    buf[0] = MAVLINK_STX_V2;
    buf[1] = payload_len;
    buf[2] = 0; // incompat_flags
    buf[3] = 0; // compat_flags
    buf[4] = seq;
    buf[5] = sys_id;
    buf[6] = comp_id;
    buf[7] = (msgid & 0xFF) as u8;
    buf[8] = ((msgid >> 8) & 0xFF) as u8;
    buf[9] = ((msgid >> 16) & 0xFF) as u8;
    MavHeader::SIZE
}

/// Finalize a MAVLink 2 frame: truncate trailing zero payload bytes, patch
/// the header length byte, and append the checksum.
///
/// MAVLink 2 requires trailing zero bytes to be removed from the payload
/// before transmission, always retaining at least one payload byte. The
/// CRC covers the truncated frame, so `offset` (the untruncated payload
/// end) may exceed the returned frame end.
fn write_crc(buf: &mut [u8], offset: usize, crc_extra: u8) -> usize {
    let mut payload_end = offset;
    while payload_end > MavHeader::SIZE + 1 && buf[payload_end - 1] == 0 {
        payload_end -= 1;
    }
    buf[1] = (payload_end - MavHeader::SIZE) as u8;
    let crc = compute_crc(&buf[1..payload_end], crc_extra);
    buf[payload_end] = (crc & 0xFF) as u8;
    buf[payload_end + 1] = ((crc >> 8) & 0xFF) as u8;
    payload_end + 2
}

fn serialize_heartbeat(
    msg: &Heartbeat,
    seq: u8,
    sys_id: u8,
    comp_id: u8,
    buf: &mut [u8],
) -> Option<usize> {
    let frame_size = MavHeader::SIZE + Heartbeat::PAYLOAD_LEN + 2;
    if buf.len() < frame_size {
        return None;
    }

    let offset = write_header(
        buf,
        Heartbeat::PAYLOAD_LEN as u8,
        seq,
        sys_id,
        comp_id,
        Heartbeat::MSG_ID,
    );

    write_u32_le(buf, offset, msg.custom_mode);
    buf[offset + 4] = msg.mav_type;
    buf[offset + 5] = msg.autopilot;
    buf[offset + 6] = msg.base_mode;
    buf[offset + 7] = msg.system_status;
    buf[offset + 8] = msg.mavlink_version;

    Some(write_crc(buf, offset + Heartbeat::PAYLOAD_LEN, 50))
}

fn serialize_system_time(
    msg: &SystemTime,
    seq: u8,
    sys_id: u8,
    comp_id: u8,
    buf: &mut [u8],
) -> Option<usize> {
    let frame_size = MavHeader::SIZE + SystemTime::PAYLOAD_LEN + 2;
    if buf.len() < frame_size {
        return None;
    }

    let offset = write_header(
        buf,
        SystemTime::PAYLOAD_LEN as u8,
        seq,
        sys_id,
        comp_id,
        SystemTime::MSG_ID,
    );

    write_u64_le(buf, offset, msg.time_unix_usec);
    write_u32_le(buf, offset + 8, msg.time_boot_ms);

    Some(write_crc(buf, offset + SystemTime::PAYLOAD_LEN, 137))
}

fn serialize_attitude_quaternion(
    msg: &AttitudeQuaternion,
    seq: u8,
    sys_id: u8,
    comp_id: u8,
    buf: &mut [u8],
) -> Option<usize> {
    let frame_size = MavHeader::SIZE + AttitudeQuaternion::PAYLOAD_LEN + 2;
    if buf.len() < frame_size {
        return None;
    }

    let offset = write_header(
        buf,
        AttitudeQuaternion::PAYLOAD_LEN as u8,
        seq,
        sys_id,
        comp_id,
        AttitudeQuaternion::MSG_ID,
    );

    write_u32_le(buf, offset, msg.time_boot_ms);
    write_f32_le(buf, offset + 4, msg.q1);
    write_f32_le(buf, offset + 8, msg.q2);
    write_f32_le(buf, offset + 12, msg.q3);
    write_f32_le(buf, offset + 16, msg.q4);
    write_f32_le(buf, offset + 20, msg.rollspeed);
    write_f32_le(buf, offset + 24, msg.pitchspeed);
    write_f32_le(buf, offset + 28, msg.yawspeed);
    write_f32_le(buf, offset + 32, msg.repr_offset_q[0]);
    write_f32_le(buf, offset + 36, msg.repr_offset_q[1]);
    write_f32_le(buf, offset + 40, msg.repr_offset_q[2]);
    write_f32_le(buf, offset + 44, msg.repr_offset_q[3]);

    Some(write_crc(
        buf,
        offset + AttitudeQuaternion::PAYLOAD_LEN,
        246,
    ))
}

fn serialize_local_position_ned(
    msg: &LocalPositionNed,
    seq: u8,
    sys_id: u8,
    comp_id: u8,
    buf: &mut [u8],
) -> Option<usize> {
    let frame_size = MavHeader::SIZE + LocalPositionNed::PAYLOAD_LEN + 2;
    if buf.len() < frame_size {
        return None;
    }

    let offset = write_header(
        buf,
        LocalPositionNed::PAYLOAD_LEN as u8,
        seq,
        sys_id,
        comp_id,
        LocalPositionNed::MSG_ID,
    );

    write_u32_le(buf, offset, msg.time_boot_ms);
    write_f32_le(buf, offset + 4, msg.x);
    write_f32_le(buf, offset + 8, msg.y);
    write_f32_le(buf, offset + 12, msg.z);
    write_f32_le(buf, offset + 16, msg.vx);
    write_f32_le(buf, offset + 20, msg.vy);
    write_f32_le(buf, offset + 24, msg.vz);

    Some(write_crc(buf, offset + LocalPositionNed::PAYLOAD_LEN, 185))
}

fn serialize_set_attitude_target(
    msg: &SetAttitudeTarget,
    seq: u8,
    sys_id: u8,
    comp_id: u8,
    buf: &mut [u8],
) -> Option<usize> {
    let frame_size = MavHeader::SIZE + SetAttitudeTarget::PAYLOAD_LEN + 2;
    if buf.len() < frame_size {
        return None;
    }

    let offset = write_header(
        buf,
        SetAttitudeTarget::PAYLOAD_LEN as u8,
        seq,
        sys_id,
        comp_id,
        SetAttitudeTarget::MSG_ID,
    );

    write_u32_le(buf, offset, msg.time_boot_ms);
    write_f32_le(buf, offset + 4, msg.q[0]);
    write_f32_le(buf, offset + 8, msg.q[1]);
    write_f32_le(buf, offset + 12, msg.q[2]);
    write_f32_le(buf, offset + 16, msg.q[3]);
    write_f32_le(buf, offset + 20, msg.body_roll_rate);
    write_f32_le(buf, offset + 24, msg.body_pitch_rate);
    write_f32_le(buf, offset + 28, msg.body_yaw_rate);
    write_f32_le(buf, offset + 32, msg.thrust);
    buf[offset + 36] = msg.target_system;
    buf[offset + 37] = msg.target_component;
    buf[offset + 38] = msg.type_mask;
    write_f32_le(buf, offset + 39, msg.thrust_body[0]);
    write_f32_le(buf, offset + 43, msg.thrust_body[1]);
    write_f32_le(buf, offset + 47, msg.thrust_body[2]);

    Some(write_crc(buf, offset + SetAttitudeTarget::PAYLOAD_LEN, 49))
}

fn serialize_set_position_target_local_ned(
    msg: &SetPositionTargetLocalNed,
    seq: u8,
    sys_id: u8,
    comp_id: u8,
    buf: &mut [u8],
) -> Option<usize> {
    let frame_size = MavHeader::SIZE + SetPositionTargetLocalNed::PAYLOAD_LEN + 2;
    if buf.len() < frame_size {
        return None;
    }

    let offset = write_header(
        buf,
        SetPositionTargetLocalNed::PAYLOAD_LEN as u8,
        seq,
        sys_id,
        comp_id,
        SetPositionTargetLocalNed::MSG_ID,
    );

    write_u32_le(buf, offset, msg.time_boot_ms);
    write_f32_le(buf, offset + 4, msg.x);
    write_f32_le(buf, offset + 8, msg.y);
    write_f32_le(buf, offset + 12, msg.z);
    write_f32_le(buf, offset + 16, msg.vx);
    write_f32_le(buf, offset + 20, msg.vy);
    write_f32_le(buf, offset + 24, msg.vz);
    write_f32_le(buf, offset + 28, msg.afx);
    write_f32_le(buf, offset + 32, msg.afy);
    write_f32_le(buf, offset + 36, msg.afz);
    write_f32_le(buf, offset + 40, msg.yaw);
    write_f32_le(buf, offset + 44, msg.yaw_rate);
    write_u16_le(buf, offset + 48, msg.type_mask);
    buf[offset + 50] = msg.target_system;
    buf[offset + 51] = msg.target_component;
    buf[offset + 52] = msg.coordinate_frame;

    Some(write_crc(
        buf,
        offset + SetPositionTargetLocalNed::PAYLOAD_LEN,
        143,
    ))
}

fn serialize_command_long(
    msg: &CommandLong,
    seq: u8,
    sys_id: u8,
    comp_id: u8,
    buf: &mut [u8],
) -> Option<usize> {
    let frame_size = MavHeader::SIZE + CommandLong::PAYLOAD_LEN + 2;
    if buf.len() < frame_size {
        return None;
    }

    let offset = write_header(
        buf,
        CommandLong::PAYLOAD_LEN as u8,
        seq,
        sys_id,
        comp_id,
        CommandLong::MSG_ID,
    );

    write_f32_le(buf, offset, msg.param1);
    write_f32_le(buf, offset + 4, msg.param2);
    write_f32_le(buf, offset + 8, msg.param3);
    write_f32_le(buf, offset + 12, msg.param4);
    write_f32_le(buf, offset + 16, msg.param5);
    write_f32_le(buf, offset + 20, msg.param6);
    write_f32_le(buf, offset + 24, msg.param7);
    write_u16_le(buf, offset + 28, msg.command);
    buf[offset + 30] = msg.target_system;
    buf[offset + 31] = msg.target_component;
    buf[offset + 32] = msg.confirmation;

    Some(write_crc(buf, offset + CommandLong::PAYLOAD_LEN, 152))
}

fn serialize_command_ack(
    msg: &CommandAck,
    seq: u8,
    sys_id: u8,
    comp_id: u8,
    buf: &mut [u8],
) -> Option<usize> {
    let frame_size = MavHeader::SIZE + CommandAck::PAYLOAD_LEN + 2;
    if buf.len() < frame_size {
        return None;
    }

    let offset = write_header(
        buf,
        CommandAck::PAYLOAD_LEN as u8,
        seq,
        sys_id,
        comp_id,
        CommandAck::MSG_ID,
    );

    write_u16_le(buf, offset, msg.command);
    buf[offset + 2] = msg.result;
    buf[offset + 3] = msg.progress;
    write_i32_le(buf, offset + 4, msg.result_param2);
    buf[offset + 8] = msg.target_system;
    buf[offset + 9] = msg.target_component;

    Some(write_crc(buf, offset + CommandAck::PAYLOAD_LEN, 143))
}

fn serialize_rc_channels_override(
    msg: &RcChannelsOverride,
    seq: u8,
    sys_id: u8,
    comp_id: u8,
    buf: &mut [u8],
) -> Option<usize> {
    let frame_size = MavHeader::SIZE + RcChannelsOverride::PAYLOAD_LEN + 2;
    if buf.len() < frame_size {
        return None;
    }

    let offset = write_header(
        buf,
        RcChannelsOverride::PAYLOAD_LEN as u8,
        seq,
        sys_id,
        comp_id,
        RcChannelsOverride::MSG_ID,
    );

    write_u16_le(buf, offset, msg.chan1_raw);
    write_u16_le(buf, offset + 2, msg.chan2_raw);
    write_u16_le(buf, offset + 4, msg.chan3_raw);
    write_u16_le(buf, offset + 6, msg.chan4_raw);
    write_u16_le(buf, offset + 8, msg.chan5_raw);
    write_u16_le(buf, offset + 10, msg.chan6_raw);
    write_u16_le(buf, offset + 12, msg.chan7_raw);
    write_u16_le(buf, offset + 14, msg.chan8_raw);
    buf[offset + 16] = msg.target_system;
    buf[offset + 17] = msg.target_component;

    // Extensions (Chan 9-18)
    let ext_offset = offset + 18;
    write_u16_le(buf, ext_offset, msg.chan9_raw);
    write_u16_le(buf, ext_offset + 2, msg.chan10_raw);
    write_u16_le(buf, ext_offset + 4, msg.chan11_raw);
    write_u16_le(buf, ext_offset + 6, msg.chan12_raw);
    write_u16_le(buf, ext_offset + 8, msg.chan13_raw);
    write_u16_le(buf, ext_offset + 10, msg.chan14_raw);
    write_u16_le(buf, ext_offset + 12, msg.chan15_raw);
    write_u16_le(buf, ext_offset + 14, msg.chan16_raw);
    write_u16_le(buf, ext_offset + 16, msg.chan17_raw);
    write_u16_le(buf, ext_offset + 18, msg.chan18_raw);

    Some(write_crc(
        buf,
        offset + RcChannelsOverride::PAYLOAD_LEN,
        124,
    ))
}

fn serialize_manual_control(
    msg: &ManualControl,
    seq: u8,
    sys_id: u8,
    comp_id: u8,
    buf: &mut [u8],
) -> Option<usize> {
    let frame_size = MavHeader::SIZE + ManualControl::PAYLOAD_LEN + 2;
    if buf.len() < frame_size {
        return None;
    }

    let offset = write_header(
        buf,
        ManualControl::PAYLOAD_LEN as u8,
        seq,
        sys_id,
        comp_id,
        ManualControl::MSG_ID,
    );

    write_i16_le(buf, offset, msg.x);
    write_i16_le(buf, offset + 2, msg.y);
    write_i16_le(buf, offset + 4, msg.z);
    write_i16_le(buf, offset + 6, msg.r);
    write_u16_le(buf, offset + 8, msg.buttons);
    buf[offset + 10] = msg.target;

    // Extensions follow in common.xml declaration order.
    write_u16_le(buf, offset + 11, msg.buttons2);
    buf[offset + 13] = msg.enabled_extensions;
    write_i16_le(buf, offset + 14, msg.s);
    write_i16_le(buf, offset + 16, msg.t);
    write_i16_le(buf, offset + 18, msg.aux1);
    write_i16_le(buf, offset + 20, msg.aux2);
    write_i16_le(buf, offset + 22, msg.aux3);
    write_i16_le(buf, offset + 24, msg.aux4);
    write_i16_le(buf, offset + 26, msg.aux5);
    write_i16_le(buf, offset + 28, msg.aux6);

    Some(write_crc(buf, offset + ManualControl::PAYLOAD_LEN, 243))
}

fn serialize_sys_status(
    msg: &SysStatus,
    seq: u8,
    sys_id: u8,
    comp_id: u8,
    buf: &mut [u8],
) -> Option<usize> {
    let frame_size = MavHeader::SIZE + SysStatus::PAYLOAD_LEN + 2;
    if buf.len() < frame_size {
        return None;
    }

    let offset = write_header(
        buf,
        SysStatus::PAYLOAD_LEN as u8,
        seq,
        sys_id,
        comp_id,
        SysStatus::MSG_ID,
    );

    write_u32_le(buf, offset, msg.onboard_control_sensors_present);
    write_u32_le(buf, offset + 4, msg.onboard_control_sensors_enabled);
    write_u32_le(buf, offset + 8, msg.onboard_control_sensors_health);
    write_u16_le(buf, offset + 12, msg.load);
    write_u16_le(buf, offset + 14, msg.voltage_battery);
    write_i16_le(buf, offset + 16, msg.current_battery);
    write_u16_le(buf, offset + 18, msg.drop_rate_comm);
    write_u16_le(buf, offset + 20, msg.errors_comm);
    write_u16_le(buf, offset + 22, msg.errors_count1);
    write_u16_le(buf, offset + 24, msg.errors_count2);
    write_u16_le(buf, offset + 26, msg.errors_count3);
    write_u16_le(buf, offset + 28, msg.errors_count4);
    buf[offset + 30] = msg.battery_remaining as u8;

    // Extensions
    let ext_offset = offset + 31;
    write_u32_le(
        buf,
        ext_offset,
        msg.onboard_control_sensors_present_extended,
    );
    write_u32_le(
        buf,
        ext_offset + 4,
        msg.onboard_control_sensors_enabled_extended,
    );
    write_u32_le(
        buf,
        ext_offset + 8,
        msg.onboard_control_sensors_health_extended,
    );

    Some(write_crc(buf, offset + SysStatus::PAYLOAD_LEN, 124))
}

fn serialize_statustext(
    msg: &Statustext,
    seq: u8,
    sys_id: u8,
    comp_id: u8,
    buf: &mut [u8],
) -> Option<usize> {
    let frame_size = MavHeader::SIZE + Statustext::PAYLOAD_LEN + 2;
    if buf.len() < frame_size {
        return None;
    }

    let offset = write_header(
        buf,
        Statustext::PAYLOAD_LEN as u8,
        seq,
        sys_id,
        comp_id,
        Statustext::MSG_ID,
    );

    buf[offset] = msg.severity;
    for i in 0..50 {
        buf[offset + 1 + i] = msg.text[i];
    }

    // Extensions
    let ext_offset = offset + 51;
    write_u16_le(buf, ext_offset, msg.id);
    buf[ext_offset + 2] = msg.chunk_seq;

    Some(write_crc(buf, offset + Statustext::PAYLOAD_LEN, 83))
}

// Byte writing helpers (little-endian)
fn write_u16_le(buf: &mut [u8], offset: usize, val: u16) {
    buf[offset] = (val & 0xFF) as u8;
    buf[offset + 1] = ((val >> 8) & 0xFF) as u8;
}

fn write_i16_le(buf: &mut [u8], offset: usize, val: i16) {
    write_u16_le(buf, offset, val as u16);
}

fn write_u32_le(buf: &mut [u8], offset: usize, val: u32) {
    buf[offset] = (val & 0xFF) as u8;
    buf[offset + 1] = ((val >> 8) & 0xFF) as u8;
    buf[offset + 2] = ((val >> 16) & 0xFF) as u8;
    buf[offset + 3] = ((val >> 24) & 0xFF) as u8;
}

fn write_i32_le(buf: &mut [u8], offset: usize, val: i32) {
    write_u32_le(buf, offset, val as u32);
}

fn write_u64_le(buf: &mut [u8], offset: usize, val: u64) {
    buf[offset] = (val & 0xFF) as u8;
    buf[offset + 1] = ((val >> 8) & 0xFF) as u8;
    buf[offset + 2] = ((val >> 16) & 0xFF) as u8;
    buf[offset + 3] = ((val >> 24) & 0xFF) as u8;
    buf[offset + 4] = ((val >> 32) & 0xFF) as u8;
    buf[offset + 5] = ((val >> 40) & 0xFF) as u8;
    buf[offset + 6] = ((val >> 48) & 0xFF) as u8;
    buf[offset + 7] = ((val >> 56) & 0xFF) as u8;
}

fn write_f32_le(buf: &mut [u8], offset: usize, val: f32) {
    write_u32_le(buf, offset, val.to_bits());
}

// Note: CRC functions are defined in the parser section above (lines 929-946)

#[cfg(test)]
mod interop_tests;

#[cfg(test)]
mod truncation_tests;

#[cfg(test)]
mod tests {
    use super::*;

    fn roundtrip<F, G>(create_msg: F, verify_msg: G)
    where
        F: Fn() -> MavMessage,
        G: Fn(MavMessage),
    {
        let msg = create_msg();
        let mut buf = [0u8; 256];
        let len = serialize_mavlink(&msg, 0, 1, 1, &mut buf);
        assert!(len.is_some());
        let Some(len) = len else {
            return;
        };

        let parsed = parse_mavlink(&buf[..len]);
        assert!(parsed.is_ok());
        let Ok((parsed, _sig, consumed)) = parsed else {
            return;
        };
        assert_eq!(consumed, len, "Consumed length mismatch");
        verify_msg(parsed);
    }

    fn parse_test_message(input: &[u8]) -> Option<(MavMessage, usize)> {
        let parsed = parse_mavlink(input);
        assert!(parsed.is_ok(), "Parse failed: {:?}", parsed.err());
        let Ok((msg, _sig, consumed)) = parsed else {
            return None;
        };
        Some((msg, consumed))
    }

    #[test]
    fn test_heartbeat() {
        roundtrip(
            || {
                MavMessage::Heartbeat(Heartbeat {
                    mav_type: 2,
                    autopilot: 18,
                    base_mode: 128,
                    custom_mode: 0,
                    system_status: 4,
                    mavlink_version: 3,
                })
            },
            |m| {
                assert!(matches!(&m, MavMessage::Heartbeat(_)));
                let MavMessage::Heartbeat(h) = m else {
                    return;
                };
                assert_eq!(h.mav_type, 2);
                assert_eq!(h.autopilot, 18);
            },
        );
    }

    #[test]
    fn test_attitude_quaternion() {
        roundtrip(
            || {
                MavMessage::AttitudeQuaternion(AttitudeQuaternion {
                    time_boot_ms: 1000,
                    q1: 1.0,
                    q2: 0.0,
                    q3: 0.0,
                    q4: 0.0,
                    rollspeed: 0.1,
                    pitchspeed: 0.2,
                    yawspeed: 0.3,
                    repr_offset_q: [0.0, 0.0, 0.0, 0.0],
                })
            },
            |m| {
                assert!(matches!(&m, MavMessage::AttitudeQuaternion(_)));
                let MavMessage::AttitudeQuaternion(h) = m else {
                    return;
                };
                assert_eq!(h.time_boot_ms, 1000);
                assert!((h.q1 - 1.0).abs() < 1e-5);
                assert!((h.rollspeed - 0.1).abs() < 1e-5);
            },
        );
    }

    #[test]
    fn test_local_position_ned() {
        roundtrip(
            || {
                MavMessage::LocalPositionNed(LocalPositionNed {
                    time_boot_ms: 2000,
                    x: 10.0,
                    y: 20.0,
                    z: -5.0,
                    vx: 1.0,
                    vy: 0.0,
                    vz: 0.0,
                })
            },
            |m| {
                assert!(matches!(&m, MavMessage::LocalPositionNed(_)));
                let MavMessage::LocalPositionNed(h) = m else {
                    return;
                };
                assert_eq!(h.time_boot_ms, 2000);
                assert!((h.x - 10.0).abs() < 1e-5);
            },
        );
    }

    #[test]
    fn test_set_attitude_target() {
        roundtrip(
            || {
                MavMessage::SetAttitudeTarget(SetAttitudeTarget {
                    time_boot_ms: 3000,
                    target_system: 1,
                    target_component: 1,
                    type_mask: 0,
                    q: [1.0, 0.0, 0.0, 0.0],
                    body_roll_rate: 0.1,
                    body_pitch_rate: 0.0,
                    body_yaw_rate: 0.0,
                    thrust: 0.5,
                    thrust_body: [0.0, 0.0, 0.0],
                })
            },
            |m| {
                assert!(matches!(&m, MavMessage::SetAttitudeTarget(_)));
                let MavMessage::SetAttitudeTarget(h) = m else {
                    return;
                };
                assert_eq!(h.time_boot_ms, 3000);
                assert!((h.thrust - 0.5).abs() < 1e-5);
            },
        );
    }

    #[test]
    fn test_command_long() {
        roundtrip(
            || {
                MavMessage::CommandLong(CommandLong {
                    param1: 1.0,
                    param2: 2.0,
                    param3: 3.0,
                    param4: 4.0,
                    param5: 5.0,
                    param6: 6.0,
                    param7: 7.0,
                    command: 400,
                    target_system: 1,
                    target_component: 1,
                    confirmation: 0,
                })
            },
            |m| {
                assert!(matches!(&m, MavMessage::CommandLong(_)));
                let MavMessage::CommandLong(h) = m else {
                    return;
                };
                assert_eq!(h.command, 400);
                assert!((h.param1 - 1.0).abs() < 1e-5);
            },
        );
    }

    #[test]
    fn test_command_ack() {
        roundtrip(
            || {
                MavMessage::CommandAck(CommandAck {
                    command: 400,
                    result: 0,
                    progress: 100,
                    result_param2: 0,
                    target_system: 1,
                    target_component: 1,
                })
            },
            |m| {
                assert!(matches!(&m, MavMessage::CommandAck(_)));
                let MavMessage::CommandAck(h) = m else {
                    return;
                };
                assert_eq!(h.command, 400);
                assert_eq!(h.result, 0);
                assert_eq!(h.progress, 100);
            },
        );
    }

    #[test]
    fn test_rc_channels_override() {
        roundtrip(
            || {
                MavMessage::RcChannelsOverride(RcChannelsOverride {
                    chan1_raw: 1500,
                    chan2_raw: 1500,
                    chan3_raw: 1000,
                    chan4_raw: 1500,
                    chan5_raw: 0,
                    chan6_raw: 0,
                    chan7_raw: 0,
                    chan8_raw: 0,
                    target_system: 1,
                    target_component: 1,
                    chan9_raw: 0,
                    chan10_raw: 0,
                    chan11_raw: 0,
                    chan12_raw: 0,
                    chan13_raw: 0,
                    chan14_raw: 0,
                    chan15_raw: 0,
                    chan16_raw: 0,
                    chan17_raw: 0,
                    chan18_raw: 0,
                })
            },
            |m| {
                assert!(matches!(&m, MavMessage::RcChannelsOverride(_)));
                let MavMessage::RcChannelsOverride(h) = m else {
                    return;
                };
                assert_eq!(h.chan3_raw, 1000);
                assert_eq!(h.target_system, 1);
            },
        );
    }

    #[test]
    fn test_manual_control() {
        roundtrip(
            || {
                MavMessage::ManualControl(ManualControl {
                    x: 100,
                    y: -100,
                    z: 500,
                    buttons: 1,
                    target: 1,
                    ..Default::default()
                })
            },
            |m| {
                assert!(matches!(&m, MavMessage::ManualControl(_)));
                let MavMessage::ManualControl(h) = m else {
                    return;
                };
                assert_eq!(h.x, 100);
                assert_eq!(h.z, 500);
                assert_eq!(h.target, 1);
            },
        );
    }

    #[test]
    fn test_sys_status() {
        roundtrip(
            || {
                MavMessage::SysStatus(SysStatus {
                    onboard_control_sensors_present: 1,
                    onboard_control_sensors_enabled: 1,
                    onboard_control_sensors_health: 1,
                    load: 500,
                    voltage_battery: 12000,
                    current_battery: 100,
                    drop_rate_comm: 0,
                    errors_comm: 0,
                    errors_count1: 0,
                    errors_count2: 0,
                    errors_count3: 0,
                    errors_count4: 0,
                    battery_remaining: 50,
                    onboard_control_sensors_present_extended: 0,
                    onboard_control_sensors_enabled_extended: 0,
                    onboard_control_sensors_health_extended: 0,
                })
            },
            |m| {
                assert!(matches!(&m, MavMessage::SysStatus(_)));
                let MavMessage::SysStatus(h) = m else {
                    return;
                };
                assert_eq!(h.voltage_battery, 12000);
                assert_eq!(h.battery_remaining, 50);
            },
        );
    }

    #[test]
    fn test_statustext() {
        let mut text = [0u8; 50];
        text[0] = b'H';
        text[1] = b'e';
        text[2] = b'l';
        text[3] = b'l';
        text[4] = b'o';
        roundtrip(
            || {
                MavMessage::Statustext(Statustext {
                    severity: 6,
                    text,
                    id: 0,
                    chunk_seq: 0,
                })
            },
            |m| {
                assert!(matches!(&m, MavMessage::Statustext(_)));
                let MavMessage::Statustext(h) = m else {
                    return;
                };
                assert_eq!(h.severity, 6);
                assert_eq!(h.text[0], b'H');
            },
        );
    }

    #[test]
    fn test_set_position_target_local_ned() {
        roundtrip(
            || {
                MavMessage::SetPositionTargetLocalNed(SetPositionTargetLocalNed {
                    time_boot_ms: 5000,
                    target_system: 1,
                    target_component: 1,
                    coordinate_frame: 1,              // MAV_FRAME_LOCAL_NED
                    type_mask: 0b0000_1111_1111_1000, // Position only (ignore velocity, accel, yaw)
                    x: 10.0,
                    y: 20.0,
                    z: -5.0, // NED: negative = up
                    vx: 0.0,
                    vy: 0.0,
                    vz: 0.0,
                    afx: 0.0,
                    afy: 0.0,
                    afz: 0.0,
                    yaw: 0.0,
                    yaw_rate: 0.0,
                })
            },
            |m| {
                assert!(matches!(&m, MavMessage::SetPositionTargetLocalNed(_)));
                let MavMessage::SetPositionTargetLocalNed(p) = m else {
                    return;
                };
                assert_eq!(p.time_boot_ms, 5000);
                assert!((p.x - 10.0).abs() < 1e-5);
                assert!((p.y - 20.0).abs() < 1e-5);
                assert!((p.z - (-5.0)).abs() < 1e-5);
                assert_eq!(p.coordinate_frame, 1);
            },
        );
    }

    // ==========================================================================
    // Pymavlink Interoperability Tests
    // Test vectors generated by pymavlink to verify cross-implementation compatibility
    // ==========================================================================

    // Generated by mavlink_interop_test_rust.py
    const PYMAVLINK_HEARTBEAT: &[u8] = &[
        253, 9, 0, 0, 0, 255, 190, 0, 0, 0, 0, 0, 0, 0, 2, 0, 128, 4, 3, 223, 47,
    ];
    const PYMAVLINK_COMMAND_ARM: &[u8] = &[
        253, 32, 0, 0, 0, 255, 190, 76, 0, 0, 0, 0, 128, 63, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 144, 1, 1, 1, 158, 78,
    ];
    const PYMAVLINK_SET_ATTITUDE: &[u8] = &[
        253, 39, 0, 0, 0, 255, 190, 82, 0, 0, 232, 3, 0, 0, 0, 0, 128, 63, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 63, 1, 1, 7, 237, 57,
    ];
    const PYMAVLINK_MANUAL_CONTROL: &[u8] = &[
        253, 11, 0, 0, 0, 255, 190, 69, 0, 0, 100, 0, 156, 255, 244, 1, 0, 0, 0, 0, 1, 35, 223,
    ];
    const PYMAVLINK_RC_OVERRIDE: &[u8] = &[
        253, 18, 0, 0, 0, 255, 190, 70, 0, 0, 220, 5, 220, 5, 232, 3, 220, 5, 0, 0, 0, 0, 0, 0, 0,
        0, 1, 1, 64, 169,
    ];

    #[test]
    fn test_pymavlink_heartbeat() {
        let Some((msg, consumed)) = parse_test_message(PYMAVLINK_HEARTBEAT) else {
            return;
        };
        assert_eq!(consumed, PYMAVLINK_HEARTBEAT.len());
        assert!(matches!(&msg, MavMessage::Heartbeat(_)));
        let MavMessage::Heartbeat(h) = msg else {
            return;
        };
        assert_eq!(h.mav_type, 2); // MAV_TYPE_QUADROTOR
        assert_eq!(h.autopilot, 0); // MAV_AUTOPILOT_GENERIC
        assert_eq!(h.base_mode, 128); // SAFETY_ARMED
        assert_eq!(h.system_status, 4); // MAV_STATE_ACTIVE
    }

    #[test]
    fn test_pymavlink_command_arm() {
        let Some((msg, consumed)) = parse_test_message(PYMAVLINK_COMMAND_ARM) else {
            return;
        };
        assert_eq!(consumed, PYMAVLINK_COMMAND_ARM.len());
        assert!(matches!(&msg, MavMessage::CommandLong(_)));
        let MavMessage::CommandLong(c) = msg else {
            return;
        };
        assert_eq!(c.command, 400); // MAV_CMD_COMPONENT_ARM_DISARM
        assert!((c.param1 - 1.0).abs() < 1e-5); // ARM
        assert_eq!(c.target_system, 1);
        assert_eq!(c.target_component, 1);
    }

    #[test]
    fn test_pymavlink_set_attitude_target() {
        let Some((msg, consumed)) = parse_test_message(PYMAVLINK_SET_ATTITUDE) else {
            return;
        };
        assert_eq!(consumed, PYMAVLINK_SET_ATTITUDE.len());
        assert!(matches!(&msg, MavMessage::SetAttitudeTarget(_)));
        let MavMessage::SetAttitudeTarget(a) = msg else {
            return;
        };
        assert_eq!(a.time_boot_ms, 1000);
        assert!((a.q[0] - 1.0).abs() < 1e-5); // w = 1
        assert!((a.q[1]).abs() < 1e-5); // x = 0
        assert!((a.thrust - 0.5).abs() < 1e-5);
        assert_eq!(a.type_mask, 7);
    }

    #[test]
    fn test_pymavlink_manual_control() {
        let Some((msg, consumed)) = parse_test_message(PYMAVLINK_MANUAL_CONTROL) else {
            return;
        };
        assert_eq!(consumed, PYMAVLINK_MANUAL_CONTROL.len());
        assert!(matches!(&msg, MavMessage::ManualControl(_)));
        let MavMessage::ManualControl(m) = msg else {
            return;
        };
        assert_eq!(m.x, 100);
        assert_eq!(m.y, -100);
        assert_eq!(m.z, 500);
        assert_eq!(m.target, 1);
    }

    #[test]
    fn test_pymavlink_rc_channels_override() {
        let Some((msg, consumed)) = parse_test_message(PYMAVLINK_RC_OVERRIDE) else {
            return;
        };
        assert_eq!(consumed, PYMAVLINK_RC_OVERRIDE.len());
        assert!(matches!(&msg, MavMessage::RcChannelsOverride(_)));
        let MavMessage::RcChannelsOverride(r) = msg else {
            return;
        };
        assert_eq!(r.chan1_raw, 1500);
        assert_eq!(r.chan2_raw, 1500);
        assert_eq!(r.chan3_raw, 1000);
        assert_eq!(r.chan4_raw, 1500);
        assert_eq!(r.target_system, 1);
    }

    // ==========================================================================
    // Python test_arm_cmd.py Interoperability Test
    // Test vector generated by the exact Python script used in SITL tests
    // ==========================================================================

    // Generated by test_arm_cmd.py's build_command_long() function:
    // - sysid=255, compid=0 (GCS)
    // - target_system=1, target_component=1
    // - command=400 (ARM_DISARM), param1=1.0 (ARM)
    // - Uses CRC extra 152 for COMMAND_LONG
    const PYTHON_TEST_COMMAND_LONG_ARM: &[u8] = &[
        253, 33, 0, 0, 0, 255, 0, 76, 0, 0, 0, 0, 128, 63, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 144, 1, 1, 1, 0, 255, 144,
    ];

    #[test]
    fn test_python_test_arm_cmd_interop() {
        let result = parse_mavlink(PYTHON_TEST_COMMAND_LONG_ARM);
        assert!(result.is_ok(), "Parse failed: {:?}", result.err());
        let Ok((msg, _sig, consumed)) = result else {
            return;
        };
        assert_eq!(consumed, PYTHON_TEST_COMMAND_LONG_ARM.len());
        assert!(matches!(&msg, MavMessage::CommandLong(_)));
        let MavMessage::CommandLong(c) = msg else {
            return;
        };
        assert_eq!(c.command, 400); // MAV_CMD_COMPONENT_ARM_DISARM
        assert!((c.param1 - 1.0).abs() < 1e-5); // ARM
        assert_eq!(c.target_system, 1);
        assert_eq!(c.target_component, 1);
    }
}
