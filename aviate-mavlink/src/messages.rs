//! MAVLink message definitions for HIL simulation and core control

// --- Existing HIL messages ---

/// HIL_SENSOR (MAVLink #107)
#[derive(Copy, Clone, Debug, Default)]
pub struct HilSensor {
    pub time_usec: u64,
    pub xacc: f32,
    pub yacc: f32,
    pub zacc: f32,
    pub xgyro: f32,
    pub ygyro: f32,
    pub zgyro: f32,
    pub xmag: f32,
    pub ymag: f32,
    pub zmag: f32,
    pub abs_pressure: f32,
    pub diff_pressure: f32,
    pub pressure_alt: f32,
    pub temperature: f32,
    pub fields_updated: u32,
    pub id: u8,
}

impl HilSensor {
    pub const MSG_ID: u32 = 107;
    pub const PAYLOAD_LEN: usize = 65;
}

/// HIL_GPS (MAVLink #113)
#[derive(Copy, Clone, Debug, Default)]
pub struct HilGps {
    pub time_usec: u64,
    pub lat: i32,
    pub lon: i32,
    pub alt: i32,
    pub eph: u16,
    pub epv: u16,
    pub vel: u16,
    pub vn: i16,
    pub ve: i16,
    pub vd: i16,
    pub cog: u16,
    pub fix_type: u8,
    pub satellites_visible: u8,
    pub id: u8,
    pub yaw: u16,
}

impl HilGps {
    pub const MSG_ID: u32 = 113;
    pub const PAYLOAD_LEN: usize = 39;
}

/// HIL_ACTUATOR_CONTROLS (MAVLink #93)
#[derive(Copy, Clone, Debug)]
pub struct HilActuatorControls {
    pub time_usec: u64,
    pub controls: [f32; 16],
    pub mode: u8,
    pub flags: u64,
}

impl Default for HilActuatorControls {
    fn default() -> Self {
        Self {
            time_usec: 0,
            controls: [0.0; 16],
            mode: 0,
            flags: 0,
        }
    }
}

impl HilActuatorControls {
    pub const MSG_ID: u32 = 93;
    pub const PAYLOAD_LEN: usize = 81;
}

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

/// HIL_STATE_QUATERNION (MAVLink #115)
#[derive(Copy, Clone, Debug, Default)]
pub struct HilStateQuaternion {
    pub time_usec: u64,
    pub attitude_quaternion: [f32; 4],
    pub rollspeed: f32,
    pub pitchspeed: f32,
    pub yawspeed: f32,
    pub lat: i32,
    pub lon: i32,
    pub alt: i32,
    pub vx: i16,
    pub vy: i16,
    pub vz: i16,
    pub ind_airspeed: u16,
    pub true_airspeed: u16,
    pub xacc: i16,
    pub yacc: i16,
    pub zacc: i16,
}

impl HilStateQuaternion {
    pub const MSG_ID: u32 = 115;
    pub const PAYLOAD_LEN: usize = 64;
}

// --- New Messages (Quaternions & Control) ---

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
    pub const PAYLOAD_LEN: usize = 33; // 11 basic + 22 extension
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
    HilSensor(HilSensor),
    HilGps(HilGps),
    HilActuatorControls(HilActuatorControls),
    HilStateQuaternion(HilStateQuaternion),
    AttitudeQuaternion(AttitudeQuaternion),
    LocalPositionNed(LocalPositionNed),
    SetAttitudeTarget(SetAttitudeTarget),
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
            MavMessage::HilSensor(_) => HilSensor::MSG_ID,
            MavMessage::HilGps(_) => HilGps::MSG_ID,
            MavMessage::HilActuatorControls(_) => HilActuatorControls::MSG_ID,
            MavMessage::HilStateQuaternion(_) => HilStateQuaternion::MSG_ID,
            MavMessage::AttitudeQuaternion(_) => AttitudeQuaternion::MSG_ID,
            MavMessage::LocalPositionNed(_) => LocalPositionNed::MSG_ID,
            MavMessage::SetAttitudeTarget(_) => SetAttitudeTarget::MSG_ID,
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
