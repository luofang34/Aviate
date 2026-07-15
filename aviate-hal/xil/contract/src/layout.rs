//! The `#[repr(C)]` shared-memory layout (LAYOUT_VERSION 2) and the
//! fail-closed attach validation rules.
//!
//! Field ownership is a hard protocol contract, not advice:
//!
//! * [`SharedStateHeader`] — written ONLY by the gz plugin.
//! * [`ModelStateBlock`] — written ONLY by the gz plugin, under its
//!   seqlock (`seq` odd while a write is in flight).
//! * [`MotorCommandBlock`] — written ONLY by the flight controller,
//!   under its seqlock.
//! * [`ControlBlock`] — split by field: lifecycle requests, lockstep
//!   toggle, and RTF target are written by the session host /
//!   test harness; the lifecycle ack by the plugin; the FC state
//!   fields by the flight controller. Every control field is a
//!   single naturally-aligned `u32` read/written atomically, so the
//!   block needs no seqlock.

/// First eight bytes of the block: ASCII `AVIATEGZ` big-endian. A
/// consumer that attaches to anything else is looking at a foreign
/// or torn mapping and must fail closed. (Spelled as a literal so
/// cbindgen exports it; the equality with the ASCII bytes is pinned
/// by a test.)
pub const MAGIC: u64 = 0x4156_4941_5445_475A;

/// Layout version of [`SharedStateV2`]. Any layout change bumps this;
/// consumers reject a mismatch on attach.
pub const LAYOUT_VERSION: u32 = 2;

/// POSIX shm object name base. Instance 0 uses
/// [`SHM_NAME_INSTANCE_0`]; instance N appends `_N`.
pub const SHM_NAME_BASE: &str = "/aviate_gz_bridge";

/// Instance-0 shm object name.
pub const SHM_NAME_INSTANCE_0: &str = "/aviate_gz_bridge";

/// Self-describing block header. Writer: gz plugin.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct SharedStateHeader {
    /// [`MAGIC`].
    pub magic: u64,
    /// [`LAYOUT_VERSION`].
    pub layout_version: u32,
    /// `size_of::<SharedStateV2>()` as the writer compiled it.
    /// Consumers compare against their own expectation — see
    /// [`validate_attach`] for the macOS page-rounding rule.
    pub declared_size: u32,
    /// Simulation-world epoch: 1 on plugin configure, +1 on every
    /// world reset. A consumer that sees this change re-establishes
    /// its freshness tracking instead of quarantining the source —
    /// the "telemetry dies after reset" fix (#265).
    pub reset_generation: u32,
    /// Non-zero while the plugin owns the mapping.
    pub plugin_ready: u32,
    /// Reserved; zero.
    pub _reserved0: u64,
}

/// Ground-truth model state. Writer: gz plugin, under `seq`.
///
/// `sim_step` and `time_us` live INSIDE the seqlock payload so a
/// reader obtains a coherent `{step, time, state}` snapshot in one
/// consistent read — the authority for aligning telemetry to sim
/// time under acceleration (#265).
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct ModelStateBlock {
    /// Seqlock: odd while the plugin is writing. Readers retry on
    /// odd or changed values (see [`crate::seqlock_read`]).
    pub seq: u32,
    /// Padding; zero.
    pub _pad0: u32,
    /// Physics step counter. Monotonic across world resets — epochs
    /// are distinguished by `reset_generation`, not by this counter
    /// restarting.
    pub sim_step: u64,
    /// Simulation time (µs). Rewinds to zero on a world reset.
    pub time_us: u64,
    /// Position (m), world ENU.
    pub pos: [f64; 3],
    /// Orientation quaternion [w, x, y, z], ENU-world / FLU-body.
    pub quat: [f64; 4],
    /// Linear velocity [m/s], world ENU.
    pub vel: [f64; 3],
    /// Angular velocity [rad/s], body FLU.
    pub ang_vel: [f64; 3],
    /// Non-zero once the first physics step has been published.
    pub valid: u32,
    /// Padding; zero.
    pub _pad1: u32,
}

/// Motor command. Writer: flight controller, under `seq`.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct MotorCommandBlock {
    /// Seqlock: odd while the FC is writing.
    pub seq: u32,
    /// Number of populated `motor_vel` lanes.
    pub num_motors: u32,
    /// Rotor angular-velocity setpoints [rad/s]. The FC applies the
    /// resolved actuator curve BEFORE writing — values here are
    /// boundary commands, never force-domain thrust (#140).
    pub motor_vel: [f64; 8],
    /// Last `sim_step` the FC finished processing. Doubles as the FC
    /// liveness heartbeat and as the lockstep acknowledgement the
    /// plugin blocks on. Accessed as a bare aligned atomic, OUTSIDE
    /// the seqlock payload: the FC acks every step even on cycles
    /// that publish no new motor values, and a monotonic u64 needs
    /// no tear protection.
    pub fc_step_ack: u64,
    /// Reserved; zero.
    pub _reserved1: u64,
}

/// Runtime control plane (#265). Field-per-owner; every field is one
/// naturally-aligned `u32` accessed atomically.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct ControlBlock {
    /// Requested lifecycle action ([`LifecycleRequest`] as u32).
    /// Writer: session host / harness.
    pub lifecycle_request: u32,
    /// Bumped by the requester with every new request; the request
    /// is `(lifecycle_request, lifecycle_request_nonce)`.
    pub lifecycle_request_nonce: u32,
    /// Set by the PLUGIN to the request nonce once the action has
    /// been forwarded to the simulator — the "ack" half of
    /// request → ack → ready.
    pub lifecycle_ack_nonce: u32,
    /// Flight-controller lifecycle state ([`FcState`] as u32) — the
    /// "ready" half of request → ack → ready. Writer: FC.
    pub fc_state: u32,
    /// The `reset_generation` that `fc_state` refers to. `Ready`
    /// only counts when this equals the header's current generation.
    /// Writer: FC.
    pub fc_state_generation: u32,
    /// Written once per FC process start (any non-repeating value);
    /// consumers detect an FC restart by a change here even though
    /// the shm object identity is unchanged. Writer: FC.
    pub fc_session_nonce: u32,
    /// Runtime lockstep toggle: non-zero = the plugin blocks each
    /// physics step on `fc_step_ack` (#265 — no longer load-time
    /// SDF-only). Writer: session host / harness.
    pub lockstep_enabled: u32,
    /// Target real-time factor in percent: 100 = 1×, 400 = 4×,
    /// 0 = as-fast-as-possible. The plugin forwards changes to the
    /// physics engine. Writer: session host / harness.
    pub target_rtf_percent: u32,
    /// Reserved; zero.
    pub _reserved2: [u32; 4],
}

/// The full version-2 shared block.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct SharedStateV2 {
    /// Self-describing header.
    pub header: SharedStateHeader,
    /// Ground-truth model state (plugin → FC/consumers).
    pub state: ModelStateBlock,
    /// Motor command (FC → plugin).
    pub command: MotorCommandBlock,
    /// Runtime control plane.
    pub control: ControlBlock,
}

/// `size_of::<SharedStateV2>()`, pinned as a constant so both the
/// generated C header and attach validation carry the same number.
pub const EXPECTED_SIZE: usize = core::mem::size_of::<SharedStateV2>();

/// Lifecycle actions a session host / harness may request (#265).
#[repr(u32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LifecycleRequest {
    /// No pending request.
    None = 0,
    /// Reset the simulation world (and thereby the FC, via the
    /// generation bump).
    Reset = 1,
    /// Pause the simulation.
    Stop = 2,
    /// Resume the simulation.
    Start = 3,
}

impl LifecycleRequest {
    /// Decode a control-block word; unknown values are `None` (a
    /// consumer must never act on a request it does not understand).
    pub fn from_u32(v: u32) -> Self {
        match v {
            1 => LifecycleRequest::Reset,
            2 => LifecycleRequest::Stop,
            3 => LifecycleRequest::Start,
            _ => LifecycleRequest::None,
        }
    }
}

/// Flight-controller lifecycle state published in the control block.
#[repr(u32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FcState {
    /// FC process starting; kernel not yet constructed.
    Init = 0,
    /// Reset observed; kernel being reconstructed.
    Resetting = 1,
    /// Kernel constructed; estimator converging on fresh sensors.
    Converging = 2,
    /// Estimator healthy; safe for a consumer to resume.
    Ready = 3,
    /// FC deliberately stopped.
    Stopped = 4,
}

impl FcState {
    /// Decode a control-block word; unknown values map to `Init`
    /// (the most conservative interpretation: not ready).
    pub fn from_u32(v: u32) -> Self {
        match v {
            1 => FcState::Resetting,
            2 => FcState::Converging,
            3 => FcState::Ready,
            4 => FcState::Stopped,
            _ => FcState::Init,
        }
    }
}

/// Attach-time validation failure. Every variant names what was
/// found so a mismatch is diagnosable from the error alone.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AttachError {
    /// First eight bytes were not [`MAGIC`].
    BadMagic {
        /// Value found at offset 0.
        found: u64,
    },
    /// Layout version differs from this crate's.
    VersionMismatch {
        /// Version found in the header.
        found: u32,
    },
    /// The writer declared a different structure size than this
    /// crate compiled.
    DeclaredSizeMismatch {
        /// `declared_size` found in the header.
        found: u32,
    },
    /// The mapped object is smaller than the structure. (Larger is
    /// legal: macOS rounds `st_size` up to the page, so the check is
    /// `actual < expected` fails — never `==`.)
    MappingTooSmall {
        /// Object size reported by the OS.
        actual: usize,
    },
}

/// Fail-closed attach validation (#262): magic, layout version,
/// declared size, and mapped-object size must all agree before a
/// single payload field is interpreted.
pub fn validate_attach(
    magic: u64,
    layout_version: u32,
    declared_size: u32,
    actual_object_size: usize,
) -> Result<(), AttachError> {
    if actual_object_size < EXPECTED_SIZE {
        return Err(AttachError::MappingTooSmall {
            actual: actual_object_size,
        });
    }
    if magic != MAGIC {
        return Err(AttachError::BadMagic { found: magic });
    }
    if layout_version != LAYOUT_VERSION {
        return Err(AttachError::VersionMismatch {
            found: layout_version,
        });
    }
    if declared_size as usize != EXPECTED_SIZE {
        return Err(AttachError::DeclaredSizeMismatch {
            found: declared_size,
        });
    }
    Ok(())
}

// Layout freeze: any drift in size or field offset is a compile
// error here before it can become a cross-process runtime bug.
const _: () = {
    use core::mem::{offset_of, size_of};
    assert!(size_of::<SharedStateHeader>() == 32);
    assert!(size_of::<ModelStateBlock>() == 136);
    assert!(size_of::<MotorCommandBlock>() == 88);
    assert!(size_of::<ControlBlock>() == 48);
    assert!(size_of::<SharedStateV2>() == 304);
    assert!(EXPECTED_SIZE == 304);

    assert!(offset_of!(SharedStateV2, header) == 0);
    assert!(offset_of!(SharedStateV2, state) == 32);
    assert!(offset_of!(SharedStateV2, command) == 168);
    assert!(offset_of!(SharedStateV2, control) == 256);

    assert!(offset_of!(SharedStateHeader, magic) == 0);
    assert!(offset_of!(SharedStateHeader, layout_version) == 8);
    assert!(offset_of!(SharedStateHeader, declared_size) == 12);
    assert!(offset_of!(SharedStateHeader, reset_generation) == 16);
    assert!(offset_of!(SharedStateHeader, plugin_ready) == 20);

    assert!(offset_of!(ModelStateBlock, seq) == 0);
    assert!(offset_of!(ModelStateBlock, sim_step) == 8);
    assert!(offset_of!(ModelStateBlock, time_us) == 16);
    assert!(offset_of!(ModelStateBlock, pos) == 24);
    assert!(offset_of!(ModelStateBlock, quat) == 48);
    assert!(offset_of!(ModelStateBlock, vel) == 80);
    assert!(offset_of!(ModelStateBlock, ang_vel) == 104);
    assert!(offset_of!(ModelStateBlock, valid) == 128);

    assert!(offset_of!(MotorCommandBlock, seq) == 0);
    assert!(offset_of!(MotorCommandBlock, num_motors) == 4);
    assert!(offset_of!(MotorCommandBlock, motor_vel) == 8);
    assert!(offset_of!(MotorCommandBlock, fc_step_ack) == 72);

    assert!(offset_of!(ControlBlock, lifecycle_request) == 0);
    assert!(offset_of!(ControlBlock, lifecycle_request_nonce) == 4);
    assert!(offset_of!(ControlBlock, lifecycle_ack_nonce) == 8);
    assert!(offset_of!(ControlBlock, fc_state) == 12);
    assert!(offset_of!(ControlBlock, fc_state_generation) == 16);
    assert!(offset_of!(ControlBlock, fc_session_nonce) == 20);
    assert!(offset_of!(ControlBlock, lockstep_enabled) == 24);
    assert!(offset_of!(ControlBlock, target_rtf_percent) == 28);
};

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
mod tests {
    use super::*;

    #[test]
    fn attach_accepts_page_rounded_sizes() {
        // macOS rounds st_size up to the page (16 KiB on Apple
        // Silicon): 16384 for a 304-byte ftruncate must PASS.
        validate_attach(MAGIC, LAYOUT_VERSION, EXPECTED_SIZE as u32, 16384).unwrap();
        validate_attach(MAGIC, LAYOUT_VERSION, EXPECTED_SIZE as u32, EXPECTED_SIZE).unwrap();
    }

    #[test]
    fn attach_fails_closed_on_every_mismatch() {
        assert!(matches!(
            validate_attach(0xDEAD, LAYOUT_VERSION, EXPECTED_SIZE as u32, 16384),
            Err(AttachError::BadMagic { .. })
        ));
        assert!(matches!(
            validate_attach(MAGIC, 1, EXPECTED_SIZE as u32, 16384),
            Err(AttachError::VersionMismatch { found: 1 })
        ));
        assert!(matches!(
            validate_attach(MAGIC, LAYOUT_VERSION, 216, 16384),
            Err(AttachError::DeclaredSizeMismatch { found: 216 })
        ));
        assert!(matches!(
            validate_attach(MAGIC, LAYOUT_VERSION, EXPECTED_SIZE as u32, 216),
            Err(AttachError::MappingTooSmall { actual: 216 })
        ));
    }

    #[test]
    fn magic_literal_is_ascii_aviategz() {
        assert_eq!(MAGIC, u64::from_be_bytes(*b"AVIATEGZ"));
    }

    #[test]
    fn unknown_wire_words_decode_conservatively() {
        assert_eq!(LifecycleRequest::from_u32(99), LifecycleRequest::None);
        assert_eq!(FcState::from_u32(99), FcState::Init);
        assert_eq!(LifecycleRequest::from_u32(1), LifecycleRequest::Reset);
        assert_eq!(FcState::from_u32(3), FcState::Ready);
    }
}
