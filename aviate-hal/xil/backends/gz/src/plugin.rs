//! FC-side client for the AviateGzPlugin shared-memory block.
//!
//! Pure Rust over `aviate-xil-shm`: fail-closed fingerprint
//! attach, proper seqlock reads, coherent `{generation, step, time,
//! state}` snapshots. No C FFI remains on this path — the C++ in
//! this backend is only the gz-sim system plugin itself.

use aviate_xil_contract::{WriterState, SHM_NAME_BASE};
use aviate_xil_shm::{AttachFailure, FcSession, HostSession};
use log::info;

/// Model state from gz-sim (SI units, ENU world / FLU body).
#[derive(Debug, Clone, Copy, Default)]
pub struct AviateModelState {
    /// Position in world frame [x, y, z] (meters, ENU)
    pub pos: [f64; 3],
    /// Orientation quaternion [w, x, y, z]
    pub quat: [f64; 4],
    /// Linear velocity in world frame [vx, vy, vz] (m/s)
    pub vel: [f64; 3],
    /// Angular velocity [wx, wy, wz] (rad/s) in the WORLD ENU frame
    /// — gz's `WorldAngularVelocity` verbatim, not a body gyro, and
    /// known to report zero while the vehicle rotates. The X500
    /// synth path derives body rates from successive `quat` samples
    /// instead of reading this.
    pub ang_vel: [f64; 3],
    /// Timestamp (simulation time in microseconds)
    pub time_us: u64,
    /// Physics step this snapshot belongs to (coherent with the
    /// payload — taken under the same seqlock read).
    pub sim_step: u64,
    /// Simulation-world epoch this snapshot belongs to (coherent
    /// with the payload; changes on every world reset).
    pub reset_generation: u32,
    /// Valid flag (non-zero if data is valid)
    pub valid: i32,
}

/// Motor command for gz-sim (boundary rotor speeds, rad/s).
#[derive(Debug, Clone, Copy)]
pub struct AviateMotorCommand {
    /// Motor velocities in rad/s (up to 8 motors)
    pub velocities: [f64; 8],
    /// Number of motors (typically 4 for quadcopter)
    pub num_motors: i32,
}

impl Default for AviateMotorCommand {
    fn default() -> Self {
        Self {
            velocities: [0.0; 8],
            num_motors: 4,
        }
    }
}

/// Error type for GzPluginBridge operations
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GzPluginError {
    /// Bridge not initialized or shared memory not available
    NotInitialized,
    /// Plugin not running (shared memory doesn't exist or has not
    /// published readiness yet)
    PluginNotRunning,
    /// The shm object exists but is not a valid contract block
    /// (wrong magic / layout version / size) — fail closed.
    ContractMismatch,
    /// Data not valid yet
    DataNotValid,
    /// Failed to set motor speeds
    MotorCommandFailed,
}

impl std::fmt::Display for GzPluginError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::NotInitialized => write!(f, "GzPluginBridge not initialized"),
            Self::PluginNotRunning => write!(f, "AviateGzPlugin not running in Gazebo"),
            Self::ContractMismatch => {
                write!(f, "shared block failed the aviate-xil-contract fingerprint")
            }
            Self::DataNotValid => write!(f, "Model state data not valid"),
            Self::MotorCommandFailed => write!(f, "Failed to send motor command"),
        }
    }
}

impl std::error::Error for GzPluginError {}

/// Safe wrapper around the gz-sim plugin's shared block.
///
/// Supports multi-vehicle simulation via instance IDs.
pub struct GzPluginBridge {
    session: FcSession,
    instance: u8,
}

fn shm_name(instance: u8) -> String {
    if instance == 0 {
        SHM_NAME_BASE.to_string()
    } else {
        format!("{SHM_NAME_BASE}_{instance}")
    }
}

/// A nonzero value that never repeats across FC sessions on this
/// host: pid folded with the wall clock, so a restarted FC (even
/// with a reused pid, even within the same instant's resolution)
/// stamps a different identity than the session it replaced. Zero is
/// reserved for "no FC has attached".
fn fresh_session_nonce() -> u32 {
    let pid = std::process::id();
    let nanos = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_nanos() as u32)
        .unwrap_or(0);
    (nanos.rotate_left(8) ^ pid) | 1
}

impl GzPluginBridge {
    /// Connect to instance 0.
    pub fn new() -> Result<Self, GzPluginError> {
        Self::for_instance(0)
    }

    /// Connect to a specific vehicle instance. Fails closed if the
    /// block exists but does not carry the expected contract
    /// fingerprint.
    pub fn for_instance(instance: u8) -> Result<Self, GzPluginError> {
        match FcSession::attach(&shm_name(instance)) {
            Ok(session) => {
                // Every attachment IS a new FC session: stamping the
                // nonce here (rather than trusting each binary to
                // remember) is what lets the host tell "the same FC,
                // still alive" from "an FC restarted behind my back".
                session.set_fc_session_nonce(fresh_session_nonce());
                Ok(Self { session, instance })
            }
            Err(AttachFailure::Io(_)) | Err(AttachFailure::NotReady) => {
                Err(GzPluginError::PluginNotRunning)
            }
            Err(AttachFailure::Contract(_)) => Err(GzPluginError::ContractMismatch),
        }
    }

    /// Try to connect to the bridge, retrying while the plugin is
    /// not up yet. A contract mismatch aborts immediately — retrying
    /// cannot fix a foreign layout.
    pub fn connect_with_retry(max_attempts: u32, delay_ms: u64) -> Result<Self, GzPluginError> {
        Self::connect_instance_with_retry(0, max_attempts, delay_ms)
    }

    /// Retry variant of [`Self::for_instance`].
    pub fn connect_instance_with_retry(
        instance: u8,
        max_attempts: u32,
        delay_ms: u64,
    ) -> Result<Self, GzPluginError> {
        for attempt in 0..max_attempts {
            match Self::for_instance(instance) {
                Ok(bridge) => {
                    if attempt > 0 {
                        info!(
                            "[GzPluginBridge] Instance {} connected after {} attempts",
                            instance,
                            attempt + 1
                        );
                    }
                    return Ok(bridge);
                }
                Err(GzPluginError::PluginNotRunning) => {
                    std::thread::sleep(std::time::Duration::from_millis(delay_ms));
                }
                Err(e) => return Err(e),
            }
        }
        Err(GzPluginError::PluginNotRunning)
    }

    /// Get the instance ID this bridge is connected to
    pub fn instance(&self) -> u8 {
        self.instance
    }

    /// One coherent `{step, time, state}` snapshot, or `None` until
    /// the first physics step publishes.
    pub fn get_model_state(&self) -> Option<AviateModelState> {
        let s = self.session.read_model_state()?;
        Some(AviateModelState {
            pos: s.pos,
            quat: s.quat,
            vel: s.vel,
            ang_vel: s.ang_vel,
            time_us: s.time_us,
            sim_step: s.sim_step,
            reset_generation: s.reset_generation,
            valid: 1,
        })
    }

    /// Publish boundary rotor-speed commands (rad/s). The resolved
    /// actuator curve is applied BEFORE this call.
    pub fn set_motor_speeds(&self, velocities: &[f64]) -> Result<(), GzPluginError> {
        self.session.write_motor_command(velocities);
        Ok(())
    }

    /// Simulation time in microseconds (coherent snapshot; 0 before
    /// the first step).
    pub fn sim_time_us(&self) -> u64 {
        self.session
            .read_model_state()
            .map(|s| s.time_us)
            .unwrap_or(0)
    }

    /// Whether the plugin currently owns the block.
    pub fn is_connected(&self) -> bool {
        self.session.plugin_ready()
    }

    /// Simulation-world epoch: bumps on every world reset.
    /// On a change, re-run estimator convergence and re-establish
    /// any freshness tracking — do not quarantine.
    pub fn reset_generation(&self) -> u32 {
        self.session.reset_generation()
    }

    /// What the plugin's shm name resolves to right now.
    /// [`WriterState::Current`] is the only state in which this
    /// bridge's reads and writes reach the live simulator.
    pub fn writer_state(&self) -> WriterState {
        self.session.writer_state()
    }

    /// Re-attach to whatever object the name resolves to now.
    ///
    /// Without this a bridge outlives its simulator: after a plugin
    /// restart the old mapping keeps answering — serving the dead
    /// world's final snapshot and swallowing every motor command
    /// into memory no one reads — while looking perfectly healthy.
    /// Callers drive it from [`Self::writer_state`].
    pub fn reconnect(&mut self) -> Result<(), GzPluginError> {
        self.session = FcSession::attach(&shm_name(self.instance)).map_err(|e| match e {
            AttachFailure::Contract(_) => GzPluginError::ContractMismatch,
            _ => GzPluginError::PluginNotRunning,
        })?;
        // A re-attachment is a new session to whoever is watching
        // the control block, same as the initial attach.
        self.session.set_fc_session_nonce(fresh_session_nonce());
        Ok(())
    }

    /// Current physics step count (coherent snapshot; 0 before the
    /// first step).
    pub fn sim_step(&self) -> u64 {
        self.session
            .read_model_state()
            .map(|s| s.sim_step)
            .unwrap_or(0)
    }

    /// Acknowledge a processed step: the lockstep gate the plugin
    /// blocks on, and the FC liveness heartbeat.
    pub fn ack_step(&self, step: u64) {
        self.session.ack_step(step);
    }

    /// Request the lockstep gate. The plugin blocks each physics
    /// step on [`Self::ack_step`] only when its SDF arms lockstep
    /// AND this word is set.
    ///
    /// Lockstep is a SESSION control, and the gate word has exactly
    /// one writer role — the session host — so this opens a
    /// transient host endpoint instead of writing it through the
    /// FC's mapping. Callers reaching this method are harness /
    /// mission drivers acting in that role.
    ///
    /// Fails loudly rather than no-op'ing: a harness that believes
    /// it is stepping deterministically while the simulator
    /// free-runs would produce quietly non-reproducible evidence.
    pub fn set_lockstep(&self, enabled: bool) -> Result<(), GzPluginError> {
        let host = HostSession::attach(&shm_name(self.instance)).map_err(|e| match e {
            AttachFailure::Contract(_) => GzPluginError::ContractMismatch,
            _ => GzPluginError::PluginNotRunning,
        })?;
        host.set_lockstep(enabled);
        Ok(())
    }

    /// Whether the lockstep gate word is currently set.
    pub fn lockstep_enabled(&self) -> bool {
        self.session.lockstep_enabled()
    }

    /// Wait for a new simulation step and process it: waits for
    /// `sim_step > last_step`, hands the coherent state to
    /// `processor`, acks the step. Returns `None` on timeout.
    pub fn wait_and_process<F, R>(
        &self,
        last_step: u64,
        timeout_us: u64,
        processor: F,
    ) -> Option<(u64, R)>
    where
        F: FnOnce(&AviateModelState) -> R,
    {
        let start = std::time::Instant::now();
        let timeout = std::time::Duration::from_micros(timeout_us);

        loop {
            if let Some(state) = self.get_model_state() {
                if state.sim_step > last_step {
                    let step = state.sim_step;
                    let result = processor(&state);
                    self.ack_step(step);
                    return Some((step, result));
                }
            }

            if start.elapsed() >= timeout {
                return None;
            }

            std::thread::sleep(std::time::Duration::from_micros(10));
        }
    }
}

/// Convert ENU position to NED
///
/// Gazebo uses ENU (East-North-Up), MAVLink uses NED (North-East-Down)
/// - ENU x (east)  -> NED y (east)
/// - ENU y (north) -> NED x (north)
/// - ENU z (up)    -> NED z (down, negated)
#[inline]
pub fn enu_to_ned(enu: [f64; 3]) -> [f64; 3] {
    [enu[1], enu[0], -enu[2]]
}

/// Convert ENU velocity to NED
#[inline]
pub fn enu_vel_to_ned(enu_vel: [f64; 3]) -> [f64; 3] {
    [enu_vel[1], enu_vel[0], -enu_vel[2]]
}

/// Convert ENU position to NED (f32 version)
#[inline]
pub fn enu_to_ned_f32(enu: [f64; 3]) -> [f32; 3] {
    [enu[1] as f32, enu[0] as f32, -enu[2] as f32]
}

/// Convert ENU velocity to NED (f32 version)
#[inline]
pub fn enu_vel_to_ned_f32(enu_vel: [f64; 3]) -> [f32; 3] {
    [enu_vel[1] as f32, enu_vel[0] as f32, -enu_vel[2] as f32]
}

/// Convert a body→world orientation quaternion from gz's
/// ENU-world / FLU-body convention to NED-world / FRD-body
/// (the convention every aviate consumer expects).
///
/// Composition:
/// * **World ENU → NED**: rotation by 180° about the East-North
///   bisector (`q_ENU→NED = (0, √½, √½, 0)`). Equivalent to
///   negating Z and swapping X/Y.
/// * **Body FRD → FLU**: 180° rotation about the forward (X)
///   axis, `q_FRD→FLU = (0, 1, 0, 0)`.
///
/// For the same physical attitude:
/// `q_NED_FRD = q_ENU→NED · q_ENU_FLU · q_FRD→FLU`
#[inline]
pub fn enu_quat_to_ned_f32(q_enu_flu: [f64; 4]) -> [f32; 4] {
    let s = core::f32::consts::FRAC_1_SQRT_2;
    let w = q_enu_flu[0] as f32;
    let x = q_enu_flu[1] as f32;
    let y = q_enu_flu[2] as f32;
    let z = q_enu_flu[3] as f32;
    [s * (w + z), s * (x + y), s * (x - y), s * (w - z)]
}

/// Body-frame vector ENU/FLU → NED/FRD.
///
/// FLU body = (forward, left, up); FRD body = (forward, right,
/// down). Flip Y and Z; X is forward in both.
#[inline]
pub fn flu_to_frd_f32(v_flu: [f64; 3]) -> [f32; 3] {
    [v_flu[0] as f32, -v_flu[1] as f32, -v_flu[2] as f32]
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
mod tests {
    use super::*;

    #[test]
    fn test_enu_to_ned() {
        // ENU: x=east, y=north, z=up
        // NED: x=north, y=east, z=down
        let enu = [1.0, 2.0, 3.0]; // 1m east, 2m north, 3m up
        let ned = enu_to_ned(enu);
        assert_eq!(ned, [2.0, 1.0, -3.0]); // 2m north, 1m east, 3m down
    }

    #[test]
    fn test_motor_command_default() {
        let cmd = AviateMotorCommand::default();
        assert_eq!(cmd.num_motors, 4);
        assert_eq!(cmd.velocities, [0.0; 8]);
    }

    #[test]
    fn bridge_fails_closed_without_plugin() {
        assert!(matches!(
            GzPluginBridge::for_instance(200),
            Err(GzPluginError::PluginNotRunning)
        ));
    }
}
