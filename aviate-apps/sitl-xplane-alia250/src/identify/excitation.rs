//! The probe script: what the identification flight commands, phase by
//! phase. The experiment loop in the parent module owns time and the
//! board; everything here is a pure function of the phase it is handed.

use std::time::Duration;

use aviate_core::control::{Command, CommandSource, ControlMode, Setpoint};
use aviate_core::math::Quaternion;
use aviate_core::types::NormalizedThrust;

/// Injected differential force per excited axis. Large enough that
/// the response dominates the closed loop's own corrections, small
/// enough to stay inside lane range at hover collective.
// Small enough that the closed attitude loop can host the probe
// without railing a lane; the fit reads gyro rates, whose response to
// even this differential is orders above the sensor floor. A probe the
// wire has to clip is a probe the plant never received.
const INJECT_FORCE: f32 = 0.2;

/// The probe frequencies, rad/s. The lower sits where the plant is
/// integrator-like and gives the cleanest K; the higher sits at the
/// crossover the gain design targets and exposes the spool lag as
/// phase. Two points also cross-check each other — a channel whose K
/// disagrees between them is telling you its measurement is polluted.
pub(super) const PROBE_RAD_S: [f32; 2] = [1.0, 2.5];

/// Excitation lengths per probe: at least three cycles each, with a
/// margin over exactly three. The report measures the window as the
/// span between its first and last recorded samples, which is up to a
/// sample interval shorter at each edge than the commanded window — a
/// length that buys 3.02 periods therefore counts 2 blocks and fails
/// the fit's floor on a clock the experiment itself accepts.
pub(super) const EXCITE_S: [f32; 2] = [23.0, 11.0];

/// Climb phase length before the experiment.
pub(super) const CLIMB: Duration = Duration::from_secs(22);

/// Settle phase between axes.
pub(super) const SETTLE: Duration = Duration::from_millis(1_500);

/// Transient rejected from the head of every excitation window.
pub(super) const TRANSIENT_SKIP: Duration = Duration::from_millis(1_800);

/// Per-SAMPLE blend of the held attitude toward the live estimate,
/// advanced on the sample clock. At the declared 80 Hz this sets a
/// restoring time constant near three seconds — an order below the
/// slowest probe frequency, an order above the rollover rate. A blend
/// advanced per loop spin instead would tighten with machine speed and
/// with the simulator's dilation, neither of which is a property of
/// the experiment.
pub(super) const HOLD_LEAK: f32 = 0.004;

/// The reversed-spin mixer's per-motor axis signs, in mixer lane
/// order — the same table the mixer applies, inverted here to
/// reconstruct the axis torque the controller actually commanded from
/// the motor outputs.
pub(super) const SIGNS: [[f32; 4]; 3] = [
    [-1.0, 1.0, 1.0, -1.0], // roll
    [1.0, -1.0, 1.0, -1.0], // pitch
    [-1.0, -1.0, 1.0, 1.0], // yaw
];

pub(super) const AXIS_NAMES: [&str; 3] = ["roll", "pitch", "yaw"];

pub(super) enum Phase {
    WaitReady,
    Climb {
        started_us: u64,
        until_us: u64,
    },
    Settle {
        axis: usize,
        freq: usize,
        until_us: u64,
    },
    Excite {
        axis: usize,
        freq: usize,
        started_us: u64,
        until_us: u64,
    },
    Lower {
        ground_y: f32,
        last_us: u64,
    },
    SettleGround {
        until_us: u64,
    },
    Done,
}

/// The command each phase holds. `None` keeps the previous command.
/// The EXCITATION does not travel through here — it is injected on the
/// actuator lanes — so every phase simply asks the closed loop to hold
/// `attitude`: the live estimate outside the windows, the caller's
/// leaky hold inside them (commanding a fixed world frame instead
/// would have the loop fighting the vehicle's heading with saturated
/// torque, drowning the probe). `sample_us` is the SAMPLE clock; the
/// climb ramp is paced in simulation time like every other deadline.
pub(super) fn command_for(
    phase: &Phase,
    sample_us: u64,
    sequence: u32,
    attitude: Quaternion,
    hover_collective: f32,
) -> Option<Command> {
    let setpoint = match phase {
        Phase::WaitReady => return None,
        Phase::Climb { started_us, .. } => {
            // Six seconds to full ramp, held for the rest: the rotor
            // inertia needs the time regardless of the command.
            let elapsed_s = sample_us.saturating_sub(*started_us) as f32 / 1_000_000.0;
            let ramp = (elapsed_s / 6.0).min(1.0);
            let target = hover_collective + 0.08;
            let collective = ramp * target;
            Setpoint {
                attitude: Some(attitude),
                collective_thrust: NormalizedThrust(collective),
                ..Setpoint::default()
            }
        }
        Phase::Settle { .. } | Phase::Excite { .. } => Setpoint {
            attitude: Some(attitude),
            collective_thrust: NormalizedThrust(hover_collective),
            ..Setpoint::default()
        },
        // Enough spin to keep the attitude loop alive, little enough
        // that releasing the stand on the ground stays a landing.
        Phase::Lower { .. } | Phase::SettleGround { .. } | Phase::Done => Setpoint {
            attitude: Some(attitude),
            collective_thrust: NormalizedThrust(hover_collective * 0.3),
            ..Setpoint::default()
        },
    };
    Some(Command {
        mode: ControlMode::Attitude,
        setpoint,
        config_mode_request: None,
        sensor_overrides: None,
        sequence,
        source: CommandSource::Autopilot,
    })
}

/// Normalized linear blend from `from` toward `to`, hemisphere-aligned.
/// At the small per-cycle fractions used here it is indistinguishable
/// from the spherical blend and needs no trigonometry.
pub(super) fn leak_toward(from: Quaternion, to: Quaternion, alpha: f32) -> Quaternion {
    let dot = from.w * to.w + from.x * to.x + from.y * to.y + from.z * to.z;
    let sign = if dot < 0.0 { -1.0 } else { 1.0 };
    let blended = Quaternion::new(
        from.w + alpha * (sign * to.w - from.w),
        from.x + alpha * (sign * to.x - from.x),
        from.y + alpha * (sign * to.y - from.y),
        from.z + alpha * (sign * to.z - from.z),
    );
    blended.normalize()
}

/// The probe on the excited axis's lanes. `sample_us` is the SAMPLE
/// clock: the fit demodulates against sample timestamps, and a sine
/// advanced on any other clock arrives at the wrong stamped frequency
/// by the simulator's dilation factor.
pub(super) fn injection_for(phase: &Phase, sample_us: u64) -> [f32; 4] {
    let Phase::Excite {
        axis,
        freq,
        started_us,
        ..
    } = phase
    else {
        return [0.0; 4];
    };
    let elapsed_s = sample_us.saturating_sub(*started_us) as f32 / 1_000_000.0;
    let value = libm::sinf(PROBE_RAD_S[*freq] * elapsed_s) * INJECT_FORCE;
    let mut lanes = [0.0; 4];
    for (lane, sign) in lanes.iter_mut().zip(SIGNS[*axis]) {
        *lane = sign * value;
    }
    lanes
}

pub(super) fn add_duration(timestamp_us: u64, duration: Duration) -> u64 {
    let micros = u64::try_from(duration.as_micros()).unwrap_or(u64::MAX);
    timestamp_us.saturating_add(micros)
}
