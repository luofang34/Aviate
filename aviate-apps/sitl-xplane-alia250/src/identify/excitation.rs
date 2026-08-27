//! The probe script: what the identification flight commands, phase by
//! phase. The experiment loop in the parent module owns time and the
//! board; everything here is a pure function of the phase it is handed.

use std::time::Duration;

use aviate_core::control::{Command, CommandSource, ControlMode, Setpoint};
use aviate_core::math::Quaternion;
use aviate_core::types::NormalizedThrust;

/// Injected differential force per excited axis. The binding bound is
/// the LANE CEILING: the rate loop answers the probe's rocking with
/// corrections a few times the injection, and probe plus answer must
/// stay inside the wire's per-lane range or the census refuses the
/// window as clipped. Yaw drives harder because its plant authority —
/// rotor drag, not thrust asymmetry — is an order below roll and
/// pitch, and at the upper probe frequency its response otherwise
/// sinks toward the gyro noise floor; its softened host loop leaves
/// the lane headroom the larger probe spends.
const INJECT_FORCE: [f32; 3] = [0.06, 0.06, 0.08];

/// The probe frequencies, rad/s. Both sit where FREE FLIGHT can host
/// the probe: an integrator plant turns a lane differential u into an
/// attitude excursion of K·u/ω², so at a plant authority near five the
/// injected amplitude demands roughly five degrees at 2.5 rad/s and
/// under two degrees at 5 rad/s — excursions the attitude loop rides out. A
/// probe an octave lower demands tens of degrees and rolls the vehicle
/// over; no fit survives a crash. Two points still cross-check each
/// other, and the spool lag shows as phase at the upper one.
pub(super) const PROBE_RAD_S: [f32; 2] = [2.5, 5.0];

/// Excitation lengths per probe: at least three cycles each after the
/// transient skip, with a margin over exactly three. The report
/// measures the window as the span between its first and last recorded
/// samples, which is up to a sample interval shorter at each edge than
/// the commanded window — a length that buys 3.02 periods therefore
/// counts 2 blocks and fails the fit's floor on a clock the experiment
/// itself accepts.
// The upper probe runs as long as the lower even though its periods
// are shorter: the fit's block estimates are one period each, and both
// the coherence and the census are RATIOS whose run-to-run variance
// shrinks with block count. Short upper windows made every gate a
// coin flip at the margin.
pub(super) const EXCITE_S: [f32; 2] = [16.0, 11.0];

/// Climb budget: the fail-closed bound on reaching the working height.
/// Rotor spool from rest is the slow part and varies with the machine;
/// the climb itself ends on achieved height, never on this clock.
pub(super) const CLIMB: Duration = Duration::from_secs(40);

/// Settle phase between windows: long enough for the trim to win back
/// the altitude the probe's oscillation costs, so consecutive windows
/// do not compound a sag into the ground guard.
pub(super) const SETTLE: Duration = Duration::from_millis(4_000);

/// Settle after the climb, before the FIRST window. The climb hands
/// over with vertical speed still decaying and the trim near its clamp;
/// the early windows otherwise inherit that transient as clipping the
/// census refuses. The later windows need only [`SETTLE`].
pub(super) const FIRST_SETTLE: Duration = Duration::from_secs(20);

/// Transient rejected from the head of every excitation window.
pub(super) const TRANSIENT_SKIP: Duration = Duration::from_millis(1_800);

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
/// actuator lanes beneath whatever the loop commands.
///
/// The climb and the windows fly under the VELOCITY loop, holding a
/// climb rate and then zero velocity: the loop that demonstrably flies
/// this vehicle also owns collective and attitude through the
/// experiment, so height keeps itself and the probe's tilt excursions
/// are ridden out. The loop's own corrections live an order below the
/// probe frequencies, and the fit reads the TOTAL applied input, so
/// they shape the probe's spectrum without invalidating it. The
/// landing phases drop to a low fixed collective in attitude mode so
/// releasing the stand on the ground stays a landing.
pub(super) fn command_for(
    phase: &Phase,
    sequence: u32,
    attitude: Quaternion,
    collective: f32,
) -> Option<Command> {
    let setpoint = match phase {
        Phase::WaitReady => return None,
        // The climb and the windows hold the live attitude while the
        // caller's slew-limited trim rides the collective: the vertical
        // ESTIMATE this kernel flies is not trustworthy enough for the
        // velocity loop, so height is kept on the GPS fix by the
        // experiment itself, gently enough not to fight rotor spool.
        Phase::Climb { .. } | Phase::Settle { .. } | Phase::Excite { .. } => Setpoint {
            attitude: Some(attitude),
            collective_thrust: NormalizedThrust(collective),
            ..Setpoint::default()
        },
        // Enough spin to keep the attitude loop alive, little enough
        // that releasing the stand on the ground stays a landing.
        Phase::Lower { .. } | Phase::SettleGround { .. } | Phase::Done => Setpoint {
            attitude: Some(attitude),
            collective_thrust: NormalizedThrust(collective * 0.3),
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
    let value = libm::sinf(PROBE_RAD_S[*freq] * elapsed_s) * INJECT_FORCE[*axis];
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
