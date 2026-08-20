//! In-simulator plant identification: measures each axis's angular
//! control authority instead of guessing it.
//!
//! The attitude-cascade derivation in the kernel crate needs the plant
//! authority K — angular acceleration per unit normalized torque. K is
//! airframe knowledge nobody should estimate by hand: an estimate off
//! by a factor of a few flips the cascade from sluggish to bang-bang,
//! and each wrong guess costs a full flight to discover.
//!
//! The experiment mirrors PX4's autotune shape but runs OFFLINE in
//! SITL: fly a short hop, hold an attitude-setpoint square wave one
//! axis at a time, and fit the open-loop relation ẏ = K·u from what was
//! MEASURED — u is the mixer's own force-domain axis torque (linear in
//! thrust under the quadratic rotor curve), y the bridge's gyro. The
//! closed loop only shapes the input spectrum; the fit does not depend
//! on the loop being any good, which is the point — it runs under
//! known-marginal gains to produce the numbers that fix them.
//!
//! The result prints as a plant-identity block to paste into the
//! kernel's derivation, with fit quality (R²) and the input-output lag
//! so a bad experiment is visible rather than silently trusted.

use std::net::UdpSocket;
use std::time::{Duration, Instant};

use aviate_core::control::{Command, CommandSource, ControlMode, Setpoint};
use aviate_core::ekf::Estimator as _;
use aviate_core::math::Quaternion;
use aviate_core::types::NormalizedThrust;

use aviate_board_sitl_xplane::XPlaneBoard;

mod report;
mod stand;
mod yaw_sign;

pub use yaw_sign::run_yaw_sign;

use report::report;
use stand::{Sample, TestStand};

/// Force-domain collective held through the experiment: the kernel's
/// own hover trim (env-overridable, so a new airframe's sweep result
/// flows straight in); vertical drift over the experiment's seconds is
/// acceptable and irrelevant to the angular fit.
fn collective() -> f32 {
    std::env::var("AVIATE_HOVER_TRIM")
        .ok()
        .and_then(|value| value.parse().ok())
        .unwrap_or(0.43)
}

/// Injected differential force per excited axis. Large enough that
/// the response dominates the closed loop's own corrections, small
/// enough to stay inside lane range at hover collective.
const INJECT_FORCE: f32 = 0.2;

/// The probe frequencies, rad/s. The lower sits where the plant is
/// integrator-like and gives the cleanest K; the higher sits at the
/// crossover the gain design targets and exposes the spool lag as
/// phase. Two points also cross-check each other — a channel whose K
/// disagrees between them is telling you its measurement is polluted.
const PROBE_RAD_S: [f32; 2] = [1.0, 2.5];

/// Excitation lengths per probe: at least three cycles each.
const EXCITE_S: [f32; 2] = [19.0, 8.0];

/// Climb phase length before the experiment.
const CLIMB: Duration = Duration::from_secs(22);

/// Settle phase between axes.
const SETTLE: Duration = Duration::from_millis(600);

/// The reversed-spin mixer's per-motor axis signs, in mixer lane
/// order — the same table the mixer applies, inverted here to
/// reconstruct the axis torque the controller actually commanded from
/// the motor outputs.
const SIGNS: [[f32; 4]; 3] = [
    [-1.0, 1.0, 1.0, -1.0], // roll
    [1.0, -1.0, 1.0, -1.0], // pitch
    [-1.0, -1.0, 1.0, 1.0], // yaw
];

const AXIS_NAMES: [&str; 3] = ["roll", "pitch", "yaw"];

enum Phase {
    WaitReady,
    Climb {
        started: Instant,
        until: Instant,
    },
    Settle {
        axis: usize,
        freq: usize,
        until: Instant,
    },
    Excite {
        axis: usize,
        freq: usize,
        until: Instant,
    },
    Done,
}

/// Runs a grounded collective staircase so the thrust curve can be
/// read against the simulator's own prop-force dataref: 12 steps, held
/// long enough for RPM to settle, printed with a timestamp to pair
/// with an RREF watcher. The vehicle stays on its gear the whole time
/// (a step that lifts it ends the sweep early in the log, which is
/// itself a data point).
pub fn run_sweep<C, M>(board: &mut XPlaneBoard<C, M>)
where
    C: aviate_core::control::VehicleController,
    M: aviate_core::mixer::Mixer,
{
    const STEPS: usize = 12;
    const HOLD: Duration = Duration::from_millis(2500);
    let mut sequence: u32 = 0;
    let mut last_heartbeat = Instant::now();
    let started = Instant::now();
    let mut armed = false;
    let mut step_started: Option<Instant> = None;
    let mut step: usize = 0;
    let mut step_printed = false;

    log::info!("collective sweep: waiting for the link");
    loop {
        let now = Instant::now();
        if last_heartbeat.elapsed() >= Duration::from_secs(1) {
            board.send_heartbeat();
            last_heartbeat = now;
        }
        if started.elapsed() > Duration::from_secs(120) {
            log::error!("sweep timed out");
            return;
        }
        if !armed {
            if board.is_ready() {
                match board.arm() {
                    Ok(()) => {
                        armed = true;
                        step_started = Some(now);
                        log::info!("armed; sweeping");
                    }
                    Err(error) => log::warn!("arm refused: {error:?}"),
                }
            }
        } else if let Some(at) = step_started {
            if now.duration_since(at) >= HOLD {
                step += 1;
                step_started = Some(now);
                step_printed = false;
                if step >= STEPS {
                    let _ = board.disarm();
                    log::info!("sweep complete");
                    return;
                }
            }
        }
        if armed {
            // Hold the CURRENT attitude, not a fixed world frame: a
            // commanded identity quaternion carries the parking
            // heading as a huge yaw error, and the resulting torque
            // rail skids the vehicle over on its gear.
            let hold = board
                .kernel()
                .pipeline()
                .estimator
                .estimate(&board.kernel().state.estimator)
                .attitude;
            let force = step as f32 / (STEPS - 1) as f32;
            let cmd = Command {
                mode: ControlMode::Attitude,
                setpoint: Setpoint {
                    attitude: Some(hold),
                    collective_thrust: NormalizedThrust(force),
                    ..Setpoint::default()
                },
                config_mode_request: None,
                sensor_overrides: None,
                sequence,
                source: CommandSource::Autopilot,
            };
            sequence = sequence.wrapping_add(1);
            board.set_command(cmd);
        }
        let out = board.step();
        if armed
            && !step_printed
            && step_started
                .is_some_and(|at| now.duration_since(at) > HOLD - Duration::from_millis(300))
        {
            // One line late in each hold, after RPM settles.
            let alt = board.last_fix().map_or(0.0, |fix| fix.alt_m);
            let est = board
                .kernel()
                .pipeline()
                .estimator
                .estimate(&board.kernel().state.estimator);
            let (er, ep, _) = est.attitude.to_euler();
            let gyro = board.last_imu().map_or([0.0; 3], |imu| imu.gyro);
            log::info!(
                "sweep step={step} force={:.2} motors=[{:.2},{:.2},{:.2},{:.2}] est_rp=({:.2},{:.2}) gyro=[{:.2},{:.2},{:.2}] alt={alt:.1}",
                step as f32 / (STEPS - 1) as f32,
                out.outputs[0].0, out.outputs[1].0, out.outputs[2].0, out.outputs[3].0,
                er, ep, gyro[0], gyro[1], gyro[2],
            );
            step_printed = true;
        }
        if !board.wait_for_sample(Duration::from_micros(2_500)) && !board.connected() {
            std::thread::sleep(Duration::from_millis(2));
        }
    }
}

/// Runs the identification flight and prints the plant identity.
/// Returns when the experiment is over; the caller exits.
pub fn run<C, M>(board: &mut XPlaneBoard<C, M>)
where
    C: aviate_core::control::VehicleController,
    M: aviate_core::mixer::Mixer,
{
    let mut phase = Phase::WaitReady;
    let mut sequence: u32 = 0;
    let mut samples: Vec<Sample> = Vec::with_capacity(8192);
    let mut windows: [[(usize, usize); 2]; 3] = [[(0, 0); 2]; 3];
    let started = Instant::now();
    let mut last_heartbeat = Instant::now();
    let mut last_report = Instant::now();
    let mut last_motor_mean = 0.0_f32;
    let mut held_attitude: Option<Quaternion> = None;
    let mut stand = TestStand::new(match UdpSocket::bind("0.0.0.0:0") {
        Ok(sock) => {
            sock.set_read_timeout(Some(Duration::from_millis(100))).ok();
            sock
        }
        Err(error) => {
            log::error!("no UDP socket for the test stand: {error}");
            return;
        }
    });

    log::info!("identification flight: waiting for the link");
    loop {
        let now = Instant::now();
        if last_heartbeat.elapsed() >= Duration::from_secs(1) {
            board.send_heartbeat();
            last_heartbeat = now;
        }
        if last_report.elapsed() >= Duration::from_secs(1) {
            // A grounded experiment fits noise; the altitude trace is
            // what says whether the numbers below mean anything.
            let alt = board
                .last_fix()
                .map_or_else(|| "?".to_owned(), |fix| format!("{:.1}", fix.alt_m));
            let vz = board.last_fix().map_or(0.0, |fix| fix.vel_ned[2]);
            log::info!("alt={alt}m vz={vz:.2} motors_mean={last_motor_mean:.2}");
            last_report = Instant::now();
        }
        if started.elapsed() > Duration::from_secs(180) {
            log::error!("identification timed out before completing");
            return;
        }

        // The attitude the loop is asked to hold: live outside the
        // excitation windows, FROZEN at each window's start so the
        // setpoint carries no content correlated with the probe.
        let estimate = board
            .kernel()
            .pipeline()
            .estimator
            .estimate(&board.kernel().state.estimator);
        let hold = match (&phase, held_attitude) {
            (Phase::Excite { .. }, Some(frozen)) => frozen,
            _ => estimate.attitude,
        };

        // The probe: a sine of differential force on the excited
        // axis's lanes at the analysis frequency, so the correlation
        // fit reads the plant exactly where the loop design needs it.
        let injection = match &phase {
            Phase::Excite { axis, freq, until } => {
                let t = Duration::from_secs_f32(EXCITE_S[*freq])
                    .saturating_sub(until.saturating_duration_since(now))
                    .as_secs_f32();
                let value = (PROBE_RAD_S[*freq] * t).sin() * INJECT_FORCE;
                let mut lanes = [0.0_f32; 4];
                for (lane, s) in lanes.iter_mut().zip(SIGNS[*axis]) {
                    *lane = s * value;
                }
                (lanes, value)
            }
            _ => ([0.0; 4], 0.0),
        };
        board.set_lane_injection(injection.0);
        if matches!(phase, Phase::Settle { .. } | Phase::Excite { .. }) {
            stand.pin();
        }

        let command = command_for(&phase, now, sequence, hold);
        sequence = sequence.wrapping_add(1);
        if let Some(cmd) = command {
            board.set_command(cmd);
        }
        let out = board.step();
        last_motor_mean = out.outputs[..4].iter().map(|m| m.0).sum::<f32>() / 4.0;

        // Record every cycle that has an IMU sample behind it.
        if let Some(imu) = board.last_imu() {
            let mut u = [0.0_f32; 3];
            for axis in 0..3 {
                let mut sum = 0.0;
                for (lane, sign) in SIGNS[axis].iter().enumerate() {
                    sum += sign * out.outputs[lane].0;
                }
                u[axis] = sum / 4.0;
            }
            if let Phase::Excite { axis, .. } = &phase {
                // The injection bypasses the kernel's output, so the
                // recorded input must add it back.
                u[*axis] += injection.1;
            }
            samples.push(Sample {
                at: now,
                u,
                gyro: imu.gyro,
            });
        }

        phase = match phase {
            Phase::WaitReady => {
                if board.is_ready() {
                    match board.arm() {
                        Ok(()) => {
                            log::info!("armed; climbing");
                            Phase::Climb {
                                started: now,
                                until: now + CLIMB,
                            }
                        }
                        Err(error) => {
                            log::warn!("arm refused: {error:?}");
                            Phase::WaitReady
                        }
                    }
                } else {
                    Phase::WaitReady
                }
            }
            Phase::Climb { until, .. } if now >= until => {
                log::info!("exciting roll");
                stand.engage(120.0);
                Phase::Settle {
                    axis: 0,
                    freq: 0,
                    until: now + SETTLE,
                }
            }
            Phase::Settle { axis, freq, until } if now >= until => {
                windows[axis][freq].0 = samples.len();
                held_attitude = Some(estimate.attitude);
                // A window starts from rotational rest, so residual
                // spin from the previous one cannot leak into it.
                stand.zero_rates();
                Phase::Excite {
                    axis,
                    freq,
                    until: now + Duration::from_secs_f32(EXCITE_S[freq]),
                }
            }
            Phase::Excite { axis, freq, until } if now >= until => {
                windows[axis][freq].1 = samples.len();
                held_attitude = None;
                if axis == 2 && freq == 1 {
                    stand.release();
                    Phase::Done
                } else {
                    let (axis, freq) = if freq == 1 { (axis + 1, 0) } else { (axis, 1) };
                    log::info!(
                        "exciting {} at {} rad/s",
                        AXIS_NAMES[axis],
                        PROBE_RAD_S[freq]
                    );
                    Phase::Settle {
                        axis,
                        freq,
                        until: now + SETTLE,
                    }
                }
            }
            Phase::Done => {
                let _ = board.disarm();
                report(&samples, &windows);
                return;
            }
            other => other,
        };

        if !board.wait_for_sample(Duration::from_micros(2_500)) && !board.connected() {
            std::thread::sleep(Duration::from_millis(2));
        }
    }
}

/// The command each phase holds. `None` keeps the previous command.
/// The EXCITATION does not travel through here — it is injected on the
/// actuator lanes — so every phase simply asks the closed loop to hold
/// `attitude` (the current estimate, frozen at each window's start:
/// commanding a fixed world frame instead would have the loop fighting
/// the vehicle's heading with saturated torque, drowning the probe).
fn command_for(
    phase: &Phase,
    now: Instant,
    sequence: u32,
    attitude: Quaternion,
) -> Option<Command> {
    let setpoint = match phase {
        Phase::WaitReady => return None,
        Phase::Climb { started, until: _ } => {
            // Ten seconds to full ramp, held for the rest: the rotor
            // inertia needs the time regardless of the command.
            let ramp = (now.duration_since(*started).as_secs_f32() / 6.0).min(1.0);
            let target = collective() + 0.08;
            let collective = ramp * target;
            Setpoint {
                attitude: Some(attitude),
                collective_thrust: NormalizedThrust(collective),
                ..Setpoint::default()
            }
        }
        Phase::Settle { .. } | Phase::Excite { .. } | Phase::Done => Setpoint {
            attitude: Some(attitude),
            collective_thrust: NormalizedThrust(collective()),
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
