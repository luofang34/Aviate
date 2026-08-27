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

use aviate_core::ekf::Estimator as _;
use aviate_core::math::Quaternion;

use aviate_board_sitl_xplane::XPlaneBoard;

mod excitation;
mod report;
mod stand;
mod sweep;
mod trace;
mod yaw_sign;

pub(super) use sweep::run_sweep;
pub(super) use yaw_sign::run_yaw_sign;

use excitation::{
    add_duration, command_for, injection_for, leak_toward, Phase, AXIS_NAMES, CLIMB, EXCITE_S,
    HOLD_LEAK, PROBE_RAD_S, SETTLE, SIGNS, TRANSIENT_SKIP,
};
use report::report;
use stand::{Sample, TestStand};

/// Failure of one identification experiment.
#[derive(Debug)]
pub(super) enum ExperimentError {
    /// The experiment made no safe progress before its wall-clock guard expired.
    Timeout(&'static str),
    /// The kernel refused to arm.
    Arm(aviate_core::ArmError),
    /// The kernel refused to disarm.
    Disarm(aviate_core::DisarmError),
    /// The X-Plane test stand did not confirm an operation.
    Stand(stand::StandError),
    /// The process could not open the X-Plane test-stand socket.
    StandSocket(std::io::Error),
    /// Simulator sample time did not increase.
    ClockRegression { previous_us: u64, next_us: u64 },
    /// Runtime identity or sample-clock evidence failed.
    RuntimeHandshake(aviate_board_sitl_xplane::RuntimeHandshakeError),
    /// The external tuning trace did not accept a packet.
    TuningTrace(String),
    /// Plant fitting rejected the trace.
    Report(String),
}

impl core::fmt::Display for ExperimentError {
    fn fmt(&self, formatter: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Timeout(name) => write!(formatter, "{name} timed out"),
            Self::Arm(error) => write!(formatter, "arm failed: {error:?}"),
            Self::Disarm(error) => write!(formatter, "disarm failed: {error:?}"),
            Self::Stand(error) => write!(formatter, "test stand failed: {error}"),
            Self::StandSocket(error) => write!(formatter, "test stand socket failed: {error}"),
            Self::ClockRegression {
                previous_us,
                next_us,
            } => write!(
                formatter,
                "simulator clock did not increase: {previous_us} then {next_us}"
            ),
            Self::RuntimeHandshake(error) => write!(formatter, "runtime handshake failed: {error}"),
            Self::TuningTrace(error) => write!(formatter, "tuning trace failed: {error}"),
            Self::Report(error) => write!(formatter, "plant report failed: {error}"),
        }
    }
}

impl std::error::Error for ExperimentError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Stand(error) => Some(error),
            Self::StandSocket(error) => Some(error),
            Self::RuntimeHandshake(error) => Some(error),
            Self::Timeout(_)
            | Self::Arm(_)
            | Self::Disarm(_)
            | Self::ClockRegression { .. }
            | Self::TuningTrace(_)
            | Self::Report(_) => None,
        }
    }
}

/// Runs the identification flight and prints the plant identity.
/// Returns when the experiment is over; the caller exits.
pub(super) fn run<C, M>(
    board: &mut XPlaneBoard<C, M>,
    run_manifest_digest: String,
) -> Result<
    (
        aviate_config::airframe_preset::PlantIdentificationArtifact,
        String,
    ),
    ExperimentError,
>
where
    C: aviate_core::control::VehicleController,
    M: aviate_core::mixer::Mixer,
{
    let hover_collective = board.kernel().cfg().hover_thrust_norm.0;
    let mut phase = Phase::WaitReady;
    let mut sequence: u32 = 0;
    let mut samples: Vec<Sample> = Vec::with_capacity(8192);
    let mut windows: [[(usize, usize); 2]; 3] = [[(0, 0); 2]; 3];
    let mut window_open: Option<(usize, usize, u64)> = None;
    let started = Instant::now();
    let mut last_heartbeat = Instant::now();
    let mut last_report = Instant::now();
    let mut last_motor_mean = 0.0_f32;
    let mut ground_y: Option<f32> = None;
    let mut held_attitude: Option<Quaternion> = None;
    let mut last_sample_us: Option<u64> = None;
    let mut stand = TestStand::new(match UdpSocket::bind("0.0.0.0:0") {
        Ok(sock) => {
            sock.set_read_timeout(Some(Duration::from_millis(100))).ok();
            sock
        }
        Err(error) => return Err(ExperimentError::StandSocket(error)),
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
        if started.elapsed() > Duration::from_secs(300) {
            return Err(ExperimentError::Timeout("plant identification"));
        }

        // The attitude the loop is asked to hold: live outside the
        // excitation windows, LEAKY inside them. A hard freeze makes the
        // loop fight every estimator wobble at full gain and rail the
        // lanes; a live hold leaves no attitude restoring at all, and a
        // residual rate integrates to a rollover before the window ends.
        // A hold that leaks toward the estimate over seconds arrests the
        // rollover while staying transparent at the probe frequency, so
        // the loop hosts the probe instead of fighting it.
        let estimate = board
            .kernel()
            .pipeline()
            .estimator
            .estimate(&board.kernel().state.estimator);
        let hold = match (&phase, held_attitude) {
            (Phase::Excite { .. }, Some(frozen)) => {
                let leaked = leak_toward(frozen, estimate.attitude, HOLD_LEAK);
                held_attitude = Some(leaked);
                leaked
            }
            _ => estimate.attitude,
        };

        // The probe: a sine of differential force on the excited
        // axis's lanes at the analysis frequency, so the correlation
        // fit reads the plant exactly where the loop design needs it.
        board.set_lane_injection(injection_for(&phase, board.now_us()));
        if matches!(phase, Phase::Lower { .. }) {
            stand.pin().map_err(ExperimentError::Stand)?;
        }

        let command = command_for(&phase, board.now_us(), sequence, hold, hover_collective);
        sequence = sequence.wrapping_add(1);
        if let Some(cmd) = command {
            board.set_command(cmd);
        }
        let out = board.step();
        check_run_gates(board)?;
        last_motor_mean = out.outputs[..4].iter().map(|m| m.0).sum::<f32>() / 4.0;

        let observations = board.take_control_observations();
        let latest_us = record_observations(
            observations,
            &mut samples,
            &mut last_sample_us,
            board.now_us(),
        )?;

        if let Some((axis, freq, from_us)) = window_open {
            if latest_us >= from_us {
                windows[axis][freq].0 = samples.len();
                window_open = None;
            }
        }

        phase = match phase {
            Phase::WaitReady => {
                if board.is_ready() {
                    ground_y = Some(stand.local_y().map_err(ExperimentError::Stand)?);
                    board.arm().map_err(ExperimentError::Arm)?;
                    log::info!("armed; climbing");
                    Phase::Climb {
                        started_us: latest_us,
                        until_us: add_duration(latest_us, CLIMB),
                    }
                } else {
                    Phase::WaitReady
                }
            }
            Phase::Climb { until_us, .. } if latest_us >= until_us => {
                // The windows run in FREE FLIGHT. A translation pin
                // couples back into rotation and rocks the vehicle
                // harder than the hover it replaces — the rocking is
                // both a rate residual no zeroing survives and the
                // noise floor under the probe. The vehicle holds a
                // clean hover on its own; the probe rides on that.
                log::info!("exciting roll");
                Phase::Settle {
                    axis: 0,
                    freq: 0,
                    until_us: add_duration(latest_us, SETTLE),
                }
            }
            Phase::Settle {
                axis,
                freq,
                until_us,
            } if latest_us >= until_us => {
                held_attitude = Some(estimate.attitude);
                // The rate zeroing lands as a gyro step, and the damping
                // term answers it with a lane spike; the first stretch
                // of every window is that transient, not the probe. The
                // fit starts after it.
                window_open = Some((axis, freq, add_duration(latest_us, TRANSIENT_SKIP)));
                Phase::Excite {
                    axis,
                    freq,
                    started_us: latest_us,
                    until_us: add_duration(latest_us, Duration::from_secs_f32(EXCITE_S[freq])),
                }
            }
            Phase::Excite {
                axis,
                freq,
                until_us,
                ..
            } if latest_us >= until_us => {
                windows[axis][freq].1 = samples.len();
                held_attitude = None;
                if axis == 2 && freq == 1 {
                    let ground = ground_y.ok_or(ExperimentError::Timeout("ground altitude"))?;
                    let _ = stand.engage().map_err(ExperimentError::Stand)?;
                    log::info!("lowering to the ground");
                    Phase::Lower {
                        ground_y: ground,
                        last_us: latest_us,
                    }
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
                        until_us: add_duration(latest_us, SETTLE),
                    }
                }
            }
            Phase::Lower { ground_y, last_us } => {
                // Sim-time paced so the ride is 4 m/s regardless of how
                // often this loop spins between samples.
                let dt_s = latest_us.saturating_sub(last_us) as f32 / 1_000_000.0;
                stand.lower(4.0 * dt_s, ground_y);
                if stand.held().is_some_and(|y| y <= ground_y + 0.05) {
                    stand.release();
                    log::info!("on the ground; settling");
                    Phase::SettleGround {
                        until_us: add_duration(latest_us, Duration::from_secs(6)),
                    }
                } else {
                    Phase::Lower {
                        ground_y,
                        last_us: latest_us,
                    }
                }
            }
            Phase::SettleGround { until_us } if latest_us >= until_us => Phase::Done,
            other => other,
        };

        if matches!(phase, Phase::Done) {
            board.set_lane_injection([0.0; 4]);
            // The artifact is the experiment's product and every sample
            // is already in hand; publish before anything that can still
            // refuse. Disarm is cleanliness, and the landed latch reads
            // an estimate the pinned excitation can leave far from the
            // ground truth, so a refusal here downgrades to a warning —
            // the vehicle is at rest on its gear with the lanes idled,
            // and the harness owns the process from here.
            let context = report::ReportContext {
                simulator_model_digest: board.model_digest().to_string(),
                run_manifest_digest,
                hover_force: hover_collective,
            };
            let published = report(&samples, &windows, context).map_err(ExperimentError::Report)?;
            let disarm_deadline = Instant::now() + Duration::from_secs(10);
            loop {
                match board.disarm() {
                    Ok(()) => break,
                    Err(error) if Instant::now() < disarm_deadline => {
                        for _ in 0..40 {
                            let cmd = command_for(
                                &phase,
                                board.now_us(),
                                sequence,
                                estimate.attitude,
                                hover_collective,
                            );
                            sequence = sequence.wrapping_add(1);
                            if let Some(cmd) = cmd {
                                board.set_command(cmd);
                            }
                            board.step();
                            board.wait_for_sample(Duration::from_micros(2_500));
                        }
                        let _ = error;
                    }
                    Err(error) => {
                        log::warn!("disarm refused after landing ({error:?}); leaving shutdown to the process owner");
                        break;
                    }
                }
            }
            return Ok(published);
        }

        if !board.wait_for_sample(Duration::from_micros(2_500)) && !board.connected() {
            std::thread::sleep(Duration::from_millis(2));
        }
    }
}

pub(super) fn check_run_gates<C, M>(board: &XPlaneBoard<C, M>) -> Result<(), ExperimentError>
where
    C: aviate_core::control::VehicleController,
    M: aviate_core::mixer::Mixer,
{
    if let Some(error) = board.runtime_handshake_failure() {
        return Err(ExperimentError::RuntimeHandshake(error.clone()));
    }
    if let Some(error) = board.tuning_trace_failure() {
        return Err(ExperimentError::TuningTrace(error.to_string()));
    }
    Ok(())
}

fn record_observations(
    observations: Vec<aviate_board_sitl_xplane::XPlaneControlObservation>,
    samples: &mut Vec<Sample>,
    last_sample_us: &mut Option<u64>,
    fallback_us: u64,
) -> Result<u64, ExperimentError> {
    // Phase arithmetic lives on the SAMPLE clock. The board clock is
    // wall time, the samples are simulation time, and the simulator
    // dilates when its flight model cannot keep real time — a deadline
    // built from one and compared against the other shortens every
    // window by the dilation factor.
    let mut latest_us = last_sample_us.unwrap_or(fallback_us);
    for observation in observations {
        if let Some(previous_us) = *last_sample_us {
            if observation.timestamp_us <= previous_us {
                return Err(ExperimentError::ClockRegression {
                    previous_us,
                    next_us: observation.timestamp_us,
                });
            }
        }
        *last_sample_us = Some(observation.timestamp_us);
        latest_us = observation.timestamp_us;
        let Some(imu) = observation.imu else {
            continue;
        };
        let mut input = [0.0; 3];
        for axis in 0..3 {
            input[axis] = SIGNS[axis]
                .iter()
                .zip(observation.applied_force_lanes)
                .map(|(sign, lane)| sign * lane)
                .sum::<f32>()
                / 4.0;
        }
        samples.push(Sample {
            timestamp_us: observation.timestamp_us,
            u: input,
            gyro: imu.gyro,
            collective_force: observation.applied_force_lanes.iter().sum::<f32>() / 4.0,
            // The probe is a pure differential, so only constraints
            // that touch individual lanes corrupt what the fit reads.
            // The mean-path constraints trim the collective every
            // sample by construction of a rate limiter, and counting
            // those as saturation refuses windows whose torque probe
            // went through untouched.
            saturated: observation.constraint_flags.lane_ceiling
                || observation.constraint_flags.injection_clamp
                || observation.constraint_flags.invalid_actuator_count
                || observation.constraint_flags.missing_actuator_answer
                || observation.constraint_flags.ground_squeeze,
        });
    }
    Ok(latest_us)
}
