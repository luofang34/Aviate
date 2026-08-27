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
//! SITL: fly a short hop, hold attitude while a differential-force sine
//! rides one axis's actuator lanes at a time, and fit the open-loop
//! relation ẏ = K·u from what was MEASURED — u is the mixer's own force-domain axis torque (linear in
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

use aviate_board_sitl_xplane::XPlaneBoard;

mod excitation;
mod refusal;
mod report;
mod stand;
mod sweep;
mod trace;
mod yaw_sign;

pub(super) use sweep::run_sweep;
pub(super) use yaw_sign::run_yaw_sign;

use excitation::{
    add_duration, command_for, injection_for, Phase, AXIS_NAMES, CLIMB, EXCITE_S, FIRST_SETTLE,
    PROBE_RAD_S, SETTLE, SIGNS, TRANSIENT_SKIP,
};
pub(crate) use refusal::ExperimentError;
use report::report;
use stand::{Sample, TestStand};

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
    let mut ground_alt_m: Option<f32> = None;
    let mut collective_trim: f32 = 0.0;
    let mut last_new_samples: usize = 0;
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
            let est = board
                .kernel()
                .pipeline()
                .estimator
                .estimate(&board.kernel().state.estimator);
            let (roll_deg, pitch_deg) = roll_pitch_deg(est.attitude);
            log::info!(
                "alt={alt}m vz={vz:.2} roll={roll_deg:.1} pitch={pitch_deg:.1} trim={collective_trim:.3} motors_mean={last_motor_mean:.2}"
            );
            last_report = Instant::now();
        }
        // Sim-paced phases stretch by the simulator's dilation on a
        // loaded machine; the wall budget bounds a hung run, not a slow
        // one.
        if started.elapsed() > Duration::from_secs(700) {
            return Err(ExperimentError::Timeout("plant identification"));
        }

        // Everything the fit will read is paced on the SAMPLE clock: the
        // probe sine, the hold leak, and every phase deadline. The board
        // clock is wall time, the simulator dilates under load, and a
        // probe advanced on the wall clock lands at the wrong stamped
        // frequency by exactly the dilation factor — the demodulator
        // then reads a sine that is not there. Wall time appears only in
        // the heartbeat, the progress log, and the global timeout.
        let clock_us = last_sample_us.unwrap_or(0);

        let estimate = board
            .kernel()
            .pipeline()
            .estimator
            .estimate(&board.kernel().state.estimator);

        // The probe: a sine of differential force on the excited
        // axis's lanes at the analysis frequency, so the correlation
        // fit reads the plant exactly where the loop design needs it.
        board.set_lane_injection(injection_for(&phase, clock_us));
        if matches!(phase, Phase::Lower { .. }) {
            stand.pin().map_err(ExperimentError::Stand)?;
        }

        // Height rides a slew-limited trim on the collective, fed by the
        // GPS fix. The gains are soft and the slew is far below rotor
        // spool, so the trim cannot excite the vertical axis the way a
        // stiff loop against spool lag does; the probe is a pure
        // differential, so the mean-path trim is invisible to the fit.
        if let (Some(ground), Some(fix)) = (ground_alt_m, board.last_fix()) {
            // The climb drives the height error four times harder than
            // the windows do: a gentle gain converges to a droop below
            // the airborne gate and spends the whole budget hovering
            // under it, while the windows need only to hold what the
            // climb won.
            let (target_m, error_gain, active) = match phase {
                Phase::Climb { .. } => (ground + 10.0, 0.02, true),
                Phase::Settle { .. } | Phase::Excite { .. } => (ground + 8.0, 0.005, true),
                _ => (0.0, 0.0, false),
            };
            if active {
                let error_m = target_m - fix.alt_m;
                let sink_m_s = fix.vel_ned[2];
                // Damping-dominant on purpose: the height error term is
                // kept an order below the sink term, so the trim acts as
                // a vertical-speed damper that drifts toward the target
                // height, overdamped even against rotor spool lag.
                // The clamp is ASYMMETRIC: falling is what kills the
                // experiment, ballooning merely wastes seconds. A deep
                // lift cut also spools the rotors down, and they answer
                // the recovery an order slower than they shed — so the
                // trim may barely cut lift, and gravity retires an
                // overshoot on its own.
                let desired = (error_gain * error_m + 0.15 * sink_m_s).clamp(-0.02, 0.2);
                let step = (desired - collective_trim).clamp(-0.003, 0.003);
                collective_trim += step * last_new_samples as f32;
            }
        }
        // The attitude reference is LEVEL TILT AT THE CURRENT HEADING.
        // Holding the live estimate blesses every drift as the new
        // setpoint and the vehicle translates instead of hovering; but a
        // reference that also freezes heading leaks any heading swing
        // into roll and pitch through the attitude error, tilting the
        // vehicle off its lift exactly when the yaw probe runs. Level
        // tilt restores; heading is never fought.
        let reference = level_at_current_heading(estimate.attitude);
        let command = command_for(
            &phase,
            sequence,
            reference,
            hover_collective + collective_trim,
        );
        sequence = sequence.wrapping_add(1);
        if let Some(cmd) = command {
            board.set_command(cmd);
        }
        let out = board.step();
        check_run_gates(board)?;
        last_motor_mean = out.outputs[..4].iter().map(|m| m.0).sum::<f32>() / 4.0;

        let observations = board.take_control_observations();
        last_new_samples = observations.len();
        let latest_us = record_observations(observations, &mut samples, &mut last_sample_us)?;

        // A window flown on the ground fits gear friction and reads as a
        // plausible-but-meaningless plant; a crashed vehicle otherwise
        // sits out the rest of the sequence being "identified". Refuse
        // loudly instead.
        if matches!(phase, Phase::Settle { .. } | Phase::Excite { .. }) {
            let fix_alt_m = board.last_fix().map(|fix| fix.alt_m);
            if let (Some(ground), Some(alt_m)) = (ground_alt_m, fix_alt_m) {
                if alt_m < ground + 1.0 {
                    board.set_lane_injection([0.0; 4]);
                    return Err(ExperimentError::GroundContact {
                        window: match phase {
                            Phase::Excite { axis, .. } => AXIS_NAMES[axis],
                            _ => "settle",
                        },
                        alt_m,
                        trace_text: trace::encode(
                            &samples,
                            &windows,
                            &board.model_digest().to_string(),
                            &run_manifest_digest,
                        ),
                    });
                }
            }
        }

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
                    ground_alt_m = board.last_fix().map(|fix| fix.alt_m);
                    board.arm().map_err(ExperimentError::Arm)?;
                    log::info!("armed; climbing");
                    Phase::Climb {
                        until_us: add_duration(latest_us, CLIMB),
                    }
                } else {
                    Phase::WaitReady
                }
            }
            // The climb ends on ACHIEVED height, not on elapsed time:
            // rotor spool from rest eats a machine-dependent share of the
            // budget, and a clock-ended climb sometimes handed the windows
            // a vehicle still standing on its gear. The budget remains as
            // the fail-closed bound.
            Phase::Climb { .. }
                if ground_alt_m
                    .zip(board.last_fix().map(|fix| fix.alt_m))
                    .is_some_and(|(ground, alt)| alt >= ground + 6.0) =>
            {
                // The windows run in FREE FLIGHT. A translation pin
                // couples back into rotation and rocks the vehicle
                // harder than the hover it replaces — the rocking is
                // both a rate residual no zeroing survives and the
                // noise floor under the probe. The vehicle holds a
                // clean hover on its own; the probe rides on that.
                log::info!("airborne; settling before the first window");
                Phase::Settle {
                    axis: 0,
                    freq: 0,
                    until_us: add_duration(latest_us, FIRST_SETTLE),
                }
            }
            Phase::Climb { until_us, .. } if latest_us >= until_us => {
                return Err(ExperimentError::NeverLifted {
                    alt_m: board.last_fix().map_or(f32::NAN, |fix| fix.alt_m),
                });
            }
            Phase::Settle {
                axis,
                freq,
                until_us,
            } if latest_us >= until_us => {
                // The first stretch of every window carries the settle's
                // own braking transient, not the probe; the fit starts
                // after it.
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
            let published =
                report(&samples, &windows, context).map_err(|refusal| ExperimentError::Report {
                    reason: refusal.reason,
                    trace_text: refusal.trace_text,
                })?;
            let disarm_deadline = Instant::now() + Duration::from_secs(10);
            loop {
                match board.disarm() {
                    Ok(()) => break,
                    Err(_) if Instant::now() < disarm_deadline => {
                        for _ in 0..40 {
                            let cmd =
                                command_for(&phase, sequence, estimate.attitude, hover_collective);
                            sequence = sequence.wrapping_add(1);
                            if let Some(cmd) = cmd {
                                board.set_command(cmd);
                            }
                            board.step();
                            board.wait_for_sample(Duration::from_micros(2_500));
                        }
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

/// A pure-yaw attitude at the estimate's heading: zero roll and pitch,
/// so the attitude error restores tilt and never contains heading.
fn level_at_current_heading(q: aviate_core::math::Quaternion) -> aviate_core::math::Quaternion {
    let yaw = libm::atan2f(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    );
    let half = yaw * 0.5;
    aviate_core::math::Quaternion::new(libm::cosf(half), 0.0, 0.0, libm::sinf(half))
}

/// Roll and pitch of an attitude, degrees, for the progress log.
fn roll_pitch_deg(q: aviate_core::math::Quaternion) -> (f32, f32) {
    let sin_pitch = 2.0 * (q.w * q.y - q.z * q.x);
    let pitch = libm::asinf(sin_pitch.clamp(-1.0, 1.0));
    let roll = libm::atan2f(
        2.0 * (q.w * q.x + q.y * q.z),
        1.0 - 2.0 * (q.x * q.x + q.y * q.y),
    );
    (roll.to_degrees(), pitch.to_degrees())
}

fn record_observations(
    observations: Vec<aviate_board_sitl_xplane::XPlaneControlObservation>,
    samples: &mut Vec<Sample>,
    last_sample_us: &mut Option<u64>,
) -> Result<u64, ExperimentError> {
    // Phase arithmetic lives on the SAMPLE clock. The board clock is
    // wall time, the samples are simulation time, and the simulator
    // dilates when its flight model cannot keep real time — a deadline
    // built from one and compared against the other shortens every
    // window by the dilation factor. Before the first sample the clock
    // reads zero; nothing leaves WaitReady until samples flow.
    let mut latest_us = last_sample_us.unwrap_or(0);
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
            constraints: [
                observation.constraint_flags.lane_ceiling,
                observation.constraint_flags.injection_clamp,
                observation.constraint_flags.invalid_actuator_count,
                observation.constraint_flags.missing_actuator_answer,
                observation.constraint_flags.ground_squeeze,
            ],
        });
    }
    Ok(latest_us)
}
