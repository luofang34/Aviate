//! X500 Gazebo SITL FC binary.
//!
//! Architecture:
//!
//! ```text
//! gz sim ── AviateGzPlugin.dylib ── POSIX shm /aviate_gz_bridge ── this binary
//!                model state (ENU)                    │
//!                                                     │ synthesize NED
//!                                                     ▼
//!                                  SitlIO.feed_sensor_packet(...)
//!                                                     │
//!                                                     ▼
//!                                  GazeboSitlBoard.step() → ActuatorCmd
//!                                                     │
//!                                                     ▼
//!                                  plugin.set_motor_speeds(...)
//! ```
//!
//! The `AviateGzPlugin` writes pose / velocity / angular-velocity into
//! shared memory each `PostUpdate` tick. This binary reads that ground
//! truth, synthesizes IMU + baro + mag + GNSS readings, feeds them into
//! the kernel via the SITL transport, runs one kernel cycle, and writes
//! the resulting motor commands back to the plugin.
//!
//! The sensor synthesis, frame conversion math, and noise model live
//! in `synthesize.rs` and `noise.rs`; main.rs owns only the FC loop.
mod noise;
mod synthesize;

use std::time::{Duration, Instant};

use aviate_backend_gz::{AviateModelState, GzPluginBridge};
use aviate_board_sitl_gazebo::GazeboSitlBoard;
use aviate_core::checks::pre_arm::PreArmFlags;
use aviate_xil_contract::FcState;

use crate::noise::{NoiseRng, NoiseTier};
use crate::synthesize::{apply_packet_noise, cmd_to_omega, synthesize_packet};

/// Cycle period for the FC loop (1 kHz, matching loop_periods::GAZEBO_US).
const CYCLE_PERIOD_US: u64 = 1_000;

fn main() -> std::io::Result<()> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();

    log::info!("aviate sitl-gazebo-x500 starting");

    let mut board = GazeboSitlBoard::new_with_retry(
        aviate_app_sitl_gazebo_x500_kernel::build_x500_kernel,
        10,
        200,
    )?;
    log::info!("board constructed");

    // The one boundary conversion (#140): the resolved configuration
    // declares the plant's actuator curve; every mixer output passes
    // through it exactly once on the way to gz-sim.
    let actuator_curve = board.kernel().cfg().actuator_curve;

    // Connect to the gz-sim system plugin via shared memory. The plugin
    // initializes the shared region as soon as gz-sim loads the SDF, so
    // a short retry loop is plenty.
    // 240 × 250ms = 60 s. Plugin Configure latency on macOS is
    // ~15–20 s on a cold cache (gz-sim doing first-time dlopen +
    // physics init); the previous 5 s window timed out before the
    // shm region was populated.
    let plugin = GzPluginBridge::connect_with_retry(240, 250)
        .map_err(|e| std::io::Error::other(format!("gz plugin: {e:?}")))?;
    log::info!("connected to AviateGzPlugin");

    // Runtime control plane (#265): stamp this FC process into the
    // block (consumers detect an FC restart by the nonce changing
    // while the shm object identity stays the same), then walk the
    // Converging -> Ready state machine against the current world
    // generation.
    let session_nonce = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_micros() as u32)
        .unwrap_or(1)
        | 1; // never zero
    plugin.set_fc_session_nonce(session_nonce);
    let mut fc_generation = plugin.reset_generation();
    let mut fc_ready = false;
    plugin.set_fc_status(FcState::Converging, fc_generation);
    let mut last_step_acked: u64 = 0;

    let mut last_state: Option<AviateModelState> = None;
    let mut last_t_us: u64 = 0;
    // Position-derived NED velocity from the previous synth call.
    // gz's `WorldLinearVelocity` is broken on macOS (returns
    // zero), so the synth pipeline derives velocity from
    // `Δposition / Δt` and acceleration from
    // `Δ(derived velocity) / Δt`. The previous derived value
    // lives here so the next call has a consistent source for
    // the second finite difference; reading `prev.vel` (zero)
    // would inject a delta spike on every cycle.
    let mut last_ned_vel: [f32; 3] = [0.0; 3];

    let noise_tier = NoiseTier::from_env();
    let mut noise_rng = NoiseRng::new(0xA17_C0DE);
    if noise_tier != NoiseTier::Off {
        log::info!("sensor noise tier: {:?}", noise_tier);
    }

    let cycle = Duration::from_micros(CYCLE_PERIOD_US);
    let mut next_tick = Instant::now() + cycle;

    loop {
        // 0. Lifecycle (#265): a generation bump means the world was
        //    reset. Rebuild the kernel IN-PROCESS (no pkill, no
        //    restart race), drop the finite-difference synth history
        //    (its deltas would spike across the discontinuity), and
        //    re-converge before reporting Ready.
        let generation = plugin.reset_generation();
        if generation != fc_generation {
            log::info!("world reset detected (generation {fc_generation} -> {generation}); rebuilding kernel in-process");
            plugin.set_fc_status(FcState::Resetting, generation);
            board = GazeboSitlBoard::new_with_retry(
                aviate_app_sitl_gazebo_x500_kernel::build_x500_kernel,
                10,
                200,
            )?;
            last_state = None;
            last_t_us = 0;
            last_ned_vel = [0.0; 3];
            fc_generation = generation;
            fc_ready = false;
            plugin.set_fc_status(FcState::Converging, fc_generation);
        }

        // 1. Read the latest ground-truth model state from the plugin.
        if let Some(state) = plugin.get_model_state() {
            // 2. Synthesize a sensor packet from ground truth. The
            //    `noise_tier` controls additive Gaussian noise on each
            //    channel — `Off` reproduces the perfect-IMU baseline
            //    used by the existing SITL missions; `Mems` /
            //    `Tactical` add representative noise so the kernel's
            //    estimator + controller are exercised against
            //    realistic inputs.
            //
            // Capture `time_us` BEFORE `last_state = Some(state)` so
            // the sequence does not silently break if a future
            // `AviateModelState` field becomes non-Copy.
            let (mut packet, ned_vel) =
                synthesize_packet(&state, last_state.as_ref(), last_t_us, last_ned_vel);
            apply_packet_noise(&mut packet, noise_tier, &mut noise_rng);
            let state_time_us = state.time_us;
            last_state = Some(state);
            last_t_us = state_time_us;
            last_ned_vel = ned_vel;

            board.transport_mut().feed_sensor_packet(&packet);
        }

        // 3. Run one kernel cycle.
        let cmd = board.step();

        // 4. Forward actuator outputs to gz-sim as rotor velocities.
        //
        // Mixer outputs are normalized per-motor THRUST (force
        // domain, #140). The resolved actuator curve — quadratic
        // for the gz rotor, `thrust = motorConstant · ω²` — is
        // applied here, exactly once, mapping force to rotor speed
        // as `ω = MAX · √thrust`.
        let motor_speeds = [
            cmd_to_omega(actuator_curve, cmd.outputs[0].0),
            cmd_to_omega(actuator_curve, cmd.outputs[1].0),
            cmd_to_omega(actuator_curve, cmd.outputs[2].0),
            cmd_to_omega(actuator_curve, cmd.outputs[3].0),
        ];
        if let Err(e) = plugin.set_motor_speeds(&motor_speeds) {
            log::warn!("set_motor_speeds failed: {e:?}");
        }

        // 5. Publish lifecycle state: Ready once the estimator has
        //    converged for the CURRENT generation (#265's "ready"
        //    half — consumers gate on state == Ready AND generation
        //    == reset_generation).
        let ekf_converged = board
            .kernel()
            .state
            .checks
            .pre_arm
            .current
            .contains(PreArmFlags::EKF_CONVERGED);
        if ekf_converged != fc_ready {
            fc_ready = ekf_converged;
            let state = if fc_ready {
                FcState::Ready
            } else {
                FcState::Converging
            };
            log::info!("fc lifecycle -> {state:?} (generation {fc_generation})");
            plugin.set_fc_status(state, fc_generation);
        }

        // 6. Acknowledge the step we consumed: the FC liveness
        //    heartbeat, and — when lockstep is enabled at runtime —
        //    the gate the plugin blocks its next physics step on.
        let consumed_step = last_state.as_ref().map(|s| s.sim_step).unwrap_or(0);
        if consumed_step > last_step_acked {
            plugin.ack_step(consumed_step);
            last_step_acked = consumed_step;
        }

        // 7. Pace the loop. Wall-clock pacing applies only in
        //    real-time (non-lockstep) mode; under runtime lockstep
        //    (#265) the sim blocks on our ack, so sleeping to a wall
        //    tick would throttle 4x / as-fast-as-possible runs back
        //    to 1x. There the pacing is the arrival of new steps.
        if plugin.lockstep_enabled() {
            next_tick = Instant::now() + cycle;
        } else {
            let now = Instant::now();
            if now < next_tick {
                std::thread::sleep(next_tick - now);
            }
            next_tick += cycle;
        }
    }
}
