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
use aviate_xil_contract::WriterState;

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

    // The one boundary conversion: the resolved configuration
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
    let mut plugin = GzPluginBridge::connect_with_retry(240, 250)
        .map_err(|e| std::io::Error::other(format!("gz plugin: {e:?}")))?;
    log::info!("connected to AviateGzPlugin");

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

    // Re-attach checks are a slow path: a few syscalls plus a
    // transient mapping. At 1 kHz that is pure overhead, so poll the
    // writer's identity about once a second while healthy — a
    // restart takes far longer than that to matter. While UNHEALTHY
    // the poll runs every cycle: no useful I/O is happening anyway,
    // and recovery should not wait out the slow cadence.
    let writer_check_every = 1_000;
    let mut cycles_since_writer_check: u32 = 0;
    // Only `Current` earns shm I/O. In every other state both
    // directions stop: reads would serve a dead or mid-init world's
    // snapshot as fresh ground truth, and motor writes would land in
    // memory the live simulator never reads (or worse, in a
    // half-initialized successor block).
    let mut sim_io_healthy = true;
    // Highest sim_step this FC has consumed, so each step is
    // heartbeat-acknowledged at most once.
    let mut last_consumed_step: Option<u64> = None;

    loop {
        // 0. Has the simulator we are bound to been replaced?
        //    Without this the FC outlives its plugin: the old
        //    mapping keeps serving the dead world's final snapshot
        //    and every motor command lands in memory no one reads,
        //    all while looking perfectly healthy.
        cycles_since_writer_check = cycles_since_writer_check.wrapping_add(1);
        if !sim_io_healthy || cycles_since_writer_check >= writer_check_every {
            cycles_since_writer_check = 0;
            match plugin.writer_state() {
                WriterState::Current => {
                    sim_io_healthy = true;
                }
                state @ (WriterState::Replaced | WriterState::Gone) => {
                    log::warn!("simulator {state:?}; re-attaching to the live block");
                    match GzPluginBridge::connect_with_retry(240, 250) {
                        Ok(fresh) => {
                            plugin = fresh;
                            last_state = None;
                            last_t_us = 0;
                            last_ned_vel = [0.0; 3];
                            last_consumed_step = None;
                            sim_io_healthy = true;
                            log::info!("re-attached to AviateGzPlugin");
                        }
                        Err(e) => {
                            return Err(std::io::Error::other(format!(
                                "simulator {state:?} and re-attach failed: {e:?}"
                            )));
                        }
                    }
                }
                WriterState::Initializing => {
                    sim_io_healthy = false;
                    log::debug!("simulator initializing; pausing shm I/O until it is ready");
                }
                WriterState::ContractMismatch => {
                    return Err(std::io::Error::other(
                        "the live shm block no longer matches this build's contract",
                    ));
                }
            }
        }

        // 1. Read the latest ground-truth model state from the
        //    plugin — but only from a healthy attachment.
        let mut consumed_step: Option<u64> = None;
        if sim_io_healthy {
            if let Some(state) = plugin.get_model_state() {
                // 2. Synthesize a sensor packet from ground truth.
                //    The `noise_tier` controls additive Gaussian
                //    noise on each channel — `Off` reproduces the
                //    perfect-IMU baseline used by the existing SITL
                //    missions; `Mems` / `Tactical` add
                //    representative noise so the kernel's estimator
                //    + controller are exercised against realistic
                //    inputs.
                //
                // Capture `time_us` BEFORE `last_state =
                // Some(state)` so the sequence does not silently
                // break if a future `AviateModelState` field
                // becomes non-Copy.
                let (mut packet, ned_vel) =
                    synthesize_packet(&state, last_state.as_ref(), last_t_us, last_ned_vel);
                apply_packet_noise(&mut packet, noise_tier, &mut noise_rng);
                let state_time_us = state.time_us;
                if last_consumed_step != Some(state.sim_step) {
                    consumed_step = Some(state.sim_step);
                }
                last_state = Some(state);
                last_t_us = state_time_us;
                last_ned_vel = ned_vel;

                board.transport_mut().feed_sensor_packet(&packet);
            }
        }

        // 3. Run one kernel cycle. The kernel keeps its own clock
        //    even while sim I/O is paused; its outputs are simply
        //    not forwarded anywhere.
        let cmd = board.step();

        // 4. Forward actuator outputs to gz-sim as rotor velocities,
        //    then heartbeat the consumed step AFTER the motor write,
        //    so the step's commands are in shared memory before its
        //    consumption is announced.
        //
        // Mixer outputs are normalized per-motor THRUST (force
        // domain). The resolved actuator curve — quadratic for the
        // gz rotor, `thrust = motorConstant · ω²` — is applied
        // here, exactly once, mapping force to rotor speed as
        // `ω = MAX · √thrust`.
        if sim_io_healthy {
            let motor_speeds = [
                cmd_to_omega(actuator_curve, cmd.outputs[0].0),
                cmd_to_omega(actuator_curve, cmd.outputs[1].0),
                cmd_to_omega(actuator_curve, cmd.outputs[2].0),
                cmd_to_omega(actuator_curve, cmd.outputs[3].0),
            ];
            if let Err(e) = plugin.set_motor_speeds(&motor_speeds) {
                log::warn!("set_motor_speeds failed: {e:?}");
            }
            if let Some(step) = consumed_step {
                // One gate, one acker. Under lockstep, fc_step_ack
                // is the word the simulator BLOCKS on, and it is
                // owned by whichever session driver armed lockstep
                // (the mission harness gates each physics step on
                // its own read-act-ack cycle). A second acker races
                // the owner: the gate opens whenever EITHER party
                // runs, which on a starved host is exactly when the
                // other — the actual controller — has fallen
                // behind. In free-run the word gates nothing, so
                // the FC heartbeats it as its per-step liveness
                // signal.
                if !plugin.lockstep_enabled() {
                    plugin.ack_step(step);
                }
                last_consumed_step = Some(step);
            }
        }

        // 5. Pace the loop. We do not lock to gz sim_step here — the
        //    plugin's `lockstep` setting (off by default in our smoke
        //    world) decides whether gz advances independently.
        let now = Instant::now();
        if now < next_tick {
            std::thread::sleep(next_tick - now);
        }
        next_tick += cycle;
    }
}
