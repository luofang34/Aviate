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

use crate::noise::{NoiseRng, NoiseTier};
use crate::synthesize::{apply_packet_noise, cmd_to_omega, synthesize_packet};

/// Cycle period for the FC loop (1 kHz, matching loop_periods::GAZEBO_US).
const CYCLE_PERIOD_US: u64 = 1_000;

fn main() -> std::io::Result<()> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();

    log::info!("aviate sitl-gazebo-x500 starting");

    let mut board = GazeboSitlBoard::new_with_retry(10, 200)?;
    log::info!("board constructed");

    // Connect to the gz-sim system plugin via shared memory. The plugin
    // initializes the shared region as soon as gz-sim loads the SDF, so
    // a short retry loop is plenty.
    let plugin = GzPluginBridge::connect_with_retry(20, 250)
        .map_err(|e| std::io::Error::other(format!("gz plugin: {e:?}")))?;
    log::info!("connected to AviateGzPlugin");

    let mut last_state: Option<AviateModelState> = None;
    let mut last_t_us: u64 = 0;

    let noise_tier = NoiseTier::from_env();
    let mut noise_rng = NoiseRng::new(0xA17_C0DE);
    if noise_tier != NoiseTier::Off {
        log::info!("sensor noise tier: {:?}", noise_tier);
    }

    let cycle = Duration::from_micros(CYCLE_PERIOD_US);
    let mut next_tick = Instant::now() + cycle;

    loop {
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
            let mut packet = synthesize_packet(&state, last_state.as_ref(), last_t_us);
            apply_packet_noise(&mut packet, noise_tier, &mut noise_rng);
            let state_time_us = state.time_us;
            last_state = Some(state);
            last_t_us = state_time_us;

            board.transport_mut().feed_sensor_packet(&packet);
        }

        // 3. Run one kernel cycle.
        let cmd = board.step();

        // 4. Forward actuator outputs to gz-sim as rotor velocities.
        //
        // Aviate's mixer produces normalized [0, 1] outputs whose
        // semantics the kernel treats as normalized **thrust** (the
        // mixer's additive corrections compose meaningfully in
        // thrust units; in motor-speed units, mid-throttle would
        // produce only `cmd²` of max thrust). The X500 rotor model
        // in PX4-gazebo-models implements quadratic thrust:
        // `thrust = motorConstant · ω²`. So normalized-thrust input
        // maps to motor angular velocity as `ω = MAX · √cmd`.
        // Without the sqrt, "0.65 hover" actually produces only
        // 0.42 of max thrust — well below the X500's 0.57 weight-
        // to-max-thrust ratio, and the vehicle sinks.
        let motor_speeds = [
            cmd_to_omega(cmd.outputs[0].0),
            cmd_to_omega(cmd.outputs[1].0),
            cmd_to_omega(cmd.outputs[2].0),
            cmd_to_omega(cmd.outputs[3].0),
        ];
        if let Err(e) = plugin.set_motor_speeds(&motor_speeds) {
            log::warn!("set_motor_speeds failed: {e:?}");
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
