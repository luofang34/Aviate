//! Grounded collective sweep.

use std::time::{Duration, Instant};

use aviate_board_sitl_xplane::XPlaneBoard;
use aviate_core::control::{Command, CommandSource, ControlMode, Setpoint};
use aviate_core::ekf::Estimator as _;
use aviate_core::types::NormalizedThrust;

use super::ExperimentError;

/// Run one grounded collective staircase.
pub(crate) fn run_sweep<C, M>(board: &mut XPlaneBoard<C, M>) -> Result<(), ExperimentError>
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
    let mut step_reported = false;
    log::info!("collective sweep: waiting for the verified link");
    loop {
        let now = Instant::now();
        if last_heartbeat.elapsed() >= Duration::from_secs(1) {
            board.send_heartbeat();
            last_heartbeat = now;
        }
        if started.elapsed() > Duration::from_secs(120) {
            return Err(ExperimentError::Timeout("collective sweep"));
        }
        if !armed && board.is_ready() {
            board.arm().map_err(ExperimentError::Arm)?;
            armed = true;
            step_started = Some(now);
            log::info!("armed; sweeping");
        } else if let Some(at) = step_started {
            if now.duration_since(at) >= HOLD {
                step = step.wrapping_add(1);
                step_started = Some(now);
                step_reported = false;
                if step >= STEPS {
                    board.disarm().map_err(ExperimentError::Disarm)?;
                    return Ok(());
                }
            }
        }
        if armed {
            send_sweep_command(board, step, &mut sequence);
        }
        let output = board.step();
        super::check_run_gates(board)?;
        if armed
            && !step_reported
            && step_started
                .is_some_and(|at| now.duration_since(at) > HOLD - Duration::from_millis(300))
        {
            report_step(board, &output, step, STEPS);
            step_reported = true;
        }
        if !board.wait_for_sample(Duration::from_micros(2_500)) && !board.connected() {
            std::thread::sleep(Duration::from_millis(2));
        }
    }
}

fn send_sweep_command<C, M>(board: &mut XPlaneBoard<C, M>, step: usize, sequence: &mut u32)
where
    C: aviate_core::control::VehicleController,
    M: aviate_core::mixer::Mixer,
{
    const STEPS: usize = 12;
    let hold = board
        .kernel()
        .pipeline()
        .estimator
        .estimate(&board.kernel().state.estimator)
        .attitude;
    let force = step as f32 / (STEPS - 1) as f32;
    board.set_command(Command {
        mode: ControlMode::Attitude,
        setpoint: Setpoint {
            attitude: Some(hold),
            collective_thrust: NormalizedThrust(force),
            ..Setpoint::default()
        },
        config_mode_request: None,
        sensor_overrides: None,
        sequence: *sequence,
        source: CommandSource::Autopilot,
    });
    *sequence = sequence.wrapping_add(1);
}

fn report_step<C, M>(
    board: &XPlaneBoard<C, M>,
    output: &aviate_core::mixer::ActuatorCmd,
    step: usize,
    steps: usize,
) where
    C: aviate_core::control::VehicleController,
    M: aviate_core::mixer::Mixer,
{
    let altitude = board.last_fix().map_or(0.0, |fix| fix.alt_m);
    let estimate = board
        .kernel()
        .pipeline()
        .estimator
        .estimate(&board.kernel().state.estimator);
    let (roll, pitch, _) = estimate.attitude.to_euler();
    let gyro = board.last_imu().map_or([0.0; 3], |imu| imu.gyro);
    log::info!(
        "sweep step={step} force={:.2} motors=[{:.2},{:.2},{:.2},{:.2}] est_rp=({roll:.2},{pitch:.2}) gyro=[{:.2},{:.2},{:.2}] alt={altitude:.1}",
        step as f32 / (steps - 1) as f32,
        output.outputs[0].0,
        output.outputs[1].0,
        output.outputs[2].0,
        output.outputs[3].0,
        gyro[0],
        gyro[1],
        gyro[2],
    );
}
