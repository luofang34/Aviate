//! The yaw-sign probe: the smallest experiment that settles which
//! way the airframe's yaw actually answers a yaw command.

use std::net::UdpSocket;
use std::time::{Duration, Instant};

use aviate_board_sitl_xplane::XPlaneBoard;

use aviate_core::control::{Command, CommandSource, ControlMode, Setpoint};
use aviate_core::ekf::Estimator as _;
use aviate_core::types::NormalizedThrust;

use super::stand::TestStand;
use super::{ExperimentError, SIGNS};

/// The yaw-sign probe: on the virtual test stand, inject a CONSTANT
/// yaw differential in each direction and read the initial body-rate
/// response from the simulator's own truth. The closed loop counters
/// within a second, so only the FIRST fraction of each step speaks —
/// but the sign of that response is exactly the fact in dispute: a
/// mixer whose yaw column disagrees with the airframe's actual rotor
/// spin directions turns the yaw loop into positive feedback, which
/// presents as a slow uncommanded heading walk no gain can fix.
pub(crate) fn run_yaw_sign<C, M>(board: &mut XPlaneBoard<C, M>) -> Result<(), ExperimentError>
where
    C: aviate_core::control::VehicleController,
    M: aviate_core::mixer::Mixer,
{
    let hover_collective = board.kernel().cfg().hover_thrust_norm.0;
    let mut stand = TestStand::new(match UdpSocket::bind("0.0.0.0:0") {
        Ok(sock) => {
            sock.set_read_timeout(Some(Duration::from_millis(100))).ok();
            sock
        }
        Err(error) => return Err(ExperimentError::StandSocket(error)),
    });
    let mut sequence: u32 = 0;
    let started = Instant::now();
    let mut armed = false;
    let mut phase_started: Option<Instant> = None;
    let mut phase: usize = 0; // 0 spool, 1 +inject, 2 settle, 3 -inject, 4 done
    let mut sums = [0.0_f32; 2];
    let mut counts = [0_u32; 2];
    let mut last_heartbeat = Instant::now();

    log::info!("yaw-sign probe: waiting for the link");
    loop {
        let now = Instant::now();
        if last_heartbeat.elapsed() >= Duration::from_secs(1) {
            board.send_heartbeat();
            last_heartbeat = now;
        }
        if started.elapsed() > Duration::from_secs(90) {
            return Err(ExperimentError::Timeout("yaw-sign probe"));
        }
        if !armed {
            if board.is_ready() {
                board.arm().map_err(ExperimentError::Arm)?;
                armed = true;
                phase_started = Some(now);
                let _ = stand.engage().map_err(ExperimentError::Stand)?;
                log::info!("armed; spooling on the stand");
            }
        } else if let Some(at) = phase_started {
            let elapsed = now.duration_since(at);
            let advance = match phase {
                0 => elapsed >= Duration::from_secs(8),
                1 | 3 => elapsed >= Duration::from_millis(3500),
                2 => elapsed >= Duration::from_secs(3),
                _ => true,
            };
            if advance {
                phase = phase.wrapping_add(1);
                phase_started = Some(now);
                stand.zero_rates().map_err(ExperimentError::Stand)?;
                if phase >= 4 {
                    board.disarm().map_err(ExperimentError::Disarm)?;
                    let plus = sums[0] / counts[0].max(1) as f32;
                    let minus = sums[1] / counts[1].max(1) as f32;
                    if counts.contains(&0) || plus <= minus {
                        return Err(ExperimentError::Report(
                            "yaw response sign does not match the compiled mixer".to_owned(),
                        ));
                    }
                    log::info!(
                        "yaw-sign result plus={plus:+.3}rad/s minus={minus:+.3}rad/s verdict=correct"
                    );
                    return Ok(());
                }
                log::info!("yaw-sign phase {phase}");
            }
        }

        // Steady collective on the stand; constant yaw differential in
        // the injection phases, measured over their first 500 ms only.
        let inject = match phase {
            1 => 0.3,
            3 => -0.3,
            _ => 0.0,
        };
        let mut lanes = [0.0_f32; 4];
        for (lane, sign) in lanes.iter_mut().zip(SIGNS[2]) {
            *lane = sign * inject;
        }
        board.set_lane_injection(lanes);

        let hold = board
            .kernel()
            .pipeline()
            .estimator
            .estimate(&board.kernel().state.estimator)
            .attitude;
        if armed {
            let cmd = Command {
                mode: ControlMode::Attitude,
                setpoint: Setpoint {
                    attitude: Some(hold),
                    collective_thrust: NormalizedThrust(hover_collective),
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
        board.step();
        super::check_run_gates(board)?;
        if armed {
            stand.pin().map_err(ExperimentError::Stand)?;
        }

        if let (1 | 3, Some(at)) = (phase, phase_started) {
            let age = now.duration_since(at);
            if age > Duration::from_millis(300) && age < Duration::from_millis(3300) {
                if let Some(imu) = board.last_imu() {
                    let slot = if phase == 1 { 0 } else { 1 };
                    sums[slot] += imu.gyro[2];
                    counts[slot] = counts[slot].wrapping_add(1);
                }
            }
        }

        if !board.wait_for_sample(Duration::from_micros(2_500)) && !board.connected() {
            std::thread::sleep(Duration::from_millis(2));
        }
    }
}
