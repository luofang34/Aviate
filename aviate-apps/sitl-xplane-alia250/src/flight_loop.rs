//! Packet-paced X-Plane flight loop and runtime reporting.

use std::process::ExitCode;
use std::time::{Duration, Instant};

use aviate_board_sitl_xplane::XPlaneBoard;
use aviate_core::ekf::Estimator as _;

const IDLE_WAIT: Duration = Duration::from_micros(2_500);

pub(super) fn run<C, M>(
    board: &mut XPlaneBoard<C, M>,
    auto_arm: Option<Duration>,
    truth_tx: Option<std::net::UdpSocket>,
) -> ExitCode
where
    C: aviate_core::control::VehicleController,
    M: aviate_core::mixer::Mixer,
{
    let mut state = FlightLoop::new(auto_arm, truth_tx);
    loop {
        let cycle_start = Instant::now();
        let command = board.step();
        if state.has_terminal_failure(board) {
            board.terminate();
            return ExitCode::FAILURE;
        }
        state.forward_truth(board);
        state.report_connection(board);
        state.send_heartbeat(board);
        state.try_auto_arm(board);
        state.report_status(board, &command);
        pace(board, cycle_start);
    }
}

struct FlightLoop {
    started: Instant,
    last_heartbeat: Instant,
    last_report: Instant,
    last_truth: Instant,
    truth_sequence: u8,
    truth_forwarded: bool,
    was_connected: bool,
    armed: bool,
    auto_arm: Option<Duration>,
    truth_tx: Option<std::net::UdpSocket>,
}

impl FlightLoop {
    fn new(auto_arm: Option<Duration>, truth_tx: Option<std::net::UdpSocket>) -> Self {
        let now = Instant::now();
        Self {
            started: now,
            last_heartbeat: now,
            last_report: now,
            last_truth: now,
            truth_sequence: 0,
            truth_forwarded: false,
            was_connected: false,
            armed: false,
            auto_arm,
            truth_tx,
        }
    }

    fn has_terminal_failure<C, M>(&self, board: &XPlaneBoard<C, M>) -> bool
    where
        C: aviate_core::control::VehicleController,
        M: aviate_core::mixer::Mixer,
    {
        if let Some(error) = board.runtime_handshake_failure() {
            log::error!("runtime handshake failed during run: {error}");
            return true;
        }
        if let Some(error) = board.tuning_trace_failure() {
            log::error!("tuning trace failed during run: {error}");
            return true;
        }
        if board.perturbation_artifact_failed() {
            log::error!("condition artifact changed during the run");
            return true;
        }
        if let Some(error) = board.perturbation_failure() {
            log::error!("condition perturbation failed during run: {error}");
            return true;
        }
        false
    }

    fn forward_truth<C, M>(&mut self, board: &mut XPlaneBoard<C, M>)
    where
        C: aviate_core::control::VehicleController,
        M: aviate_core::mixer::Mixer,
    {
        let Some(truth) = board.take_truth() else {
            return;
        };
        if self.last_truth.elapsed() < Duration::from_millis(100) {
            return;
        }
        if self.truth_tx.is_some() {
            let mut buffer = [0_u8; 256];
            let message = aviate_backend_mavlink_hil::messages::HilMessage::StateQuaternion(truth);
            if let Some(length) = aviate_backend_mavlink_hil::serialize_frame(
                &message,
                self.truth_sequence,
                1,
                1,
                &mut buffer,
            ) {
                self.note_first_truth();
                self.truth_sequence = self.truth_sequence.wrapping_add(1);
                if let Some(socket) = self.truth_tx.as_ref() {
                    if let Err(error) = socket.send(&buffer[..length]) {
                        log::debug!("sim-truth send failed: {error}");
                    }
                }
            }
        }
        self.last_truth = Instant::now();
    }

    fn note_first_truth(&mut self) {
        if !self.truth_forwarded {
            self.truth_forwarded = true;
            log::info!("first sim-truth frame forwarded");
        }
    }

    fn report_connection<C, M>(&mut self, board: &XPlaneBoard<C, M>)
    where
        C: aviate_core::control::VehicleController,
        M: aviate_core::mixer::Mixer,
    {
        let connected = board.connected();
        if connected != self.was_connected {
            log::info!(
                "{}",
                if connected {
                    "HIL link up"
                } else {
                    "HIL link down; retrying"
                }
            );
            self.was_connected = connected;
        }
    }

    fn send_heartbeat<C, M>(&mut self, board: &mut XPlaneBoard<C, M>)
    where
        C: aviate_core::control::VehicleController,
        M: aviate_core::mixer::Mixer,
    {
        if self.last_heartbeat.elapsed() >= Duration::from_secs(1) {
            board.send_heartbeat();
            self.last_heartbeat = Instant::now();
        }
    }

    fn try_auto_arm<C, M>(&mut self, board: &mut XPlaneBoard<C, M>)
    where
        C: aviate_core::control::VehicleController,
        M: aviate_core::mixer::Mixer,
    {
        let due = self
            .auto_arm
            .is_some_and(|delay| self.started.elapsed() >= delay);
        if self.armed || !board.is_ready() || !due {
            return;
        }
        match board.arm() {
            Ok(()) => {
                log::info!("auto-armed");
                self.armed = true;
            }
            Err(error) => log::warn!("auto-arm refused: {error:?}"),
        }
    }

    fn report_status<C, M>(
        &mut self,
        board: &XPlaneBoard<C, M>,
        command: &aviate_core::mixer::ActuatorCmd,
    ) where
        C: aviate_core::control::VehicleController,
        M: aviate_core::mixer::Mixer,
    {
        if self.last_report.elapsed() < Duration::from_secs(5) {
            return;
        }
        let (rx, tx, crc, unsent, connects) = board.stats();
        let estimate = board
            .kernel()
            .pipeline()
            .estimator
            .estimate(&board.kernel().state.estimator);
        let fix = fix_summary(board.last_fix());
        let outputs = output_summary(command);
        let phase = phase(board);
        let vertical_speed = estimate.velocity_ned[2].0;
        let vertical_position = estimate.position_ned[2].0;
        log::info!(
            "link rx={rx} tx={tx} crc_errors={crc} unsent={unsent} connects={connects} \
             phase={phase} {fix} est_d={vertical_position:.1}m \
             est_vz={vertical_speed:.2} motors=[{outputs}]"
        );
        self.last_report = Instant::now();
    }
}

fn fix_summary(fix: Option<&aviate_hal_xil::sim_types::SimGnssData>) -> String {
    fix.map_or_else(
        || "fix=none".to_owned(),
        |value| {
            format!(
                "fix={:?} sats={} n={:.1}m e={:.1}m d={:.1}m alt={:.1}m",
                value.fix,
                value.satellites,
                value.position_ned[0],
                value.position_ned[1],
                value.position_ned[2],
                value.alt_m
            )
        },
    )
}

fn output_summary(command: &aviate_core::mixer::ActuatorCmd) -> String {
    command.outputs[..4]
        .iter()
        .map(|lane| format!("{:.2}", lane.0))
        .collect::<Vec<_>>()
        .join(",")
}

fn phase<C, M>(board: &XPlaneBoard<C, M>) -> &'static str
where
    C: aviate_core::control::VehicleController,
    M: aviate_core::mixer::Mixer,
{
    if board.is_armed() {
        "armed"
    } else if board.is_ready() {
        "ready"
    } else {
        "init"
    }
}

fn pace<C, M>(board: &mut XPlaneBoard<C, M>, cycle_start: Instant)
where
    C: aviate_core::control::VehicleController,
    M: aviate_core::mixer::Mixer,
{
    if !board.wait_for_sample(IDLE_WAIT) && !board.connected() {
        if let Some(remaining) = IDLE_WAIT.checked_sub(cycle_start.elapsed()) {
            std::thread::sleep(remaining);
        }
    }
}
