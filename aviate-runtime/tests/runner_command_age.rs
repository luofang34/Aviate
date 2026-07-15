//! #133 regression, at the real command type: the runner's ingress
//! keeps ONE freshness per command class. A redundant `Arm` on a link
//! that stopped carrying setpoints must never refresh the retained
//! setpoint's age — the pre-fix path re-fed a stale setpoint with
//! age≈0 into the kernel and released the CommandLoss terminal. No
//! sleeps: ticks are driven with synthetic clocks.
//!
//! The kernel side of the loop — stale age engages Direct/CommandLoss
//! and only restored recency releases it — is pinned by
//! `command_recovery_releases_descend_terminal` (LLR-FLT-209) in
//! aviate-core's behavioral tests; these tests pin that no discrete
//! command can fake that recency.

#![allow(clippy::expect_used, clippy::panic)]

use std::cell::RefCell;
use std::collections::VecDeque;
use std::rc::Rc;

use aviate_core::control::{Command, CommandSource, ControlMode, Setpoint};
use aviate_core::types::NormalizedThrust;
use aviate_hal_io::{
    SystemCommand, SystemState, TimeHal, TransportHal, TransportStatus, WatchdogHal,
};
use aviate_runtime::runner::{BoardStep, CommandTick, FlightRunner};

fn flight(thrust: f32) -> SystemCommand {
    SystemCommand::FlightControl(Command {
        mode: ControlMode::Attitude,
        setpoint: Setpoint {
            collective_thrust: NormalizedThrust(thrust),
            ..Default::default()
        },
        config_mode_request: None,
        sensor_overrides: None,
        sequence: 0,
        source: CommandSource::Failsafe,
    })
}

#[derive(Debug, Clone, PartialEq)]
struct Seen {
    event: Option<&'static str>,
    setpoint_thrust: Option<f32>,
    setpoint_age_ms: u32,
    link_ok: bool,
}

struct ProbeBoard {
    seen: Rc<RefCell<Vec<Seen>>>,
}

impl BoardStep for ProbeBoard {
    type Cmd = SystemCommand;

    fn board_step(
        &mut self,
        _tick_us: u64,
        _now_us: u64,
        _dt_us: u32,
        cmd: CommandTick<'_, SystemCommand>,
        link_ok: bool,
    ) {
        let event = cmd.event.map(|e| match e {
            SystemCommand::Arm => "arm",
            SystemCommand::Disarm => "disarm",
            SystemCommand::FlightControl(_) => "flight",
        });
        let setpoint_thrust = match cmd.setpoint {
            Some(SystemCommand::FlightControl(c)) => Some(c.setpoint.collective_thrust.0),
            _ => None,
        };
        self.seen.borrow_mut().push(Seen {
            event,
            setpoint_thrust,
            setpoint_age_ms: cmd.setpoint_age_ms,
            link_ok,
        });
    }

    fn sensors_ok(&self) -> bool {
        true
    }

    fn ekf_converged(&self) -> bool {
        true
    }
}

struct FakeTime;
impl TimeHal for FakeTime {
    fn now_us(&mut self) -> u64 {
        0
    }
    fn sleep_until_us(&mut self, _target_us: u64) {}
}

struct FakeWatchdog;
impl WatchdogHal for FakeWatchdog {
    fn kick(&mut self) {}
}

struct QueueTransport {
    queue: VecDeque<SystemCommand>,
}
impl TransportHal<SystemCommand> for QueueTransport {
    fn try_recv_command(&mut self) -> Option<SystemCommand> {
        self.queue.pop_front()
    }
    fn try_send_telemetry(&mut self, _frame: &[u8]) -> bool {
        true
    }
    fn set_system_state(&mut self, _state: SystemState) {}
    fn set_armed(&mut self, _armed: bool) {}
    fn poll(&mut self) {}
    fn status(&self) -> TransportStatus {
        TransportStatus::default()
    }
}

type ProbeRunner = FlightRunner<ProbeBoard, FakeTime, QueueTransport, FakeWatchdog, SystemCommand>;

fn runner(cmds: Vec<SystemCommand>) -> (ProbeRunner, Rc<RefCell<Vec<Seen>>>) {
    let seen = Rc::new(RefCell::new(Vec::new()));
    let board = ProbeBoard { seen: seen.clone() };
    let transport = QueueTransport { queue: cmds.into() };
    (
        FlightRunner::new(board, FakeTime, transport, FakeWatchdog),
        seen,
    )
}

/// The #133 core: a setpoint arrives, the link then carries only a
/// redundant `Arm`. The Arm is delivered exactly once as an event —
/// and the retained setpoint's age KEEPS GROWING through it, so the
/// kernel's command-timeout can fire despite the discrete traffic.
#[test]
fn redundant_arm_never_refreshes_setpoint_age() {
    let (mut r, seen) = runner(vec![flight(0.6), SystemCommand::Arm]);

    r.step(0, 0, 1000); // setpoint arrives at t=0
    r.step(1_000, 800_000, 1000); // Arm arrives at t=0.8 s
    r.step(2_000, 2_000_000, 1000); // silence at t=2 s

    let s = seen.borrow();
    assert_eq!(s[0].event, Some("flight"));
    assert_eq!(s[0].setpoint_age_ms, 0);
    assert_eq!(s[0].setpoint_thrust, Some(0.6));

    assert_eq!(s[1].event, Some("arm"), "discrete delivered exactly once");
    assert_eq!(
        s[1].setpoint_age_ms, 800,
        "Arm must NOT refresh the setpoint age"
    );
    assert_eq!(
        s[1].setpoint_thrust,
        Some(0.6),
        "setpoint retained through the discrete event"
    );

    assert_eq!(s[2].event, None);
    assert_eq!(
        s[2].setpoint_age_ms, 2_000,
        "age keeps growing after the Arm; the timeout can fire"
    );
    assert!(
        !s[2].link_ok,
        "link drops on silence even though an Arm passed at 0.8 s? \
         No — link_ok tracks ANY traffic; 1.2 s since the Arm exceeds \
         the 1 s window"
    );
}

/// Before any setpoint has ever arrived the setpoint age saturates —
/// an `Arm` alone cannot make the vehicle look commanded.
#[test]
fn arm_alone_leaves_the_vehicle_command_stale() {
    let (mut r, seen) = runner(vec![SystemCommand::Arm]);
    r.step(0, 0, 1000);
    r.step(1_000, 100_000, 1000);
    let s = seen.borrow();
    assert_eq!(s[0].event, Some("arm"));
    assert_eq!(s[0].setpoint_age_ms, u32::MAX);
    assert_eq!(s[0].setpoint_thrust, None, "no setpoint to retain");
    assert_eq!(s[1].setpoint_age_ms, u32::MAX);
}

/// Each setpoint receive resets the age; replays between receives
/// deliver the retained value with honest staleness.
#[test]
fn setpoint_receives_reset_age_and_retain_latest() {
    let (mut r, seen) = runner(vec![flight(0.3), flight(0.9)]);
    r.step(0, 0, 1000);
    r.step(1_000, 700_000, 1000); // second setpoint at t=0.7 s
    r.step(2_000, 900_000, 1000); // replay tick
    let s = seen.borrow();
    assert_eq!(s[1].setpoint_age_ms, 0, "age resets on the new receive");
    assert_eq!(s[2].event, None);
    assert_eq!(s[2].setpoint_age_ms, 200);
    assert_eq!(s[2].setpoint_thrust, Some(0.9), "latest setpoint retained");
}
