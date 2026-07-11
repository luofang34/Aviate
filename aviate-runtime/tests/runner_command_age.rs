//! #133 regression: the runner owns command freshness. A retained
//! command replayed to the board on later ticks must NOT read as a
//! receive — the board sees `fresh` only on the arrival tick and an
//! age that keeps growing afterward, so the kernel's command-timeout
//! terminal can fire on link loss. No sleeps: ticks are driven with
//! synthetic clocks.

#![allow(clippy::expect_used, clippy::panic)]

use std::cell::RefCell;
use std::collections::VecDeque;
use std::rc::Rc;

use aviate_hal_io::{SystemState, TimeHal, TransportHal, TransportStatus, WatchdogHal};
use aviate_runtime::runner::{BoardStep, CommandTick, FlightRunner};

#[derive(Debug, Clone, Copy, PartialEq)]
struct Seen {
    fresh: bool,
    age_ms: u32,
    link_ok: bool,
    cmd: u32,
}

/// Probe board: records what the runner hands it each tick.
struct ProbeBoard {
    seen: Rc<RefCell<Vec<Seen>>>,
}

impl BoardStep for ProbeBoard {
    type Cmd = u32;

    fn board_step(
        &mut self,
        _tick_us: u64,
        _now_us: u64,
        _dt_us: u32,
        cmd: CommandTick<'_, u32>,
        link_ok: bool,
    ) {
        self.seen.borrow_mut().push(Seen {
            fresh: cmd.fresh,
            age_ms: cmd.age_ms,
            link_ok,
            cmd: *cmd.cmd,
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
    queue: VecDeque<u32>,
}
impl TransportHal<u32> for QueueTransport {
    fn try_recv_command(&mut self) -> Option<u32> {
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

type ProbeRunner = FlightRunner<ProbeBoard, FakeTime, QueueTransport, FakeWatchdog, u32>;

fn runner(cmds: &[u32]) -> (ProbeRunner, Rc<RefCell<Vec<Seen>>>) {
    let seen = Rc::new(RefCell::new(Vec::new()));
    let board = ProbeBoard { seen: seen.clone() };
    let transport = QueueTransport {
        queue: cmds.iter().copied().collect(),
    };
    (
        FlightRunner::new(board, FakeTime, transport, FakeWatchdog, 0),
        seen,
    )
}

/// The core #133 shape: one command arrives, the link then goes
/// silent, and the runner replays the retained value. The board must
/// see the replay as stale with a growing age — the pre-fix hardware
/// board re-stamped its receive time on every replay, pinning the age
/// near zero forever.
#[test]
fn replayed_retained_command_ages_out() {
    let (mut r, seen) = runner(&[7]);

    r.step(0, 0, 1000); // command 7 arrives at t=0
    r.step(1_000, 500_000, 1000); // replay at t=0.5 s
    r.step(2_000, 2_000_000, 1000); // replay at t=2 s (past 1 s link timeout)

    let s = seen.borrow();
    assert_eq!(
        s[0],
        Seen {
            fresh: true,
            age_ms: 0,
            link_ok: true,
            cmd: 7
        }
    );
    assert_eq!(
        s[1],
        Seen {
            fresh: false,
            age_ms: 500,
            link_ok: true,
            cmd: 7
        },
        "a replay is not a receive: stale, age growing"
    );
    assert!(!s[2].fresh);
    assert_eq!(s[2].age_ms, 2_000, "age keeps growing after link loss");
    assert!(
        !s[2].link_ok,
        "link must drop once the age passes the timeout"
    );
    assert_eq!(s[2].cmd, 7, "retained setpoint still replayed to the board");
}

/// Before any command has ever arrived the age saturates at u32::MAX
/// — a fresh boot is command-stale (failsafe posture), never
/// age-zero-by-default.
#[test]
fn never_received_reads_saturated_age() {
    let (mut r, seen) = runner(&[]);
    r.step(0, 5_000_000, 1000);
    let s = seen.borrow();
    assert!(!s[0].fresh);
    assert_eq!(s[0].age_ms, u32::MAX);
    assert!(!s[0].link_ok);
}

/// A second command re-freshens: fresh flags exactly the arrival
/// ticks, and the age resets on each true receive.
#[test]
fn each_receive_is_fresh_exactly_once() {
    let (mut r, seen) = runner(&[1, 2]);
    r.step(0, 0, 1000); // 1 arrives
    r.step(1_000, 700_000, 1000); // 2 arrives at t=0.7 s
    r.step(2_000, 900_000, 1000); // replay of 2
    let s = seen.borrow();
    assert!(s[0].fresh && s[1].fresh && !s[2].fresh);
    assert_eq!(s[1].age_ms, 0, "age resets on the new receive");
    assert_eq!(s[2].age_ms, 200);
    assert_eq!(s[2].cmd, 2);
}
