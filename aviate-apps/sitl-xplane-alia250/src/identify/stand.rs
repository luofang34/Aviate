//! The identification experiment's X-Plane side: dataref reads and
//! writes over the simulator's UDP protocol, the virtual test stand
//! that pins translation while leaving rotation to the flight
//! model, and the recorded control cycle.

use std::net::UdpSocket;
use std::time::{Duration, Instant};

/// X-Plane test-stand failure.
#[derive(Debug)]
pub(crate) enum StandError {
    Transport(std::io::Error),
    NoResponse(&'static str),
    Readback {
        field: &'static str,
        expected: f32,
        actual: f32,
    },
}

impl core::fmt::Display for StandError {
    fn fmt(&self, formatter: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Transport(error) => write!(formatter, "X-Plane UDP transport failed: {error}"),
            Self::NoResponse(field) => write!(formatter, "X-Plane did not return {field}"),
            Self::Readback {
                field,
                expected,
                actual,
            } => write!(
                formatter,
                "X-Plane {field} readback {actual} does not match {expected}"
            ),
        }
    }
}

impl std::error::Error for StandError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Transport(error) => Some(error),
            Self::NoResponse(_) | Self::Readback { .. } => None,
        }
    }
}

/// X-Plane's UDP command/dataref port on this host.
const XPLANE_UDP: &str = "127.0.0.1:49000";

/// Reads one dataref value via RREF (subscribe, take the first
/// answer, unsubscribe). `None` when X-Plane does not answer in time.
pub(super) fn read_dataref(
    sock: &UdpSocket,
    path: &str,
    field: &'static str,
) -> Result<f32, StandError> {
    let mut req = Vec::with_capacity(413);
    req.extend_from_slice(b"RREF\x00");
    req.extend_from_slice(&10_i32.to_le_bytes());
    req.extend_from_slice(&1_i32.to_le_bytes());
    let mut name = [0_u8; 400];
    name[..path.len().min(400)].copy_from_slice(&path.as_bytes()[..path.len().min(400)]);
    req.extend_from_slice(&name);
    sock.send_to(&req, XPLANE_UDP)
        .map_err(StandError::Transport)?;
    let mut value = None;
    let deadline = Instant::now() + Duration::from_millis(800);
    let mut buf = [0_u8; 1024];
    while Instant::now() < deadline {
        let (len, _) = match sock.recv_from(&mut buf) {
            Ok(received) => received,
            Err(error)
                if matches!(
                    error.kind(),
                    std::io::ErrorKind::WouldBlock | std::io::ErrorKind::TimedOut
                ) =>
            {
                continue;
            }
            Err(error) => return Err(StandError::Transport(error)),
        };
        if len >= 13 && &buf[..4] == b"RREF" {
            let mut idx = [0_u8; 4];
            idx.copy_from_slice(&buf[5..9]);
            if i32::from_le_bytes(idx) == 1 {
                let mut raw = [0_u8; 4];
                raw.copy_from_slice(&buf[9..13]);
                value = Some(f32::from_le_bytes(raw));
                break;
            }
        }
    }
    // Unsubscribe.
    let mut off = Vec::with_capacity(413);
    off.extend_from_slice(b"RREF\x00");
    off.extend_from_slice(&0_i32.to_le_bytes());
    off.extend_from_slice(&1_i32.to_le_bytes());
    off.extend_from_slice(&name);
    sock.send_to(&off, XPLANE_UDP).ok();
    value.ok_or(StandError::NoResponse(field))
}

/// Writes one dataref via DREF.
pub(super) fn write_dataref(sock: &UdpSocket, path: &str, value: f32) -> Result<(), StandError> {
    let mut req = Vec::with_capacity(509);
    req.extend_from_slice(b"DREF\x00");
    req.extend_from_slice(&value.to_le_bytes());
    let mut name = [b' '; 500];
    let len = path.len().min(499);
    name[..len].copy_from_slice(&path.as_bytes()[..len]);
    name[len] = 0;
    req.extend_from_slice(&name);
    sock.send_to(&req, XPLANE_UDP)
        .map(|_| ())
        .map_err(StandError::Transport)
}

/// The virtual test stand: pins the vehicle's TRANSLATION (altitude
/// held, linear velocity zeroed) while leaving its ROTATION entirely
/// to the flight model. The excitation windows do NOT use it — a
/// translation pin couples back into rotation and rocks the vehicle
/// harder than the free hover it would replace — so its one flight
/// role is the landing ride at the end of the sequence, plus the
/// ground-referenced altitude reads the sequence plans against.
pub(super) struct TestStand {
    sock: UdpSocket,
    held_y: Option<f32>,
    confirmed: bool,
}

impl TestStand {
    pub(super) fn new(sock: UdpSocket) -> Self {
        Self {
            sock,
            held_y: None,
            confirmed: false,
        }
    }

    /// Engages the stand AT the current altitude. Raising to the
    /// working height is a ride, not a step: a teleport arrives at the
    /// estimator as a position innovation the size of the jump, and the
    /// filter's chase poisons every state the controller then trusts.
    pub(super) fn engage(&mut self) -> Result<f32, StandError> {
        let y = read_dataref(&self.sock, "sim/flightmodel/position/local_y", "local_y")?;
        self.held_y = Some(y);
        self.confirmed = false;
        log::info!("test stand engaged");
        Ok(y)
    }

    /// One pin: linear velocity zeroed, altitude restored.
    pub(super) fn pin(&mut self) -> Result<(), StandError> {
        let Some(y) = self.held_y else {
            return Err(StandError::NoResponse("held local_y"));
        };
        for axis in ["local_vx", "local_vy", "local_vz"] {
            write_dataref(&self.sock, &format!("sim/flightmodel/position/{axis}"), 0.0)?;
        }
        write_dataref(&self.sock, "sim/flightmodel/position/local_y", y)?;
        if !self.confirmed {
            let actual = read_dataref(&self.sock, "sim/flightmodel/position/local_y", "local_y")?;
            verify_readback("local_y", y, actual, 0.5)?;
            self.confirmed = true;
        }
        Ok(())
    }

    /// Zeroes the body rotation rates, so each excitation window
    /// starts from rotational rest.
    ///
    /// The readback races the flight model: at least one physics frame
    /// runs between the write and the answer, and the live attitude
    /// loop rebuilds a small rate in that frame. Each retry re-zeroes,
    /// and the accepted residual is far below the probe amplitudes the
    /// windows drive, so a pass means "at rest" in every sense the
    /// correlation fit can see.
    pub(super) fn zero_rates(&self) -> Result<(), StandError> {
        for axis in ["P", "Q", "R"] {
            let path = format!("sim/flightmodel/position/{axis}");
            let mut outcome = Ok(());
            for _ in 0..5 {
                write_dataref(&self.sock, &path, 0.0)?;
                let actual = read_dataref(&self.sock, &path, "body rate")?;
                outcome = verify_readback("body rate", 0.0, actual, 0.15);
                if outcome.is_ok() {
                    break;
                }
            }
            outcome?;
        }
        Ok(())
    }

    /// Steps the hold altitude down, toward a landing the interlock
    /// will accept. The pin keeps zeroing velocity, so this is an
    /// elevator ride, not a fall.
    pub(super) fn lower(&mut self, by_m: f32, floor_y: f32) {
        if let Some(y) = self.held_y {
            self.held_y = Some((y - by_m).max(floor_y));
        }
    }

    /// The current hold altitude, when engaged.
    pub(super) fn held(&self) -> Option<f32> {
        self.held_y
    }

    /// Reads the vehicle's current local vertical position.
    pub(super) fn local_y(&self) -> Result<f32, StandError> {
        read_dataref(&self.sock, "sim/flightmodel/position/local_y", "local_y")
    }

    pub(super) fn release(&mut self) {
        self.held_y = None;
        self.confirmed = false;
    }
}

/// One recorded control cycle.
#[derive(Clone, Copy)]
pub(super) struct Sample {
    pub(super) timestamp_us: u64,
    /// Reconstructed normalized axis torques (force domain).
    pub(super) u: [f32; 3],
    /// Body rates, rad/s.
    pub(super) gyro: [f32; 3],
    /// Applied mean force-domain collective.
    pub(super) collective_force: f32,
    /// The individual census constraints, in [`CONSTRAINT_NAMES`] order.
    pub(super) constraints: [bool; 5],
}

impl Sample {
    /// True when any census constraint touched this sample — the one
    /// derivation, so the census and its breakdown cannot disagree.
    pub(super) fn saturated(&self) -> bool {
        self.constraints.iter().any(|hit| *hit)
    }
}

/// Names for [`Sample::constraints`], in field order.
pub(super) const CONSTRAINT_NAMES: [&str; 5] = [
    "lane-ceiling",
    "injection-clamp",
    "actuator-count",
    "missing-answer",
    "ground-squeeze",
];

fn verify_readback(
    field: &'static str,
    expected: f32,
    actual: f32,
    tolerance: f32,
) -> Result<(), StandError> {
    if (expected - actual).abs() <= tolerance {
        Ok(())
    } else {
        Err(StandError::Readback {
            field,
            expected,
            actual,
        })
    }
}
