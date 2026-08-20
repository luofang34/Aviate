//! The identification experiment's X-Plane side: dataref reads and
//! writes over the simulator's UDP protocol, the virtual test stand
//! that pins translation while leaving rotation to the flight
//! model, and the recorded control cycle.

use std::net::UdpSocket;
use std::time::{Duration, Instant};

/// X-Plane's UDP command/dataref port on this host.
const XPLANE_UDP: &str = "127.0.0.1:49000";

/// Reads one dataref value via RREF (subscribe, take the first
/// answer, unsubscribe). `None` when X-Plane does not answer in time.
pub(super) fn read_dataref(sock: &UdpSocket, path: &str) -> Option<f32> {
    let mut req = Vec::with_capacity(413);
    req.extend_from_slice(b"RREF\x00");
    req.extend_from_slice(&10_i32.to_le_bytes());
    req.extend_from_slice(&1_i32.to_le_bytes());
    let mut name = [0_u8; 400];
    name[..path.len().min(400)].copy_from_slice(&path.as_bytes()[..path.len().min(400)]);
    req.extend_from_slice(&name);
    sock.send_to(&req, XPLANE_UDP).ok()?;
    let mut value = None;
    let deadline = Instant::now() + Duration::from_millis(800);
    let mut buf = [0_u8; 1024];
    while Instant::now() < deadline {
        let Ok((len, _)) = sock.recv_from(&mut buf) else {
            continue;
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
    value
}

/// Writes one dataref via DREF.
pub(super) fn write_dataref(sock: &UdpSocket, path: &str, value: f32) {
    let mut req = Vec::with_capacity(509);
    req.extend_from_slice(b"DREF\x00");
    req.extend_from_slice(&value.to_le_bytes());
    let mut name = [b' '; 500];
    let len = path.len().min(499);
    name[..len].copy_from_slice(&path.as_bytes()[..len]);
    name[len] = 0;
    req.extend_from_slice(&name);
    sock.send_to(&req, XPLANE_UDP).ok();
}

/// The virtual test stand: every cycle of an excitation window pins
/// the vehicle's TRANSLATION (altitude held, linear velocity zeroed)
/// while leaving its ROTATION entirely to the flight model — the
/// degree-of-freedom separation a physical identification rig provides
/// with a gimbal mount. A SITL-only affordance, and the reason the
/// experiment needs no working attitude cascade: free fall would bury
/// the torque response under the aerodynamics of a 30 m/s plunge.
pub(super) struct TestStand {
    sock: UdpSocket,
    held_y: Option<f32>,
}

impl TestStand {
    pub(super) fn new(sock: UdpSocket) -> Self {
        Self { sock, held_y: None }
    }

    /// Captures the hold altitude `delta_m` above the current one.
    pub(super) fn engage(&mut self, delta_m: f32) {
        match read_dataref(&self.sock, "sim/flightmodel/position/local_y") {
            Some(y) => {
                self.held_y = Some(y + delta_m);
                log::info!("test stand engaged {delta_m:.0} m up");
            }
            None => {
                log::warn!("test stand: X-Plane did not answer; exciting in free flight");
            }
        }
    }

    /// One pin: linear velocity zeroed, altitude restored.
    pub(super) fn pin(&self) {
        let Some(y) = self.held_y else {
            return;
        };
        for axis in ["local_vx", "local_vy", "local_vz"] {
            write_dataref(&self.sock, &format!("sim/flightmodel/position/{axis}"), 0.0);
        }
        write_dataref(&self.sock, "sim/flightmodel/position/local_y", y);
    }

    /// Zeroes the body rotation rates, so each excitation window
    /// starts from rotational rest.
    pub(super) fn zero_rates(&self) {
        for axis in ["P", "Q", "R"] {
            write_dataref(&self.sock, &format!("sim/flightmodel/position/{axis}"), 0.0);
        }
    }

    pub(super) fn release(&mut self) {
        self.held_y = None;
    }
}

/// One recorded control cycle.
pub(super) struct Sample {
    pub(super) at: Instant,
    /// Reconstructed normalized axis torques (force domain).
    pub(super) u: [f32; 3],
    /// Body rates, rad/s.
    pub(super) gyro: [f32; 3],
}
