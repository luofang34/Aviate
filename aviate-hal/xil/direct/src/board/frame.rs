//! Navigation-frame rebasing of [`super::DirectXilBoard`].
//!
//! The estimator works in a local tangent plane. The board moves the
//! plane's origin to the vehicle when the vehicle is farther than the
//! rebase radius from it, through the estimator's rigid
//! [`aviate_core::ekf::rebase`] hook. Hosts keep reporting GNSS in the
//! configured frame; the board maps each fix into the present frame.
//!
//! `LOCAL_POSITION_NED` and local position targets are relative to the
//! newest `GPS_GLOBAL_ORIGIN`, which the board sends after each move. A
//! retained target moves with the frame. A height is above the datum
//! and does not change with the origin.

use aviate_core::control::Command;
use aviate_core::ekf::rebase::{FrameShift, NavOrigin};
use aviate_core::types::{Meters, MetersPerSecond, Radians};
use aviate_hal_io::RawGnssReading;
use world_core::{GeodeticPosition, LocalTangentPlane, Ned};

use super::DirectXilBoard;

/// Default distance \[m\] from the frame origin at which the board moves
/// the origin to the vehicle.
///
/// A tangent plane drops below the ellipsoid by `d² / 2R`: 0.08 m at
/// 1 km, 0.31 m at 2 km, 1.96 m at 5 km. At 1 km the drop is well below
/// the noise of a barometer, so a height target and a plane-z estimate
/// agree within it, and a height target steps by at most 0.08 m at a
/// rebase.
pub const DEFAULT_REBASE_RADIUS_M: f64 = 1_000.0;

/// The navigation frame of the board: the origin of every local
/// quantity it reports or reads.
///
/// `LOCAL_POSITION_NED`, the estimate's position and velocity, the
/// retained setpoint, and local position and velocity targets are all
/// relative to `origin` and along its north, east and down axes. The
/// origin starts at the configured origin and moves to the vehicle at
/// each rebase, so a host reads local quantities against this origin
/// and not against the configured one.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DirectNavFrame {
    /// Origin of the tangent plane, on the configured datum height.
    pub origin: GeodeticPosition,
    /// Number of times the origin has moved since the estimator was seeded.
    pub rebases: u32,
}

/// Rows: the north, east and down axes of the tangent plane at
/// `(latitude, longitude)` in Earth-fixed axes.
fn plane_axes(latitude_deg: f64, longitude_deg: f64) -> [[f64; 3]; 3] {
    let (sl, cl) = (
        libm::sin(latitude_deg.to_radians()),
        libm::cos(latitude_deg.to_radians()),
    );
    let (so, co) = (
        libm::sin(longitude_deg.to_radians()),
        libm::cos(longitude_deg.to_radians()),
    );
    [
        [-sl * co, -sl * so, cl],
        [-so, co, 0.0],
        [-cl * co, -cl * so, -sl],
    ]
}

/// Rotation from the axes of the plane at `from` to those at `to`.
fn rotation_between(from: &NavOrigin, to: &NavOrigin) -> [[f64; 3]; 3] {
    let old = plane_axes(from.latitude_deg, from.longitude_deg);
    let new = plane_axes(to.latitude_deg, to.longitude_deg);
    new.map(|new_axis| {
        old.map(|old_axis| {
            new_axis
                .iter()
                .zip(old_axis)
                .map(|(a, b)| a * b)
                .sum::<f64>()
        })
    })
}

/// The rigid map from the plane at `from` to the plane at `to`, both on
/// the datum height `height_m`. `None` for an origin that is not a
/// well-formed position.
fn shift_between(from: &NavOrigin, to: &NavOrigin, height_m: f64) -> Option<FrameShift> {
    let plane = LocalTangentPlane::new(GeodeticPosition::wgs84(
        from.latitude_deg,
        from.longitude_deg,
        height_m,
    ))
    .ok()?;
    let offset = plane
        .to_ned(GeodeticPosition::wgs84(
            to.latitude_deg,
            to.longitude_deg,
            height_m,
        ))
        .ok()?;
    Some(FrameShift {
        rotation: rotation_between(from, to),
        offset_m: [offset.north_m, offset.east_m, offset.down_m],
        origin: *to,
    })
}

impl<C, M> DirectXilBoard<C, M>
where
    C: aviate_core::control::VehicleController,
    M: aviate_core::mixer::Mixer,
{
    /// Set the distance \[m\] from the frame origin at which the board
    /// moves the origin to the vehicle; see [`DEFAULT_REBASE_RADIUS_M`].
    /// A value that is not finite and positive keeps the present one.
    pub fn set_rebase_radius(&mut self, radius_m: f64) {
        if radius_m.is_finite() && radius_m > 0.0 {
            self.rebase_radius_m = radius_m;
        }
    }

    /// The navigation frame that local quantities are relative to.
    ///
    /// Read it with every local quantity: after a rebase the origin is
    /// no longer the configured one. `GPS_GLOBAL_ORIGIN` announces the
    /// same origin over the link.
    #[must_use]
    pub fn navigation_frame(&self) -> DirectNavFrame {
        let origin = self.frame_origin();
        DirectNavFrame {
            origin: GeodeticPosition::wgs84(
                origin.latitude_deg,
                origin.longitude_deg,
                self.config.origin.ellipsoidal_height_m,
            ),
            rebases: self.kernel.state.estimator.frame.rebases,
        }
    }

    /// The configured origin, as a frame origin.
    fn configured_origin(&self) -> NavOrigin {
        NavOrigin {
            latitude_deg: self.config.origin.latitude_deg,
            longitude_deg: self.config.origin.longitude_deg,
        }
    }

    /// Origin of the frame that the estimate is in.
    pub(super) fn frame_origin(&self) -> NavOrigin {
        self.kernel
            .state
            .estimator
            .frame
            .origin
            .unwrap_or_else(|| self.configured_origin())
    }

    /// Move the frame origin to the vehicle when it is beyond the radius.
    ///
    /// The new origin is the point of the datum below the estimated
    /// position. Returns whether the frame moved.
    pub(super) fn maybe_rebase(&mut self) -> bool {
        let estimator = &self.kernel.state.estimator;
        if !self.ekf_initialized || !estimator.is_initialized() {
            return false;
        }
        let [north, east, down] =
            [estimator.pos.x.0, estimator.pos.y.0, estimator.pos.z.0].map(f64::from);
        if libm::hypot(north, east) <= self.rebase_radius_m {
            return false;
        }
        let height_m = self.config.origin.ellipsoidal_height_m;
        let from = self.frame_origin();
        let Ok(plane) = LocalTangentPlane::new(GeodeticPosition::wgs84(
            from.latitude_deg,
            from.longitude_deg,
            height_m,
        )) else {
            return false;
        };
        let below = plane.to_geodetic(Ned {
            north_m: north,
            east_m: east,
            down_m: down,
        });
        let to = NavOrigin {
            latitude_deg: below.latitude_deg,
            longitude_deg: below.longitude_deg,
        };
        let (Some(shift), Some(map)) = (
            shift_between(&from, &to, height_m),
            shift_between(&self.configured_origin(), &to, height_m),
        ) else {
            return false;
        };
        self.kernel.rebase_navigation_frame(&shift);
        self.ingress.rebase(&shift);
        self.frame_map = Some(map);
        // Hosts express local targets against the newest origin.
        self.origin_announced = false;
        log::info!(
            "DirectXilBoard frame origin moved to {:.7}, {:.7}",
            to.latitude_deg,
            to.longitude_deg
        );
        true
    }

    /// Express a host GNSS fix, reported in the configured frame, in the
    /// frame of the estimate.
    pub(super) fn map_gnss(&self, mut reading: RawGnssReading) -> RawGnssReading {
        let Some(map) = self.frame_map.as_ref() else {
            return reading;
        };
        let to_f32 = |v: [f64; 3]| v.map(|c| c as f32);
        reading.position_ned = to_f32(map.position(reading.position_ned.map(f64::from)));
        reading.vel_ned = to_f32(map.direction(reading.vel_ned.map(f64::from)));
        reading
    }
}

/// Move a retained command into a new frame: positions and velocities
/// rigidly, a heading by the turn of north. Heights are above the datum
/// and stay.
pub(super) fn rebase_command(command: &mut Command, shift: &FrameShift) {
    let setpoint = &mut command.setpoint;
    if let Some(position) = setpoint.position.as_mut() {
        let moved = shift.position(position.map(|m| f64::from(m.0)));
        *position = moved.map(|c| Meters(c as f32));
    }
    if let Some(velocity) = setpoint.velocity.as_mut() {
        let turned = shift.direction(velocity.map(|v| f64::from(v.0)));
        *velocity = turned.map(|c| MetersPerSecond(c as f32));
    }
    if let Some(heading) = setpoint.heading.as_mut() {
        *heading = Radians(heading.0 + shift.heading_change() as f32);
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]
mod tests;
