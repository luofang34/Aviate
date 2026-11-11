//! A curved Earth under the flat plant.
//!
//! The plant flies in its own local level: its north, east and down are
//! those at the vehicle, its height is above the datum, and its body
//! kinematics are relative to that level. Its north and east positions
//! are the configured tangent plane's horizontal coordinates. The host
//! models then see what a real receiver and a real IMU give over a
//! curved Earth: the GNSS position and velocity in the configured plane,
//! and a gyroscope that also measures the turn of the local level as the
//! vehicle moves over the ellipsoid (the transport rate).

use aviate_xil_direct::synthesis::rotate_world_to_body;
use world_core::{GeodeticPosition, LocalTangentPlane, Ned};

const WGS84_A_M: f64 = 6_378_137.0;
const WGS84_E2: f64 = 6.694_379_990_14e-3;

pub struct Earth {
    plane: LocalTangentPlane,
    datum_height_m: f64,
}

/// Rows: the north, east and down axes at a point in Earth-fixed axes.
fn axes(latitude_deg: f64, longitude_deg: f64) -> [[f64; 3]; 3] {
    let (sl, cl) = latitude_deg.to_radians().sin_cos();
    let (so, co) = longitude_deg.to_radians().sin_cos();
    [
        [-sl * co, -sl * so, cl],
        [-so, co, 0.0],
        [-cl * co, -cl * so, -sl],
    ]
}

impl Earth {
    pub fn new(origin: GeodeticPosition) -> Self {
        Self {
            plane: LocalTangentPlane::new(origin).expect("origin"),
            datum_height_m: origin.ellipsoidal_height_m,
        }
    }

    /// The vehicle's geodetic position.
    pub fn position(&self, north_m: f32, east_m: f32, height_m: f32) -> GeodeticPosition {
        let surface = self.plane.to_geodetic(Ned {
            north_m: f64::from(north_m),
            east_m: f64::from(east_m),
            down_m: 0.0,
        });
        GeodeticPosition::wgs84(
            surface.latitude_deg,
            surface.longitude_deg,
            self.datum_height_m + f64::from(height_m),
        )
    }

    /// Position in the configured plane.
    pub fn plane_position(&self, at: GeodeticPosition) -> [f32; 3] {
        let ned = self.plane.to_ned(at).expect("well formed");
        [ned.north_m as f32, ned.east_m as f32, ned.down_m as f32]
    }

    /// Position in the plane at `origin_lat_lon`, on the datum height.
    pub fn position_in_frame(&self, at: GeodeticPosition, origin_lat_lon: (f64, f64)) -> [f32; 3] {
        let plane = LocalTangentPlane::new(GeodeticPosition::wgs84(
            origin_lat_lon.0,
            origin_lat_lon.1,
            self.datum_height_m,
        ))
        .expect("frame origin");
        let ned = plane.to_ned(at).expect("well formed");
        [ned.north_m as f32, ned.east_m as f32, ned.down_m as f32]
    }

    /// A local north-east-down vector at `at` in the configured plane.
    pub fn plane_vector(&self, at: GeodeticPosition, local: [f32; 3]) -> [f32; 3] {
        let origin = self.plane.origin();
        let plane_axes = axes(origin.latitude_deg, origin.longitude_deg);
        let local_axes = axes(at.latitude_deg, at.longitude_deg);
        let v = local.map(f64::from);
        // Earth-fixed vector, then plane components.
        let ecef: [f64; 3] = core::array::from_fn(|k| {
            local_axes[0][k] * v[0] + local_axes[1][k] * v[1] + local_axes[2][k] * v[2]
        });
        plane_axes.map(|axis| (axis[0] * ecef[0] + axis[1] * ecef[1] + axis[2] * ecef[2]) as f32)
    }

    /// Turn rate of the local level over the ellipsoid, in body axes, for
    /// a vehicle with local ground velocity `velocity` and attitude
    /// `attitude_wxyz` relative to the local level.
    pub fn transport_rate_body(
        &self,
        at: GeodeticPosition,
        velocity: [f32; 3],
        attitude_wxyz: [f32; 4],
    ) -> [f32; 3] {
        let latitude = at.latitude_deg.to_radians();
        let w = 1.0 - WGS84_E2 * latitude.sin().powi(2);
        let prime_vertical = WGS84_A_M / w.sqrt() + at.ellipsoidal_height_m;
        let meridian = WGS84_A_M * (1.0 - WGS84_E2) / w.powf(1.5) + at.ellipsoidal_height_m;
        let [north, east, _] = velocity.map(f64::from);
        let rate = [
            east / prime_vertical,
            -north / meridian,
            -east * latitude.tan() / prime_vertical,
        ];
        rotate_world_to_body(attitude_wxyz, rate.map(|r| r as f32))
    }
}
