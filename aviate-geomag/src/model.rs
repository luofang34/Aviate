//! Spherical-harmonic synthesis of the WMM main field.
//!
//! The recursion is the one of the NOAA reference implementation: Schmidt
//! semi-normalized associated Legendre functions in geocentric
//! coordinates, the field summed in a spherical frame, then rotated to the
//! geodetic north-east-down frame. At the geographic poles the east
//! component is summed with the reference implementation's special-case
//! recursion, so it stays finite.

use libm::{atan2, cos, sin, sqrt};

use crate::coefficients::{COEFFICIENTS, EPOCH, MAX_DEGREE};
use crate::error::GeomagError;
use crate::field::{GeodeticPoint, MagneticField, WMM2025};

/// WGS-84 semi-major axis, km.
const WGS84_A_KM: f64 = 6_378.137;
/// WGS-84 semi-minor axis, km.
const WGS84_B_KM: f64 = 6_356.752_314_2;
/// Geomagnetic reference radius, km.
const REFERENCE_RADIUS_KM: f64 = 6_371.2;
/// The model's height domain, km above the ellipsoid.
const MIN_HEIGHT_KM: f64 = -1.0;
const MAX_HEIGHT_KM: f64 = 850.0;

const N: usize = MAX_DEGREE + 1;

/// Coefficients at one date, Schmidt-normalized for the unnormalized
/// Legendre recursion: `c[m][n]` holds g(n, m) and `c[n][m - 1]` holds
/// h(n, m), the layout of the reference implementation.
struct Normalized {
    c: [[f64; N]; N],
    k: [[f64; N]; N],
}

fn normalized_at(decimal_year: f64) -> Normalized {
    let dt = decimal_year - EPOCH;
    let mut c = [[0.0; N]; N];
    for &(n, m, g, h, g_dot, h_dot) in COEFFICIENTS.iter() {
        c[m][n] = g + dt * g_dot;
        if m > 0 {
            c[n][m - 1] = h + dt * h_dot;
        }
    }

    let mut snorm = [[0.0; N]; N];
    let mut k = [[0.0; N]; N];
    snorm[0][0] = 1.0;
    for n in 1..N {
        let nf = n as f64;
        snorm[0][n] = snorm[0][n - 1] * (2.0 * nf - 1.0) / nf;
        let mut j = 2.0;
        for m in 0..=n {
            let mf = m as f64;
            k[m][n] = ((nf - 1.0) * (nf - 1.0) - mf * mf) / ((2.0 * nf - 1.0) * (2.0 * nf - 3.0));
            if m > 0 {
                let flnmj = (nf - mf + 1.0) * j / (nf + mf);
                snorm[m][n] = snorm[m - 1][n] * sqrt(flnmj);
                j = 1.0;
                c[n][m - 1] *= snorm[m][n];
            }
            c[m][n] *= snorm[m][n];
        }
    }
    Normalized { c, k }
}

/// Evaluate WMM2025 at `point` on `decimal_year`.
///
/// # Errors
///
/// [`GeomagError::OutsideValidity`] for a date outside 2025.0..=2030.0,
/// and [`GeomagError::InvalidPoint`] for a latitude outside [-90, 90], a
/// height outside -1 km..=850 km, or any non-finite input.
pub fn wmm2025(point: GeodeticPoint, decimal_year: f64) -> Result<MagneticField, GeomagError> {
    if !(WMM2025.valid_from..=WMM2025.valid_until).contains(&decimal_year) {
        return Err(GeomagError::OutsideValidity {
            year: decimal_year,
            valid_from: WMM2025.valid_from,
            valid_until: WMM2025.valid_until,
        });
    }
    let height_km = point.height_m / 1_000.0;
    if !(-90.0..=90.0).contains(&point.latitude_deg)
        || !point.longitude_deg.is_finite()
        || !(MIN_HEIGHT_KM..=MAX_HEIGHT_KM).contains(&height_km)
    {
        return Err(GeomagError::InvalidPoint);
    }

    let model = normalized_at(decimal_year);
    let [north, east, down] = synthesize(&model, point, height_km);
    let horizontal = sqrt(north * north + east * east);
    Ok(MagneticField {
        north_nt: north,
        east_nt: east,
        down_nt: down,
        horizontal_nt: horizontal,
        total_nt: sqrt(horizontal * horizontal + down * down),
        declination_deg: atan2(east, north).to_degrees(),
        inclination_deg: atan2(down, horizontal).to_degrees(),
        model: WMM2025,
    })
}

/// The field in geodetic north-east-down components, nT.
fn synthesize(model: &Normalized, point: GeodeticPoint, alt: f64) -> [f64; 3] {
    let (a2, b2) = (WGS84_A_KM * WGS84_A_KM, WGS84_B_KM * WGS84_B_KM);
    let c2 = a2 - b2;
    let (a4, b4) = (a2 * a2, b2 * b2);
    let c4 = a4 - b4;

    let rlat = point.latitude_deg.to_radians();
    let rlon = point.longitude_deg.to_radians();
    let (srlat, crlat) = (sin(rlat), cos(rlat));
    let (srlat2, crlat2) = (srlat * srlat, crlat * crlat);

    // Geodetic → geocentric spherical coordinates.
    let q = sqrt(a2 - c2 * srlat2);
    let q1 = alt * q;
    let q2 = ((q1 + a2) / (q1 + b2)) * ((q1 + a2) / (q1 + b2));
    let ct = srlat / sqrt(q2 * crlat2 + srlat2);
    let st = sqrt((1.0 - ct * ct).max(0.0));
    let r = sqrt(alt * alt + 2.0 * q1 + (a4 - c4 * srlat2) / (q * q));
    let d = sqrt(a2 * crlat2 + b2 * srlat2);
    let ca = (alt + d) / r;
    let sa = c2 * crlat * srlat / (r * d);

    let mut sp = [0.0; N];
    let mut cp = [0.0; N];
    cp[0] = 1.0;
    sp[1] = sin(rlon);
    cp[1] = cos(rlon);
    for m in 2..N {
        sp[m] = sp[1] * cp[m - 1] + cp[1] * sp[m - 1];
        cp[m] = cp[1] * cp[m - 1] - sp[1] * sp[m - 1];
    }

    let mut p = [[0.0; N]; N];
    let mut dp = [[0.0; N]; N];
    let mut pp = [0.0; N];
    p[0][0] = 1.0;
    pp[0] = 1.0;

    let aor = REFERENCE_RADIUS_KM / r;
    let mut ar = aor * aor;
    let (mut br, mut bt, mut bp, mut bpp) = (0.0, 0.0, 0.0, 0.0);
    for n in 1..N {
        ar *= aor;
        for m in 0..=n {
            if n == m {
                p[m][n] = st * p[m - 1][n - 1];
                dp[m][n] = st * dp[m - 1][n - 1] + ct * p[m - 1][n - 1];
            } else if n == 1 {
                p[m][n] = ct * p[m][n - 1];
                dp[m][n] = ct * dp[m][n - 1] - st * p[m][n - 1];
            } else {
                let (p2, dp2) = if m + 2 > n {
                    (0.0, 0.0)
                } else {
                    (p[m][n - 2], dp[m][n - 2])
                };
                p[m][n] = ct * p[m][n - 1] - model.k[m][n] * p2;
                dp[m][n] = ct * dp[m][n - 1] - st * p[m][n - 1] - model.k[m][n] * dp2;
            }

            let g = model.c[m][n];
            let (temp1, temp2) = if m == 0 {
                (g * cp[m], g * sp[m])
            } else {
                let h = model.c[n][m - 1];
                (g * cp[m] + h * sp[m], g * sp[m] - h * cp[m])
            };
            let par = ar * p[m][n];
            bt -= ar * temp1 * dp[m][n];
            bp += m as f64 * temp2 * par;
            br += (n as f64 + 1.0) * temp1 * par;

            // At a geographic pole `bp / st` is 0/0; the reference
            // implementation sums the m = 1 terms with their own recursion.
            if st == 0.0 && m == 1 {
                pp[n] = if n == 1 {
                    pp[n - 1]
                } else {
                    ct * pp[n - 1] - model.k[m][n] * pp[n - 2]
                };
                bpp += temp2 * ar * pp[n];
            }
        }
    }
    let bp = if st == 0.0 { bpp } else { bp / st };

    // Spherical → geodetic north-east-down.
    [-bt * ca - br * sa, bp, bt * sa - br * ca]
}

#[cfg(test)]
mod tests;
