#![allow(
    clippy::expect_used,
    clippy::panic,
    clippy::unwrap_used,
    dead_code,
    unused_imports
)]
use aviate_airframe_fixed_wing::c172;
use aviate_core::control::{ControlMode, Setpoint};
use aviate_xil_direct::{DirectNoiseTier, DirectSensorRates};
#[path = "plant/flight.rs"]
mod flight;
use flight::*;
fn run(tag: &str, faults: Faults, noise: DirectNoiseTier, outage: Option<(usize, usize)>, probe: bool) {
    let mut f = Flight::cruising_with(&c172(), PlantModel::C172, noise, DirectSensorRates::default().with_airspeed(40_000), faults);
    if probe { f.probe_errors = Some(ProbeErrors::new(3)); }
    let headings = [0.0_f32, 1.5708, 3.1416, -1.5708, 0.0, 1.5708, 3.1416];
    for (leg, hdg) in headings.iter().enumerate() {
        if let Some((from, to)) = outage { f.gnss_silent = leg >= from && leg < to; }
        let mode = if faults.no_gnss { ControlMode::AltitudeHold } else { ControlMode::VelocityControl };
        let mut worst = [0.0_f32; 3]; let (mut hmin, mut hmax, mut imin, mut imax) = (1e9_f32, -1e9_f32, 1e9_f32, -1e9_f32);
        let mut nees_max = 0.0_f32; let mut poserr = 0.0_f32;
        for _ in 0..12 {
            let seen = f.fly(5.0, mode, |_| if faults.no_gnss { heading_speed_height(*hdg, 55.0, 300.0) } else { course_speed_height(*hdg, 55.0, 300.0) });
            for i in 0..3 { worst[i] = worst[i].max(seen.max_attitude_error[i]); }
            hmin = hmin.min(seen.min_height); hmax = hmax.max(seen.max_height); imin = imin.min(seen.min_indicated); imax = imax.max(seen.max_indicated);
            let e = f.estimate(); let est = &f.board.kernel().state.estimator;
            let dn = e.position_ned[0].0 - f.plant.pos_ned[0]; let de = e.position_ned[1].0 - f.plant.pos_ned[1];
            let (pn, pe) = (est.p_cov.get(0,0), est.p_cov.get(1,1));
            nees_max = nees_max.max(dn*dn/pn + de*de/pe); poserr = (dn*dn+de*de).sqrt();
        }
        let e = f.estimate(); let est = &f.board.kernel().state.estimator;
        println!("{tag} L{leg} gnss={} att_err_max=({:.2},{:.2},{:.2}) h[{:.1},{:.1}] ias[{:.1},{:.1}] gb_dps=({:.3},{:.3},{:.3}) ab=({:.2},{:.2},{:.2}) wind=({:.1},{:.1}) Pww={:.1} poserr={:.0} sig_pos={:.0} nees2d_max={:.1} flags={:?} mode={:?}",
            !f.gnss_silent, worst[0].to_degrees(), worst[1].to_degrees(), worst[2].to_degrees(), hmin, hmax, imin, imax,
            est.gyro_bias.x.0.to_degrees(), est.gyro_bias.y.0.to_degrees(), est.gyro_bias.z.0.to_degrees(), est.accel_bias.x.0, est.accel_bias.y.0, est.accel_bias.z.0,
            est.air.wind_ne[0], est.air.wind_ne[1], est.air.p_ww[0][0], poserr, est.p_cov.get(0,0).sqrt(), nees_max, e.valid_flags, f.board.last_effective_command().mode);
    }
}
fn trace(tag: &str, faults: Faults) {
    let mut f = Flight::cruising_with(&c172(), PlantModel::C172, DirectNoiseTier::Industrial, DirectSensorRates::default().with_airspeed(40_000), faults);
    f.probe_errors = Some(ProbeErrors::new(3));
    for k in 0..40 {
        let hdg = if k < 20 { 0.0 } else { 1.5708 };
        let seen = f.fly(2.0, ControlMode::AltitudeHold, |_| heading_speed_height(hdg, 55.0, 300.0));
        let e = f.estimate(); let est = &f.board.kernel().state.estimator; let a = f.attitude_error();
        let p = &f.plant;
        let tv = [p.airspeed*p.path_angle.cos()*p.course.cos(), p.airspeed*p.path_angle.cos()*p.course.sin(), -p.airspeed*p.path_angle.sin()];
        println!("{tag} t={:3} att=({:6.2},{:6.2},{:6.2}) max=({:5.2},{:5.2},{:5.2}) gb=({:6.3},{:6.3},{:6.3}) sgb=({:.3},{:.3},{:.3}) satt=({:.2},{:.2},{:.2}) ab=({:5.2},{:5.2},{:5.2}) dv=({:5.1},{:5.1},{:5.1}) w=({:4.1},{:4.1}) h={:.1} bref={:?}",
            (k+1)*2, a[0].to_degrees(), a[1].to_degrees(), a[2].to_degrees(),
            seen.max_attitude_error[0].to_degrees(), seen.max_attitude_error[1].to_degrees(), seen.max_attitude_error[2].to_degrees(),
            est.gyro_bias.x.0.to_degrees(), est.gyro_bias.y.0.to_degrees(), est.gyro_bias.z.0.to_degrees(),
            est.p_cov.get(9,9).sqrt().to_degrees(), est.p_cov.get(10,10).sqrt().to_degrees(), est.p_cov.get(11,11).sqrt().to_degrees(),
            est.p_cov.get(6,6).sqrt().to_degrees(), est.p_cov.get(7,7).sqrt().to_degrees(), est.p_cov.get(8,8).sqrt().to_degrees(),
            est.accel_bias.x.0, est.accel_bias.y.0, est.accel_bias.z.0,
            e.velocity_ned[0].0 - tv[0], e.velocity_ned[1].0 - tv[1], e.velocity_ned[2].0 - tv[2],
            est.air.wind_ne[0], est.air.wind_ne[1], -p.pos_ned[2], est.baro_ref);
    }
}
fn windows(tag: &str, faults: Faults) {
    let mut f = Flight::cruising_with(&c172(), PlantModel::C172, DirectNoiseTier::Industrial, DirectSensorRates::default().with_airspeed(40_000), faults);
    f.probe_errors = Some(ProbeErrors::new(3));
    let headings = [0.0_f32, 1.5708, 3.1416, -1.5708, 0.0, 1.5708];
    let mut line = String::new();
    for (leg, hdg) in headings.iter().enumerate() {
        for half in 0..2 {
            let mut worst = [0.0_f32; 3]; let (mut hmin, mut hmax) = (1e9_f32, -1e9_f32);
            for _ in 0..5 {
                let seen = f.fly(5.0, ControlMode::AltitudeHold, |_| heading_speed_height(*hdg, 55.0, 300.0));
                for i in 0..3 { worst[i] = worst[i].max(seen.max_attitude_error[i]); }
                hmin = hmin.min(seen.min_height); hmax = hmax.max(seen.max_height);
            }
            line += &format!(" | {}{} ({:.1},{:.1},{:.1}) h[{:.0},{:.0}]", leg, if half == 0 {"a"} else {"b"}, worst[0].to_degrees(), worst[1].to_degrees(), worst[2].to_degrees(), hmin, hmax);
        }
    }
    let est = &f.board.kernel().state.estimator;
    println!("{tag}{line} gb=({:.2},{:.2},{:.2}) ab=({:.2},{:.2},{:.2})", est.gyro_bias.x.0.to_degrees(), est.gyro_bias.y.0.to_degrees(), est.gyro_bias.z.0.to_degrees(), est.accel_bias.x.0, est.accel_bias.y.0, est.accel_bias.z.0);
}
#[test]
fn dbg() {
    let b = 0.5_f32.to_radians();
    for mode in [3u8, 4] {
        aviate_core::ekf::scalar::DBG_HEADING_MODE.store(mode, std::sync::atomic::Ordering::Relaxed);
        windows(&format!("W m{mode} bias"), Faults { no_gnss: true, gyro_bias_rps: [b, -b, b] });
        windows(&format!("W m{mode} zero"), Faults { no_gnss: true, gyro_bias_rps: [0.0; 3] });
    }
}
