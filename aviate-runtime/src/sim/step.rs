//! The `SitlRunner::step()` per-cycle method.
//!
//! Extracted from `sim.rs` to keep that file under the 500-line cap.
//! Just an `impl SitlRunner` block split across files — no re-exports
//! (sidesteps rustc's coverage phantom-DA issue documented on the
//! control.rs split).

use log::{info, warn};

use super::{init_state_to_mav_state, SitlRunner};
use crate::telemetry::TelemetrySnapshot;

use aviate_core::hal::{ActuatorHal, SensorHal, SystemHal};
use aviate_core::math::{Quaternion, Vector3};
use aviate_core::mixer::ActuatorCmd;
use aviate_core::time::TimeDelta;
use aviate_core::types::{Meters, MetersPerSecond, Seconds};
use aviate_core::ChannelId;
use aviate_hal_io::{CommandHal, SystemCommand};
use aviate_hal_xil::SimActuatorCmd;

impl<C, M> SitlRunner<C, M>
where
    C: aviate_core::control::VehicleController,
    M: aviate_core::mixer::Mixer,
{
    /// Step the flight controller (extracted from GazeboSitlBoard::step)
    ///
    /// This is the ~165 lines of stepping logic that was duplicated across SITL boards.
    ///
    /// ## Steps:
    /// 1. Poll transport for incoming messages
    /// 2. Feed fake sensors with HIL data (via BoardHal accessors)
    /// 3. Read sensors via BoardHal's SensorHal implementation
    /// 4. Calculate dt from IMU timestamps
    /// 5. Cache sensor readings for EKF init
    /// 6. Receive commands via transport
    /// 7. Initialize EKF once we have sensor data (one-time)
    /// 8. Run kernel initialization state machine
    /// 9. Step kernel with sensor data and commands
    /// 10. Write actuator outputs via BoardHal
    /// 11. Forward actuator commands to simulator
    /// 12. Kick watchdog
    pub fn step(&mut self) -> ActuatorCmd {
        // 1. Poll transport for incoming messages
        self.transport.poll();

        // 1b. Poll the fault command listener (if bound). Inbound
        //     `FaultCommand`s are applied to the fake sensors here,
        //     BEFORE we feed them new HIL data — so a NaN-inject
        //     command takes effect on the very next sensor read,
        //     not the one after.
        if let Some(ref mut fault_ctrl) = self.fault_ctrl {
            let (imu, baro, mag, gnss) = self.board_hal.sensors_mut();
            let mut sensors = aviate_hal_xil::FaultSensors {
                imu,
                baro,
                mag,
                gnss,
            };
            fault_ctrl.poll(&mut sensors);
        }

        // 2. Feed fake sensors with HIL data (via BoardHal accessors)
        //    This is the key integration point - same pattern as real HW feeding real sensors
        if let Some(sensor_data) = self.transport.take_sensor_data() {
            // Feed IMU
            self.board_hal.imu_mut().feed(sensor_data.imu);
            // Feed Baro
            self.board_hal.baro_mut().feed(sensor_data.baro);
            // Feed Mag
            self.board_hal.mag_mut().feed(sensor_data.mag);
        }

        if let Some(gps_data) = self.transport.take_gps_data() {
            // Feed GNSS
            self.board_hal.gnss_mut().feed(gps_data.gnss);
        }

        // 3. Read sensors via BoardHal's SensorHal implementation
        //    This is the SAME code path that real hardware uses!
        let mut current_dt = 0.001;
        let mut current_delta_us = 1000u64;

        if let Some(imu) = self.board_hal.read_imu() {
            let current_time = imu.timestamp.ticks;
            let delta_us_val = if let Some(last) = self.last_imu_time {
                current_time.saturating_sub(last)
            } else {
                1000
            };
            current_dt = (delta_us_val as f32) * 1e-6;
            current_delta_us = delta_us_val;
            self.last_imu_time = Some(current_time);
            current_dt = current_dt.clamp(0.0001, 0.1);
            self.sensor_cache.imu = Some(imu);
        }

        if let Some(gnss) = self.board_hal.read_gnss() {
            self.sensor_cache.gnss = Some(gnss);
        }

        if let Some(baro) = self.board_hal.read_baro() {
            self.sensor_cache.baro = Some(baro);
        }

        if let Some(mag) = self.board_hal.read_mag() {
            self.sensor_cache.mag = Some(mag);
        }

        let time_delta = TimeDelta {
            dt_sec: Seconds(current_dt),
            tick_delta: current_delta_us,
        };

        // 4. Receive commands via transport, routed through the same
        // ingress state machine the hardware runner uses (#133).
        // Discrete Arm/Disarm fire exactly once and never refresh the
        // setpoint age; an armed vehicle with no commander is
        // command-stale by definition — the CommandLoss terminal
        // engages (safe: zero-collective on the ground) and releases
        // on the first real setpoint per LLR-FLT-209. The old "anchor
        // freshness on arm" hack was exactly the #133 defect.
        if let Some(sys_cmd) = self.transport.recv_command() {
            let now_ticks = self.transport.now().ticks;
            if let SystemCommand::FlightControl(cmd) = &sys_cmd {
                self.kernel
                    .state
                    .checks
                    .pre_arm
                    // Throttle-low gate: commanded collective below 1 % of maximum
                    // thrust (force domain).
                    .update_throttle(cmd.setpoint.collective_thrust.0 < 0.01);
            }
            match &sys_cmd {
                SystemCommand::Arm => {
                    info!("Arm command (state={:?})", self.kernel.state.init_state);
                    info!("Faults: {:?}", self.kernel.state.faults);
                    if let Err(e) = self.kernel.arm() {
                        let pre_arm = &self.kernel.state.checks.pre_arm;
                        warn!("Arming failed: {:?}", e);
                        warn!("Missing pre-arm: {:?}", pre_arm.missing());
                        warn!("Faults: {:?}", self.kernel.state.faults);
                    } else {
                        info!("Armed successfully");
                        // Only arm HAL and transport if kernel arm succeeded
                        self.board_hal.arm();
                        self.transport.set_armed(true);
                    }
                }
                SystemCommand::Disarm => {
                    info!("Disarm command");
                    self.kernel.disarm();
                    // Disarm through BoardHal and notify transport
                    self.board_hal.disarm();
                    self.transport.set_armed(false);
                }
                SystemCommand::FlightControl(_) => {}
            }
            self.ingress.receive(sys_cmd, now_ticks);
        }

        // 4b. Sync Telemetry target address from SitlIO
        //     SitlIO handles incoming MAVLink and learns the GCS address (e.g. gcs-test ephemeral port).
        //     We must update the TelemetryTask to send data to that address.
        if let Some(ref mut telem) = self.telemetry {
            if let Some(addr) = self.transport.gcs_addr() {
                telem.frame_tx_mut().set_addr(addr);
            }
        }

        // 5. Initialize EKF once we have sensor data.
        //
        // Seed the attitude estimate from the first IMU sample via
        // a TRIAD-style closed form: the gravity vector points
        // straight down in NED, so the IMU's specific-force vector
        // (which points up against gravity at rest) tells us the
        // body's tilt directly. Yaw is unobservable from accel
        // alone — leave it at zero and let the mag update refine
        // it. Initializing close to the truth avoids the cold-start
        // attitude transient that otherwise wrestles with the
        // closed-loop controller during takeoff and saturates
        // motors against an EKF that lags reality.
        if !self.ekf_initialized
            && self.sensor_cache.imu.is_some()
            && self.sensor_cache.mag.is_some()
        {
            info!("Initializing EKF with sensor data");
            // TRIAD-style: roll & pitch come from the gravity
            // vector (specific force at rest), yaw from the mag
            // heading after tilt compensation. Seeding yaw to zero
            // and letting the mag pull it later confuses the
            // Kalman update — the correction routes through the
            // attitude/gyro-bias correlation block, builds a
            // phantom gyro bias, and the predict step then
            // integrates a phantom yaw rate that physically yaws
            // the vehicle once the cascade is closed.
            let init_quat = {
                let imu = self
                    .sensor_cache
                    .imu
                    .as_ref()
                    .map(|s| s.value)
                    .unwrap_or_default();
                let mag = self
                    .sensor_cache
                    .mag
                    .as_ref()
                    .map(|s| s.value)
                    .unwrap_or_default();
                triad_init_quat(
                    [imu.accel[0].0, imu.accel[1].0, imu.accel[2].0],
                    [mag.field_ut[0].0, mag.field_ut[1].0, mag.field_ut[2].0],
                )
            };
            self.kernel.state.estimator.init(
                Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
                Vector3::new(
                    MetersPerSecond(0.0),
                    MetersPerSecond(0.0),
                    MetersPerSecond(0.0),
                ),
                init_quat,
            );
            self.ekf_initialized = true;
        }

        // 6. Run init state machine
        let sensors = self.sensor_cache.to_sensor_set();
        if !self.kernel.is_ready() {
            let ts = self.transport.now();
            let prev_state = self.kernel.state.init_state;
            self.kernel.init_step(&sensors, ts);

            // Log state transitions and update MAVLink system status
            if self.kernel.state.init_state != prev_state {
                info!(
                    "Init state: {:?} -> {:?}",
                    prev_state, self.kernel.state.init_state
                );
                // Update MAVLink system_status based on init state
                let mav_state = init_state_to_mav_state(self.kernel.state.init_state);
                self.transport.set_system_status(mav_state);
            }
        }

        // 7. Step kernel.
        //
        // Setpoint age comes from the shared ingress: only a
        // FlightControl receive refreshes it; u32::MAX before the
        // first so a fresh boot is command-stale (failsafe posture).
        let now_ticks = self.transport.now().ticks;
        let command_age_ms = self.ingress.setpoint_age_ms(now_ticks);
        let flight_cmd: aviate_core::control::Command = match self.ingress.setpoint() {
            Some(aviate_hal_io::SystemCommand::FlightControl(c)) => c.clone(),
            _ => crate::sim::default_command(),
        };
        let result = self.kernel.update(
            ChannelId(0),
            time_delta,
            &sensors,
            &flight_cmd,
            command_age_ms,
            &aviate_core::mixer::ActuatorState::default(),
            None,
        );
        let actuator_cmd = result.actuator.clone();

        // 8. Write outputs via BoardHal (ActuatorHal implementation)
        //    This writes to FakeActuator, same path as real hardware
        self.board_hal.write(&actuator_cmd);

        // 9. Forward actuator command to simulator
        //    Take command from FakeActuator and set for backend to retrieve
        if let Some(raw_cmd) = self.board_hal.actuator_mut().take_cmd() {
            let sim_cmd = SimActuatorCmd {
                timestamp_us: self.transport.now_us(),
                outputs: raw_cmd.outputs,
                count: raw_cmd.count,
                armed: self.is_armed(),
            };
            self.transport.set_actuator_cmd(sim_cmd);
        }

        // 10. Update telemetry snapshot (HIGH-DAL: trivial field copies only)
        self.iteration = self.iteration.wrapping_add(1);
        let time_ms = (self.transport.now_us() / 1000) as u32;
        if let Some(ref mut telem) = self.telemetry {
            let snapshot = TelemetrySnapshot {
                time_ms,
                iteration: self.iteration,
                status: result.status,
                state: result.estimate,
            };
            telem.update_state(snapshot); // Just copies, easy to audit
        }

        // 11. Format + queue + send telemetry (LOW-DAL: MAVLink formatting, I/O)
        if let Some(ref mut telem) = self.telemetry {
            telem.tick_and_flush(); // All MAVLink work happens here
        }

        // 12. Watchdog
        self.transport.kick_watchdog();

        actuator_cmd
    }
}

/// Seed the EKF attitude quaternion from a single body-frame
/// accelerometer + magnetometer sample (TRIAD).
///
/// The accel vector at rest points opposite to NED gravity, so it
/// fixes roll and pitch. The mag vector resolves the remaining
/// rotation about the gravity axis (yaw). Seeding all three angles
/// at init avoids the cold-start scenario where a mag update on a
/// stationary vehicle routes its correction through the
/// attitude/gyro-bias covariance block and produces a phantom
/// gyro bias — which the predict step then integrates into real
/// vehicle motion once the rate loop is closed.
fn triad_init_quat(accel_body: [f32; 3], mag_body: [f32; 3]) -> Quaternion {
    use aviate_core::math::Vector3 as V3;
    let ax = accel_body[0];
    let ay = accel_body[1];
    let az = accel_body[2];
    let a_mag = (ax * ax + ay * ay + az * az).sqrt();
    if !(a_mag.is_finite()) || a_mag < 1.0 {
        return Quaternion::IDENTITY;
    }
    let mx = mag_body[0];
    let my = mag_body[1];
    let mz = mag_body[2];
    let m_mag = (mx * mx + my * my + mz * mz).sqrt();
    if !(m_mag.is_finite()) || m_mag < 1.0 {
        return Quaternion::IDENTITY;
    }

    // Body axes in world (NED) frame, derived from the two
    // observed vectors:
    //   z_world_in_body = direction of NED gravity ≈ -accel/|accel|
    //   n_world_in_body = horizontal component of mag, orthogonal to z
    let z_body = V3::new(-ax / a_mag, -ay / a_mag, -az / a_mag);
    let m_body = V3::new(mx / m_mag, my / m_mag, mz / m_mag);
    // Project mag onto plane perpendicular to z_body (remove
    // vertical component). What remains is the horizontal mag
    // direction = NED north in body frame.
    let dot_mz = m_body.x * z_body.x + m_body.y * z_body.y + m_body.z * z_body.z;
    let n_body = V3::new(
        m_body.x - dot_mz * z_body.x,
        m_body.y - dot_mz * z_body.y,
        m_body.z - dot_mz * z_body.z,
    );
    let n_norm = (n_body.x * n_body.x + n_body.y * n_body.y + n_body.z * n_body.z).sqrt();
    if !(n_norm.is_finite()) || n_norm < 1e-6 {
        return Quaternion::IDENTITY;
    }
    let n_body = V3::new(n_body.x / n_norm, n_body.y / n_norm, n_body.z / n_norm);
    // East in body = D × N (right-hand rule for NED: N × E = D ⇒ E = D × N).
    let e_body = V3::new(
        z_body.y * n_body.z - z_body.z * n_body.y,
        z_body.z * n_body.x - z_body.x * n_body.z,
        z_body.x * n_body.y - z_body.y * n_body.x,
    );

    // Rotation matrix R such that v_world = R · v_body. Its
    // ROWS are the world axes expressed in body frame
    // (R_ij = projection of e_i_world onto e_j_body).
    let r00 = n_body.x;
    let r01 = n_body.y;
    let r02 = n_body.z;
    let r10 = e_body.x;
    let r11 = e_body.y;
    let r12 = e_body.z;
    let r20 = z_body.x;
    let r21 = z_body.y;
    let r22 = z_body.z;

    // Shepperd's method: pick the most numerically stable branch
    // by selecting the largest diagonal magnitude. Avoids the
    // gimbal-pole singularity that the direct trace formula has.
    let trace = r00 + r11 + r22;
    let (w, x, y, z) = if trace > 0.0 {
        let s = (trace + 1.0).sqrt() * 2.0;
        (0.25 * s, (r21 - r12) / s, (r02 - r20) / s, (r10 - r01) / s)
    } else if r00 > r11 && r00 > r22 {
        let s = (1.0 + r00 - r11 - r22).sqrt() * 2.0;
        ((r21 - r12) / s, 0.25 * s, (r01 + r10) / s, (r02 + r20) / s)
    } else if r11 > r22 {
        let s = (1.0 + r11 - r00 - r22).sqrt() * 2.0;
        ((r02 - r20) / s, (r01 + r10) / s, 0.25 * s, (r12 + r21) / s)
    } else {
        let s = (1.0 + r22 - r00 - r11).sqrt() * 2.0;
        ((r10 - r01) / s, (r02 + r20) / s, (r12 + r21) / s, 0.25 * s)
    };
    let n = (w * w + x * x + y * y + z * z).sqrt();
    if !(n.is_finite()) || n < 1e-9 {
        return Quaternion::IDENTITY;
    }
    Quaternion::new(w / n, x / n, y / n, z / n)
}
