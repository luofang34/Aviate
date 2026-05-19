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

impl SitlRunner {
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

        // 4. Receive commands via transport
        if let Some(sys_cmd) = self.transport.recv_command() {
            match sys_cmd {
                SystemCommand::FlightControl(cmd) => {
                    self.kernel
                        .state
                        .checks
                        .pre_arm
                        .update_throttle(cmd.setpoint.collective_thrust.0 < 0.1);
                    self.last_cmd = cmd;
                    self.last_cmd_rx_ticks = Some(self.transport.now().ticks);
                }
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
            }
        }

        // 4b. Sync Telemetry target address from SitlIO
        //     SitlIO handles incoming MAVLink and learns the GCS address (e.g. gcs-test ephemeral port).
        //     We must update the TelemetryTask to send data to that address.
        if let Some(ref mut telem) = self.telemetry {
            if let Some(addr) = self.transport.gcs_addr() {
                telem.frame_tx_mut().set_addr(addr);
            }
        }

        // 5. Initialize EKF once we have sensor data
        if !self.ekf_initialized && self.sensor_cache.imu.is_some() {
            info!("Initializing EKF with sensor data");
            self.kernel.state.estimator.init(
                Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
                Vector3::new(
                    MetersPerSecond(0.0),
                    MetersPerSecond(0.0),
                    MetersPerSecond(0.0),
                ),
                Quaternion::IDENTITY,
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
        // command_age_ms is measured from the most recent
        // SystemCommand::FlightControl arrival (last_cmd_rx_ticks)
        // against the transport's microsecond clock. Until the
        // first command lands the runner clamps to u32::MAX so the
        // kernel's command-timeout check fires immediately —
        // failsafe behavior, not COMMAND_RECENT-by-default.
        let command_age_ms = match self.last_cmd_rx_ticks {
            Some(rx_ticks) => {
                let now_ticks = self.transport.now().ticks;
                let age_us = now_ticks.saturating_sub(rx_ticks);
                u32::try_from(age_us / 1_000).unwrap_or(u32::MAX)
            }
            None => u32::MAX,
        };
        let result = self.kernel.update(
            ChannelId(0),
            time_delta,
            &sensors,
            &self.last_cmd,
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
