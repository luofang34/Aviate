//! Outbound MAVLink v2 telemetry formatting for [`super::DirectMavlinkEndpoint`].

use aviate_core::fault::FaultFlags;
use aviate_core::sensor::GeomagneticReference;
use aviate_core::state::StateEstimate;
use aviate_hal_io::RawGnssReading;
use aviate_link::mavlink::protocol::{Heartbeat, SysStatus};
use aviate_link::mavlink::telemetry::{format_geomagnetic_reference, format_scaled_pressure};
use aviate_link::mavlink::{
    format_attitude, format_aviate_estimator_status, format_estimator_status,
    format_global_position, format_gps_raw, format_local_position, MavAutopilot, MavMessage,
    MavModeFlag, MavType,
};
use aviate_link::telemetry::GeodeticOrigin;

use super::{DirectMavlinkEndpoint, MAX_FRAME_BUF};

const ESTIMATE_GROUP_BUF: usize = 512;

const SENSOR_GYRO_3D: u32 = 0x0001;
const SENSOR_ACCEL_3D: u32 = 0x0002;
const SENSOR_MAG_3D: u32 = 0x0004;
const SENSOR_ABS_PRESSURE: u32 = 0x0008;
const SENSOR_GPS: u32 = 0x0020;
const SENSOR_CONTROL_SUITE: u32 = 0x87C0;
const DEFAULT_SENSORS_PRESENT: u32 = SENSOR_GYRO_3D
    | SENSOR_ACCEL_3D
    | SENSOR_MAG_3D
    | SENSOR_ABS_PRESSURE
    | SENSOR_GPS
    | SENSOR_CONTROL_SUITE;

/// Snapshot of vehicle state formatted into outbound MAVLink v2 frames.
pub(crate) struct DirectTelemetryFrameInput<'a> {
    pub sim_time_us: u64,
    pub time_boot_ms: u32,
    pub faults: FaultFlags,
    pub state: &'a StateEstimate,
    pub baro_pressure_pa: Option<f32>,
    pub baro_temperature_c: f32,
    pub geomagnetic: Option<GeomagneticReference>,
    pub geodetic_origin: GeodeticOrigin,
    pub gnss_fix: Option<&'a RawGnssReading>,
}

impl<const RX_CAP: usize, const TX_CAP: usize> DirectMavlinkEndpoint<RX_CAP, TX_CAP> {
    pub(crate) fn emit_telemetry(&mut self, input: &DirectTelemetryFrameInput<'_>) {
        if rate_due(
            &mut self.last_heartbeat_us,
            input.sim_time_us,
            self.rates.heartbeat_hz,
        ) {
            self.emit_heartbeat_and_geomag(input.time_boot_ms, input.geomagnetic.as_ref());
        }
        if rate_due(
            &mut self.last_sys_status_us,
            input.sim_time_us,
            self.rates.sys_status_hz,
        ) {
            self.emit_sys_status(input.faults);
        }
        let emit_att = rate_due(
            &mut self.last_attitude_us,
            input.sim_time_us,
            self.rates.attitude_hz,
        );
        let emit_pos = rate_due(
            &mut self.last_position_us,
            input.sim_time_us,
            self.rates.position_hz,
        );
        let emit_est = rate_due(
            &mut self.last_estimator_status_us,
            input.sim_time_us,
            self.rates.estimator_status_hz,
        );
        if emit_att || emit_pos || emit_est {
            self.emit_estimate_group(input, emit_att, emit_pos);
        }
        if let (true, Some(fix)) = (emit_pos, input.gnss_fix) {
            self.emit_gps_raw(fix, input.time_boot_ms);
        }
    }

    fn emit_gps_raw(&mut self, fix: &RawGnssReading, time_boot_ms: u32) {
        let mut buf = [0_u8; MAX_FRAME_BUF];
        let saved_seq = self.seq;
        if let Ok(len) = format_gps_raw(
            fix,
            time_boot_ms,
            (self.sys_id, self.comp_id),
            &mut self.seq,
            &mut buf,
        ) {
            if let Some(frame) = buf.get(..len) {
                if !self.tx_ring.push_all(frame) {
                    self.seq = saved_seq;
                }
            }
        }
    }

    fn emit_heartbeat_and_geomag(
        &mut self,
        time_boot_ms: u32,
        geomagnetic: Option<&GeomagneticReference>,
    ) {
        let base_mode = if self.armed {
            MavModeFlag::SAFETY_ARMED.0 | MavModeFlag::HIL_ENABLED.0
        } else {
            MavModeFlag::HIL_ENABLED.0
        };
        let hb = Heartbeat {
            mav_type: MavType::Quadrotor as u8,
            autopilot: MavAutopilot::Aviate as u8,
            base_mode,
            custom_mode: 0,
            system_status: self.mav_state as u8,
            mavlink_version: 3,
        };
        self.push_mav_message(&MavMessage::Heartbeat(hb));

        let mut buf = [0_u8; MAX_FRAME_BUF];
        let saved_seq = self.seq;
        if let Ok(len) = format_geomagnetic_reference(
            geomagnetic,
            time_boot_ms,
            (self.sys_id, self.comp_id),
            &mut self.seq,
            &mut buf,
        ) {
            if let Some(frame) = buf.get(..len) {
                if !self.tx_ring.push_all(frame) {
                    self.seq = saved_seq;
                }
            }
        }
    }

    fn emit_sys_status(&mut self, faults: FaultFlags) {
        let mut health = DEFAULT_SENSORS_PRESENT;
        if faults.contains(FaultFlags::ALL_IMU_FAILED) {
            health &= !(SENSOR_GYRO_3D | SENSOR_ACCEL_3D);
        }
        if faults.contains(FaultFlags::MAG_FAILED) {
            health &= !SENSOR_MAG_3D;
        }
        if faults.contains(FaultFlags::BARO_FAILED) {
            health &= !SENSOR_ABS_PRESSURE;
        }
        if faults.contains(FaultFlags::ALL_GNSS_LOST) {
            health &= !SENSOR_GPS;
        }
        let status = SysStatus {
            onboard_control_sensors_present: DEFAULT_SENSORS_PRESENT,
            onboard_control_sensors_enabled: DEFAULT_SENSORS_PRESENT,
            onboard_control_sensors_health: health,
            load: 100,
            voltage_battery: 16_000,
            current_battery: -1,
            battery_remaining: 100,
            ..SysStatus::default()
        };
        self.push_mav_message(&MavMessage::SysStatus(status));
    }

    fn emit_estimate_group(
        &mut self,
        input: &DirectTelemetryFrameInput<'_>,
        emit_attitude: bool,
        emit_position: bool,
    ) {
        let mut group = [0_u8; ESTIMATE_GROUP_BUF];
        let saved_seq = self.seq;
        let Some(mut offset) = self.append_status_pair(input, &mut group) else {
            self.seq = saved_seq;
            return;
        };
        if emit_attitude {
            if let Some(dst) = group.get_mut(offset..) {
                if let Ok(len) = format_attitude(
                    input.state,
                    input.time_boot_ms,
                    self.sys_id,
                    self.comp_id,
                    &mut self.seq,
                    dst,
                ) {
                    offset = offset.wrapping_add(len);
                }
            }
        }
        if emit_position {
            offset = self.append_position_frames(input, &mut group, offset);
        }
        let Some(bytes) = group.get(..offset) else {
            self.seq = saved_seq;
            return;
        };
        if !self.tx_ring.push_all(bytes) {
            self.seq = saved_seq;
        }
    }

    fn append_status_pair(
        &mut self,
        input: &DirectTelemetryFrameInput<'_>,
        group: &mut [u8; ESTIMATE_GROUP_BUF],
    ) -> Option<usize> {
        let n1 = format_estimator_status(
            input.state,
            input.time_boot_ms,
            self.sys_id,
            self.comp_id,
            &mut self.seq,
            group.as_mut_slice(),
        )
        .ok()?;
        let s2 = group.get_mut(n1..)?;
        let n2 = format_aviate_estimator_status(
            input.state,
            input.time_boot_ms,
            self.sys_id,
            self.comp_id,
            &mut self.seq,
            s2,
        )
        .ok()?;
        Some(n1.wrapping_add(n2))
    }

    fn append_position_frames(
        &mut self,
        input: &DirectTelemetryFrameInput<'_>,
        group: &mut [u8; ESTIMATE_GROUP_BUF],
        mut offset: usize,
    ) -> usize {
        let ids = (self.sys_id, self.comp_id);
        if let Some(dst) = group.get_mut(offset..) {
            if let Ok(len) = format_local_position(
                input.state,
                input.time_boot_ms,
                ids.0,
                ids.1,
                &mut self.seq,
                dst,
            ) {
                offset = offset.wrapping_add(len);
            }
        }
        if let Some(dst) = group.get_mut(offset..) {
            if let Ok(len) = format_global_position(
                input.state,
                &input.geodetic_origin,
                input.time_boot_ms,
                ids,
                &mut self.seq,
                dst,
            ) {
                offset = offset.wrapping_add(len);
            }
        }
        if let Some(pressure_pa) = input.baro_pressure_pa {
            if let Some(dst) = group.get_mut(offset..) {
                if let Ok(len) = format_scaled_pressure(
                    pressure_pa,
                    input.baro_temperature_c,
                    input.time_boot_ms,
                    ids.0,
                    ids.1,
                    &mut self.seq,
                    dst,
                ) {
                    offset = offset.wrapping_add(len);
                }
            }
        }
        offset
    }
}

fn rate_due(last_us: &mut Option<u64>, now_us: u64, rate_hz: u8) -> bool {
    if rate_hz == 0 {
        return false;
    }
    let period_us = 1_000_000_u64 / u64::from(rate_hz);
    match *last_us {
        None => {
            *last_us = Some(now_us);
            true
        }
        Some(prev_us) if now_us.saturating_sub(prev_us) >= period_us => {
            *last_us = Some(now_us);
            true
        }
        Some(_) => false,
    }
}
