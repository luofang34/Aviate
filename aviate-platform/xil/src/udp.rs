//! UDP MAVLink SITL HAL
//!
//! Connects to external simulators (jMAVSim, Gazebo, AirSim) via UDP MAVLink.

use std::net::UdpSocket;
use std::io;

use aviate_core::hal::{SensorHal, ActuatorHal, SystemHal, AviateHal, CommandHal, SystemCommand};
use aviate_core::sensor::{
    SensorReading, ImuData, GnssData, BaroData, MagData, SensorHealth, GnssHealth, GnssFix, AirData,
};
use aviate_core::mixer::ActuatorCmd;
use aviate_core::time::{Timestamp, TimeSource};
use aviate_core::types::{
    MetersPerSecondSquared, RadiansPerSecond, Meters, MetersPerSecond, Microtesla, Pascals, Celsius,
};

use aviate_mavlink::{
    parse_mavlink, serialize_mavlink, MavMessage, HilActuatorControls, HilSensor, HilGps,
    Heartbeat, MavAutopilot, MavType, MavState, MavModeFlag, SetAttitudeTarget,
    SetPositionTargetLocalNed, CommandLong, mav_cmd,
};

use crate::{XilConfig, bridge};

// Alias for compatibility
type SitlConfig = XilConfig;

pub struct UdpMavlinkHal {
    recv_socket: UdpSocket,
    send_socket: UdpSocket,
    config: SitlConfig,
    start_time: std::time::Instant,
    armed: bool,
    seq: u8,

    // Buffered sensor data
    imu_data: Option<SensorReading<ImuData>>,
    gnss_data: Option<SensorReading<GnssData>>,
    baro_data: Option<SensorReading<BaroData>>,
    mag_data: Option<SensorReading<MagData>>,
    
    // Buffered command
    command: Option<SystemCommand>,

    // Heartbeat timing
    last_heartbeat_us: u64,

    // Statistics
    rx_count: u64,
    tx_count: u64,
}

impl UdpMavlinkHal {
    pub fn new(config: SitlConfig) -> io::Result<Self> {
        // Socket to receive sensor data from simulator
        let recv_socket = UdpSocket::bind(("0.0.0.0", config.sensor_port))?;
        recv_socket.set_nonblocking(true)?;

        // Socket to send actuator commands to simulator
        let send_socket = UdpSocket::bind("0.0.0.0:0")?; // Bind to any available port
        send_socket.set_nonblocking(true)?;

        Ok(Self {
            recv_socket,
            send_socket,
            config,
            start_time: std::time::Instant::now(),
            armed: false,
            seq: 0,
            imu_data: None,
            gnss_data: None,
            baro_data: None,
            mag_data: None,
            command: None,
            last_heartbeat_us: 0,
            rx_count: 0,
            tx_count: 0,
        })
    }

    /// Poll for incoming MAVLink messages and update sensor data
    pub fn poll(&mut self) {
        let mut buf = [0u8; 512];

        // Process all available messages
        loop {
            match self.recv_socket.recv_from(&mut buf) {
                Ok((len, _src)) => {
                    self.process_mavlink_data(&buf[..len]);
                }
                Err(ref e) if e.kind() == io::ErrorKind::WouldBlock => break,
                Err(_) => break,
            }
        }

        // Send heartbeat at 1 Hz
        let now_us = self.now_us();
        if now_us - self.last_heartbeat_us >= 1_000_000 {
            self.send_heartbeat();
            self.last_heartbeat_us = now_us;
        }
    }

    /// Process received MAVLink data
    fn process_mavlink_data(&mut self, data: &[u8]) {
        // Try to parse MAVLink message
        match parse_mavlink(data) {
            Ok((msg, _consumed)) => {
                self.rx_count += 1;
                self.handle_message(msg);
            }
            Err(_e) => {
                // Silently ignore parse errors (might be partial frames)
            }
        }
    }

    /// Handle a parsed MAVLink message
    fn handle_message(&mut self, msg: MavMessage) {
        let ts = Timestamp { ticks: self.now_us(), source: TimeSource::Internal };

        match msg {
            MavMessage::HilSensor(sensor) => self.handle_hil_sensor(sensor, ts),
            MavMessage::HilGps(gps) => self.handle_hil_gps(gps, ts),
            MavMessage::SetAttitudeTarget(tgt) => self.handle_set_attitude_target(tgt),
            MavMessage::SetPositionTargetLocalNed(tgt) => self.handle_set_position_target(tgt),
            MavMessage::CommandLong(cmd) => self.handle_command_long(cmd),
            _ => {
                // Ignore other messages
            }
        }
    }

    fn handle_hil_sensor(&mut self, sensor: HilSensor, ts: Timestamp) {
        self.imu_data = Some(SensorReading {
            value: ImuData {
                accel: [
                    MetersPerSecondSquared(sensor.xacc),
                    MetersPerSecondSquared(sensor.yacc),
                    MetersPerSecondSquared(sensor.zacc),
                ],
                gyro: [
                    RadiansPerSecond(sensor.xgyro),
                    RadiansPerSecond(sensor.ygyro),
                    RadiansPerSecond(sensor.zgyro),
                ],
            },
            valid: true,
            source_id: sensor.id,
            timestamp: ts,
            health: SensorHealth::Good,
        });

        // Convert pressure to altitude (Standard Atmosphere)
        let pressure_pa = sensor.abs_pressure * 100.0; // mbar to Pa
        let altitude_m = (1.0 - (pressure_pa / 101325.0_f32).powf(0.190284)) * 44330.77;

        self.baro_data = Some(SensorReading {
            value: BaroData {
                altitude: Some(Meters(altitude_m)),
                air: AirData {
                    static_pressure: Some(Pascals(pressure_pa)),
                    dynamic_pressure: None,
                    total_pressure: None,
                    temperature: Some(Celsius(sensor.temperature)),
                    indicated_airspeed: None,
                    true_airspeed: None,
                },
            },
            valid: true,
            source_id: sensor.id,
            timestamp: ts,
            health: SensorHealth::Good,
        });

        self.mag_data = Some(SensorReading {
            value: MagData {
                field_ut: [
                    Microtesla(sensor.xmag * 100.0), // Gauss to µT
                    Microtesla(sensor.ymag * 100.0),
                    Microtesla(sensor.zmag * 100.0),
                ],
            },
            valid: true,
            source_id: sensor.id,
            timestamp: ts,
            health: SensorHealth::Good,
        });
    }

    fn handle_hil_gps(&mut self, gps: HilGps, ts: Timestamp) {
        let fix = match gps.fix_type {
            0 | 1 => GnssFix::None,
            2 => GnssFix::TwoD,
            3 => GnssFix::ThreeD,
            4 => GnssFix::ThreeD, // DGPS
            5 => GnssFix::RtkFloat,
            6 => GnssFix::RtkFixed,
            _ => GnssFix::None,
        };

        let vel_n = MetersPerSecond((gps.vn as f32) / 100.0);
        let vel_e = MetersPerSecond((gps.ve as f32) / 100.0);
        let vel_d = MetersPerSecond((gps.vd as f32) / 100.0);

        // Simplified NED position
        let position_ned = [Meters(0.0), Meters(0.0), Meters(-(gps.alt as f32) / 1000.0)];

        let health = if fix == GnssFix::None { GnssHealth::Lost } else { GnssHealth::Good };

        self.gnss_data = Some(SensorReading {
            value: GnssData {
                position_ned,
                velocity_ned: [vel_n, vel_e, vel_d],
                fix,
                health,
            },
            valid: fix != GnssFix::None,
            source_id: gps.id,
            timestamp: ts,
            health: if health == GnssHealth::Good { SensorHealth::Good } else { SensorHealth::Failed },
        });
    }
    
    fn handle_set_attitude_target(&mut self, tgt: SetAttitudeTarget) {
        let cmd = bridge::mavlink_to_command(&tgt);
        self.command = Some(SystemCommand::FlightControl(cmd));
    }

    fn handle_set_position_target(&mut self, tgt: SetPositionTargetLocalNed) {
        let cmd = bridge::mavlink_position_to_command(&tgt);
        self.command = Some(SystemCommand::FlightControl(cmd));
    }

    fn handle_command_long(&mut self, cmd: CommandLong) {
        if cmd.command == mav_cmd::COMPONENT_ARM_DISARM {
            if cmd.param1 == 1.0 {
                self.command = Some(SystemCommand::Arm);
            } else if cmd.param1 == 0.0 {
                self.command = Some(SystemCommand::Disarm);
            }
        }
    }

    /// Send heartbeat message
    fn send_heartbeat(&mut self) {
        let hb = Heartbeat {
            mav_type: MavType::Quadrotor as u8,
            autopilot: MavAutopilot::Aviate as u8,
            base_mode: if self.armed {
                MavModeFlag::SAFETY_ARMED.0 | MavModeFlag::HIL_ENABLED.0
            } else {
                MavModeFlag::HIL_ENABLED.0
            },
            custom_mode: 0,
            system_status: MavState::Active as u8,
            mavlink_version: 3,
        };

        self.send_message(&MavMessage::Heartbeat(hb));
    }

    /// Send HIL_ACTUATOR_CONTROLS message
    fn send_actuator_controls(&mut self, cmd: &ActuatorCmd) {
        let mut controls = [0.0f32; 16];
        for (i, output) in cmd.outputs.iter().enumerate().take(16) {
            controls[i] = output.0;
        }

        let msg = HilActuatorControls {
            time_usec: self.now_us(),
            controls,
            mode: if self.armed { MavModeFlag::SAFETY_ARMED.0 } else { 0 },
            flags: 0,
        };

        self.send_message(&MavMessage::HilActuatorControls(msg));
    }

    /// Send a MAVLink message
    fn send_message(&mut self, msg: &MavMessage) {
        let mut buf = [0u8; 300];
        if let Some(len) = serialize_mavlink(msg, self.seq, &mut buf) {
            self.seq = self.seq.wrapping_add(1);
            let _ = self.send_socket.send_to(&buf[..len], self.config.simulator_addr);
            self.tx_count += 1;
        }
    }

    fn now_us(&self) -> u64 {
        self.start_time.elapsed().as_micros() as u64
    }

    /// Get statistics
    pub fn stats(&self) -> (u64, u64) {
        (self.rx_count, self.tx_count)
    }
}

impl SensorHal for UdpMavlinkHal {
    fn read_imu(&mut self) -> Option<SensorReading<ImuData>> {
        self.poll();
        self.imu_data.take()
    }

    fn read_gnss(&mut self) -> Option<SensorReading<GnssData>> {
        self.gnss_data.take()
    }

    fn read_baro(&mut self) -> Option<SensorReading<BaroData>> {
        self.baro_data.take()
    }

    fn read_mag(&mut self) -> Option<SensorReading<MagData>> {
        self.mag_data.take()
    }
}

impl ActuatorHal for UdpMavlinkHal {
    fn write(&mut self, cmd: &ActuatorCmd) {
        self.send_actuator_controls(cmd);
    }

    fn arm(&mut self) {
        self.armed = true;
        eprintln!("[INFO] HAL armed");
    }

    fn disarm(&mut self) {
        self.armed = false;
        eprintln!("[INFO] HAL disarmed");
    }

    fn is_armed(&self) -> bool {
        true // SITL always hardware-armed (safety switch)
    }
}

impl SystemHal for UdpMavlinkHal {
    fn now(&self) -> Timestamp {
        Timestamp {
            ticks: self.now_us(),
            source: TimeSource::Internal,
        }
    }

    fn now_us(&self) -> u64 {
        self.start_time.elapsed().as_micros() as u64
    }

    fn delay_us(&self, us: u32) {
        std::thread::sleep(std::time::Duration::from_micros(us as u64));
    }

    fn kick_watchdog(&mut self) {}

    fn reboot(&mut self) -> ! {
        eprintln!("[INFO] Reboot requested");
        std::process::exit(0);
    }

    fn enter_bootloader(&mut self) -> ! {
        eprintln!("[WARN] Bootloader not supported in SITL");
        std::process::exit(1);
    }
}

impl CommandHal for UdpMavlinkHal {
    fn recv_command(&mut self) -> Option<SystemCommand> {
        self.poll(); 
        self.command.take()
    }
}

impl AviateHal for UdpMavlinkHal {}
