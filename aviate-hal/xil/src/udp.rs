//! UDP MAVLink SITL HAL
//!
//! Connects to external simulators (jMAVSim, Gazebo, AirSim) via UDP MAVLink.
//! Uses fake sensors from aviate-hal-io to provide a unified sensor interface.

use std::io;
use std::net::UdpSocket;

use aviate_core::hal::{ActuatorHal, AviateHal, CommandHal, SensorHal, SystemCommand, SystemHal};
use aviate_core::mixer::ActuatorCmd;
use aviate_core::sensor::{
    AirData, BaroData, GnssData, GnssFix, GnssHealth, ImuData, MagData, SensorHealth, SensorReading,
};
use aviate_core::time::{TimeSource, Timestamp};
use aviate_core::types::{
    Celsius, Meters, MetersPerSecond, MetersPerSecondSquared, Microtesla, Pascals, RadiansPerSecond,
};

use aviate_hal_io::{
    BaroDriver, FakeBaro, FakeGnss, FakeImu, FakeMag, GnssDriver, GnssFix as EmbeddedGnssFix,
    ImuDriver, MagDriver, RawBaroReading, RawGnssReading, RawImuReading, RawMagReading,
};

use aviate_mavlink::{
    mav_cmd, parse_mavlink, serialize_mavlink, CommandLong, Heartbeat, HilActuatorControls, HilGps,
    HilSensor, MavAutopilot, MavMessage, MavModeFlag, MavState, MavType, SetAttitudeTarget,
    SetPositionTargetLocalNed,
};

use crate::{bridge, XilConfig};

// Alias for compatibility
type SitlConfig = XilConfig;

/// UDP MAVLink HAL using fake sensors
///
/// This HAL connects to external simulators via UDP MAVLink and uses
/// fake sensor drivers to provide sensor data. The fake sensors can be
/// accessed directly for custom setups, or the HAL implements SensorHal
/// for standard usage.
pub struct UdpMavlinkHal {
    recv_socket: UdpSocket,
    send_socket: UdpSocket,
    config: SitlConfig,
    start_time: std::time::Instant,
    armed: bool,
    seq: u8,

    // Fake sensors (same interface as real hardware)
    fake_imu: FakeImu,
    fake_baro: FakeBaro,
    fake_mag: FakeMag,
    fake_gnss: FakeGnss,

    // Buffered command
    command: Option<SystemCommand>,

    // Heartbeat timing
    last_heartbeat_us: u64,

    // GCS/client address (for responding to commands)
    gcs_addr: Option<std::net::SocketAddr>,

    // Statistics
    rx_count: u64,
    tx_count: u64,
}

impl UdpMavlinkHal {
    pub fn new(config: SitlConfig) -> io::Result<Self> {
        // Socket to receive sensor data from simulator
        let recv_socket = UdpSocket::bind(("0.0.0.0", config.sensor_port()))?;
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
            fake_imu: FakeImu::new(),
            fake_baro: FakeBaro::new(),
            fake_mag: FakeMag::new(),
            fake_gnss: FakeGnss::new(),
            command: None,
            last_heartbeat_us: 0,
            gcs_addr: None,
            rx_count: 0,
            tx_count: 0,
        })
    }

    /// Get a reference to the fake IMU
    pub fn fake_imu(&self) -> &FakeImu {
        &self.fake_imu
    }

    /// Get a mutable reference to the fake IMU
    pub fn fake_imu_mut(&mut self) -> &mut FakeImu {
        &mut self.fake_imu
    }

    /// Get a reference to the fake barometer
    pub fn fake_baro(&self) -> &FakeBaro {
        &self.fake_baro
    }

    /// Get a mutable reference to the fake barometer
    pub fn fake_baro_mut(&mut self) -> &mut FakeBaro {
        &mut self.fake_baro
    }

    /// Get a reference to the fake magnetometer
    pub fn fake_mag(&self) -> &FakeMag {
        &self.fake_mag
    }

    /// Get a mutable reference to the fake magnetometer
    pub fn fake_mag_mut(&mut self) -> &mut FakeMag {
        &mut self.fake_mag
    }

    /// Get a reference to the fake GNSS
    pub fn fake_gnss(&self) -> &FakeGnss {
        &self.fake_gnss
    }

    /// Get a mutable reference to the fake GNSS
    pub fn fake_gnss_mut(&mut self) -> &mut FakeGnss {
        &mut self.fake_gnss
    }

    /// Poll for incoming MAVLink messages and update sensor data
    pub fn poll(&mut self) {
        let mut buf = [0u8; 512];

        // Process all available messages
        loop {
            match self.recv_socket.recv_from(&mut buf) {
                Ok((len, src)) => {
                    self.process_mavlink_data(&buf[..len], src);
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
    fn process_mavlink_data(&mut self, data: &[u8], src: std::net::SocketAddr) {
        // Try to parse MAVLink message
        match parse_mavlink(data) {
            Ok((msg, _consumed)) => {
                self.rx_count += 1;
                self.handle_message(msg, src);
            }
            Err(_e) => {
                // Silently ignore parse errors (might be partial frames)
            }
        }
    }

    /// Handle a parsed MAVLink message
    fn handle_message(&mut self, msg: MavMessage, src: std::net::SocketAddr) {
        match msg {
            MavMessage::HilSensor(sensor) => self.handle_hil_sensor(sensor),
            MavMessage::HilGps(gps) => self.handle_hil_gps(gps),
            MavMessage::SetAttitudeTarget(tgt) => {
                // Track GCS address for response
                self.gcs_addr = Some(src);
                self.handle_set_attitude_target(tgt);
            }
            MavMessage::SetPositionTargetLocalNed(tgt) => {
                self.gcs_addr = Some(src);
                self.handle_set_position_target(tgt);
            }
            MavMessage::CommandLong(cmd) => {
                self.gcs_addr = Some(src);
                self.handle_command_long(cmd);
            }
            MavMessage::Heartbeat(_) => {
                // Track GCS address when receiving heartbeats from GCS
                self.gcs_addr = Some(src);
            }
            _ => {
                // Ignore other messages
            }
        }
    }

    fn handle_hil_sensor(&mut self, sensor: HilSensor) {
        // Feed IMU data to fake sensor
        self.fake_imu.feed(RawImuReading {
            accel: [sensor.xacc, sensor.yacc, sensor.zacc],
            gyro: [sensor.xgyro, sensor.ygyro, sensor.zgyro],
            temperature: Some(sensor.temperature),
        });

        // Feed barometer data (convert mbar to Pa)
        self.fake_baro.feed(RawBaroReading {
            pressure_pa: sensor.abs_pressure * 100.0,
            temperature_c: sensor.temperature,
        });

        // Feed magnetometer data (convert Gauss to µT)
        self.fake_mag.feed(RawMagReading {
            field_ut: [
                sensor.xmag * 100.0,
                sensor.ymag * 100.0,
                sensor.zmag * 100.0,
            ],
        });
    }

    fn handle_hil_gps(&mut self, gps: HilGps) {
        let fix = match gps.fix_type {
            0 | 1 => EmbeddedGnssFix::None,
            2 => EmbeddedGnssFix::TwoD,
            3 | 4 => EmbeddedGnssFix::ThreeD,
            5 => EmbeddedGnssFix::RtkFloat,
            6 => EmbeddedGnssFix::RtkFixed,
            _ => EmbeddedGnssFix::None,
        };

        self.fake_gnss.feed(RawGnssReading {
            lat_deg: (gps.lat as f64) / 1e7,
            lon_deg: (gps.lon as f64) / 1e7,
            alt_m: (gps.alt as f32) / 1000.0,
            vel_ned: [
                (gps.vn as f32) / 100.0,
                (gps.ve as f32) / 100.0,
                (gps.vd as f32) / 100.0,
            ],
            fix,
            h_acc: (gps.eph as f32) / 100.0,
            v_acc: (gps.epv as f32) / 100.0,
            satellites: gps.satellites_visible,
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

        // Send to simulator
        self.send_message(&MavMessage::Heartbeat(hb));

        // Also send to GCS/client if known
        if let Some(gcs_addr) = self.gcs_addr {
            self.send_message_to(&MavMessage::Heartbeat(hb), gcs_addr);
        }
    }

    /// Send a MAVLink message to a specific address
    fn send_message_to(&mut self, msg: &MavMessage, addr: std::net::SocketAddr) {
        let mut buf = [0u8; 300];
        if let Some(len) = serialize_mavlink(msg, self.seq, &mut buf) {
            self.seq = self.seq.wrapping_add(1);
            let _ = self.send_socket.send_to(&buf[..len], addr);
            self.tx_count += 1;
        }
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
            mode: if self.armed {
                MavModeFlag::SAFETY_ARMED.0
            } else {
                0
            },
            flags: 0,
        };

        self.send_message(&MavMessage::HilActuatorControls(msg));
    }

    /// Send a MAVLink message
    fn send_message(&mut self, msg: &MavMessage) {
        let mut buf = [0u8; 300];
        if let Some(len) = serialize_mavlink(msg, self.seq, &mut buf) {
            self.seq = self.seq.wrapping_add(1);
            let _ = self
                .send_socket
                .send_to(&buf[..len], self.config.simulator_addr());
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

    // Helper to create timestamp
    fn timestamp(&self) -> Timestamp {
        Timestamp {
            ticks: self.now_us(),
            source: TimeSource::Internal,
        }
    }
}

// Implement SensorHal by reading from fake sensors
// This provides the same interface as real hardware
impl SensorHal for UdpMavlinkHal {
    fn read_imu(&mut self) -> Option<SensorReading<ImuData>> {
        self.poll();

        // Check if fake sensor has data
        if !self.fake_imu.has_data() {
            return None;
        }

        let ts = self.timestamp();

        match self.fake_imu.read() {
            Ok(raw) => Some(SensorReading {
                value: ImuData {
                    accel: [
                        MetersPerSecondSquared(raw.accel[0]),
                        MetersPerSecondSquared(raw.accel[1]),
                        MetersPerSecondSquared(raw.accel[2]),
                    ],
                    gyro: [
                        RadiansPerSecond(raw.gyro[0]),
                        RadiansPerSecond(raw.gyro[1]),
                        RadiansPerSecond(raw.gyro[2]),
                    ],
                },
                valid: true,
                source_id: self.fake_imu.source_id(),
                timestamp: ts,
                health: SensorHealth::Good,
            }),
            Err(_) => None,
        }
    }

    fn read_gnss(&mut self) -> Option<SensorReading<GnssData>> {
        if !self.fake_gnss.has_data() {
            return None;
        }

        let ts = self.timestamp();

        match self.fake_gnss.read() {
            Ok(raw) => {
                let fix = match raw.fix {
                    EmbeddedGnssFix::None => GnssFix::None,
                    EmbeddedGnssFix::TwoD => GnssFix::TwoD,
                    EmbeddedGnssFix::ThreeD => GnssFix::ThreeD,
                    EmbeddedGnssFix::RtkFloat => GnssFix::RtkFloat,
                    EmbeddedGnssFix::RtkFixed => GnssFix::RtkFixed,
                };

                let health = if raw.fix == EmbeddedGnssFix::None {
                    GnssHealth::Lost
                } else {
                    GnssHealth::Good
                };

                Some(SensorReading {
                    value: GnssData {
                        position_ned: [Meters(0.0), Meters(0.0), Meters(-raw.alt_m)],
                        velocity_ned: [
                            MetersPerSecond(raw.vel_ned[0]),
                            MetersPerSecond(raw.vel_ned[1]),
                            MetersPerSecond(raw.vel_ned[2]),
                        ],
                        fix,
                        health,
                    },
                    valid: raw.fix != EmbeddedGnssFix::None,
                    source_id: self.fake_gnss.source_id(),
                    timestamp: ts,
                    health: if health == GnssHealth::Good {
                        SensorHealth::Good
                    } else {
                        SensorHealth::Failed
                    },
                })
            }
            Err(_) => None,
        }
    }

    fn read_baro(&mut self) -> Option<SensorReading<BaroData>> {
        if !self.fake_baro.has_data() {
            return None;
        }

        let ts = self.timestamp();

        match self.fake_baro.read() {
            Ok(raw) => {
                let altitude_m = raw.altitude_m();

                Some(SensorReading {
                    value: BaroData {
                        altitude: Some(Meters(altitude_m)),
                        air: AirData {
                            static_pressure: Some(Pascals(raw.pressure_pa)),
                            dynamic_pressure: None,
                            total_pressure: None,
                            temperature: Some(Celsius(raw.temperature_c)),
                            indicated_airspeed: None,
                            true_airspeed: None,
                        },
                    },
                    valid: true,
                    source_id: self.fake_baro.source_id(),
                    timestamp: ts,
                    health: SensorHealth::Good,
                })
            }
            Err(_) => None,
        }
    }

    fn read_mag(&mut self) -> Option<SensorReading<MagData>> {
        if !self.fake_mag.has_data() {
            return None;
        }

        let ts = self.timestamp();

        match self.fake_mag.read() {
            Ok(raw) => Some(SensorReading {
                value: MagData {
                    field_ut: [
                        Microtesla(raw.field_ut[0]),
                        Microtesla(raw.field_ut[1]),
                        Microtesla(raw.field_ut[2]),
                    ],
                },
                valid: true,
                source_id: self.fake_mag.source_id(),
                timestamp: ts,
                health: SensorHealth::Good,
            }),
            Err(_) => None,
        }
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
