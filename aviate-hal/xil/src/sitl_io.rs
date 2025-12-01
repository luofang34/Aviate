//! SITL Transport Layer
//!
//! Handles network communication between the flight controller and simulator.
//! This is the transport layer for SITL - it moves data between the FC and simulator,
//! but does NOT implement HAL traits. HAL abstraction lives in `aviate-hal-io`.
//!
//! ## Responsibilities
//!
//! - **Sensor input**: Receives sensor data from simulator, buffers for BoardHal
//! - **Actuator output**: Sends actuator commands to simulator
//! - **Command input**: Receives arm/disarm and setpoint commands
//! - **Heartbeat**: Maintains connection with simulator
//!
//! ## Architecture
//!
//! ```text
//! ┌─────────────────────────────────────────────────────────────────────────┐
//! │                            aviate-hal-io                                │
//! │  BoardHal<I,B,M,G,T,A> implements SensorHal + ActuatorHal              │
//! │  - FakeImu, FakeBaro, FakeMag, FakeGnss (sensor drivers)              │
//! │  - FakeActuator (actuator driver)                                      │
//! └─────────────────────────────────────────────────────────────────────────┘
//!                    ↑ feed()                      ↓ take_cmd()
//! ┌─────────────────────────────────────────────────────────────────────────┐
//! │                     aviate-hal-xil (this module)                        │
//! │  SitlIO - Transport layer using MAVLink/UDP                            │
//! │  - take_sensor_data() → feeds fake sensors                             │
//! │  - send_actuator() ← reads from fake actuator                          │
//! └─────────────────────────────────────────────────────────────────────────┘
//!                    ↑ UDP                         ↓ UDP
//! ┌─────────────────────────────────────────────────────────────────────────┐
//! │                          Simulator (Gazebo)                             │
//! │  HIL_SENSOR, HIL_GPS → sensor data                                     │
//! │  HIL_ACTUATOR_CONTROLS ← actuator commands                             │
//! └─────────────────────────────────────────────────────────────────────────┘
//! ```
//!
//! ## Note on MAVLink
//!
//! MAVLink is used as the wire protocol because it's the standard for HIL simulation.
//! However, the actuator and sensor abstractions in `aviate-hal-io` are completely
//! transport-agnostic. A different transport (e.g., shared memory, custom UDP) could
//! be used by implementing a different transport layer.

use std::io;
use std::net::UdpSocket;

use aviate_core::hal::{CommandHal, SystemCommand, SystemHal};
use aviate_core::time::{TimeSource, Timestamp};

use aviate_hal_io::{
    GnssFix, RawActuatorCmd, RawBaroReading, RawGnssReading, RawImuReading, RawMagReading,
};

use aviate_mavlink::{
    mav_cmd, parse_mavlink, serialize_mavlink, CommandLong, Heartbeat, HilActuatorControls, HilGps,
    HilSensor, MavAutopilot, MavMessage, MavModeFlag, MavState, MavType, SetAttitudeTarget,
    SetPositionTargetLocalNed,
};

use crate::{bridge, XilConfig};

/// Raw sensor data from HIL_SENSOR message
#[derive(Debug, Clone, Default)]
pub struct HilSensorData {
    pub imu: RawImuReading,
    pub baro: RawBaroReading,
    pub mag: RawMagReading,
}

/// Raw GPS data from HIL_GPS message
#[derive(Debug, Clone, Default)]
pub struct HilGpsData {
    pub gnss: RawGnssReading,
}

/// SITL I/O transport layer
///
/// Handles communication with the simulator. Does NOT implement HAL traits -
/// those are implemented by `BoardHal` in `aviate-hal-io` using fake drivers.
///
/// ## Data Flow
///
/// **Sensors (input):**
/// ```text
/// Simulator → SitlIO.poll() → take_sensor_data() → board feeds fake sensors
/// ```
///
/// **Actuators (output):**
/// ```text
/// BoardHal.write() → FakeActuator → board takes cmd → SitlIO.send_actuator()
/// ```
///
/// **Commands (input):**
/// ```text
/// GCS → SitlIO.recv_command() → board processes arm/disarm/setpoints
/// ```
pub struct SitlIO {
    recv_socket: UdpSocket,
    send_socket: UdpSocket,
    config: XilConfig,
    start_time: std::time::Instant,
    armed: bool,
    seq: u8,

    // Buffered sensor data (from last poll)
    sensor_data: Option<HilSensorData>,
    gps_data: Option<HilGpsData>,

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

impl SitlIO {
    /// Create a new SITL I/O transport
    pub fn new(config: XilConfig) -> io::Result<Self> {
        // Socket to receive sensor data from simulator
        let recv_socket = UdpSocket::bind(("0.0.0.0", config.sensor_port()))?;
        recv_socket.set_nonblocking(true)?;

        // Socket to send actuator commands to simulator
        let send_socket = UdpSocket::bind("0.0.0.0:0")?;
        send_socket.set_nonblocking(true)?;

        Ok(Self {
            recv_socket,
            send_socket,
            config,
            start_time: std::time::Instant::now(),
            armed: false,
            seq: 0,
            sensor_data: None,
            gps_data: None,
            command: None,
            last_heartbeat_us: 0,
            gcs_addr: None,
            rx_count: 0,
            tx_count: 0,
        })
    }

    /// Poll for incoming MAVLink messages
    ///
    /// Receives all available messages, updates internal buffers, and sends heartbeat.
    /// Call this at the start of each control loop iteration.
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

    /// Take buffered sensor data (from HIL_SENSOR)
    ///
    /// Returns None if no new sensor data received since last take.
    pub fn take_sensor_data(&mut self) -> Option<HilSensorData> {
        self.sensor_data.take()
    }

    /// Take buffered GPS data (from HIL_GPS)
    ///
    /// Returns None if no new GPS data received since last take.
    pub fn take_gps_data(&mut self) -> Option<HilGpsData> {
        self.gps_data.take()
    }

    /// Process received MAVLink data
    fn process_mavlink_data(&mut self, data: &[u8], src: std::net::SocketAddr) {
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
                self.gcs_addr = Some(src);
            }
            _ => {}
        }
    }

    fn handle_hil_sensor(&mut self, sensor: HilSensor) {
        self.sensor_data = Some(HilSensorData {
            imu: RawImuReading {
                accel: [sensor.xacc, sensor.yacc, sensor.zacc],
                gyro: [sensor.xgyro, sensor.ygyro, sensor.zgyro],
                temperature: Some(sensor.temperature),
            },
            baro: RawBaroReading {
                pressure_pa: sensor.abs_pressure * 100.0, // mbar to Pa
                temperature_c: sensor.temperature,
            },
            mag: RawMagReading {
                field_ut: [
                    sensor.xmag * 100.0, // Gauss to uT
                    sensor.ymag * 100.0,
                    sensor.zmag * 100.0,
                ],
            },
        });
    }

    fn handle_hil_gps(&mut self, gps: HilGps) {
        let fix = match gps.fix_type {
            0 | 1 => GnssFix::None,
            2 => GnssFix::TwoD,
            3 | 4 => GnssFix::ThreeD,
            5 => GnssFix::RtkFloat,
            6 => GnssFix::RtkFixed,
            _ => GnssFix::None,
        };

        self.gps_data = Some(HilGpsData {
            gnss: RawGnssReading {
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
            },
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

    /// Send actuator command to simulator via HIL_ACTUATOR_CONTROLS
    ///
    /// This is the MAVLink output for actuator commands. The board layer calls this
    /// after reading from FakeActuator (via BoardHal's ActuatorHal implementation).
    ///
    /// ## Data Flow
    ///
    /// ```text
    /// Kernel → BoardHal.write(&ActuatorCmd) → FakeActuator
    ///                                              ↓
    ///                            board reads via actuator_mut().take_cmd()
    ///                                              ↓
    ///                            SitlMavlink.send_actuator(&RawActuatorCmd)
    ///                                              ↓
    ///                            HIL_ACTUATOR_CONTROLS → Gazebo
    /// ```
    pub fn send_actuator(&mut self, cmd: &RawActuatorCmd) {
        let msg = HilActuatorControls {
            time_usec: self.now_us(),
            controls: cmd.outputs,
            mode: if self.armed {
                MavModeFlag::SAFETY_ARMED.0
            } else {
                0
            },
            flags: 0,
        };

        self.send_message(&MavMessage::HilActuatorControls(msg));
    }

    /// Set armed state (for MAVLink mode flags)
    pub fn set_armed(&mut self, armed: bool) {
        self.armed = armed;
        if armed {
            eprintln!("[INFO] MAVLink armed");
        } else {
            eprintln!("[INFO] MAVLink disarmed");
        }
    }

    /// Check if armed
    pub fn is_armed(&self) -> bool {
        self.armed
    }

    /// Send a MAVLink message to simulator
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

    /// Get statistics
    pub fn stats(&self) -> (u64, u64) {
        (self.rx_count, self.tx_count)
    }
}

// Implement SystemHal - timing and system functions
impl SystemHal for SitlIO {
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

// Implement CommandHal - receives commands from GCS
impl CommandHal for SitlIO {
    fn recv_command(&mut self) -> Option<SystemCommand> {
        self.poll();
        self.command.take()
    }
}
