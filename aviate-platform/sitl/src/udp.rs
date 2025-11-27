//! UDP MAVLink SITL HAL
//!
//! Connects to external simulators (jMAVSim, Gazebo, AirSim) via UDP MAVLink.

use std::net::UdpSocket;
use std::io;

use aviate_core::hal::{AviateHal, SensorHal, ActuatorHal, SystemHal};
use aviate_core::sensor::{
    SensorReading, ImuData, GnssData, BaroData, MagData, SensorHealth, GnssHealth, GnssFix, AirData,
};
use aviate_core::mixer::ActuatorCmd;
use aviate_core::time::{Timestamp, TimeSource};
use aviate_core::types::{
    MetersPerSecondSquared, RadiansPerSecond, Meters, MetersPerSecond, Microtesla, Pascals,
};

// Note: We don't have a real MAVLink crate in this workspace yet (aviate-mavlink is shown in tree but not implemented),
// so we will stub the MAVLink parsing logic or assume aviate-mavlink exists.
// For this exercise, since aviate-mavlink is shown in the tree, I'll assume I can use it, but I should check its content first.
// The user's tree output showed `aviate-mavlink/src/lib.rs` etc.

use crate::SitlConfig;

// Stubbing MAVLink structures if aviate-mavlink is not fully ready or to avoid complex deps in this single file view
// In a real scenario, these would come from `aviate-mavlink`.

pub struct UdpMavlinkHal {
    socket: UdpSocket,
    config: SitlConfig,
    start_time: std::time::Instant,
    armed: bool,
    seq: u8,

    // Buffered sensor data (from latest MAVLink messages)
    imu_data: Option<SensorReading<ImuData>>,
    gnss_data: Option<SensorReading<GnssData>>,
    baro_data: Option<SensorReading<BaroData>>,
    mag_data: Option<SensorReading<MagData>>,

    // Heartbeat tracking
    last_heartbeat_us: u64,
}

impl UdpMavlinkHal {
    pub fn new(config: SitlConfig) -> io::Result<Self> {
        let socket = UdpSocket::bind(("0.0.0.0", config.sensor_port))?;
        socket.set_nonblocking(true)?;

        Ok(Self {
            socket,
            config,
            start_time: std::time::Instant::now(),
            armed: false,
            seq: 0,
            imu_data: None,
            gnss_data: None,
            baro_data: None,
            mag_data: None,
            last_heartbeat_us: 0,
        })
    }

    pub fn poll(&mut self) {
        let mut temp_buf = [0u8; 2048];
        loop {
            match self.socket.recv_from(&mut temp_buf) {
                Ok((len, _src)) => {
                    // Parse packet here. For now, we just print received bytes count
                    // In real impl, parse_mavlink(&temp_buf[..len])
                }
                Err(ref e) if e.kind() == io::ErrorKind::WouldBlock => break,
                Err(_) => break,
            }
        }
        
        // Mock data for now if no simulator connected
        if self.imu_data.is_none() {
            // Provide some fake data to keep loop running
            let ts = Timestamp { ticks: self.now_us(), source: TimeSource::Internal };
            self.imu_data = Some(SensorReading {
                value: ImuData::default(),
                valid: true,
                source_id: 0,
                timestamp: ts,
                health: SensorHealth::Good,
            });
        }
    }

    fn now_us(&self) -> u64 {
        self.start_time.elapsed().as_micros() as u64
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
        // In previous stub it was write_actuators.
        // Construct HIL_ACTUATOR_CONTROLS message and send via UDP
        // Stub for now, eventually implement sending
    }

    fn arm(&mut self) {
        self.armed = true;
    }

    fn disarm(&mut self) {
        self.armed = false;
    }

    fn is_armed(&self) -> bool {
        true // SITL always hardware-armed (switch)
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
        println!("UDP SITL: Reboot");
        std::process::exit(0);
    }

    fn enter_bootloader(&mut self) -> ! {
        println!("UDP SITL: Bootloader not supported");
        std::process::exit(1);
    }
}

impl AviateHal for UdpMavlinkHal {}