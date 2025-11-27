#![forbid(unsafe_code)]

use aviate_core::hal::{SensorHal, ActuatorHal, SystemHal, AviateHal};
use aviate_core::sensor::{SensorReading, ImuData, GnssData, BaroData, MagData};
use aviate_core::mixer::ActuatorCmd;
use aviate_core::time::{Timestamp, TimeSource};

pub struct SitlHal {
    start_time: std::time::Instant,
    armed: bool,
    // Mock sensor states
    pub imu_data: Option<SensorReading<ImuData>>,
    pub gnss_data: Option<SensorReading<GnssData>>,
    pub baro_data: Option<SensorReading<BaroData>>,
    pub mag_data: Option<SensorReading<MagData>>,
}

impl SitlHal {
    pub fn new() -> Self {
        Self {
            start_time: std::time::Instant::now(),
            armed: false,
            imu_data: None,
            gnss_data: None,
            baro_data: None,
            mag_data: None,
        }
    }

    pub fn set_imu(&mut self, data: SensorReading<ImuData>) {
        self.imu_data = Some(data);
    }

    pub fn set_gnss(&mut self, data: SensorReading<GnssData>) {
        self.gnss_data = Some(data);
    }

    pub fn set_baro(&mut self, data: SensorReading<BaroData>) {
        self.baro_data = Some(data);
    }

    pub fn set_mag(&mut self, data: SensorReading<MagData>) {
        self.mag_data = Some(data);
    }
}

impl Default for SitlHal {
    fn default() -> Self {
        Self::new()
    }
}

impl SensorHal for SitlHal {
    fn read_imu(&mut self) -> Option<SensorReading<ImuData>> {
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

impl ActuatorHal for SitlHal {
    fn write(&mut self, cmd: &ActuatorCmd) {
        if self.armed {
            println!("ACTUATOR: {:?}", cmd.outputs);
        }
    }

    fn arm(&mut self) {
        self.armed = true;
        println!("SITL: Armed");
    }

    fn disarm(&mut self) {
        self.armed = false;
        println!("SITL: Disarmed");
    }

    fn is_armed(&self) -> bool {
        self.armed
    }
}

impl SystemHal for SitlHal {
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

    fn kick_watchdog(&mut self) {
        // No-op for SITL
    }

    fn reboot(&mut self) -> ! {
        println!("SITL: Reboot requested");
        std::process::exit(0);
    }

    fn enter_bootloader(&mut self) -> ! {
        println!("SITL: Bootloader mode not supported");
        std::process::exit(1);
    }
}

// Blanket impl for combined trait
impl AviateHal for SitlHal {}
