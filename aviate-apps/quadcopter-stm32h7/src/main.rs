#![no_std]
#![no_main]

use aviate_core::AviateKernel;
use aviate_core::control::mc::McController;
use aviate_core::control::{Command, Setpoint, CommandSource, ControlMode, ConfigMode};
use aviate_core::types::Normalized;
use aviate_core::mixer::{Mixer, ActuatorCmd, ModeConfig};
use aviate_core::sensor::{SensorSet, SensorReading, ImuData, GnssData, MagData, BaroData, AirspeedData};
use cortex_m_rt::entry;
use panic_halt as _;

// Force a symbol to be kept
#[used]
static mut SINK: u32 = 0;

struct DummyMixer;
impl Mixer for DummyMixer {
    fn mix(&self, _axis: &aviate_core::control::AxisCommand) -> ActuatorCmd {
        ActuatorCmd::default()
    }
}

#[entry]
fn main() -> ! {
    let mode_config = ModeConfig {
        mode: ConfigMode::Hover,
        groups: &[],
    };
    let mut kernel = AviateKernel::new(McController::default(), DummyMixer, mode_config);
    let cmd = Command {
        mode: ControlMode::Attitude,
        setpoint: Setpoint {
            collective_thrust: Normalized(0.5),
            ..Default::default()
        },
        config_mode_request: None,
        sensor_overrides: None,
        sequence: 0,
        source: CommandSource::Pilot,
    };
    let sensors = SensorSet {
        imus: [SensorReading::<ImuData>::default(); 3],
        gnss: [SensorReading::<GnssData>::default(); 2],
        mags: [SensorReading::<MagData>::default(); 2],
        baros: [SensorReading::<BaroData>::default(); 2],
        airspeeds: [SensorReading::<AirspeedData>::default(); 2],
        geometry: None,
    };

    loop {
        let output = kernel.step(&cmd, &sensors, 0);

        // Force side effect
        unsafe {
             // Accessing array elements for side effect
             let val = output.outputs[0].0;
             core::ptr::write_volatile(&mut SINK, val.to_bits());
        }
    }
}