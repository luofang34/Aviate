//! MicoAir H743-V2 Hardware Abstraction
//!
//! This module provides the `MicoAirBoard` struct that implements the `BoardStep` trait,
//! enabling the FlightRunner to execute the control loop on real hardware.

#![allow(dead_code)] // Phase 1 - not all fields used yet

#[cfg(feature = "env-flight")]
extern crate alloc;
#[cfg(feature = "env-flight")]
use embedded_alloc::Heap;

#[cfg(feature = "env-flight")]
#[global_allocator]
static HEAP: Heap = Heap::empty();

use aviate_core::control::multirotor::MultirotorController;
use aviate_core::control::{Command, CommandSource, ConfigMode, ControlMode, Setpoint};
use aviate_core::ekf::Ekf;
use aviate_core::hal::{ActuatorHal, SensorHal, SystemCommand};
use aviate_core::math::{Quaternion, Vector3};
use aviate_core::mixer::{ModeConfig, QuadXMixer, Sanitizer};
use aviate_core::time::{TimeDelta, TimeSource, Timestamp};
use aviate_core::types::{Meters, MetersPerSecond, Normalized, Seconds};
use aviate_core::{AviateKernel, ChannelId, DefaultAviateKernel, InitState};

use aviate_hal_io::traits::{BaroDriver, ImuDriver, MagDriver};
use aviate_hal_io::{BoardHal, FakeActuator, FakeBaro, FakeImu, FakeMag};

use aviate_hal_stm32h7::{NoSleep, Stm32h7Time};
use stm32h7xx_hal::{
    self,
    pac::{CorePeripherals, Peripherals, IWDG},
    prelude::*,
    rcc::rec::UsbClkSel,
};

use crate::led::LedHeartbeat;
use crate::usb_cdc::BoardTransport;
use crate::watchdog::BoardWatchdog;

use aviate_runtime::runner::BoardStep;

#[cfg(feature = "env-flight")]
use embedded_hal_compat::ForwardCompat;

// Real sensor drivers
#[cfg(all(feature = "env-flight", not(feature = "env-hitl")))]
use crate::drivers::Bmi088Imu;
#[cfg(all(feature = "env-flight", not(feature = "env-hitl")))]
use alloc::boxed::Box;

// ============================================================================
// Compatibility Wrappers (I2C 0.2->1.0, Delay Copy)
// ============================================================================

#[cfg(all(feature = "env-flight", not(feature = "env-hitl")))]
pub mod compat {
    use embedded_hal::delay::DelayNs;
    use embedded_hal::i2c::{ErrorKind, ErrorType, I2c, Operation};
    use embedded_hal_compat::eh0_2::blocking::i2c::{
        Read as Read02, Write as Write02, WriteRead as WriteRead02,
    };

    pub struct I2cWrapper<T>(pub T);

    #[derive(Debug)]
    pub struct WrapperError;
    impl embedded_hal::i2c::Error for WrapperError {
        fn kind(&self) -> ErrorKind {
            ErrorKind::Other
        }
    }

    impl<T: Read02 + Write02 + WriteRead02> ErrorType for I2cWrapper<T> {
        type Error = WrapperError;
    }

    impl<T: Read02 + Write02 + WriteRead02> I2c for I2cWrapper<T> {
        fn transaction(
            &mut self,
            address: u8,
            operations: &mut [Operation],
        ) -> Result<(), Self::Error> {
            match operations {
                [Operation::Write(w)] => self.0.write(address, w).map_err(|_| WrapperError),
                [Operation::Write(w), Operation::Read(r)] => {
                    self.0.write_read(address, w, r).map_err(|_| WrapperError)
                }
                [Operation::Read(r)] => self.0.read(address, r).map_err(|_| WrapperError),
                _ => Err(WrapperError),
            }
        }
    }

    #[derive(Clone, Copy)]
    pub struct SysDelay {
        pub sysclk: u32,
    }

    impl SysDelay {
        pub fn new(sysclk: u32) -> Self {
            Self { sysclk }
        }
    }

    impl DelayNs for SysDelay {
        fn delay_ns(&mut self, ns: u32) {
            let cycles = (ns as u64 * self.sysclk as u64) / 1_000_000_000;
            let cycles = if cycles == 0 && ns > 0 { 1 } else { cycles };
            cortex_m::asm::delay(cycles as u32);
        }

        fn delay_us(&mut self, us: u32) {
            let cycles = (us as u64 * self.sysclk as u64) / 1_000_000;
            cortex_m::asm::delay(cycles as u32);
        }

        fn delay_ms(&mut self, ms: u32) {
            self.delay_us(ms * 1000);
        }
    }
}

// ============================================================================
// Boxed Driver Wrappers
// ============================================================================

#[cfg(all(feature = "env-flight", not(feature = "env-hitl")))]
pub struct BoxedImu(pub Box<dyn ImuDriver>);
#[cfg(all(feature = "env-flight", not(feature = "env-hitl")))]
impl ImuDriver for BoxedImu {
    fn read(&mut self) -> aviate_hal_io::error::SensorResult<aviate_hal_io::traits::RawImuReading> {
        self.0.read()
    }
    fn source_id(&self) -> u8 {
        self.0.source_id()
    }
}

#[cfg(all(feature = "env-flight", not(feature = "env-hitl")))]
pub struct BoxedBaro(pub Box<dyn BaroDriver>);
#[cfg(all(feature = "env-flight", not(feature = "env-hitl")))]
impl BaroDriver for BoxedBaro {
    fn read(
        &mut self,
    ) -> aviate_hal_io::error::SensorResult<aviate_hal_io::traits::RawBaroReading> {
        self.0.read()
    }
    fn data_ready(&mut self) -> aviate_hal_io::error::SensorResult<bool> {
        self.0.data_ready()
    }
    fn source_id(&self) -> u8 {
        self.0.source_id()
    }
}

#[cfg(all(feature = "env-flight", not(feature = "env-hitl")))]
pub struct BoxedMag(pub Box<dyn MagDriver>);
#[cfg(all(feature = "env-flight", not(feature = "env-hitl")))]
impl MagDriver for BoxedMag {
    fn read(&mut self) -> aviate_hal_io::error::SensorResult<aviate_hal_io::traits::RawMagReading> {
        self.0.read()
    }
    fn source_id(&self) -> u8 {
        self.0.source_id()
    }
}

// ============================================================================
// Local SensorCache
// ============================================================================

use aviate_core::sensor::{BaroData, ImuData, MagData, SensorReading, SensorSet};

pub struct SensorCache {
    pub imu: Option<SensorReading<ImuData>>,
    pub baro: Option<SensorReading<BaroData>>,
    pub mag: Option<SensorReading<MagData>>,
}

impl SensorCache {
    pub fn new() -> Self {
        Self {
            imu: None,
            baro: None,
            mag: None,
        }
    }

    pub fn to_sensor_set(&self) -> SensorSet {
        SensorSet {
            imus: [
                self.imu.unwrap_or_default(),
                SensorReading::default(),
                SensorReading::default(),
            ],
            // No GNSS on this board
            gnss: [SensorReading::default(), SensorReading::default()],
            mags: [self.mag.unwrap_or_default(), SensorReading::default()],
            baros: [self.baro.unwrap_or_default(), SensorReading::default()],
            airspeeds: [SensorReading::default(), SensorReading::default()],
            geometry: None,
        }
    }
}

impl Default for SensorCache {
    fn default() -> Self {
        Self::new()
    }
}

// ============================================================================
// Type Aliases
// ============================================================================

#[cfg(any(feature = "env-hitl", not(feature = "env-flight")))]
pub type HwBoardHal = BoardHal<FakeImu, FakeBaro, FakeMag, (), HwTimeSource, FakeActuator>;

#[cfg(all(feature = "env-flight", not(feature = "env-hitl")))]
pub type HwBoardHal = BoardHal<BoxedImu, BoxedBaro, BoxedMag, (), HwTimeSource, FakeActuator>;

pub type HwKernel = DefaultAviateKernel<MultirotorController, QuadXMixer>;

// ============================================================================
// Generic Board Exports
// ============================================================================

pub type FlightBoard = MicoAirBoard;

pub type HwFlightRunner = aviate_runtime::runner::FlightRunner<
    FlightBoard,
    Stm32h7Time<NoSleep>,
    BoardTransport,
    BoardWatchdog,
    SystemCommand,
>;

#[derive(Debug, Clone, Copy)]
pub struct HwTimeSource {
    time_us: u64,
}

impl HwTimeSource {
    pub fn new() -> Self {
        Self { time_us: 0 }
    }

    pub fn sync(&mut self, now_us: u64) {
        self.time_us = now_us;
    }
}

impl Default for HwTimeSource {
    fn default() -> Self {
        Self::new()
    }
}

impl aviate_hal_io::TimeSource for HwTimeSource {
    fn now_us(&self) -> u64 {
        self.time_us
    }
}

// ============================================================================
// MicoAirBoard
// ============================================================================

pub struct MicoAirBoard {
    board_hal: HwBoardHal,
    kernel: HwKernel,
    sensor_cache: SensorCache,
    last_imu_time: Option<u64>,
    ekf_initialized: bool,
    iteration: u32,
    iwdg: Option<IWDG>,
    transport: Option<BoardTransport>,
    led: LedHeartbeat,
}

#[derive(Debug, Clone, Copy)]
pub struct RunnerCreationError;

impl MicoAirBoard {
    pub fn new(dp: Peripherals, mut cp: CorePeripherals) -> Self {
        #[cfg(feature = "env-flight")]
        {
            use core::mem::MaybeUninit;
            // Align heap to 4 bytes using u32
            static mut HEAP_MEM: [MaybeUninit<u32>; 2048] = [MaybeUninit::uninit(); 2048];

            // Safety: No other thread access at init.
            unsafe {
                let heap_ptr = core::ptr::addr_of_mut!(HEAP_MEM);
                HEAP.init(heap_ptr as usize, 8192)
            }
        }

        let iwdg = Some(dp.IWDG);
        cp.DCB.enable_trace();
        cp.DWT.enable_cycle_counter();

        // Safety: Ensure clean state from bootloader
        unsafe {
            // Disable interrupts to prevent early firing
            cortex_m::interrupt::disable();

            // Disable MPU (if enabled by bootloader)
            cp.MPU.ctrl.write(0);
            cortex_m::asm::dsb();
            cortex_m::asm::isb();

            // Relocate Vector Table to App Start (0x08020000)
            cp.SCB.vtor.write(0x0802_0000);
            cortex_m::asm::dsb();
            cortex_m::asm::isb();
        }

        // Clock Configuration (HSI48 for USB + system clocks)
        dp.RCC.cr.modify(|_, w| w.hsi48on().set_bit());
        while !dp.RCC.cr.read().hsi48rdy().bit_is_set() {
            cortex_m::asm::nop();
        }

        let pwr = dp.PWR.constrain();
        let pwrcfg = pwr.freeze();
        let rcc = dp.RCC.constrain();
        let mut ccdr = rcc
            .sys_ck(400.MHz())
            .use_hse(25.MHz())
            .pll1_q_ck(100.MHz()) // SPI1/2/3 Defaults to PLL1Q. Must be enabled!
            .freeze(pwrcfg, &dp.SYSCFG);
        ccdr.peripheral.kernel_usb_clk_mux(UsbClkSel::Hsi48);

        // USB Regulator
        let pwr = unsafe { &*stm32h7xx_hal::pac::PWR::ptr() };
        pwr.cr3.modify(|_, w| w.usb33den().set_bit());
        for _ in 0..10000 {
            cortex_m::asm::nop();
        }

        // USB Transport Init
        let gpioa_pac = dp.GPIOA;
        // PA8 VBUS sensing (match bootloader config)
        gpioa_pac
            .moder
            .modify(|r, w| unsafe { w.bits(r.bits() & !(0b11 << 16)) }); // Input
        gpioa_pac
            .pupdr
            .modify(|r, w| unsafe { w.bits((r.bits() & !(0b11 << 16)) | (0b10 << 16)) }); // Pull-down

        let gpioa = gpioa_pac.split(ccdr.peripheral.GPIOA);
        let usb_dm = gpioa.pa11.into_alternate();
        let usb_dp = gpioa.pa12.into_alternate();

        // Reset USB OTG2 Core
        unsafe {
            let rcc = &*stm32h7xx_hal::pac::RCC::ptr();
            rcc.ahb1rstr.modify(|_, w| w.usb2otgrst().set_bit());
            cortex_m::asm::delay(1000); // Small delay
            rcc.ahb1rstr.modify(|_, w| w.usb2otgrst().clear_bit());
        }

        let usb2 = stm32h7xx_hal::usb_hs::USB2::new(
            dp.OTG2_HS_GLOBAL,
            dp.OTG2_HS_DEVICE,
            dp.OTG2_HS_PWRCLK,
            usb_dm,
            usb_dp,
            ccdr.peripheral.USB2OTG,
            &ccdr.clocks,
        );

        // Force VBUS detection: MicoAir uses PA8, but OTG2 expects PB13.
        // We must disable internal sensing and force "Session Valid".
        unsafe {
            let otg = &*stm32h7xx_hal::pac::OTG2_HS_GLOBAL::ptr();
            otg.gccfg.modify(|_, w| w.vbden().clear_bit());
            otg.gotgctl
                .modify(|r, w| w.bits(r.bits() | (1 << 6) | (1 << 7)));
        }

        let mut transport = BoardTransport::new();
        transport.init_usb(usb2);

        // LED Init
        let gpioe = dp.GPIOE.split(ccdr.peripheral.GPIOE);
        let green = gpioe.pe2.into_push_pull_output();
        let red = gpioe.pe3.into_push_pull_output();
        let blue = gpioe.pe4.into_push_pull_output();
        let led = LedHeartbeat::new(green, red, blue);

        let time_source = HwTimeSource::new();

        #[cfg(any(feature = "env-hitl", not(feature = "env-flight")))]
        let board_hal = {
            BoardHal::new(
                FakeImu::new(),
                FakeBaro::new(),
                FakeMag::new(),
                (), // No GNSS
                time_source,
                FakeActuator::new(),
            )
        };

        #[cfg(all(feature = "env-flight", not(feature = "env-hitl")))]
        let board_hal = {
            use crate::drivers::Bmi088Imu;
            use crate::hw::compat::{I2cWrapper, SysDelay};
            use core::cell::RefCell;
            use embedded_hal_bus::spi::RefCellDevice;

            let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);
            let _gpioc = dp.GPIOC.split(ccdr.peripheral.GPIOC);
            let gpiod = dp.GPIOD.split(ccdr.peripheral.GPIOD);

            // SPI2 Init for BMI088 (PB13/PB14/PB15)
            let sck = gpiob.pb13.into_alternate();
            let miso = gpiob.pb14.into_alternate();
            let mosi = gpiob.pb15.into_alternate();

            let spi2 = dp
                .SPI2
                .spi(
                    (sck, miso, mosi),
                    stm32h7xx_hal::spi::MODE_0,
                    10.MHz(),
                    ccdr.peripheral.SPI2,
                    &ccdr.clocks,
                )
                .forward();

            // Leak SPI bus to static lifetime to allow RefCellDevice in BoxedImu
            let spi2_bus = Box::leak(Box::new(RefCell::new(spi2)));

            let mut delay = SysDelay::new(400_000_000); // 400MHz System Clock

            // I2C2 Init for SPL06 (PB10/PB11)
            let scl = gpiob.pb10.into_alternate_open_drain();
            let sda = gpiob.pb11.into_alternate_open_drain();

            let i2c2 = dp
                .I2C2
                .i2c((scl, sda), 400.kHz(), ccdr.peripheral.I2C2, &ccdr.clocks);

            let i2c2 = I2cWrapper(i2c2);

            use embedded_hal::digital::OutputPin;
            let mut gyro_cs = gpiod.pd5.into_push_pull_output().forward();
            let mut accel_cs = gpiod.pd4.into_push_pull_output().forward();
            let _ = gyro_cs.set_high();
            let _ = accel_cs.set_high();

            // Safety delay to ensure sensors are powered up
            use embedded_hal::delay::DelayNs;
            delay.delay_ms(100);

            // SPI Device wrappers (RefCellDevice)
            let accel_dev = RefCellDevice::new(spi2_bus, accel_cs, delay.clone())
                .expect("Failed to create accel device");
            let gyro_dev = RefCellDevice::new(spi2_bus, gyro_cs, delay.clone())
                .expect("Failed to create gyro device");

            // Step 1: Real BMI088
            let boxed_imu =
                match Bmi088Imu::new(accel_dev, gyro_dev, &mut delay, crate::Rotation::None) {
                    Ok(dev) => BoxedImu(Box::new(dev)),
                    Err(_) => BoxedImu(Box::new(FakeImu::new())),
                };

            // Step 2: Fake SPL06 (Baro) - embedded-hal version mismatch
            let _i2c2 = i2c2;
            let boxed_baro = BoxedBaro(Box::new(FakeBaro::new()));

            // Step 3: Real QMC5883L (Mag) - on I2C1 (PB8/PB9)
            let scl1 = gpiob.pb8.into_alternate_open_drain();
            let sda1 = gpiob.pb9.into_alternate_open_drain();
            let i2c1 = dp
                .I2C1
                .i2c((scl1, sda1), 400.kHz(), ccdr.peripheral.I2C1, &ccdr.clocks);
            // I2C1 initialized but magnetometer disabled (embedded-hal version mismatch)
            let _i2c1 = I2cWrapper(i2c1);

            // Fake Mag (QMC5883L) - using fake for now
            let boxed_mag = BoxedMag(Box::new(FakeMag::new()));

            // Note: I2C init logic removed for checking SPI only

            BoardHal::new(
                boxed_imu,
                boxed_baro,
                boxed_mag,
                (), // No GNSS
                time_source,
                FakeActuator::new(),
            )
        };

        let kernel = create_kernel();

        // Safety: Enable global interrupts now that everything is initialized
        unsafe {
            cortex_m::interrupt::enable();
        }

        Self {
            board_hal,
            kernel,
            sensor_cache: SensorCache::new(),
            last_imu_time: None,
            ekf_initialized: false,
            iteration: 0,
            iwdg,
            transport: Some(transport),
            led,
        }
    }

    pub fn try_into_runner(mut self) -> Result<HwFlightRunner, RunnerCreationError> {
        let time = Stm32h7Time::new(400, NoSleep);
        let default_cmd = default_command();
        let transport = self.transport.take().ok_or(RunnerCreationError)?;
        let iwdg = self.iwdg.take().ok_or(RunnerCreationError)?;
        let watchdog = BoardWatchdog::new(iwdg, 500);

        Ok(aviate_runtime::runner::FlightRunner::new(
            self,
            time,
            transport,
            watchdog,
            SystemCommand::FlightControl(default_cmd),
        ))
    }

    pub fn into_runner(self) -> HwFlightRunner {
        match self.try_into_runner() {
            Ok(runner) => runner,
            Err(_) => loop {
                cortex_m::asm::wfi();
            },
        }
    }

    pub fn board_hal_mut(&mut self) -> &mut HwBoardHal {
        &mut self.board_hal
    }
    pub fn kernel_mut(&mut self) -> &mut HwKernel {
        &mut self.kernel
    }
    pub fn is_armed(&self) -> bool {
        self.kernel.state.init_state == InitState::Armed
    }
}

impl BoardStep for MicoAirBoard {
    type Cmd = SystemCommand;

    fn board_step(
        &mut self,
        tick_us: u64,
        _now_us: u64,
        dt_us: u32,
        sys_cmd: &SystemCommand,
        _link_ok: bool,
    ) {
        // COV:EXCL_START(STUB)
        let mut current_dt = (dt_us as f32) * 1e-6;

        if let Some(imu) = self.board_hal.read_imu() {
            let current_time = imu.timestamp.ticks;
            if let Some(last) = self.last_imu_time {
                let delta_us = current_time.saturating_sub(last);
                current_dt = (delta_us as f32) * 1e-6;
                current_dt = current_dt.clamp(0.0001, 0.1);
            }
            self.last_imu_time = Some(current_time);
            self.sensor_cache.imu = Some(imu);
        }

        // GNSS removed
        if let Some(baro) = self.board_hal.read_baro() {
            self.sensor_cache.baro = Some(baro);
        }
        if let Some(mag) = self.board_hal.read_mag() {
            self.sensor_cache.mag = Some(mag);
        }

        let time_delta = TimeDelta {
            dt_sec: Seconds(current_dt),
            tick_delta: dt_us as u64,
        };

        let cmd: Command = match sys_cmd {
            SystemCommand::Arm => {
                if self.kernel.arm().is_ok() {
                    self.board_hal.arm();
                }
                default_command()
            }
            SystemCommand::Disarm => {
                self.kernel.disarm();
                self.board_hal.disarm();
                default_command()
            }
            SystemCommand::FlightControl(flight_cmd) => {
                self.kernel
                    .state.checks
                    .pre_arm
                    .update_throttle(flight_cmd.setpoint.collective_thrust.0 < 0.1);
                flight_cmd.clone()
            }
        };

        if !self.ekf_initialized && self.sensor_cache.imu.is_some() {
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

        let sensors = self.sensor_cache.to_sensor_set();
        if !self.kernel.is_ready() {
            let ts = Timestamp {
                ticks: tick_us,
                source: TimeSource::Internal,
            };
            self.kernel.init_step(&sensors, ts);
        }

        let result = self.kernel.update(
            ChannelId(0),
            time_delta,
            &sensors,
            &cmd,
            &aviate_core::mixer::ActuatorState::default(),
            None,
        );
        self.board_hal.write(&result.actuator);
        self.iteration = self.iteration.wrapping_add(1);
        // COV:EXCL_STOP
    }

    fn sensors_ok(&self) -> bool {
        self.sensor_cache.imu.is_some()
    }
    fn ekf_converged(&self) -> bool {
        self.ekf_initialized && self.kernel.is_ready()
    }
}

fn create_kernel() -> HwKernel {
    let controller = MultirotorController::default();
    let mixer = QuadXMixer {
        timestamp_source: hw_timestamp,
    };
    let mode_config = ModeConfig {
        mode: ConfigMode::Hover,
        groups: &[],
    };
    let mut kernel = AviateKernel::new(
        Ekf::default(),
        controller,
        mixer,
        Sanitizer,
        mode_config,
    );
    kernel.state.checks.pre_arm.update_throttle(true);
    kernel
}

pub fn default_command() -> Command {
    Command {
        mode: ControlMode::Attitude,
        setpoint: Setpoint {
            collective_thrust: Normalized(0.0),
            ..Default::default()
        },
        source: CommandSource::Failsafe,
        sequence: 0,
        config_mode_request: None,
        sensor_overrides: None,
    }
}

fn hw_timestamp() -> Timestamp {
    Timestamp {
        ticks: 0,
        source: TimeSource::Internal,
    }
}

impl MicoAirBoard {
    pub fn process_command(&mut self, sys_cmd: SystemCommand) {
        match sys_cmd {
            SystemCommand::FlightControl(cmd) => {
                self.kernel
                    .state.checks
                    .pre_arm
                    .update_throttle(cmd.setpoint.collective_thrust.0 < 0.1);
            }
            SystemCommand::Arm => {
                if self.kernel.arm().is_ok() {
                    self.board_hal.arm();
                }
            }
            SystemCommand::Disarm => {
                self.kernel.disarm();
                self.board_hal.disarm();
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn test_default_command() {
        let cmd = default_command();
        assert_eq!(cmd.setpoint.collective_thrust.0, 0.0);
    }
    #[test]
    fn test_hw_time_source() {
        let mut ts = HwTimeSource::new();
        ts.sync(1000);
        assert_eq!(ts.now_us(), 1000);
    }
}
