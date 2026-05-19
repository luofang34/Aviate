//! Fault Controller for XIL Testing
//!
//! UDP listener that receives FaultCommand messages from test runner
//! and applies faults to FakeDriver sensors.
//!
//! ## Usage
//!
//! ```ignore
//! use aviate_hal_xil::{FaultController, FaultSensors, XilConfig};
//! use aviate_hal_io::{FakeImu, FakeBaro, FakeMag, FakeGnss};
//!
//! let config = XilConfig::for_instance(0);
//! let mut ctrl = FaultController::new(&config)?;
//!
//! // Sensors live where the FC owns them — no Arc<Mutex> needed.
//! let mut imu = FakeImu::new();
//! let mut baro = FakeBaro::new();
//! let mut mag = FakeMag::new();
//! let mut gnss = FakeGnss::new();
//!
//! // Per-cycle: hand the FaultController mutable references and
//! // let it apply any inbound fault commands to the sensors.
//! ctrl.poll(&mut FaultSensors { imu: &mut imu, baro: &mut baro,
//!                                mag: &mut mag, gnss: &mut gnss });
//! ```

#![forbid(unsafe_code)]

use std::io;
use std::net::UdpSocket;

use crate::fault_protocol::{AckStatus, FaultAck, FaultCommand, FAULT_CMD_SIZE};
#[cfg(feature = "xil-fault")]
use crate::mission::{FaultSpec, SensorTarget};
use crate::{PortSlot, XilConfig};

// Re-export SensorFault when xil-fault feature is enabled in aviate-hal-io
#[cfg(feature = "xil-fault")]
use aviate_hal_io::SensorFault;
use aviate_hal_io::{FakeBaro, FakeGnss, FakeImu, FakeMag};

/// Mutable-borrow bundle of the four fake sensors a `FaultController`
/// can corrupt. Used as the second argument to `poll()` so the
/// controller does not need to own the sensors — the SitlRunner can
/// keep them inside `BoardHal` and just hand out references each
/// cycle. Avoids the `Arc<Mutex<>>` machinery that an owning design
/// would require in a single-threaded runtime.
pub struct FaultSensors<'a> {
    pub imu: &'a mut FakeImu,
    pub baro: &'a mut FakeBaro,
    pub mag: &'a mut FakeMag,
    pub gnss: &'a mut FakeGnss,
}

/// Fault controller error
#[derive(Debug)]
pub enum FaultCtrlError {
    /// Failed to bind UDP socket
    BindFailed(io::Error),
    /// Failed to send acknowledgment
    SendFailed(io::Error),
}

impl std::fmt::Display for FaultCtrlError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::BindFailed(e) => write!(f, "Failed to bind fault control socket: {}", e),
            Self::SendFailed(e) => write!(f, "Failed to send fault acknowledgment: {}", e),
        }
    }
}

impl std::error::Error for FaultCtrlError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::BindFailed(e) | Self::SendFailed(e) => Some(e),
        }
    }
}

/// Fault controller that receives commands and applies faults to sensors
///
/// Only functional when the `xil-fault` feature is enabled on aviate-hal-io.
/// When disabled, commands are acknowledged with `NotEnabled` status.
pub struct FaultController {
    /// UDP socket for receiving commands
    socket: UdpSocket,
    /// Receive buffer
    buf: [u8; 64],
}

impl FaultController {
    /// Create a new fault controller bound to the per-instance fault
    /// command port. Sensors are NOT owned — they are passed in by
    /// reference at `poll` time, so the SitlRunner can keep them in
    /// its `BoardHal` without `Arc<Mutex<>>` overhead.
    pub fn new(config: &XilConfig) -> Result<Self, FaultCtrlError> {
        let port = config.net.port(config.instance as u16, PortSlot::FaultCmd);
        let addr = format!("127.0.0.1:{}", port);

        let socket = UdpSocket::bind(&addr).map_err(FaultCtrlError::BindFailed)?;
        socket
            .set_nonblocking(true)
            .map_err(FaultCtrlError::BindFailed)?;

        Ok(Self {
            socket,
            buf: [0u8; 64],
        })
    }

    /// Poll for incoming fault commands (non-blocking). Each command
    /// is applied to the sensors borrowed via `FaultSensors`, and an
    /// ack is sent back to the originating address.
    ///
    /// Returns the number of commands processed.
    pub fn poll(&mut self, sensors: &mut FaultSensors<'_>) -> usize {
        let mut count = 0;

        loop {
            match self.socket.recv_from(&mut self.buf) {
                Ok((len, src)) => {
                    if len >= FAULT_CMD_SIZE {
                        if let Some(cmd) = FaultCommand::from_bytes(&self.buf[..len]) {
                            let ack = Self::handle_command(&cmd, sensors);
                            let ack_bytes = ack.to_bytes();
                            let _ = self.socket.send_to(&ack_bytes, src);
                            count += 1;
                        }
                    }
                }
                Err(ref e) if e.kind() == io::ErrorKind::WouldBlock => {
                    // No more data available
                    break;
                }
                Err(_) => {
                    // Other error, ignore and continue
                    break;
                }
            }
        }

        count
    }

    /// Handle a single fault command
    fn handle_command(cmd: &FaultCommand, sensors: &mut FaultSensors<'_>) -> FaultAck {
        // When xil-fault feature is not enabled, return NotEnabled
        #[cfg(not(feature = "xil-fault"))]
        {
            let _ = sensors;
            FaultAck::error(cmd, AckStatus::NotEnabled)
        }

        #[cfg(feature = "xil-fault")]
        {
            // Apply fault based on target
            match cmd.target {
                None => {
                    // Clear all sensors
                    if cmd.fault.is_none() {
                        Self::clear_all(sensors);
                        FaultAck::ok(cmd)
                    } else {
                        // Can't apply fault to "all" - only clear is valid
                        FaultAck::error(cmd, AckStatus::InvalidParams)
                    }
                }
                Some(target) => {
                    let result = match &cmd.fault {
                        None => Self::clear_sensor(target, sensors),
                        Some(spec) => Self::apply_fault(target, spec, sensors),
                    };

                    if result {
                        FaultAck::ok(cmd)
                    } else {
                        FaultAck::error(cmd, AckStatus::InvalidParams)
                    }
                }
            }
        }
    }

    /// Clear all sensor faults
    #[cfg(feature = "xil-fault")]
    fn clear_all(sensors: &mut FaultSensors<'_>) {
        sensors.imu.clear_faults();
        sensors.baro.clear_faults();
        sensors.mag.clear_faults();
        sensors.gnss.clear_faults();
    }

    /// Clear a specific sensor's faults
    #[cfg(feature = "xil-fault")]
    fn clear_sensor(target: SensorTarget, sensors: &mut FaultSensors<'_>) -> bool {
        match target {
            SensorTarget::Imu => sensors.imu.clear_faults(),
            SensorTarget::Baro => sensors.baro.clear_faults(),
            SensorTarget::Mag => sensors.mag.clear_faults(),
            SensorTarget::Gnss => sensors.gnss.clear_faults(),
        }
        true
    }

    /// Apply a fault to a specific sensor
    #[cfg(feature = "xil-fault")]
    fn apply_fault(target: SensorTarget, spec: &FaultSpec, sensors: &mut FaultSensors<'_>) -> bool {
        match target {
            SensorTarget::Imu => Self::apply_imu_fault(spec, sensors.imu),
            SensorTarget::Baro => Self::apply_baro_fault(spec, sensors.baro),
            SensorTarget::Mag => Self::apply_mag_fault(spec, sensors.mag),
            SensorTarget::Gnss => Self::apply_gnss_fault(spec, sensors.gnss),
        }
    }

    /// Apply fault to IMU
    #[cfg(feature = "xil-fault")]
    fn apply_imu_fault(spec: &FaultSpec, imu: &mut FakeImu) -> bool {
        let fault = match spec {
            FaultSpec::HealthDegraded => SensorFault::HealthDegraded,
            FaultSpec::HealthFailed => SensorFault::HealthFailed,
            FaultSpec::NaN => SensorFault::NaN,
            FaultSpec::Dropout { cycles } => SensorFault::Dropout {
                remaining_cycles: *cycles,
            },
            FaultSpec::BiasShift { offset } => SensorFault::BiasShift { offset: *offset },
            FaultSpec::BiasScalar { .. } => {
                // BiasScalar not applicable to IMU
                return false;
            }
        };
        imu.inject_fault(fault);
        true
    }

    /// Apply fault to barometer
    #[cfg(feature = "xil-fault")]
    fn apply_baro_fault(spec: &FaultSpec, baro: &mut FakeBaro) -> bool {
        let fault = match spec {
            FaultSpec::HealthDegraded => SensorFault::HealthDegraded,
            FaultSpec::HealthFailed => SensorFault::HealthFailed,
            FaultSpec::NaN => SensorFault::NaN,
            FaultSpec::Dropout { cycles } => SensorFault::Dropout {
                remaining_cycles: *cycles,
            },
            FaultSpec::BiasScalar { offset } => SensorFault::BiasShiftScalar { offset: *offset },
            FaultSpec::BiasShift { .. } => {
                // BiasShift (3-axis) not applicable to Baro
                return false;
            }
        };
        baro.inject_fault(fault);
        true
    }

    /// Apply fault to magnetometer
    #[cfg(feature = "xil-fault")]
    fn apply_mag_fault(spec: &FaultSpec, mag: &mut FakeMag) -> bool {
        let fault = match spec {
            FaultSpec::HealthDegraded => SensorFault::HealthDegraded,
            FaultSpec::HealthFailed => SensorFault::HealthFailed,
            FaultSpec::NaN => SensorFault::NaN,
            FaultSpec::Dropout { cycles } => SensorFault::Dropout {
                remaining_cycles: *cycles,
            },
            FaultSpec::BiasShift { offset } => SensorFault::BiasShift { offset: *offset },
            FaultSpec::BiasScalar { .. } => {
                // BiasScalar not applicable to Mag
                return false;
            }
        };
        mag.inject_fault(fault);
        true
    }

    /// Apply fault to GNSS
    #[cfg(feature = "xil-fault")]
    fn apply_gnss_fault(spec: &FaultSpec, gnss: &mut FakeGnss) -> bool {
        let fault = match spec {
            FaultSpec::HealthDegraded => SensorFault::HealthDegraded,
            FaultSpec::HealthFailed => SensorFault::HealthFailed,
            FaultSpec::NaN => SensorFault::NaN,
            FaultSpec::Dropout { cycles } => SensorFault::Dropout {
                remaining_cycles: *cycles,
            },
            FaultSpec::BiasShift { offset } => SensorFault::BiasShift { offset: *offset },
            FaultSpec::BiasScalar { .. } => {
                // BiasScalar not applicable to GNSS
                return false;
            }
        };
        gnss.inject_fault(fault);
        true
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::panic)]
mod tests {
    use super::*;

    /// Build a fresh four-tuple of fake sensors for in-process tests.
    fn fresh_sensors() -> (FakeImu, FakeBaro, FakeMag, FakeGnss) {
        (
            FakeImu::new(),
            FakeBaro::new(),
            FakeMag::new(),
            FakeGnss::new(),
        )
    }

    #[test]
    fn test_fault_controller_creation() {
        let config = XilConfig::for_instance(99); // high instance to avoid port conflicts
        let ctrl = FaultController::new(&config);
        assert!(ctrl.is_ok());
    }

    #[test]
    fn test_fault_controller_poll_empty() {
        let config = XilConfig::for_instance(98);
        let mut ctrl =
            FaultController::new(&config).expect("controller should bind on free instance");
        let (mut imu, mut baro, mut mag, mut gnss) = fresh_sensors();
        let mut sensors = FaultSensors {
            imu: &mut imu,
            baro: &mut baro,
            mag: &mut mag,
            gnss: &mut gnss,
        };

        // Poll should return 0 when no commands pending
        assert_eq!(ctrl.poll(&mut sensors), 0);
    }

    #[cfg(feature = "xil-fault")]
    #[test]
    fn test_fault_controller_apply_imu_fault() {
        let (mut imu, _baro, _mag, _gnss) = fresh_sensors();
        assert!(FaultController::apply_imu_fault(
            &FaultSpec::HealthDegraded,
            &mut imu
        ));
        assert!(imu.has_fault());
    }

    #[cfg(feature = "xil-fault")]
    #[test]
    fn test_fault_controller_clear_all() {
        let (mut imu, mut baro, mut mag, mut gnss) = fresh_sensors();
        imu.inject_fault(SensorFault::HealthFailed);
        baro.inject_fault(SensorFault::HealthDegraded);

        let mut sensors = FaultSensors {
            imu: &mut imu,
            baro: &mut baro,
            mag: &mut mag,
            gnss: &mut gnss,
        };
        FaultController::clear_all(&mut sensors);

        assert!(!imu.has_fault());
        assert!(!baro.has_fault());
    }
}

/// Integration tests for FaultClient + FaultController end-to-end.
///
/// These tests do need `Arc<Mutex<>>` because they spawn a polling
/// thread to receive the UDP fault command — the thread and the
/// asserting test body both want to read fault state. In production
/// the poll happens in the FC's single-threaded main loop and does
/// not need the Mutex.
#[cfg(all(test, feature = "xil-fault"))]
#[allow(clippy::expect_used, clippy::panic)]
mod integration_tests {
    use super::*;
    use crate::fault_protocol::FaultClient;
    use crate::mission::{FaultSpec, SensorTarget};
    use std::sync::{Arc, Mutex};
    use std::thread;
    use std::time::Duration;

    type SharedSensors = (
        Arc<Mutex<FakeImu>>,
        Arc<Mutex<FakeBaro>>,
        Arc<Mutex<FakeMag>>,
        Arc<Mutex<FakeGnss>>,
    );

    fn fresh_shared_sensors() -> SharedSensors {
        (
            Arc::new(Mutex::new(FakeImu::new())),
            Arc::new(Mutex::new(FakeBaro::new())),
            Arc::new(Mutex::new(FakeMag::new())),
            Arc::new(Mutex::new(FakeGnss::new())),
        )
    }

    /// Spawn a single-poll thread that takes ownership of `ctrl`,
    /// locks the shared sensors briefly, and runs one `poll()`.
    /// Returns the join handle and a count of processed commands.
    fn spawn_one_poll(
        mut ctrl: FaultController,
        sensors: SharedSensors,
        delay: Duration,
    ) -> thread::JoinHandle<usize> {
        thread::spawn(move || {
            thread::sleep(delay);
            let (imu, baro, mag, gnss) = sensors;
            let mut imu_g = imu.lock().expect("imu lock");
            let mut baro_g = baro.lock().expect("baro lock");
            let mut mag_g = mag.lock().expect("mag lock");
            let mut gnss_g = gnss.lock().expect("gnss lock");
            ctrl.poll(&mut FaultSensors {
                imu: &mut imu_g,
                baro: &mut baro_g,
                mag: &mut mag_g,
                gnss: &mut gnss_g,
            })
        })
    }

    #[test]
    fn test_client_controller_inject_imu_fault() {
        let instance = 80;
        let config = XilConfig::for_instance(instance);
        let ctrl = FaultController::new(&config).expect("ctrl bind");
        let shared = fresh_shared_sensors();

        let mut client = FaultClient::new(&config).expect("client bind");

        let handle = spawn_one_poll(ctrl, shared.clone(), Duration::from_millis(10));
        thread::sleep(Duration::from_millis(5));

        let ack = client.inject(SensorTarget::Imu, FaultSpec::HealthDegraded);
        let ack = ack.expect("inject send");
        assert!(ack.is_ok(), "ack status: {:?}", ack.status);

        let count = handle.join().expect("poll thread join");
        assert_eq!(count, 1);

        let (imu, _baro, _mag, _gnss) = shared;
        assert!(imu.lock().expect("imu read").has_fault());
    }

    #[test]
    fn test_client_controller_clear_all() {
        let instance = 81;
        let config = XilConfig::for_instance(instance);
        let ctrl = FaultController::new(&config).expect("ctrl bind");
        let shared = fresh_shared_sensors();

        // Pre-inject faults directly.
        shared
            .0
            .lock()
            .expect("imu pre")
            .inject_fault(SensorFault::HealthFailed);
        shared
            .1
            .lock()
            .expect("baro pre")
            .inject_fault(SensorFault::NaN);

        let mut client = FaultClient::new(&config).expect("client bind");

        let handle = spawn_one_poll(ctrl, shared.clone(), Duration::from_millis(10));
        thread::sleep(Duration::from_millis(5));

        let ack = client.clear_all().expect("clear_all send");
        assert!(ack.is_ok());
        handle.join().expect("poll thread join");

        assert!(!shared.0.lock().expect("imu post").has_fault());
        assert!(!shared.1.lock().expect("baro post").has_fault());
    }

    #[test]
    fn test_reproducibility_same_sequence_same_result() {
        // Two runs, identical fault sequences, identical observable
        // sensor states. No UDP — exercise the apply path directly.
        for run in 0..2 {
            let instance = 82 + run;
            let (imu, baro, mag, gnss) = fresh_shared_sensors();
            let _config = XilConfig::for_instance(instance);

            let fault_sequence = [
                (SensorTarget::Imu, FaultSpec::HealthDegraded),
                (SensorTarget::Baro, FaultSpec::BiasScalar { offset: 100.0 }),
                (SensorTarget::Mag, FaultSpec::Dropout { cycles: 5 }),
                (SensorTarget::Gnss, FaultSpec::HealthFailed),
            ];

            for (target, fault) in &fault_sequence {
                let mut imu_g = imu.lock().expect("imu lock");
                let mut baro_g = baro.lock().expect("baro lock");
                let mut mag_g = mag.lock().expect("mag lock");
                let mut gnss_g = gnss.lock().expect("gnss lock");
                let mut sensors = FaultSensors {
                    imu: &mut imu_g,
                    baro: &mut baro_g,
                    mag: &mut mag_g,
                    gnss: &mut gnss_g,
                };
                assert!(
                    FaultController::apply_fault(*target, fault, &mut sensors),
                    "apply failed for {:?}",
                    target
                );
            }

            assert!(imu.lock().expect("imu post").has_fault());
            assert!(baro.lock().expect("baro post").has_fault());
            assert!(mag.lock().expect("mag post").has_fault());
            assert!(gnss.lock().expect("gnss post").has_fault());

            // Clear via direct call to the helper.
            let mut imu_g = imu.lock().expect("imu lock2");
            let mut baro_g = baro.lock().expect("baro lock2");
            let mut mag_g = mag.lock().expect("mag lock2");
            let mut gnss_g = gnss.lock().expect("gnss lock2");
            FaultController::clear_all(&mut FaultSensors {
                imu: &mut imu_g,
                baro: &mut baro_g,
                mag: &mut mag_g,
                gnss: &mut gnss_g,
            });
            drop((imu_g, baro_g, mag_g, gnss_g));

            assert!(!imu.lock().expect("imu cleared").has_fault());
            assert!(!baro.lock().expect("baro cleared").has_fault());
            assert!(!mag.lock().expect("mag cleared").has_fault());
            assert!(!gnss.lock().expect("gnss cleared").has_fault());
        }
    }

    #[test]
    fn test_multiple_inject_clear_cycles() {
        let mut imu = FakeImu::new();

        for cycle in 0..3 {
            assert!(FaultController::apply_imu_fault(
                &FaultSpec::HealthDegraded,
                &mut imu
            ));
            assert!(imu.has_fault(), "cycle {} fault should be active", cycle);

            imu.clear_faults();
            assert!(!imu.has_fault(), "cycle {} fault should be cleared", cycle);
        }
    }

    #[test]
    fn test_different_fault_types_per_sensor() {
        let mut imu = FakeImu::new();
        let mut baro = FakeBaro::new();
        let mut mag = FakeMag::new();
        let mut gnss = FakeGnss::new();

        assert!(FaultController::apply_imu_fault(
            &FaultSpec::BiasShift {
                offset: [0.1, 0.2, 0.3]
            },
            &mut imu
        ));
        assert!(FaultController::apply_baro_fault(
            &FaultSpec::BiasScalar { offset: 50.0 },
            &mut baro
        ));
        assert!(FaultController::apply_mag_fault(&FaultSpec::NaN, &mut mag));
        assert!(FaultController::apply_gnss_fault(
            &FaultSpec::Dropout { cycles: 10 },
            &mut gnss
        ));

        assert!(imu.has_fault());
        assert!(baro.has_fault());
        assert!(mag.has_fault());
        assert!(gnss.has_fault());

        // Clear only IMU
        imu.clear_faults();
        assert!(!imu.has_fault());
        assert!(baro.has_fault());
        assert!(mag.has_fault());
        assert!(gnss.has_fault());
    }
}
