//! Fault Controller for XIL Testing
//!
//! UDP listener that receives FaultCommand messages from test runner
//! and applies faults to FakeDriver sensors.
//!
//! ## Usage
//!
//! ```ignore
//! use aviate_hal_xil::{FaultController, XilConfig};
//! use aviate_hal_io::{FakeImu, FakeBaro, FakeMag, FakeGnss};
//!
//! // Create fake sensors
//! let imu = Arc::new(Mutex::new(FakeImu::new()));
//! let baro = Arc::new(Mutex::new(FakeBaro::new()));
//! let mag = Arc::new(Mutex::new(FakeMag::new()));
//! let gnss = Arc::new(Mutex::new(FakeGnss::new()));
//!
//! // Create fault controller
//! let config = XilConfig::for_instance(0);
//! let ctrl = FaultController::new(&config, imu, baro, mag, gnss)?;
//!
//! // Poll for commands (non-blocking)
//! ctrl.poll();
//! ```

#![forbid(unsafe_code)]

use std::io;
use std::net::UdpSocket;
use std::sync::{Arc, Mutex};

use crate::fault_protocol::{AckStatus, FaultAck, FaultCommand, FAULT_CMD_SIZE};
#[cfg(feature = "xil-fault")]
use crate::mission::{FaultSpec, SensorTarget};
use crate::{PortSlot, XilConfig};

// Re-export SensorFault when xil-fault feature is enabled in aviate-hal-io
#[cfg(feature = "xil-fault")]
use aviate_hal_io::SensorFault;
use aviate_hal_io::{FakeBaro, FakeGnss, FakeImu, FakeMag};

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
#[allow(dead_code)] // Fields are used when xil-fault feature is enabled
pub struct FaultController {
    /// UDP socket for receiving commands
    socket: UdpSocket,
    /// IMU sensor (shared with board HAL)
    imu: Arc<Mutex<FakeImu>>,
    /// Barometer sensor
    baro: Arc<Mutex<FakeBaro>>,
    /// Magnetometer sensor
    mag: Arc<Mutex<FakeMag>>,
    /// GNSS sensor
    gnss: Arc<Mutex<FakeGnss>>,
    /// Receive buffer
    buf: [u8; 64],
}

impl FaultController {
    /// Create a new fault controller
    ///
    /// Binds to the fault command port for the given instance.
    pub fn new(
        config: &XilConfig,
        imu: Arc<Mutex<FakeImu>>,
        baro: Arc<Mutex<FakeBaro>>,
        mag: Arc<Mutex<FakeMag>>,
        gnss: Arc<Mutex<FakeGnss>>,
    ) -> Result<Self, FaultCtrlError> {
        let port = config.net.port(config.instance as u16, PortSlot::FaultCmd);
        let addr = format!("127.0.0.1:{}", port);

        let socket = UdpSocket::bind(&addr).map_err(FaultCtrlError::BindFailed)?;
        socket
            .set_nonblocking(true)
            .map_err(FaultCtrlError::BindFailed)?;

        Ok(Self {
            socket,
            imu,
            baro,
            mag,
            gnss,
            buf: [0u8; 64],
        })
    }

    /// Poll for incoming fault commands (non-blocking)
    ///
    /// Returns the number of commands processed.
    pub fn poll(&mut self) -> usize {
        let mut count = 0;

        loop {
            match self.socket.recv_from(&mut self.buf) {
                Ok((len, src)) => {
                    if len >= FAULT_CMD_SIZE {
                        if let Some(cmd) = FaultCommand::from_bytes(&self.buf[..len]) {
                            let ack = self.handle_command(&cmd);
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
    fn handle_command(&self, cmd: &FaultCommand) -> FaultAck {
        // When xil-fault feature is not enabled, return NotEnabled
        #[cfg(not(feature = "xil-fault"))]
        {
            let _ = cmd; // suppress unused warning
            FaultAck::error(cmd, AckStatus::NotEnabled)
        }

        #[cfg(feature = "xil-fault")]
        {
            // Apply fault based on target
            match cmd.target {
                None => {
                    // Clear all sensors
                    if cmd.fault.is_none() {
                        self.clear_all();
                        FaultAck::ok(cmd)
                    } else {
                        // Can't apply fault to "all" - only clear is valid
                        FaultAck::error(cmd, AckStatus::InvalidParams)
                    }
                }
                Some(target) => {
                    let result = match &cmd.fault {
                        None => self.clear_sensor(target),
                        Some(spec) => self.apply_fault(target, spec),
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
    fn clear_all(&self) {
        if let Ok(mut imu) = self.imu.lock() {
            imu.clear_faults();
        }
        if let Ok(mut baro) = self.baro.lock() {
            baro.clear_faults();
        }
        if let Ok(mut mag) = self.mag.lock() {
            mag.clear_faults();
        }
        if let Ok(mut gnss) = self.gnss.lock() {
            gnss.clear_faults();
        }
    }

    /// Clear a specific sensor's faults
    #[cfg(feature = "xil-fault")]
    fn clear_sensor(&self, target: SensorTarget) -> bool {
        match target {
            SensorTarget::Imu => {
                if let Ok(mut imu) = self.imu.lock() {
                    imu.clear_faults();
                    true
                } else {
                    false
                }
            }
            SensorTarget::Baro => {
                if let Ok(mut baro) = self.baro.lock() {
                    baro.clear_faults();
                    true
                } else {
                    false
                }
            }
            SensorTarget::Mag => {
                if let Ok(mut mag) = self.mag.lock() {
                    mag.clear_faults();
                    true
                } else {
                    false
                }
            }
            SensorTarget::Gnss => {
                if let Ok(mut gnss) = self.gnss.lock() {
                    gnss.clear_faults();
                    true
                } else {
                    false
                }
            }
        }
    }

    /// Apply a fault to a specific sensor
    #[cfg(feature = "xil-fault")]
    fn apply_fault(&self, target: SensorTarget, spec: &FaultSpec) -> bool {
        match target {
            SensorTarget::Imu => self.apply_imu_fault(spec),
            SensorTarget::Baro => self.apply_baro_fault(spec),
            SensorTarget::Mag => self.apply_mag_fault(spec),
            SensorTarget::Gnss => self.apply_gnss_fault(spec),
        }
    }

    /// Apply fault to IMU
    #[cfg(feature = "xil-fault")]
    fn apply_imu_fault(&self, spec: &FaultSpec) -> bool {
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

        if let Ok(mut imu) = self.imu.lock() {
            imu.inject_fault(fault);
            true
        } else {
            false
        }
    }

    /// Apply fault to barometer
    #[cfg(feature = "xil-fault")]
    fn apply_baro_fault(&self, spec: &FaultSpec) -> bool {
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

        if let Ok(mut baro) = self.baro.lock() {
            baro.inject_fault(fault);
            true
        } else {
            false
        }
    }

    /// Apply fault to magnetometer
    #[cfg(feature = "xil-fault")]
    fn apply_mag_fault(&self, spec: &FaultSpec) -> bool {
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

        if let Ok(mut mag) = self.mag.lock() {
            mag.inject_fault(fault);
            true
        } else {
            false
        }
    }

    /// Apply fault to GNSS
    #[cfg(feature = "xil-fault")]
    fn apply_gnss_fault(&self, spec: &FaultSpec) -> bool {
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

        if let Ok(mut gnss) = self.gnss.lock() {
            gnss.inject_fault(fault);
            true
        } else {
            false
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_fault_controller_creation() {
        let config = XilConfig::for_instance(99); // Use high instance to avoid port conflicts
        let imu = Arc::new(Mutex::new(FakeImu::new()));
        let baro = Arc::new(Mutex::new(FakeBaro::new()));
        let mag = Arc::new(Mutex::new(FakeMag::new()));
        let gnss = Arc::new(Mutex::new(FakeGnss::new()));

        let ctrl = FaultController::new(&config, imu, baro, mag, gnss);
        assert!(ctrl.is_ok());
    }

    #[test]
    fn test_fault_controller_poll_empty() {
        let config = XilConfig::for_instance(98);
        let imu = Arc::new(Mutex::new(FakeImu::new()));
        let baro = Arc::new(Mutex::new(FakeBaro::new()));
        let mag = Arc::new(Mutex::new(FakeMag::new()));
        let gnss = Arc::new(Mutex::new(FakeGnss::new()));

        let mut ctrl = FaultController::new(&config, imu, baro, mag, gnss).unwrap();

        // Poll should return 0 when no commands pending
        assert_eq!(ctrl.poll(), 0);
    }

    #[cfg(feature = "xil-fault")]
    #[test]
    fn test_fault_controller_apply_imu_fault() {
        let config = XilConfig::for_instance(97);
        let imu = Arc::new(Mutex::new(FakeImu::new()));
        let baro = Arc::new(Mutex::new(FakeBaro::new()));
        let mag = Arc::new(Mutex::new(FakeMag::new()));
        let gnss = Arc::new(Mutex::new(FakeGnss::new()));

        let ctrl = FaultController::new(
            &config,
            Arc::clone(&imu),
            Arc::clone(&baro),
            Arc::clone(&mag),
            Arc::clone(&gnss),
        )
        .unwrap();

        // Apply fault directly (bypassing UDP)
        assert!(ctrl.apply_imu_fault(&FaultSpec::HealthDegraded));

        // Verify fault was applied
        let imu = imu.lock().unwrap();
        assert!(imu.has_fault());
    }

    #[cfg(feature = "xil-fault")]
    #[test]
    fn test_fault_controller_clear_all() {
        let config = XilConfig::for_instance(96);
        let imu = Arc::new(Mutex::new(FakeImu::new()));
        let baro = Arc::new(Mutex::new(FakeBaro::new()));
        let mag = Arc::new(Mutex::new(FakeMag::new()));
        let gnss = Arc::new(Mutex::new(FakeGnss::new()));

        // Inject faults
        imu.lock().unwrap().inject_fault(SensorFault::HealthFailed);
        baro.lock()
            .unwrap()
            .inject_fault(SensorFault::HealthDegraded);

        let ctrl = FaultController::new(
            &config,
            Arc::clone(&imu),
            Arc::clone(&baro),
            Arc::clone(&mag),
            Arc::clone(&gnss),
        )
        .unwrap();

        // Clear all
        ctrl.clear_all();

        // Verify faults were cleared
        assert!(!imu.lock().unwrap().has_fault());
        assert!(!baro.lock().unwrap().has_fault());
    }
}

/// Integration tests for FaultClient + FaultController end-to-end
#[cfg(all(test, feature = "xil-fault"))]
mod integration_tests {
    use super::*;
    use crate::fault_protocol::FaultClient;
    use crate::mission::{FaultSpec, SensorTarget};
    use std::thread;
    use std::time::Duration;

    /// Helper to create a fault controller with shared sensors
    fn setup_fault_ctrl(
        instance: u8,
    ) -> (
        FaultController,
        Arc<Mutex<FakeImu>>,
        Arc<Mutex<FakeBaro>>,
        Arc<Mutex<FakeMag>>,
        Arc<Mutex<FakeGnss>>,
    ) {
        let config = XilConfig::for_instance(instance);
        let imu = Arc::new(Mutex::new(FakeImu::new()));
        let baro = Arc::new(Mutex::new(FakeBaro::new()));
        let mag = Arc::new(Mutex::new(FakeMag::new()));
        let gnss = Arc::new(Mutex::new(FakeGnss::new()));

        let ctrl = FaultController::new(
            &config,
            Arc::clone(&imu),
            Arc::clone(&baro),
            Arc::clone(&mag),
            Arc::clone(&gnss),
        )
        .unwrap();

        (ctrl, imu, baro, mag, gnss)
    }

    #[test]
    fn test_client_controller_inject_imu_fault() {
        // Use high instance number to avoid port conflicts with parallel tests
        let instance = 80;
        let (mut ctrl, imu, _baro, _mag, _gnss) = setup_fault_ctrl(instance);

        // Create client for same instance
        let config = XilConfig::for_instance(instance);
        let mut client = FaultClient::new(&config).unwrap();

        // Spawn controller poll in background thread
        let handle = thread::spawn(move || {
            // Give client time to send
            thread::sleep(Duration::from_millis(10));
            ctrl.poll()
        });

        // Wait for controller to start listening
        thread::sleep(Duration::from_millis(5));

        // Send inject command
        let ack = client.inject(SensorTarget::Imu, FaultSpec::HealthDegraded);
        assert!(ack.is_ok());
        let ack = ack.unwrap();
        assert!(ack.is_ok());

        // Wait for controller thread
        let count = handle.join().unwrap();
        assert_eq!(count, 1);

        // Verify fault was applied
        assert!(imu.lock().unwrap().has_fault());
    }

    #[test]
    fn test_client_controller_clear_all() {
        let instance = 81;
        let (mut ctrl, imu, baro, _mag, _gnss) = setup_fault_ctrl(instance);

        // Pre-inject faults directly
        imu.lock().unwrap().inject_fault(SensorFault::HealthFailed);
        baro.lock().unwrap().inject_fault(SensorFault::NaN);

        // Create client
        let config = XilConfig::for_instance(instance);
        let mut client = FaultClient::new(&config).unwrap();

        // Spawn controller poll
        let handle = thread::spawn(move || {
            thread::sleep(Duration::from_millis(10));
            ctrl.poll()
        });

        thread::sleep(Duration::from_millis(5));

        // Send clear all
        let ack = client.clear_all();
        assert!(ack.is_ok());
        assert!(ack.unwrap().is_ok());

        handle.join().unwrap();

        // Verify faults were cleared
        assert!(!imu.lock().unwrap().has_fault());
        assert!(!baro.lock().unwrap().has_fault());
    }

    #[test]
    fn test_reproducibility_same_sequence_same_result() {
        // Test that the same fault sequence produces identical results
        // This is critical for deterministic SITL testing

        // Run the test twice with identical fault sequences
        for run in 0..2 {
            let instance = 82 + run;
            let (ctrl, imu, baro, mag, gnss) = setup_fault_ctrl(instance);

            let config = XilConfig::for_instance(instance);
            let _client = FaultClient::new(&config).unwrap();

            // Define a deterministic fault sequence
            let fault_sequence = [
                (SensorTarget::Imu, FaultSpec::HealthDegraded),
                (SensorTarget::Baro, FaultSpec::BiasScalar { offset: 100.0 }),
                (SensorTarget::Mag, FaultSpec::Dropout { cycles: 5 }),
                (SensorTarget::Gnss, FaultSpec::HealthFailed),
            ];

            // Apply faults (using direct apply to avoid UDP timing issues in test)
            for (target, fault) in &fault_sequence {
                let result = match target {
                    SensorTarget::Imu => ctrl.apply_imu_fault(fault),
                    SensorTarget::Baro => ctrl.apply_baro_fault(fault),
                    SensorTarget::Mag => ctrl.apply_mag_fault(fault),
                    SensorTarget::Gnss => ctrl.apply_gnss_fault(fault),
                };
                assert!(result, "Failed to apply fault for {:?}", target);
            }

            // Verify consistent state
            assert!(imu.lock().unwrap().has_fault());
            assert!(baro.lock().unwrap().has_fault());
            assert!(mag.lock().unwrap().has_fault());
            assert!(gnss.lock().unwrap().has_fault());

            // Clear all and verify clean state
            ctrl.clear_all();
            assert!(!imu.lock().unwrap().has_fault());
            assert!(!baro.lock().unwrap().has_fault());
            assert!(!mag.lock().unwrap().has_fault());
            assert!(!gnss.lock().unwrap().has_fault());
        }
    }

    #[test]
    fn test_multiple_inject_clear_cycles() {
        // Verify fault injection is deterministic across multiple inject/clear cycles
        let instance = 84;
        let (ctrl, imu, _baro, _mag, _gnss) = setup_fault_ctrl(instance);

        for cycle in 0..3 {
            // Inject
            assert!(ctrl.apply_imu_fault(&FaultSpec::HealthDegraded));
            assert!(
                imu.lock().unwrap().has_fault(),
                "Cycle {}: fault should be active",
                cycle
            );

            // Clear
            ctrl.clear_sensor(SensorTarget::Imu);
            assert!(
                !imu.lock().unwrap().has_fault(),
                "Cycle {}: fault should be cleared",
                cycle
            );
        }
    }

    #[test]
    fn test_different_fault_types_per_sensor() {
        let instance = 85;
        let (ctrl, imu, baro, mag, gnss) = setup_fault_ctrl(instance);

        // Apply different fault types to each sensor
        assert!(ctrl.apply_imu_fault(&FaultSpec::BiasShift {
            offset: [0.1, 0.2, 0.3]
        }));
        assert!(ctrl.apply_baro_fault(&FaultSpec::BiasScalar { offset: 50.0 }));
        assert!(ctrl.apply_mag_fault(&FaultSpec::NaN));
        assert!(ctrl.apply_gnss_fault(&FaultSpec::Dropout { cycles: 10 }));

        // All should have faults
        assert!(imu.lock().unwrap().has_fault());
        assert!(baro.lock().unwrap().has_fault());
        assert!(mag.lock().unwrap().has_fault());
        assert!(gnss.lock().unwrap().has_fault());

        // Clear only IMU
        ctrl.clear_sensor(SensorTarget::Imu);
        assert!(!imu.lock().unwrap().has_fault());
        assert!(baro.lock().unwrap().has_fault()); // still has fault
        assert!(mag.lock().unwrap().has_fault()); // still has fault
        assert!(gnss.lock().unwrap().has_fault()); // still has fault
    }
}
