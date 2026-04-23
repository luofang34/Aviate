//! Invariant checks for DO-178C formal verification.
//!
//! These verify that kernel state is internally consistent across the
//! check categories (pre-arm vs faults, sample counts monotonicity, etc).
//! Exercised in debug builds / verification harnesses, not the hot loop.

use super::in_flight::{InFlightFlags, InFlightStatus};
use super::pre_arm::{PreArmFlags, PreArmStatus, SampleCounts};
use crate::fault::FaultFlags;

/// Invariant verification for DO-178C compliance
///
/// These checks verify that the system state is internally consistent.
/// They are run in debug builds and can be used for formal verification.
pub struct CheckInvariants;

// COV:EXCL_START(FORMAL: invariant checks for DO-178C formal verification, not unit tested)
impl CheckInvariants {
    /// INV-001: ALL_IMU_FAILED fault implies !IMU_HEALTHY pre-arm flag
    ///
    /// If all IMUs have failed, we cannot have IMU_HEALTHY set.
    pub fn check_imu_consistency(faults: FaultFlags, pre_arm: &PreArmStatus) -> bool {
        if faults.contains(FaultFlags::ALL_IMU_FAILED) {
            !pre_arm.current.contains(PreArmFlags::IMU_HEALTHY)
        } else {
            true // Consistent by default if fault not present
        }
    }

    /// INV-002: ALL_GNSS_LOST fault implies !GNSS_AVAILABLE pre-arm flag
    ///
    /// If GNSS is completely lost, GNSS_AVAILABLE should not be set.
    pub fn check_gnss_consistency(faults: FaultFlags, pre_arm: &PreArmStatus) -> bool {
        if faults.contains(FaultFlags::ALL_GNSS_LOST) {
            !pre_arm.current.contains(PreArmFlags::GNSS_AVAILABLE)
        } else {
            true
        }
    }

    /// INV-003: NO_FAULTS pre-arm flag iff faults.is_empty()
    ///
    /// The NO_FAULTS flag must be consistent with the actual fault state.
    pub fn check_no_faults_consistency(faults: FaultFlags, pre_arm: &PreArmStatus) -> bool {
        let has_no_faults_flag = pre_arm.current.contains(PreArmFlags::NO_FAULTS);
        let actually_no_faults = faults.is_empty();
        has_no_faults_flag == actually_no_faults
    }

    /// INV-004: EKF_CONVERGED implies IMU_CONVERGED
    ///
    /// The EKF cannot converge without IMU data converging first.
    pub fn check_ekf_convergence_consistency(pre_arm: &PreArmStatus) -> bool {
        if pre_arm.current.contains(PreArmFlags::EKF_CONVERGED) {
            pre_arm.current.contains(PreArmFlags::IMU_CONVERGED)
        } else {
            true
        }
    }

    /// INV-005: POSITION_VALID in-flight implies ATTITUDE_VALID
    ///
    /// Position estimate requires a valid attitude estimate.
    pub fn check_position_attitude_consistency(in_flight: &InFlightStatus) -> bool {
        if in_flight.current.contains(InFlightFlags::POSITION_VALID) {
            in_flight.current.contains(InFlightFlags::ATTITUDE_VALID)
        } else {
            true
        }
    }

    /// INV-006: Sample counts must be monotonically increasing (except on reset)
    ///
    /// This invariant is checked by comparing with previous sample counts.
    pub fn check_sample_count_monotonic(prev: &SampleCounts, curr: &SampleCounts) -> bool {
        // Counts should be >= previous unless they were reset to 0
        let imu_ok = curr.imu >= prev.imu || curr.imu == 0;
        let baro_ok = curr.baro >= prev.baro || curr.baro == 0;
        let mag_ok = curr.mag >= prev.mag || curr.mag == 0;
        let gnss_ok = curr.gnss >= prev.gnss || curr.gnss == 0;
        imu_ok && baro_ok && mag_ok && gnss_ok
    }

    /// Run all state consistency checks
    ///
    /// Returns true if all invariants hold, false if any is violated.
    pub fn verify_all(
        faults: FaultFlags,
        pre_arm: &PreArmStatus,
        in_flight: &InFlightStatus,
    ) -> bool {
        Self::check_imu_consistency(faults, pre_arm)
            && Self::check_gnss_consistency(faults, pre_arm)
            && Self::check_no_faults_consistency(faults, pre_arm)
            && Self::check_ekf_convergence_consistency(pre_arm)
            && Self::check_position_attitude_consistency(in_flight)
    }

    /// Get a bitmask of which invariants are violated
    ///
    /// Each bit corresponds to an invariant (bit 0 = INV-001, etc.)
    pub fn get_violations(
        faults: FaultFlags,
        pre_arm: &PreArmStatus,
        in_flight: &InFlightStatus,
    ) -> u8 {
        let mut violations = 0u8;
        if !Self::check_imu_consistency(faults, pre_arm) {
            violations |= 1 << 0; // INV-001
        }
        if !Self::check_gnss_consistency(faults, pre_arm) {
            violations |= 1 << 1; // INV-002
        }
        if !Self::check_no_faults_consistency(faults, pre_arm) {
            violations |= 1 << 2; // INV-003
        }
        if !Self::check_ekf_convergence_consistency(pre_arm) {
            violations |= 1 << 3; // INV-004
        }
        if !Self::check_position_attitude_consistency(in_flight) {
            violations |= 1 << 4; // INV-005
        }
        violations
    }
}
// COV:EXCL_STOP
