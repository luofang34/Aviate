# Aviate Configuration IDs

Per DO-178C Section 7.3 (Configuration Management), all software configurations must be documented and traceable.

**Status**: Development/Testing Configuration Only
**Date**: 2024-12-12
**Revision**: 0.1

## ⚠️  Development Notice

This document currently contains **development and testing configurations only**.

Production flight configurations will be added after:
- Hardware-in-the-loop testing
- Security key provisioning
- Flight certification review

## Allowed Feature Combinations (Development)

| Config ID | Crate | Features | Target | Purpose | Status |
|-----------|-------|----------|--------|---------|--------|
| CFG-DEV-001 | micoair-h743-v2-test | `[]` | stm32h743v | Hardware test (no security) | ✅ Active |
| CFG-DEV-002 | micoair-h743-v2-test | `["software-bootloader"]` | stm32h743v | Hardware test + USB DFU | ✅ Active |
| CFG-DEV-003 | sitl-gazebo-x500 | `[]` | x86_64-linux | SITL simulation | ✅ Active |

## Build Commands

### CFG-DEV-001: Basic Hardware Test
```bash
cd aviate-apps/micoair-h743-v2-test
cargo build --release --target thumbv7em-none-eabihf
```

**Purpose**: Sensor probing, I2C/SPI validation, LED testing
**Security**: None (test environment only)
**Telemetry**: USB CDC serial
**Commands**: Text-based UART commands

### CFG-DEV-002: Hardware Test + Bootloader
```bash
cd aviate-apps/micoair-h743-v2-test
cargo build --release --target thumbv7em-none-eabihf --features software-bootloader
```

**Purpose**: Same as CFG-DEV-001 + USB DFU support
**Security**: None (test environment only)
**Additional**: Protected reboot-to-bootloader command

### CFG-DEV-003: SITL Simulation
```bash
cd aviate-apps/sitl-gazebo-x500
cargo build --release
```

**Purpose**: Software-in-the-loop development and testing
**Security**: None (simulation environment)
**Telemetry**: MAVLink over UDP
**Commands**: MAVLink over UDP

## Planned Feature Combinations (Production - NOT YET IMPLEMENTED)

| Config ID | Crate | Features | Target | Purpose | Status |
|-----------|-------|----------|--------|---------|--------|
| CFG-PROD-001 | micoair-h743-v2 | `["secure-link"]` | stm32h743v | Production flight (OTP keys) | ⏸️  Planned |
| CFG-PROD-002 | micoair-h743-v2 | `["secure-link", "software-bootloader"]` | stm32h743v | Production + DFU | ⏸️  Planned |

### CFG-PROD-001: Production Flight Configuration (PLANNED)
```bash
cd aviate-apps/micoair-h743-v2
cargo build --release --target thumbv7em-none-eabihf --features secure-link
```

**Purpose**: Production flight operations
**Security**: HMAC-SHA256 command signing, OTP keys, anti-replay
**Telemetry**: MAVLink over USB/UART
**Commands**: Signed MAVLink only
**Status**: Requires OTP key provisioning and flight certification

## Prohibited Combinations

The following feature combinations are **explicitly prohibited**:

| Features | Reason | Risk Level |
|----------|--------|------------|
| `secure-link` + `debug` | Debug features may leak keys | **CRITICAL** |
| `secure-link` + `test` | Test keys in production build | **CRITICAL** |
| Production target without `secure-link` | Unsigned commands in flight | **HIGH** |

## CI Enforcement

Continuous Integration (CI) MUST:

1. **Only build documented configurations** from the tables above
2. **Fail if any prohibited combination is detected**
3. **Verify feature flag consistency** across workspace members
4. **Check that production builds use secure-link**

Example CI check:
```bash
#!/bin/bash
# Verify only documented configurations are built

ALLOWED_CONFIGS=(
    "micoair-h743-v2-test::"
    "micoair-h743-v2-test::software-bootloader"
    "sitl-gazebo-x500::"
)

# Parse build command and verify against ALLOWED_CONFIGS
# Exit 1 if configuration not documented
```

## Configuration Change Process

### Adding a New Configuration

1. **Document the configuration** in this file before building:
   - Add row to appropriate table (Development or Production)
   - Specify exact crate name, features, target, and purpose
   - Assign unique Config ID (e.g., CFG-DEV-004)

2. **Test the configuration**:
   - Build successfully
   - Run test suite
   - Verify functionality on target hardware/simulator

3. **Update CI** to include new configuration in allow-list

4. **Get review**:
   - Development configs: Technical review
   - Production configs: Safety review + certification approval

### Removing a Configuration

1. **Mark as deprecated** in this file (update Status column)
2. **Remove from CI** allow-list after 1 release cycle
3. **Delete from documentation** after 2 release cycles

## Critical Safety Rule

⚠️  **Any new feature combination MUST NOT be flown until**:

1. Added to this document with unique Config ID
2. Tested on hardware (for hardware configs) or simulator (for SITL configs)
3. Reviewed and approved by safety team (for production configs)
4. CI updated to build and test the configuration

**Violation of this rule is a DO-178C certification failure.**

## Version History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 0.1 | 2024-12-12 | Claude | Initial development configurations |

## Next Steps (Before Production)

- [ ] Provision OTP keys on production hardware
- [ ] Implement `secure-link` feature flag
- [ ] Add hardware-in-the-loop test configuration
- [ ] Define production key rotation procedure
- [ ] Add firmware signing verification configuration
- [ ] Complete DO-178C certification review

---

**Document Status**: Development Only
**Classification**: Unclassified / Internal Use
**Review Cycle**: Update before each release
