//! Bounded sensor perturbations at the flight-controller input seam.

use sha2::{Digest as _, Sha256};

use crate::sim_types::SimSensorPacket;

use super::{PerturbationError, PerturbationIdentity};

const SENSOR_NOISE_DOMAIN: &[u8] = b"pilotage-sensor-noise-v1";
const MAX_UPDATE_INTERVAL_SAMPLES: u32 = 100_000;

/// One physical sensor lane in flight-controller body-FRD units.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum SensorLane {
    /// Accelerometer X in m/s².
    AccelerometerX = 0,
    /// Accelerometer Y in m/s².
    AccelerometerY = 1,
    /// Accelerometer Z in m/s².
    AccelerometerZ = 2,
    /// Gyroscope X in rad/s.
    GyroscopeX = 3,
    /// Gyroscope Y in rad/s.
    GyroscopeY = 4,
    /// Gyroscope Z in rad/s.
    GyroscopeZ = 5,
    /// Magnetometer X in µT.
    MagnetometerX = 6,
    /// Magnetometer Y in µT.
    MagnetometerY = 7,
    /// Magnetometer Z in µT.
    MagnetometerZ = 8,
    /// Absolute pressure in Pa.
    AbsolutePressure = 9,
    /// Differential pressure in Pa.
    DifferentialPressure = 10,
    /// Pressure altitude in m.
    PressureAltitude = 11,
}

impl SensorLane {
    const fn index(self) -> usize {
        self as usize
    }

    const fn presence_bit(self) -> u16 {
        1_u16 << (self as u8)
    }

    const fn maximum_amplitude(self) -> f32 {
        match self {
            Self::AccelerometerX | Self::AccelerometerY | Self::AccelerometerZ => 20.0,
            Self::GyroscopeX | Self::GyroscopeY | Self::GyroscopeZ => 10.0,
            Self::MagnetometerX | Self::MagnetometerY | Self::MagnetometerZ => 200.0,
            Self::AbsolutePressure | Self::DifferentialPressure => 20_000.0,
            Self::PressureAltitude => 2_000.0,
        }
    }
}

/// One deterministic bounded-noise request in physical SI units.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SensorNoise {
    /// Target sensor lane.
    pub lane: SensorLane,
    /// Peak absolute amplitude in the lane unit.
    pub peak_amplitude: f32,
    /// Number of global samples in one zero-order-hold bucket.
    pub update_interval_samples: u32,
}

/// Sensor evidence for one global sample.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct SensorApplication {
    /// Digest of the converted input before perturbation.
    pub raw_digest: [u8; 32],
    /// Digest of the input supplied to the flight controller.
    pub effective_digest: [u8; 32],
    /// Converted input bits in stable sensor-lane order.
    pub raw_value_bits: [Option<u32>; 12],
    /// Flight-controller input bits in stable sensor-lane order.
    pub effective_value_bits: [Option<u32>; 12],
    /// Raw presence bits supplied by the backend.
    pub presence_mask: u16,
    /// Lanes whose finite values changed.
    pub changed_mask: u16,
    /// Global update bucket for each configured and present lane.
    pub update_buckets: [Option<u64>; 12],
}

pub(super) struct SensorEngine {
    identity: PerturbationIdentity,
    requests: Vec<SensorNoise>,
}

impl SensorEngine {
    pub(super) fn new(
        identity: PerturbationIdentity,
        requests: Vec<SensorNoise>,
    ) -> Result<Self, PerturbationError> {
        validate_requests(&requests)?;
        Ok(Self { identity, requests })
    }

    pub(super) fn apply(
        &mut self,
        sequence: u64,
        packet: &mut SimSensorPacket,
    ) -> Result<SensorApplication, PerturbationError> {
        validate_presence(packet)?;
        let raw_digest = digest_packet(packet);
        let raw_value_bits = sensor_value_bits(packet);
        let mut effective = *packet;
        let mut changed_mask = 0;
        let mut update_buckets = [None; 12];
        for request in &self.requests {
            apply_request(
                self.identity,
                sequence,
                &mut effective,
                *request,
                &mut changed_mask,
                &mut update_buckets,
            )?;
        }
        let application = SensorApplication {
            raw_digest,
            effective_digest: digest_packet(&effective),
            raw_value_bits,
            effective_value_bits: sensor_value_bits(&effective),
            presence_mask: packet.presence_mask,
            changed_mask,
            update_buckets,
        };
        *packet = effective;
        Ok(application)
    }
}

fn validate_presence(packet: &SimSensorPacket) -> Result<(), PerturbationError> {
    let accel = packet.presence_mask & 0b111;
    let gyro = packet.presence_mask & (0b111 << 3);
    let imu_present = accel == 0b111 && gyro == 0b111 << 3;
    if accel != 0 && accel != 0b111 || gyro != 0 && gyro != 0b111 << 3 {
        return Err(PerturbationError::SensorPresenceMismatch("IMU"));
    }
    if packet.imu.is_some() != imu_present {
        return Err(PerturbationError::SensorPresenceMismatch("IMU"));
    }
    let mag = packet.presence_mask & (0b111 << 6);
    if mag != 0 && mag != 0b111 << 6 {
        return Err(PerturbationError::SensorPresenceMismatch("magnetometer"));
    }
    if packet.mag.is_some() != (mag == 0b111 << 6) {
        return Err(PerturbationError::SensorPresenceMismatch("magnetometer"));
    }
    validate_pressure_presence(packet)
}

fn validate_pressure_presence(packet: &SimSensorPacket) -> Result<(), PerturbationError> {
    let static_present = packet.presence_mask & (1 << 9) != 0;
    let dynamic_present = packet.presence_mask & (1 << 10) != 0;
    let altitude_present = packet.presence_mask & (1 << 11) != 0;
    if packet.baro.is_some() != static_present
        || (dynamic_present || altitude_present) && !static_present
    {
        return Err(PerturbationError::SensorPresenceMismatch("pressure"));
    }
    let Some(baro) = packet.baro else {
        return Ok(());
    };
    if baro.differential_pressure_pa.is_some() != dynamic_present
        || baro.pressure_altitude_m.is_some() != altitude_present
    {
        Err(PerturbationError::SensorPresenceMismatch("pressure"))
    } else {
        Ok(())
    }
}

fn validate_requests(requests: &[SensorNoise]) -> Result<(), PerturbationError> {
    for (index, request) in requests.iter().enumerate() {
        if requests[..index]
            .iter()
            .any(|prior| prior.lane == request.lane)
        {
            return Err(PerturbationError::DuplicateSensorLane(request.lane));
        }
        if !request.peak_amplitude.is_finite()
            || request.peak_amplitude <= 0.0
            || request.peak_amplitude > request.lane.maximum_amplitude()
        {
            return Err(PerturbationError::InvalidSensorAmplitude(request.lane));
        }
        if !(1..=MAX_UPDATE_INTERVAL_SAMPLES).contains(&request.update_interval_samples) {
            return Err(PerturbationError::InvalidSensorInterval(request.lane));
        }
    }
    Ok(())
}

fn apply_request(
    identity: PerturbationIdentity,
    sequence: u64,
    packet: &mut SimSensorPacket,
    request: SensorNoise,
    changed_mask: &mut u16,
    update_buckets: &mut [Option<u64>; 12],
) -> Result<(), PerturbationError> {
    if packet.presence_mask & request.lane.presence_bit() == 0 {
        return Ok(());
    }
    let Some(value) = lane_value_mut(packet, request.lane) else {
        return Err(PerturbationError::SensorPresenceMismatch(
            "requested sensor lane",
        ));
    };
    if !value.is_finite() {
        return Err(PerturbationError::NonFiniteSensor(request.lane));
    }
    let raw_bits = value.to_bits();
    let bucket = sequence / u64::from(request.update_interval_samples);
    let offset = bounded_offset(identity, request.lane, bucket, request.peak_amplitude);
    *value += offset;
    if value.to_bits() != raw_bits {
        *changed_mask |= request.lane.presence_bit();
    }
    update_buckets[request.lane.index()] = Some(bucket);
    Ok(())
}

fn bounded_offset(
    identity: PerturbationIdentity,
    lane: SensorLane,
    bucket: u64,
    amplitude: f32,
) -> f32 {
    let mut hasher = Sha256::new();
    hasher.update(SENSOR_NOISE_DOMAIN);
    hasher.update(identity.condition_digest);
    hasher.update(identity.run_seed.to_le_bytes());
    hasher.update([lane as u8]);
    hasher.update(bucket.to_le_bytes());
    let bytes = hasher.finalize();
    let sample = u32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]);
    let unit = f64::from(sample) / f64::from(u32::MAX);
    ((unit.mul_add(2.0, -1.0)) * f64::from(amplitude)) as f32
}

fn lane_value_mut(packet: &mut SimSensorPacket, lane: SensorLane) -> Option<&mut f32> {
    match lane {
        SensorLane::AccelerometerX => packet.imu.as_mut().map(|value| &mut value.accel[0]),
        SensorLane::AccelerometerY => packet.imu.as_mut().map(|value| &mut value.accel[1]),
        SensorLane::AccelerometerZ => packet.imu.as_mut().map(|value| &mut value.accel[2]),
        SensorLane::GyroscopeX => packet.imu.as_mut().map(|value| &mut value.gyro[0]),
        SensorLane::GyroscopeY => packet.imu.as_mut().map(|value| &mut value.gyro[1]),
        SensorLane::GyroscopeZ => packet.imu.as_mut().map(|value| &mut value.gyro[2]),
        SensorLane::MagnetometerX => packet.mag.as_mut().map(|value| &mut value.field_ut[0]),
        SensorLane::MagnetometerY => packet.mag.as_mut().map(|value| &mut value.field_ut[1]),
        SensorLane::MagnetometerZ => packet.mag.as_mut().map(|value| &mut value.field_ut[2]),
        SensorLane::AbsolutePressure => packet.baro.as_mut().map(|value| &mut value.pressure_pa),
        SensorLane::DifferentialPressure => packet
            .baro
            .as_mut()
            .and_then(|value| value.differential_pressure_pa.as_mut()),
        SensorLane::PressureAltitude => packet
            .baro
            .as_mut()
            .and_then(|value| value.pressure_altitude_m.as_mut()),
    }
}

fn digest_packet(packet: &SimSensorPacket) -> [u8; 32] {
    let mut hasher = Sha256::new();
    hasher.update(b"aviate-effective-sensor-v1");
    hasher.update(packet.presence_mask.to_le_bytes());
    for value in sensor_values(packet) {
        hasher.update(value.map_or(0_u32, f32::to_bits).to_le_bytes());
    }
    hasher.finalize().into()
}

fn sensor_value_bits(packet: &SimSensorPacket) -> [Option<u32>; 12] {
    sensor_values(packet).map(|value| value.map(f32::to_bits))
}

fn sensor_values(packet: &SimSensorPacket) -> [Option<f32>; 12] {
    let imu = packet.imu;
    let mag = packet.mag;
    let baro = packet.baro;
    [
        imu.map(|value| value.accel[0]),
        imu.map(|value| value.accel[1]),
        imu.map(|value| value.accel[2]),
        imu.map(|value| value.gyro[0]),
        imu.map(|value| value.gyro[1]),
        imu.map(|value| value.gyro[2]),
        mag.map(|value| value.field_ut[0]),
        mag.map(|value| value.field_ut[1]),
        mag.map(|value| value.field_ut[2]),
        baro.map(|value| value.pressure_pa),
        baro.and_then(|value| value.differential_pressure_pa),
        baro.and_then(|value| value.pressure_altitude_m),
    ]
}
