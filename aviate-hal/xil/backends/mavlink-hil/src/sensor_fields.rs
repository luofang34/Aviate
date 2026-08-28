//! Presence decoding for `HIL_SENSOR` lanes.

/// The `HIL_SENSOR.fields_updated` lane bitmap.
#[derive(Debug, Clone, Copy)]
pub struct SensorFields(u32);

impl SensorFields {
    const ACCEL: u32 = 0b111;
    const GYRO: u32 = 0b111 << 3;
    const MAG: u32 = 0b111 << 6;
    const BARO: u32 = 1 << 9;
    const DIFFERENTIAL_PRESSURE: u32 = 1 << 10;
    const PRESSURE_ALTITUDE: u32 = 1 << 11;

    /// Decode a bitmap. Zero declares all lanes for simulator compatibility.
    #[must_use]
    pub fn from_bits(bits: u32) -> Self {
        Self(if bits == 0 { u32::MAX } else { bits })
    }

    /// Return true when all accelerometer and gyroscope lanes are present.
    #[must_use]
    pub fn imu(self) -> bool {
        self.0 & Self::ACCEL == Self::ACCEL && self.0 & Self::GYRO == Self::GYRO
    }

    /// Return true when all magnetometer lanes are present.
    #[must_use]
    pub fn mag(self) -> bool {
        self.0 & Self::MAG == Self::MAG
    }

    /// Return true when static pressure is present.
    #[must_use]
    pub fn baro(self) -> bool {
        self.0 & Self::BARO != 0
    }

    /// Return true when differential pressure is present.
    #[must_use]
    pub fn differential_pressure(self) -> bool {
        self.0 & Self::DIFFERENTIAL_PRESSURE != 0
    }

    /// Return true when pressure altitude is present.
    #[must_use]
    pub fn pressure_altitude(self) -> bool {
        self.0 & Self::PRESSURE_ALTITUDE != 0
    }

    pub(super) fn known_presence_mask(self) -> u16 {
        (self.0 & 0x0fff) as u16
    }
}

#[cfg(test)]
mod tests;
