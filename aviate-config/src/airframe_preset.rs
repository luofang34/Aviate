//! `AirframePresetV1` — the single versioned airframe data contract
//! (#75, consolidating #120's config side).
//!
//! A preset selects *data* within the compiled controller family; it
//! never selects Rust types at runtime and can never supply an
//! `ALGORITHM_ID` (`deny_unknown_fields` rejects the attempt). The
//! app maps [`MixerKind`] onto its compiled mixer types and fails
//! startup on anything it does not recognize.
//!
//! Parsing follows this crate's DAL rule: TOML once at startup,
//! validated to a typed value, or fail to arm.

use alloc::string::String;
use serde::Deserialize;

/// Registered mixer geometries a preset may name. The app resolves
/// the variant to a compiled mixer type; the variant (not the TOML
/// string) is what must reach the kernel's canonical hash so lockstep
/// witnesses cover the selected geometry.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum MixerKind {
    /// Generic quad-X (CW on FR+RL diagonal).
    QuadX,
    /// PX4-gazebo-models X500 pattern (CW on FL+RR diagonal) —
    /// opposite yaw signs from [`MixerKind::QuadX`].
    QuadXX500,
}

impl MixerKind {
    /// Motor count this geometry drives.
    pub fn motor_count(self) -> u8 {
        match self {
            MixerKind::QuadX | MixerKind::QuadXX500 => 4,
        }
    }
}

/// Actuator curve between the cascade's normalized-thrust output and
/// the boundary command (#140): `quadratic` plants (gz rotors, most
/// ESC+prop stacks) need `cmd = sqrt(thrust)` at the boundary.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum ActuatorCurve {
    /// Thrust proportional to command.
    Linear,
    /// Thrust proportional to command² (rotor-speed commands).
    Quadratic,
}

/// Per-axis cascade tuning as plain data. Field-for-field mirror of
/// `aviate_core::control::cascade_gains::CascadeGains`; the app
/// converts after validation.
#[derive(Debug, Clone, Copy, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct GainsPreset {
    /// Position loop P gains \[1/s\], X/Y/Z.
    pub pos_p: [f32; 3],
    /// Position loop accel limits \[m/s²\], X/Y/Z.
    pub pos_accel_limits: [f32; 3],
    /// Position loop velocity caps \[m/s\], X/Y/Z.
    pub pos_vel_caps: [f32; 3],
    /// Velocity loop P gains, X/Y/Z.
    pub vel_p: [f32; 3],
    /// Velocity loop I gains, X/Y/Z.
    pub vel_i: [f32; 3],
    /// Velocity loop D gains, X/Y/Z.
    pub vel_d: [f32; 3],
    /// Max roll/pitch tilt the velocity loop may command \[rad\].
    pub vel_max_roll_pitch: f32,
    /// Acceleration feedforward scale \[0..1\].
    pub vel_accel_ff: f32,
    /// Attitude loop P gains \[1/s\], roll/pitch/yaw.
    pub att_p: [f32; 3],
    /// Rate loop P gains, roll/pitch/yaw.
    pub rate_p: [f32; 3],
    /// Rate loop D gains, roll/pitch/yaw.
    pub rate_d: [f32; 3],
    /// Rate D-term LPF coefficient \[0..1\].
    pub rate_d_lpf_alpha: f32,
}

/// Flight-envelope limits as plain data. Mirror of
/// `aviate_core::control::Limits`.
#[derive(Debug, Clone, Copy, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LimitsPreset {
    /// Max roll angle \[rad\].
    pub max_roll: f32,
    /// Max pitch angle \[rad\].
    pub max_pitch: f32,
    /// Max roll rate \[rad/s\].
    pub max_roll_rate: f32,
    /// Max pitch rate \[rad/s\].
    pub max_pitch_rate: f32,
    /// Max yaw rate \[rad/s\].
    pub max_yaw_rate: f32,
    /// Max horizontal speed \[m/s\].
    pub max_horizontal_speed: f32,
    /// Max climb rate \[m/s\].
    pub max_climb_rate: f32,
    /// Max descent rate \[m/s\].
    pub max_descent_rate: f32,
    /// Geofence ceiling \[m\].
    pub max_altitude: f32,
    /// Geofence floor \[m\].
    pub min_altitude: f32,
}

/// The versioned airframe preset.
#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AirframePresetV1 {
    /// Must be `1`. A future incompatible layout bumps the number and
    /// gets its own struct; a v1 parser refuses anything else.
    pub schema_version: u32,
    /// Human-readable airframe name (logs/telemetry).
    pub name: String,
    /// Registered mixer geometry.
    pub mixer: MixerKind,
    /// Number of motors; must agree with the mixer geometry.
    pub motor_count: u8,
    /// Hover trim seed. Domain is defined by `actuator_curve` until
    /// #140 lands the NormalizedThrust contract end to end; see the
    /// preset file comments.
    pub hover_thrust_seed: f32,
    /// Plant curve between cascade output and boundary command.
    pub actuator_curve: ActuatorCurve,
    /// Cascade tuning.
    pub gains: GainsPreset,
    /// Flight envelope.
    pub limits: LimitsPreset,
}

/// Preset validation failure. Every variant names the offending
/// field so a bad preset is diagnosable from the abort message.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum PresetError {
    /// `schema_version` is not the supported version.
    UnsupportedSchema {
        /// Version found in the file.
        found: u32,
    },
    /// A numeric field is NaN/Inf or outside its allowed range.
    FieldOutOfRange {
        /// Dotted path of the offending field.
        field: &'static str,
    },
    /// `motor_count` does not match the mixer geometry.
    MotorCountMismatch {
        /// Count declared in the preset.
        declared: u8,
        /// Count the mixer geometry requires.
        required: u8,
    },
}

impl core::fmt::Display for PresetError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            PresetError::UnsupportedSchema { found } => {
                write!(f, "unsupported preset schema_version {found} (expected 1)")
            }
            PresetError::FieldOutOfRange { field } => {
                write!(f, "preset field {field} is non-finite or out of range")
            }
            PresetError::MotorCountMismatch { declared, required } => {
                write!(
                    f,
                    "motor_count {declared} does not match mixer geometry (requires {required})"
                )
            }
        }
    }
}

impl AirframePresetV1 {
    /// Validate the preset. Called by the loader; apps must not use a
    /// preset that fails here (fail to arm, not fly-with-defaults).
    pub fn validate(&self) -> Result<(), PresetError> {
        if self.schema_version != 1 {
            return Err(PresetError::UnsupportedSchema {
                found: self.schema_version,
            });
        }
        let required = self.mixer.motor_count();
        if self.motor_count != required {
            return Err(PresetError::MotorCountMismatch {
                declared: self.motor_count,
                required,
            });
        }
        if !self.hover_thrust_seed.is_finite()
            || self.hover_thrust_seed <= 0.0
            || self.hover_thrust_seed >= 1.0
        {
            return Err(PresetError::FieldOutOfRange {
                field: "hover_thrust_seed",
            });
        }
        self.validate_gains()?;
        self.validate_limits()
    }

    fn validate_gains(&self) -> Result<(), PresetError> {
        let g = &self.gains;
        let triples: [(&'static str, &[f32; 3]); 8] = [
            ("gains.pos_p", &g.pos_p),
            ("gains.pos_accel_limits", &g.pos_accel_limits),
            ("gains.pos_vel_caps", &g.pos_vel_caps),
            ("gains.vel_p", &g.vel_p),
            ("gains.vel_i", &g.vel_i),
            ("gains.vel_d", &g.vel_d),
            ("gains.att_p", &g.att_p),
            ("gains.rate_p", &g.rate_p),
        ];
        for (name, t) in triples {
            if t.iter().any(|v| !v.is_finite() || *v < 0.0) {
                return Err(PresetError::FieldOutOfRange { field: name });
            }
        }
        if g.rate_d.iter().any(|v| !v.is_finite() || *v < 0.0) {
            return Err(PresetError::FieldOutOfRange {
                field: "gains.rate_d",
            });
        }
        if !g.vel_max_roll_pitch.is_finite() || g.vel_max_roll_pitch <= 0.0 {
            return Err(PresetError::FieldOutOfRange {
                field: "gains.vel_max_roll_pitch",
            });
        }
        for (name, v) in [
            ("gains.vel_accel_ff", g.vel_accel_ff),
            ("gains.rate_d_lpf_alpha", g.rate_d_lpf_alpha),
        ] {
            if !v.is_finite() || !(0.0..=1.0).contains(&v) {
                return Err(PresetError::FieldOutOfRange { field: name });
            }
        }
        Ok(())
    }

    fn validate_limits(&self) -> Result<(), PresetError> {
        let l = &self.limits;
        let positives: [(&'static str, f32); 8] = [
            ("limits.max_roll", l.max_roll),
            ("limits.max_pitch", l.max_pitch),
            ("limits.max_roll_rate", l.max_roll_rate),
            ("limits.max_pitch_rate", l.max_pitch_rate),
            ("limits.max_yaw_rate", l.max_yaw_rate),
            ("limits.max_horizontal_speed", l.max_horizontal_speed),
            ("limits.max_climb_rate", l.max_climb_rate),
            ("limits.max_descent_rate", l.max_descent_rate),
        ];
        for (name, v) in positives {
            if !v.is_finite() || v <= 0.0 {
                return Err(PresetError::FieldOutOfRange { field: name });
            }
        }
        if !l.max_altitude.is_finite()
            || !l.min_altitude.is_finite()
            || l.max_altitude <= l.min_altitude
        {
            return Err(PresetError::FieldOutOfRange {
                field: "limits.max_altitude/min_altitude",
            });
        }
        Ok(())
    }
}

/// Parse and validate a preset from TOML text.
///
/// # Errors
///
/// Returns the TOML parse error (unknown fields included, via
/// `deny_unknown_fields`) or the first [`PresetError`] as a string —
/// startup code aborts arming on any of them.
pub fn preset_from_toml_str(text: &str) -> Result<AirframePresetV1, alloc::string::String> {
    use alloc::string::ToString;
    let preset: AirframePresetV1 = toml::from_str(text).map_err(|e| e.to_string())?;
    preset.validate().map_err(|e| e.to_string())?;
    Ok(preset)
}
