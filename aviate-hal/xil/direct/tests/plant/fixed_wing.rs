//! Point-mass fixed-wing plant for the direct-board integration tests.
//!
//! The model is the smallest one that has the behaviour the control law
//! must handle: a lift curve with a stall, a drag polar, a propeller whose
//! thrust falls with airspeed, first-order roll and pitch rate responses
//! whose authority grows with airspeed, pitch stability about a trim angle
//! of attack that needs a constant elevator deflection, and a yaw axis
//! with sideslip: directional stability, yaw damping, rudder power,
//! adverse yaw from the ailerons, side force and roll from sideslip.
//! There is no wind.
//!
//! Two parameter sets exist: a Cessna 172 class aeroplane and a Rascal
//! 110 class small unmanned aeroplane.

use aviate_xil_direct::{DirectActuatorCmd, DirectMeasuredKinematics, DirectTruthSample};

/// Constants of one aeroplane.
#[derive(Clone, Copy)]
pub struct PlantModel {
    mass_kg: f32,
    wing_area_m2: f32,
    lift_at_zero_alpha: f32,
    lift_slope_per_rad: f32,
    max_lift_coefficient: f32,
    zero_lift_drag: f32,
    induced_drag_factor: f32,
    thrust_power_w: f32,
    /// Airspeed below which the propeller thrust stops growing.
    static_thrust_speed_mps: f32,
    /// Airspeed at which the surface rates below are defined, and the
    /// trim airspeed with neutral elevator.
    pub reference_speed_mps: f32,
    /// Roll rate at full aileron at the reference speed.
    roll_rate_at_full_aileron_rps: f32,
    /// Pitch rate at full elevator at the reference speed.
    pitch_rate_at_full_elevator_rps: f32,
    /// Pitch rate per radian of angle of attack away from trim.
    pitch_stability_per_s: f32,
    rate_response_time_s: f32,
    /// Angle of attack that the vehicle holds with the elevator at neutral.
    trim_alpha_rad: f32,
    /// Throttle for level flight at the reference speed.
    pub trim_throttle: f32,
    /// Elevator, in the pilot's sense, that holds the trim angle of attack.
    pub trim_elevator: f32,
    /// Yaw acceleration per radian of sideslip at the reference speed.
    directional_stability_per_s2: f32,
    /// Yaw acceleration per rad/s of yaw rate at the reference speed.
    yaw_damping_per_s: f32,
    /// Yaw acceleration at full rudder at the reference speed.
    rudder_power_per_s2: f32,
    /// Yaw acceleration against the turn at full aileron at the reference speed.
    adverse_yaw_per_s2: f32,
    /// Course rate per radian of sideslip at the reference speed.
    side_force_per_s: f32,
    /// Roll rate per radian of sideslip.
    dihedral_per_s: f32,
}

impl PlantModel {
    /// 1043 kg, 16.2 m2, stall at 25 m/s indicated, 95 kW of thrust power.
    pub const C172: Self = Self {
        mass_kg: 1043.0,
        wing_area_m2: 16.2,
        lift_at_zero_alpha: 0.30,
        lift_slope_per_rad: 5.0,
        max_lift_coefficient: 1.6,
        zero_lift_drag: 0.032,
        induced_drag_factor: 0.055,
        thrust_power_w: 95_000.0,
        static_thrust_speed_mps: 20.0,
        reference_speed_mps: 55.0,
        roll_rate_at_full_aileron_rps: 0.9,
        pitch_rate_at_full_elevator_rps: 0.5,
        pitch_stability_per_s: 2.0,
        rate_response_time_s: 0.15,
        trim_alpha_rad: 0.008,
        trim_throttle: 0.667,
        trim_elevator: 0.12,
        directional_stability_per_s2: 9.0,
        yaw_damping_per_s: 1.2,
        rudder_power_per_s2: 2.3,
        adverse_yaw_per_s2: 0.5,
        side_force_per_s: 0.16,
        dihedral_per_s: 0.5,
    };

    /// 6.8 kg, 0.98 m2, stall at 9 m/s indicated, 500 W of thrust power.
    pub const RASCAL: Self = Self {
        mass_kg: 6.8,
        wing_area_m2: 0.98,
        lift_at_zero_alpha: 0.20,
        lift_slope_per_rad: 4.5,
        max_lift_coefficient: 1.3,
        zero_lift_drag: 0.035,
        induced_drag_factor: 0.060,
        thrust_power_w: 500.0,
        static_thrust_speed_mps: 8.0,
        reference_speed_mps: 22.0,
        roll_rate_at_full_aileron_rps: 2.0,
        pitch_rate_at_full_elevator_rps: 1.2,
        pitch_stability_per_s: 3.0,
        rate_response_time_s: 0.08,
        trim_alpha_rad: 0.0067,
        trim_throttle: 0.49,
        trim_elevator: 0.08,
        directional_stability_per_s2: 25.0,
        yaw_damping_per_s: 3.0,
        rudder_power_per_s2: 6.0,
        adverse_yaw_per_s2: 1.0,
        side_force_per_s: 0.3,
        dihedral_per_s: 1.0,
    };
}

/// State of the point-mass aeroplane.
pub struct FixedWingPlant {
    /// Position in local NED meters.
    pub pos_ned: [f32; 3],
    /// True airspeed in meters per second.
    pub airspeed: f32,
    /// Flight-path angle in radians, positive in a climb.
    pub path_angle: f32,
    /// Course in radians from north, positive to the east.
    pub course: f32,
    /// Heading in radians from north, positive to the east.
    pub heading: f32,
    /// Bank angle in radians, positive right wing down.
    pub roll: f32,
    /// Pitch angle in radians, positive nose up.
    pub pitch: f32,
    /// Load factor of the last step.
    pub load_factor: f32,
    /// Fraction of the propeller thrust that is available: 0 is a failed engine.
    pub engine: f32,
    /// Body roll rate in radians per second.
    pub roll_rate: f32,
    model: PlantModel,
    pitch_rate: f32,
    yaw_rate: f32,
    accel_ned: [f32; 3],
    gravity_mps2: f32,
}

fn air_density(height_m: f32) -> f32 {
    // Troposphere of the standard atmosphere.
    1.225 * (1.0 - 2.2558e-5 * height_m).powf(4.2559)
}

impl FixedWingPlant {
    /// Level flight due north at `height_m` and the reference speed.
    pub fn cruising(model: PlantModel, height_m: f32, gravity_mps2: f32) -> Self {
        Self {
            model,
            pos_ned: [0.0, 0.0, -height_m],
            airspeed: model.reference_speed_mps,
            path_angle: 0.0,
            course: 0.0,
            heading: 0.0,
            roll: 0.0,
            pitch: model.trim_alpha_rad,
            load_factor: 1.0,
            engine: 1.0,
            roll_rate: 0.0,
            pitch_rate: 0.0,
            yaw_rate: 0.0,
            accel_ned: [0.0; 3],
            gravity_mps2,
        }
    }

    /// Height above the origin in meters.
    pub fn height(&self) -> f32 {
        -self.pos_ned[2]
    }

    /// Indicated airspeed in meters per second.
    pub fn indicated_airspeed(&self) -> f32 {
        self.airspeed * (air_density(self.height()) / 1.225).sqrt()
    }

    /// Velocity in local NED meters per second.
    pub fn velocity_ned(&self) -> [f32; 3] {
        let horizontal = self.airspeed * self.path_angle.cos();
        [
            horizontal * self.course.cos(),
            horizontal * self.course.sin(),
            -self.airspeed * self.path_angle.sin(),
        ]
    }

    /// Sideslip in radians: positive when the airflow comes from the
    /// right of the nose, which is when the course is right of the heading.
    pub fn sideslip(&self) -> f32 {
        let slip = self.course - self.heading;
        slip.sin().atan2(slip.cos())
    }

    fn attitude_wxyz(&self) -> [f32; 4] {
        // ZYX Euler angles.
        let (sr, cr) = (0.5 * self.roll).sin_cos();
        let (sp, cp) = (0.5 * self.pitch).sin_cos();
        let (sy, cy) = (0.5 * self.heading).sin_cos();
        [
            cy * cp * cr + sy * sp * sr,
            cy * cp * sr - sy * sp * cr,
            cy * sp * cr + sy * cp * sr,
            sy * cp * cr - cy * sp * sr,
        ]
    }

    /// Return the current state as a truth sample.
    pub fn truth(&self, time_us: u64, step: u64) -> DirectTruthSample {
        let attitude = self.attitude_wxyz();
        let specific_force_ned = [
            self.accel_ned[0],
            self.accel_ned[1],
            self.accel_ned[2] - self.gravity_mps2,
        ];
        DirectTruthSample {
            time_us,
            reset_generation: 0,
            step,
            position_ned_m: self.pos_ned,
            attitude_wxyz: attitude,
            measured: Some(DirectMeasuredKinematics {
                velocity_ned_mps: self.velocity_ned(),
                body_rates_frd_rps: [self.roll_rate, self.pitch_rate, self.yaw_rate],
                specific_force_frd_mps2: aviate_xil_direct::synthesis::rotate_world_to_body(
                    attitude,
                    specific_force_ned,
                ),
            }),
        }
    }

    /// Integrate for `dt_s` seconds under `cmd`.
    ///
    /// A disarmed board commands nothing: the plant then flies with the
    /// trim throttle and trim elevator and with neutral aileron and rudder,
    /// as a pilot would hold it.
    pub fn step(&mut self, cmd: &DirectActuatorCmd, dt_s: f32) {
        let model = self.model;
        let surface = |lane: usize| (2.0 * cmd.outputs[lane] - 1.0).clamp(-1.0, 1.0);
        let (throttle, aileron, elevator, rudder) = if cmd.armed {
            (
                cmd.outputs[0].clamp(0.0, 1.0),
                surface(1),
                surface(2),
                surface(3),
            )
        } else {
            (model.trim_throttle, 0.0, model.trim_elevator, 0.0)
        };
        let g = self.gravity_mps2;
        let weight = model.mass_kg * g;
        let speed = self.airspeed.max(0.2 * model.reference_speed_mps);
        let dynamic_pressure = 0.5 * air_density(self.height()) * speed * speed;

        let cos_roll = self.roll.cos().max(0.3);
        let alpha = (self.pitch - self.path_angle) / cos_roll;
        let lift_coefficient = (model.lift_at_zero_alpha + model.lift_slope_per_rad * alpha)
            .clamp(-0.5, model.max_lift_coefficient);
        let lift = dynamic_pressure * model.wing_area_m2 * lift_coefficient;
        let drag = dynamic_pressure
            * model.wing_area_m2
            * (model.zero_lift_drag
                + model.induced_drag_factor * lift_coefficient * lift_coefficient);
        let thrust = self.engine * throttle * model.thrust_power_w
            / speed.max(model.static_thrust_speed_mps);
        self.load_factor = lift / weight;

        // Surface and stability moments grow with dynamic pressure; the
        // rate that a deflection holds, and the damping, grow with airspeed.
        let authority = speed / model.reference_speed_mps;
        let sideslip = self.sideslip();

        let speed_rate = (thrust - drag) / model.mass_kg - g * self.path_angle.sin();
        let path_rate =
            (lift * self.roll.cos() - weight * self.path_angle.cos()) / (model.mass_kg * speed);
        // The side force of the fuselage and fin turns the velocity toward the nose.
        let course_rate = lift * self.roll.sin() / (model.mass_kg * speed * self.path_angle.cos())
            - model.side_force_per_s * authority * sideslip;

        let turn_pitch_rate = course_rate * self.pitch.cos() * self.roll.sin();
        let roll_rate_target = model.roll_rate_at_full_aileron_rps * authority * aileron
            - model.dihedral_per_s * sideslip;
        let pitch_rate_target =
            model.pitch_rate_at_full_elevator_rps * authority * (elevator - model.trim_elevator)
                - model.pitch_stability_per_s * (alpha - model.trim_alpha_rad)
                + turn_pitch_rate;
        let blend = dt_s / model.rate_response_time_s;
        self.roll_rate += blend * (roll_rate_target - self.roll_rate);
        self.pitch_rate += blend * (pitch_rate_target - self.pitch_rate);
        let yaw_acceleration = authority
            * authority
            * (model.directional_stability_per_s2 * sideslip + model.rudder_power_per_s2 * rudder
                - model.adverse_yaw_per_s2 * aileron)
            - authority * model.yaw_damping_per_s * self.yaw_rate;
        self.yaw_rate += dt_s * yaw_acceleration;

        let velocity_before = self.velocity_ned();
        let (sin_roll, cos_roll) = self.roll.sin_cos();
        let heading_rate =
            (self.pitch_rate * sin_roll + self.yaw_rate * cos_roll) / self.pitch.cos();
        self.roll += dt_s * (self.roll_rate + self.pitch.sin() * heading_rate);
        self.pitch += dt_s * (self.pitch_rate * cos_roll - self.yaw_rate * sin_roll);
        self.heading += dt_s * heading_rate;
        self.airspeed += dt_s * speed_rate;
        self.path_angle += dt_s * path_rate;
        self.course += dt_s * course_rate;
        let velocity = self.velocity_ned();
        for axis in 0..3 {
            self.accel_ned[axis] = (velocity[axis] - velocity_before[axis]) / dt_s;
            self.pos_ned[axis] += dt_s * velocity[axis];
        }
    }
}
