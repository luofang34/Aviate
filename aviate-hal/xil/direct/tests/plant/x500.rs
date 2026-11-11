//! Deterministic 6-DoF X500 quadrotor plant shared by the direct-board integration tests.

use aviate_xil_direct::{DirectActuatorCmd, DirectMeasuredKinematics, DirectTruthSample};
use world_core::{normal_gravity, GeodeticPosition};

/// Deterministic 6-DoF rigid-body quadrotor plant matched to `build_x500_kernel()`.
pub struct QuadX500Plant {
    /// Position in local NED meters.
    pub pos_ned: [f32; 3],
    /// Velocity in local NED meters per second.
    pub vel_ned: [f32; 3],
    /// Unit quaternion `[w, x, y, z]` rotating body FRD into world NED.
    pub quat_wxyz: [f32; 4],
    omega_frd: [f32; 3],
    specific_force_frd: [f32; 3],
    gravity_mps2: f32,
    mass_kg: f32,
}

impl QuadX500Plant {
    /// Place the vehicle at rest on the pad at `origin`.
    pub fn new_on_pad(origin: GeodeticPosition) -> Self {
        let g = normal_gravity(origin).expect("valid origin gravity") as f32;
        let weight_n = 20.25_f32;
        Self {
            pos_ned: [0.0, 0.0, 0.0],
            vel_ned: [0.0, 0.0, 0.0],
            quat_wxyz: [1.0, 0.0, 0.0, 0.0],
            omega_frd: [0.0, 0.0, 0.0],
            specific_force_frd: [0.0, 0.0, -g],
            gravity_mps2: g,
            mass_kg: weight_n / g,
        }
    }

    /// Return the current state as a truth sample.
    pub fn truth(&self, time_us: u64, generation: u32, step: u64) -> DirectTruthSample {
        DirectTruthSample {
            time_us,
            reset_generation: generation,
            step,
            position_ned_m: self.pos_ned,
            attitude_wxyz: self.quat_wxyz,
            measured: Some(DirectMeasuredKinematics {
                velocity_ned_mps: self.vel_ned,
                body_rates_frd_rps: self.omega_frd,
                specific_force_frd_mps2: self.specific_force_frd,
            }),
        }
    }

    /// Integrate the rigid body for `dt_s` seconds under `cmd`.
    pub fn step(&mut self, cmd: &DirectActuatorCmd, dt_s: f32) {
        let max_per_rotor_n = 34.19_f32 * 0.25;
        let thrusts = if cmd.armed {
            [
                cmd.outputs[0].clamp(0.0, 1.0) * max_per_rotor_n,
                cmd.outputs[1].clamp(0.0, 1.0) * max_per_rotor_n,
                cmd.outputs[2].clamp(0.0, 1.0) * max_per_rotor_n,
                cmd.outputs[3].clamp(0.0, 1.0) * max_per_rotor_n,
            ]
        } else {
            [0.0; 4]
        };
        let total_thrust_n = thrusts[0] + thrusts[1] + thrusts[2] + thrusts[3];
        let arm_m = 0.174_f32;
        let k_yaw_m = 0.016_f32;
        let tau_x = arm_m * (-thrusts[0] + thrusts[1] + thrusts[2] - thrusts[3]);
        let tau_y = arm_m * (thrusts[0] - thrusts[1] + thrusts[2] - thrusts[3]);
        let tau_z = k_yaw_m * (thrusts[0] + thrusts[1] - thrusts[2] - thrusts[3]);

        let inertia = [0.022_f32, 0.022_f32, 0.040_f32];
        let ang_acc = [
            (tau_x - 0.02 * self.omega_frd[0]) / inertia[0],
            (tau_y - 0.02 * self.omega_frd[1]) / inertia[1],
            (tau_z - 0.02 * self.omega_frd[2]) / inertia[2],
        ];
        for (omega, &alpha) in self.omega_frd.iter_mut().zip(&ang_acc) {
            *omega += alpha * dt_s;
        }
        self.quat_wxyz = integrate_quat(self.quat_wxyz, self.omega_frd, dt_s);

        let drag_ned = [
            -0.15 * self.vel_ned[0],
            -0.15 * self.vel_ned[1],
            -0.15 * self.vel_ned[2],
        ];
        let thrust_ned = rotate_body_to_world(self.quat_wxyz, [0.0, 0.0, -total_thrust_n]);
        let mut acc_ned = [
            (thrust_ned[0] + drag_ned[0]) / self.mass_kg,
            (thrust_ned[1] + drag_ned[1]) / self.mass_kg,
            (thrust_ned[2] + drag_ned[2]) / self.mass_kg + self.gravity_mps2,
        ];

        for ((pos, vel), &acc) in self
            .pos_ned
            .iter_mut()
            .zip(self.vel_ned.iter_mut())
            .zip(&acc_ned)
        {
            *vel += acc * dt_s;
            *pos += *vel * dt_s;
        }
        if self.pos_ned[2] >= 0.0 && self.vel_ned[2] >= 0.0 {
            self.pos_ned[2] = 0.0;
            self.vel_ned = [0.0, 0.0, 0.0];
            self.omega_frd = [0.0, 0.0, 0.0];
            self.quat_wxyz = [1.0, 0.0, 0.0, 0.0];
            acc_ned = [0.0, 0.0, 0.0];
        }

        let sp_ned = [acc_ned[0], acc_ned[1], acc_ned[2] - self.gravity_mps2];
        self.specific_force_frd =
            aviate_xil_direct::synthesis::rotate_world_to_body(self.quat_wxyz, sp_ned);
    }
}

fn rotate_body_to_world(q_wxyz: [f32; 4], v_body: [f32; 3]) -> [f32; 3] {
    let [w, x, y, z] = q_wxyz;
    aviate_xil_direct::synthesis::rotate_world_to_body([w, -x, -y, -z], v_body)
}

fn integrate_quat(q: [f32; 4], omega: [f32; 3], dt_s: f32) -> [f32; 4] {
    let [w, x, y, z] = q;
    let [p, q_rate, r] = omega;
    let nw = w + 0.5 * dt_s * (-x * p - y * q_rate - z * r);
    let nx = x + 0.5 * dt_s * (w * p + y * r - z * q_rate);
    let ny = y + 0.5 * dt_s * (w * q_rate - x * r + z * p);
    let nz = z + 0.5 * dt_s * (w * r + x * q_rate - y * p);
    let norm = (nw * nw + nx * nx + ny * ny + nz * nz).sqrt();
    [nw / norm, nx / norm, ny / norm, nz / norm]
}
