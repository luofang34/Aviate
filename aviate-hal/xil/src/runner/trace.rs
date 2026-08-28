//! CSV output for one mission trace.

use crate::mission::PhaseResult;

use super::criteria::quat_to_rpy;

/// Write the per-step flight trace to a CSV.
///
/// The file contains one row for each backend frame.
/// Each row contains time, phase, action, position, velocity, attitude,
/// and angular velocity. This function replaces the file for each mission.
pub(super) fn write_trace_csv(
    path: &std::path::Path,
    mission_name: &str,
    phases: &[PhaseResult],
) -> std::io::Result<()> {
    use std::io::Write;
    let mut f = std::fs::File::create(path)?;
    writeln!(
        f,
        "# mission={}\nt_s,phase,action,x_ned_m,y_ned_m,z_ned_m,vx_mps,vy_mps,vz_mps,qw,qx,qy,qz,roll_deg,pitch_deg,yaw_deg,p_radps,q_radps,r_radps"
    , mission_name)?;
    let mut t_offset = 0.0f32;
    for phase in phases {
        // Replace delimiters because the debug value can contain commas.
        let action = phase
            .action_tag
            .replace([',', '\n', '\r'], ";")
            .replace("  ", " ");
        for s in &phase.trace {
            let (roll, pitch, yaw) = quat_to_rpy(s.attitude);
            writeln!(
                f,
                "{:.4},{},{},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.6},{:.6},{:.6},{:.6},{:.3},{:.3},{:.3},{:.4},{:.4},{:.4}",
                t_offset + s.elapsed,
                phase.name,
                action,
                s.position[0], s.position[1], s.position[2],
                s.velocity[0], s.velocity[1], s.velocity[2],
                s.attitude[0], s.attitude[1], s.attitude[2], s.attitude[3],
                roll.to_degrees(), pitch.to_degrees(), yaw.to_degrees(),
                s.angular_velocity[0], s.angular_velocity[1], s.angular_velocity[2],
            )?;
        }
        t_offset += phase.duration_actual.as_secs_f32();
    }
    Ok(())
}
