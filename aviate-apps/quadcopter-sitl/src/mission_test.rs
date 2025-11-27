//! Mission Test Runner
//!
//! This binary runs mission-based tests under lockstep simulation.
//!
//! Usage:
//!   # Start Gazebo with lockstep world:
//!   LOCKSTEP=1 HEADLESS=1 ./scripts/launch_gazebo.sh
//!
//!   # Run missions:
//!   ./target/debug/mission-test [--mission <name>]
//!
//! Available missions:
//!   - basic_takeoff_land (default)
//!   - hover_hold

#[cfg(feature = "gz-plugin")]
use aviate_app_quadcopter_sitl::{Mission, run_mission_suite};

fn main() {
    println!("Aviate Mission Test Runner");
    println!("==========================");
    println!();

    #[cfg(not(feature = "gz-plugin"))]
    {
        eprintln!("Error: gz-plugin feature not enabled");
        eprintln!("Build with: cargo build --features gz-plugin -p aviate-app-quadcopter-sitl");
        std::process::exit(1);
    }

    #[cfg(feature = "gz-plugin")]
    {
        run_missions();
    }
}

#[cfg(feature = "gz-plugin")]
fn run_missions() {
    // Parse command line args
    let args: Vec<String> = std::env::args().collect();
    let mission_name = if args.len() > 2 && args[1] == "--mission" {
        args[2].as_str()
    } else {
        "basic_takeoff_land"
    };

    // Select mission(s) to run
    let missions: Vec<Mission> = match mission_name {
        "basic_takeoff_land" => vec![Mission::basic_takeoff_land()],
        "hover_hold" => vec![Mission::hover_hold()],
        "all" => vec![Mission::basic_takeoff_land(), Mission::hover_hold()],
        _ => {
            eprintln!("Unknown mission: {}", mission_name);
            eprintln!("Available: basic_takeoff_land, hover_hold, all");
            std::process::exit(1);
        }
    };

    println!("Running {} mission(s):", missions.len());
    for m in &missions {
        println!("  - {} ({:?})", m.name, m.total_duration());
    }
    println!();

    // Run missions
    let results = run_mission_suite(&missions);

    // Summary
    println!();
    println!("=== Mission Suite Summary ===");
    let passed = results.iter().filter(|r| r.passed).count();
    let total = results.len();

    for result in &results {
        let status = if result.passed { "PASS" } else { "FAIL" };
        println!("  [{}] {} - max alt: {:.2}m, duration: {:.2}s",
            status,
            result.mission_name,
            result.max_altitude,
            result.total_duration.as_secs_f32()
        );
    }

    println!();
    println!("Results: {}/{} missions passed", passed, total);

    if passed == total {
        println!("ALL MISSIONS PASSED");
        std::process::exit(0);
    } else {
        println!("SOME MISSIONS FAILED");
        std::process::exit(1);
    }
}
