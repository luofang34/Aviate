//! jMAVSim SITL Flight Controller Application
//!
//! This application runs the Aviate flight controller with jMAVSim simulator
//! using the standard MAVLink HIL protocol.
//!
//! ## Prerequisites
//!
//! jMAVSim must be built first:
//! ```bash
//! cd ~/jMAVSim && ant create_run_jar copy_res
//! ```
//!
//! ## Usage
//!
//! ```bash
//! # Automatic mode (starts jMAVSim automatically):
//! cargo run -p aviate-app-jmavsim-sitl
//!
//! # Manual mode (connect to already-running jMAVSim):
//! cargo run -p aviate-app-jmavsim-sitl -- --no-sim
//!
//! # Headless mode (no jMAVSim GUI):
//! cargo run -p aviate-app-jmavsim-sitl -- --headless
//!
//! # Auto-arm after 5 seconds:
//! cargo run -p aviate-app-jmavsim-sitl -- --auto-arm 5
//!
//! # Custom port:
//! cargo run -p aviate-app-jmavsim-sitl -- --port 14561
//!
//! # Custom jMAVSim directory:
//! cargo run -p aviate-app-jmavsim-sitl -- --jmavsim-dir /path/to/jMAVSim
//! ```
//!
//! ## Options
//!
//! - `--port`, `-p <PORT>`: Simulator UDP port (default: 14560)
//! - `--no-sim`: Don't start jMAVSim, connect to already-running instance
//! - `--headless`: Run jMAVSim without GUI window
//! - `--auto-arm [SECONDS]`: Auto-arm after delay (default: 5 seconds)
//! - `--jmavsim-dir <PATH>`: Path to jMAVSim directory (default: ~/jMAVSim)

use std::io::{BufRead, BufReader};
use std::process::{Child, Command, Stdio};
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use aviate_board_sitl_jmavsim::{JmavSimBoard, JmavSimConfig};

/// Global flag for graceful shutdown
static RUNNING: AtomicBool = AtomicBool::new(true);

/// Default jMAVSim location
const JMAVSIM_DIR: &str = concat!(env!("HOME"), "/jMAVSim");

fn main() {
    // Parse command line arguments
    let args: Vec<String> = std::env::args().collect();
    let port = parse_port(&args).unwrap_or(14560);
    let auto_arm_delay = parse_auto_arm(&args);
    let no_sim = args.iter().any(|a| a == "--no-sim");
    let headless = args.iter().any(|a| a == "--headless");
    let jmavsim_dir = parse_jmavsim_dir(&args).unwrap_or_else(|| JMAVSIM_DIR.to_string());

    println!("===========================================");
    println!("  Aviate jMAVSim SITL Flight Controller");
    println!("===========================================");
    println!();
    println!("Board: {}", JmavSimBoard::board_id());
    println!("Airframe: {}", JmavSimBoard::airframe_id());
    println!("Simulator port: {}", port);
    println!();

    // Start jMAVSim if requested
    let mut jmavsim_process: Option<Child> = None;
    if !no_sim {
        println!("[INFO] Starting jMAVSim...");
        match start_jmavsim(&jmavsim_dir, port, headless) {
            Ok(child) => {
                jmavsim_process = Some(child);
                println!(
                    "[INFO] jMAVSim started (PID: {})",
                    jmavsim_process.as_ref().map(|c| c.id()).unwrap_or(0)
                );
                // Give jMAVSim time to start up
                std::thread::sleep(Duration::from_secs(2));
            }
            Err(e) => {
                eprintln!("[ERROR] Failed to start jMAVSim: {}", e);
                eprintln!("[INFO] You can start jMAVSim manually with:");
                eprintln!(
                    "  cd {} && java -jar out/production/jmavsim_run.jar -udp {}",
                    jmavsim_dir, port
                );
                eprintln!("[INFO] Or run with --no-sim to connect to an existing simulator");
                std::process::exit(1);
            }
        }
    } else {
        println!("[INFO] Running in manual mode (--no-sim)");
        println!("[INFO] Start jMAVSim manually with:");
        println!(
            "  cd {} && java -jar out/production/jmavsim_run.jar -udp {}",
            jmavsim_dir, port
        );
    }

    // Create board configuration
    let config = JmavSimConfig {
        local_port: 0, // Ephemeral port
        simulator_port: port,
        ..Default::default()
    };

    // Create board
    let mut board = match JmavSimBoard::with_config(config) {
        Ok(b) => b,
        Err(e) => {
            eprintln!("[ERROR] Failed to create board: {}", e);
            cleanup_jmavsim(&mut jmavsim_process);
            std::process::exit(1);
        }
    };

    println!(
        "[INFO] Flight controller bound to port {}",
        board.local_port()
    );
    println!("[INFO] Sending handshake to jMAVSim...");

    // Send handshake to trigger jMAVSim connection
    for _ in 0..10 {
        board.send_handshake();
        std::thread::sleep(Duration::from_millis(100));
    }

    println!("[INFO] Waiting for sensor data...");
    println!();

    // Main loop
    let loop_period = Duration::from_micros(2500); // 400Hz
    let heartbeat_interval = Duration::from_secs(1); // 1Hz heartbeat
    let stats_interval = Duration::from_secs(5);
    let mut last_stats = Instant::now();
    let mut last_heartbeat = Instant::now();
    let mut iteration = 0u64;
    let start_time = Instant::now();
    let mut first_sensor = false;
    let mut armed = false;

    while RUNNING.load(Ordering::Relaxed) {
        let loop_start = Instant::now();

        // Send periodic heartbeat (1Hz) to keep jMAVSim connection alive
        if last_heartbeat.elapsed() >= heartbeat_interval {
            board.send_heartbeat();
            last_heartbeat = Instant::now();
        }

        // Run one control loop iteration
        board.step();
        iteration += 1;

        // Check for first sensor data
        if !first_sensor {
            let (rx, _, _) = board.stats();
            if rx > 0 {
                first_sensor = true;
                println!("[INFO] Receiving sensor data from jMAVSim!");
            }
        }

        // Auto-arm after delay if configured
        if let Some(delay_secs) = auto_arm_delay {
            if !armed && board.is_ready() && start_time.elapsed() > Duration::from_secs(delay_secs)
            {
                match board.arm() {
                    Ok(()) => {
                        armed = true;
                        println!("[INFO] Auto-armed after {}s delay", delay_secs);
                    }
                    Err(e) => {
                        eprintln!("[WARN] Auto-arm failed: {:?}", e);
                    }
                }
            }
        }

        // Print stats periodically
        if last_stats.elapsed() >= stats_interval {
            let (rx, tx, crc_errors) = board.stats();
            let elapsed = start_time.elapsed().as_secs_f32();
            let rate = iteration as f32 / elapsed;

            println!(
                "[STATS] iter={} rate={:.1}Hz rx={} tx={} crc_err={} ready={} armed={}",
                iteration,
                rate,
                rx,
                tx,
                crc_errors,
                board.is_ready(),
                board.is_armed()
            );
            last_stats = Instant::now();

            // Check if jMAVSim is still running
            if let Some(ref mut child) = jmavsim_process {
                match child.try_wait() {
                    Ok(Some(status)) => {
                        eprintln!("[WARN] jMAVSim exited with status: {}", status);
                        break;
                    }
                    Ok(None) => {} // Still running
                    Err(e) => {
                        eprintln!("[WARN] Error checking jMAVSim status: {}", e);
                    }
                }
            }
        }

        // Sleep to maintain loop rate
        let elapsed = loop_start.elapsed();
        if elapsed < loop_period {
            std::thread::sleep(loop_period - elapsed);
        }

        // Safety limit for testing
        if iteration > 1_000_000_000 {
            break;
        }
    }

    println!();
    println!("[INFO] Shutting down...");
    board.disarm();
    cleanup_jmavsim(&mut jmavsim_process);
    println!("[INFO] Goodbye!");
}

/// Start jMAVSim as a subprocess
fn start_jmavsim(jmavsim_dir: &str, port: u16, headless: bool) -> Result<Child, String> {
    let jar_path = format!("{}/out/production/jmavsim_run.jar", jmavsim_dir);

    // Check if jMAVSim JAR exists
    if !std::path::Path::new(&jar_path).exists() {
        return Err(format!(
            "jMAVSim JAR not found at {}. Please build jMAVSim first:\n  cd {} && ant create_run_jar copy_res",
            jar_path, jmavsim_dir
        ));
    }

    let mut cmd = Command::new("java");

    // Java options for compatibility
    cmd.args([
        "--add-exports",
        "java.base/java.lang=ALL-UNNAMED",
        "--add-exports",
        "java.desktop/sun.awt=ALL-UNNAMED",
        "--add-exports",
        "java.desktop/sun.java2d=ALL-UNNAMED",
        "-XX:GCTimeRatio=20",
        "-Djava.ext.dirs=",
        "-jar",
        &jar_path,
        "-udp",
        &port.to_string(),
    ]);

    if headless {
        cmd.arg("-no-gui");
    }

    // Set working directory
    cmd.current_dir(jmavsim_dir);

    // Capture output
    cmd.stdout(Stdio::piped());
    cmd.stderr(Stdio::piped());

    let mut child = cmd
        .spawn()
        .map_err(|e| format!("Failed to spawn jMAVSim: {}", e))?;

    // Spawn thread to read and print jMAVSim output
    if let Some(stdout) = child.stdout.take() {
        std::thread::spawn(move || {
            let reader = BufReader::new(stdout);
            for line in reader.lines().map_while(Result::ok) {
                println!("[jMAVSim] {}", line);
            }
        });
    }

    if let Some(stderr) = child.stderr.take() {
        std::thread::spawn(move || {
            let reader = BufReader::new(stderr);
            for line in reader.lines().map_while(Result::ok) {
                eprintln!("[jMAVSim] {}", line);
            }
        });
    }

    Ok(child)
}

/// Clean up jMAVSim process
fn cleanup_jmavsim(process: &mut Option<Child>) {
    if let Some(ref mut child) = process {
        println!("[INFO] Stopping jMAVSim...");
        let _ = child.kill();
        let _ = child.wait();
    }
}

fn parse_port(args: &[String]) -> Option<u16> {
    for (i, arg) in args.iter().enumerate() {
        if arg == "--port" || arg == "-p" {
            if let Some(val) = args.get(i + 1) {
                return val.parse().ok();
            }
        }
    }
    None
}

fn parse_auto_arm(args: &[String]) -> Option<u64> {
    for (i, arg) in args.iter().enumerate() {
        if arg == "--auto-arm" {
            if let Some(val) = args.get(i + 1) {
                if let Ok(v) = val.parse() {
                    return Some(v);
                }
            }
            // Default to 5 seconds if no value specified
            return Some(5);
        }
    }
    None
}

fn parse_jmavsim_dir(args: &[String]) -> Option<String> {
    for (i, arg) in args.iter().enumerate() {
        if arg == "--jmavsim-dir" {
            if let Some(val) = args.get(i + 1) {
                return Some(val.clone());
            }
        }
    }
    None
}
