//! Process Spawner
//!
//! Spawns and manages FC, simulator, and mavrouter processes for XIL testing.
//! Supports both SITL (Gazebo) and HITL modes.

#![allow(dead_code)] // Only used with gazebo feature

use std::env;
use std::path::{Path, PathBuf};
use std::process::{Child, Command, Stdio};
use std::time::Duration;
#[cfg(target_os = "linux")]
use std::time::Instant;

/// FC process configuration
pub struct FcConfig {
    pub binary_path: PathBuf,
    pub args: Vec<String>,
    pub instance: u8,
    pub headless: bool,
}

impl Default for FcConfig {
    fn default() -> Self {
        Self {
            binary_path: PathBuf::from("./target/debug/sitl-gazebo-x500"),
            args: vec!["--interactive".to_string()],
            instance: 0,
            headless: true,
        }
    }
}

/// Spawned process handle
pub struct ProcessHandle {
    pub child: Child,
    pub name: String,
}

impl ProcessHandle {
    pub fn kill(&mut self) {
        let _ = self.child.kill();
    }

    pub fn is_running(&mut self) -> bool {
        self.child.try_wait().ok().flatten().is_none()
    }
}

use log::info;

/// Gazebo spawner for SITL mode
pub struct GazeboSpawner {
    child: Option<Child>,
    world_path: Option<PathBuf>,
}

impl GazeboSpawner {
    pub fn new() -> Self {
        Self {
            child: None,
            world_path: None,
        }
    }

    /// Launch Gazebo with the specified world file
    pub fn launch(&mut self, world_path: &Path, headless: bool) -> Result<(), String> {
        // Clean up any existing processes first
        self.cleanup();

        let child = launch_gazebo(world_path, headless)
            .map_err(|e| format!("Failed to launch Gazebo: {}", e))?;

        info!(
            target: "gcs",
            "Gazebo started (PID: {}, headless={})",
            child.id(),
            headless
        );

        self.child = Some(child);
        self.world_path = Some(world_path.to_path_buf());

        Ok(())
    }

    // ... <skip to spawn_router>

    /// Wait for Gazebo shared memory to be ready
    pub fn wait_for_ready(&self, timeout: Duration) -> bool {
        wait_for_shm(timeout)
    }

    /// Check if Gazebo is running
    pub fn is_running(&mut self) -> bool {
        if let Some(ref mut child) = self.child {
            child.try_wait().ok().flatten().is_none()
        } else {
            false
        }
    }

    /// Cleanup Gazebo processes and shared memory
    pub fn cleanup(&mut self) {
        if let Some(ref mut child) = self.child {
            let _ = child.kill();
            // Give SIGKILL a moment to land before unlinking shm.
            std::thread::sleep(Duration::from_millis(200));
        }
        self.child = None;

        cleanup_gazebo_shm();
    }
}

impl Default for GazeboSpawner {
    fn default() -> Self {
        Self::new()
    }
}

impl Drop for GazeboSpawner {
    fn drop(&mut self) {
        self.cleanup();
    }
}

/// Spawner manages FC and simulator processes
pub struct Spawner {
    fc_handles: Vec<ProcessHandle>,
    gazebo: GazeboSpawner,
    router_handle: Option<ProcessHandle>,
}

impl Spawner {
    pub fn new() -> Self {
        Self {
            fc_handles: Vec::new(),
            gazebo: GazeboSpawner::new(),
            router_handle: None,
        }
    }

    /// Launch Gazebo with the specified world file
    pub fn launch_gazebo(&mut self, world_path: &Path, headless: bool) -> Result<(), String> {
        self.gazebo.launch(world_path, headless)
    }

    /// Wait for Gazebo to be ready
    pub fn wait_for_gazebo(&self, timeout: Duration) -> bool {
        self.gazebo.wait_for_ready(timeout)
    }

    /// Spawn an FC process
    pub fn spawn_fc(&mut self, config: &FcConfig) -> std::io::Result<()> {
        let mut cmd = Command::new(&config.binary_path);
        cmd.args(&config.args);

        if config.headless {
            cmd.env("HEADLESS", "1");
        }

        cmd.env("AVIATE_INSTANCE", config.instance.to_string());

        // Set LD_LIBRARY_PATH for Gazebo plugin
        let plugin_dir = env::current_dir()
            .unwrap_or_default()
            .join("aviate-hal/xil/backends/gz/plugin/build");
        if plugin_dir.exists() {
            let existing = env::var("LD_LIBRARY_PATH").unwrap_or_default();
            let combined = if existing.is_empty() {
                plugin_dir.to_string_lossy().to_string()
            } else {
                format!("{}:{}", plugin_dir.display(), existing)
            };
            cmd.env("LD_LIBRARY_PATH", combined);
        }

        cmd.stdout(Stdio::inherit());
        cmd.stderr(Stdio::inherit());

        let child = cmd.spawn()?;

        self.fc_handles.push(ProcessHandle {
            child,
            name: config
                .binary_path
                .file_name()
                .map(|s| s.to_string_lossy().to_string())
                .unwrap_or_else(|| "fc".to_string()),
        });

        Ok(())
    }

    /// Spawn mavrouter with the given config file
    pub fn spawn_router(&mut self, config_path: &Path) -> std::io::Result<()> {
        let mut cmd = Command::new("mavrouter");
        cmd.arg("--config").arg(config_path);
        cmd.stdout(Stdio::inherit());
        cmd.stderr(Stdio::inherit());

        let child = cmd.spawn()?;

        info!(target: "gcs", "mavrouter started (PID: {})", child.id());

        self.router_handle = Some(ProcessHandle {
            child,
            name: "mavrouter".to_string(),
        });

        Ok(())
    }

    /// Wait for FC to be ready
    pub fn wait_for_fc_ready(&self, _timeout: Duration) -> bool {
        // Minimal wait to allow process to start up.
        // Connection logic is handled by MavClient checking for heartbeats.
        std::thread::sleep(Duration::from_secs(2));
        true
    }

    /// Kill all spawned processes
    pub fn cleanup(&mut self) {
        // Kill FC processes
        for handle in &mut self.fc_handles {
            handle.kill();
        }
        self.fc_handles.clear();

        // Kill router
        if let Some(ref mut handle) = self.router_handle {
            handle.kill();
        }
        self.router_handle = None;

        // Cleanup Gazebo
        self.gazebo.cleanup();
    }
}

impl Drop for Spawner {
    fn drop(&mut self) {
        self.cleanup();
    }
}

impl Default for Spawner {
    fn default() -> Self {
        Self::new()
    }
}

// ============================================================================
// Gazebo Management Functions
// ============================================================================

/// Launch Gazebo with the specified world file
fn launch_gazebo(world_path: &Path, headless: bool) -> Result<Child, std::io::Error> {
    // Set up environment paths
    let aviate_dir = env::current_dir().unwrap_or_default();

    // Model path
    let local_models = aviate_dir.join("models");
    let px4_models = aviate_dir.join("external/PX4-gazebo-models/models");
    let mut model_paths = Vec::new();
    if local_models.exists() {
        model_paths.push(local_models.to_string_lossy().to_string());
    }
    if px4_models.exists() {
        model_paths.push(px4_models.to_string_lossy().to_string());
    }
    let gz_resource_path = model_paths.join(":");

    // Plugin path
    let plugin_dir = aviate_dir.join("aviate-hal/xil/backends/gz/plugin/build");
    let gz_plugin_path = plugin_dir.to_string_lossy().to_string();

    let mut cmd = Command::new("gz");
    cmd.arg("sim");

    if headless {
        cmd.arg("-s"); // Server only
        cmd.arg("-r"); // Run immediately
        cmd.arg("--headless-rendering");
        cmd.env_remove("DISPLAY");
    } else {
        cmd.arg("-r"); // Run immediately
    }

    cmd.arg(world_path);

    // Set environment
    if !gz_resource_path.is_empty() {
        let existing = env::var("GZ_SIM_RESOURCE_PATH").unwrap_or_default();
        let combined = if existing.is_empty() {
            gz_resource_path
        } else {
            format!("{}:{}", gz_resource_path, existing)
        };
        cmd.env("GZ_SIM_RESOURCE_PATH", combined);
    }

    if !gz_plugin_path.is_empty() {
        let existing = env::var("GZ_SIM_SYSTEM_PLUGIN_PATH").unwrap_or_default();
        let combined = if existing.is_empty() {
            gz_plugin_path.clone()
        } else {
            format!("{}:{}", gz_plugin_path, existing)
        };
        cmd.env("GZ_SIM_SYSTEM_PLUGIN_PATH", combined);
        cmd.env("LD_LIBRARY_PATH", gz_plugin_path);
    }

    cmd.stdout(Stdio::inherit());
    cmd.stderr(Stdio::inherit());

    cmd.spawn()
}

/// Clean up only the Linux shm mirror file (no broad `pkill`).
///
/// `cleanup` already kills the `Child` it spawned by PID; nuking
/// every `gz sim` on the host was hostile to parallel CI runs and
/// to developers running a separate Gazebo session. `/dev/shm/...`
/// only exists on Linux — on macOS POSIX shm is virtual.
///
/// If `cleanup` is called on a never-spawned spawner (or
/// best-effort recovery from a previous run), there's nothing to
/// kill and the shm mirror file is the only thing worth touching.
fn cleanup_gazebo_shm() {
    #[cfg(target_os = "linux")]
    {
        let _ = std::fs::remove_file("/dev/shm/aviate_gz_bridge");
    }
}

/// Wait for Gazebo shared memory to be ready.
///
/// On Linux POSIX shm is backed by a `/dev/shm/<name>` path, so we
/// can poll its existence. macOS has no such mirror — `shm_open()`
/// returns a virtual fd with no filesystem trace — so the only
/// signal we have is the gz-sim process being alive. Sleep a fixed
/// startup quantum and let the FC binary handle its own retry loop
/// (`GzPluginBridge::connect_with_retry`) once it starts.
fn wait_for_shm(timeout: Duration) -> bool {
    #[cfg(target_os = "linux")]
    {
        let shm_path = Path::new("/dev/shm/aviate_gz_bridge");
        let start = Instant::now();
        while start.elapsed() < timeout {
            if shm_path.exists() {
                return true;
            }
            std::thread::sleep(Duration::from_millis(250));
        }
        false
    }
    #[cfg(not(target_os = "linux"))]
    {
        // macOS / non-Linux: no /dev/shm. Cap the wait to the smaller
        // of the caller's timeout and a 3 s startup quantum, then
        // return true unconditionally — the FC binary will retry its
        // shm_open and surface a clearer error if the plugin is
        // genuinely not running.
        let warmup = Duration::from_secs(3).min(timeout);
        std::thread::sleep(warmup);
        true
    }
}
