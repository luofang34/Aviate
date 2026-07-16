//! Genuine cross-PROCESS coherence.
//!
//! The single-process tests map the same object twice inside one
//! process, where a compiler could in principle reason about both
//! sides. Production is two binaries: the gz plugin publishes while
//! the FC reads and commands. These re-execute the test binary as a
//! real child process so neither side can be reasoned about by the
//! other's compiler.

use aviate_xil_contract::WriterState;

use super::super::{ConsumerSession, FcSession, ModelStateSnapshot, SimWriterSession};
use super::unique_name;
use crate::AttachFailure;

const CROSS_PROC_ENV: &str = "AVXT_CROSS_PROC_NAME";
const CROSS_PROC_MOTOR_ENV: &str = "AVXT_CROSS_PROC_MOTOR";
/// Where the child records that it actually ran its assertions.
const CROSS_PROC_DONE_ENV: &str = "AVXT_CROSS_PROC_DONE";

/// A child that never ran — a renamed or moved test, so `--exact`
/// matches nothing — exits 0, which a parent waiting only on exit
/// status reads as success. The child therefore has to leave proof
/// of work behind, and the parent has to demand it.
fn done_marker() -> std::path::PathBuf {
    std::env::temp_dir().join(format!("avxt_done_{}", std::process::id()))
}

fn mark_done() {
    if let Ok(path) = std::env::var(CROSS_PROC_DONE_ENV) {
        std::fs::write(path, b"done").expect("child must record its proof of work");
    }
}

/// The child half. A normal `cargo test` run reaches this without
/// the env var set and returns immediately; only the parent below
/// re-executes it with a target block.
#[test]
fn cross_process_child_reader() {
    let Ok(name) = std::env::var(CROSS_PROC_ENV) else {
        return;
    };
    let reader = ConsumerSession::attach(&name).expect("child must attach to the parent's block");
    // Generous: a torn read fails on the very first sample, so this
    // deadline only ever forgives scheduling — CI runners are slow,
    // contended, and run this under coverage instrumentation while
    // the sibling cross-process test spins its own two processes.
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(180);
    // Count DISTINCT, strictly advancing steps. Counting reads would
    // let this pass by re-reading one frozen snapshot 5000 times —
    // which is exactly what a broken publisher looks like.
    let mut distinct = 0u32;
    let mut last_step = 0u64;
    while distinct < 2_000 {
        assert!(
            std::time::Instant::now() < deadline,
            "child saw only {distinct} distinct steps before timing out"
        );
        if let Some(s) = reader.read_model_state() {
            // Every lane is a pure function of sim_step, so any
            // mixing of two publications is arithmetic, not luck.
            let v = s.sim_step as f64;
            assert_eq!(
                s.time_us,
                s.sim_step * 1000,
                "torn step/time across processes"
            );
            assert_eq!(s.pos, [v, v + 1.0, v + 2.0], "torn pos across processes");
            assert_eq!(s.vel, [v, v, v], "torn vel across processes");
            assert_eq!(s.reset_generation, 1, "torn generation across processes");
            assert!(
                s.sim_step >= last_step,
                "the world went backwards: {} after {last_step}",
                s.sim_step
            );
            if s.sim_step > last_step {
                distinct += 1;
                last_step = s.sim_step;
            }
        }
    }
    mark_done();
}

/// The motor half, in the child: attach as the FC and publish
/// commands whose lanes are fixed multiples of a counter.
#[test]
fn cross_process_child_motor_writer() {
    let Ok(name) = std::env::var(CROSS_PROC_MOTOR_ENV) else {
        return;
    };
    let fc = FcSession::attach(&name).expect("child FC must attach");
    // Publish until the PARENT has seen enough and kills us. A fixed
    // iteration budget makes the parent race the child's lifetime:
    // on a slower, CPU-contended runner the child finishes its quota
    // and exits before a starved parent has sampled its quota, and
    // the test fails for scheduling reasons rather than for a
    // protocol defect. The consumer decides when it is done; this
    // deadline is only a safety net against an abandoned child —
    // kept just above the parent's own budget so a parent that dies
    // on an assertion cannot leave a process spinning a CI core.
    let safety = std::time::Instant::now() + std::time::Duration::from_secs(150);
    let mut i = 0u64;
    while std::time::Instant::now() < safety {
        i = i.wrapping_add(1);
        let v = i as f64;
        fc.write_motor_command(&[v, v * 2.0, v * 3.0, v * 4.0]);
        fc.ack_step(i);
    }
}

#[test]
fn cross_process_reader_never_sees_a_torn_snapshot() {
    if std::env::var(CROSS_PROC_ENV).is_ok() || std::env::var(CROSS_PROC_MOTOR_ENV).is_ok() {
        return; // this process IS a child; do not recurse
    }
    let name = unique_name("xp");
    let marker = done_marker();
    std::fs::remove_file(&marker).ok();
    let writer = SimWriterSession::create(&name).unwrap();

    let mut child = std::process::Command::new(std::env::current_exe().expect("test binary path"))
        .args([
            "--exact",
            "roles::tests::cross_process::cross_process_child_reader",
            "--nocapture",
        ])
        .env(CROSS_PROC_ENV, &name)
        .env(CROSS_PROC_DONE_ENV, &marker)
        .spawn()
        .expect("spawn the reader child");

    let mut i = 0u64;
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(240);
    loop {
        match child.try_wait().expect("poll the child") {
            Some(status) => {
                assert!(status.success(), "cross-process reader failed: {status}");
                assert!(
                    marker.exists(),
                    "the child exited 0 without running its assertions — \
                     a filtered-out test name would pass this vacuously"
                );
                std::fs::remove_file(&marker).ok();
                break;
            }
            None => {
                assert!(
                    std::time::Instant::now() < deadline,
                    "child never finished; killing"
                );
                i = i.wrapping_add(1);
                let v = i as f64;
                writer.write_model_state(&ModelStateSnapshot {
                    reset_generation: 1,
                    sim_step: i,
                    time_us: i * 1000,
                    pos: [v, v + 1.0, v + 2.0],
                    quat: [1.0, 0.0, 0.0, 0.0],
                    vel: [v, v, v],
                    ang_vel: [0.0; 3],
                });
            }
        }
    }
}

#[test]
fn cross_process_motor_commands_are_coherent() {
    // The FC-writes / simulator-reads direction, genuinely across
    // processes: the motor WRITER runs in the child and this process
    // reads as the simulation writer. The earlier version of this
    // test ran BOTH motor ends in the parent and only the state
    // reader in the child — it proved nothing about the motor
    // seqlock across an address-space boundary, despite its name.
    if std::env::var(CROSS_PROC_ENV).is_ok() || std::env::var(CROSS_PROC_MOTOR_ENV).is_ok() {
        return;
    }
    let name = unique_name("xm");
    let writer = SimWriterSession::create(&name).unwrap();

    let mut child = std::process::Command::new(std::env::current_exe().expect("test binary path"))
        .args([
            "--exact",
            "roles::tests::cross_process::cross_process_child_motor_writer",
            "--nocapture",
        ])
        .env(CROSS_PROC_MOTOR_ENV, &name)
        .spawn()
        .expect("spawn the motor-writer child");

    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(120);
    let mut distinct = 0u32;
    let mut last_first = 0.0_f64;
    while distinct < 2_000 {
        assert!(
            std::time::Instant::now() < deadline,
            "saw only {distinct} distinct motor commands from the child"
        );
        // A child that died is a failure to report, not a reason to
        // spin until the deadline and blame timing.
        if let Some(status) = child.try_wait().expect("poll the motor child") {
            panic!(
                "the motor-writer child exited early ({status}) after {distinct} distinct commands"
            );
        }
        if let Some((lanes, n)) = writer.read_motor_command() {
            if lanes[0] == 0.0 {
                continue; // nothing published yet
            }
            assert_eq!(n, 4, "torn lane count across processes");
            assert_eq!(
                lanes[1],
                lanes[0] * 2.0,
                "torn motor lanes across processes"
            );
            assert_eq!(
                lanes[2],
                lanes[0] * 3.0,
                "torn motor lanes across processes"
            );
            assert_eq!(
                lanes[3],
                lanes[0] * 4.0,
                "torn motor lanes across processes"
            );
            if lanes[0] != last_first {
                distinct += 1;
                last_first = lanes[0];
            }
        }
    }
    assert!(
        writer.fc_step_ack() > 0,
        "the child FC must be acking steps"
    );

    child.kill().ok();
    child.wait().ok();
}

// ---------------------------------------------------------------
// Writer death. A crash means the PROCESS died: the kernel releases
// the writer lease while every in-block signal — name, size,
// fingerprint, ready flag, incarnation — survives and keeps
// describing a perfectly healthy world. Only a real child process
// can model that; `mem::forget` in-process keeps the lease fd open
// and the lease (correctly) still reports the writer alive.
// ---------------------------------------------------------------

const CROSS_PROC_CRASH_ENV: &str = "AVXT_CROSS_PROC_CRASH";
/// Path the parent watches to know the child's block is up.
const CROSS_PROC_READY_ENV: &str = "AVXT_CROSS_PROC_READY";
/// Path whose appearance tells the child to crash.
const CROSS_PROC_GO_ENV: &str = "AVXT_CROSS_PROC_GO";

/// The crashing writer, in the child: create, publish one snapshot,
/// signal readiness, wait for the go-file, then die WITHOUT any
/// cleanup — no ready-clear, no unlink, no lease release except the
/// kernel's own on process death.
#[test]
fn cross_process_child_crashing_writer() {
    let Ok(name) = std::env::var(CROSS_PROC_CRASH_ENV) else {
        return;
    };
    let ready_path = std::env::var(CROSS_PROC_READY_ENV).expect("ready path");
    let go_path = std::env::var(CROSS_PROC_GO_ENV).expect("go path");

    let writer = SimWriterSession::create(&name).expect("child writer must create");
    writer.write_model_state(&ModelStateSnapshot {
        reset_generation: 1,
        sim_step: 9,
        time_us: 9_000,
        pos: [7.0, 7.0, 7.0],
        quat: [1.0, 0.0, 0.0, 0.0],
        vel: [0.0; 3],
        ang_vel: [0.0; 3],
    });
    std::fs::write(&ready_path, b"up").expect("signal readiness");

    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(120);
    while !std::path::Path::new(&go_path).exists() {
        assert!(
            std::time::Instant::now() < deadline,
            "no go signal; refusing to spin forever"
        );
        std::thread::sleep(std::time::Duration::from_millis(5));
    }
    // Crash. abort() runs no destructors: the lease is released by
    // the KERNEL, not by Drop — which is the entire point.
    std::process::abort();
}

#[test]
fn a_crashed_writer_is_gone_then_replaced_after_restart() {
    if std::env::var(CROSS_PROC_ENV).is_ok()
        || std::env::var(CROSS_PROC_MOTOR_ENV).is_ok()
        || std::env::var(CROSS_PROC_CRASH_ENV).is_ok()
    {
        return; // this process IS a child; do not recurse
    }
    let name = unique_name("cr");
    let ready_path = std::env::temp_dir().join(format!("avxt_up_{}", std::process::id()));
    let go_path = std::env::temp_dir().join(format!("avxt_go_{}", std::process::id()));
    std::fs::remove_file(&ready_path).ok();
    std::fs::remove_file(&go_path).ok();

    let mut child = std::process::Command::new(std::env::current_exe().expect("test binary path"))
        .args([
            "--exact",
            "roles::tests::cross_process::cross_process_child_crashing_writer",
            "--nocapture",
        ])
        .env(CROSS_PROC_CRASH_ENV, &name)
        .env(CROSS_PROC_READY_ENV, &ready_path)
        .env(CROSS_PROC_GO_ENV, &go_path)
        .spawn()
        .expect("spawn the crashing writer");

    // Wait for the child's world, then attach and verify it is live.
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(120);
    while !ready_path.exists() {
        assert!(
            std::time::Instant::now() < deadline,
            "child never brought its world up"
        );
        std::thread::sleep(std::time::Duration::from_millis(5));
    }
    let consumer = ConsumerSession::attach(&name).expect("attach to the live child world");
    assert_eq!(consumer.writer_state(), WriterState::Current);
    assert!(consumer.read_model_state().is_some());

    // Kill it — and PROVE the death was a crash, not a clean exit.
    std::fs::write(&go_path, b"die").expect("send go signal");
    let status = child.wait().expect("reap the child");
    assert!(
        !status.success(),
        "the child must have died by abort, not exited cleanly: {status}"
    );

    // Crash WITHOUT replacement: no in-block signal changed, yet the
    // writer must read as Gone — this is the case a boolean or an
    // incarnation-only check calls healthy forever.
    assert_eq!(
        consumer.writer_state(),
        WriterState::Gone,
        "a dead writer's intact block must not read as Current"
    );
    // And a NEW consumer must not be allowed to bind to the corpse.
    assert!(
        matches!(ConsumerSession::attach(&name), Err(AttachFailure::NotReady)),
        "attaching to a dead writer's block must be refused"
    );

    // Crash-restart: a fresh writer takes the (kernel-released)
    // lease and creates a new object; the consumer still holding
    // the corpse's mapping sees the identity change and re-attaches
    // into the new world.
    let fresh = SimWriterSession::create(&name).expect("restart over a crashed predecessor");
    assert_eq!(
        consumer.writer_state(),
        WriterState::Replaced,
        "after a restart the pre-crash attachment must see a new identity"
    );
    let reattached = ConsumerSession::attach(&name).expect("re-attach to the restarted world");
    assert_eq!(reattached.writer_state(), WriterState::Current);
    assert_eq!(reattached.reset_generation(), 1);
    assert_eq!(
        reattached.read_model_state(),
        None,
        "the fresh world has published no snapshot yet"
    );
    drop(fresh);

    std::fs::remove_file(&ready_path).ok();
    std::fs::remove_file(&go_path).ok();
}
