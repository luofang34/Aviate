//! End-to-end tests over a real POSIX shm object: the role
//! endpoints play their production parts — SimWriter (plugin),
//! FcSession, HostSession, read-only Consumer — the exact
//! cross-process topology, minus the process boundary.

use super::*;
use core::sync::atomic::Ordering;

fn unique_name(tag: &str) -> String {
    // Distinct per test; PIDs keep parallel CI shards apart. Keep it
    // SHORT: macOS caps POSIX shm names at 31 characters (PSHMNAMLEN)
    // and rejects longer ones with ENAMETOOLONG.
    format!("/avxt_{}_{}", tag, std::process::id())
}

#[test]
fn attach_fails_closed_without_writer() {
    let err = ConsumerSession::attach("/avxt_none").unwrap_err();
    assert!(matches!(err, AttachFailure::Io(_)));
}

#[test]
fn valid_block_attaches_with_fresh_generation() {
    // The fingerprint gate's happy path over a real object; each
    // mismatch arm (magic / version / declared size / short object)
    // is pinned unit-level by the contract crate's validate_attach
    // tests, which every attach path calls verbatim.
    let name = unique_name("ok");
    let _writer = SimWriterSession::create(&name).unwrap();
    let reader = ConsumerSession::attach(&name).unwrap();
    assert!(reader.plugin_ready());
    assert_eq!(reader.reset_generation(), 1);
}

#[test]
fn attach_fails_closed_while_writer_absent_after_drop() {
    // plugin_ready is an attach precondition: once the writer
    // drops, a NEW attach is refused as NotReady/Io instead of
    // handing out a mapping whose writer is gone.
    let name = unique_name("rdy");
    let writer = SimWriterSession::create(&name).unwrap();
    drop(writer);
    assert!(ConsumerSession::attach(&name).is_err());
}

#[test]
fn model_state_round_trips_coherently() {
    let name = unique_name("st");
    let writer = SimWriterSession::create(&name).unwrap();
    let reader = ConsumerSession::attach(&name).unwrap();

    assert_eq!(
        reader.read_model_state(),
        None,
        "no snapshot before the first publish (valid=0)"
    );

    let snap = ModelStateSnapshot {
        reset_generation: 1,
        sim_step: 42,
        time_us: 42_000,
        pos: [1.0, 2.0, 3.0],
        quat: [1.0, 0.0, 0.0, 0.0],
        vel: [0.1, 0.2, 0.3],
        ang_vel: [0.01, 0.02, 0.03],
    };
    writer.write_model_state(&snap);
    assert_eq!(reader.read_model_state(), Some(snap));
}

#[test]
fn motor_command_round_trips() {
    let name = unique_name("cm");
    let writer = SimWriterSession::create(&name).unwrap();
    let fc = FcSession::attach(&name).unwrap();

    fc.write_motor_command(&[100.0, 200.0, 300.0, 400.0]);
    fc.ack_step(7);
    let (vels, n) = writer.read_motor_command().unwrap();
    assert_eq!(n, 4);
    assert_eq!(&vels[..4], &[100.0, 200.0, 300.0, 400.0]);
    assert_eq!(writer.fc_step_ack(), 7, "ack is the FC heartbeat");
}

#[test]
fn lifecycle_request_ack_ready_handshake() {
    let name = unique_name("lc");
    let sim = SimWriterSession::create(&name).unwrap();
    let host = HostSession::attach(&name).unwrap();
    let fc = FcSession::attach(&name).unwrap();

    // Host posts a reset request: one packed word, one nonce.
    let nonce = host.post_lifecycle_request(LifecycleRequest::Reset);
    assert_eq!(nonce, 1);

    // Sim side reads ONE coherent (nonce, request) pair — no hidden
    // read-order convention (#267) — performs the world reset, acks
    // only after success, bumps the generation.
    let (req_nonce, req) = sim.lifecycle_request();
    assert_eq!((req_nonce, req), (nonce, LifecycleRequest::Reset));
    assert_ne!(sim.lifecycle_ack_nonce(), req_nonce, "not yet acked");
    let generation = sim.bump_reset_generation();
    sim.set_lifecycle_ack_nonce(req_nonce);
    assert_eq!(generation, 2);

    // A duplicate poll sees nonce == ack nonce: complete/duplicate,
    // never re-executed.
    let (again_nonce, _) = sim.lifecycle_request();
    assert_eq!(again_nonce, sim.lifecycle_ack_nonce());

    // FC observes the generation change and walks its state machine;
    // status is one packed (generation, state) word.
    assert_eq!(fc.reset_generation(), 2);
    fc.set_fc_status(FcState::Resetting, 2);
    fc.set_fc_status(FcState::Converging, 2);
    fc.set_fc_status(FcState::Ready, 2);

    // Host sees ack + Ready-for-current-generation in ONE read:
    // reset complete.
    assert_eq!(host.lifecycle_ack_nonce(), nonce);
    let (status_generation, state) = host.fc_status();
    assert_eq!(state, FcState::Ready);
    assert_eq!(status_generation, host.reset_generation());
}

#[test]
fn time_controls_are_plain_shared_words() {
    let name = unique_name("tm");
    let sim = SimWriterSession::create(&name).unwrap();
    let host = HostSession::attach(&name).unwrap();

    assert!(!sim.lockstep_enabled(), "async by default");
    host.set_lockstep(true);
    host.set_target_rtf_percent(400);
    assert!(sim.lockstep_enabled());
    assert_eq!(sim.target_rtf_percent(), 400);
    host.set_target_rtf_percent(0); // as-fast-as-possible
    assert_eq!(sim.target_rtf_percent(), 0);
}

#[test]
fn concurrent_writer_reader_never_tear_across_the_mapping() {
    // The contract crate proves the seqlock in-process; this pins it
    // across two distinct mappings of the same object, where the
    // compiler can assume nothing.
    let name = unique_name("tr");
    let writer = SimWriterSession::create(&name).unwrap();
    let reader = ConsumerSession::attach(&name).unwrap();

    let stop = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false));
    let w = {
        let stop = stop.clone();
        std::thread::spawn(move || {
            let mut i = 0_u64;
            while !stop.load(Ordering::Relaxed) {
                i = i.wrapping_add(1);
                let v = i as f64;
                writer.write_model_state(&ModelStateSnapshot {
                    reset_generation: 1,
                    sim_step: i,
                    time_us: i * 1000,
                    pos: [v, v + 1.0, v + 2.0],
                    quat: [1.0, 0.0, 0.0, 0.0],
                    vel: [v, v, v],
                    ang_vel: [0.0, 0.0, 0.0],
                });
            }
        })
    };

    let mut seen = 0;
    while seen < 20_000 {
        if let Some(s) = reader.read_model_state() {
            let v = s.sim_step as f64;
            assert_eq!(s.time_us, s.sim_step * 1000, "torn step/time pair");
            assert_eq!(s.pos, [v, v + 1.0, v + 2.0], "torn pos payload");
            seen += 1;
        }
    }
    stop.store(true, Ordering::Relaxed);
    w.join().unwrap();
}

// ---------------------------------------------------------------
// Writer lifecycle: a departed or replaced writer must never keep
// feeding its final snapshot as if the world were still running.
// ---------------------------------------------------------------

#[test]
fn attach_mid_init_is_retryable_not_a_contract_mismatch() {
    // The writer zeroes the block, stamps the fingerprint, and only
    // then publishes readiness. An attacher landing inside that
    // window sees a zeroed header; if it validated the fingerprint
    // FIRST it would report BadMagic -> ContractMismatch, which
    // callers correctly refuse to retry — a permanent startup
    // failure for a normal microsecond-wide window.
    let name = unique_name("mi");
    let _mid_init = SimWriterSession::create_mid_init_for_test(&name).unwrap();
    match ConsumerSession::attach(&name) {
        Err(AttachFailure::NotReady) => {}
        other => panic!("mid-init attach must be retryable NotReady, got {other:?}"),
    }
}

#[test]
fn clean_writer_exit_stops_the_stream() {
    let name = unique_name("ce");
    let writer = SimWriterSession::create(&name).unwrap();
    writer.write_model_state(&ModelStateSnapshot {
        reset_generation: 1,
        sim_step: 5,
        time_us: 5_000,
        pos: [1.0, 2.0, 3.0],
        quat: [1.0, 0.0, 0.0, 0.0],
        vel: [0.0; 3],
        ang_vel: [0.0; 3],
    });
    let consumer = ConsumerSession::attach(&name).unwrap();
    assert!(consumer.read_model_state().is_some(), "live writer feeds");

    drop(writer);

    assert!(!consumer.plugin_ready());
    assert_eq!(
        consumer.read_model_state(),
        None,
        "a departed writer must not keep serving its last snapshot forever"
    );
}

#[test]
fn consumer_detects_a_replaced_writer_and_can_reattach() {
    // Writer CRASH (no cleanup): plugin_ready stays set in the
    // orphaned memory and the object stays alive because we still
    // map it, so `plugin_ready` alone cannot detect this. The new
    // writer creates a fresh object under the same name; the old
    // mapping would otherwise serve the dead world's last snapshot
    // forever.
    let name = unique_name("rp");
    let crashed = SimWriterSession::create(&name).unwrap();
    crashed.write_model_state(&ModelStateSnapshot {
        reset_generation: 1,
        sim_step: 9,
        time_us: 9_000,
        pos: [7.0, 7.0, 7.0],
        quat: [1.0, 0.0, 0.0, 0.0],
        vel: [0.0; 3],
        ang_vel: [0.0; 3],
    });
    let consumer = ConsumerSession::attach(&name).unwrap();
    assert!(consumer.read_model_state().is_some());
    assert!(!consumer.writer_replaced(), "same object, not replaced");

    // Model the crash: no Drop runs, so no ready-clear and no unlink.
    core::mem::forget(crashed);
    let _fresh = SimWriterSession::create(&name).unwrap();

    assert!(
        consumer.read_model_state().is_some(),
        "precondition: the orphaned mapping still looks perfectly valid"
    );
    assert!(
        consumer.writer_replaced(),
        "object identity changed: the consumer must be able to see it"
    );

    let reattached = ConsumerSession::attach(&name).unwrap();
    assert!(!reattached.writer_replaced());
    assert_eq!(reattached.reset_generation(), 1);
    assert_eq!(
        reattached.read_model_state(),
        None,
        "the fresh world has published no snapshot yet"
    );
}

// ---------------------------------------------------------------
// Genuine cross-PROCESS coherence. The tests above map the same
// object twice inside one process, where a compiler could in
// principle reason about both sides. Production is two binaries: the
// gz plugin publishes while the FC reads. This re-executes the test
// binary as a real child process so neither side can be reasoned
// about by the other's compiler.
// ---------------------------------------------------------------

const CROSS_PROC_ENV: &str = "AVXT_CROSS_PROC_NAME";

/// The child half. A normal `cargo test` run reaches this without
/// the env var set and returns immediately; only the parent below
/// re-executes it with a target block.
#[test]
fn cross_process_child_reader() {
    let Ok(name) = std::env::var(CROSS_PROC_ENV) else {
        return;
    };
    let reader = ConsumerSession::attach(&name).expect("child must attach to the parent's block");
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(30);
    let mut seen = 0u32;
    while seen < 5_000 {
        assert!(
            std::time::Instant::now() < deadline,
            "child timed out after {seen} coherent reads"
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
            seen += 1;
        }
    }
}

#[test]
fn cross_process_reader_never_sees_a_torn_snapshot() {
    if std::env::var(CROSS_PROC_ENV).is_ok() {
        return; // this process IS the child; do not recurse
    }
    let name = unique_name("xp");
    let writer = SimWriterSession::create(&name).unwrap();

    let mut child = std::process::Command::new(std::env::current_exe().expect("test binary path"))
        .args([
            "--exact",
            "roles::tests::cross_process_child_reader",
            "--nocapture",
        ])
        .env(CROSS_PROC_ENV, &name)
        .spawn()
        .expect("spawn the reader child");

    let mut i = 0u64;
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(60);
    loop {
        match child.try_wait().expect("poll the child") {
            Some(status) => {
                assert!(status.success(), "cross-process reader failed: {status}");
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
fn cross_process_motor_commands_round_trip() {
    // The FC-writes / plugin-reads direction, across processes.
    if std::env::var(CROSS_PROC_ENV).is_ok() {
        return;
    }
    let name = unique_name("xm");
    let writer = SimWriterSession::create(&name).unwrap();
    let fc = FcSession::attach(&name).unwrap();

    let mut child = std::process::Command::new(std::env::current_exe().expect("test binary path"))
        .args([
            "--exact",
            "roles::tests::cross_process_child_reader",
            "--nocapture",
        ])
        .env(CROSS_PROC_ENV, &name)
        .spawn()
        .expect("spawn the reader child");

    // Publish state for the child while hammering motor commands
    // from this process: both seqlocks are live at once, which is
    // the production topology.
    let mut i = 0u64;
    loop {
        match child.try_wait().expect("poll the child") {
            Some(status) => {
                assert!(status.success(), "cross-process reader failed: {status}");
                break;
            }
            None => {
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
                fc.write_motor_command(&[v, v * 2.0, v * 3.0, v * 4.0]);
                if let Some((lanes, n)) = writer.read_motor_command() {
                    assert_eq!(n, 4);
                    // A torn motor snapshot mixes lanes from two
                    // publications; each is a fixed multiple of the
                    // first.
                    if lanes[0] != 0.0 {
                        assert_eq!(lanes[1], lanes[0] * 2.0, "torn motor lanes");
                        assert_eq!(lanes[2], lanes[0] * 3.0, "torn motor lanes");
                        assert_eq!(lanes[3], lanes[0] * 4.0, "torn motor lanes");
                    }
                }
            }
        }
    }
}
