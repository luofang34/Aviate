//! End-to-end tests over a real POSIX shm object: the role
//! endpoints play their production parts — SimWriter (plugin),
//! FcSession, HostSession, read-only Consumer — the exact
//! cross-process topology, minus the process boundary.

use super::*;
use aviate_xil_contract::WriterState;
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
    // read-order convention — performs the world reset, acks
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

// The writer-crash scenarios live in tests/cross_process.rs: a
// crash means the PROCESS died and the kernel released its lease,
// which `mem::forget` inside this process cannot model — a forgotten
// writer's lease fd stays open here, so the lease (correctly!)
// reports it alive.

#[test]
fn a_second_writer_cannot_take_over_a_live_object() {
    // An unconditional pre-create unlink would let a second
    // writer — or a dying writer's late cleanup — destroy a LIVE
    // peer's object out from under every consumer. The lease makes
    // that a loud failure instead.
    let name = unique_name("dw");
    let first = SimWriterSession::create(&name).unwrap();
    first.write_model_state(&ModelStateSnapshot {
        reset_generation: 1,
        sim_step: 3,
        time_us: 3_000,
        pos: [1.0, 1.0, 1.0],
        quat: [1.0, 0.0, 0.0, 0.0],
        vel: [0.0; 3],
        ang_vel: [0.0; 3],
    });
    let consumer = ConsumerSession::attach(&name).unwrap();

    let second = SimWriterSession::create(&name);
    assert!(
        second.is_err(),
        "a live writer's object must not be silently taken over"
    );

    // And the failed attempt must not have damaged the live world.
    assert_eq!(consumer.writer_state(), WriterState::Current);
    assert!(consumer.read_model_state().is_some());

    // Once the first writer exits cleanly, the name is free again.
    drop(first);
    let _third = SimWriterSession::create(&name).unwrap();
}

#[test]
fn zero_sized_creation_window_is_retryable() {
    // shm_open(O_CREAT) publishes the name before ftruncate sizes
    // it. An attacher landing in that window must get a RETRYABLE
    // refusal: the Contract channel is translated by callers into a
    // permanent never-retried failure, which would turn this
    // microsecond of normal startup into a deadlock. The mid-init
    // test below cannot catch this — its object is already
    // full-sized — so this one manufactures a real zero-length
    // POSIX object.
    let name = unique_name("zs");
    let zero = crate::mapping::Mapping::create_zero_sized_for_test(&name).unwrap();
    match ConsumerSession::attach(&name) {
        Err(AttachFailure::NotReady) => {}
        other => panic!("zero-sized window must be NotReady, got {other:?}"),
    }

    // The window closes: a real writer replaces it and the same
    // consumer retry succeeds end to end.
    drop(zero);
    let _writer = SimWriterSession::create(&name).unwrap();
    let consumer = ConsumerSession::attach(&name).unwrap();
    assert_eq!(consumer.writer_state(), WriterState::Current);
}

#[test]
fn a_departed_writer_is_gone_not_current() {
    // The bug a boolean `writer_replaced` hides: the name stops
    // resolving, so "not replaced" reads as "healthy" and the orphan
    // is trusted forever.
    let name = unique_name("gn");
    let writer = SimWriterSession::create(&name).unwrap();
    let consumer = ConsumerSession::attach(&name).unwrap();
    assert_eq!(consumer.writer_state(), WriterState::Current);

    drop(writer);

    assert_eq!(
        consumer.writer_state(),
        WriterState::Gone,
        "an unlinked name is a dead writer, never a healthy one"
    );
}

#[test]
fn a_reset_retires_the_previous_epochs_snapshot() {
    // Between the generation bump and the new world's first publish,
    // the block still holds the retired epoch's pose: valid, coherent,
    // and from a world that no longer exists.
    let name = unique_name("rg");
    let writer = SimWriterSession::create(&name).unwrap();
    let consumer = ConsumerSession::attach(&name).unwrap();
    writer.write_model_state(&ModelStateSnapshot {
        reset_generation: 1,
        sim_step: 100,
        time_us: 100_000,
        pos: [42.0, 42.0, -42.0],
        quat: [1.0, 0.0, 0.0, 0.0],
        vel: [3.0; 3],
        ang_vel: [0.0; 3],
    });
    assert_eq!(
        consumer.read_model_state().unwrap().pos,
        [42.0, 42.0, -42.0]
    );

    let generation = writer.bump_reset_generation();
    assert_eq!(generation, 2);
    assert_eq!(
        consumer.read_model_state(),
        None,
        "the pre-reset pose must not survive the reset"
    );
    // Contract: a reset bumps the epoch IN PLACE. Same object, same
    // incarnation, same attachment — a consumer re-keys, it does NOT
    // re-attach. (Re-creation is the writer-restart path, pinned in
    // the cross-process crash test.)
    assert_eq!(consumer.writer_state(), WriterState::Current);

    // The new world publishes; the consumer resumes, in the new epoch.
    writer.write_model_state(&ModelStateSnapshot {
        reset_generation: 2,
        sim_step: 101,
        time_us: 0,
        pos: [0.0, 0.0, 0.0],
        quat: [1.0, 0.0, 0.0, 0.0],
        vel: [0.0; 3],
        ang_vel: [0.0; 3],
    });
    let fresh = consumer.read_model_state().unwrap();
    assert_eq!(fresh.reset_generation, 2);
    assert_eq!(fresh.pos, [0.0, 0.0, 0.0]);
}

#[test]
fn a_snapshot_from_a_stale_epoch_is_refused() {
    // Direct pin of the reader's generation double-check: a
    // publisher that keeps stamping the retired epoch after a bump (a
    // writer that has not noticed the reset) must not be believed.
    let name = unique_name("se");
    let writer = SimWriterSession::create(&name).unwrap();
    let consumer = ConsumerSession::attach(&name).unwrap();
    writer.bump_reset_generation();
    writer.write_model_state(&ModelStateSnapshot {
        reset_generation: 1, // stale epoch
        sim_step: 7,
        time_us: 7_000,
        pos: [9.0, 9.0, 9.0],
        quat: [1.0, 0.0, 0.0, 0.0],
        vel: [0.0; 3],
        ang_vel: [0.0; 3],
    });
    assert_eq!(
        consumer.read_model_state(),
        None,
        "a snapshot stamped with a dead epoch must be refused"
    );
}

#[test]
fn incarnations_are_monotonic_across_rapid_restarts() {
    // A same-process, same-instant restart is the case a pid- or
    // clock-derived identity can collide on: the pid is identical
    // and the clock may not have advanced past its granularity.
    // The lease-counter identity is deterministic — each grant on
    // one name advances by exactly one and never lands on zero.
    let name = unique_name("rr");
    let mut prev = 0u64;
    for round in 0..5 {
        let writer = SimWriterSession::create(&name).unwrap();
        let inc = writer.mapping.incarnation();
        assert_ne!(inc, 0, "zero is reserved for \"not stamped\"");
        if round > 0 {
            assert_eq!(
                inc,
                prev.wrapping_add(1),
                "each grant advances the persisted counter by one"
            );
        }
        prev = inc;
        drop(writer);
    }
}

#[test]
fn a_rapid_same_process_restart_reads_replaced_not_current() {
    // Identity/liveness binding: a consumer attached to one
    // incarnation must see Current only while THAT writer is alive
    // (liveness), and Replaced the instant a successor exists
    // (identity) — even when predecessor and successor share a pid
    // and an instant.
    let name = unique_name("rp");
    let first = SimWriterSession::create(&name).unwrap();
    first.write_model_state(&ModelStateSnapshot {
        reset_generation: 1,
        sim_step: 1,
        time_us: 1_000,
        pos: [0.0; 3],
        quat: [1.0, 0.0, 0.0, 0.0],
        vel: [0.0; 3],
        ang_vel: [0.0; 3],
    });
    let consumer = ConsumerSession::attach(&name).unwrap();
    assert_eq!(consumer.writer_state(), WriterState::Current);

    drop(first);
    assert_eq!(
        consumer.writer_state(),
        WriterState::Gone,
        "no successor yet: the lease is free, so the writer is dead"
    );

    let _second = SimWriterSession::create(&name).unwrap();
    assert_eq!(
        consumer.writer_state(),
        WriterState::Replaced,
        "a successor under the same pid in the same instant must read as a new identity"
    );
}

#[test]
fn fc_attachments_stamp_a_monotonic_session_nonce() {
    let name = unique_name("sn");
    let writer = SimWriterSession::create(&name).unwrap();
    assert_eq!(
        writer.fc_session_nonce(),
        0,
        "zero means no FC has ever attached"
    );

    let fc1 = FcSession::attach(&name).unwrap();
    assert_eq!(writer.fc_session_nonce(), 1);
    drop(fc1);

    let _fc2 = FcSession::attach(&name).unwrap();
    assert_eq!(
        writer.fc_session_nonce(),
        2,
        "every attachment is a new session"
    );
}

#[test]
fn the_session_nonce_wraps_past_zero() {
    let name = unique_name("sw");
    let writer = SimWriterSession::create(&name).unwrap();
    let fc = FcSession::attach(&name).unwrap();
    fc.mapping.set_fc_session_nonce(u32::MAX);
    drop(fc);

    let _fc2 = FcSession::attach(&name).unwrap();
    assert_eq!(
        writer.fc_session_nonce(),
        1,
        "the wrap must skip zero — zero would read as \"no FC ever attached\""
    );
}

#[path = "tests/cross_process.rs"]
mod cross_process;
