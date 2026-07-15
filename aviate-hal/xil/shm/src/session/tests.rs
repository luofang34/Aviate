//! End-to-end tests over a real POSIX shm object: one creator
//! session plays the simulation writer, one attached session plays
//! the FC/consumer — the exact cross-process topology, minus the
//! process boundary (the mapping and protocols are identical).

use super::*;

fn unique_name(tag: &str) -> String {
    // Distinct per test; PIDs keep parallel CI shards apart. Keep it
    // SHORT: macOS caps POSIX shm names at 31 characters (PSHMNAMLEN)
    // and rejects longer ones with ENAMETOOLONG.
    format!("/avxt_{}_{}", tag, std::process::id())
}

#[test]
fn attach_fails_closed_without_writer() {
    let err = ShmSession::attach("/aviate_xil_test_nonexistent").unwrap_err();
    assert!(matches!(err, AttachFailure::Io(_)));
}

#[test]
fn valid_block_attaches_with_fresh_generation() {
    // The fingerprint gate's happy path over a real object; each
    // mismatch arm (magic / version / declared size / short object)
    // is pinned unit-level by the contract crate's validate_attach
    // tests, which this attach path calls verbatim.
    let name = unique_name("ok");
    let _writer = ShmSession::create(&name).unwrap();
    let reader = ShmSession::attach(&name).unwrap();
    assert!(reader.plugin_ready());
    assert_eq!(reader.reset_generation(), 1);
}

#[test]
fn model_state_round_trips_coherently() {
    let name = unique_name("st");
    let writer = ShmSession::create(&name).unwrap();
    let reader = ShmSession::attach(&name).unwrap();

    assert_eq!(
        reader.read_model_state(),
        None,
        "no snapshot before the first publish (valid=0)"
    );

    let snap = ModelStateSnapshot {
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
    let writer = ShmSession::create(&name).unwrap();
    let fc = ShmSession::attach(&name).unwrap();

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
    let sim = ShmSession::create(&name).unwrap();
    let host = ShmSession::attach(&name).unwrap();
    let fc = ShmSession::attach(&name).unwrap();

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
    let sim = ShmSession::create(&name).unwrap();
    let host = ShmSession::attach(&name).unwrap();

    assert_eq!(sim.lockstep_enabled_raw(), 0, "async by default");
    host.set_lockstep_enabled_raw(1);
    host.set_target_rtf_percent(400);
    assert_eq!(sim.lockstep_enabled_raw(), 1);
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
    let writer = ShmSession::create(&name).unwrap();
    let reader = ShmSession::attach(&name).unwrap();

    let stop = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false));
    let w = {
        let stop = stop.clone();
        std::thread::spawn(move || {
            let mut i = 0_u64;
            while !stop.load(Ordering::Relaxed) {
                i = i.wrapping_add(1);
                let v = i as f64;
                writer.write_model_state(&ModelStateSnapshot {
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
