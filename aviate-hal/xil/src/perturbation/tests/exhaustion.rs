use super::*;

#[test]
fn an_accepted_maximum_sequence_has_no_successor() {
    let mut engine = PerturbationEngine::new(sensor_config(7)).expect("engine");
    let mut final_sample = SimSensorPacket::new(u64::MAX);
    engine
        .apply_sensor(u64::MAX, &mut final_sample)
        .expect("maximum sensor sequence");
    complete_reply(&mut engine, u64::MAX);

    assert_eq!(
        engine.apply_sensor(0, &mut SimSensorPacket::new(0)),
        Err(PerturbationError::SampleSequenceExhausted)
    );
    assert_eq!(
        engine.apply_sensor(u64::MAX, &mut final_sample),
        Err(PerturbationError::Quarantined)
    );
}
