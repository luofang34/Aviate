use super::*;

fn manifest() -> serde_json::Value {
    serde_json::json!({
        "missions": {
            "a": { "gate": "blocking", "shard": "one", "runs": 3, "pass_threshold": 2 },
            "b": { "gate": "blocking", "shard": "two", "runs": 3, "pass_threshold": 2 },
            "q": { "gate": "quarantined", "shard": "quarantine", "runs": 3, "pass_threshold": 2 },
            "m": { "gate": "manual", "reason": "on demand" }
        }
    })
}

fn names(selected: &[Selected]) -> Vec<&str> {
    let mut names: Vec<&str> = selected.iter().map(|s| s.name.as_str()).collect();
    names.sort_unstable();
    names
}

#[test]
fn blocking_selects_every_gated_mission_with_its_bar() {
    let selected = select(&manifest(), &["blocking".to_owned()]).expect("selects");
    assert_eq!(names(&selected), ["a", "b"]);
    assert!(selected
        .iter()
        .all(|s| s.runs == 3 && s.pass_threshold == 2));
}

#[test]
fn a_shard_or_a_name_selects_its_missions() {
    let quarantine = select(&manifest(), &["quarantine".to_owned()]).expect("selects");
    assert_eq!(names(&quarantine), ["q"]);
    let manual = select(&manifest(), &["m".to_owned()]).expect("selects");
    assert_eq!(names(&manual), ["m"]);
    assert_eq!((manual[0].runs, manual[0].pass_threshold), (1, 1));
}

#[test]
fn an_unknown_selection_is_refused() {
    assert!(select(&manifest(), &["nope".to_owned()]).is_err());
}

#[test]
fn the_committed_manifest_gates_its_blocking_missions() {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../tests/missions");
    let manifest = read_manifest(&dir).expect("the committed manifest parses");
    let blocking = select(&manifest, &["blocking".to_owned()]).expect("selects");
    assert!(!blocking.is_empty());
    for mission in &blocking {
        assert!(
            dir.join(format!("{}.toml", mission.name)).is_file(),
            "{}",
            mission.name
        );
        assert!(mission.pass_threshold <= mission.runs);
    }
}
