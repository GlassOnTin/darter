//! sim_run --track integration tests: the CLI loads track files before
//! anything flies, emits checkpoint events into summary.json, and changes
//! nothing else about the run (record hash identical with and without).

use std::path::Path;
use std::path::PathBuf;
use std::process::Command;

fn temp_dir(name: &str) -> PathBuf {
    let d = std::env::temp_dir().join(format!("darter-track-{name}-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&d);
    std::fs::create_dir_all(&d).expect("temp dir");
    d
}

/// Hover throttle + -14 m/s westward transit from (0, 0, 35): the record
/// crosses x -5 near t 0.36 s, -15 near 1.2 s, -25 just before 2 s (the
/// same flight profile as the measured corridor record, gate -15 at t
/// 1.112). Gate z 34.8 sits under the pz arc (35.1 falling slowly), radius
/// 8 m keeps every crossing a hit despite the altitude drift.
const TRACK_JSON: &str = r#"{
    "schema": "darter_track",
    "version": 1,
    "name": "transit_test",
    "spawn": {"x": 0.0, "y": 0.0, "z": 35.0},
    "checkpoints": [
        {"kind": "gate", "x": -5.0, "y": 0.0, "z": 34.8, "radius_m": 8.0},
        {"kind": "gate", "x": -15.0, "y": 0.0, "z": 34.8, "radius_m": 8.0},
        {"kind": "gate", "x": -25.0, "y": 0.0, "z": 34.8, "radius_m": 8.0}
    ]
}"#;

/// Loop variant: cps ordered start(-10) -> gate(-5) -> gate(-15). The gate
/// normals wrap around the list (gate -5: n = cp2 - cp0 = -x; gate -15:
/// n = cp0 - cp1 = -x), so the -x transit fires them in order [1, 2]. The
/// start's own normal is n = cp1 - cp2 = +x, so a one-way transit can never
/// fire it (a return pass would move +x) - its silence IS the assertion, and
/// lap_splits stays an empty array (no lap opened), not null.
const LOOP_JSON: &str = r#"{
    "schema": "darter_track",
    "version": 1,
    "name": "loop_test",
    "spawn": {"x": 0.0, "y": 0.0, "z": 35.0},
    "checkpoints": [
        {"kind": "start", "x": -10.0, "y": 0.0, "z": 34.8, "radius_m": 8.0},
        {"kind": "gate", "x": -5.0, "y": 0.0, "z": 34.8, "radius_m": 8.0},
        {"kind": "gate", "x": -15.0, "y": 0.0, "z": 34.8, "radius_m": 8.0}
    ]
}"#;

fn sim_run(out: &Path, extra: &[&str]) -> std::process::Output {
    Command::new(env!("CARGO_BIN_EXE_sim_run"))
        .args(extra)
        .arg("--out")
        .arg(out)
        .output()
        .expect("spawn sim_run")
}

fn run_transit(dir: &Path, track_arg: Option<&str>) -> serde_json::Value {
    const FLIGHT: [&str; 12] = [
        "--mode", "core", "--seed", "7", "--duration", "2.5", "--alt", "35",
        "--throttle", "0.157", "--vx", "-14",
    ];
    let mut args: Vec<&str> = FLIGHT.to_vec();
    if let Some(track_arg) = track_arg {
        args.push("--track");
        args.push(track_arg);
    }
    let out = sim_run(dir, &args);
    assert!(
        out.status.success(),
        "sim_run failed: {}",
        String::from_utf8_lossy(&out.stderr)
    );
    let text = std::fs::read_to_string(dir.join("summary.json")).expect("summary");
    serde_json::from_str(&text).expect("summary.json parses")
}

fn track_block(dir: &Path, track_arg: Option<&str>) -> serde_json::Value {
    run_transit(dir, track_arg)
        .get("track")
        .expect("summary carries a track block when --track was given")
        .clone()
}

fn write_track(dir: &Path, name: &str, body: &str) -> PathBuf {
    let p = dir.join(name);
    std::fs::write(&p, body).expect("write track fixture");
    p
}

/// Events over a straight-line transit flight, chronological, then indices
/// strictly ascending (the record threads the gates in list order); the
/// open track reports lap_splits null.
#[test]
fn track_cli_transit_events() {
    let dir = temp_dir("transit");
    let track = write_track(&dir, "track.json", TRACK_JSON);
    let trk = track_block(&dir, Some(track.to_str().unwrap()));
    assert_eq!(trk["schema"], "darter_track");
    assert_eq!(trk["version"], serde_json::json!(1));
    assert_eq!(trk["name"], "transit_test");
    assert_eq!(trk["checkpoints"], serde_json::json!(3));
    assert_eq!(trk["loop"], serde_json::json!(false));
    assert!(trk["lap_splits"].is_null());
    assert!(trk["sha256"].as_str().map(|s| s.len() == 64 && s.chars().all(|c| c.is_ascii_hexdigit())).unwrap_or(false));
    assert_eq!(trk["path"].as_str().map(String::from), Some(track.to_string_lossy().into_owned()));

    let events = trk["events"].as_array().expect("events array");
    assert_eq!(events.len(), 3, "all three gates transited: {events:?}");
    let mut prev_t = 0.0f64;
    for (k, e) in events.iter().enumerate() {
        let idx = e[0].as_u64().expect("event index");
        let t = e[1].as_f64().expect("event time");
        assert_eq!(idx, k as u64, "event {k}: {e:?}");
        assert!(t > prev_t, "times strictly increasing: {events:?}");
        assert!(t < 2.5, "event inside the flown duration: {t}");
        prev_t = t;
    }
    let _ = std::fs::remove_dir_all(&dir);
}

/// A missing or malformed track file is a hard error BEFORE anything flies:
/// exit 1, stderr names `track`, and no summary.json is written.
#[test]
fn track_cli_bad_path_is_hard_error() {
    let dir = temp_dir("badpath");
    let out = sim_run(
        &dir,
        &["--mode", "core", "--seed", "7", "--duration", "2", "--alt", "35",
          "--track", "no_such_track.json"],
    );
    assert!(!out.status.success());
    let stderr = String::from_utf8_lossy(&out.stderr);
    assert!(stderr.contains("track no_such_track.json"), "stderr: {stderr}");
    assert!(!dir.join("summary.json").exists(), "no summary on a hard error");

    // Same for a structurally bad file.
    let bad = write_track(&dir, "bad.json", "{\"schema\": \"wrong\", \"version\": 1}");
    let out = sim_run(
        &dir,
        &["--mode", "core", "--seed", "7", "--duration", "2", "--alt", "35",
          "--track", bad.to_str().unwrap()],
    );
    assert!(!out.status.success());
    let stderr = String::from_utf8_lossy(&out.stderr);
    assert!(
        stderr.contains("schema must be \"darter_track\"") && stderr.contains("track"),
        "stderr: {stderr}"
    );
    assert!(!dir.join("summary.json").exists());
    let _ = std::fs::remove_dir_all(&dir);
}

/// A loop track (start first) plumbs loop: true and exercises the wrap
/// neighbour rule for the gate normals. On a one-way -x transit the start
/// plane is crossed backwards (its normal points along the return direction,
/// +x), so events stay chronological gate order [1, 2] with the start
/// silent - the passive event log reports no complaint - and lap_splits is
/// an empty array (no lap opened), not null. Real lap arithmetic is covered
/// by track.rs's loop_laps_after_repeat_start with a returning flight.
#[test]
fn track_cli_loop_flag_plumbs() {
    let dir = temp_dir("loop");
    let track = write_track(&dir, "loop.json", LOOP_JSON);
    let trk = track_block(&dir, Some(track.to_str().unwrap()));
    assert_eq!(trk["loop"], serde_json::json!(true));
    assert_eq!(trk["checkpoints"], serde_json::json!(3));
    let splits = trk["lap_splits"].as_array().expect("loop -> array, not null");
    assert!(splits.is_empty(), "one-way transit: no lap opened: {splits:?}");
    let events = trk["events"].as_array().expect("events array");
    let got: Vec<u64> = events.iter().map(|e| e[0].as_u64().expect("idx")).collect();
    assert_eq!(got, vec![1, 2], "wrap-normal gates fire, start stays silent: {events:?}");
    let _ = std::fs::remove_dir_all(&dir);
}

/// --track changes nothing about the flight: identical record hashes with
/// and without the flag.
#[test]
fn track_cli_leaves_record_hash_unchanged() {
    let dir = temp_dir("hash");
    let plain = temp_dir("hash-plain");
    let with = temp_dir("hash-with");
    let track = write_track(&dir, "track.json", TRACK_JSON);
    let _ = run_transit(&plain, None);
    let _ = run_transit(&with, Some(track.to_str().unwrap()));
    for d in [&plain, &with] {
        assert!(d.join("summary.json").exists());
    }
    let h = |d: &Path| {
        let text = std::fs::read_to_string(d.join("summary.json")).unwrap();
        let v: serde_json::Value = serde_json::from_str(&text).unwrap();
        v["record_hash"].as_str().expect("record_hash").to_string()
    };
    assert_eq!(h(&plain), h(&with), "record hash must not depend on --track");
    let _ = std::fs::remove_dir_all(&dir);
    let _ = std::fs::remove_dir_all(&plain);
    let _ = std::fs::remove_dir_all(&with);
}