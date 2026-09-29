//! T6 area-pack integration test: the OSM bbox -> offline pack CLI is driven
//! end to end on the cached Overpass fixture (no network), and the written
//! pack is re-asserted from two independent sides:
//!
//! - the tool's own validator subcommand must accept the pack and reject
//!   deliberately corrupted copies (schema version, OSM attribution,
//!   counts-vs-arrays mismatch);
//! - this test re-parses pack.json itself and cross-checks the metadata:
//!   counts match the element arrays, bounds agree with the derived ground
//!   span, the recorded input sha256 matches the fixture bytes, and
//!   scene.obj's o-groups match the counts (the OBJ is derived from pack.json
//!   alone, so a group-count mismatch means the two drifted apart).
//!
//! Determinism: the same fixture + seed must rebuild byte-identical pack.json
//! AND scene.obj (sorted JSON keys, fixed float rounding/formatting, one
//! seeded RNG in one fixed order — see tools/area_pack.py).
//!
//! Elevation is flat-only in this build; `--elevation glo30` must fail loudly
//! with NotImplementedError rather than silently producing fake terrain (the
//! DEM pipeline is a stated gap: rasterio is not in the test environment).
//!
//! The tool is stdlib-only Python (no pinned interpreter — provenance lives
//! in the pack itself: input sha256 + seed + origin, and the byte-identity
//! gate re-runs here with whatever interpreter builds the pack).

use std::path::{Path, PathBuf};
use std::process::Command;

const FIXTURE: &str = "tests/fixtures/osm_home_area.json";
const HOME_LAT: f64 = 50.8989;
const HOME_LON: f64 = -1.0586;
const SEED: u64 = 5;
/// The fixture is a ~0.9 km bbox plus whole-way Overpass spill (one 2.3 km
/// footway); bounds beyond this would mean the scene exploded.
const MAX_EXTENT_M: f64 = 3000.0;
/// Cached-fixture scale: the anisoptera home area carries 1372 building ways;
/// a big drop means the parser lost ways.
const MIN_BUILDINGS: usize = 1000;

fn temp_dir(name: &str) -> PathBuf {
    let d = std::env::temp_dir().join(format!("darter-area-pack-{name}-{}", std::process::id()));
    std::fs::create_dir_all(&d).unwrap();
    d
}

fn tool() -> Command {
    let mut c = Command::new("python3");
    c.arg("tools/area_pack.py");
    c
}

fn run(mut cmd: Command) -> (bool, String, String) {
    let out = cmd.output().expect("spawn area_pack.py");
    (
        out.status.success(),
        String::from_utf8_lossy(&out.stdout).into_owned(),
        String::from_utf8_lossy(&out.stderr).into_owned(),
    )
}

// ---- minimal readers for the generated (sorted-key, fixed-field) JSON ----
// pack.json string values carry no braces, so brace matching is safe here.

/// Inner text of a top-level JSON object `"<key>": { ... }`.
fn json_object<'a>(text: &'a str, key: &str) -> &'a str {
    let pat = format!("\"{key}\": {{");
    let i = text
        .find(&pat)
        .unwrap_or_else(|| panic!("object {key} missing in pack.json"));
    let start = i + pat.len();
    let bytes = text.as_bytes();
    let mut depth = 1usize;
    let mut j = start;
    while depth > 0 {
        match bytes[j] {
            b'{' => depth += 1,
            b'}' => depth -= 1,
            _ => {}
        }
        j += 1;
    }
    &text[start..j - 1]
}

fn num_in(text: &str, key: &str) -> f64 {
    let pat = format!("\"{key}\":");
    let i = text
        .find(&pat)
        .unwrap_or_else(|| panic!("field {key} missing in {text}"));
    let rest = text[i + pat.len()..].trim_start();
    let end = rest.find([',', '}']).unwrap_or(rest.len());
    // trailing whitespace (indent=1 puts keys on their own lines) is not
    // accepted by f64::from_str
    rest[..end].trim().parse().expect("number parse")
}

fn str_in(text: &str, key: &str) -> String {
    let pat = format!("\"{key}\": \"");
    let i = text
        .find(&pat)
        .unwrap_or_else(|| panic!("string field {key} missing in pack.json"));
    let rest = &text[i + pat.len()..];
    let end = rest.find('"').expect("string terminator");
    rest[..end].to_string()
}

/// The counts block only: `"trees": <digits>,` — the trees ARRAY is
/// `"trees": [`, so digits disambiguate the counts entry.
fn counts_trees_entry(text: &str) -> (String, u64) {
    let key = "\"trees\": ";
    let i = text.find(key).expect("counts.trees entry");
    let after = &text[i + key.len()..];
    let n_digits = after
        .chars()
        .take_while(|c| c.is_ascii_digit())
        .count();
    assert!(n_digits > 0, "\"trees\": not followed by digits");
    let n: u64 = after[..n_digits].parse().unwrap();
    (text[..i + key.len()].to_owned() + &after[..n_digits], n)
}

fn replace_attribution_value(text: &str, with: &str) -> String {
    let key = "\"attribution\": \"";
    let i = text.find(key).expect("attribution entry");
    let val_start = i + key.len();
    let val_end = text[val_start..].find('"').expect("attribution terminator") + val_start;
    format!("{}{}{}", &text[..val_start], with, &text[val_end..])
}

fn count_lines(text: &str, prefix: &str) -> usize {
    text.lines().filter(|l| l.starts_with(prefix)).count()
}

fn build_pack(dir: &Path) {
    let (ok, stdout, stderr) = run({
        let mut c = tool();
        c.args([
            "--osm",
            FIXTURE,
            "--lat",
            &HOME_LAT.to_string(),
            "--lon",
            &HOME_LON.to_string(),
            "--seed",
            &SEED.to_string(),
            "--out",
        ])
        .arg(dir);
        c
    });
    assert!(ok, "area_pack build failed: {stdout}{stderr}");
    println!("build: {stdout}");
}

fn validate(dir: &Path) -> (bool, String, String) {
    let mut c = tool();
    c.arg(dir);
    run(c)
}

fn read_pack_text(dir: &Path) -> String {
    std::fs::read_to_string(dir.join("pack.json")).expect("pack.json")
}

/// The full metadata cross-check (used by the build test; the byte-identity
/// test calls it on the second copy too, so "identical" never means "junk").
fn assert_metadata(text: &str, dir: &Path) {
    assert_eq!(str_in(text, "schema"), "darter_area_pack");
    // v2: counts.dashes + paint_white dash OBJ groups (S3b, 2026-09-29).
    assert!((num_in(text, "version") - 2.0).abs() < f64::EPSILON);
    assert!(str_in(text, "attribution").contains("OpenStreetMap"));
    let elevation = json_object(text, "elevation");
    assert_eq!(str_in(elevation, "model"), "flat");
    assert!((num_in(text, "seed") - SEED as f64).abs() < f64::EPSILON);

    // provenance: the recorded input sha256 must match the fixture bytes
    let fixture = std::fs::read(FIXTURE).expect("read fixture");
    let mut h = darter_core::sha256::Sha256::new();
    h.update(&fixture);
    let got = darter_core::sha256::to_hex(&h.finish());
    assert_eq!(str_in(text, "osm_json_sha256"), got, "input sha256 mismatch");
    assert!((num_in(text, "origin_lat") - HOME_LAT).abs() < 1e-9);
    assert!((num_in(text, "origin_lon") - HOME_LON).abs() < 1e-9);

    // counts vs arrays (ids are one letter + index per element family)
    let counts = json_object(text, "counts");
    let counts_b = num_in(counts, "buildings") as usize;
    let counts_r = num_in(counts, "roads") as usize;
    let counts_s = num_in(counts, "strips") as usize;
    let counts_g = num_in(counts, "grass") as usize;
    let counts_t = num_in(counts, "trees") as usize;
    let counts_tt = num_in(counts, "trees_tagged") as usize;
    assert_eq!(counts_b, text.matches("\"id\": \"b").count(), "buildings");
    assert_eq!(counts_r, text.matches("\"id\": \"r").count(), "roads");
    assert_eq!(counts_s, text.matches("\"id\": \"s").count(), "strips");
    assert_eq!(counts_g, text.matches("\"id\": \"g").count(), "grass");
    assert_eq!(counts_t, text.matches("\"id\": \"t").count(), "trees");
    assert_eq!(counts_tt, text.matches("\"source\": \"tagged\"").count());
    assert!(counts_tt <= counts_t);
    assert!(counts_b > MIN_BUILDINGS, "buildings {counts_b} below floor");

    // bounds vs derived ground span; scene stays near the origin
    let bounds = json_object(text, "bounds");
    let min_x = num_in(bounds, "min_x");
    let max_x = num_in(bounds, "max_x");
    let min_y = num_in(bounds, "min_y");
    let max_y = num_in(bounds, "max_y");
    let span = 2.0 * min_x.abs().max(max_x.abs()).max(min_y.abs()).max(max_y.abs()) + 200.0;
    let stored = num_in(text, "span_m");
    assert!((stored - span).abs() < 0.01, "span {stored} vs derived {span}");
    assert!(max_x.abs().max(min_x.abs()).max(max_y.abs()).max(min_y.abs()) < MAX_EXTENT_M);
    assert!(text.contains("\"origin_inside_building\": false"));

    // scene.obj o-groups vs counts (OBJ is derived from pack.json alone)
    let obj = std::fs::read_to_string(dir.join("scene.obj")).expect("scene.obj");
    assert_eq!(count_lines(&obj, "o bld_"), counts_b, "bld groups");
    assert_eq!(count_lines(&obj, "o bldroof_"), counts_b, "bldroof groups");
    assert_eq!(count_lines(&obj, "o road_"), counts_r, "road groups");
    assert_eq!(
        count_lines(&obj, "o hedge_") + count_lines(&obj, "o fence_"),
        counts_s,
        "strip groups"
    );
    assert_eq!(count_lines(&obj, "o grass_"), counts_g, "grass groups");
    assert_eq!(count_lines(&obj, "o tree_"), counts_t, "tree groups");
    assert_eq!(count_lines(&obj, "o treec_"), counts_t, "treec groups");
    assert!(obj.lines().any(|l| l == "o ground"));
    assert!(obj.lines().any(|l| l.starts_with("v ")), "no vertices");
}

#[test]
fn area_pack_build_validate_metadata() {
    let dir = temp_dir("meta");
    build_pack(&dir);
    let (ok, _out, err) = validate(&dir);
    assert!(ok, "validator rejected a freshly built pack: {err}");

    let text = read_pack_text(&dir);
    assert_metadata(&text, &dir);

    // the stated elevation gap fails loudly, never with fake terrain.
    // No positional (that would enter validate mode, which takes precedence).
    let (ok, _out, err) = run({
        let mut c = tool();
        c.args(["--osm", FIXTURE, "--elevation", "glo30"]);
        c
    });
    assert!(!ok, "--elevation glo30 must fail, not build a pack");
    assert!(
        err.contains("NotImplementedError"),
        "glo30 must raise NotImplementedError, got: {err}"
    );
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn area_pack_byte_identical_rebuild() {
    let a = temp_dir("byte-a");
    let b = temp_dir("byte-b");
    build_pack(&a);
    build_pack(&b);
    for name in ["pack.json", "scene.obj"] {
        let fa = std::fs::read(a.join(name)).expect(name);
        let fb = std::fs::read(b.join(name)).expect(name);
        assert_eq!(fa, fb, "{name} not byte-identical across rebuilds");
        assert!(!fa.is_empty());
    }
    // both copies also carry the full metadata (identity is not emptiness)
    assert_metadata(&read_pack_text(&b), &b);
    let _ = std::fs::remove_dir_all(&a);
    let _ = std::fs::remove_dir_all(&b);
}

#[test]
fn area_pack_validator_rejects_corruption() {
    let base = temp_dir("corrupt-base");
    build_pack(&base);
    let text = read_pack_text(&base);

    let (counts_trees_prefix, counts_trees_n) = counts_trees_entry(&text);
    let counts_trees_mutated =
        format!("{counts_trees_prefix}{}{}", counts_trees_n + 1, &text[counts_trees_prefix.len()..]);
    let cases: Vec<(&str, String, &str)> = vec![
        (
            "version",
            text.replacen("\"version\": 2", "\"version\": 3", 1),
            "version",
        ),
        (
            "attribution",
            replace_attribution_value(&text, "no credit"),
            "attribution",
        ),
        ("counts.trees", counts_trees_mutated, "counts.trees"),
    ];
    for (name, mutated, expect_err) in cases {
        assert_ne!(mutated, text, "{name}: mutation did not apply");
        let d = temp_dir(&format!("corrupt-{name}"));
        std::fs::create_dir_all(&d).unwrap();
        std::fs::write(d.join("pack.json"), &mutated).unwrap();
        std::fs::copy(base.join("scene.obj"), d.join("scene.obj")).unwrap();
        let (ok, _out, err) = validate(&d);
        assert!(!ok, "validator accepted corrupted pack ({name})");
        assert!(err.contains(expect_err), "{name}: stderr missing '{expect_err}': {err}");
        let _ = std::fs::remove_dir_all(&d);
    }
    let _ = std::fs::remove_dir_all(&base);
}

/// Opt-in: hits the live Overpass endpoint (network). The assertions stay
/// loose — remote content is not pinned — but the fetch must produce a
/// pack that validates, with the fetch date and endpoint recorded.
#[test]
#[ignore = "needs network (live Overpass)"]
fn area_pack_live_fetch() {
    let dir = temp_dir("live");
    let (ok, stdout, stderr) = run({
        let mut c = tool();
        c.args([
            "--fetch",
            "400",
            "--lat",
            &HOME_LAT.to_string(),
            "--lon",
            &HOME_LON.to_string(),
            "--seed",
            &SEED.to_string(),
            "--out",
        ])
        .arg(&dir);
        c
    });
    assert!(ok, "live fetch failed: {stdout}{stderr}");
    println!("live fetch: {stdout}");
    let (ok, _out, err) = validate(&dir);
    assert!(ok, "validator rejected the live-fetched pack: {err}");
    let text = read_pack_text(&dir);
    assert_eq!(str_in(&text, "schema"), "darter_area_pack");
    assert!(json_object(json_object(&text, "source"), "fetched").contains("endpoint"));
    assert!(num_in(json_object(&text, "counts"), "buildings") > 0.0);
    let _ = std::fs::remove_dir_all(&dir);
}