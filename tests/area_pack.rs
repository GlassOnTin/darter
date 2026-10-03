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
//! The DEM fixture (tests/fixtures/dem_home_area.tif) is a clip of the
//! Copernicus GLO-30 home tile around the fixture origin: source tile
//! N50_00_W002_00 (2400x3600 float32 deflate + floating-point predictor,
//! sha256 e2d23f4652b1f2e3bf01a29e20ea315d728799db6519fe2c2f50aaf7c18f2134,
//! fetched 2026-10-01), clipped to rows 283..444 / cols 2174..2344, written
//! as uncompressed float32 strips by the importer's own decoder and pinned
//! here by sha256 b298b3a345cfc2c1462a751ecb429f34a2db21a1d0fccd080b6df5094
//! 33b55cf. glo30 tests assert the pack against that pinned sha, so a DEM
//! re-clip cannot silently change the grid.
//!
//! The tool is stdlib-only Python (no pinned interpreter — provenance lives
//! in the pack itself: input sha256 + seed + origin, and the byte-identity
//! gate re-runs here with whatever interpreter builds the pack).

use std::path::{Path, PathBuf};
use std::process::Command;

const FIXTURE: &str = "tests/fixtures/osm_home_area.json";
const DEM_FIXTURE: &str = "tests/fixtures/dem_home_area.tif";
/// sha256 of DEM_FIXTURE (provenance in the header; re-clips fail by design).
const DEM_FIXTURE_SHA256: &str = "b298b3a345cfc2c1462a751ecb429f34\
a2db21a1d0fccd080b6df509433b55cf";
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

/// Inner text of a top-level JSON object `"<key>": { ... }`.
/// (pack.json string values carry no braces, so depth counting is safe.)
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

/// Inner text of a top-level JSON `"<key>": <value>` where the value is an
/// object OR an array (buildings/grass/roads/strips/trees are arrays,
/// bounds/counts/source are objects). Same no-brackets-inside-strings
/// assumption as json_object.
fn json_value<'a>(text: &'a str, key: &str) -> &'a str {
    // newline + single-space indent anchors the search to a TOP-LEVEL
    // member: counts carries nested "grass"/"roads"/"trees" counters that
    // would otherwise shadow the arrays of the same names
    let pat = format!("\n \"{key}\":");
    let i = text
        .find(&pat)
        .unwrap_or_else(|| panic!("key {key} missing in pack.json"));
    let rest = &text[i + pat.len()..];
    let start = i + pat.len() + (rest.len() - rest.trim_start().len());
    let open = text.as_bytes()[start];
    let close = match open {
        b'{' => b'}',
        b'[' => b']',
        b'"' => {
            // fixed tool-generated strings (schema, frame): no escapes
            let end = text[start + 1..].find('"').expect("string value closed")
                + start
                + 1;
            return &text[start + 1..end];
        }
        b'0'..=b'9' | b'-' => {
            // bare number (seed): no nesting inside pack.json numbers
            let end = text[start..].find(['\n', ',']).expect("number terminated")
                + start;
            return text[start..end].trim_end();
        }
        b => panic!("key {key}: unexpected value start byte {b}"),
    };
    let bytes = text.as_bytes();
    let mut depth = 1usize;
    let mut j = start + 1;
    while depth > 0 {
        match bytes[j] {
            b if b == open => depth += 1,
            b if b == close => depth -= 1,
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

/// Metadata shared by every pack regardless of elevation model; the callers
/// add the model-specific elevation block and span identity. Returns the
/// span derived from the bounds (2*max extent + GROUND_PAD).
fn assert_common_metadata(text: &str, dir: &Path) -> f64 {
    assert_eq!(str_in(text, "schema"), "darter_area_pack");
    // v3: glo30 elevation object + terrain.bin sidecar (M1, 2026-10-01).
    assert!((num_in(text, "version") - 3.0).abs() < f64::EPSILON);
    assert!(str_in(text, "attribution").contains("OpenStreetMap"));
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
    2.0 * min_x.abs().max(max_x.abs()).max(min_y.abs()).max(max_y.abs()) + 200.0
}

/// Flat-pack metadata: elevation == flat and the GROUND_PAD span formula.
fn assert_metadata(text: &str, dir: &Path) {
    let derived = assert_common_metadata(text, dir);
    let elevation = json_object(text, "elevation");
    assert_eq!(str_in(elevation, "model"), "flat");
    let stored = num_in(text, "span_m");
    assert!((stored - derived).abs() < 0.01, "span {stored} vs derived {derived}");
}

/// Fixture-only provenance (assert_glo30_metadata is shared with the live
/// test, whose tile name and sha differ).
fn assert_dem_fixture_recorded(text: &str) {
    let elevation = json_object(text, "elevation");
    assert!(
        elevation.contains("\"name\": \"dem_home_area.tif\""),
        "fixture tile name not recorded in the elevation block"
    );
    assert!(
        text.contains(&format!("\"sha256\": \"{DEM_FIXTURE_SHA256}\"")),
        "fixture sha not recorded in pack.json"
    );
}

/// glo30-pack metadata: elevation block shape, the terrain.bin sidecar
/// (size, datum node == 0), and the snapped span identity (cols-1)*step,
/// which only rounds the derived span UP.
fn assert_glo30_metadata(text: &str, dir: &Path) {
    let derived = assert_common_metadata(text, dir);
    let elevation = json_object(text, "elevation");
    assert_eq!(str_in(elevation, "model"), "glo30");
    assert_eq!(str_in(elevation, "datum"), "origin_ground");
    assert!(str_in(elevation, "licence").contains("Copernicus"));
    assert!(str_in(text, "attribution").contains("Copernicus DEM"));

    let cols = num_in(elevation, "cols") as usize;
    let rows = num_in(elevation, "rows") as usize;
    let step = num_in(elevation, "step_m");
    assert!(cols >= 2 && rows >= 2, "degenerate grid {cols}x{rows}");
    assert!((step - 30.0).abs() < f64::EPSILON, "step {step}");

    // relief envelope: measured on the pinned fixture (2026-10-01,
    // z -20.5..+74.2); generous bounds catch a re-clip or decoder break
    // without pinning the exact lattice
    let z_min = num_in(elevation, "z_min");
    let z_max = num_in(elevation, "z_max");
    assert!((-60.0..=-5.0).contains(&z_min), "z_min {z_min} off envelope");
    assert!((20.0..=150.0).contains(&z_max), "z_max {z_max} off envelope");

    // terrain.bin sidecar: exact size, datum node (0,0) == 0.0
    let bin = std::fs::read(dir.join("terrain.bin")).expect("terrain.bin");
    assert_eq!(
        bin.len(),
        56 + 8 * cols * rows,
        "terrain.bin size {} vs 56 + 8*{cols}*{rows}",
        bin.len()
    );
    let z00 = f64::from_le_bytes(bin[56..64].try_into().expect("8 bytes"));
    assert!((z00 - 0.0).abs() < f64::EPSILON, "datum node not 0.0: {z00}");

    // snapped span: exact (cols-1)*step identity, at most one node step
    // above the derived span, never below it
    let stored = num_in(text, "span_m");
    assert!(
        (stored - (cols - 1) as f64 * step).abs() < 0.001,
        "span {stored} != (cols-1)*step {}",
        (cols - 1) as f64 * step
    );
    assert!(stored >= derived - 0.001, "snap {stored} below derived {derived}");
    assert!(stored - derived < 30.0 + 0.001, "snap {stored} far above {derived}");
}

#[test]
fn area_pack_build_validate_metadata() {
    let dir = temp_dir("meta");
    build_pack(&dir);
    let (ok, _out, err) = validate(&dir);
    assert!(ok, "validator rejected a freshly built pack: {err}");

    let text = read_pack_text(&dir);
    assert_metadata(&text, &dir);
    let _ = std::fs::remove_dir_all(&dir);
}

/// glo30 from the committed DEM fixture (offline): build, validate, and
/// cross-check the elevation block + terrain.bin against the pinned demo
/// provenance. This is the M1 headline test: DEM -> grid -> drape -> pack.
fn build_pack_glo30(dir: &Path) {
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
            "--elevation",
            "glo30",
            "--dem-file",
            DEM_FIXTURE,
            "--out",
        ])
        .arg(dir);
        c
    });
    assert!(ok, "glo30 build failed: {stdout}{stderr}");
    println!("build glo30: {stdout}");
}

#[test]
fn area_pack_demfile_glo30_build_validate() {
    let dir = temp_dir("glo30");
    build_pack_glo30(&dir);
    let (ok, _out, err) = validate(&dir);
    assert!(ok, "validator rejected the glo30 pack: {err}");
    let text = read_pack_text(&dir);
    assert_glo30_metadata(&text, &dir);
    assert_dem_fixture_recorded(&text);
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn area_pack_demfile_glo30_byte_identical_rebuild() {
    let a = temp_dir("glo30-byte-a");
    let b = temp_dir("glo30-byte-b");
    build_pack_glo30(&a);
    build_pack_glo30(&b);
    for name in ["pack.json", "scene.obj", "terrain.bin"] {
        let fa = std::fs::read(a.join(name)).expect(name);
        let fb = std::fs::read(b.join(name)).expect(name);
        assert_eq!(fa, fb, "{name} not byte-identical across glo30 rebuilds");
        assert!(!fa.is_empty());
    }
    let text_b = read_pack_text(&b);
    assert_glo30_metadata(&text_b, &b);
    assert_dem_fixture_recorded(&text_b);
    let _ = std::fs::remove_dir_all(&a);
    let _ = std::fs::remove_dir_all(&b);
}

/// The drape consumes no RNG: the seeded scene streams (grass scatter,
/// building mats, tree params) must produce the same arrays whether the
/// geometry is flat or draped, so a glo30 build keeps today's scene and
/// only the elevation block, the snapped span, and the attribution differ.
#[test]
fn area_pack_glo30_drape_rng_untouched() {
    let flat = temp_dir("drape-flat");
    let glo = temp_dir("drape-glo");
    build_pack(&flat);
    build_pack_glo30(&glo);
    let flat_text = read_pack_text(&flat);
    let glo_text = read_pack_text(&glo);
    // per-object slices (bounds, buildings, counts, frame, grass, roads,
    // schema, seed, source, strips, trees) and the scalar flag
    for key in [
        "bounds",
        "buildings",
        "counts",
        "frame",
        "grass",
        "roads",
        "schema",
        "seed",
        "source",
        "strips",
        "trees",
    ] {
        assert_eq!(
            json_value(&flat_text, key),
            json_value(&glo_text, key),
            "{key} diverged between flat and glo30 (drape consumed RNG?)"
        );
    }
    assert!(flat_text.contains("\"origin_inside_building\": false"));
    assert!(glo_text.contains("\"origin_inside_building\": false"));
    let _ = std::fs::remove_dir_all(&flat);
    let _ = std::fs::remove_dir_all(&glo);
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
            text.replacen("\"version\": 3", "\"version\": 2", 1),
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

/// terrain-corruption cases (mutate pack.json or terrain.bin, keep the
/// other files): a flipped elevation-sha hex (file sha no longer matches
/// the JSON), a truncated sidecar (size check), and one flipped payload
/// byte of a deep interior node (off the 0.1 m lattice AND off the recorded
/// sha). tiles[] shas are provenance only — the pack carries no DEM bytes,
/// so the validator cannot re-derive them; corrupting one there is NOT
/// caught offline (stated here rather than hidden).
#[test]
fn area_pack_validator_rejects_terrain_corruption() {
    let base = temp_dir("tc-base");
    build_pack_glo30(&base);
    let text = read_pack_text(&base);
    let full_bin = std::fs::read(base.join("terrain.bin")).expect("terrain.bin");

    // target the elevation block's own sha256 (the terrain.bin bytes' sha,
    // hashed here independently of the tool that recorded it)
    let mut h = darter_core::sha256::Sha256::new();
    h.update(&full_bin);
    let terr_hex = darter_core::sha256::to_hex(&h.finish());
    let sha_field = format!("\"sha256\": \"{terr_hex}\"");
    let i = text
        .find(&sha_field)
        .expect("terrain.bin sha recorded in pack.json");
    // hex digits sit at field indices 11..len-2, so the last digit is at
    // len-2 (len-1 is the closing quote — pointing there ate the quote and
    // produced an unterminated string)
    let dig = i + sha_field.len() - 2;
    let repl = if text.as_bytes()[dig] == b'2' { "3" } else { "2" };
    let sha_mut = format!("{}{}{}", &text[..dig], repl, &text[dig + 1..]);
    assert_ne!(sha_mut, text, "sha mutation did not apply");

    let mut truncated = full_bin.clone();
    truncated.truncate(full_bin.len() - 8);
    let mut node_flip = full_bin.clone();
    node_flip[56 + 64] ^= 0x01;

    let cases: Vec<(&str, &str, &Vec<u8>)> = vec![
        ("terrain-sha", &sha_mut, &full_bin),
        ("truncate", &text, &truncated),
        ("node-byte", &text, &node_flip),
    ];
    for (name, mutated_json, mutated_bin) in cases {
        let d = temp_dir(&format!("tc-{name}"));
        std::fs::create_dir_all(&d).unwrap();
        std::fs::write(d.join("pack.json"), mutated_json).unwrap();
        std::fs::write(d.join("terrain.bin"), mutated_bin).unwrap();
        std::fs::copy(base.join("scene.obj"), d.join("scene.obj")).unwrap();
        let (ok, _out, err) = validate(&d);
        assert!(!ok, "validator accepted corrupted pack ({name})");
        assert!(err.contains("INVALID"), "{name}: no INVALID in {err}");
        let _ = std::fs::remove_dir_all(&d);
    }
    let _ = std::fs::remove_dir_all(&base);
}
// ---- curated demo-area fixtures (VISION pillar 3: city/suburb/coast/hills) ----
//
// Three new fixture pairs fetched 2026-10-03 (Overpass caches from the named
// endpoints, tools/area_pack.py:67-68) and GLO-30 clips cut from the cached
// full tiles by tools/cut_dem.py in the dem_home_area.tif style. Per-area
// relief and counts are dated measurements from THESE bytes; any re-fetch or
// re-clip changes the shas and fails here by design.
//
// Each pair drives a full offline glo30 pack build (validator green) whose
// terrain.bin was checked byte-equal against a full-tile build cut from the
// same cache (S1 evidence, recorded in the commit message) — the committed
// clip covers every sampled node.

struct DemoArea {
    name: &'static str,
    lat: f64,
    lon: f64,
    seed: u64,
    /// sha256 of the committed osm_<name>.json bytes
    osm_sha256: &'static str,
    /// sha256 of the committed dem_<name>.tif bytes
    dem_sha256: &'static str,
    /// sha256 of the source GLO-30 tile (from the clip's provenance)
    full_tile_sha256: &'static str,
    /// the clip's source tile (must appear in the provenance sentence)
    glo30_tile: &'static str,
    min_elements: usize,
    min_buildings: usize,
    min_roads: usize,
    z_min: f64,
    z_max: f64,
    min_span_m: f64,
}

// Pin table for the 2026-10-03 fetches. All three went through the maps.mail.ru
// Overpass instance (0.7.62.4, data timestamp 2026-10-03) — both named endpoints
// in fetch_osm 406'd/504'd fleet-wide that day; the real endpoint is recorded
// honestly in tools/godot_smoke/areas/index.json at S3.
// Dated origin nudges (plan risk 4, one per area): coast Calshot Spit ->
// Stone shore (50.8130,-1.3070; the spit shingle is unmapped, the village west
// of it is); hills Butser -> (50.9761,-0.9457), 318 m S / 163 m E of the plan
// pin, so the 4530 m grid stays fully inside tile N50_00_W001_00 and one
// committed clip covers every node.
const DEMO_AREAS: &[DemoArea] = &[
    DemoArea { name: "city",  lat: 50.9060, lon: -1.4012, seed: 11, osm_sha256: "84d5a655f5507e639dba5ab08cf8e7b110c238d3352d466622b1f6c146d170a8", dem_sha256: "44f286eba288248851253c760d463f90d4920b2164225b968105d333be7282d0", full_tile_sha256: "e2d23f4652b1f2e3bf01a29e20ea315d728799db6519fe2c2f50aaf7c18f2134", glo30_tile: "N50_00_W002_00", min_elements: 3793, min_buildings: 1740, min_roads: 1488, z_min: -2.8, z_max: 36.9, min_span_m: 2190.0 },
    DemoArea { name: "coast", lat: 50.8130, lon: -1.3070, seed: 13, osm_sha256: "d22680be8978f761e140eefe110d2cf4d52299813d96378309276549417eda29", dem_sha256: "84d3ce38735ea9b36ef1d9af374c0b500d422fa311f8e1aa6e6015e8dfa54168", full_tile_sha256: "e2d23f4652b1f2e3bf01a29e20ea315d728799db6519fe2c2f50aaf7c18f2134", glo30_tile: "N50_00_W002_00", min_elements: 191, min_buildings: 162, min_roads: 22, z_min: -1.7, z_max: 16.6, min_span_m: 1950.0 },
    DemoArea { name: "hills", lat: 50.9761, lon: -0.9457, seed: 17, osm_sha256: "e226352c4583c938fb7345e0c377dd85646e399ccd6f9c22b474a891132bfb49", dem_sha256: "870d4f9318b7047c11d2eabd246ae0c5dbb9e93c3012517bf75150e7af435884", full_tile_sha256: "09609412e26651cfdc4ccf0e336f66581f4af64145670a11aae13848c7429c6d", glo30_tile: "N50_00_W001_00", min_elements: 304, min_buildings: 143, min_roads: 133, z_min: -94.2, z_max: 124.9, min_span_m: 4530.0 },
];

#[test]
fn demo_area_fixtures_inputs() {
    for a in DEMO_AREAS {
        let osm_path = format!("tests/fixtures/osm_{}.json", a.name);
        let dem_path = format!("tests/fixtures/dem_{}.tif", a.name);
        assert!(
            Path::new(&osm_path).exists(),
            "missing fixture {osm_path}: run the S1 fetch first"
        );
        assert!(
            Path::new(&dem_path).exists(),
            "missing fixture {dem_path}: cut it with tools/cut_dem.py"
        );

        // bytes pinned: any re-fetch or re-clip fails here
        let osm_bytes = std::fs::read(&osm_path).expect("read overpass fixture");
        let mut h = darter_core::sha256::Sha256::new();
        h.update(&osm_bytes);
        assert_eq!(
            darter_core::sha256::to_hex(&h.finish()),
            a.osm_sha256,
            "osm_{}.json drifted from the 2026-10-03 pin",
            a.name
        );
        let dem_bytes = std::fs::read(&dem_path).expect("read dem clip");
        let mut h = darter_core::sha256::Sha256::new();
        h.update(&dem_bytes);
        assert_eq!(
            darter_core::sha256::to_hex(&h.finish()),
            a.dem_sha256,
            "dem_{}.tif drifted from the 2026-10-03 pin",
            a.name
        );

        // the Overpass cache parses and has the measured breadth
        let data: serde_json::Value =
            serde_json::from_slice(&osm_bytes).expect("overpass json parse");
        let elements = data
            .get("elements")
            .and_then(|v| v.as_array())
            .expect("overpass elements array");
        assert!(
            elements.len() >= a.min_elements,
            "{osm_path}: {} elements below dated floor {}",
            elements.len(),
            a.min_elements
        );
        let buildings = elements
            .iter()
            .filter(|e| {
                e.get("type").and_then(|v| v.as_str()) == Some("way")
                    && e.get("tags")
                        .and_then(|v| v.get("building"))
                        .is_some()
            })
            .count();
        assert!(
            buildings >= a.min_buildings,
            "{osm_path}: {} building ways below dated floor {}",
            buildings,
            a.min_buildings
        );
        let roads = elements
            .iter()
            .filter(|e| {
                e.get("type").and_then(|v| v.as_str()) == Some("way")
                    && e.get("tags")
                        .and_then(|v| v.get("highway"))
                        .is_some()
            })
            .count();
        assert!(
            roads >= a.min_roads,
            "{osm_path}: {} highway ways below dated floor {}",
            roads,
            a.min_roads
        );

        // clip provenance sentence (ImageDescription, raw byte search)
        let raw = String::from_utf8_lossy(&dem_bytes);
        assert!(
            raw.contains("darter DEM clip"),
            "dem_{}.tif: provenance sentence missing",
            a.name
        );
        assert!(
            raw.contains(a.glo30_tile),
            "dem_{}.tif: provenance must name the source tile {}",
            a.name,
            a.glo30_tile
        );
        assert!(
            raw.contains(a.full_tile_sha256),
            "dem_{}.tif: provenance must carry the full-tile sha256",
            a.name
        );

        // offline build from the committed pair + the tool's own validator
        let dir = temp_dir(&format!("demo-{}", a.name));
        let (ok, stdout, stderr) = run({
            let mut c = tool();
            c.args([
                "--osm",
                &osm_path,
                "--lat",
                &a.lat.to_string(),
                "--lon",
                &a.lon.to_string(),
                "--seed",
                &a.seed.to_string(),
                "--elevation",
                "glo30",
                "--dem-file",
                &dem_path,
                "--out",
            ])
            .arg(&dir);
            c
        });
        assert!(ok, "{} build failed: {stdout}{stderr}", a.name);
        let (ok, _out, err) = validate(&dir);
        assert!(ok, "::validator rejected the {} pack: {err}", a.name);

        let text = read_pack_text(&dir);
        assert_eq!(str_in(&text, "schema"), "darter_area_pack");
        assert!((num_in(&text, "version") - 3.0).abs() < f64::EPSILON);
        assert!(str_in(&text, "attribution").contains("OpenStreetMap"));
        assert!(str_in(&text, "attribution").contains("Copernicus DEM"));
        assert!((num_in(&text, "seed") - a.seed as f64).abs() < f64::EPSILON);
        assert!((num_in(&text, "origin_lat") - a.lat).abs() < 1e-9);
        assert!((num_in(&text, "origin_lon") - a.lon).abs() < 1e-9);
        // input sha equals the committed fixture bytes (same path as
        // assert_common_metadata's check)
        assert_eq!(str_in(&text, "osm_json_sha256"), a.osm_sha256);
        assert!(
            text.contains("\"origin_inside_building\": false"),
            "{}: origin inside a building",
            a.name
        );

        // elevation block: the dated relief pins + the snap identity
        let elevation = json_object(&text, "elevation");
        assert_eq!(str_in(elevation, "model"), "glo30");
        assert_eq!(str_in(elevation, "datum"), "origin_ground");
        assert!((num_in(elevation, "step_m") - 30.0).abs() < f64::EPSILON);
        let z_min = num_in(elevation, "z_min");
        let z_max = num_in(elevation, "z_max");
        assert!(
            (z_min - a.z_min).abs() <= 0.05 && (z_max - a.z_max).abs() <= 0.05,
            "{}: relief {z_min}..{z_max} vs pinned {}..{}",
            a.name,
            a.z_min,
            a.z_max
        );
        let cols = num_in(elevation, "cols") as usize;
        let rows = num_in(elevation, "rows") as usize;
        let stored = num_in(&text, "span_m");
        assert!(
            (stored - (cols - 1) as f64 * 30.0).abs() < 0.001,
            "{}: span {} != snapped (cols-1)*30.0",
            a.name,
            stored
        );
        assert!(stored >= a.min_span_m, "{}: span {stored} below floor", a.name);

        // sidecar: exact size + datum node 0
        let bin = std::fs::read(dir.join("terrain.bin")).expect("terrain.bin");
        assert_eq!(bin.len(), 56 + 8 * cols * rows, "{}: terrain.bin size", a.name);
        let z00 = f64::from_le_bytes(bin[56..64].try_into().expect("8 bytes"));
        assert!((z00 - 0.0).abs() < f64::EPSILON, "{}: datum node not 0.0", a.name);

        let _ = std::fs::remove_dir_all(&dir);
    }
}

// ---- S3: the committed areas index (tools/godot_smoke/areas/index.json) ----
//
// The payload layout under the same paths (areas/<name>) is asserted by T8's
// bundle-vs-regen equality (tests/godot_pack.rs); this plain test owns the
// committed-side identity: schema/order, attribution verbatim, per-area
// input shas vs the fixture bytes, seeds vs the S1 pin table, and the four
// track files' shape (6 gate rings, radius 4.0) — the deep darter_track
// parse itself lives in sim_run and is exercised by T8 slice (b) per area.

const AREAS_INDEX: &str = "tools/godot_smoke/areas/index.json";
const TRACK_DIR: &str = "tools/godot_smoke/track";
const ATTRIBUTION: &str =
    "© OpenStreetMap contributors, ODbL 1.0 | © Copernicus DEM / ESA (GLO-30)";

const AREA_ORDER: [&str; 4] = ["city", "suburb", "coast", "hills"];
const RECORD_SEEDS: [(&str, u64); 4] =
    [("city", 21), ("suburb", 7), ("coast", 23), ("hills", 29)];
const MAX_CROSSINGS: [(&str, f64); 4] =
    [("city", 17.793), ("suburb", 17.035), ("coast", 17.805), ("hills", 17.826)];

#[test]
fn demo_areas_index_commit() {
    let text = std::fs::read_to_string(AREAS_INDEX)
        .expect("read areas index.json (S3)");
    let idx: serde_json::Value = serde_json::from_str(&text).expect("index.json parse");

    assert_eq!(
        idx.get("schema").and_then(|v| v.as_str()),
        Some("darter_areas"),
        "index schema"
    );
    assert!(idx.get("version").and_then(|v| v.as_i64()) == Some(1), "index version 1");
    assert_eq!(
        idx.get("attribution").and_then(|v| v.as_str()),
        Some(ATTRIBUTION),
        "attribution must be the verbatim ODbL + Copernicus credit"
    );

    let areas = idx
        .get("areas")
        .and_then(|v| v.as_array())
        .expect("areas array");
    assert_eq!(areas.len(), 4, "four demo areas (city, suburb, coast, hills)");

    for (row, want_name) in areas.iter().zip(AREA_ORDER.iter()) {
        assert_eq!(
            row.get("name").and_then(|v| v.as_str()),
            Some(*want_name),
            "area order: index must match the AREAS table (city, suburb, coast, hills)"
        );

        // track ref: uniform area-local name, source file parses as a
        // darter_track v1 gate ring stack
        let track_ref = row
            .get("track")
            .and_then(|v| v.as_str())
            .expect("track ref");
        assert_eq!(track_ref, "track.json", "{want_name}: bundle-local track ref");
        // the corridor's source track keeps its historical file name; the
        // three new areas live next to it as <name>.json — both bundle to
        // areas/<name>/track.json (T8's (e) equality covers the copy)
        let track_src = if *want_name == "suburb" { "demo_track.json" } else { &format!("{want_name}.json") };
        let track_path = format!("{TRACK_DIR}/{track_src}");
        let track_text = std::fs::read_to_string(&track_path)
            .unwrap_or_else(|e| panic!("read {track_path}: {e}"));
        let track: serde_json::Value =
            serde_json::from_str(&track_text).expect("track.json parse");
        assert_eq!(
            track.get("schema").and_then(|v| v.as_str()),
            Some("darter_track"),
            "{track_path}: schema"
        );
        assert!(track.get("version").and_then(|v| v.as_i64()) == Some(1));
        let cps = track
            .get("checkpoints")
            .and_then(|v| v.as_array())
            .expect("checkpoints array");
        assert_eq!(cps.len(), 6, "{track_path}: 6 checkpoints");
        for cp in cps.iter() {
            assert_eq!(cp.get("kind").and_then(|v| v.as_str()), Some("gate"));
            assert!(cp.get("radius_m").and_then(|v| v.as_f64()) == Some(4.0));
            assert!(cp.get("x").and_then(|v| v.as_f64()).is_some());
            assert!(cp.get("y").and_then(|v| v.as_f64()).is_some());
            assert!(cp.get("z").and_then(|v| v.as_f64()).is_some());
        }
        assert!(track.get("spawn").and_then(|v| v.get("x")).and_then(|v| v.as_f64()).is_some());

        // inputs vs the committed fixture bytes
        let inputs = row.get("inputs").expect("inputs block");
        let osm_name = inputs.get("osm").and_then(|v| v.as_str()).expect("osm input name");
        let osm_bytes = std::fs::read(format!("tests/fixtures/{osm_name}")).expect("osm fixture");
        let osm_sha = {
            let mut h = darter_core::sha256::Sha256::new();
            h.update(&osm_bytes);
            darter_core::sha256::to_hex(&h.finish())
        };
        assert_eq!(
            inputs.get("osm_sha256").and_then(|v| v.as_str()),
            Some(osm_sha.as_str()),
            "{want_name}: osm sha vs {osm_name} bytes"
        );
        let dem_name = inputs.get("dem").and_then(|v| v.as_str()).expect("dem input name");
        let dem_bytes = std::fs::read(format!("tests/fixtures/{dem_name}")).expect("dem fixture");
        let dem_sha = {
            let mut h = darter_core::sha256::Sha256::new();
            h.update(&dem_bytes);
            darter_core::sha256::to_hex(&h.finish())
        };
        assert_eq!(
            inputs.get("dem_sha256").and_then(|v| v.as_str()),
            Some(dem_sha.as_str()),
            "{want_name}: dem sha vs {dem_name} bytes"
        );

        // seeds + origin vs the pin table (DEMO_AREAS for the three new
        // areas, the home constants for suburb)
        let lat = row.get("lat").and_then(|v| v.as_f64()).expect("lat");
        let lon = row.get("lon").and_then(|v| v.as_f64()).expect("lon");
        let seed = row.get("seed").and_then(|v| v.as_i64()).expect("seed");
        let rseed = row
            .get("record_seed")
            .and_then(|v| v.as_i64())
            .expect("record_seed");
        assert_eq!(rseed as u64, RECORD_SEEDS.iter().find(|(n, _)| *n == *want_name).expect("record seed pin").1);
        if *want_name == "suburb" {
            assert!((lat - HOME_LAT).abs() < 1e-9);
            assert!((lon - HOME_LON).abs() < 1e-9);
            assert_eq!(seed as u64, SEED);
        } else {
            let pin = DEMO_AREAS.iter().find(|a| a.name == *want_name).expect("pin");
            assert!((lat - pin.lat).abs() < 1e-9, "{want_name}: lat");
            assert!((lon - pin.lon).abs() < 1e-9, "{want_name}: lon");
            assert_eq!(seed as u64, pin.seed, "{want_name}: pack seed");
        }

        // measured crossings stay inside the replay window (T8 gates < 19 s)
        let crossing = row
            .get("record")
            .and_then(|v| v.get("max_crossing_s"))
            .and_then(|v| v.as_f64())
            .expect("record max_crossing_s");
        assert_eq!(crossing, MAX_CROSSINGS.iter().find(|(n, _)| *n == *want_name).expect("crossing pin").1);
        assert!(crossing <= 19.0, "{want_name}: crossing {crossing} beyond 19 s");
    }
}

// ---- water rung (fixture: osm_water_toy.json, hand-authored: see the
// generator comment inside the fixture). Two coastline ways share an exact
// endpoint node and chain into one sea polygon (land inside on the LEFT of
// the west->east travel, sea on the RIGHT), a closed coastline ring in the
// sea band is an islet (land inside, CCW -> the sea polygon's hole), and a
// closed natural=water ring on the land side is an explicit water polygon.
// Water polygons are clipped to the ground rect and their points are
// EXCLUDED from bounds/span (the clip box derives from the span: circular).
// Flat-pack z ladder: water polygons land at 0.020 + 0.002 * k (2 mm
// stagger against overlapping fills; above the grass drape at 0.005, below
// the 0.05 road ladder). ----

const WATER_FIXTURE: &str = "tests/fixtures/osm_water_toy.json";
const WATER_LAT: f64 = 51.0;
const WATER_LON: f64 = -2.0;
const WATER_SEED: u64 = 13;

/// Shoelace signed area from a tool-formatted `[\n [x,\n y], ...]` list of
/// coordinate pairs (open ring: the endpoint is not repeated). Positive =
/// CCW in the pack's y-north frame.
fn ring_area(list_text: &str) -> f64 {
    let bytes = list_text.as_bytes();
    let mut pts: Vec<(f64, f64)> = Vec::new();
    let mut i = 0;
    while i < bytes.len() {
        if bytes[i] == b'[' {
            let close = list_text[i..].find(']').expect("pair close");
            let inner = &list_text[i + 1..i + close];
            if !inner.contains('[') {
                let mut it = inner.split(',');
                let x: f64 = it.next().unwrap().trim().parse().expect("pair x");
                let y: f64 = it.next().unwrap().trim().parse().expect("pair y");
                pts.push((x, y));
            }
        }
        i += 1;
    }
    assert!(pts.len() >= 3, "ring has {} points, need >= 3", pts.len());
    let mut a = 0.0;
    for k in 0..pts.len() {
        let (x1, y1) = pts[k];
        let (x2, y2) = pts[(k + 1) % pts.len()];
        a += x1 * y2 - x2 * y1;
    }
    a * 0.5
}

/// Items of a tool-formatted JSON array's inner text: depth-split on the
/// top-level brackets (works for `holes`, a list of rings).
fn list_items(list_text: &str) -> Vec<&str> {
    let bytes = list_text.as_bytes();
    let mut items = Vec::new();
    let mut depth = 0usize;
    let mut start = 0usize;
    let mut open = false;
    for (i, b) in bytes.iter().enumerate() {
        match b {
            b'[' => {
                depth += 1;
                if depth == 1 {
                    start = i;
                    open = true;
                }
            }
            b']' => {
                depth -= 1;
                if open && depth == 0 {
                    items.push(&list_text[start..=i]);
                    open = false;
                }
            }
            _ => {}
        }
    }
    items
}

/// Inner text of an entry-level `"<key>": <value>` member (container
/// values only; pack strings carry no key names).
fn json_member<'a>(text: &'a str, key: &str) -> &'a str {
    let pat = format!("\"{key}\": ");
    let i = text
        .find(&pat)
        .unwrap_or_else(|| panic!("member {key} missing"));
    let bytes = text.as_bytes();
    let start = i + pat.len();
    let open = bytes[start];
    let close = match open {
        b'{' => b'}',
        b'[' => b']',
        _ => panic!("member {key}: value is not a container"),
    };
    let mut depth = 1usize;
    let mut j = start + 1;
    while depth > 0 {
        match bytes[j] {
            b if b == open => depth += 1,
            b if b == close => depth -= 1,
            _ => {}
        }
        j += 1;
    }
    &text[start + 1..j - 1]
}

fn build_water_pack(dir: &Path) {
    let (ok, stdout, stderr) = run({
        let mut c = tool();
        c.args([
            "--osm",
            WATER_FIXTURE,
            "--lat",
            &WATER_LAT.to_string(),
            "--lon",
            &WATER_LON.to_string(),
            "--seed",
            &WATER_SEED.to_string(),
            "--out",
        ])
        .arg(dir);
        c
    });
    assert!(ok, "water-pack build failed: {stdout}{stderr}");
}

#[test]
fn area_pack_water_polygons() {
    let dir = temp_dir("water");
    build_water_pack(&dir);
    let text = read_pack_text(&dir);

    // the validator's water rules (area identity, ring orientation, counts)
    let (ok, _out, err) = validate(&dir);
    assert!(ok, "validator rejected the water pack: {err}");

    // counts.water == 2 (sea + lake; the islet is a hole, not a polygon)
    let counts = json_object(&text, "counts");
    let n_water = num_in(counts, "water") as usize;
    assert_eq!(n_water, 2, "sea + lake");
    assert_eq!(text.matches("\"id\": \"w").count(), 2, "water ids");
    // building survived alongside the water (ids never collide)
    assert_eq!(text.matches("\"id\": \"b").count(), 1, "building count");
    let obj = std::fs::read_to_string(dir.join("scene.obj")).expect("scene.obj");
    assert_eq!(count_lines(&obj, "o bld_"), 1, "bld groups");
    assert_eq!(count_lines(&obj, "o water_"), 2, "water groups");

    // bound/span rule: the building alone drives the span here (water
    // points are excluded). Building max abs = 84 -> 2*84 + 200 = 368;
    // the ground rect is +-184, so the coastline must be clipped AT the
    // rect (never wider), and the sea fill still touches the clip edges.
    let derived_span = 2.0 * (26.0f64.max(84.0)) + 200.0;
    let stored = num_in(&text, "span_m");
    assert!((stored - derived_span).abs() < 0.01, "span {stored}");

    let water = json_value(&text, "water");
    // entries are `{...}` objects in order: w0 = sea, w1 = the lake
    let entries = water.split('{').collect::<Vec<&str>>();
    assert_eq!(entries.len(), 3, "two water entries");
    let sea = entries[1];
    let lake = entries[2];
    assert!(sea.contains("\"id\": \"w0\""), "{sea}");
    assert!(sea.contains("\"kind\": \"sea\""), "w0 is the sea");
    assert!(lake.contains("\"id\": \"w1\""), "{lake}");
    assert!(lake.contains("\"kind\": \"water\""), "w1 is the lake");
    assert!(sea.contains("\"origin\": \"coastline\""), "sea origin");
    assert!(lake.contains("\"origin\": \"natural=water\""), "lake origin");

    // sea outer ring: CCW, touches the clip rect (the rect half is
    // derived_span / 2: +-184), and covers the whole clipped chain band
    let outer = json_member(sea, "outer");
    let sea_outer = ring_area(outer);
    assert!(
        sea_outer > 0.0,
        "sea outer must be CCW (positive), got {sea_outer}"
    );
    assert!(
        (70_000.0..95_000.0).contains(&sea_outer.abs()),
        "sea outer area {sea_outer}"
    );
    // clip-rect corner vertices survive (the arc closes at the rect)
    assert!(outer.contains("-184.0"), "sea touches the clip rect: {outer}");

    // the islet hole: a single CW ring, ~1200 m^2 (r=20 dodecagon)
    let holes = list_items(json_member(sea, "holes"));
    assert_eq!(holes.len(), 1, "one hole: the islet");
    let hole_area = ring_area(holes[0]);
    assert!(
        hole_area < 0.0,
        "holes must be CW (negative), got {hole_area}"
    );
    assert!(
        (1000.0..1500.0).contains(&hole_area.abs()),
        "islet hole area {hole_area}"
    );
    // area_m2 == outer + holes (holes carry negative signed area)
    let area = num_in(sea, "area_m2");
    assert!(
        (area - (sea_outer + hole_area)).abs() < 0.5,
        "sea area_m2 {area} vs rings {sea_outer} + {hole_area}"
    );

    // the lake: closed natural=water polygon, no holes, ~768 m^2
    assert_eq!(list_items(json_member(lake, "holes")).len(), 0);
    let lake_area = num_in(lake, "area_m2");
    assert!(
        (700.0..1000.0).contains(&lake_area),
        "lake area {lake_area} (r=16 dodecagon ~= 768)"
    );

    // flat-pack z ladder in the OBJ: sea group at 0.020, lake at 0.022
    for (group, z) in [("water_w0", "0.020"), ("water_w1", "0.022")] {
        let from = obj
            .find(&format!("o {group}"))
            .unwrap_or_else(|| panic!("o {group} present"));
        let zs = obj[from..]
            .lines()
            .skip(1)
            .take_while(|l| !l.starts_with("o "))
            .filter(|l| l.starts_with("v "))
            .map(|l| l.split_whitespace().nth(3).expect("v z").to_string())
            .collect::<Vec<_>>();
        assert!(zs.len() >= 3, "{group} has {} verts", zs.len());
        assert!(
            zs.iter().all(|zv| zv == z),
            "{group} z values {zs:?} vs {z}"
        );
    }

    // determinism: the pack rebuild is byte-identical (the flat build
    // writes exactly these two files — no terrain.bin)
    let dir2 = temp_dir("water-b");
    build_water_pack(&dir2);
    let dirs = [dir.clone(), dir2.clone()];
    for name in ["pack.json", "scene.obj"] {
        let a = std::fs::read(dirs[0].join(name)).expect(name);
        let b = std::fs::read(dirs[1].join(name)).expect(name);
        assert_eq!(a, b, "{name} not byte-identical across rebuilds");
    }
    let _ = std::fs::remove_dir_all(&dir);
    let _ = std::fs::remove_dir_all(&dir2);
}

/// The water count must not just be present but POLICED: a counts.water
/// that disagrees with the water array is rejected like any other family.
#[test]
fn area_pack_water_count_corruption_rejected() {
    let dir = temp_dir("water-corrupt");
    build_water_pack(&dir);
    let text = read_pack_text(&dir);
    // counts.water is the unique `"water": 2` literal (the top-level array
    // starts with `"water": [`, so the digits disambiguate)
    let key = "\"water\": 2";
    assert_eq!(text.matches(key).count(), 1, "counts.water entry unique");
    let corrupt = text.replacen(key, "\"water\": 3", 1);
    std::fs::write(dir.join("pack.json"), corrupt).expect("write corrupt pack");
    let (ok, _out, err) = validate(&dir);
    assert!(!ok, "validator accepted a counts.water mismatch");
    assert!(
        err.contains("water"),
        "rejection should name the family: {err}"
    );
    let _ = std::fs::remove_dir_all(&dir);
}

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
/// Opt-in: hits the live GLO-30 mirror (network) — the tile-download path
/// with no --dem-file. Home origin 50.8989 / -1.0586 floors into tile
/// N50_00_W002_00 (SW-corner naming, verified from the TIFF's own tags);
/// the recorded full-tile sha must equal the upstream bytes fetched
/// 2026-10-01 (e2d23f46...2134), so a Copernicus re-publication fails here
/// honestly and the pin is re-baselined with evidence.
#[test]
#[ignore = "needs network (live GLO-30 mirror)"]
fn area_pack_live_dem_fetch() {
    let dir = temp_dir("live-dem-pack");
    let cache = temp_dir("live-dem-cache");
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
            "--elevation",
            "glo30",
            "--dem-cache",
        ])
        .arg(&cache)
        .arg("--out")
        .arg(&dir);
        c
    });
    assert!(ok, "live glo30 build failed: {stdout}{stderr}");
    println!("live dem: {stdout}");
    assert!(
        stdout.contains("terrain tile: N50_00_W002_00.tif"),
        "expected the N50_00_W002_00 tile, got: {stdout}"
    );
    let (ok, _out, err) = validate(&dir);
    assert!(ok, "validator rejected the live-fetched glo30 pack: {err}");
    let text = read_pack_text(&dir);
    assert_glo30_metadata(&text, &dir);
    assert!(
        text.contains(
            "e2d23f4652b1f2e3bf01a29e20ea315d728799db6519fe2c2f50aaf7c18f2134"
        ),
        "live tile sha drifted from the 2026-10-01 pin"
    );
    let _ = std::fs::remove_dir_all(&dir);
    let _ = std::fs::remove_dir_all(&cache);
}
