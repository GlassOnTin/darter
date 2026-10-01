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
