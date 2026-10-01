//! T7 area-pack renderer integration: an M1 area pack (pack.json +
//! scene.obj + terrain.bin, GLO-30 relief from the committed fixture DEM) is
//! loaded by the GDScript pack reader (tools/godot_smoke/pack_replay.gd)
//! into one ArrayMesh and a real flight record — flown by sim_run over that
//! same terrain via --terrain, so the hills the camera sees are the hills
//! the physics flew — is replayed through the rendered neighbourhood. The
//! gates, in order:
//!
//! - the loader's own contract check: its o-group family counts must equal
//!   pack.json's counts (the consumer side of the T6 writer/validator
//!   contract; a mismatch aborts the run);
//! - the replay must deliver exactly 20 s x 30 fps = 600 frames whose
//!   sampled positions match the record (1e-6, same projection as T5);
//! - frames must be non-blank (brightness variance > 0, mean in bounds) AND
//!   carry scene content: the pixel-class fractions classify a 16-px sample
//!   grid into sky (blue-dominant), vegetation (green-dominant) and man-made
//!   (the rest). An empty scene (failed loader) has no green; a
//!   ground-plane-only scene has almost no man-made fraction. Both gates
//!   must hold, so the two degenerate scenes cannot pass for each other.
//!   Fog grey and sky blue both land in "sky"/"man-made" by colour — that is
//!   why the vegetation gate carries the empty-scene regression, not the
//!   man-made one;
//! - a second full run must reproduce the deterministic projection exactly
//!   (positions, brightness, and the class fractions — all derived from the
//!   record + the byte-identical pack; wall-clock frame deltas excluded).
//!
//! Opt-in (`cargo test --test godot_pack -- --ignored`) like tests/godot.rs:
//! it needs the pinned Godot binary (same provenance gate) plus the cached
//! Overpass fixture for the pack build. The render path is xvfb +
//! gl_compatibility only — T5 measured the headless path as impossible on
//! this machine (dummy render device; viewport read SIGSEGVs), so it is not
//! retried here.

use std::path::{Path, PathBuf};
use std::process::Command;

use darter_core::sha256::Sha256;

const GODOT_SHA256: &str = "8d106cbe6144c2dc7e881d61d2429c1a8a76e6b22ef48bd5e48dcf934953f71e";
const GODOT_VERSION: &str = "4.7.2.stable.official.ed1daf0bf";
const MOVIE_FPS: f64 = 30.0;
/// 20 s of record at the 30 fps Movie Maker rate.
const EXPECTED_FRAMES: usize = 600;

const FIXTURE: &str = "tests/fixtures/osm_home_area.json";
const HOME_LAT: f64 = 50.8989;
const HOME_LON: f64 = -1.0586;
const SEED: u64 = 5;

fn temp_dir(name: &str) -> PathBuf {
    let d = std::env::temp_dir().join(format!("darter-godot-pack-{name}-{}", std::process::id()));
    std::fs::create_dir_all(&d).unwrap();
    d
}

/// Locate and hash-verify the pinned Godot binary (same provenance as T5).
fn locate_godot() -> PathBuf {
    let path = match std::env::var("DARTER_GODOT") {
        Ok(p) => PathBuf::from(p),
        Err(_) => PathBuf::from("tools/godot/bin/godot"),
    };
    assert!(path.exists(), "Godot binary missing at {path:?}: run the download pinned in tests/godot.rs, or set DARTER_GODOT");
    let bytes = std::fs::read(&path).expect("read godot binary");
    let mut h = Sha256::new();
    h.update(&bytes);
    let got = darter_core::sha256::to_hex(&h.finish());
    assert_eq!(
        got, GODOT_SHA256,
        "Godot binary at {path:?} does not match the pinned provenance"
    );
    path
}

fn num(line: &str, key: &str) -> f64 {
    let pat = format!("\"{key}\":");
    let i = line.find(&pat).unwrap_or_else(|| panic!("field {key} missing in {line}"));
    let rest = &line[i + pat.len()..];
    let end = rest.find([',', '}']).unwrap_or(rest.len());
    rest[..end].parse().expect("float parse")
}

/// (t, px, py, pz) rows of a flight record.
fn read_record(path: &Path) -> Vec<(f64, f64, f64, f64)> {
    let text = std::fs::read_to_string(path).expect("flight record");
    let mut rows = Vec::new();
    for line in text.lines() {
        if line.starts_with("{\"schema") {
            continue;
        }
        rows.push((num(line, "t"), num(line, "px"), num(line, "py"), num(line, "pz")));
    }
    rows
}

struct Sample {
    t: f64,
    px: f64,
    py: f64,
    pz: f64,
    bm: f64,
    bv: f64,
    sky: f64,
    veg: f64,
    mm: f64,
    du: u64,
    /// Render primitives in the last completed frame (measurement only —
    /// canon() excludes it; the M1 relief growth is measured against it).
    pr: u64,
}

/// The deterministic projection for the determinism gate. du is wall-clock
/// and excluded (see tests/godot.rs); the class fractions are pure functions
/// of the record and the pack bytes, so they are included.
fn canon(samples: &[Sample]) -> String {
    samples
        .iter()
        .map(|s| {
            format!(
                "{},{},{},{},{:.9},{:.9},{:.4},{:.4},{:.4}",
                s.t, s.px, s.py, s.pz, s.bm, s.bv, s.sky, s.veg, s.mm
            )
        })
        .collect::<Vec<_>>()
        .join(";")
}

/// Parse the replay JSON's samples array (hand-formatted, fixed field order)
/// and the loader's pack summary block.
struct PackSummary {
    load_ms: u64,
    materials: usize,
    groups: Vec<(String, u64)>,
    /// (z_min, z_max) echoed from pack.json's elevation object (M1); None on
    /// a flat pack.
    relief: Option<(f64, f64)>,
}

fn parse_replay(path: &Path) -> (usize, Vec<Sample>, PackSummary) {
    let text = std::fs::read_to_string(path).expect("replay JSON");
    let i = text.find("\"samples\":[").expect("samples array");
    let body = &text[i + "\"samples\":[".len()..];
    let end = body.rfind(']').expect("samples terminator");
    let mut samples = Vec::new();
    for entry in body[..end].split("},{") {
        let e = entry.trim_start_matches('{').trim_end_matches('}');
        samples.push(Sample {
            t: num(e, "t"),
            px: num(e, "px"),
            py: num(e, "py"),
            pz: num(e, "pz"),
            bm: num(e, "bm"),
            bv: num(e, "bv"),
            sky: num(e, "sky"),
            veg: num(e, "veg"),
            mm: num(e, "mm"),
            du: num(e, "du") as u64,
            pr: num(e, "pr") as u64,
        });
    }
    let frames = num(&text[..i], "frames") as usize;

    let gk = "\"groups\":{";
    let gi = text.find(gk).expect("pack groups block");
    let gend = text[gi..].find('}').expect("pack groups terminator") + gi;
    let mut groups = Vec::new();
    for pair in text[gi + gk.len()..gend].split(',') {
        let (k, v) = pair.split_once(':').expect("group pair");
        groups.push((k.trim().trim_matches('"').to_string(), v.trim().parse().unwrap()));
    }
    let relief = match text[..i].find("\"relief\":{") {
        Some(ri) => {
            let block = &text[..i][ri + "\"relief\":{".len()..];
            let block = &block[..block.find('}').expect("relief block terminator")];
            Some((num(block, "z_min"), num(block, "z_max")))
        }
        None => None,
    };
    let pack = PackSummary {
        load_ms: num(&text[..i], "load_ms") as u64,
        materials: num(&text[..i], "materials") as usize,
        groups,
        relief,
    };
    (frames, samples, pack)
}

/// The gates: frame count, positions vs the record, non-blank brightness,
/// and scene content (vegetation + man-made fractions). Returns a summary.
fn validate(replay: &Path, record_rows: &[(f64, f64, f64, f64)]) -> Result<String, String> {
    let (frames, samples, _pack) = parse_replay(replay);
    if frames != EXPECTED_FRAMES {
        return Err(format!("frames {frames} vs expected {EXPECTED_FRAMES}"));
    }
    if samples.len() != EXPECTED_FRAMES {
        return Err(format!("samples {} vs expected {EXPECTED_FRAMES}", samples.len()));
    }
    let mut worst_pos = 0.0f64;
    for (k, s) in samples.iter().enumerate().step_by(25) {
        let row = record_rows
            .iter()
            .find(|r| r.0 == s.t)
            .ok_or_else(|| format!("frame {k}: record t {} not found", s.t))?;
        for (meas, exp) in [(s.px, row.1), (s.py, row.2), (s.pz, row.3)] {
            worst_pos = worst_pos.max((meas - exp).abs());
        }
    }
    if worst_pos > 1e-6 {
        return Err(format!("position mismatch worst {worst_pos:.2e}"));
    }
    for s in &samples {
        if s.bm < 0.0 {
            return Err("blank/unreadable frame (bm sentinel -1)".into());
        }
        if s.bv <= 0.0 {
            return Err(format!("zero brightness variance at frame t {}", s.t));
        }
    }
    let mean = |f: &dyn Fn(&Sample) -> f64| samples.iter().map(f).sum::<f64>() / samples.len() as f64;
    let mean_bv = mean(&|s| s.bv);
    let mean_bm = mean(&|s| s.bm);
    let mean_veg = mean(&|s| s.veg);
    let min_veg = samples.iter().map(|s| s.veg).fold(f64::INFINITY, f64::min);
    let mean_mm = mean(&|s| s.mm);
    let min_mm = samples.iter().map(|s| s.mm).fold(f64::INFINITY, f64::min);
    let mean_sky = mean(&|s| s.sky);
    if mean_bv < 1e-4 {
        return Err(format!("mean brightness variance {mean_bv:.2e} too low"));
    }
    if !(0.02..0.98).contains(&mean_bm) {
        return Err(format!("mean brightness {mean_bm:.3} outside (0.02, 0.98)"));
    }
    // Empty scene (loader failure): fog + sky only -> no green at all.
    if mean_veg < 0.20 {
        return Err(format!("mean vegetation fraction {mean_veg:.3} < 0.20 (scene empty?)"));
    }
    if min_veg < 0.10 {
        return Err(format!("min vegetation fraction {min_veg:.3} < 0.10"));
    }
    // Man-made content: buildings/roads fill part of every frame. The
    // measured mins/means on the fixture pack were ~0.44/~0.52 under the
    // flat-look lighting (dark sky); the S1 lit-sky look (blue sky fills
    // ~34% of every frame, measured 2026-09-29) moves them to
    // ~0.14/~0.18, so the mean bound follows the look down: purpose
    // unchanged (man-made geometry visible in every frame), min bound
    // stays the operative per-frame floor.
    // S2 facade/roof shaders (dark-neutral windows, storey bands, per-building
    // roof shade) move mean_mm by <1% and min_mm not at all below the S1
    // floor: the S1 bound set holds without re-baselining (2026-09-29).
    // S3 CC0 ground textures + S3b lane paint: measured set identical to S2
    // (mean_bm 0.647, veg 0.412, mm 0.424, sky 0.1635) — no re-baseline.
    // S4 foliage/bark/fence shaders + crown sway: measured set identical to S3
    // (mean_bm 0.647, veg 0.412, mm 0.424, sky 0.1635) — no re-baseline.
    // M1 glo30 pack (terrain draped, relief -20.5..+74.2 m): measured set
    // near-identical — mean_bm 0.649, veg 0.426 (min 0.376), mm 0.421
    // (min 0.366), sky 0.1529 (hills eat ~1 pt of horizon sky) — no
    // re-baseline.
    if mean_mm < 0.12 {
        return Err(format!("mean man-made fraction {mean_mm:.3} < 0.12"));
    }
    if min_mm < 0.10 {
        return Err(format!("min man-made fraction {min_mm:.3} < 0.10"));
    }
    if mean_sky < 0.005 {
        return Err(format!("mean sky fraction {mean_sky:.4} < 0.005"));
    }
    let mut dus: Vec<u64> = samples.iter().map(|s| s.du).collect();
    dus.sort_unstable();
    let mean_du = dus.iter().sum::<u64>() / dus.len() as u64;
    let p95 = dus[(dus.len() as f64 * 0.95) as usize];
    // Measurement only (no gate): the M1 relief tri growth is read off this.
    let mean_pr = samples.iter().map(|s| s.pr).sum::<u64>() / samples.len() as u64;
    Ok(format!(
        "frames {frames}, mean brightness {mean_bm:.3}, variance {mean_bv:.4}, veg {mean_veg:.3} (min {min_veg:.3}), man-made {mean_mm:.3} (min {min_mm:.3}), sky {mean_sky:.4}, worst pos err {worst_pos:.2e}, frame dt mean {mean_du} us / p95 {p95} us, pr mean {mean_pr}"
    ))
}

// ---- pack.json readers (same minimal style as tests/area_pack.rs) ----

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

fn num_in(text: &str, key: &str) -> u64 {
    let pat = format!("\"{key}\":");
    let i = text
        .find(&pat)
        .unwrap_or_else(|| panic!("field {key} missing"));
    let rest = text[i + pat.len()..].trim_start();
    let end = rest.find([',', '}']).unwrap_or(rest.len());
    rest[..end].trim().parse().expect("u64 parse")
}

/// Same, but a signed/float field (pack.json elevation's z_min/z_max).
fn fnum_in(text: &str, key: &str) -> f64 {
    let pat = format!("\"{key}\":");
    let i = text
        .find(&pat)
        .unwrap_or_else(|| panic!("field {key} missing"));
    let rest = text[i + pat.len()..].trim_start();
    let end = rest.find([',', '}']).unwrap_or(rest.len());
    rest[..end].trim().parse().expect("f64 parse")
}

/// Build the area pack from the cached fixture (offline): GLO-30 relief via
/// the committed fixture DEM (the flagship glo30 path; the flat contract
/// stays covered by tests/area_pack.rs).
fn build_pack(dir: &Path) {
    let out = Command::new("python3")
        .args([
            "tools/area_pack.py",
            "--osm",
            FIXTURE,
            "--lat",
            &HOME_LAT.to_string(),
            "--lon",
            &HOME_LON.to_string(),
            "--elevation",
            "glo30",
            "--dem-file",
            "tests/fixtures/dem_home_area.tif",
            "--seed",
            &SEED.to_string(),
            "--out",
        ])
        .arg(dir)
        .output()
        .expect("spawn area_pack.py");
    assert!(
        out.status.success(),
        "area_pack build failed: {}{}",
        String::from_utf8_lossy(&out.stdout),
        String::from_utf8_lossy(&out.stderr)
    );
}

/// The loader's group counts must equal pack.json's counts, family by family
/// (the consumer-side contract check the GDScript also enforces at runtime).
fn assert_pack_counts(pack: &PackSummary, pack_json: &Path) {
    let text = std::fs::read_to_string(pack_json).expect("pack.json");
    let counts = json_object(text.trim(), "counts");
    let want: Vec<(String, u64)> = [
        ("ground", 1),
        ("grass", num_in(counts, "grass")),
        ("road", num_in(counts, "roads")),
        ("bld", num_in(counts, "buildings")),
        ("bldroof", num_in(counts, "buildings")),
        ("tree", num_in(counts, "trees")),
        ("treec", num_in(counts, "trees")),
        ("dash", num_in(counts, "dashes")),
    ]
    .iter()
    .map(|(k, v)| (k.to_string(), *v))
    .collect();
    for (fam, n) in &want {
        let got = pack.groups.iter().find(|(k, _)| k == fam).unwrap_or_else(|| panic!("family {fam} missing from pack summary"));
        assert_eq!(got.1, *n, "family {fam}: loader saw {} vs pack.json {}", got.1, n);
    }
    let strips = num_in(counts, "strips");
    let hedge = pack.groups.iter().find(|(k, _)| k == "hedge").map(|g| g.1).unwrap_or(0);
    let fence = pack.groups.iter().find(|(k, _)| k == "fence").map(|g| g.1).unwrap_or(0);
    assert_eq!(hedge + fence, strips, "hedge+fence vs strips");
    assert!(pack.load_ms < 60_000, "pack load took {} ms", pack.load_ms);
    assert!(pack.materials >= 15, "only {} surfaces built", pack.materials);
}

/// One full xvfb Godot Movie Maker run of the pack replay scene.
fn run_godot(godot: &Path, project: &Path, dir: &Path, record: &Path, pack_dir: &Path) -> Result<String, String> {
    let mut cmd = Command::new("xvfb-run");
    cmd.arg("-a").arg(godot);
    cmd.arg("--path").arg(project);
    cmd.arg("--rendering-method").arg("gl_compatibility");
    cmd.arg("--rendering-driver").arg("opengl3");
    let out_json = dir.join("replay.json");
    std::fs::create_dir_all(dir).expect("create run dir");
    cmd.arg("--write-movie")
        .arg(dir.join("movie.png"))
        .arg("--fixed-fps")
        .arg(format!("{MOVIE_FPS}"))
        .arg("--quit-after")
        .arg(format!("{}", EXPECTED_FRAMES + 10))
        // Positional scene override (the project main scene is T5's
        // replay.tscn — running it here would produce T5-format samples).
        .arg("res://pack_replay.tscn")
        .env("DARTER_RECORD", record)
        .env("DARTER_PACK", pack_dir)
        .env("DARTER_REPLAY_OUT", &out_json);
    let out = cmd.output().expect("spawn godot");
    let log = format!(
        "stdout: {}\nstderr: {}",
        String::from_utf8_lossy(&out.stdout),
        String::from_utf8_lossy(&out.stderr)
    );
    if !out.status.success() {
        return Err(format!("exit {:?}\n{log}", out.status.code()));
    }
    if !out_json.exists() {
        return Err(format!("no replay JSON written\n{log}"));
    }
    let record_rows = read_record(record);
    validate(&out_json, &record_rows)
        .map(|summary| {
            let pngs = std::fs::read_dir(dir)
                .unwrap()
                .filter_map(|e| e.ok())
                .filter(|e| e.path().extension().is_some_and(|x| x == "png"))
                .count();
            format!("{summary}; {pngs} PNG frames")
        })
        .map_err(|e| format!("{e}\n{log}"))
}

/// The T7 integration: pack build -> record -> render -> gates -> counts ->
/// determinism.
#[test]
#[ignore = "needs tools/godot/bin/godot (pinned 4.7.2-stable) + xvfb; run: cargo test --test godot_pack -- --ignored"]
fn godot_pack_replay() {
    println!("godot pinned version {GODOT_VERSION}, sha256 {GODOT_SHA256}");
    let godot = locate_godot();
    let dir = temp_dir("run");
    let project = PathBuf::from("tools/godot_smoke");

    let pack_dir = dir.join("pack");
    build_pack(&pack_dir);
    assert!(pack_dir.join("pack.json").exists() && pack_dir.join("scene.obj").exists());
    assert!(pack_dir.join("terrain.bin").exists(), "glo30 pack must carry terrain.bin");

    // The flight: a scripted level transit west into the dense band — hover
    // throttle +0.002 (pre-compensating the T1 spool-up/own-wash dip, a
    // documented T1 behaviour: no hover equilibrium from standstill at
    // altitude) plus an initial 14 m/s eastward->west velocity that drag
    // bleeds off (the honest core-mode physics of a level transit; no
    // attitude controller exists outside closed mode).
    let rec_dir = dir.join("rec");
    let out = Command::new(env!("CARGO_BIN_EXE_sim_run"))
        .args([
            "--seed", "7",
            "--duration", "20",
            "--alt", "35",
            "--throttle", "0.157",
            "--vx", "-14",
            "--determinism-check",
            // The record is no longer pack-independent (M1): it flies the
            // pack's own terrain (spawn alt + h(spawn), ground contact at
            // h) — the rendered hills and the simulated hills are one grid.
            "--terrain",
        ])
        .arg(pack_dir.join("terrain.bin"))
        .arg("--out")
        .arg(&rec_dir)
        .output()
        .expect("spawn sim_run");
    assert!(out.status.success(), "sim_run failed: {}", String::from_utf8_lossy(&out.stderr));
    let record = rec_dir.join("flight.jsonl");
    let record_rows = read_record(&record);
    assert!(record_rows.len() >= 5000, "record too short: {}", record_rows.len());

    let first = run_godot(&godot, &project, &dir.join("a"), &record, &pack_dir);
    assert!(first.is_ok(), "pack replay failed: {:?}", first.err());
    println!("pack replay: {}", first.unwrap());

    let (frames, samples, pack) = parse_replay(&dir.join("a").join("replay.json"));
    println!(
        "loader: {} ms, {} surfaces, groups {:?}",
        pack.load_ms, pack.materials, pack.groups
    );
    assert_pack_counts(&pack, &pack_dir.join("pack.json"));

    // Relief echo (M1): the loader's summary must equal pack.json's
    // elevation block — the hills the camera sees are the contact hills.
    let pack_json_text = std::fs::read_to_string(pack_dir.join("pack.json")).expect("pack.json");
    let elev = json_object(pack_json_text.trim(), "elevation");
    assert_eq!(
        pack.relief,
        Some((fnum_in(elev, "z_min"), fnum_in(elev, "z_max"))),
        "relief echo != pack.json elevation range"
    );

    // Determinism: second full run, identical deterministic projection
    // (positions, brightness, class fractions).
    let again = run_godot(&godot, &project, &dir.join("b"), &record, &pack_dir);
    assert!(again.is_ok(), "second run failed: {:?}", again.err());
    let (b_frames, b_samples, _b_pack) = parse_replay(&dir.join("b").join("replay.json"));
    assert_eq!(frames, b_frames, "frame count differs between runs");
    assert_eq!(canon(&samples), canon(&b_samples), "second replay run diverged");
    println!(
        "second run: deterministic projection identical ({} samples), pack reload {} ms",
        b_samples.len(),
        parse_replay(&dir.join("b").join("replay.json")).2.load_ms
    );
    let _ = std::fs::remove_dir_all(&dir);
}