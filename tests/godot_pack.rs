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
//! Overpass fixture for the pack build. T8 `demo_areas` (same opt-in) gates
//! the curated demo-areas rung: index + tracks + envelope + the DARTER_AREA
//! device path + the picker; it stays red through slice S5's landings. The render path is xvfb +
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

/// The committed demo track (corridor_slalom): gate sites measured on this
/// same T7-mirror corridor flight (on-line pz at x -15/-40/-70/-100/-130/
/// -160, py == 0, speed 14 -> 6.06 m/s).
const DEMO_TRACK: &str = "tools/godot_smoke/track/demo_track.json";

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

/// Per-area render-content floors: the pixel-class gates that are scene
/// dependent (the rest of validate_with's gates are scene-agnostic).
#[derive(Clone, Copy, Debug)]
struct PixelFloors {
    mean_veg: f64,
    min_veg: f64,
    mean_mm: f64,
    min_mm: f64,
}

/// T7's corridor floors, exactly the pre-refactor literals (2026-10-03 canon:
/// veg mean 0.417 / min 0.339, mm mean 0.422 / min 0.335 run far above them).
const CORRIDOR_FLOORS: PixelFloors = PixelFloors {
    mean_veg: 0.20,
    min_veg: 0.10,
    mean_mm: 0.12,
    min_mm: 0.10,
};

/// T7's wrapper: signature and corridor numbers unchanged.
fn validate(replay: &Path, record_rows: &[(f64, f64, f64, f64)]) -> Result<String, String> {
    validate_with(replay, record_rows, CORRIDOR_FLOORS)
}

/// The parameterized core (T8's demo areas fly different scenes; each area
/// carries its own measured floors in the AREAS table, pinned at S5).
fn validate_with(
    replay: &Path,
    record_rows: &[(f64, f64, f64, f64)],
    floors: PixelFloors,
) -> Result<String, String> {
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
    if mean_veg < floors.mean_veg {
        return Err(format!(
            "mean vegetation fraction {mean_veg:.3} < {} (scene empty?)",
            floors.mean_veg
        ));
    }
    if min_veg < floors.min_veg {
        return Err(format!("min vegetation fraction {min_veg:.3} < {}", floors.min_veg));
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
    // M1 track rung (gate rings + HUD over the corridor): mean_bm 0.634,
    // veg 0.417 (min 0.339), mm 0.422 (min 0.335), sky 0.1612 — every
    // floor holds with wide margins, no re-baseline (2026-10-03).
    if mean_mm < floors.mean_mm {
        return Err(format!("mean man-made fraction {mean_mm:.3} < {}", floors.mean_mm));
    }
    if min_mm < floors.min_mm {
        return Err(format!("min man-made fraction {min_mm:.3} < {}", floors.min_mm));
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

/// Build an area pack from a fixture pair (offline): GLO-30 relief via the
/// committed fixture DEM (the flagship glo30 path; the flat contract stays
/// covered by tests/area_pack.rs).
fn build_pack_for(dir: &Path, osm: &str, lat: f64, lon: f64, dem: &str, seed: u64) {
    let out = Command::new("python3")
        .args([
            "tools/area_pack.py",
            "--osm",
            osm,
            "--lat",
            &lat.to_string(),
            "--lon",
            &lon.to_string(),
            "--elevation",
            "glo30",
            "--dem-file",
            dem,
            "--seed",
            &seed.to_string(),
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

/// The T7 corridor home pack.
fn build_pack(dir: &Path) {
    build_pack_for(
        dir,
        FIXTURE,
        HOME_LAT,
        HOME_LON,
        "tests/fixtures/dem_home_area.tif",
        SEED,
    )
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
    // water: asserted only when the pack carries water — water-free packs
    // have no counts.water key at all (num_in panics on a missing field).
    if counts.contains("\"water\":") {
        let n = num_in(counts, "water");
        let got = pack.groups.iter().find(|(k, _)| k == "water").map(|g| g.1).unwrap_or(0);
        assert_eq!(got, n, "family water: loader saw {got} vs pack.json {n}");
    }
    let strips = num_in(counts, "strips");
    let hedge = pack.groups.iter().find(|(k, _)| k == "hedge").map(|g| g.1).unwrap_or(0);
    let fence = pack.groups.iter().find(|(k, _)| k == "fence").map(|g| g.1).unwrap_or(0);
    assert_eq!(hedge + fence, strips, "hedge+fence vs strips");
    assert!(pack.load_ms < 60_000, "pack load took {} ms", pack.load_ms);
    assert!(pack.materials >= 15, "only {} surfaces built", pack.materials);
}

/// One full xvfb Godot Movie Maker run of the pack replay scene.
fn run_godot(
    godot: &Path,
    project: &Path,
    dir: &Path,
    record: &Path,
    pack_dir: &Path,
    track: Option<&Path>,
) -> Result<String, String> {
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
    if let Some(track) = track {
        // Godot's FileAccess resolves a relative path inside the project
        // dir, so the env value must be absolute (tests run from the
        // package root, where DEMO_TRACK is what the sim_run CLI wants).
        cmd.env(
            "DARTER_TRACK",
            std::fs::canonicalize(track).expect("demo track path"),
        );
    }
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

/// The device-path run (T8 gate d): the loader gets ONLY the area name and
/// the replay-out path; pack, record and track must come from
/// `res://areas/<name>/` on the selection ladder — the same resolution the
/// APK export will use. The DARTER_PACK path stays untouched and T7-covered.
fn run_godot_area(godot: &Path, project: &Path, dir: &Path, area: &str) -> Result<(), String> {
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
        .arg("res://pack_replay.tscn")
        .env("DARTER_AREA", area)
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
    Ok(())
}

/// The replay pack block's "area" member, if the loader emitted one. S5 sets
/// it ONLY on DARTER_AREA runs; the DARTER_PACK path must emit the pack
/// block byte-identical to the T7 canon (no area member there).
fn parse_area_field(replay: &Path) -> Option<String> {
    let text = std::fs::read_to_string(replay).ok()?;
    let i = text.find("\"pack\":{")?;
    let j = text[i..].find("\"samples\":[").map(|k| i + k).unwrap_or(text.len());
    let header = &text[i..j];
    let k = header.find("\"area\":\"")?;
    Some(header[k + "\"area\":\"".len()..].split('"').next()?.to_string())
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
        .arg("--track")
        .arg(DEMO_TRACK)
        .arg("--out")
        .arg(&rec_dir)
        .output()
        .expect("spawn sim_run");
    assert!(out.status.success(), "sim_run failed: {}", String::from_utf8_lossy(&out.stderr));
    let record = rec_dir.join("flight.jsonl");
    let record_rows = read_record(&record);
    assert!(record_rows.len() >= 5000, "record too short: {}", record_rows.len());

    let first = run_godot(&godot, &project, &dir.join("a"), &record, &pack_dir, Some(Path::new(DEMO_TRACK)));
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

    // Track rung (M1): the corridor record threads all six demo gates via
    // the CLI's post-hoc evaluation; the replay's renderer-side mirror must
    // equal the CLI events (chronological gate order; |dt| <= 0.01 s — the
    // mirror recomputes at display cadence from the same f64 formulas — the
    // slack covers general-case row-pair divergence on curved approaches,
    // documented in track.rs, not exercised by this straight-line track).
    let summary_text = std::fs::read_to_string(rec_dir.join("summary.json")).expect("summary");
    let summary: serde_json::Value =
        serde_json::from_str(&summary_text).expect("summary.json parses");
    let trk = summary.get("track").expect("summary carries the track block");
    assert_eq!(trk["checkpoints"], serde_json::json!(6), "demo gate count");
    assert_eq!(trk["loop"], serde_json::json!(false), "demo is open");
    let cli_events = trk["events"].as_array().expect("CLI events array");
    assert_eq!(cli_events.len(), 6, "all six gates threaded: {cli_events:?}");
    println!(
        "cli track events: {:?}",
        cli_events
            .iter()
            .map(|e| (e[0].as_u64().unwrap(), e[1].as_f64().unwrap()))
            .collect::<Vec<_>>()
    );
    for (k, e) in cli_events.iter().enumerate() {
        assert_eq!(
            e[0].as_u64().expect("event index"),
            k as u64,
            "chronological gate order"
        );
        assert!(
            k == 0
                || e[1].as_f64().unwrap() > cli_events[k - 1][1].as_f64().unwrap(),
            "event times strictly increasing"
        );
    }
    let rep_a = parse_track_block(&dir.join("a").join("replay.json"));
    assert_track_events_match(&rep_a, cli_events, "run-a");

    // Determinism: second full run, identical deterministic projection
    // (positions, brightness, class fractions).
    let again = run_godot(&godot, &project, &dir.join("b"), &record, &pack_dir, Some(Path::new(DEMO_TRACK)));
    assert!(again.is_ok(), "second run failed: {:?}", again.err());
    let (b_frames, b_samples, _b_pack) = parse_replay(&dir.join("b").join("replay.json"));
    assert_eq!(frames, b_frames, "frame count differs between runs");
    assert_eq!(canon(&samples), canon(&b_samples), "second replay run diverged");
    println!(
        "second run: deterministic projection identical ({} samples), pack reload {} ms",
        b_samples.len(),
        parse_replay(&dir.join("b").join("replay.json")).2.load_ms
    );
    let rep_b = parse_track_block(&dir.join("b").join("replay.json"));
    assert_track_events_match(&rep_b, cli_events, "run-b");
    let _ = std::fs::remove_dir_all(&dir);
}

/// The replay/summary track block: the replay JSON is machine-authored
/// fixed-order output, so the whole file parses and the block is one member.
fn parse_track_block(path: &Path) -> serde_json::Value {
    let text = std::fs::read_to_string(path).expect("replay JSON");
    let value: serde_json::Value =
        serde_json::from_str(&text).expect("whole replay/summary JSON parses");
    value
        .get("track")
        .expect("track block present").clone()
}

/// The renderer-side mirror must equal the CLI events: identical indices
/// (chronological gate order), |dt| <= 0.01 s per event.
fn assert_track_events_match(
    block: &serde_json::Value,
    cli_events: &[serde_json::Value],
    what: &str,
) {
    assert_eq!(block["schema"], "darter_track", "{what}");
    let evs = block["events"].as_array().unwrap_or_else(|| panic!("{what}: track events array"));
    assert_eq!(evs.len(), cli_events.len(), "{what}: event count vs CLI");
    for (a, b) in evs.iter().zip(cli_events) {
        assert_eq!(a[0], b[0], "{what}: event index order");
        let dt = (a[1].as_f64().unwrap() - b[1].as_f64().unwrap()).abs();
        assert!(dt <= 0.01, "{what}: mirror vs CLI dt {dt} > 0.01");
    }
}

/// parse_track_block pinned standalone (runs in the plain suite; the T7
/// comparisons above rely on this parser).
#[test]
fn parse_track_block_parses_events() {
    let dir = temp_dir("parse_track");
    let path = dir.join("replay.json");
    std::fs::write(
        &path,
        r#"{"movie_fps":30,"frames":2,"pack":{"load_ms":1},"track":{"schema":"darter_track","version":1,"name":"t","checkpoints":2,"loop":false,"events":[[0,1.5],[1,2.75]],"lap_splits":null},"samples":[]}"#,
    )
    .unwrap();
    let trk = parse_track_block(&path);
    assert_eq!(trk["name"], "t");
    assert_eq!(trk["checkpoints"], serde_json::json!(2));
    assert_eq!(trk["loop"], serde_json::json!(false));
    assert!(trk["lap_splits"].is_null());
    let events = trk["events"].as_array().expect("events");
    assert_eq!(events.len(), 2);
    assert_eq!(events[0][0], serde_json::json!(0));
    assert_eq!(events[1][1], serde_json::json!(2.75));
    let _ = std::fs::remove_dir_all(&dir);
}

// ---- T8: the curated demo-areas rung (city, suburb, coast, hills) ----

/// The attribution string VISION.md requires to be visible, verbatim: the
/// picker footer and index.json's field both carry exactly this.
const ATTRIBUTION: &str =
    "© OpenStreetMap contributors, ODbL 1.0 | © Copernicus DEM / ESA (GLO-30)";

/// The four curated areas, in index.json order (S3 authors index.json in
/// this order; T8's picker assertion pins it). Sites/seeds: the approved
/// plan's table, with S1's two dated origin nudges folded in (coast Calshot
/// Spit -> Stone shore 50.8130/-1.3070; hills Butser -> 50.9761/-0.9457 so
/// the 4530 m grid stays inside tile N50_00_W001_00 — both committed with
/// the fixture pins in tests/area_pack.rs).
struct AreaSpec {
    name: &'static str,
    osm: &'static str,
    dem: &'static str,
    lat: f64,
    lon: f64,
    /// area_pack build seed.
    seed: u64,
    /// The record: per-area sim_run seed, start altitude, straight-line vx.
    rec_seed: u64,
    alt: u32,
    vx: f64,
    /// The track file the record flies (suburb reuses the committed corridor
    /// track; the other three land at S3).
    track: &'static str,
    /// Pixel floors: the measured DARTER_AREA canon per area (the three new
    /// areas pin from their first full pass; suburb keeps the corridor
    /// constants it is byte-identical to). All are dated measured facts.
    floors: PixelFloors,
}

/// Unpinned-floor sentinel was Infinity until S5; per-area floors below are
/// pinned from the first full DARTER_AREA pass (deterministic projection,
/// gl_compatibility desktop canon, sample_every=1) with the corridor's
/// margin convention: every floor at ~half the measured value — the
/// measured pass is exact, so these doors sit well below the canon.
const CITY_FLOORS: PixelFloors = PixelFloors {
    mean_veg: 0.19, // measured 0.394 (2026-10-03)
    min_veg: 0.09,  // measured min 0.184
    mean_mm: 0.18,  // measured 0.364
    min_mm: 0.14,   // measured min 0.295
};

// Post-water re-pin (2026-10-03): the sea paints as dark water and its
// blue-dominant pixels land in the sky pixel class (the classifier has no
// water class), so the vegetation signal drops to the land strip only:
// measured veg 0.036 (min 0.0000 — over-open-sea frames carry no vegetation
// at all), mm 0.321 (min 0.272), sky 0.644. min_veg is 0.0 because the
// measured min IS zero; the empty-scene door is mean_veg.
const COAST_FLOORS: PixelFloors = PixelFloors {
    mean_veg: 0.018, // measured 0.036 (2026-10-03, post-water)
    min_veg: 0.0,    // measured 0.0000 (pure sea+sky frames exist)
    mean_mm: 0.16,   // measured 0.321
    min_mm: 0.135,   // measured 0.272
};

const HILLS_FLOORS: PixelFloors = PixelFloors {
    mean_veg: 0.27, // measured 0.546 (2026-10-03)
    min_veg: 0.23,  // measured min 0.473
    mean_mm: 0.18,  // measured 0.379
    min_mm: 0.15,   // measured min 0.312
};

const AREAS: &[AreaSpec] = &[
    AreaSpec {
        name: "city",
        osm: "tests/fixtures/osm_city.json",
        dem: "tests/fixtures/dem_city.tif",
        lat: 50.9060,
        lon: -1.4012,
        seed: 11,
        rec_seed: 21,
        alt: 30,
        vx: -14.0,
        track: "tools/godot_smoke/track/city.json",
        floors: CITY_FLOORS,
    },
    AreaSpec {
        name: "suburb",
        osm: FIXTURE,
        dem: "tests/fixtures/dem_home_area.tif",
        lat: HOME_LAT,
        lon: HOME_LON,
        seed: SEED,
        rec_seed: 7,
        alt: 35,
        vx: -14.0,
        track: DEMO_TRACK,
        floors: CORRIDOR_FLOORS,
    },
    AreaSpec {
        name: "coast",
        osm: "tests/fixtures/osm_coast.json",
        dem: "tests/fixtures/dem_coast.tif",
        lat: 50.8130,
        lon: -1.3070,
        seed: 13,
        rec_seed: 23,
        alt: 30,
        vx: -14.0,
        track: "tools/godot_smoke/track/coast.json",
        floors: COAST_FLOORS,
    },
    AreaSpec {
        name: "hills",
        osm: "tests/fixtures/osm_hills.json",
        dem: "tests/fixtures/dem_hills.tif",
        lat: 50.9761,
        lon: -0.9457,
        seed: 17,
        rec_seed: 29,
        alt: 60,
        vx: -14.0,
        track: "tools/godot_smoke/track/hills.json",
        floors: HILLS_FLOORS,
    },
];

/// (e) The bundle payload must be byte-identical to the offline regen for
/// every staged file. The suburb row's equality doubles as the proof that
/// T7's pack build == the bundled suburb payload. T8's own regen lives in
/// two dirs: the pack build (pack/scene) and the record build (flight.jsonl)
/// — make_bundle.sh builds both into one scratch dir, so the paths differ
/// but the comparison is the same.
fn assert_bundle_matches_regen(area: &str, pack_dir: &Path, rec_dir: &Path) {
    let bundle = PathBuf::from(format!("tools/godot_smoke/areas/{area}"));
    let track = AREAS
        .iter()
        .find(|x| x.name == area)
        .expect("area in table")
        .track;
    let pairs = [
        (bundle.join("pack.json"), pack_dir.join("pack.json")),
        (bundle.join("scene.packobj"), pack_dir.join("scene.obj")),
        (bundle.join("record.jsonl"), rec_dir.join("flight.jsonl")),
        (bundle.join("track.json"), PathBuf::from(track)),
    ];
    for (bundle_path, regen_path) in pairs {
        let b = std::fs::read(&bundle_path)
            .unwrap_or_else(|e| panic!("{} unreadable: {e} (bundle stale or missing; run make_bundle.sh)", bundle_path.display()));
        let r = std::fs::read(&regen_path).expect("regen payload");
        assert_eq!(b, r, "bundle vs regen drift: {}", bundle_path.display());
    }
}

/// The demo-areas run gate (Slices S2..S6; red at the first missing slice,
/// green by S6). Per area, in order: (a) offline regen pack build from the
/// committed fixtures + validator exit 0; (b) sim_run record with the
/// per-area track + determinism: 6 checkpoints, chronological events, all
/// crossings <= 19.0 s, >= 5000 rows; (c) envelope_check exit 0; (e) bundle-
/// vs-regen byte equality (currency proof before anything runs the bundle);
/// (d) the device path — the loader resolves pack/record/track from
/// `res://areas/<name>/` given ONLY DARTER_AREA, with counts, relief echo,
/// per-area pixel floors, the area member and the track mirror matching the
/// CLI. Then: (f) hills determinism re-run; (g) the picker prints
/// PICKER_READY with the four areas + the verbatim attribution; (h) a bare
/// run hard-errors naming DARTER_PACK, DARTER_AREA and the four areas (the
/// legacy res://pack/res://record fallbacks are gone).
#[test]
#[ignore = "needs tools/godot/bin/godot (pinned 4.7.2-stable) + xvfb; run: cargo test --test godot_pack -- --ignored"]
fn demo_areas() {
    let godot = locate_godot();
    let project = PathBuf::from("tools/godot_smoke");
    let dir = temp_dir("demo");

    // The index gate first (S3 authors it to the asserted shape): schema,
    // table order, track refs, and the attribution string the picker must
    // print verbatim.
    let index_path = project.join("areas/index.json");
    let index_text = std::fs::read_to_string(&index_path)
        .expect("tools/godot_smoke/areas/index.json missing (S3)");
    let index: serde_json::Value = serde_json::from_str(&index_text).expect("index parses");
    assert_eq!(index["schema"], "darter_areas", "index schema");
    assert_eq!(index["version"], 1, "index version");
    let idx_areas = index["areas"].as_array().expect("index areas array");
    assert_eq!(idx_areas.len(), AREAS.len(), "index area count");
    for (row, spec) in idx_areas.iter().zip(AREAS) {
        assert_eq!(row["name"].as_str().expect("area name"), spec.name, "index order");
        // index.json carries the bundle-relative name the loader reads;
        // spec.track stays the SOURCE track file the record flies from.
        assert_eq!(row["track"].as_str().expect("track ref"), "track.json", "track ref");
        assert!(PathBuf::from(spec.track).is_file(), "source track: {}", spec.name);
    }
    assert_eq!(index["attribution"].as_str().expect("attribution"), ATTRIBUTION);

    // Offline stage first (bundle currency proven for every area before the
    // first replay runs), then the device stage. regen_rows/cli_events carry
    // what the device stage needs from the offline stage.
    let mut regen_rows: Vec<Vec<(f64, f64, f64, f64)>> = Vec::new();
    let mut cli_events: Vec<Vec<serde_json::Value>> = Vec::new();

    for spec in AREAS {
        let regen = dir.join(spec.name);

        // (a) offline regen + validator exit 0.
        build_pack_for(&regen, spec.osm, spec.lat, spec.lon, spec.dem, spec.seed);
        let v = Command::new("python3")
            .args(["tools/area_pack.py"])
            .arg(&regen)
            .output()
            .expect("spawn validator");
        assert!(
            v.status.success(),
            "{}: regen validator failed: {}{}",
            spec.name,
            String::from_utf8_lossy(&v.stdout),
            String::from_utf8_lossy(&v.stderr)
        );

        // (b) record on the regen terrain, flying the area's track.
        let rec_dir = dir.join(format!("{}-rec", spec.name));
        let out = Command::new(env!("CARGO_BIN_EXE_sim_run"))
            .args([
                "--seed",
                &spec.rec_seed.to_string(),
                "--duration",
                "20",
                "--alt",
                &spec.alt.to_string(),
                "--throttle",
                "0.157",
                "--vx",
                &spec.vx.to_string(),
                "--determinism-check",
                "--terrain",
            ])
            .arg(regen.join("terrain.bin"))
            .arg("--track")
            .arg(spec.track)
            .arg("--out")
            .arg(&rec_dir)
            .output()
            .expect("spawn sim_run");
        assert!(
            out.status.success(),
            "{}: sim_run failed: {}",
            spec.name,
            String::from_utf8_lossy(&out.stderr)
        );
        let record = rec_dir.join("flight.jsonl");
        let rows = read_record(&record);
        assert!(rows.len() >= 5000, "{}: record too short {}", spec.name, rows.len());
        let summary: serde_json::Value = serde_json::from_str(
            &std::fs::read_to_string(rec_dir.join("summary.json")).expect("summary"),
        )
        .expect("summary.json parses");
        let trk = summary.get("track").expect("summary carries the track block");
        assert_eq!(trk["checkpoints"], serde_json::json!(6), "{}: gate count", spec.name);
        assert_eq!(trk["loop"], serde_json::json!(false), "{}: demo is open", spec.name);
        let evs = trk["events"].as_array().expect("CLI events array");
        assert_eq!(evs.len(), 6, "{}: all gates threaded: {evs:?}", spec.name);
        let mut last_t = 0.0f64;
        for (k, e) in evs.iter().enumerate() {
            assert_eq!(
                e[0].as_u64().expect("event index"),
                k as u64,
                "{}: chronological gate order",
                spec.name
            );
            let t = e[1].as_f64().expect("event time");
            assert!(t > last_t, "{}: event times strictly increasing", spec.name);
            assert!(t <= 19.0, "{}: crossing {t} s > 19.0", spec.name);
            last_t = t;
        }
        println!("{}: cli track events {evs:?}", spec.name);

        // (c) envelope: clearance pz - h_at >= 8 m on every row of the flown
        // window; no row inside a generator building ring's bbox under the
        // top cap.
        let c = Command::new("python3")
            .args(["tools/envelope_check.py"])
            .arg(&regen)
            .arg("--record")
            .arg(&record)
            .output()
            .expect("spawn envelope_check.py");
        assert!(
            c.status.success(),
            "{}: envelope_check failed: {}{}",
            spec.name,
            String::from_utf8_lossy(&c.stdout),
            String::from_utf8_lossy(&c.stderr)
        );

        // (e) bundle currency before anything runs it.
        assert_bundle_matches_regen(spec.name, &regen, &rec_dir);

        // The device stage reads these back: rows gate the replay mirror,
        // CLI events gate the replay's track block.
        regen_rows.push(rows);
        cli_events.push(evs.clone());
        println!("{}: offline gates green (regen, validator, record, envelope, bundle)", spec.name);
    }

    // (d) the device path: DARTER_AREA only. Bundle == regen (e), so the
    // gates read the regen record rows.
    for (k, spec) in AREAS.iter().enumerate() {
        let rows = regen_rows[k].clone();
        let evs = cli_events[k].clone();
        let run_dir = dir.join(format!("{}-run", spec.name));
        let run = run_godot_area(&godot, &project, &run_dir, spec.name);
        assert!(run.is_ok(), "{}: DARTER_AREA replay failed: {:?}", spec.name, run.err());
        let replay = run_dir.join("replay.json");
        let (frames, _samples, pack) = parse_replay(&replay);
        assert_eq!(frames, EXPECTED_FRAMES, "{}: frame count", spec.name);
        let bjson_path =
            PathBuf::from(format!("tools/godot_smoke/areas/{}/pack.json", spec.name));
        assert_pack_counts(&pack, &bjson_path);
        let bjson = std::fs::read_to_string(&bjson_path).expect("area pack.json");
        let elev = json_object(bjson.trim(), "elevation");
        assert_eq!(
            pack.relief,
            Some((fnum_in(elev, "z_min"), fnum_in(elev, "z_max"))),
            "{}: relief echo != pack.json elevation range",
            spec.name
        );
        let summary = validate_with(&replay, &rows, spec.floors)
            .unwrap_or_else(|e| panic!("{}: DARTER_AREA gates: {e}", spec.name));
        assert_eq!(
            parse_area_field(&replay).as_deref(),
            Some(spec.name),
            "{}: replay pack block must carry the area member",
            spec.name
        );
        let rep = parse_track_block(&replay);
        assert_track_events_match(&rep, &evs, spec.name);
        println!("{} DARTER_AREA: {summary}", spec.name);

        // (f) hills determinism spot check: a second device-path run with
        // the identical deterministic projection.
        if spec.name == "hills" {
            let again_dir = dir.join("hills-run2");
            let again = run_godot_area(&godot, &project, &again_dir, "hills");
            assert!(again.is_ok(), "hills second run failed: {:?}", again.err());
            let (a_frames, a_samples, _) = parse_replay(&replay);
            let (b_frames, b_samples, _) = parse_replay(&again_dir.join("replay.json"));
            assert_eq!(a_frames, b_frames, "hills: frame count differs");
            assert_eq!(canon(&a_samples), canon(&b_samples), "hills second replay diverged");
        }
    }

    // (g) the picker: positional scene, no env; prints the area list with
    // the attribution visible verbatim.
    let pick_dir = dir.join("picker");
    std::fs::create_dir_all(&pick_dir).unwrap();
    let out = Command::new("xvfb-run")
        .arg("-a")
        .arg(&godot)
        .arg("--path")
        .arg(&project)
        .arg("--rendering-method")
        .arg("gl_compatibility")
        .arg("--rendering-driver")
        .arg("opengl3")
        .arg("--quit-after")
        .arg("5")
        .arg("res://area_picker.tscn")
        .output()
        .expect("spawn godot for picker");
    let text = format!(
        "{}{}",
        String::from_utf8_lossy(&out.stdout),
        String::from_utf8_lossy(&out.stderr)
    );
    assert!(
        out.status.success(),
        "picker exited early:\n{text}"
    );
    let want_line = format!(
        "PICKER_READY areas={}",
        AREAS.iter().map(|a| a.name).collect::<Vec<_>>().join(",")
    );
    assert!(text.contains(&want_line), "missing {want_line:?} in picker output:\n{text}");
    assert!(text.contains(ATTRIBUTION), "attribution not printed verbatim\n{text}");

    // (h) no-env hard error: pack_replay with no DARTER_* envs exits 1 and
    // names both resolution options and the four areas (the legacy
    // res://pack + res://record fallbacks are gone by S5).
    let bare_dir = dir.join("bare");
    std::fs::create_dir_all(&bare_dir).unwrap();
    let out = Command::new("xvfb-run")
        .arg("-a")
        .arg(&godot)
        .arg("--path")
        .arg(&project)
        .arg("res://pack_replay.tscn")
        .output()
        .expect("spawn godot bare");
    let text = format!(
        "{}{}",
        String::from_utf8_lossy(&out.stdout),
        String::from_utf8_lossy(&out.stderr)
    );
    assert_eq!(out.status.code(), Some(1), "bare pack_replay must exit 1\n{text}");
    for needle in ["DARTER_PACK", "DARTER_AREA", "city", "suburb", "coast", "hills"] {
        assert!(
            text.contains(needle),
            "bare-run error does not name {needle}:\n{text}"
        );
    }
    let _ = std::fs::remove_dir_all(&dir);
}