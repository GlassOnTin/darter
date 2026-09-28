//! T5 Godot 4 renderer smoke test: the renderer choice (M0 open question)
//! is verified by actually replaying a darter flight record through Godot 4
//! in Movie Maker mode and asserting on what comes back — sampled quad
//! positions match the record, per-frame brightness is non-blank (positive
//! variance), and a second run reproduces the deterministic projection (the
//! wall-clock frame delta carried alongside is informational, see canon).
//!
//! Opt-in (`cargo test --test godot -- --ignored`), like the live-SITL
//! tests: it needs the pinned Godot binary, which is not committed.
//! Provenance is enforced at runtime: the binary must hash to the pinned
//! SHA-256 below (official 4.7.2-stable, cross-checked against the release's
//! SHA512-SUMS.txt on download).
//!
//! Verification order from the plan: attempt 1 is `--headless --write-movie`
//! on the default Vulkan path; if that fails or produces blank frames (a
//! tracked upstream bug), attempt 2 is `xvfb-run ... --rendering-method
//! gl_compatibility` (no --headless — gl_compatibility does not render
//! headless). The test logs which path won. Frame-time deltas are recorded
//! in the replay JSON but are informational only (xvfb/Vulkan variance).
//!
//! Measured on this machine (4.7.2-stable): the headless path cannot work at
//! all — the headless display server only supports the dummy render device
//! (`--rendering-driver vulkan` does not override it), and reading the
//! viewport texture then faults in the dummy storage (SIGSEGV in
//! texture_2d_get). So attempt 2 (xvfb gl_compatibility) is the path that
//! wins here; the fallback ordering is kept in the test so a machine where
//! headless rendering works (if any) still prefers it.

use std::path::{Path, PathBuf};
use std::process::Command;

use darter_core::sha256::Sha256;

/// Official Godot 4.7.2-stable Linux x86_64, standard (non-.NET) build.
/// sha256 of the extracted binary; the zip matched the release's published
/// SHA512-SUMS.txt entry before extraction.
const GODOT_SHA256: &str = "8d106cbe6144c2dc7e881d61d2429c1a8a76e6b22ef48bd5e48dcf934953f71e";
const GODOT_VERSION: &str = "4.7.2.stable.official.ed1daf0bf";
const MOVIE_FPS: f64 = 30.0;
/// 10 s of record at the 30 fps Movie Maker rate.
const EXPECTED_FRAMES: usize = 300;

fn temp_dir(name: &str) -> PathBuf {
    let d = std::env::temp_dir().join(format!("darter-godot-{name}-{}", std::process::id()));
    std::fs::create_dir_all(&d).unwrap();
    d
}

/// Locate and hash-verify the pinned Godot binary: env DARTER_GODOT, else
/// tools/godot/bin/godot relative to the workspace root (cargo's cwd).
fn locate_godot() -> PathBuf {
    let path = match std::env::var("DARTER_GODOT") {
        Ok(p) => PathBuf::from(p),
        Err(_) => PathBuf::from("tools/godot/bin/godot"),
    };
    assert!(path.exists(), "Godot binary missing at {path:?}: run the download pinned in tests/godot.rs (Godot 4.7.2-stable linux.x86_64 from godotengine/godot GitHub releases), or set DARTER_GODOT");
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
    // The last field of the last sample has no terminator (parse_replay
    // trims the closing brace), so end-of-string also ends a value.
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

/// One replay sample, parsed from the hand-formatted JSON entries.
struct Sample {
    t: f64,
    px: f64,
    py: f64,
    pz: f64,
    bm: f64,
    bv: f64,
    du: u64,
}

/// The deterministic projection of the replay samples for the determinism
/// gate. The wall-clock frame delta `du` is parsed but excluded here: two
/// runs cannot agree on wall time by construction, and the plan scopes
/// frame-time as informational. This projection is the physics/graphics
/// determinism the plan's identical-JSON clause verifies.
fn canon(samples: &[Sample]) -> String {
    samples
        .iter()
        .map(|s| format!("{},{},{},{},{:.9},{:.9}", s.t, s.px, s.py, s.pz, s.bm, s.bv))
        .collect::<Vec<_>>()
        .join(";")
}
/// Parse the replay JSON's samples array (hand-formatted, fixed field order).
fn parse_replay(path: &Path) -> (usize, Vec<Sample>) {
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
            du: num(e, "du") as u64,
        });
    }
    let frames = num(&text[..i], "frames") as usize;
    (frames, samples)
}

/// The gates: frame count, positions vs the record, brightness non-blank.
/// Returns a human-readable summary for the log.
fn validate(replay: &Path, record_rows: &[(f64, f64, f64, f64)]) -> Result<String, String> {
    let (frames, samples) = parse_replay(replay);
    if frames != EXPECTED_FRAMES {
        return Err(format!("frames {frames} vs expected {EXPECTED_FRAMES}"));
    }
    if samples.len() != EXPECTED_FRAMES {
        return Err(format!("samples {} vs expected {EXPECTED_FRAMES}", samples.len()));
    }
    // Positions: raw record coordinates, same text both sides, so 1e-6.
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
    // Brightness: every frame non-blank (variance > 0, mean off the rails),
    // and the aggregate not a flat wall.
    let mean_bv: f64 = samples.iter().map(|s| s.bv).sum::<f64>() / samples.len() as f64;
    let mean_bm: f64 = samples.iter().map(|s| s.bm).sum::<f64>() / samples.len() as f64;
    for s in &samples {
        if s.bm < 0.0 {
            return Err("blank/unreadable frame (bm sentinel -1)".into());
        }
        if s.bv <= 0.0 {
            return Err(format!("zero brightness variance at frame t {}", s.t));
        }
    }
    if mean_bv < 1e-4 {
        return Err(format!("mean brightness variance {mean_bv:.2e} too low"));
    }
    if !(0.02..0.98).contains(&mean_bm) {
        return Err(format!("mean brightness {mean_bm:.3} outside (0.02, 0.98)"));
    }
    // Frame-time instrumentation: informational, printed for the log.
    let mut dus: Vec<u64> = samples.iter().map(|s| s.du).collect();
    dus.sort_unstable();
    let mean_du = dus.iter().sum::<u64>() / dus.len() as u64;
    let p95 = dus[(dus.len() as f64 * 0.95) as usize];
    Ok(format!(
        "frames {frames}, mean brightness {mean_bm:.3}, mean variance {mean_bv:.4}, worst pos err {worst_pos:.2e}, frame dt mean {mean_du} us / p95 {p95} us"
    ))
}

/// One full Godot Movie Maker run; fails when the binary exits non-zero or
/// produces no replay JSON.
fn run_godot(
    godot: &Path,
    project: &Path,
    dir: &Path,
    record: &Path,
    xvfb: bool,
) -> Result<String, String> {
    let mut cmd = if xvfb {
        let mut c = Command::new("xvfb-run");
        c.arg("-a").arg(godot);
        c
    } else {
        Command::new(godot)
    };
    cmd.arg("--path").arg(project);
    if !xvfb {
        // Headless: force the Vulkan render device (the headless display
        // server alone would fall back to the dummy renderer, which cannot
        // produce images).
        cmd.arg("--headless").arg("--rendering-driver").arg("vulkan");
    } else {
        cmd.arg("--rendering-method").arg("gl_compatibility");
        cmd.arg("--rendering-driver").arg("opengl3");
    }
    let out_json = dir.join("replay.json");
    std::fs::create_dir_all(dir).expect("create run dir");
    cmd.arg("--write-movie")
        .arg(dir.join("movie.png"))
        .arg("--fixed-fps")
        .arg(format!("{MOVIE_FPS}"))
        .arg("--quit-after")
        .arg(format!("{}", EXPECTED_FRAMES + 10))
        .env("DARTER_RECORD", record)
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

/// The smoke test: record -> replay -> gates -> determinism.
#[test]
#[ignore = "needs tools/godot/bin/godot (pinned 4.7.2-stable); run: cargo test --test godot -- --ignored"]
fn godot_replay_smoke() {
    println!("godot pinned version {GODOT_VERSION}, sha256 {GODOT_SHA256}");
    let godot = locate_godot();
    let dir = temp_dir("smoke");
    let project = PathBuf::from("tools/godot_smoke");

    // A real flight record: core-mode climb (throttle 0.55 from 20 m is a
    // fast ascent, so the view sweeps ground -> sky).
    let rec_dir = dir.join("rec");
    let out = Command::new(env!("CARGO_BIN_EXE_sim_run"))
        .args(["--seed", "5", "--duration", "10", "--throttle", "0.55", "--alt", "20", "--out"])
        .arg(&rec_dir)
        .output()
        .expect("spawn sim_run");
    assert!(out.status.success(), "sim_run failed: {}", String::from_utf8_lossy(&out.stderr));
    let record = rec_dir.join("flight.jsonl");
    let record_rows = read_record(&record);
    assert!(record_rows.len() >= 2500, "record too short");

    // Attempt 1: headless Vulkan. Attempt 2 (fallback): xvfb gl_compatibility.
    let mut tried = Vec::new();
    let headless = run_godot(&godot, &project, &dir.join("headless"), &record, false);
    match &headless {
        Ok(summary) => {
            println!("godot path: HEADLESS VULKAN");
            println!("godot smoke: {summary}");
        }
        Err(e) => {
            tried.push(format!("headless vulkan: {e}"));
            println!("headless Vulkan path failed, falling back to xvfb gl_compatibility: {e}");
            let xvfb = run_godot(&godot, &project, &dir.join("xvfb"), &record, true);
            match &xvfb {
                Ok(summary) => {
                    println!("godot path: XVFB GL_COMPATIBILITY");
                    println!("godot smoke: {summary}");
                }
                Err(e2) => {
                    tried.push(format!("xvfb gl_compatibility: {e2}"));
                    panic!("both render paths failed:\n{}", tried.join("\n"));
                }
            }
        }
    }

    // Determinism: a second run produces the same deterministic projection
    // (t, position, brightness — see canon for why du is excluded).
    let winning_dir = if headless.is_ok() { dir.join("headless") } else { dir.join("xvfb") };
    let again = run_godot(&godot, &project, &dir.join("again"), &record, headless.is_err());
    assert!(again.is_ok(), "second run failed: {:?}", again.err());
    let (a_frames, a_samples) = parse_replay(&winning_dir.join("replay.json"));
    let (b_frames, b_samples) = parse_replay(&dir.join("again").join("replay.json"));
    assert_eq!(a_frames, b_frames, "frame count differs between runs");
    assert_eq!(canon(&a_samples), canon(&b_samples), "second replay run diverged");
    let du_a: Vec<u64> = a_samples.iter().map(|s| s.du).collect();
    let du_b: Vec<u64> = b_samples.iter().map(|s| s.du).collect();
    println!(
        "second run: deterministic projection identical ({} samples); wall frame dt (informational) run1 mean {} us, run2 mean {} us",
        a_samples.len(),
        du_a.iter().sum::<u64>() / du_a.len() as u64,
        du_b.iter().sum::<u64>() / du_b.len() as u64
    );
    let _ = std::fs::remove_dir_all(&dir);
}