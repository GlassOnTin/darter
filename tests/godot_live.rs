//! M2 rung 1 — darter-core as a Godot GDExtension: the live plant.
//!
//! The renderer project today only replays recorded flights (pack_replay).
//! Every later M2 rung — live flight with the Pocket, the Betaflight SITL
//! loop on device — needs the physics core running inside the Godot
//! process, in real time, driven from GDScript. `tools/gdext` is that
//! bridge: a standalone godot-rust (gdext) cdylib, `libdarter_gd.so`,
//! exposing a `DarterQuad` class over the core's `Quad`. The core crate
//! itself stays Godot-free (glam + libc only); the extension is the only
//! place where the two meet.
//!
//! The acceptance contract is the same one the rest of the repo is held
//! to: the class's flight loop must be indistinguishable from sim_run's
//! core mode. Per test, a `sim_run --mode core` flight (32 x 125 us
//! substeps per 250 Hz tick, one sample per tick, the darter_record
//! header it writes) and the same flight flown through the extension's
//! per-call API from `live_probe.gd` produce byte-identical
//! `darter_record` JSONL files. Byte equality covers the header fields
//! (mode, seed, duration, preset, profile lines) and every sampled
//! position, velocity, quaternion and motor number — no float tolerance,
//! because both sides are the same f64 loop shape written by the same
//! `RecordWriter`.
//!
//! Two cases: flat ground (no `--terrain`) and a small hand-built terrain
//! grid (the `terrain.bin` byte layout `write_terrain_bin` uses — the
//! same construction tests/terrain.rs checks), so the spawn-over-DEM and
//! `h_at` integration inside the class is covered, not only the free-air
//! path. Wind and sensors stay off (sim_run core mode defaults), and the
//! profile lines pin that: identical headers are impossible otherwise.
//!
//! Provenance is enforced at runtime like tests/godot.rs does: the pinned
//! Godot binary (official 4.7.2-stable linux.x86_64) must hash to the
//! constant below.
//!
//! Recipe (mirrors the CI godot-replay job):
//!   cargo build --release --manifest-path tools/gdext/Cargo.toml
//!   cargo test --test godot_live -- --ignored
//! The suite stages the built .so into tools/godot_smoke/ext/ (the path
//! `ext/darter_gd.gdextension` points at; gitignored payload) once per
//! process, then re-runs the --import pass so the extension registers.
//! gdext api-4-7 requires Godot 4.7 exactly; the pinned editor satisfies it.
//!
//! Not verified here (later M2 rungs): the live API in real time per
//! rendered frame (pacing, not determinism — no wall clock in this
//! suite), the Android arm64 .so inside the exported APK, the arming/
//! MSP closed loop with a spawned Betaflight SITL child.

use std::path::{Path, PathBuf};
use std::process::Command;
use std::sync::Once;

/// Official Godot 4.7.2-stable Linux x86_64, standard (non-.NET) build;
/// identical provenance pin to tests/godot.rs.
const GODOT_SHA256: &str = "8d106cbe6144c2dc7e881d61d2429c1a8a76e6b22ef48bd5e48dcf934953f71e";

const PROJECT: &str = "tools/godot_smoke";
/// The extension library path the committed
/// tools/godot_smoke/ext/darter_gd.gdextension entry points at.
const EXT_DEST: &str = "tools/godot_smoke/ext/libdarter_gd.so";

fn temp_dir(name: &str) -> PathBuf {
    let d = std::env::temp_dir().join(format!("darter-godot-live-{name}-{}", std::process::id()));
    std::fs::create_dir_all(&d).unwrap();
    d
}

/// Locate and hash-verify the pinned Godot binary (same contract as
/// tests/godot.rs).
fn locate_godot() -> PathBuf {
    let path = match std::env::var("DARTER_GODOT") {
        Ok(p) => PathBuf::from(p),
        Err(_) => PathBuf::from("tools/godot/bin/godot"),
    };
    assert!(path.exists(), "Godot binary missing at {path:?}: run the download pinned in tests/godot.rs (Godot 4.7.2-stable linux.x86_64), or set DARTER_GODOT");
    let bytes = std::fs::read(&path).expect("read godot binary");
    let mut h = darter_core::sha256::Sha256::new();
    h.update(&bytes);
    let got = darter_core::sha256::to_hex(&h.finish());
    assert_eq!(
        got, GODOT_SHA256,
        "Godot binary at {path:?} does not match the pinned provenance"
    );
    path
}

/// The built GDExtension cdylib. Built separately from `cargo test` on
/// purpose: gdext is a standalone project (own Cargo.lock, own target), so
/// the offline cargo tier never has to fetch or compile Godot bindings.
fn locate_extension() -> PathBuf {
    let path = PathBuf::from("tools/gdext/target/release/libdarter_gd.so");
    assert!(
        path.exists(),
        "GDExtension missing at {path:?}: build it first (cargo build --release --manifest-path tools/gdext/Cargo.toml)"
    );
    path
}

/// Stage the .so where the committed .gdextension entry resolves it
/// (inside the project; gitignored payload like areas/*/), then re-run
/// the import pass with it present so extension_list.cfg registers the
/// DarterQuad class — a fresh checkout's earlier import, without the
/// .so, would have left it out. Runs once per test-binary process: the
/// parallel test threads would otherwise race two --import passes over
/// the same .godot editor state and intermittently fail one.
fn prepare_extension(godot: &Path) {
    static ONCE: Once = Once::new();
    ONCE.call_once(|| {
        std::fs::create_dir_all("tools/godot_smoke/ext").unwrap();
        std::fs::copy(locate_extension(), EXT_DEST).unwrap();
        let imp = Command::new(godot)
            .arg("--headless")
            .arg("--path").arg(PROJECT)
            .arg("--import")
            .output()
            .expect("spawn godot --import");
        let log = format!("stdout: {}\nstderr: {}", String::from_utf8_lossy(&imp.stdout), String::from_utf8_lossy(&imp.stderr));
        assert!(imp.status.success(), "godot --import pass failed\n{log}");
    });
}

/// Run one `sim_run --mode core` flight and return its 16-hex record hash.
fn run_sim_run(
    out_dir: &Path,
    throttle: f64,
    duration: f64,
    terrain: Option<&Path>,
) -> String {
    let bin = PathBuf::from("target/debug/sim_run");
    assert!(bin.exists(), "sim_run missing at {bin:?}: cargo test builds it; run the suite via cargo test");
    let mut cmd = Command::new(bin);
    cmd.arg("--mode").arg("core")
        .arg("--seed").arg("1")
        .arg("--throttle").arg(format!("{throttle}"))
        .arg("--duration").arg(format!("{duration}"))
        .arg("--out").arg(out_dir);
    if let Some(t) = terrain {
        cmd.arg("--terrain").arg(t);
    }
    let out = cmd.output().expect("spawn sim_run");
    let log = format!("stdout: {}\nstderr: {}", String::from_utf8_lossy(&out.stdout), String::from_utf8_lossy(&out.stderr));
    assert!(out.status.success(), "sim_run failed\n{log}");
    hash_from_log(&log, "record hash ")
}

/// Run one probe flight through the extension and return its record hash.
/// live_probe.gd drives the DarterQuad class per the env contract; the
/// headless display server is enough because nothing renders a view.
fn run_probe(
    godot: &Path,
    out_dir: &Path,
    throttle: f64,
    duration: f64,
    ticks: usize,
    terrain: Option<&Path>,
) -> String {
    std::fs::create_dir_all(out_dir).unwrap();
    let record = out_dir.join("flight.jsonl");
    let mut cmd = Command::new(godot);
    cmd.arg("--headless")
        .arg("--path").arg(PROJECT)
        .arg("res://live_probe.tscn")
        .env("DARTER_PROBE_RECORD", &record)
        .env("DARTER_PROBE_DURATION", format!("{duration}"))
        .env("DARTER_PROBE_SEED", "1")
        .env("DARTER_PROBE_THROTTLE", format!("{throttle}"))
        .env("DARTER_PROBE_TICKS", ticks.to_string())
        .env("DARTER_PROBE_TERRAIN", terrain.map_or(String::new(), |p| p.display().to_string()));
    let out = cmd.output().expect("spawn godot");
    let log = format!("stdout: {}\nstderr: {}", String::from_utf8_lossy(&out.stdout), String::from_utf8_lossy(&out.stderr));
    assert!(out.status.success(), "probe scene failed\n{log}");
    hash_from_log(&log, "PROBE hash=")
}

/// Parse the 16 hex chars directly following `prefix` in a log line and
/// assert they are well-formed.
fn hash_from_log(log: &str, prefix: &str) -> String {
    let mut found: Option<String> = None;
    for line in log.lines() {
        if let Some(i) = line.find(prefix) {
            let rest = &line[i + prefix.len()..];
            let hex: String = rest.chars().take(16).collect();
            assert!(
                hex.len() == 16 && hex.chars().all(|c| c.is_ascii_hexdigit()),
                "malformed hash after {prefix:?} in {line:?}"
            );
            found = Some(hex);
            break;
        }
    }
    found.unwrap_or_else(|| panic!("no hash after {prefix:?} in log:\n{log}"))
}

/// terrain.bin bytes as tools/area_pack.py's write_terrain_bin lays them
/// down (tests/terrain.rs mirrors the layout; duplicated here so both
/// suites stay standalone): 56-byte LE header then row-major f64 heights.
fn grid_bytes(cols: usize, rows: usize, step: f64, ox: f64, oy: f64, z: &[f64]) -> Vec<u8> {
    assert_eq!(z.len(), cols * rows, "z payload size");
    let (mut zmin, mut zmax) = (f64::INFINITY, f64::NEG_INFINITY);
    for &v in z {
        zmin = zmin.min(v);
        zmax = zmax.max(v);
    }
    let mut b = Vec::with_capacity(56 + 8 * z.len());
    b.extend_from_slice(&darter_core::terrain::TERRAIN_MAGIC.to_le_bytes());
    b.extend_from_slice(&1u32.to_le_bytes()); // TERRAIN_FMT_VERSION
    b.extend_from_slice(&(cols as u32).to_le_bytes());
    b.extend_from_slice(&(rows as u32).to_le_bytes());
    b.extend_from_slice(&step.to_le_bytes());
    b.extend_from_slice(&ox.to_le_bytes());
    b.extend_from_slice(&oy.to_le_bytes());
    b.extend_from_slice(&zmin.to_le_bytes());
    b.extend_from_slice(&zmax.to_le_bytes());
    for v in z {
        b.extend_from_slice(&v.to_le_bytes());
    }
    b
}

/// The shared flight+parity protocol: a sim_run core flight, then the same
/// flight through the extension, byte-compared. Ticks come from sim_run's
/// summary (its own TICK_DT expression decides the count — no duplicated
/// float maths on this side).
fn assert_parity(name: &str, throttle: f64, duration: f64, terrain: Option<&Path>) {
    let godot = locate_godot();
    prepare_extension(&godot);
    let dir = temp_dir(name);
    let sim_dir = dir.join("sim");
    let probe_dir = dir.join("probe");

    let sim_hash = run_sim_run(&sim_dir, throttle, duration, terrain);
    let summary = std::fs::read_to_string(sim_dir.join("summary.json")).unwrap();
    let i = summary.find("\"ticks\": ").unwrap();
    let ticks: usize = summary[i + "\"ticks\": ".len()..]
        .split(',')
        .next()
        .unwrap()
        .trim()
        .parse()
        .unwrap();
    println!("sim ticks {ticks}, hash {sim_hash}");

    let probe_hash = run_probe(&godot, &probe_dir, throttle, duration, ticks, terrain);
    println!("probe ticks {ticks}, hash {probe_hash}");
    assert_eq!(sim_hash, probe_hash, "record hashes differ");

    let sim_record = sim_dir.join("flight.jsonl");
    let probe_record = probe_dir.join("flight.jsonl");
    assert_eq!(
        std::fs::read(&sim_record).unwrap_or_else(|e| panic!("read {sim_record:?}: {e}")),
        std::fs::read(&probe_record).unwrap_or_else(|e| panic!("read {probe_record:?}: {e}")),
        "records not byte-identical"
    );
    println!("parity: records byte-identical ({name})");
}

/// Flat ground: no --terrain on the sim side, empty DARTER_PROBE_TERRAIN on
/// the extension side. Ground::None, still-air core default.
#[test]
#[ignore = "needs tools/godot/bin/godot (pinned 4.7.2-stable) and a built tools/gdext extension; run: cargo test --test godot_live -- --ignored"]
fn godot_live_flat_parity() {
    assert_parity("flat", 0.2, 1.0, None);
}

/// Hand-built terrain grid: an x-ramp, so the spawn z is the DEM height at
/// the origin and ground contact follows the grid on both sides.
#[test]
#[ignore = "needs tools/godot/bin/godot (pinned 4.7.2-stable) and a built tools/gdext extension; run: cargo test --test godot_live -- --ignored"]
fn godot_live_terrain_parity() {
    let dir = temp_dir("terrain-src");
    let terrain = dir.join("terrain.bin");
    // 8x8 nodes, 5 m step, origin centred, heights ramping with x:
    // row-major over (iy, ix), ix = i % 8.
    let (cols, rows, step) = (8usize, 8usize, 5.0f64);
    let (ox, oy) = (-17.5f64, -17.5f64);
    let mut z = Vec::with_capacity(cols * rows);
    for i in 0..cols * rows {
        let ix = i % cols;
        z.push(ix as f64 * 2.0);
    }
    std::fs::write(&terrain, grid_bytes(cols, rows, step, ox, oy, &z)).unwrap();
    assert_parity("terrain", 0.2, 1.0, Some(&terrain));
}