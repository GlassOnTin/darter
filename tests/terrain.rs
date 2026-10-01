//! M1 terrain acceptance tests (src/terrain.rs + the quad ground branch).
//!
//! Four layers, in record order:
//! 1. terrain.bin parsing — the byte contract tools/area_pack.py
//!    write_terrain_bin lays down — every rejection _validate_terrain_bin
//!    raises before its lattice checks, plus a clean parse.
//! 2. h_at: node values exact, on-diagonal points linear between the
//!    diagonal endpoints, the two triangle branches continuous across the
//!    shared diagonal, out-of-range queries clamped to the edge nodes.
//! 3. Python/Rust bit parity: build the real fixture DEM pack with the
//!    python tool, then compare TerrainGrid::h_at against
//!    area_pack.terrain_h_at bit-exactly (repr/parse round-trip both ways)
//!    over a few hundred sample points. The Rust h_at is an op-for-op
//!    mirror of terrain_h_at in f64; any accidental divergence fails here.
//! 4. The record path via sim_run: a zero-height grid produces a
//!    byte-identical flight record to no --terrain (the flat golden — the
//!    summary's terrain block is the only difference), and a motors-off
//!    ramp flight lands on the ramp (pz settles at h(x, y)) with a pinned
//!    record hash.
//!
//! grid_bytes()/ramp_bytes() below assemble terrain.bin with the f64-LE
//! layout the python writer uses; the ramp byte-identification pin comes
//! from a file python wrote with the identical node values, so byte drift
//! on either side of the bridge breaks it.

use std::path::{Path, PathBuf};
use std::process::Command;

use darter_core::preset::Preset;
use darter_core::quad::{G, Quad};
use darter_core::sha256::sha256_hex;
use darter_core::terrain::{Ground, TerrainGrid};
use darter_core::DVec3;

fn temp_dir(name: &str) -> PathBuf {
    let d = std::env::temp_dir().join(format!("darter-terrain-{name}-{}", std::process::id()));
    std::fs::create_dir_all(&d).unwrap();
    d
}

/// terrain.bin bytes as write_terrain_bin makes them: 56-byte LE header
/// (magic, fmt, cols, rows, step, origin_x, origin_y, z_min, z_max) then
/// row-major f64 node heights, z_min/z_max recomputed from the payload.
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
    for &v in z {
        b.extend_from_slice(&v.to_le_bytes());
    }
    b
}

/// Synthetic DEM ramp: 10x10 nodes @ 10 m, world x,y in [-45, 45]. Node
/// heights 2*i + 2.5*j are exactly representable and node (0,0) is 0.0, so
/// this is the plane h(x,y) = 0.2*(x+45) + 0.25*(y+45), which bilinear
/// reproduces to <1e-9 (fp rounding only). /tmp/ramp.bin from the bring-up
/// smoke was written by python with the same values.
fn ramp_bytes() -> Vec<u8> {
    let mut z = Vec::with_capacity(100);
    for j in 0..10 {
        for i in 0..10 {
            z.push(2.0 * i as f64 + 2.5 * j as f64);
        }
    }
    grid_bytes(10, 10, 10.0, -45.0, -45.0, &z)
}

// --- 1. parse -------------------------------------------------------------

#[test]
fn terrain_parse_rejects_bad_files() {
    let good = ramp_bytes();
    let g = TerrainGrid::parse(&good).expect("valid grid parses");
    assert_eq!(g.cols, 10);
    assert_eq!(g.rows, 10);
    assert_eq!(g.step, 10.0);
    assert_eq!(g.origin_x, -45.0);
    assert_eq!(g.origin_y, -45.0);
    assert_eq!(g.z_min, 0.0);
    assert_eq!(g.z_max, 40.5);
    assert_eq!(g.h_at(-45.0, -45.0), 0.0);

    let patch_u32 = |b: &mut Vec<u8>, off: usize, v: u32| {
        b[off..off + 4].copy_from_slice(&v.to_le_bytes());
    };
    let patch_f64 = |b: &mut Vec<u8>, off: usize, v: f64| {
        b[off..off + 8].copy_from_slice(&v.to_le_bytes());
    };
    let node_off = |k: usize| 56 + 8 * k;

    // (case name, mutated bytes, expected error fragment)
    let cases: Vec<(&str, Vec<u8>, &str)> = vec![
        (
            "bad magic",
            {
                let mut b = good.clone();
                patch_u32(&mut b, 0, 0x4452_4E54);
                b
            },
            "magic",
        ),
        (
            "wrong format version",
            {
                let mut b = good.clone();
                patch_u32(&mut b, 4, 2);
                b
            },
            "format version",
        ),
        (
            "cols < 2",
            {
                let mut b = good.clone();
                patch_u32(&mut b, 8, 1);
                b
            },
            "must be >= 2",
        ),
        (
            "rows < 2",
            {
                let mut b = good.clone();
                patch_u32(&mut b, 12, 1);
                b
            },
            "must be >= 2",
        ),
        ("short header", good[..40].to_vec(), "short"),
        (
            "payload truncated",
            good[..good.len() - 8].to_vec(),
            "bytes !=",
        ),
        (
            "payload one byte long",
            {
                let mut b = good.clone();
                b.push(0xFF);
                b
            },
            "bytes !=",
        ),
        (
            "step zero",
            {
                let mut b = good.clone();
                patch_f64(&mut b, 16, 0.0);
                b
            },
            "step",
        ),
        (
            "step negative",
            {
                let mut b = good.clone();
                patch_f64(&mut b, 16, -1.0);
                b
            },
            "step",
        ),
        (
            "step NaN",
            {
                let mut b = good.clone();
                patch_f64(&mut b, 16, f64::NAN);
                b
            },
            "step",
        ),
        (
            "origin not finite",
            {
                let mut b = good.clone();
                patch_f64(&mut b, 24, f64::INFINITY);
                b
            },
            "origin",
        ),
        (
            "z range not finite",
            {
                let mut b = good.clone();
                patch_f64(&mut b, 48, f64::NAN);
                b
            },
            "z range not finite",
        ),
        (
            "non-finite node",
            {
                let mut b = good.clone();
                patch_f64(&mut b, node_off(5), f64::NAN);
                b
            },
            "non-finite node",
        ),
        (
            "payload range disagrees with header",
            {
                let mut b = good.clone();
                patch_f64(&mut b, node_off(7), 999.0);
                b
            },
            "payload range",
        ),
        (
            "node (0,0) not the datum",
            {
                // Lift node (0,0) and move the header's z_min with it, so the
                // range check passes and the datum check is what fires.
                let mut b = good.clone();
                patch_f64(&mut b, node_off(0), 3.5);
                patch_f64(&mut b, 40, 2.0);
                b
            },
            "not 0.0",
        ),
    ];
    for (name, bytes, frag) in cases {
        let err = TerrainGrid::parse(&bytes)
            .err()
            .unwrap_or_else(|| panic!("case {name}: parser accepted bad bytes"));
        assert!(err.contains(frag), "case {name}: error {err:?} lacks {frag:?}");
    }

    let _ = std::fs::remove_dir_all(temp_dir("parse"));
}

// --- 2. h_at ---------------------------------------------------------------

#[test]
fn terrain_h_at_contract() {
    // cols 4, rows 3, step 8, origin (-12, -8): world x in [-12, 12],
    // y in [-8, 8]. Row 0 col 0 is the 0.0 datum.
    let z = vec![
        vec![0.0, 1.0, 2.5, 4.0],
        vec![-1.0, 0.5, 1.5, 3.0],
        vec![-2.0, -0.5, 0.75, 2.0],
    ];
    let mut flat = Vec::new();
    for row in &z {
        flat.extend_from_slice(row);
    }
    let g = TerrainGrid::parse(&grid_bytes(4, 3, 8.0, -12.0, -8.0, &flat)).unwrap();

    // Node values are exact.
    for j in 0..3 {
        for i in 0..4 {
            let v = g.h_at(-12.0 + 8.0 * i as f64, -8.0 + 8.0 * j as f64);
            assert_eq!(v, z[j][i], "node ({i},{j})");
        }
    }

    // Out-of-range queries clamp to the same edge-row interpolation, exactly
    // identical bits (the clamped query changes only which clamp ran).
    assert_eq!(g.h_at(-1000.0, -8.0), 0.0);
    assert_eq!(g.h_at(1000.0, -8.0), 4.0, "east clamp == row-0 corner node");
    assert_eq!(g.h_at(1000.0, -8.0), g.h_at(12.0, -8.0));
    assert_eq!(g.h_at(0.0, -1000.0), g.h_at(0.0, -8.0));
    assert_eq!(g.h_at(0.0, 1000.0), g.h_at(0.0, 8.0));

    // Cell (1,1): u,v in [1,2] = x in [-4,4], y in [0,8]; diagonal endpoints
    // z[1][1] = 0.5 and z[2][2] = 0.75. The midpoint of the shared diagonal
    // is exactly midway (the u >= v branch reduces to the linear form).
    let mid = g.h_at(0.0, 4.0);
    assert_eq!(mid, 0.625);

    // The two branches are continuous across the shared diagonal: queries
    // 1e-14 apart (one past uc == vc on each side) agree to <1e-12.
    let e = 1e-14;
    let p_above = g.h_at(-12.0 + 8.0 * (1.5 + e), 4.0); // u > v: branch A
    let p_below = g.h_at(0.0, -8.0 + 8.0 * (1.5 + e)); // v > u: branch B
    let diff = (p_above - p_below).abs();
    assert!(
        diff < 1e-12,
        "diagonal discontinuity {diff:?} between {p_above:?} and {p_below:?}"
    );
    assert!((p_above - 0.625).abs() < 1e-12);
    assert!((p_below - 0.625).abs() < 1e-12);

    let _ = std::fs::remove_dir_all(temp_dir("hat"));
}

#[test]
fn terrain_h_at_reproduces_a_plane() {
    let g = TerrainGrid::parse(&ramp_bytes()).unwrap();
    let n = 40usize;
    for a in 0..=n {
        for b in 0..=n {
            // Off-node probes: node + fractional-cell offsets.
            let x = -45.0 + 90.0 * a as f64 / n as f64 + 0.37 * g.step;
            let y = -45.0 + 90.0 * b as f64 / n as f64 - 0.19 * g.step;
            // Analytic value at the CLAMPED probe point: the grid reproduces
            // the plane inside the hull, and clamps to the hull outside.
            let xc = x.max(-45.0).min(45.0);
            let yc = y.max(-45.0).min(45.0);
            let want = 0.2 * (xc + 45.0) + 0.25 * (yc + 45.0);
            let got = g.h_at(x, y);
            assert!(
                (got - want).abs() < 1e-9,
                "plane at ({x}, {y}): {got:?} != {want:?}"
            );
        }
    }
    // Beyond-edge clamps land on the same plane.
    assert_eq!(g.h_at(-1000.0, 500.0), 22.5);
    // x clamps to the east edge (0.2*90 = 18), y to the south edge (0).
    assert_eq!(g.h_at(1000.0, -1000.0), 18.0);
    assert!((g.h_at(-1000.0, 12.3) - 0.25 * (12.3 + 45.0)).abs() < 1e-9);
}

// --- 3. python bit parity ---------------------------------------------------

/// Runs area_pack's own validator + terrain_h_at over the points Rust
/// passes in and prints one repr'd height per line.
const PY_PROG: &str = r#"
import json, sys
sys.path.insert(0, "tools")
import area_pack
pdir = sys.argv[1]
pack = json.load(open(pdir + "/pack.json"))
errs = []
grid = area_pack._validate_terrain_bin(pdir + "/terrain.bin", pack["elevation"], errs.append)
if errs or grid is None:
    sys.stderr.write("validation failed: %r\n" % (errs,))
    sys.exit(2)
out = []
for pair in sys.argv[2].split(";"):
    xs, ys = pair.split(",")
    out.append(repr(area_pack.terrain_h_at(grid, float(xs), float(ys))))
sys.stdout.write("\n".join(out))
"#;

fn build_fixture_pack(dir: &Path) {
    let out = Command::new("python3")
        .args([
            "tools/area_pack.py",
            "--osm",
            "tests/fixtures/osm_home_area.json",
            "--lat",
            "50.8989",
            "--lon",
            "-1.0586",
            "--seed",
            "5",
            "--elevation",
            "glo30",
            "--dem-file",
            "tests/fixtures/dem_home_area.tif",
            "--out",
        ])
        .arg(dir)
        .output()
        .expect("python3 tools/area_pack.py");
    assert!(
        out.status.success(),
        "glo30 fixture build failed:\n{}{}",
        String::from_utf8_lossy(&out.stdout),
        String::from_utf8_lossy(&out.stderr)
    );
}

#[test]
fn terrain_h_at_bit_matches_python_on_fixture_pack() {
    let dir = temp_dir("parity");
    build_fixture_pack(&dir);
    let bytes = std::fs::read(dir.join("terrain.bin")).expect("fixture terrain.bin");
    let g = TerrainGrid::parse(&bytes).expect("fixture terrain.bin parses");

    // Sample set: nodes on strides, a 16x16 lattice across the span with
    // both edges included, on-diagonal + just-off-diagonal probes, and a few
    // beyond-edge corners (the python side clamps the same way).
    let sp = (g.cols - 1) as f64 * g.step;
    let sy = (g.rows - 1) as f64 * g.step;
    let mut pts: Vec<(f64, f64)> = Vec::new();
    for j in (0..g.rows).step_by(31) {
        for i in (0..g.cols).step_by(29) {
            pts.push((g.origin_x + i as f64 * g.step, g.origin_y + j as f64 * g.step));
        }
    }
    let n = 15usize;
    for a in 0..=n {
        for b in 0..=n {
            let x = g.origin_x + (a as f64 / n as f64) * sp;
            let y = g.origin_y + (b as f64 / n as f64) * sy;
            pts.push((x, y));
        }
    }
    for &(cell_i, cell_j) in &[(30u64, 40u64), (75, 75), (120, 100), (1, 1)] {
        let u = cell_i as f64 + 0.3;
        let v = cell_j as f64 + 0.3;
        pts.push((g.origin_x + u * g.step, g.origin_y + v * g.step));
        pts.push((g.origin_x + (u + 1e-14) * g.step, g.origin_y + v * g.step));
        pts.push((g.origin_x + u * g.step, g.origin_y + (v + 1e-14) * g.step));
    }
    pts.push((g.origin_x - 7.0, g.origin_y - 7.0));
    pts.push((g.origin_x + sp + 7.0, g.origin_y + sy + 7.0));
    pts.push((g.origin_x - 7.0, g.origin_y + sy + 7.0));
    pts.push((g.origin_x + 0.5 * sp, g.origin_y + sy + 3.0));

    let arg: String = pts
        .iter()
        .map(|(x, y)| format!("{x},{y}"))
        .collect::<Vec<_>>()
        .join(";");
    let out = Command::new("python3")
        .arg("-c")
        .arg(PY_PROG)
        .arg(dir.to_str().unwrap())
        .arg(&arg)
        .output()
        .expect("python3 h_at prober");
    assert!(
        out.status.success(),
        "python prober failed:\n{}{}",
        String::from_utf8_lossy(&out.stdout),
        String::from_utf8_lossy(&out.stderr)
    );
    let text = String::from_utf8_lossy(&out.stdout);
    let lines: Vec<&str> = text.lines().collect();
    assert_eq!(lines.len(), pts.len(), "python returned one height per point");
    let mut bad = 0usize;
    for (i, ((x, y), line)) in pts.iter().zip(&lines).enumerate() {
        let want: f64 = line.trim().parse().expect("python repr float");
        let got = g.h_at(*x, *y);
        if got != want {
            if bad < 5 {
                eprintln!("#{i} @ ({x},{y}): rust {got:?} != python {want:?} ({line})");
            }
            bad += 1;
        }
    }
    assert_eq!(bad, 0, "{bad}/{} heights differ from terrain_h_at", pts.len());
    let _ = std::fs::remove_dir_all(&dir);
}

// --- 4. the record path ------------------------------------------------------

/// Spawn sim_run in core mode; returns stdout (the log evidence).
fn run_sim(dir: &Path, args: &[&str]) -> String {
    let out = Command::new(env!("CARGO_BIN_EXE_sim_run"))
        .args(args)
        .arg("--out")
        .arg(dir)
        .output()
        .expect("spawn sim_run");
    assert!(
        out.status.success(),
        "sim_run failed: {}",
        String::from_utf8_lossy(&out.stderr)
    );
    String::from_utf8_lossy(&out.stdout).into_owned()
}

struct Row {
    pz: f64,
    vx: f64,
    vy: f64,
    vz: f64,
}

fn num(line: &str, key: &str) -> f64 {
    let pat = format!("\"{key}\":");
    let i = line.find(&pat).unwrap_or_else(|| panic!("field {key} missing in {line}"));
    let rest = &line[i + pat.len()..];
    let end = rest.find([',', '}']).expect("field terminator");
    rest[..end].parse().expect("float parse")
}

fn read_record(path: &Path) -> (Vec<Row>, Vec<String>) {
    let text = std::fs::read_to_string(path).expect("flight record");
    let mut rows = Vec::new();
    let mut raw = Vec::new();
    for line in text.lines() {
        if line.starts_with("{\"schema") {
            continue;
        }
        raw.push(line.to_string());
        rows.push(Row {
            pz: num(line, "pz"),
            vx: num(line, "vx"),
            vy: num(line, "vy"),
            vz: num(line, "vz"),
        });
    }
    (rows, raw)
}

fn record_hash(summary: &str) -> String {
    let i = summary.find("\"record_hash\": \"0x").expect("record_hash in summary");
    summary[i + 18..i + 34].to_string()
}

/// The flat golden: a zero-height grid must produce a byte-identical record
/// to no --terrain at all. That equality is the reason the record schema
/// carries no terrain field and the spawn keeps the AGL convention: h is
/// exactly 0.0 everywhere, `alt + h == alt` bit-exactly, and Ground::Flat
/// stays the pre-M1 constant branch.
#[test]
fn sim_run_flat_terrain_record_matches_no_terrain() {
    let dir = temp_dir("flatgolden");
    let bytes = grid_bytes(8, 6, 30.0, -105.0, -75.0, &vec![0.0f64; 48]);
    std::fs::write(dir.join("zero.bin"), &bytes).unwrap();
    let tp = dir.join("zero.bin").to_str().unwrap().to_string();

    run_sim(&dir.join("a"), &["--seed", "5", "--duration", "2"]);
    run_sim(&dir.join("b"), &["--seed", "5", "--duration", "2", "--terrain", &tp]);

    let ra = std::fs::read(dir.join("a").join("flight.jsonl")).unwrap();
    let rb = std::fs::read(dir.join("b").join("flight.jsonl")).unwrap();
    assert_eq!(ra, rb, "a zero-height terrain grid changed the record");

    let sa = std::fs::read_to_string(dir.join("a").join("summary.json")).unwrap();
    let sb = std::fs::read_to_string(dir.join("b").join("summary.json")).unwrap();
    assert_eq!(record_hash(&sa), record_hash(&sb));
    assert!(!sa.contains("\"terrain\""), "flat summary grew a terrain block");
    assert!(sb.contains(&format!("\"path\": \"{tp}\",")), "terrain path in summary");
    assert!(sb.contains(&format!("\"sha256\": \"{}\"", sha256_hex(&bytes))));

    let _ = std::fs::remove_dir_all(&dir);
}

/// Motors-off drop from 30 m AGL over the plane ramp (h(0,0) = 20.25, so the
/// spawn sits at 50.25): the craft falls, bounces out (0.2 restitution), and
/// settles ON the ramp with pz = h(x, y). The record hash is pinned so any
/// drift in the terrain height path (h_at, the ground branch, the spawn
/// offset) breaks a test rather than slipping into a record.
///
/// If this pin fails after an intentional change: rerun
/// `sim_run --terrain <ramp.bin> --seed 7 --duration 8 --throttle 0 --alt 30`
/// and update PINNED_RAMP_HASH from summary.json.
#[test]
fn sim_run_ramp_flight_lands_on_the_ramp() {
    // Byte-identification pin against the python-built bring-up file: same
    // node values + header, so the sha must match /tmp/ramp.bin's.
    let bytes = ramp_bytes();
    assert_eq!(
        sha256_hex(&bytes),
        "ed7eb951d04041c432aba5df18f64298d8346138cfa64939ac9b5be6ef12fe4d",
        "ramp fixture drifted from the python writer's bytes"
    );

    let dir = temp_dir("rampflight");
    std::fs::write(dir.join("ramp.bin"), &bytes).unwrap();
    let tp = dir.join("ramp.bin").to_str().unwrap().to_string();
    let stdout = run_sim(
        &dir,
        &["--seed", "7", "--duration", "8", "--throttle", "0", "--alt", "30", "--terrain", &tp],
    );
    assert!(stdout.contains("grid 10x10, step 10 m, z [0.0, 40.5]"), "terrain banner: {stdout}");

    let (rows, _) = read_record(&dir.join("flight.jsonl"));
    let last = rows.last().expect("record rows");
    assert_eq!(last.vx, 0.0, "no horizontal drift with motors off");
    assert_eq!(last.vy, 0.0);
    assert!(last.vz.abs() < 0.01, "settled vertical velocity: {}", last.vz);
    assert!((last.pz - 20.25).abs() < 1e-6, "rests on the ramp plane: pz {}", last.pz);

    let peak = rows.iter().map(|r| r.pz).fold(f64::NEG_INFINITY, f64::max);
    assert!(peak > 50.0, "started at 50.25, saw {peak}");
    assert!(peak < 50.26, "no bounce above spawn: {peak}");

    let summary = std::fs::read_to_string(dir.join("summary.json")).unwrap();
    const PINNED_RAMP_HASH: &str = "f5bf940927db2705";
    assert_eq!(
        record_hash(&summary),
        PINNED_RAMP_HASH,
        "ramp record hash changed; if intentional, regenerate with \
         `sim_run --terrain <ramp.bin> --seed 7 --duration 8 --throttle 0 --alt 30` \
         and update PINNED_RAMP_HASH in tests/terrain.rs"
    );
    assert!(summary.contains(&format!("\"sha256\": \"{}\"", sha256_hex(&bytes))));

    let _ = std::fs::remove_dir_all(&dir);
}

/// Direct quad check complementing the pinned record: a resting airframe on
/// the ramp holds px/py bit-exactly, pz = h(x, y) bit-exactly each step, and
/// reads the grounded +1 g proper acceleration the IMU model hands out.
#[test]
fn quad_rests_on_ramp() {
    let g = TerrainGrid::parse(&ramp_bytes()).unwrap();

    let (x, y) = (5.0f64, -15.0f64); // h = 0.2*50 + 0.25*30 = 17.5
    let mut q = Quad::new(Preset::FREESTYLE_5IN, DVec3::new(x, y, g.h_at(x, y)));
    q.ground = Ground::Grid(g.clone());
    for _ in 0..2000 {
        q.step(0.001);
    }
    assert_eq!(q.state.pos.x, x, "no lateral force on a rest airframe");
    assert_eq!(q.state.pos.y, y);
    assert_eq!(q.state.pos.z, g.h_at(x, y), "pz = h(x,y) exactly each step");
    assert!(
        (q.state.pos.z - 17.5).abs() < 1e-9,
        "plane value at ({x},{y}): {}",
        q.state.pos.z
    );
    let (omega, accel) = q.imu();
    assert!(omega.x.abs() < 1e-12 && omega.y.abs() < 1e-12 && omega.z.abs() < 1e-12);
    assert_eq!(accel.z, G, "grounded airframe reads +1 g");
    assert_eq!(accel.x, 0.0);
    assert_eq!(accel.y, 0.0);
    assert!(q.state.vel.z.abs() < 0.01, "settled vz: {}", q.state.vel.z);

    let mut q2 = Quad::new(Preset::FREESTYLE_5IN, DVec3::new(x, y, g.h_at(x, y) + 40.0));
    q2.ground = Ground::Grid(g.clone());
    for _ in 0..5000 {
        q2.step(0.001);
    }
    assert_eq!(q2.state.pos.x, x);
    assert_eq!(q2.state.pos.y, y);
    assert_eq!(q2.state.pos.z, g.h_at(x, y), "dropped 40 m settles on the ramp");
}