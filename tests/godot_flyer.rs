//! M2 rung 2 — the closed-flyer class: real-firmware flight driven from Godot.
//!
//! sim_run --mode closed wraps a whole runner behind a CLI: spawn the
//! Betaflight SITL child in a fresh cwd, apply the profile over the MSPv2
//! CLI with a diff-readback gate, fly the scripted-RC arming state machine
//! over the two links (UDP FDM/servo, MSP TCP telemetry on a dedicated
//! thread), record the darter_record schema. tools/gdext exposes that same
//! runner as a `DarterFlyer` class (src/flyer.rs, lifted from sim_run so
//! both call paths run the identical code). This suite is its acceptance
//! test, and unlike M2a's byte-identical parity (tests/godot_live.rs) the
//! contract here is BEHAVIOURAL: the SITL's own PID loop runs on wall-clock
//! time, so identical inputs vary run-to-run and byte equality is impossible
//! by design (tests/sitl_loop.rs, docs/physics.md section 11). The asserts
//! mirror sitl_loop's measured-yaw test exactly, driven instead through the
//! class API from fly_probe.gd, plus the record-header contract (mode
//! "closed", the applied profile lines, SITL provenance with a Betaflight
//! version string).
//!
//! Recipe (dev machine; NOT in CI — it spawns a real SITL child on fixed
//! ports TCP 5761 / UDP 9002-9004, which needs the Betaflight SITL binary
//! at the path named by DARTER_FLYER_SITL_BIN / the runner default, absent
//! on CI runners):
//!   cargo build --release --manifest-path tools/gdext/Cargo.toml
//!   cargo test --test godot_flyer -- --ignored --test-threads=1
//! Serial like tests/sitl_loop.rs: parallel runs race for the ports (a busy
//! port skips, as there).
//!
//! Not verified here: the live per-rendered-frame API (pacing under a real
//! display driver; this suite is headless and wall-paced only), terrain/
//! sensor/wind configs through the class (sim_run covers those), the
//! Android arm64 .so, and the M2c spawned-SITL seam (the sitl-binary
//! parameter becomes an on-device path there).

use std::io::Read as _;
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use std::sync::Once;
use std::time::{Duration, Instant};

/// Official Godot 4.7.2-stable Linux x86_64, standard (non-.NET) build;
/// identical provenance pin to tests/godot.rs and tests/godot_live.rs.
const GODOT_SHA256: &str = "8d106cbe6144c2dc7e881d61d2429c1a8a76e6b22ef48bd5e48dcf934953f71e";

const PROJECT: &str = "tools/godot_smoke";
const EXT_DEST: &str = "tools/godot_smoke/ext/libdarter_gd.so";

/// Wall-clock cap on one probe process. The flight itself is ~13 s
/// wall-paced plus SITL spawn and godot boot; anything well past that means
/// a hung headless godot (one that never loaded its script idles forever —
/// 440 s measured in the first RED run) and is killed here.
const PROBE_TIMEOUT: Duration = Duration::from_secs(240);

const DURATION_S: f64 = 13.0;

fn temp_dir(name: &str) -> PathBuf {
    let d = std::env::temp_dir().join(format!("darter-godot-flyer-{name}-{}", std::process::id()));
    std::fs::create_dir_all(&d).unwrap();
    d
}

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

fn locate_extension() -> PathBuf {
    let path = PathBuf::from("tools/gdext/target/release/libdarter_gd.so");
    assert!(
        path.exists(),
        "GDExtension missing at {path:?}: build it first (cargo build --release --manifest-path tools/gdext/Cargo.toml)"
    );
    path
}

/// Stage the .so where the committed .gdextension entry resolves it and
/// re-run the import pass so the DarterFlyer class registers. Once per
/// test-binary process (parallel --import passes race editor state).
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

/// The fixed SITL MSP port must be free before spawning (a leftover SITL
/// would make the spawn fail confusingly). Best-effort probe.
fn port_free_or_skip() -> bool {
    match std::net::TcpListener::bind("127.0.0.1:5761") {
        Ok(_) => true,
        Err(_) => false,
    }
}

/// Run one closed flight through the DarterFlyer class and return the work
/// dir holding its record. The probe pumps 50 ticks per call, so the whole
/// run flows through flyer_pump() chunking — the class's per-call contract.
fn run_probe(name: &str) -> PathBuf {
    let godot = locate_godot();
    prepare_extension(&godot);
    let dir = temp_dir(name);
    let env_bin = std::env::var("DARTER_FLYER_SITL_BIN")
        .unwrap_or_else(|_| darter_core::flyer::DEFAULT_SITL_BIN.to_string());
    let mut child = Command::new(godot)
        .arg("--headless")
        .arg("--path").arg(PROJECT)
        .arg("res://fly_probe.tscn")
        .env("DARTER_FLYER_WORK", &dir)
        .env("DARTER_FLYER_BIN", &env_bin)
        .env("DARTER_FLYER_DURATION", format!("{DURATION_S}"))
        .env("DARTER_FLYER_SEED", "5")
        .env("DARTER_FLYER_THROTTLE", "0.16")
        .env("DARTER_FLYER_YAW", "0.5")
        .env("DARTER_FLYER_YAW_UNTIL", "6.0")
        .env("DARTER_FLYER_PROFILE", "")
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .expect("spawn godot");
    // Poll to a deadline instead of blocking on wait_with_output: the probe
    // outputs only a handful of lines (nowhere near the pipe buffer), so a
    // piped parent that never reads cannot stall it.
    let deadline = Instant::now() + PROBE_TIMEOUT;
    let status = loop {
        match child.try_wait().expect("wait godot") {
            Some(st) => break st,
            None if Instant::now() >= deadline => {
                let _ = child.kill();
                let _ = child.wait();
                panic!(
                    "flyer probe scene still running after {} s — killed",
                    PROBE_TIMEOUT.as_secs()
                );
            }
            None => std::thread::sleep(Duration::from_millis(200)),
        }
    };
    let mut stdout = Vec::new();
    let mut stderr = Vec::new();
    if let Some(mut s) = child.stdout.take() {
        let _ = s.read_to_end(&mut stdout);
    }
    if let Some(mut s) = child.stderr.take() {
        let _ = s.read_to_end(&mut stderr);
    }
    let log = format!(
        "stdout: {}\nstderr: {}",
        String::from_utf8_lossy(&stdout),
        String::from_utf8_lossy(&stderr)
    );
    assert!(status.success(), "flyer probe scene failed\n{log}");
    if !log.contains("FLYER hash=") {
        panic!("probe produced no FLYER hash line\n{log}");
    }
    dir
}

/// One sampled row of flight.jsonl (the darter_record schema the closed
/// runner writes). Only rows carrying FC telemetry are loaded.
struct Row {
    t: f64,
    qw: f64,
    qx: f64,
    qy: f64,
    qz: f64,
    att_y_deg: f64,
    armed: bool,
    /// The FC's arming_disable bitfield (runtime_config.h), NOT an armed
    /// boolean — armed detection is flags & 1 below.
    arm_disable: u32,
}

fn fnum(line: &str, key: &str) -> f64 {
    let needle = format!("\"{key}\":");
    let i = line
        .find(&needle)
        .unwrap_or_else(|| panic!("field {key} missing in: {line}"));
    let rest = &line[i + needle.len()..];
    let end = rest
        .find(|c: char| !(c.is_ascii_digit() || matches!(c, '.' | '-' | '+' | 'e' | 'E')))
        .unwrap_or(rest.len());
    rest[..end]
        .parse()
        .unwrap_or_else(|_| panic!("field {key} not numeric in: {line}"))
}

fn load_record(out_dir: &Path) -> (String, Vec<Row>) {
    let path = out_dir.join("flight.jsonl");
    let text = std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("record {path:?}: {e}"));
    let mut lines = text.lines();
    let header = lines
        .next()
        .expect("record needs a header line")
        .to_string();
    let rows: Vec<Row> = lines
        .filter(|l| l.starts_with("{\"t\"") && l.contains("\"att_r\""))
        .map(|l| Row {
            t: fnum(l, "t"),
            qw: fnum(l, "qw"),
            qx: fnum(l, "qx"),
            qy: fnum(l, "qy"),
            qz: fnum(l, "qz"),
            att_y_deg: fnum(l, "att_y"),
            armed: fnum(l, "flags") as u32 & 1 == 1,
            arm_disable: fnum(l, "arm") as u32,
        })
        .collect();
    (header, rows)
}

fn wrap180(d: f64) -> f64 {
    (d + 180.0).rem_euclid(360.0) - 180.0
}

fn armed_rows(rows: &[Row]) -> Vec<&Row> {
    rows.iter().filter(|r| r.armed).collect()
}

/// Yaw stick right (+0.5) for the first 6 s of flight, through the class.
/// Same measured closed-loop behaviour sitl_loop.rs documents for the
/// sim_run path (the runner is the same code): a CW burst ~100 deg within
/// ~1.5 s (props-in direction), then either a bounded ~4.3 Hz limit cycle
/// or a RUNAWAY_TAKEOFF disarm — varying run to run at the same seed, so
/// both are accepted. Asserts only what is common and stable.
#[test]
#[ignore = "needs tools/godot/bin/godot (pinned 4.7.2-stable), a built tools/gdext extension, and the Betaflight SITL binary; run: cargo test --test godot_flyer -- --ignored --test-threads=1"]
fn godot_flyer_closed_yaw_driven_through_class() {
    if !port_free_or_skip() {
        eprintln!("port 5761 busy — skipping");
        return;
    }
    let dir = run_probe("yaw");
    let (header, rows) = load_record(&dir);

    // Record-header contract: closed mode, the applied profile, the SITL
    // provenance.
    assert!(
        header.contains("\"mode\":\"closed\""),
        "header mode not closed: {header}"
    );
    assert!(
        header.contains("\"profile\":[\""),
        "profile not applied (header): {header}"
    );
    assert!(
        header.contains("\"sitl\":{\"path\":") && header.contains("Betaflight /"),
        "sitl provenance/version missing (header): {header}"
    );

    let armed = armed_rows(&rows);
    assert!(!armed.is_empty(), "never armed — record in {dir:?}");
    let a0 = armed[0].t;
    let yaw_enu_deg = |r: &Row| {
        (2.0 * (r.qw * r.qz + r.qx * r.qy))
            .atan2(1.0 - 2.0 * (r.qy * r.qy + r.qz * r.qz))
            .to_degrees()
    };
    let net_turn = |w: &[&Row]| -> f64 {
        let mut net = 0.0;
        for pair in w.windows(2) {
            net += wrap180(yaw_enu_deg(pair[1]) - yaw_enu_deg(pair[0]));
        }
        net
    };
    let burst: Vec<&Row> = armed
        .iter()
        .copied()
        .filter(|r| a0 <= r.t && r.t <= a0 + 1.5)
        .collect();
    let burst_turn = net_turn(&burst);
    assert!(
        burst_turn <= -30.0,
        "stick right did not burst CW: net {burst_turn:.1} deg in the first 1.5 s"
    );

    // Heading tracking (all armed rows): att_y = -yaw_enu + lag offset when
    // ground speed stays ~0.
    let mut offs = Vec::new();
    for r in &armed {
        offs.push(wrap180(r.att_y_deg - (-yaw_enu_deg(r))));
    }
    let mean_off = offs.iter().sum::<f64>() / offs.len() as f64;
    let max_off = offs.iter().fold(0.0f64, |m, o| m.max(o.abs()));
    assert!(
        mean_off.abs() <= 15.0,
        "att_y heading tracking: mean offset {mean_off:.1} deg, |max| {max_off:.1} deg (n={})",
        offs.len()
    );

    // Post-burst outcome, two accepted modes (see sitl_loop.rs).
    let post_burst: Vec<&Row> = armed
        .iter()
        .copied()
        .filter(|r| a0 + 1.5 <= r.t && r.t <= a0 + 6.0)
        .collect();
    const RUNAWAY_TAKEOFF: u32 = 1 << 5;
    let runaway_seen = rows
        .iter()
        .any(|r| r.t >= a0 && r.arm_disable & RUNAWAY_TAKEOFF != 0);
    let mode;
    if post_burst.len() >= 50 {
        let mut ys: Vec<f64> = post_burst.iter().copied().map(yaw_enu_deg).collect();
        ys.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let spread = ys.last().unwrap() - ys[0];
        assert!(
            spread <= 40.0,
            "bounded mode but yaw spread unbounded: {spread:.1} deg (n={})",
            post_burst.len()
        );
        mode = format!("bounded: spread {spread:.1} deg (n={})", post_burst.len());
    } else if runaway_seen {
        let disarm_t = rows
            .iter()
            .find(|r| r.t >= a0 && r.arm_disable & RUNAWAY_TAKEOFF != 0)
            .map(|r| r.t)
            .unwrap();
        mode = format!(
            "escalation: RUNAWAY_TAKEOFF disarm (bit first set t={disarm_t:.3}), \
             armed rows post-burst n={}",
            post_burst.len()
        );
    } else {
        panic!(
            "neither bounded (post-burst n={}) nor escalation (no RUNAWAY_TAKEOFF bit): \
             record in {dir:?}",
            post_burst.len()
        );
    }
    println!(
        "flyer yaw (class-driven): burst {burst_turn:.0} deg CW in 1.5 s, {mode}, \
         att_y-(-yaw_enu) mean {mean_off:+.1} |max| {max_off:.1} deg (n={})",
        armed.len()
    );
}