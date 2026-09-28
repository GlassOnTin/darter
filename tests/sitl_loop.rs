//! Closed-loop SITL behaviour tests — opt-in
//! (`cargo test --test sitl_loop -- --ignored --test-threads=1`; serial is
//! required: every run spawns its own SITL child bound to the fixed ports
//! TCP 5761 / UDP 9002-9004, so parallel tests kill each other), driving
//! `sim_run --mode closed` as a child process and asserting on the flight
//! record: truth quaternion/omega from the physics core vs the FC's MSP
//! telemetry. Unlike tests/msp_live.rs these verify behaviour, not wiring.
//!
//! - Yaw: stick right bursts the craft CW viewed from above (props-in quad)
//!   and the FC's MSP 108 yaw estimate keeps tracking the truth heading
//!   (offset <= 15 deg mean over the armed span). Measured gap, documented
//!   not asserted away: with the default profile the yaw loop either
//!   limit-cycles about the burst heading (~4.3 Hz, motors slamming between
//!   mixer extremes; same shape at hover throttle, 0.5 throttle, and stick
//!   0.2) or escalates from the cycle into a sustained spin, which the FC
//!   disarms via its RUNAWAY_TAKEOFF protection ~1.4 s after arming.
//!   Which of the two happens varies run to run at identical seed (the SITL
//!   PID loop runs on wall-clock time, so its dt carries UDP jitter); the
//!   test accepts both as closed-loop outcomes. This test is the behaviour
//!   evidence behind the FLU-polarity yaw choice in src/sitl.rs
//!   (fdm_from_state_imu).
//! - Level hover with the sensor model on: the FC's roll/pitch estimates stay
//!   within ±2 deg of truth. Yaw heading is excluded on purpose — MSP 108 yaw
//!   is a gyro integration with no mag fusion on this build (see the
//!   Attitude doc in src/msp.rs for the measured limits).

use std::path::PathBuf;
use std::process::Command;

const DURATION_S: f64 = 13.0;

/// One sampled row of flight.jsonl, restricted to the fields these tests use.
/// Only rows carrying FC telemetry are loaded (fresh runs' first rows omit
/// the fc fields by design).
struct Row {
    t: f64,
    qw: f64,
    qx: f64,
    qy: f64,
    qz: f64,
    att_r_deg: f64,
    att_p_deg: f64,
    att_y_deg: f64,
    armed: bool,
    /// The FC's arming_disable bitfield (runtime_config.h), NOT an armed
    /// boolean — armed detection is flags & 1 below.
    arm_disable: u32,
}

/// The record is hand-formatted fixed-order JSON; pull one number by exact
/// key. Panic on a missing field — these tests run against our own writer.
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

fn load_record(out_dir: &PathBuf) -> Vec<Row> {
    let path = out_dir.join("flight.jsonl");
    let text = std::fs::read_to_string(&path)
        .unwrap_or_else(|e| panic!("record {path:?}: {e}"));
    text.lines()
        .filter(|l| l.starts_with("{\"t\"") && l.contains("\"att_r\""))
        .map(|l| Row {
            t: fnum(l, "t"),
            qw: fnum(l, "qw"),
            qx: fnum(l, "qx"),
            qy: fnum(l, "qy"),
            qz: fnum(l, "qz"),
            att_r_deg: fnum(l, "att_r") / 10.0,
            att_p_deg: fnum(l, "att_p") / 10.0,
            att_y_deg: fnum(l, "att_y"),
            armed: fnum(l, "flags") as u32 & 1 == 1,
            arm_disable: fnum(l, "arm") as u32,
        })
        .collect()
}

/// sim_run closed mode as a child; fails the test on a nonzero exit.
fn run_sim(tag: &str, extra: &[&str]) -> PathBuf {
    let out_dir =
        std::env::temp_dir().join(format!("darter-sitlloop-{tag}-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&out_dir);
    let out = Command::new(env!("CARGO_BIN_EXE_sim_run"))
        .args([
            "--mode",
            "closed",
            "--duration",
            &format!("{DURATION_S}"),
            "--out",
            out_dir.to_str().unwrap(),
        ])
        .args(extra)
        .output()
        .expect("spawn sim_run");
    assert!(
        out.status.success(),
        "sim_run failed: {}\nstdout:\n{}\nstderr:\n{}",
        out.status,
        String::from_utf8_lossy(&out.stdout),
        String::from_utf8_lossy(&out.stderr)
    );
    out_dir
}

/// The fixed SITL port must be free before spawning (a leftover SITL would
/// make the child fail confusingly). Best-effort probe.
fn port_free_or_skip() -> bool {
    match std::net::TcpListener::bind("127.0.0.1:5761") {
        Ok(_) => true,
        Err(_) => false,
    }
}

fn wrap180(d: f64) -> f64 {
    (d + 180.0).rem_euclid(360.0) - 180.0
}

fn truth_roll_pitch_deg(r: &Row) -> (f64, f64) {
    let roll = (2.0 * (r.qw * r.qx + r.qy * r.qz))
        .atan2(1.0 - 2.0 * (r.qx * r.qx + r.qy * r.qy))
        .to_degrees();
    let sinp = (2.0 * (r.qw * r.qy - r.qz * r.qx)).clamp(-1.0, 1.0);
    (roll, sinp.asin().to_degrees())
}

fn armed_rows(rows: &[Row]) -> Vec<&Row> {
    rows.iter().filter(|r| r.armed).collect()
}

/// Yaw stick right (+0.5) for the first 6 s of flight. Measured closed-loop
/// behaviour with the default profile (seed 5, hover throttle; same shape
/// verified at 0.5 throttle and at stick 0.2): the craft BURSTS CW ~100 deg
/// within ~1.5 s (props-in direction, correct; net −245..−246 deg over the
/// full armed span in both observed runs), then one of two outcomes follows
/// — a documented yaw-chain fidelity gap, not asserted away:
///   - bounded: the loop limit-cycles about the burst heading (~4.3 Hz,
///     motors slamming between mixer extremes 0.05-0.81), or
///   - escalation: the cycle grows into a sustained spin and the FC disarms
///     via RUNAWAY_TAKEOFF (~1.4 s after arm; arming_disable gains bit 0x20
///     alongside THROTTLE/ARM_SWITCH; motors zeroed while armed).
/// Which one occurs varies run to run at the same seed (the SITL PID loop
/// runs on wall-clock time, so its dt carries UDP jitter) — the test accepts
/// both and asserts only what is common and stable: the CW burst, heading
/// tracking, and the post-burst outcome (bounded spread <= 40 deg, or the
/// runaway-takeoff disarm actually appearing in the record).
/// Per-interval rate comparison is NOT meaningful at 25 Hz telemetry against
/// a ~4.3 Hz cycle (measured correlation −0.3 — 25 Hz sampling plus the ~20
/// ms truth/telemetry row-time skew aliasing the cycle, not a tracking
/// failure); the heading-angle agreement is the valid form.
#[test]
#[ignore]
fn closed_yaw_stick_right_bursts_cw_estimate_tracks_heading() {
    if !port_free_or_skip() {
        eprintln!("port 5761 busy — skipping");
        return;
    }
    let dir = run_sim(
        "yaw",
        &["--seed", "5", "--yaw", "0.5", "--yaw-until", "6.0"],
    );
    let rows = load_record(&dir);
    let armed = armed_rows(&rows);
    assert!(!armed.is_empty(), "never armed — record in {dir:?}");
    let a0 = armed[0].t;
    let yaw_enu_deg = |r: &Row| {
        (2.0 * (r.qw * r.qz + r.qx * r.qy))
            .atan2(1.0 - 2.0 * (r.qy * r.qy + r.qz * r.qz))
            .to_degrees()
    };
    // Net turn from a series' start via cumulative wrapped deltas (no unwrap).
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

    // Estimate tracking over ALL armed rows (not a window — the escalation
    // mode's armed span is short). Under no COG snap (gs stays ~0 in these
    // runs) the internal reference makes att_y = -yaw_enu + (lag offset).
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

    // Post-burst outcome, two accepted modes:
    // - bounded: enough armed rows remain to measure a cycle (n >= 50) and
    //   the heading spread stays <= 40 deg (measured 18.2, 26.6).
    // - escalation: the yaw cycle grew until RUNAWAY_TAKEOFF disarmed the
    //   craft — the arming_disable bitfield must show it (bit 1<<5).
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
        mode = format!(
            "bounded: spread {spread:.1} deg (n={})",
            post_burst.len()
        );
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
        "yaw: burst {burst_turn:.0} deg CW in 1.5 s, {mode}, \
         att_y-(-yaw_enu) mean {mean_off:+.1} |max| {max_off:.1} deg (n={})",
        armed.len()
    );
}

/// Level hover with the sensor model on: the FC's Mahony roll/pitch estimates
/// stay within ±2 deg mean of truth (|max| jitter allowed to 3.5 deg — the
/// acc-belief term jitters with gyro noise). Level flight only; yaw heading
/// is excluded (see module doc).
#[test]
#[ignore]
fn closed_level_hover_roll_pitch_estimate_within_2deg_with_noise() {
    if !port_free_or_skip() {
        eprintln!("port 5761 busy — skipping");
        return;
    }
    let dir = run_sim("hover", &["--seed", "9", "--sensors"]);
    let rows = load_record(&dir);
    let armed = armed_rows(&rows);
    assert!(!armed.is_empty(), "never armed — record in {dir:?}");
    let a0 = armed[0].t;
    let t_end = rows.last().unwrap().t;
    let window: Vec<&Row> = armed
        .iter()
        .copied()
        .filter(|r| a0 + 2.0 <= r.t && r.t <= t_end.min(a0 + 7.0))
        .collect();
    assert!(window.len() > 50, "hover window too short: {}", window.len());

    let mut errs_r = Vec::new();
    let mut errs_p = Vec::new();
    for r in &window {
        let (roll_t, pitch_t) = truth_roll_pitch_deg(r);
        errs_r.push(wrap180(r.att_r_deg - roll_t));
        errs_p.push(wrap180(r.att_p_deg - pitch_t));
    }
    let mean = |v: &[f64]| v.iter().sum::<f64>() / v.len() as f64;
    let abs_mean = |v: &[f64]| mean(&v.iter().map(|x| x.abs()).collect::<Vec<_>>());
    let abs_max = |v: &[f64]| v.iter().fold(0.0f64, |m, x| m.max(x.abs()));
    let (mr, mp) = (abs_mean(&errs_r), abs_mean(&errs_p));
    let (xr, xp) = (abs_max(&errs_r), abs_max(&errs_p));
    println!(
        "hover: roll |err| mean {mr:.2} |max| {xr:.2} deg; pitch |err| mean {mp:.2} |max| {xp:.2} deg (n={})",
        window.len()
    );
    assert!(mr <= 2.0, "roll estimate mean |err| {mr:.2} deg > 2");
    assert!(mp <= 2.0, "pitch estimate mean |err| {mp:.2} deg > 2");
    assert!(xr <= 3.5, "roll estimate |max| {xr:.2} deg > 3.5");
    assert!(xp <= 3.5, "pitch estimate |max| {xp:.2} deg > 3.5");
}