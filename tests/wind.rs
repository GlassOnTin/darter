//! T4 acceptance tests for the wind model (src/wind.rs).
//!
//! Two layers:
//! 1. The gust generator's spectrum, judged by a Welch PSD estimator (Goertzel
//!    per bin, Hann window, 50 non-overlapping 40 s segments) against the
//!    analytic MIL-F-8785C target S(f) = 2 sigma^2 L / V * (1+3x^2)/(1+x^2)^2,
//!    x = 2 pi f L / V. The estimator itself is pinned on unit-variance white
//!    noise first (flat at 2/fs per Hz).
//! 2. The wind -> physics -> record path end to end: spawned `sim_run` core
//!    runs, motors off, compared against the closed-form solution of the
//!    simulated per-axis quadratic drag. Per axis with air-relative velocity
//!    u = v - w the sim integrates du/dt = -c_i(z) u |u| with
//!    c_i(z) = 0.5 rho(z) cda_i / m, whose exact solution given the recorded
//!    trajectory z(t) is
//!    v_i(t) = w_i - (w_i - v0) / (1 + |w_i - v0| * I_i(t)),
//!    I_i(t) = integral of c_i(z(tau)) dtau.
//!    With v(0) = 0 (the spawn state) this is the approach form
//!    v_i(t) = w_i |w_i| I_i(t) / (1 + |w_i| I_i(t)).
//!    I_i is integrated over the record's own z(t) because density varies ~9%
//!    over the 800 m fall; the horizontal ODEs are then exact, not
//!    approximate (drag does not feed back into z except through vz, which is
//!    recorded).
//!
//! Deviations from the plan's wording, stated: the plan's "motors-off
//! steady-state drift speed equals the mean wind exactly" is unattainable —
//! quadratic drag decays the air-relative speed as 1/(c t), so the approach
//! to the mean is algebraic (0.34 m/s residual after 600 s), a powered hover
//! dies with the battery in ~360 s, and a landed craft is pinned by the
//! ground branch (0.9-per-substep horizontal damping). These tests assert the
//! sharp form that IS attainable: the exact finite-time solution above
//! (measured error < 0.001 m/s at 5/10/20/40 s, gate 0.03), the vertical
//! terminal-velocity fixed point (the true "steady state equals" assertion),
//! and bitwise-zero horizontal velocity in still air.
//!
//! PSD-gate deviation, found during T4: the plan's "Welch PSD matches the
//! Dryden target within +-20% in-band" is not attainable against the point
//! target with a 40 s Hann window. The horizontal spectrum at 100 ft is so
//! steep in the lowest bins (corner f_c = V/(2 pi L_h) = 0.010 Hz, S drops
//! ~40% per 0.025 Hz at 0.05 Hz) that the window's 0.05 Hz-wide main lobe
//! smears high-power low-frequency content into them: the EXPECTED estimator
//! is +10.5% above the point target integrated over 0.05..2 Hz (east), and
//! single realizations scatter +-0.07 around that. Established in design by
//! (a) the exact time-domain second-moment formula below and (b) a
//! 200-realization Monte Carlo of the same estimator (they agree to 0.4%
//! integrated). The test therefore gates against the exact expected
//! estimator, not the point target; the realization scatter is covered by
//! the stated gates, and the point-target ratio is printed as evidence.
//!
//! Live closed-loop runs (SITL) are out of scope here: every test is offline.

use std::path::PathBuf;
use std::process::Command;

use darter_core::air;
use darter_core::preset::Preset;
use darter_core::quad::G;
use darter_core::wind::{scales, WindConfig, WindModel};
use darter_core::DVec3;

use std::f64::consts::PI;

const TICK_DT: f64 = 4e-3; // sim_run's tick: 125 us x 32 substeps
const FS: f64 = 1.0 / TICK_DT; // 250 Hz

fn temp_dir(name: &str) -> PathBuf {
    let d = std::env::temp_dir().join(format!("darter-wind-{name}-{}", std::process::id()));
    std::fs::create_dir_all(&d).unwrap();
    d
}

/// Spawn the harness in core mode; returns stdout (the log evidence).
fn run_sim(dir: &std::path::Path, args: &[&str]) -> String {
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
    t: f64,
    pz: f64,
    vx: f64,
    vy: f64,
    vz: f64,
}

/// Parse the f64 value of `key` from one record line (fixed field order).
fn num(line: &str, key: &str) -> f64 {
    let pat = format!("\"{key}\":");
    let i = line.find(&pat).unwrap_or_else(|| panic!("field {key} missing in {line}"));
    let rest = &line[i + pat.len()..];
    let end = rest.find([',', '}']).expect("field terminator");
    rest[..end].parse().expect("float parse")
}

fn field_text<'a>(line: &'a str, key: &str) -> &'a str {
    let pat = format!("\"{key}\":");
    let i = line.find(&pat).expect("field");
    let rest = &line[i + pat.len()..];
    let end = rest.find([',', '}']).expect("field terminator");
    &rest[..end]
}

fn read_record(path: &std::path::Path) -> (Vec<Row>, Vec<String>) {
    let text = std::fs::read_to_string(path).expect("flight record");
    let mut rows = Vec::new();
    let mut raw = Vec::new();
    for line in text.lines() {
        if line.starts_with("{\"schema") {
            continue;
        }
        raw.push(line.to_string());
        rows.push(Row {
            t: num(line, "t"),
            pz: num(line, "pz"),
            vx: num(line, "vx"),
            vy: num(line, "vy"),
            vz: num(line, "vz"),
        });
    }
    (rows, raw)
}

// --- Welch machinery ------------------------------------------------------

/// Hann window (periodic) and its sum of squares.
fn hann(seg: usize) -> (Vec<f64>, f64) {
    let win: Vec<f64> = (0..seg)
        .map(|i| 0.5 * (1.0 - (2.0 * PI * i as f64 / seg as f64).cos()))
        .collect();
    let sum2 = win.iter().map(|w| w * w).sum();
    (win, sum2)
}

/// Modified periodogram of one windowed segment at frequency `f`:
/// P(f) = 2 |X|^2 / (fs * sum(w^2)). For unit-variance white noise this is
/// unbiased at 2/fs per Hz (E|X|^2 = fs * S(f) * sum(w^2), two-sided S/2
/// convention folded into the one-sided factor 2).
fn goertzel_power(x: &[f64], f: f64, fs: f64, win_sum2: f64) -> f64 {
    let w = 2.0 * PI * f / fs;
    let (mut s1, mut s2) = (0.0f64, 0.0f64);
    for &v in x {
        let s0 = v + 2.0 * w.cos() * s1 - s2;
        s2 = s1;
        s1 = s0;
    }
    let re = s1 - w.cos() * s2;
    let im = w.sin() * s2;
    2.0 * (re * re + im * im) / (fs * win_sum2)
}

/// Raw Welch PSD of `series` (Goertzel per bin, Hann, 50 non-overlapping
/// 40 s segments): the 36 log-spaced bins 0.05..2.0 Hz and the per-bin
/// averaged estimate.
fn welch_estimate(series: &[f64]) -> (Vec<f64>, Vec<f64>) {
    let seg = 10_000usize; // 40 s
    let n_seg = series.len() / seg;
    let (win, sw2) = hann(seg);
    let bins: Vec<f64> = (0..36)
        .map(|k| 0.05 * (40.0f64).powf(k as f64 / 35.0))
        .collect();
    let mut est = vec![0.0f64; bins.len()];
    for s in 0..n_seg {
        let xs: Vec<f64> = series[s * seg..(s + 1) * seg]
            .iter()
            .zip(&win)
            .map(|(x, w)| x * w)
            .collect();
        for (k, &f) in bins.iter().enumerate() {
            est[k] += goertzel_power(&xs, f, FS, sw2);
        }
    }
    for e in est.iter_mut() {
        *e /= n_seg as f64;
    }
    (bins, est)
}

/// Linear (non-circular) autocorrelation of the window: cw(m) =
/// sum_i win[i] win[i+m], m in 0..seg. Exactly zero for m >= seg, so the
/// expected-estimator sum below needs no truncation.
fn hann_autocorr(win: &[f64]) -> Vec<f64> {
    let seg = win.len();
    let mut cw = vec![0.0f64; seg];
    for m in 0..seg {
        let mut s = 0.0;
        for i in 0..seg - m {
            s += win[i] * win[i + m];
        }
        cw[m] = s;
    }
    cw
}

/// Autocovariance of the discrete Dryden cascade at lag m (m >= 0), closed
/// form over its impulse response h(n) = k (sqrt3 (1-e) e^n + (1-sqrt3)
/// (1-e)^2 (n+1) e^n) with q = e^2:
///   gamma(m) = k^2 e^m [ 3 (1-e)^2 / (1-q)
///     + sqrt3 (1-sqrt3) (1-e)^3 ( 2/(1-q)^2 + m/(1-q) )
///     + (1-sqrt3)^2 (1-e)^4 ( 1/(1-q) + 2q/(1-q)^2 + q(1+q)/(1-q)^3
///                            + m/(1-q)^2 ) ].
/// The cross term carries BOTH orderings h1*h2 + h2*h1 — they are not
/// equal and neither is the -m form of the other (a first-draft here got
/// this wrong and was caught by the assertion test below).
fn gamma_dryden(m: f64, sigma: f64, len: f64, v: f64) -> f64 {
    let e = (-TICK_DT / (len / v)).exp();
    let q = e * e;
    let a = 1.0 - e;
    let k = sigma * (len / (v * TICK_DT)).sqrt();
    let sqrt3 = 3.0f64.sqrt();
    k * k
        * e.powf(m)
        * (3.0 * a * a / (1.0 - q)
            + sqrt3 * (1.0 - sqrt3) * a * a * a
                * (2.0 / (1.0 - q).powi(2) + m / (1.0 - q))
            + (1.0 - sqrt3) * (1.0 - sqrt3) * a * a * a * a
                * (1.0 / (1.0 - q)
                    + 2.0 * q / (1.0 - q).powi(2)
                    + q * (1.0 + q) / (1.0 - q).powi(3)
                    + m / (1.0 - q).powi(2)))
}

/// Exact expected value of the Welch estimator at bin frequency `f` for one
/// gust channel with Dryden scales (sigma, len) and advection speed `v`:
///   E[P(f)] = 2/(fs sum w^2) * (gamma(0) sum w^2
///             + 2 sum_{m>=1} gamma(m) cw(m) cos(2 pi f m dt)),
/// from E|X|^2 = sum_{i,j} w_i w_j gamma(|i-j|) cos(2 pi f |i-j| dt). No
/// truncation enters (cw vanishes beyond seg). Validated in design against
/// a 200-realization Monte Carlo of this exact estimator: per-bin means
/// within 3%, integrated-band mean within 0.4%.
fn expected_welch(f: f64, sigma: f64, len: f64, v: f64, cw: &[f64], sw2: f64) -> f64 {
    let w = 2.0 * PI * f * TICK_DT;
    let mut acc = 0.0;
    for (m, &cwm) in cw.iter().enumerate().skip(1) {
        acc += gamma_dryden(m as f64, sigma, len, v) * cwm * (w * m as f64).cos();
    }
    2.0 / (FS * sw2) * (gamma_dryden(0.0, sigma, len, v) * sw2 + 2.0 * acc)
}

/// gamma_dryden's closed form asserted against the direct definition
/// sum_n h(n) h(n+m) of the impulse-response autocorrelation, for both the
/// horizontal and the vertical scales — the guard that keeps the comparator
/// pinned to the filter the lib actually implements.
#[test]
fn gamma_closed_form_matches_impulse_response() {
    let m_check = [0usize, 1, 7, 60, 500, 4000, 9999];
    for (sigma, len, tag) in [
        (2.6476f64, 153.93f64, "east"),
        (1.543, 30.48, "up"),
    ] {
        // Impulse response out to 40 tau: the largest checked lag (9999
        // samples = 40 s) must leave many correlation times of tail after it
        // or the direct sum's truncation error exceeds the tolerance.
        let steps = (40.0 * len / 10.0 / TICK_DT) as usize;
        let e = (-TICK_DT / (len / 10.0)).exp();
        let a = 1.0 - e;
        let k = sigma * (len / (10.0 * TICK_DT)).sqrt();
        let sqrt3 = 3.0f64.sqrt();
        let h: Vec<f64> = (0..steps)
            .map(|n| {
                let j = n as f64;
                k * (sqrt3 * a * e.powi(n as i32) + (1.0 - sqrt3) * a * a * (j + 1.0) * e.powi(n as i32))
            })
            .collect();
        for &m in &m_check {
            let mut direct = 0.0;
            for j in 0..steps - m {
                direct += h[j] * h[j + m];
            }
            let closed = gamma_dryden(m as f64, sigma, len, 10.0);
            let rel = (closed / direct - 1.0).abs();
            assert!(
                rel < 1e-4,
                "{tag} gamma({m}): closed {closed:.6e} vs direct {direct:.6e} (rel {rel:.2e})"
            );
        }
    }
}

/// The Welch estimator on unit-variance white noise: flat at 2/fs = 0.008
/// per Hz (mean over bins within 10%), with the expected per-bin scatter
/// (relative CV of a 50-segment average ~ 1/sqrt(50) = 0.14).
#[test]
fn welch_white_noise_sanity() {
    let seg = 10_000usize;
    let n = 500_000usize;
    // Minimal deterministic unit-variance white noise: LCG + Box-Muller.
    let mut state = 0x9e37_79b9_7f4a_7c15u64;
    let mut series = Vec::with_capacity(n);
    while series.len() < n {
        state = state
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        let u1 = ((state >> 11) as f64 + 0.5) * (1.0 / 9007199254740992.0);
        state = state
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        let u2 = ((state >> 11) as f64 + 0.5) * (1.0 / 9007199254740992.0);
        let r = (-2.0 * u1.ln()).sqrt();
        series.push(r * (2.0 * PI * u2).cos());
        series.push(r * (2.0 * PI * u2).sin());
    }
    let (win, sw2) = hann(seg);
    let n_seg = n / seg;
    let bins: Vec<f64> = (0..36)
        .map(|k| 0.05 * (40.0f64).powf(k as f64 / 35.0))
        .collect();
    let mut est = vec![0.0; bins.len()];
    for s in 0..n_seg {
        let xs: Vec<f64> = series[s * seg..(s + 1) * seg]
            .iter()
            .zip(&win)
            .map(|(x, w)| x * w)
            .collect();
        for (k, &f) in bins.iter().enumerate() {
            est[k] += goertzel_power(&xs, f, FS, sw2);
        }
    }
    for e in est.iter_mut() {
        *e /= n_seg as f64;
    }
    let flat = 2.0 / FS; // 0.008 per Hz
    let mean = est.iter().sum::<f64>() / est.len() as f64;
    assert!(
        (mean - flat).abs() / flat < 0.10,
        "white-noise PSD mean {mean:.6} vs flat {flat:.6}"
    );
    // Per-bin scatter around the flat level: CV of a 50-segment average.
    let cv = (est.iter().map(|e| (e - mean) * (e - mean)).sum::<f64>() / (est.len() - 1) as f64)
        .sqrt()
        / mean;
    assert!(
        (0.05..=0.30).contains(&cv),
        "per-bin CV {cv:.3} vs expected ~0.14"
    );
}

/// The east (lateral-spectrum) and up (vertical-spectrum) channels of a
/// 2000 s gust run match the EXACT expected Welch estimator (the point
/// Dryden target as the 40 s Hann window actually sees it, leakage
/// included): every per-bin est/expected ratio in [0.4, 2.5], integrated
/// band within 0.25 of 1.0. The scatter numbers behind those gates: the
/// integrated ratio's measured realization std is 0.073 over 100
/// realizations (so 0.25 is ~3.4 sigma); per-bin single-realization scatter
/// is wider still (chi-square over 50 segments plus low-f correlation).
/// Series is bit-deterministic (pinned seed), so these gates are fixed
/// numbers, not a distribution. The est/point-target integrated ratios are
/// printed as evidence of the leakage bias (module doc).
#[test]
fn dryden_psd_matches_target() {
    let cfg = WindConfig {
        mean: DVec3::new(10.0, 0.0, 0.0),
        w20_ms: 15.43,
        seed: 11,
    };
    // Pinned altitude 100 ft (the scales the lib tests pin too); V is the
    // 10 m/s mean (above the 1 m/s floor).
    let (l_h, l_w, s_h, s_w) = scales(100.0 * 0.3048, 15.43);
    let v = cfg.mean.length();
    let mut m = WindModel::new(cfg);
    let n = 2000 * 250;
    let mut east = Vec::with_capacity(n);
    let mut up = Vec::with_capacity(n);
    for _ in 0..n {
        let w = m.step(TICK_DT, 100.0 * 0.3048);
        east.push(w.x - cfg.mean.x);
        up.push(w.z - cfg.mean.z);
    }
    let (win, sw2) = hann(10_000);
    let cw = hann_autocorr(&win);
    let (bins, est_e) = welch_estimate(&east);
    let (_, est_u) = welch_estimate(&up);
    // Trapezoid band weights over the log-spaced bins.
    let df = |k: usize| {
        if k == 0 {
            bins[1] - bins[0]
        } else if k == bins.len() - 1 {
            bins[k] - bins[k - 1]
        } else {
            (bins[k + 1] - bins[k - 1]) / 2.0
        }
    };
    for (tag, est, sigma, l) in
        [("east", &est_e, s_h, l_h), ("up", &est_u, s_w, l_w)]
    {
        let (mut est_num, mut exp_den, mut pt_den) = (0.0, 0.0, 0.0);
        let mut ratios = Vec::with_capacity(bins.len());
        for (k, &f) in bins.iter().enumerate() {
            let expected = expected_welch(f, sigma, l, v, &cw, sw2);
            let r = est[k] / expected;
            assert!(
                (0.4..=2.5).contains(&r),
                "{tag} bin {k} at {f:.3} Hz: est/expected {r:.3}"
            );
            ratios.push(r);
            est_num += est[k] * df(k);
            exp_den += expected * df(k);
            let x = 2.0 * PI * f * l / v;
            pt_den += 2.0 * sigma * sigma * l / v * (1.0 + 3.0 * x * x)
                / (1.0 + x * x).powi(2) * df(k);
        }
        let (band, point) = (est_num / exp_den, est_num / pt_den);
        println!("{tag} integrated est/expected {band:.4}; est/point-target {point:.4}");
        println!(
            "{tag} per-bin est/expected: {:?}",
            ratios.iter().map(|r| (r * 1000.0).round() / 1000.0).collect::<Vec<_>>()
        );
        assert!(
            (band - 1.0).abs() < 0.25,
            "{tag} integrated est/expected {band:.3} vs 1.0"
        );
    }
}

/// Same seed -> bit-identical record (the harness's own --determinism-check),
/// different gust seed -> different record hash.
#[test]
fn wind_runs_deterministic_and_seed_sensitive() {
    let dir = temp_dir("det");
    let common = ["--seed", "42", "--duration", "10", "--wind=ex=10,w20=15.43,seed=7"];
    let out = run_sim(&dir.join("a"), &["--determinism-check", common[0], common[1], common[2], common[3]]);
    assert!(
        out.contains("determinism check passed"),
        "same-seed runs diverged: {out}"
    );
    run_sim(&dir.join("b"), &common);
    let hash = |sub: &str| {
        let text =
            std::fs::read_to_string(dir.join(sub).join("summary.json")).expect("summary.json");
        let i = text.find("\"record_hash\": \"0x").expect("record_hash in summary");
        text[i + 18..i + 34].to_string()
    };
    let ha = hash("a");
    let hb = hash("b");
    assert_ne!(ha, hb, "different gust seeds produced identical records");
    let _ = std::fs::remove_dir_all(&dir);
}

/// Motors-off fall in mean wind (8, 6, 0), gusts off: horizontal velocity
/// follows the closed-form solution of the simulated per-axis quadratic
/// drag, evaluated with the density integral over the record's own z(t),
/// within 0.03 m/s at 5/10/20/40 s (measured error < 0.001 m/s).
#[test]
fn motors_off_drift_matches_quadratic_drag() {
    let p = Preset::FREESTYLE_5IN;
    let dir = temp_dir("drift");
    run_sim(
        &dir,
        &[
            "--seed", "5",
            "--duration", "40",
            "--throttle", "0",
            "--alt", "1000",
            "--wind=ex=8,ny=6,w20=0",
        ],
    );
    let (rows, _) = read_record(&dir.join("flight.jsonl"));
    // c_i(alt) = 0.5 rho(alt) cda_i / m for the two horizontal axes.
    let c = |alt: f64, axis: usize| 0.5 * air::density(alt) * p.cda[axis] / p.mass_kg;
    const SAMPLES: [f64; 4] = [5.0, 10.0, 20.0, 40.0];
    let samples = SAMPLES;
    let w = [8.0f64, 6.0];
    let mut i_snap = [[0.0f64; 2]; SAMPLES.len()];
    let mut next = 0usize;
    let (mut ix, mut iy) = (0.0f64, 0.0f64);
    for i in 0..rows.len() {
        if i > 0 {
            let dt = rows[i].t - rows[i - 1].t;
            ix += (c(rows[i - 1].pz, 0) + c(rows[i].pz, 0)) / 2.0 * dt;
            iy += (c(rows[i - 1].pz, 1) + c(rows[i].pz, 1)) / 2.0 * dt;
        }
        if next < samples.len() && rows[i].t == samples[next] {
            i_snap[next] = [ix, iy];
            next += 1;
        }
    }
    assert_eq!(next, samples.len(), "sample rows missing from the record");
    for (k, &ts) in samples.iter().enumerate() {
        let row = rows.iter().find(|r| r.t == ts).unwrap();
        let meas = [row.vx, row.vy];
        for a in 0..2 {
            let wi = w[a];
            let expect = wi * wi.abs() * i_snap[k][a] / (1.0 + wi.abs() * i_snap[k][a]);
            let err = meas[a] - expect;
            println!(
                "drift t={ts:.0} axis {a}: measured {:.4} analytic {:.4} err {err:+.4} m/s",
                meas[a], expect
            );
            assert!(
                err.abs() < 0.03,
                "t={ts} axis {a}: measured {:.4} vs analytic {:.4}",
                meas[a], expect
            );
        }
    }
    let _ = std::fs::remove_dir_all(&dir);
}

/// Vertical fixed point of the same quadratic drag: at 10/20/40 s the fall
/// speed sits at the terminal velocity evaluated at the craft's own altitude
/// within 0.2 m/s (t=5 is excluded: the tanh relaxation still lags ~0.37).
#[test]
fn vertical_terminal_fixed_point() {
    let p = Preset::FREESTYLE_5IN;
    let dir = temp_dir("terminal");
    run_sim(
        &dir,
        &[
            "--seed", "5",
            "--duration", "40",
            "--throttle", "0",
            "--alt", "1000",
            "--wind=w20=0",
        ],
    );
    let (rows, _) = read_record(&dir.join("flight.jsonl"));
    for &ts in &[10.0f64, 20.0, 40.0] {
        let row = rows.iter().find(|r| r.t == ts).unwrap();
        let v_t = (2.0 * p.mass_kg * G / (air::density(row.pz) * p.cda[2])).sqrt();
        let err = row.vz + v_t;
        println!("terminal t={ts:.0}: vz {:.3} vs -{v_t:.3} (err {err:+.3})", row.vz);
        assert!(
            err.abs() < 0.2,
            "t={ts}: vz {:.3} vs terminal -{v_t:.3} at its own altitude",
            row.vz
        );
    }
    let _ = std::fs::remove_dir_all(&dir);
}

/// The air-relative velocity really is air-relative: in a steady 3 m/s
/// updraft the fall speed is terminal minus 3, within 0.25 m/s at t=10.
#[test]
fn updraft_offsets_fall_speed() {
    let p = Preset::FREESTYLE_5IN;
    let dir = temp_dir("updraft");
    run_sim(
        &dir,
        &[
            "--seed", "5",
            "--duration", "40",
            "--throttle", "0",
            "--alt", "1000",
            "--wind=uz=3,w20=0",
        ],
    );
    let (rows, _) = read_record(&dir.join("flight.jsonl"));
    let row = rows.iter().find(|r| r.t == 10.0).unwrap();
    let v_t = (2.0 * p.mass_kg * G / (air::density(row.pz) * p.cda[2])).sqrt();
    let expect = 3.0 - v_t;
    let err = row.vz - expect;
    println!(
        "updraft t=10: vz {:.3} vs terminal-3 {expect:.3} (err {err:+.3})",
        row.vz
    );
    assert!(
        err.abs() < 0.25,
        "vz {:.3} vs {expect:.3} (terminal - updraft)",
        row.vz
    );
    let _ = std::fs::remove_dir_all(&dir);
}

/// Gusts off (w20=0) with the wind path active: horizontal velocity stays
/// exactly zero in every recorded row — text-exact, so even a -0.0 would
/// fail. Covers the ground-contact damping branch too (the craft lands at
/// ~1.4 s and the x/y velocities must remain zero through it).
#[test]
fn zero_wind_is_exact_still_air() {
    let dir = temp_dir("zerowind");
    run_sim(
        &dir,
        &[
            "--seed", "5",
            "--duration", "5",
            "--throttle", "0",
            "--alt", "10",
            "--wind=w20=0",
        ],
    );
    let text = std::fs::read_to_string(dir.join("flight.jsonl")).unwrap();
    let mut n_rows = 0usize;
    for line in text.lines() {
        if line.starts_with("{\"schema") {
            continue;
        }
        n_rows += 1;
        assert_eq!(field_text(line, "vx"), "0.000000", "row: {line}");
        assert_eq!(field_text(line, "vy"), "0.000000", "row: {line}");
    }
    assert!(n_rows > 1000, "expected a full 5 s record, got {n_rows} rows");
    let _ = std::fs::remove_dir_all(&dir);
}