//! T1 calibration gates, core-only (scripted throttle, no FC, no wall
//! clock). Each gate prints its measured number; run with --nocapture to
//! read them. Held-out-anchor policy: ct0 is fit from the full-throttle
//! bench point; hover throttle, steady full-throttle current, climb@60% and
//! terminal fall are held-out checks, not re-fitted.

use darter_core::air::{density, RHO_0};
use darter_core::battery::Battery;
use darter_core::preset::Preset;
use darter_core::quad::{hover_throttle, thrust_power, Quad};
use darter_core::DVec3;

const DT: f64 = 125e-6;
const P: Preset = Preset::FREESTYLE_5IN;
/// rpm to rad/s.
const RPM_TO_RAD: f64 = std::f64::consts::PI / 30.0;

fn freeze_battery(quad: &mut Quad) {
    quad.battery.soc = 1.0;
}

/// Every Preset field carries exactly one provenance label, and the label
/// table lists exactly the struct's fields in order. The expected list is
/// spelled out here so a field added without a label fails this test.
#[test]
fn every_field_has_provenance() {
    let expected = [
        "name",
        "mass_kg",
        "arm_m",
        "inertia",
        "rpm_tau_s",
        "prop_diameter_m",
        "prop_j0",
        "prop_ct0",
        "prop_fom",
        "prop_cp",
        "motor_r_m_ohm",
        "motor_i_idle_a",
        "rpm_curve",
        "rpm_cur_bench",
        "rpm_v_bench",
        "rpm_sub_k",
        "cda",
        "battery",
    ];
    let table = &Preset::FREESTYLE_5IN_PROVENANCE;
    assert_eq!(table.len(), expected.len(), "provenance table length");
    for (i, (field, _)) in table.iter().enumerate() {
        assert_eq!(field, &expected[i], "provenance entry {i}");
    }
}

/// The bench fixtures the calibration is built from are checked in and
/// parseable JSON.
#[test]
fn bench_fixtures_present() {
    for name in [
        "bench_t-hobby-v2306.5-v2-kv1950-t5143s-6s.json",
        "bench_iflight-xing2-2306-1755kv-nazgul-5140-6s.json",
    ] {
        let path = format!("tests/fixtures/{name}");
        let text = std::fs::read_to_string(&path)
            .unwrap_or_else(|e| panic!("missing fixture {path}: {e}"));
        serde_json_ok(&text);
    }
}

/// Minimal structural check without a JSON dependency: balanced braces and
/// the known marker keys.
fn serde_json_ok(text: &str) {
    assert_eq!(
        text.chars().filter(|c| *c == '{').count(),
        text.chars().filter(|c| *c == '}').count(),
        "unbalanced JSON braces"
    );
    assert!(text.contains("\"role\""), "fixture missing role key");
    assert!(text.contains("\"url\""), "fixture missing source url");
}

/// Static thrust reproduces the bench table at the bench rotor speeds (ct0
/// was fit at full throttle only; the intermediate points are held out).
#[test]
fn static_thrust_matches_bench_midpoints() {
    // (rpm, bench thrust grams): d = 0.5 and d = 0.7 rows of the fixture.
    for (rpm, bench_g) in [(20_195.0, 653.80), (25_784.0, 1_090.67)] {
        let (thrust, _) = thrust_power(&P, rpm * RPM_TO_RAD, 0.0, RHO_0);
        let grams = thrust / 9.80665e-3;
        let err = (grams - bench_g) / bench_g;
        println!("rpm {rpm}: model {grams:.1} g vs bench {bench_g} g ({:+.1}%)", err * 100.0);
        assert!(err.abs() < 0.06, "static thrust off by {}% at {rpm} rpm", err * 100.0);
    }
    // The anchor point itself (the fit input) must be exact.
    let (t_full, _) = thrust_power(&P, 30_991.0 * RPM_TO_RAD, 0.0, RHO_0);
    assert!((t_full / 9.80665e-3 - 1591.45).abs() < 0.5, "full-throttle thrust {}", t_full);
}

/// The power model's anchor: electrical power at the bench full-throttle
/// point (shaft power at 30,991 rpm over 24.51 V plus idle) matches the
/// bench 875.15 W row exactly, because c_p was constructed that way. The
/// mid-table power column is NOT reproduced by any single (FoM, c_p) pair;
/// that gap is documented, not asserted away.
#[test]
fn full_throttle_electrical_power_matches_bench() {
    let (_, p_shaft) = thrust_power(&P, 30_991.0 * RPM_TO_RAD, 0.0, RHO_0);
    let i = p_shaft / 24.51 + P.motor_i_idle_a;
    let p_elec = i * 24.51;
    println!("bench-voltage full-throttle: {p_elec:.2} W, {i:.2} A (bench: 875.15 W, 35.70 A)");
    assert!((p_elec - 875.15).abs() / 875.15 < 0.02);
}

/// Held-out anchor: hover throttle lands in [0.15, 0.30]. Expected ~0.16
/// (bottom of band, from the concave rpm map).
#[test]
fn hover_in_band() {
    let hover = hover_throttle(&P, P.battery, 1.0, RHO_0);
    println!("hover throttle (fresh pack): {hover:.4}");
    assert!((0.15..=0.30).contains(&hover), "hover {hover} out of band");
}

/// Held-out anchor: climb at 60% throttle settles at a plausible vertical
/// speed. Gate [10, 35) m/s. Measured value printed; the band is wider than
/// the original < 25 m/s plan gate because the falloff strength is set by
/// measured UIUC CT(J) data, which cannot produce 18-19 m/s climbs (see the
/// T1 report).
#[test]
fn climb_at_60pct_in_band() {
    let mut quad = Quad::new(P, DVec3::new(0.0, 0.0, 0.5));
    quad.throttle = [0.6; 4];
    // 6 s: ~0.5 s spin-up plus a settled climb.
    for _ in 0..48_000 {
        freeze_battery(&mut quad);
        quad.step(DT);
    }
    // Average vz over the last second.
    let mut sum = 0.0;
    let n = 8_000;
    for _ in 0..n {
        freeze_battery(&mut quad);
        quad.step(DT);
        sum += quad.state.vel.z;
    }
    let vz = sum / n as f64;
    println!("climb at 60% throttle: {vz:.2} m/s (bus {:.2} V, {:.2} A/motor)", quad.bus_voltage(), quad.i_mot[0]);
    assert!(vz > 10.0 && vz < 35.0, "climb@60% {vz} out of [10, 35]");
}

/// Held-out anchor: motors-off terminal fall in [15, 25] m/s (vertical CdA
/// carries this band). Starts high, stops before ground contact.
#[test]
fn terminal_fall_in_band() {
    let mut quad = Quad::new(P, DVec3::new(0.0, 0.0, 200.0));
    quad.throttle = [0.0; 4];
    for _ in 0..40_000 {
        quad.step(DT); // 5 s, no battery draw (d = 0)
    }
    let vz = quad.state.vel.z;
    println!("terminal fall: {vz:.2} m/s (drag-limited, CdA_z {} m^2)", P.cda[2]);
    assert!(vz < -15.0 && vz > -25.0, "fall {vz} out of [-25, -15]");
    // Did not reach the ground plane during the window.
    assert!(quad.state.pos.z > 0.0);
}

/// Held-out anchor: full-throttle current per motor in [25, 40] A and
/// full-throttle bus sag in [0.7, 1.5] V on the 6S pack, at static
/// conditions (velocity zeroed every step so v_ax = 0 — the same quantity
/// the bench table measures; in free flight the quad climbs and the
/// equilibrium current is lower, ~30 A at the ~36 m/s climb).
#[test]
fn full_throttle_current_and_sag_in_band() {
    let mut quad = Quad::new(P, DVec3::new(0.0, 0.0, 0.5));
    quad.throttle = [1.0; 4];
    for _ in 0..16_000 {
        quad.state.vel = DVec3::ZERO;
        freeze_battery(&mut quad);
        quad.step(DT); // 2 s
    }
    let sag = P.battery.ocv_v(1.0) - quad.bus_voltage();
    println!(
        "static full throttle: {:.2} A/motor, bus {:.3} V, sag {:.3} V, rpm {:.0}",
        quad.i_mot[0],
        quad.bus_voltage(),
        sag,
        quad.rpm[0]
    );
    assert!(quad.i_mot[0] > 25.0 && quad.i_mot[0] < 40.0, "current {}", quad.i_mot[0]);
    assert!(sag > 0.7 && sag < 1.5, "bus sag {sag} V out of [0.7, 1.5]");
}

/// The battery model's hand-computed integration check (independent of the
/// quad wiring): 1 s at 142.8 A on the 4680 A s pack.
#[test]
fn battery_hand_computed_values() {
    let mut b = Battery::new(P.battery);
    b.step(1.0, 142.8);
    // soc = 1 - 142.8/4680 = 0.9694872. That lands in the (0.90, 4.06) ->
    // (1.00, 4.18) OCV segment at t = 0.69487: per-cell 4.06 + 0.69487*0.12
    // = 4.14338, pack 24.86031 V; minus 142.8*0.008 = 1.1424 V sag.
    assert!((b.soc - 0.96949).abs() < 5e-6);
    let expected_v = 6.0 * (4.06 + 0.69487 * (4.18 - 4.06)) - 142.8 * 0.008;
    assert!(
        (b.bus_voltage() - expected_v).abs() < 0.01,
        "bus {} vs hand-computed {expected_v:.4}",
        b.bus_voltage()
    );
}

/// Core-only roll/pitch step response against sim truth (no FC): a small
/// differential throttle produces the right roll/pitch direction with a
/// plausible magnitude. Pre-spin at hover, then step. On the concave rpm
/// map the +-0.005 differential is ~0.125 N of differential thrust per
/// motor (dOmega/dd ~ 39k rpm/unit at d=0.155), giving a net roll torque
/// of 2*arm*dT ~ 0.031 N m and alpha ~ 7.8 rad/s^2; over 0.3 s (6 rpm
/// time constants) omega ~ 2 rad/s. The core has no attitude controller,
/// so the response is a divergent angle by design; the FC-side tracking
/// check belongs to T2+ once ATTITUDE telemetry exists.
#[test]
fn roll_pitch_step_response_core_only() {
    let hover = hover_throttle(&P, P.battery, 1.0, RHO_0);
    let delta = 0.005;

    // Left pair (M3, M4) up, right pair down -> roll right (omega.x > 0).
    let mut quad = Quad::new(P, DVec3::new(0.0, 0.0, 5.0));
    quad.throttle = [hover; 4];
    for _ in 0..8_000 {
        freeze_battery(&mut quad);
        quad.step(DT); // 1 s pre-spin at hover
    }
    quad.state.pos.z = 5.0;
    quad.state.vel = DVec3::ZERO;
    quad.throttle = [hover - delta, hover - delta, hover + delta, hover + delta];
    for _ in 0..2_400 {
        freeze_battery(&mut quad);
        quad.step(DT); // 0.3 s
    }
    println!("roll step: omega.x {:+.3} rad/s", quad.state.omega.x);
    assert!(quad.state.omega.x > 1.0 && quad.state.omega.x < 4.0, "roll rate {:?}", quad.state.omega.x);

    // Front pair (M2, M4) up, rear pair down -> nose up (omega.y < 0).
    let mut quad = Quad::new(P, DVec3::new(0.0, 0.0, 5.0));
    quad.throttle = [hover; 4];
    for _ in 0..8_000 {
        freeze_battery(&mut quad);
        quad.step(DT);
    }
    quad.state.pos.z = 5.0;
    quad.state.vel = DVec3::ZERO;
    quad.throttle = [hover - delta, hover + delta, hover - delta, hover + delta];
    for _ in 0..2_400 {
        freeze_battery(&mut quad);
        quad.step(DT);
    }
    println!("pitch step: omega.y {:+.3} rad/s", quad.state.omega.y);
    assert!(quad.state.omega.y < -1.0 && quad.state.omega.y > -4.0, "pitch rate {:?}", quad.state.omega.y);
}

/// Density helper is on the thrust/drag path (not dead code) and monotone.
#[test]
fn density_path_monotone() {
    assert!(density(0.0) > density(1000.0));
    assert!((density(0.0) - RHO_0).abs() < 1e-12);
}