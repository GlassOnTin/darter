//! Aircraft presets: the physical data the physics core runs against.
//!
//! Every field carries a provenance label in the matching `*_PROVENANCE`
//! table (`Measured` / `Estimated` / `Derived`), asserted by
//! `tests/calibration.rs::every_field_has_provenance`. Numbers come from the
//! bench fixture `tests/fixtures/bench_t-hobby-v2306.5-v2-kv1950-t5143s-6s.json`
//! (URL + access date recorded there); fits made from that table are marked
//! `Derived`, judgment calls `Estimated`, manufacturer spec-sheet values
//! `Measured`.
//!
//! All SI. Callers must surface which is which (VISION: where a value is
//! estimated the UI says so).
use crate::battery::LIPO_6S_1300;
use crate::battery::BatterySpec;

/// Where a preset value came from.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Provenance {
    /// Manufacturer spec or bench measurement.
    Measured,
    /// Judgment call, bounded where possible.
    Estimated,
    /// Fit from recorded bench data through a documented model.
    Derived,
}

#[derive(Clone, Copy, Debug)]
pub struct Preset {
    pub name: &'static str,
    /// All-up weight (kg).
    pub mass_kg: f64,
    /// Motor-to-CG distance in the body XY plane (m).
    pub arm_m: f64,
    /// Diagonal inertia about body x, y, z (kg m^2).
    pub inertia: [f64; 3],
    /// First-order rotor lag time constant (s).
    pub rpm_tau_s: f64,

    // --- Prop: T-HOBBY T5143S, 5.1x4.3 tri-blade ---
    /// Prop disc diameter (m).
    pub prop_diameter_m: f64,
    /// Zero-thrust advance ratio of the axial thrust-falloff law
    /// `T = T0 * (1 - J/J0)`, J = axial airspeed / (rev_s * D). Taken as the
    /// geometric pitch/diameter ratio; the UIUC 5x3.75 tri-blade polar
    /// (fixture) measures zero-thrust at J ~= 0.77 vs geometric 0.75, so
    /// pitch/diameter is the right order.
    pub prop_j0: f64,
    /// Static thrust coefficient: T0 = ct0 * rho * D^4 * Omega^2 (N, rad/s).
    /// Fit from the full-throttle bench point; intermediate bench thrusts
    /// then follow within ~3.5%.
    pub prop_ct0: f64,
    /// Rotor figure of merit: shaft power = ideal induced power / FoM.
    pub prop_fom: f64,
    /// Profile-drag power coefficient: P_profile = c_p * rho * D^5 * Omega^3.
    /// Fit so the full-throttle electrical current matches the bench table.
    pub prop_cp: f64,

    // --- Motor: T-HOBBY Velox V2306.5 V2, KV1950, 6S ---
    /// Motor terminal resistance (ohm), spec sheet.
    pub motor_r_m_ohm: f64,
    /// ESC idle draw per motor (A) at operating voltage, spec sheet.
    pub motor_i_idle_a: f64,
    /// Bench (throttle, rpm) points at the recorded bench voltage; the
    /// quasi-static rotor-speed law. Piecewise-linear on [first..last],
    /// power law Omega = rpm0 * (d/d0)^k_sub below the first point.
    pub rpm_curve: &'static [(f64, f64)],
    /// Bench current (A) at each rpm_curve point: the sag normalisation.
    pub rpm_cur_bench: &'static [f64],
    /// Bench bus voltage (V) at each rpm_curve point.
    pub rpm_v_bench: &'static [f64],
    /// Power-law exponent of the sub-throttle rpm extension (below the first
    /// bench point, which starts at d = 0.5): Omega ~ d^k_sub.
    pub rpm_sub_k: f64,

    // --- Drag: per-axis quadratic on air-relative velocity ---
    /// CdA per body axis (m^2): F_i = -0.5 * rho * CdA_i * |v_i| * v_i.
    pub cda: [f64; 3],

    /// Battery spec flown with this preset.
    pub battery: BatterySpec,
}

impl Preset {
    /// 5" freestyle quad, 6S. Calibrated against the T-HOBBY V2306.5 V2
    /// KV1950 + T5143S bench table (see fixture); the iFlight XING2 +
    /// Nazgul 5140 table (second fixture) is the independent cross-check on
    /// max thrust and current.
    pub const FREESTYLE_5IN: Preset = Preset {
        name: "5in-freestyle-6s",
        mass_kg: 0.650,
        arm_m: 0.125,
        inertia: [0.004, 0.004, 0.009],
        rpm_tau_s: 0.05,

        prop_diameter_m: 0.12954, // 5.1 in
        prop_j0: 4.3 / 5.1,       // geometric pitch/diameter
        prop_ct0: 4.296e-3,
        prop_fom: 0.70,
        prop_cp: 2.315e-4,

        motor_r_m_ohm: 0.067,
        motor_i_idle_a: 1.28,
        rpm_curve: &[
            (0.50, 20_195.0),
            (0.55, 21_835.0),
            (0.60, 23_313.0),
            (0.65, 24_650.0),
            (0.70, 25_784.0),
            (0.75, 26_791.0),
            (0.80, 27_651.0),
            (0.85, 28_604.0),
            (0.90, 29_593.0),
            (0.95, 30_160.0),
            (1.00, 30_991.0),
        ],
        rpm_cur_bench: &[7.51, 9.80, 12.26, 14.90, 17.46, 20.10, 22.76, 25.90, 28.93, 32.27, 35.70],
        rpm_v_bench: &[25.05, 25.06, 25.01, 24.95, 24.90, 24.84, 24.78, 24.71, 24.65, 24.58, 24.51],
        rpm_sub_k: 0.6179,

        cda: [0.005, 0.005, 0.026],

        battery: LIPO_6S_1300,
    };

    /// Provenance of every field above, machine-checked by
    /// `tests/calibration.rs::every_field_has_provenance`. Keep in sync with
    /// the struct: one entry per field, same order.
    pub const FREESTYLE_5IN_PROVENANCE: [(&str, Provenance); 18] = [
        ("name", Provenance::Measured),
        ("mass_kg", Provenance::Estimated),
        ("arm_m", Provenance::Estimated),
        ("inertia", Provenance::Estimated),
        ("rpm_tau_s", Provenance::Estimated),
        ("prop_diameter_m", Provenance::Measured),
        ("prop_j0", Provenance::Estimated),
        ("prop_ct0", Provenance::Derived),
        ("prop_fom", Provenance::Estimated),
        ("prop_cp", Provenance::Derived),
        ("motor_r_m_ohm", Provenance::Measured),
        ("motor_i_idle_a", Provenance::Measured),
        ("rpm_curve", Provenance::Measured),
        ("rpm_cur_bench", Provenance::Measured),
        ("rpm_v_bench", Provenance::Measured),
        ("rpm_sub_k", Provenance::Derived),
        ("cda", Provenance::Derived),
        ("battery", Provenance::Estimated),
    ];
}