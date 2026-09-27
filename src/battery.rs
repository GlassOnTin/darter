//! Bus battery model: open-circuit voltage vs state of charge, pack
//! resistance sag, coulomb-counted discharge.
//!
//! `SoC' = -I_bus*dt / Q_As`, `V_bus = V_ocv(SoC) - I_bus * R_pack`.
//! Deterministic: everything is an explicit f64 update, no hidden state.
//!
//! This models the pack only. The FC never sees it on the SITL bridge
//! (no virtual battery sensor exists upstream) — the FC-side vbat path is
//! an untestable upstream gap; this model feeds the physics (motor sag)
//! and the sim-side telemetry.

/// Spec of a LiPo pack: capacity, resistance, and an OCV table.
/// `ocv_v` entries are per-cell open-circuit voltages at the matching `soc`.
#[derive(Clone, Copy, Debug)]
pub struct BatterySpec {
    /// Nominal capacity (A s). e.g. a 1300 mAh pack is 4680 A s.
    pub q_as: f64,
    /// Total pack internal resistance (ohm), all cells in series.
    pub r_pack: f64,
    /// (state of charge, per-cell OCV) table, sorted by soc. Estimated.
    pub ocv: &'static [(f64, f64)],
    /// Number of cells in series (6S -> 6).
    pub cells: u32,
}

/// 6S LiPo discharge curve. Estimated: a representative LiPo OCV curve
/// (cell voltage 4.18 V full down to 3.30 V near empty), interpolated
/// piecewise-linearly. Not a measurement of any particular pack.
pub const LIPO_6S_1300: BatterySpec = BatterySpec {
    q_as: 1.3 * 3600.0,
    r_pack: 0.008,
    cells: 6,
    ocv: &[
        (0.00, 3.30),
        (0.05, 3.62),
        (0.10, 3.72),
        (0.20, 3.80),
        (0.35, 3.85),
        (0.50, 3.88),
        (0.65, 3.92),
        (0.80, 3.99),
        (0.90, 4.06),
        (1.00, 4.18),
    ],
};

impl BatterySpec {
    /// Open-circuit pack voltage (V) at a state of charge, table-interpolated.
    pub fn ocv_v(&self, soc: f64) -> f64 {
        let soc = soc.clamp(0.0, 1.0);
        let mut soc_lo = self.ocv[0].0;
        let mut v_lo = self.ocv[0].1;
        for &(s_hi, v_hi) in &self.ocv[1..] {
            if soc <= s_hi {
                let t = if s_hi > soc_lo { (soc - soc_lo) / (s_hi - soc_lo) } else { 0.0 };
                return self.cells as f64 * (v_lo + t * (v_hi - v_lo));
            }
            soc_lo = s_hi;
            v_lo = v_hi;
        }
        self.cells as f64 * self.ocv[self.ocv.len() - 1].1
    }
}

/// Live pack state.
#[derive(Clone, Copy, Debug)]
pub struct Battery {
    pub spec: BatterySpec,
    /// State of charge, 0..1.
    pub soc: f64,
    /// Last bus current (A), kept for the voltage read.
    pub i_bus: f64,
}

impl Battery {
    pub fn new(spec: BatterySpec) -> Self {
        Self { spec, soc: 1.0, i_bus: 0.0 }
    }

    /// Advance `dt` seconds at a bus current of `i_bus` (A, positive =
    /// discharge). `i_bus` is also stored so reads between steps are
    /// consistent.
    pub fn step(&mut self, dt: f64, i_bus: f64) {
        self.i_bus = i_bus;
        self.soc = (self.soc - i_bus * dt / self.spec.q_as).max(0.0);
    }

    /// Terminal bus voltage (V) under the last-seen current.
    pub fn bus_voltage(&self) -> f64 {
        self.spec.ocv_v(self.soc) - self.i_bus * self.spec.r_pack
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ocv_interpolates_and_bounds() {
        assert!((LIPO_6S_1300.ocv_v(1.0) - 25.08).abs() < 1e-9);
        assert!((LIPO_6S_1300.ocv_v(0.0) - 19.80).abs() < 1e-9);
        // Midpoint of the (0.9, 4.06) / (1.0, 4.18) segment.
        assert!((LIPO_6S_1300.ocv_v(0.95) - 6.0 * 4.12).abs() < 1e-9);
        // Out-of-range soches clamp.
        assert!((LIPO_6S_1300.ocv_v(2.0) - 25.08).abs() < 1e-9);
        assert!((LIPO_6S_1300.ocv_v(-1.0) - 19.80).abs() < 1e-9);
    }

    #[test]
    fn coulomb_count_and_sag() {
        let mut b = Battery::new(LIPO_6S_1300);
        // Hand-computed: 142.8 A for 1 s on a 4680 A s pack draws
        // 142.8/4680 = 3.05% of the charge.
        b.step(1.0, 142.8);
        let expected_soc = 1.0 - 142.8 / (1.3 * 3600.0);
        assert!((b.soc - expected_soc).abs() < 1e-12);
        // Sag: 142.8 A * 8 mOhm = 1.1424 V; OCV drop over the small dSoC.
        let ocv = LIPO_6S_1300.ocv_v(b.soc);
        assert!((b.bus_voltage() - (ocv - 142.8 * 0.008)).abs() < 1e-12);
        // Empty packs clamp at 0, never go negative.
        for _ in 0..100 {
            b.step(1000.0, 200.0);
        }
        assert_eq!(b.soc, 0.0);
        assert!((b.bus_voltage() - LIPO_6S_1300.ocv_v(0.0) + 200.0 * 0.008).abs() < 1e-12);
    }
}