//! Wind: a mean vector plus seeded Dryden gusts (MIL-F-8785C, low-altitude
//! band), fed into `Quad::wind` once per tick. Drag and rotor axial inflow
//! already act on air-relative velocity (T1), so this module is purely the
//! wind SOURCE.
//!
//! Model (pinned from MIL-F-8785C via two independent implementations that
//! agree — pyfly dryden.py and the MathWorks Aerospace Blockset Dryden doc,
//! both read 2026-09-28; h in feet, 10-1000 ft band):
//!   sigma_w   = 0.1 * W20                              (W20 = wind speed at
//!                                                       20 ft: 7.7/15.4/23.2
//!                                                       m/s light/mod/severe)
//!   sigma_h   = sigma_w / (0.177 + 0.000823 h)^0.4
//!   L_w_ft    = h
//!   L_h_ft    = h / (0.177 + 0.000823 h)^1.2
//!   Phi(omega) = sigma^2 L / (pi V) * (1 + 3 (L omega / V)^2)
//!                / (1 + (L omega / V)^2)^2             (one-sided, rad/s)
//! where h = altitude, V = advection speed, and the same temporal spectrum
//! serves the lateral and vertical channels.
//!
//! Adaptation, labelled: MIL-F-8785C defines three channels along a flight
//! path (u longitudinal, v lateral, w vertical) for a vehicle flying THROUGH
//! the turbulence at speed V. A hovering/omnidirectional quad has no
//! longitudinal axis, so the two HORIZONTAL world axes (east, north) both use
//! the lateral spectrum with independent streams (isotropic in the horizontal
//! plane — the spectra themselves are direction-independent) and the vertical
//! axis uses the vertical spectrum. The longitudinal channel is dropped, not
//! reused.
//!
//! Discrete realization: each channel is the exact 2-state cascade of the
//! continuous prototype H(s) = K' (1 + sqrt(3) tau s) / (1 + tau s)^2,
//!   x1' = e x1 + (1-e) w,  x2' = e x2 + (1-e) x1',  e = exp(-dt/tau),
//!   y = K' ( sqrt(3) x1 + (1 - sqrt(3)) x2 ),
//! driven by unit-variance white samples. K' = sigma sqrt(L / (V dt)) makes
//! the sampled sequence's one-sided per-Hz PSD equal the target
//! 2 sigma^2 L / V * (1+3x^2)/(1+x^2)^2 (per-Hz form of Phi, x = 2 pi f L / V)
//! in-band: the pi factors cancel through the 2 pi conversion. Verified
//! against the analytic PSD by Welch estimator in tests/wind.rs.
//!
//! Gaps, stated: the 1000-2000 ft interpolation band of the MIL spec is not
//! implemented (h clamps at 1000 ft); wind shadowing / terrain interaction
//! stays out (VISION stretch goal); the advection speed V is floored at 1 m/s
//! so hovering in still air still sees a plausible gust timescale (estimated,
//! not from the spec).

use crate::rng::Rng;
use crate::DVec3;

const FT_PER_M: f64 = 1.0 / 0.3048;
/// MIL-F-8785C low-altitude band edges (the 1000-2000 ft interpolation band
/// above this is not implemented).
const H_MIN_FT: f64 = 10.0;
const H_MAX_FT: f64 = 1000.0;
/// Advection-speed floor so hover still has a sensible gust timescale
/// (L/V) when the mean wind is calm. Estimated.
const V_FLOOR: f64 = 1.0;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WindConfig {
    /// Mean wind, ENU m/s (x east, y north, z up).
    pub mean: DVec3,
    /// W20 gust intensity: the mean wind speed at 20 ft per MIL-F-8785C.
    /// 0 = gusts off (the model then outputs exactly `mean`, consuming no
    /// RNG, so gust-off runs are seed-independent).
    pub w20_ms: f64,
    /// Seed for the gust stream (independent of the sensor-model stream).
    pub seed: u64,
}

/// Dryden low-altitude turbulence scales at altitude `h_m` for intensity
/// `w20_ms`: (L_horizontal_m, L_vertical_m, sigma_horizontal_ms,
/// sigma_vertical_ms). h clamps to the 10-1000 ft band. Public so the record
/// consumer and tests can reproduce the target spectra.
pub fn scales(h_m: f64, w20_ms: f64) -> (f64, f64, f64, f64) {
    let h_ft = (h_m * FT_PER_M).clamp(H_MIN_FT, H_MAX_FT);
    let d = 0.177 + 0.000823 * h_ft;
    let l_h_ft = h_ft / d.powf(1.2);
    let l_w_ft = h_ft;
    let sigma_w = 0.1 * w20_ms;
    let sigma_h = sigma_w / d.powf(0.4);
    (
        l_h_ft * 0.3048,
        l_w_ft * 0.3048,
        sigma_h,
        sigma_w,
    )
}

/// One gust channel: the cascaded 2-state filter's states.
#[derive(Clone, Copy, Default)]
struct Channel {
    x1: f64,
    x2: f64,
}

/// The running model. `step` is called once per TICK (250 Hz) with the
/// craft's altitude; the returned total (mean + gust) is written straight
/// into `Quad::wind`.
pub struct WindModel {
    cfg: WindConfig,
    rng: Rng,
    /// Gust channels in world axes: east, north (lateral spectrum), up
    /// (vertical spectrum).
    ch: [Channel; 3],
}

impl WindModel {
    pub fn new(cfg: WindConfig) -> Self {
        Self {
            cfg,
            rng: Rng::new(cfg.seed),
            ch: [Channel::default(); 3],
        }
    }

    pub fn config(&self) -> &WindConfig {
        &self.cfg
    }

    /// Total wind (mean + gust), world ENU m/s, at altitude `alt_m`. With
    /// `w20_ms <= 0` this is exactly the mean and consumes no RNG. With
    /// gusts on, the RNG consumption order is pinned: one normal per
    /// channel, east then north then up.
    pub fn step(&mut self, dt: f64, alt_m: f64) -> DVec3 {
        if self.cfg.w20_ms <= 0.0 {
            return self.cfg.mean;
        }
        let (l_h, l_w, s_h, s_w) = scales(alt_m, self.cfg.w20_ms);
        // Advection speed: the mean wind sets the gust timescale; floored
        // so hovering in calm air still gets a plausible spectrum.
        let v = self.cfg.mean.length().max(V_FLOOR);
        let shapes = [(s_h, l_h), (s_h, l_h), (s_w, l_w)];
        let mut g = [0.0f64; 3];
        for (i, ch) in self.ch.iter_mut().enumerate() {
            let (sigma, len) = shapes[i];
            let tau = len / v;
            let e = (-dt / tau).exp();
            // Per-sample unit-variance drive -> the 1/sqrt(dt) gain below
            // makes the sampled output's PSD the target spectrum (module
            // doc).
            let k = sigma * (len / (v * dt)).sqrt();
            let w = self.rng.normal();
            ch.x1 = e * ch.x1 + (1.0 - e) * w;
            ch.x2 = e * ch.x2 + (1.0 - e) * ch.x1;
            const SQRT3: f64 = 1.7320508075688772;
            g[i] = k * (SQRT3 * ch.x1 + (1.0 - SQRT3) * ch.x2);
        }
        self.cfg.mean + DVec3::new(g[0], g[1], g[2])
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Pinned MIL-F-8785C low-altitude scales at 100 ft, hand-computed from
    /// the spec relations (d = 0.2593, d^1.2 = 0.19802, d^0.4 = 0.58281):
    /// L_h = 100/0.19802 ft = 505.0 ft = 153.9 m, L_w = 30.48 m,
    /// sigma_w = 0.1 * 15.43 = 1.543 m/s, sigma_h = 1.543/0.58281 = 2.648.
    #[test]
    fn scales_match_hand_computed_spec_values() {
        let (l_h, l_w, s_h, s_w) = scales(100.0 * 0.3048, 15.43);
        assert!((l_h - 153.93).abs() < 0.05, "L_h {l_h} vs 153.93 m");
        assert!((l_w - 30.48).abs() < 1e-9, "L_w {l_w} vs 30.48 m");
        assert!((s_h - 2.6476).abs() < 0.001, "sigma_h {s_h} vs 2.648");
        assert!((s_w - 1.543).abs() < 1e-9, "sigma_w {s_w} vs 1.543");
        // Band clamp: 3 m and 400 m both hit the 10/1000 ft clamps.
        let (l_lo, _, s_lo, _) = scales(3.0, 15.43);
        let (l_hi, _, s_hi, _) = scales(400.0, 15.43);
        assert!((l_lo - scales(10.0 * 0.3048, 15.43).0).abs() < 1e-9);
        assert!((s_lo - scales(10.0 * 0.3048, 15.43).2).abs() < 1e-12);
        assert!((l_hi - scales(1000.0 * 0.3048, 15.43).0).abs() < 1e-9);
        assert!((s_hi - scales(1000.0 * 0.3048, 15.43).2).abs() < 1e-12);
    }

    /// Gusts off (w20 = 0): the model is exactly the mean regardless of
    /// seed, and consumes no RNG (two models on different seeds agree bit
    /// for bit).
    #[test]
    fn gusts_off_is_pure_mean() {
        let cfg = WindConfig { mean: DVec3::new(1.5, -2.0, 0.25), w20_ms: 0.0, seed: 1 };
        let mut a = WindModel::new(cfg);
        let mut b = WindModel::new(WindConfig { seed: 99, ..cfg });
        for i in 0..1000 {
            let dt = 0.004;
            let va = a.step(dt, 30.0 + i as f64 * 0.01);
            let vb = b.step(dt, 30.0 + i as f64 * 0.017);
            assert_eq!(va.to_array(), vb.to_array(), "step {i}");
            assert_eq!(va, cfg.mean);
        }
    }

    /// Same seed and config -> bit-identical series; different seed ->
    /// different series.
    #[test]
    fn seeded_series_bit_identical() {
        let cfg = WindConfig { mean: DVec3::new(10.0, 0.0, 0.0), w20_ms: 15.43, seed: 7 };
        let run = |seed: u64| {
            let mut m = WindModel::new(WindConfig { seed, ..cfg });
            let mut out = Vec::new();
            for _ in 0..2000 {
                out.push(m.step(0.004, 30.48).to_array());
            }
            out
        };
        let a = run(7);
        let b = run(7);
        assert_eq!(a, b, "same seed must reproduce bit-identical gusts");
        let c = run(8);
        assert!(
            a.iter().zip(&c).any(|(x, y)| x != y),
            "different seeds produced identical series"
        );
    }

    /// Long-run statistics of the sampled gust series: per-channel mean
    /// unbiased against the analytic sigma (n_eff argument in
    /// tests/wind.rs) and std within 10% of the analytic sigma — the direct
    /// check that the K' gain normalisation is right.
    #[test]
    fn gust_series_matches_sigma() {
        // Mean wind 10 m/s east so the advection speed is realistic; fixed
        // 100 ft altitude; 2000 s at the 250 Hz tick.
        let cfg = WindConfig { mean: DVec3::new(10.0, 0.0, 0.0), w20_ms: 15.43, seed: 3 };
        let (l_h, l_w, s_h, s_w) = scales(100.0 * 0.3048, 15.43);
        let mut m = WindModel::new(cfg);
        let n = 2000 * 250;
        let (mut sum, mut sum2) = ([0.0f64; 3], [0.0f64; 3]);
        for _ in 0..n {
            let w = m.step(0.004, 100.0 * 0.3048);
            let g = [w.x - cfg.mean.x, w.y - cfg.mean.y, w.z - cfg.mean.z];
            for k in 0..3 {
                sum[k] += g[k];
                sum2[k] += g[k] * g[k];
            }
        }
        let expect = [s_h, s_h, s_w];
        for k in 0..3 {
            let mean = sum[k] / n as f64;
            let std = (sum2[k] / n as f64 - mean * mean).sqrt();
            // n_eff: correlation time ~ 2 tau = 2 L / V (~31 s horizontal,
            // ~6 s vertical) over 2000 s -> 60+ effective samples; 0.5 sigma
            // is ~4 sigma_eff.
            let tau = if k == 2 { l_w } else { l_h } / 10.0;
            let n_eff = 2000.0 / (2.0 * tau);
            assert!(
                mean.abs() < 0.5 * expect[k],
                "ch {k}: mean {mean:.3} vs sigma {:.3} (n_eff {:.0})",
                expect[k], n_eff
            );
            assert!(
                (std - expect[k]).abs() / expect[k] < 0.10,
                "ch {k}: std {std:.3} vs sigma {:.3}",
                expect[k]
            );
        }
    }
}