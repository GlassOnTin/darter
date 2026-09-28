//! Seeded sensor model: what the FC's IMU sees, layered on the rigid-body
//! truth (src/quad.rs keeps the truth clean — VISION: sensor imperfection is
//! never baked into the state).
//!
//! Three effects, each individually switchable to zero (all zero = the
//! model is a pass-through and callers may skip it entirely — the record
//! bytes stay bit-identical to the no-sensor path):
//! - gyro / accel white noise (per-sample Gaussian),
//! - gyro bias: fixed random draw per flight plus a seeded random walk,
//! - rpm-keyed frame vibration: sinusoids at the blade-pass frequency
//!   (2 blades × motor rotation frequency) and its second harmonic,
//!   amplitude linear in throttle — the tuning input for Betaflight's
//!   dynamic RPM notch.
//!
//! All magnitudes are ESTIMATED (typical MEMS datasheet figures scaled to an
//! 8 kHz sample rate), not measured; defaults sit near a 5" freestyle quad's
//! MPU-6000-class gyro. Deterministic: a seeded xoshiro256** drives
//! everything, identical seeds and step sequences reproduce identical
//! samples bit for bit.

use std::f64::consts::PI;

use crate::rng::Rng;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SensorConfig {
    /// Gyro white-noise sigma per sample (rad/s).
    pub gyro_noise_std: f64,
    /// Gyro initial-bias draw sigma (rad/s) — one Gaussian per axis at
    /// construction.
    pub gyro_bias_std: f64,
    /// Gyro bias random-walk sigma (rad/s per sqrt(s)).
    pub gyro_bias_rw_std: f64,
    /// Accel white-noise sigma per sample (m/s^2).
    pub accel_noise_std: f64,
    /// Blade-pass vibration amplitude at full throttle: accel (m/s^2) and
    /// gyro (rad/s), plus the second harmonic as a fraction of the first.
    pub vib_accel_amp: f64,
    pub vib_gyro_amp: f64,
    pub vib2_scale: f64,
    /// Seed for the whole model (bias draw, walks, noise, all one stream).
    pub seed: u64,
}

impl SensorConfig {
    /// All effects off — pass-through.
    pub const OFF: Self = Self {
        gyro_noise_std: 0.0,
        gyro_bias_std: 0.0,
        gyro_bias_rw_std: 0.0,
        accel_noise_std: 0.0,
        vib_accel_amp: 0.0,
        vib_gyro_amp: 0.0,
        vib2_scale: 0.0,
        seed: 0,
    };

    /// Default 5" quad estimate (all values estimated, not measured):
    /// MPU-6000-class densities integrated over an 8 kHz sample rate.
    pub const DEFAULT: Self = Self {
        gyro_noise_std: 0.0055,   // 0.005 dps/sqrt(Hz) over 8 kHz
        gyro_bias_std: 0.0087,    // 0.5 dps turn-on bias
        gyro_bias_rw_std: 1.7e-6, // 0.01 dps/sqrt(s) in-flight walk
        accel_noise_std: 0.25,    // 400 ug/sqrt(Hz) over 8 kHz
        vib_accel_amp: 2.0,       // ~0.2 g frame vibration at full throttle
        vib_gyro_amp: 0.02,
        vib2_scale: 0.5,
        seed: 1,
    };

    pub fn is_off(&self) -> bool {
        self.gyro_noise_std == 0.0
            && self.gyro_bias_std == 0.0
            && self.gyro_bias_rw_std == 0.0
            && self.accel_noise_std == 0.0
            && self.vib_accel_amp == 0.0
            && self.vib_gyro_amp == 0.0
    }
}

/// The running model. `step` is called once per core substep in the same
/// order as `Quad::step`, so noise timestamps line up with the physics.
pub struct SensorModel {
    cfg: SensorConfig,
    rng: Rng,
    gyro_bias: crate::DVec3,
    /// Blade-pass phase (rad), integrated from the per-step mean rotor speed
    /// so rpm changes stay phase-continuous.
    vib_phase: f64,
}

impl SensorModel {
    pub fn new(cfg: SensorConfig) -> Self {
        let mut rng = Rng::new(cfg.seed);
        let gyro_bias = if cfg.gyro_bias_std > 0.0 {
            cfg.gyro_bias_std * rng.normal3()
        } else {
            crate::DVec3::ZERO
        };
        Self { cfg, rng, gyro_bias, vib_phase: 0.0 }
    }

    pub fn config(&self) -> &SensorConfig {
        &self.cfg
    }

    /// One sensor sample. `rpm_mean`/`thr_mean` drive the vibration terms
    /// (pass the quad's mean rotor speed and throttle); truth is the clean
    /// gyro (rad/s) and proper acceleration (m/s^2), FLU body frame.
    pub fn step(
        &mut self,
        dt: f64,
        truth_gyro: crate::DVec3,
        truth_accel: crate::DVec3,
        rpm_mean: f64,
        thr_mean: f64,
    ) -> (crate::DVec3, crate::DVec3) {
        // Bias walk (rad/s), seeded, continuous in time.
        if self.cfg.gyro_bias_rw_std > 0.0 {
            self.gyro_bias += self.cfg.gyro_bias_rw_std * dt.sqrt() * self.rng.normal3();
        }

        let mut g = truth_gyro + self.gyro_bias;
        let mut a = truth_accel;

        if self.cfg.gyro_noise_std > 0.0 {
            g += self.cfg.gyro_noise_std * self.rng.normal3();
        }
        if self.cfg.accel_noise_std > 0.0 {
            a += self.cfg.accel_noise_std * self.rng.normal3();
        }

        // rpm-keyed vibration: blade pass (2 blades) + harmonic, amplitude
        // linear in throttle. Frame vibration is not a rigid-body quantity;
        // this injects an isotropic per-axis sinusoid as a placeholder shape
        // (labelled-unvalidated, like the descent wash clamp).
        if self.cfg.vib_accel_amp > 0.0 || self.cfg.vib_gyro_amp > 0.0 {
            let f0 = 2.0 * rpm_mean / 60.0; // blade-pass Hz, 2-blade props
            self.vib_phase += 2.0 * PI * f0 * dt;
            let s1 = self.vib_phase.sin();
            let s2 = (2.0 * self.vib_phase).sin();
            a += self.cfg.vib_accel_amp * thr_mean * (s1 + self.cfg.vib2_scale * s2);
            g += self.cfg.vib_gyro_amp * thr_mean * (s1 + self.cfg.vib2_scale * s2);
        }

        (g, a)
    }

    /// Current gyro bias (rad/s) — exposed for the record and tests.
    pub fn gyro_bias(&self) -> crate::DVec3 {
        self.gyro_bias
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::DVec3;

    /// Two models on the same seed produce bit-identical streams (compare
    /// the raw bits — equality through floats would miss -0.0 vs 0.0).
    #[test]
    fn seeded_streams_bit_identical() {
        let run = |cfg: SensorConfig| {
            let mut m = SensorModel::new(cfg);
            let mut out = Vec::new();
            for _ in 0..1000 {
                let (g, a) = m.step(125e-6, DVec3::new(0.1, 0.2, 0.3), DVec3::new(0.0, 0.0, 9.8), 10000.0, 0.16);
                out.push((g.to_array(), a.to_array()));
            }
            out
        };
        let a = run(SensorConfig::DEFAULT);
        let b = run(SensorConfig::DEFAULT);
        for (x, y) in a.iter().zip(&b) {
            for k in 0..3 {
                assert_eq!(x.0[k].to_bits(), y.0[k].to_bits());
                assert_eq!(x.1[k].to_bits(), y.1[k].to_bits());
            }
        }
        // Different seed -> different stream (with overwhelming probability;
        // one mismatch is enough).
        let c = run(SensorConfig { seed: 2, ..SensorConfig::DEFAULT });
        let differ = a.iter().zip(&c).any(|(x, y)| {
            (0..3).any(|k| x.0[k] != y.0[k] || x.1[k] != y.1[k])
        });
        assert!(differ, "different seeds produced identical streams");
    }

    /// White-noise sigma matches the config within ±5% on a pinned sample
    /// count (80000 samples: the 1/sqrt(2N) sample-sigma error is ~0.25%, so
    /// the tolerance bounds model drift, not the estimator).
    #[test]
    fn white_noise_sigma_matches_config() {
        for &(std, axis) in &[(0.0055f64, 0usize), (0.25, 1usize)] {
            let cfg = SensorConfig {
                gyro_noise_std: if axis == 0 { std } else { 0.0 },
                accel_noise_std: if axis == 1 { std } else { 0.0 },
                ..SensorConfig::OFF
            };
            let mut m = SensorModel::new(cfg);
            let n = 80_000usize;
            let mut sum = 0.0;
            let mut sum2 = 0.0f64;
            for _ in 0..n {
                let (g, a) = m.step(125e-6, DVec3::ZERO, DVec3::ZERO, 0.0, 0.0);
                let v = if axis == 0 { g.x } else { a.y };
                sum += v;
                sum2 += v * v;
            }
            let mean = sum / n as f64;
            let var = sum2 / n as f64 - mean * mean;
            let sigma = var.sqrt();
            let rel = (sigma - std).abs() / std;
            assert!(rel < 0.05, "axis {axis}: sigma {sigma:.6} vs config {std:.6} (rel {rel:.4})");
            // Zero-mean within a few standard errors.
            let sem = std / (n as f64).sqrt();
            assert!(mean.abs() < 4.0 * sem, "mean {mean} vs sem {sem}");
        }
    }

    /// Gyro bias random walk: two one-step-apart reads differ by a seeded
    /// amount and the walk grows like sqrt(t) (variance ratio over a 4x time
    /// span, generous ±30% band; N is pinned).
    #[test]
    fn gyro_bias_random_walk() {
        let cfg = SensorConfig { gyro_bias_rw_std: 1.7e-6, ..SensorConfig::OFF };
        let mut m = SensorModel::new(cfg);
        let dt = 125e-6;
        let (g0, _) = m.step(dt, DVec3::ZERO, DVec3::ZERO, 0.0, 0.0);
        // One-second walk: dt^0.5 scaling means bias after N steps has
        // sigma = rw_std * sqrt(N*dt).
        let n = 8000;
        for _ in 0..n {
            m.step(dt, DVec3::ZERO, DVec3::ZERO, 0.0, 0.0);
        }
        let b = m.gyro_bias();
        let expected_sigma = cfg.gyro_bias_rw_std * ((n + 1) as f64 * dt).sqrt();
        // 3-sigma band on one draw is weak, so repeat the ensemble 64 times
        // and check the empirical std against the analytic one.
        let mut vals = Vec::new();
        for s in 1..=64u64 {
            let mut m = SensorModel::new(SensorConfig { seed: s, ..cfg });
            for _ in 0..n {
                m.step(dt, DVec3::ZERO, DVec3::ZERO, 0.0, 0.0);
            }
            vals.push(m.gyro_bias().x);
        }
        let mean = vals.iter().sum::<f64>() / vals.len() as f64;
        let var = vals.iter().map(|v| (v - mean) * (v - mean)).sum::<f64>() / (vals.len() - 1) as f64;
        let std = var.sqrt();
        let rel = (std - expected_sigma).abs() / expected_sigma;
        assert!(rel < 0.30, "walk std {std:.3e} vs analytic {expected_sigma:.3e} (rel {rel:.2})");
        let _ = (g0, b);
    }

    /// Blade-pass vibration lands on the rpm-keyed frequency: a Goertzel
    /// amplitude estimate at f0 and 2f0 over a pinned, period-exact window
    /// recovers the configured amplitudes within ±20%; an off-frequency bin
    /// sees almost nothing.
    #[test]
    fn vibration_is_rpm_keyed() {
        let rpm = 12_000.0; // blade pass 400 Hz, 2nd harmonic 800 Hz
        let thr = 1.0;
        let cfg = SensorConfig {
            vib_accel_amp: 2.0,
            vib_gyro_amp: 0.0,
            vib2_scale: 0.5,
            ..SensorConfig::OFF
        };
        let mut m = SensorModel::new(cfg);
        let dt = 125e-6; // 8 kHz
        let n = 8000; // 1 s: 400 cycles; Goertzel bins are exact at 1 Hz spacing
        let mut samples = Vec::with_capacity(n);
        for _ in 0..n {
            let (_, a) = m.step(dt, DVec3::ZERO, DVec3::ZERO, rpm, thr);
            samples.push(a.z);
        }
        let goertzel = |freq: f64, x: &[f64]| -> f64 {
            let w = 2.0 * PI * freq * dt;
            let (mut s1, mut s2) = (0.0f64, 0.0f64);
            for &v in x {
                let s0 = v + 2.0 * w.cos() * s1 - s2;
                s2 = s1;
                s1 = s0;
            }
            let c = s1 - w.cos() * s2;
            let si = w.sin() * s2;
            2.0 * (c * c + si * si).sqrt() / x.len() as f64
        };
        let a400 = goertzel(400.0, &samples);
        let a800 = goertzel(800.0, &samples);
        let a401 = goertzel(401.0, &samples);
        assert!((a400 - 2.0).abs() < 0.4, "f0 amplitude {a400} vs 2.0");
        assert!((a800 - 1.0).abs() < 0.2, "2f0 amplitude {a800} vs 1.0");
        assert!(a401 < 0.05, "off-bin leakage {a401}");
        // Vibration scales linearly with throttle: half throttle, half amp.
        let mut m2 = SensorModel::new(cfg);
        let mut samples2 = Vec::with_capacity(n);
        for _ in 0..n {
            let (_, a) = m2.step(dt, DVec3::ZERO, DVec3::ZERO, rpm, 0.5);
            samples2.push(a.z);
        }
        let half = goertzel(400.0, &samples2);
        assert!((half - 1.0).abs() < 0.2, "half-throttle amplitude {half} vs 1.0");
    }

    /// Zero-throttle means zero vibration even at speed (amplitude is
    /// throttle-keyed).
    #[test]
    fn vibration_zero_at_zero_throttle() {
        let mut m = SensorModel::new(SensorConfig {
            vib_accel_amp: 2.0,
            vib_gyro_amp: 0.5,
            ..SensorConfig::OFF
        });
        let mut max_g = 0.0f64;
        for _ in 0..1000 {
            let (g, a) = m.step(125e-6, DVec3::ZERO, DVec3::ZERO, 12000.0, 0.0);
            max_g = max_g.max(g.x.abs()).max(a.z.abs());
        }
        assert_eq!(max_g, 0.0);
    }
}