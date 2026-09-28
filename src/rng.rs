//! Shared deterministic RNG: xoshiro256** seeded through splitmix64. Small,
//! no dependency. Used by the sensor model (src/sensor.rs) and the wind
//! model (src/wind.rs) — each gets its own stream from its own seed, so the
//! two noise sources stay independent.
//!
//! Good enough for sensor noise and gusts (not cryptographic; provenance
//! hashes use src/sha256.rs). Moved here from sensor.rs verbatim (T4) so
//! both models share one implementation.

pub(crate) struct Rng {
    s: [u64; 4],
    /// Box-Muller spare, held between `normal` calls.
    spare: Option<f64>,
}

impl Rng {
    pub(crate) fn new(seed: u64) -> Self {
        let mut z = seed;
        let mut next = move || {
            z = z.wrapping_add(0x9E37_79B9_7F4A_7C15);
            let mut w = z;
            w = (w ^ (w >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
            w = (w ^ (w >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
            w ^ (w >> 31)
        };
        Self { s: [next(), next(), next(), next()], spare: None }
    }

    pub(crate) fn u64(&mut self) -> u64 {
        let r = self.s[1].wrapping_mul(5).rotate_left(7).wrapping_mul(9);
        let t = self.s[1] << 17;
        self.s[2] ^= self.s[0];
        self.s[3] ^= self.s[1];
        self.s[1] ^= self.s[2];
        self.s[0] ^= self.s[3];
        self.s[2] ^= t;
        self.s[3] = self.s[3].rotate_left(45);
        r
    }

    /// Standard normal via Box-Muller with a one-sample spare (pinned
    /// consumption order: cos first, sin spare).
    pub(crate) fn normal(&mut self) -> f64 {
        if let Some(v) = self.spare.take() {
            return v;
        }
        let scale = 1.0 / (1u64 << 53) as f64;
        // u1 in (0, 1] so the log never sees zero.
        let u1 = 1.0 - ((self.u64() >> 11) as f64) * scale;
        let u2 = ((self.u64() >> 11) as f64) * scale;
        let r = (-2.0 * u1.ln()).sqrt();
        let th = 2.0 * std::f64::consts::PI * u2;
        self.spare = Some(r * th.sin());
        r * th.cos()
    }

    /// Vector of 3 independent normals.
    pub(crate) fn normal3(&mut self) -> crate::DVec3 {
        crate::DVec3::new(self.normal(), self.normal(), self.normal())
    }
}