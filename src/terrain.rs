//! Terrain grid for ground contact: the physics-core counterpart of
//! area_pack.py's terrain.bin sidecar (M1 GLO-30). The file's byte layout is
//! the contract (tools/area_pack.py write_terrain_bin): a 56-byte LE header
//! "<IIII5d" — magic, format version, cols, rows, then step, origin_x,
//! origin_y, z_min, z_max — followed by rows*cols f64-LE node heights,
//! row-major (row j at y = origin_y + j*step, index j*cols+i). Node (0,0) is
//! exactly 0.0: z datum = grid origin ground, so every stored height is
//! relative to the pack's own origin, the same convention as the record.
//!
//! h_at mirrors tools/area_pack.py terrain_h_at operation for operation in
//! f64 (same clamps, same index computation, same triangle split), using the
//! u >= v fixed diagonal — the split ObjWriter.quad draws with — so the
//! drawn surface and the contact surface are the same two triangles.

/// terrain.bin magic: 0x31524E54 little-endian = the bytes b"TNR1".
pub const TERRAIN_MAGIC: u32 = 0x3152_4E54;
const TERRAIN_FMT: u32 = 1;
const HEADER_LEN: usize = 56;

#[derive(Clone)]
pub struct TerrainGrid {
    pub cols: usize,
    pub rows: usize,
    pub step: f64,
    pub origin_x: f64,
    pub origin_y: f64,
    pub z_min: f64,
    pub z_max: f64,
    /// Node heights relative to the datum, row-major.
    z: Vec<f64>,
}

impl TerrainGrid {
    /// Parse terrain.bin bytes (the python writer's contract). Rejects bad
    /// magic, wrong format version, dimensions under 2x2, size mismatch,
    /// non-finite header/payload values, a header range disagreeing with the
    /// payload, and a non-zero node (0,0) — the same gates as
    /// _validate_terrain_bin, so any file one side accepts the other does.
    pub fn parse(data: &[u8]) -> Result<Self, String> {
        if data.len() < HEADER_LEN {
            return Err(format!(
                "terrain.bin short: {} bytes < {} header",
                data.len(),
                HEADER_LEN
            ));
        }
        let u32_at = |i: usize| u32::from_le_bytes(data[i..i + 4].try_into().unwrap());
        let f64_at = |i: usize| f64::from_le_bytes(data[i..i + 8].try_into().unwrap());
        let magic = u32_at(0);
        if magic != TERRAIN_MAGIC {
            return Err(format!("terrain.bin magic 0x{magic:08X} != 0x{TERRAIN_MAGIC:08X}"));
        }
        let fmt = u32_at(4);
        if fmt != TERRAIN_FMT {
            return Err(format!("terrain.bin format version {fmt} != {TERRAIN_FMT}"));
        }
        let cols = u32_at(8) as usize;
        let rows = u32_at(12) as usize;
        if cols < 2 || rows < 2 {
            return Err(format!("terrain.bin grid {cols}x{rows}: both dims must be >= 2"));
        }
        let (step, origin_x, origin_y) = (f64_at(16), f64_at(24), f64_at(32));
        let (z_min, z_max) = (f64_at(40), f64_at(48));
        if !step.is_finite() || step <= 0.0 {
            return Err(format!("terrain.bin step {step:?} not a positive finite length"));
        }
        if !origin_x.is_finite() || !origin_y.is_finite() {
            return Err(format!("terrain.bin origin ({origin_x:?}, {origin_y:?}) not finite"));
        }
        if !z_min.is_finite() || !z_max.is_finite() {
            return Err("terrain.bin z range not finite".to_string());
        }
        let want = HEADER_LEN + 8 * rows * cols;
        if data.len() != want {
            return Err(format!(
                "terrain.bin {} bytes != {want} (header + 8 * {rows} * {cols})",
                data.len()
            ));
        }
        let z: Vec<f64> = data[HEADER_LEN..]
            .chunks_exact(8)
            .map(|b| f64::from_le_bytes(b.try_into().unwrap()))
            .collect();
        let mut pmin = f64::INFINITY;
        let mut pmax = f64::NEG_INFINITY;
        for &v in &z {
            if !v.is_finite() {
                return Err(format!("terrain.bin has a non-finite node: {v:?}"));
            }
            pmin = pmin.min(v);
            pmax = pmax.max(v);
        }
        if pmin != z_min || pmax != z_max {
            return Err(format!(
                "terrain.bin payload range [{pmin:?}, {pmax:?}] != header [{z_min:?}, {z_max:?}]"
            ));
        }
        if z[0] != 0.0 {
            return Err(format!("terrain.bin node (0,0) is {:?}, not 0.0 (z datum)", z[0]));
        }
        Ok(Self {
            cols,
            rows,
            step,
            origin_x,
            origin_y,
            z_min,
            z_max,
            z,
        })
    }

    /// Load terrain.bin from a file path.
    pub fn load(path: &str) -> Result<Self, String> {
        let bytes = std::fs::read(path).map_err(|e| format!("read {path}: {e}"))?;
        Self::parse(&bytes).map_err(|e| format!("{path}: {e}"))
    }

    /// Height (m, relative to the datum) at world (x, y): bilinear over the
    /// u >= v triangle. Operation order matches terrain_h_at in
    /// tools/area_pack.py exactly.
    pub fn h_at(&self, x: f64, y: f64) -> f64 {
        let ic = self.cols - 1;
        let jc = self.rows - 1;
        // Clamp order mirrors python (max 0.0 first, then the corner count).
        let mut u = (x - self.origin_x) / self.step;
        let mut v = (y - self.origin_y) / self.step;
        u = u.max(0.0).min(ic as f64);
        v = v.max(0.0).min(jc as f64);
        let i = (ic - 1).min(u as usize);
        let j = (jc - 1).min(v as usize);
        let uc = u - i as f64;
        let vc = v - j as f64;
        let z00 = self.z[j * self.cols + i];
        let z10 = self.z[j * self.cols + i + 1];
        let z01 = self.z[(j + 1) * self.cols + i];
        let z11 = self.z[(j + 1) * self.cols + i + 1];
        if uc >= vc {
            (1.0 - uc) * z00 + (uc - vc) * z10 + vc * z11
        } else {
            (1.0 - vc) * z00 + (vc - uc) * z01 + uc * z11
        }
    }
}

/// The physical ground a quad collides with: Flat (z = 0, the pre-M1
/// behaviour) or a DEM grid. An enum rather than a trait object: Quad::step
/// calls it every substep and Flat must stay a plain constant branch with the
/// exact pre-M1 f64 result.
#[derive(Clone)]
pub enum Ground {
    Flat,
    Grid(TerrainGrid),
}

impl Ground {
    pub fn height(&self, x: f64, y: f64) -> f64 {
        match self {
            Ground::Flat => 0.0,
            Ground::Grid(g) => g.h_at(x, y),
        }
    }
}