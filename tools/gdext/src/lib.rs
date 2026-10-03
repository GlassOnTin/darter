//! Darter live plant: the darter-core physics flight as a Godot class.
//!
//! `DarterQuad` wraps darter_core::flight::CoreFlight — the exact loop
//! `sim_run --mode core` runs (250 Hz ticks of 32 x 125 us substeps, one
//! darter_record sample per tick) — per-call, so GDScript drives the flight
//! live: spawn, throttle, drive ticks, read state, hash up the record.
//! The acceptance contract is tests/godot_live.rs: a flight driven through
//! this class is byte-identical to the same flight through sim_run.
//!
//! Panics never cross the FFI boundary: every fallible call reports via
//! godot_error! plus a boolean / empty-string / empty-state return, and the
//! probe scene treats those as a failure.

use darter_core::flight::{CoreFlight, CoreSetup, TICK_DT};
use darter_core::preset::Preset;
use darter_core::terrain::TerrainGrid;
use darter_core::DVec3;
use godot::prelude::*;

/// The extension entry struct (gdext wants a named unit type; the class
/// work happens through the derive macros below).
struct DarterGd;

#[gdextension]
unsafe impl ExtensionLibrary for DarterGd {}

#[derive(GodotClass)]
#[class(base = RefCounted)]
struct DarterQuad {
    /// Per-motor throttle in [0, 1]. Unset = the solved hover value, as
    /// sim_run's default. `record_open` freezes the scalar (all four must
    /// agree or it refuses): the record header describes the whole flight's
    /// throttle, so a mid-record change is not representable.
    throttle: Option<[f64; 4]>,
    /// (x, y, AGL offset) the flight spawns at, set by `setup`.
    spawn: Option<(f64, f64, f64)>,
    /// Terrain grid loaded by `setup` (empty path = flat ground).
    terrain: Option<TerrainGrid>,
    /// The flight under construction / in progress; `finish_record`
    /// consumes it.
    flight: Option<CoreFlight>,
}

#[godot_api]
impl IRefCounted for DarterQuad {
    fn init(_base: Base<RefCounted>) -> Self {
        Self {
            throttle: None,
            spawn: None,
            terrain: None,
            flight: None,
        }
    }
}

#[godot_api]
impl DarterQuad {
    /// Load the optional terrain grid (empty path = flat ground) and set the
    /// spawn position convention sim_run's core mode uses: (x, y) in the
    /// record frame, the offset measured above the ground under (x, y).
    /// Returns false (and logs a godot error) on a missing or bad terrain
    /// file; on failure no state is left half-applied.
    #[func]
    fn setup(&mut self, terrain_path: GString, spawn_x: f64, spawn_y: f64, agl: f64) -> bool {
        let path = terrain_path.to_string();
        self.terrain = None;
        if !path.is_empty() {
            match std::fs::read(&path) {
                Ok(bytes) => match TerrainGrid::parse(&bytes) {
                    Ok(grid) => self.terrain = Some(grid),
                    Err(e) => {
                        godot_error!("DarterQuad: terrain {path}: {e}");
                        return false;
                    }
                },
                Err(e) => {
                    godot_error!("DarterQuad: terrain {path}: read: {e}");
                    return false;
                }
            }
        }
        self.spawn = Some((spawn_x, spawn_y, agl));
        true
    }

    /// Set the per-motor throttle for the next flight. A real live stack
    /// writes these every tick (M2b+); here they only define the scripted
    /// value the record describes.
    #[func]
    fn set_throttle(&mut self, t0: f64, t1: f64, t2: f64, t3: f64) {
        self.throttle = Some([t0, t1, t2, t3]);
    }

    /// Open the darter_record file for the next flight: header built from
    /// the settled inputs (locked throttle, spawn, terrain), first byte
    /// written now. Same header contract as sim_run --mode core.
    /// Returns false (and logs a godot error) without opening on any
    /// invalid combo.
    #[func]
    fn record_open(&mut self, record_path: GString, planned_duration_s: f64, seed: i64) -> bool {
        // Godot ints are i64; a negative seed is refused rather than wrapped.
        let seed = match u64::try_from(seed) {
            Ok(s) => s,
            Err(_) => {
                godot_error!("DarterQuad: seed must be non-negative (got {seed})");
                return false;
            }
        };
        let Some((x, y, agl)) = self.spawn else {
            godot_error!("DarterQuad: record_open before setup");
            return false;
        };
        let thr = match self.throttle {
            Some([t0, t1, t2, t3]) => {
                if t1 != t0 || t2 != t0 || t3 != t0 {
                    godot_error!("DarterQuad: per-motor throttle must agree for a scripted flight (got {t0:?}, {t1:?}, {t2:?}, {t3:?})");
                    return false;
                }
                Some(t0)
            }
            None => None, // the solved hover value, as sim_run's default
        };
        let setup = CoreSetup {
            preset: Preset::FREESTYLE_5IN,
            seed,
            duration_s: planned_duration_s,
            throttle: thr,
            alt_agl: agl,
            x,
            y,
            vel: DVec3::ZERO,
            wind_cfg: None,
            terrain: self.terrain.clone(),
        };
        match CoreFlight::new(setup, std::path::Path::new(&record_path.to_string())) {
            Ok(f) => {
                self.flight = Some(f);
                true
            }
            Err(e) => {
                godot_error!("DarterQuad: record open: {e}");
                false
            }
        }
    }

    /// Drive `n` more 250 Hz ticks (clamped at the planned duration's tick
    /// count). Returns the empty string on success, else the failure text —
    /// errors cannot panic across the FFI boundary.
    #[func]
    fn advance_ticks(&mut self, n: i64) -> String {
        if n < 0 {
            godot_error!("DarterQuad: advance_ticks got negative {n}");
            return format!("advance_ticks got negative {n}");
        }
        let Some(f) = self.flight.as_mut() else {
            godot_error!("DarterQuad: advance_ticks before record_open");
            return "advance_ticks before record_open".into();
        };
        match f.advance_ticks(n.unsigned_abs() as usize) {
            Ok(()) => String::new(),
            Err(e) => {
                godot_error!("DarterQuad: advance: {e}");
                e
            }
        }
    }

    /// Commit the record and return its 16-hex fnv1a64 hash — the value
    /// sim_run prints and the parity test compares byte-for-byte records
    /// against. Empty string on failure (a godot error was logged).
    #[func]
    fn finish_record(&mut self) -> String {
        match self.flight.as_mut() {
            None => {
                godot_error!("DarterQuad: finish_record with no open record");
                String::new()
            }
            Some(f) => match f.finish() {
                Ok(stats) => format!("{:016x}", stats.record_hash),
                Err(e) => {
                    godot_error!("DarterQuad: record finish: {e}");
                    String::new()
                }
            },
        }
    }

    /// Live view of the flight for the render loop: position and velocity in
    /// metres, orientation quaternion, per-motor rpm, state of charge and
    /// bus voltage; `soc` -1.0 means no flight is open.
    #[func]
    fn state_dict(&mut self) -> Dictionary<GString, Variant> {
        let mut d = Dictionary::new();
        match self.flight.as_mut() {
            None => {
                d.set("soc", -1.0);
                d.set("ticks_done", 0);
            }
            Some(f) => {
                let (pos, vel, quat, ticks) = f.state();
                d.set("pos", Vector3::new(pos.x as f32, pos.y as f32, pos.z as f32));
                d.set("vel", Vector3::new(vel.x as f32, vel.y as f32, vel.z as f32));
                d.set(
                    "quat",
                    Quaternion::new(quat.x as f32, quat.y as f32, quat.z as f32, quat.w as f32),
                );
                let rpm = to_godot_packed(&f.rpm_view());
                d.set("rpm", &rpm);
                d.set("soc", f.soc_view() as f32);
                d.set("vbus", f.vbus_view() as f32);
                d.set("ticks_done", ticks as i64);
            }
        }
        d
    }

    /// The tick period the record's time axis runs on, exposed for the
    /// renderer's pacing arithmetic (250 Hz ticks at 4 ms).
    #[func]
    fn tick_dt(&self) -> f64 {
        TICK_DT
    }
}

fn to_godot_packed(v: &[f64]) -> PackedFloat32Array {
    PackedFloat32Array::from(v.iter().map(|x| *x as f32).collect::<Vec<f32>>())
}