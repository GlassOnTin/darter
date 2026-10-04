//! Darter live plant: the darter-core physics flight as a Godot class.
//!
//! `DarterQuad` wraps darter_core::flight::CoreFlight — the exact loop
//! `sim_run --mode core` runs (250 Hz ticks of 32 x 125 us substeps, one
//! darter_record sample per tick) — per-call, so GDScript drives the flight
//! live: spawn, throttle, drive ticks, read state, hash up the record.
//! The acceptance contract is tests/godot_live.rs: a flight driven through
//! this class is byte-identical to the same flight through sim_run.
//!
//! M2b adds `DarterFlyer`: the whole closed-mode runner (darter_core::flyer,
//! lifted verbatim from sim_run's closed mode) as a class — `flyer_start`
//! spawns the Betaflight SITL child and applies the profile over MSP CLI,
//! `flyer_pump` drives 250 Hz ticks in chunks (each wall-paced inside),
//! `flyer_state_dict` samples the flight for a render loop, `flyer_finish`
//! closes the record. Its contract with sim_run is shared behavior, not byte
//! parity: the SITL's own PID runs on wall-clock time, so identical inputs
//! vary run to run (tests/godot_flyer.rs is the acceptance suite).
//!
//! Panics never cross the FFI boundary: every fallible call reports via
//! godot_error! plus a boolean / empty-string / empty-state return, and the
//! probe scene treats those as a failure.

use darter_core::flight::{CoreFlight, CoreSetup, TICK_DT};
use darter_core::flyer::{Flyer, FlyerConfig, LiveInput, LiveInputSource};
use darter_core::preset::Preset;
use darter_core::sensor::SensorConfig;
use darter_core::terrain::TerrainGrid;
use darter_core::DVec3;
use godot::global::JoyAxis;
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

/// The closed-flyer runner (darter_core::flyer) as a Godot class: one flight
/// from SITL spawn to closed record, driven per-call the same way sim_run
/// --mode closed runs it. `flyer_start` → pump chunks → `flyer_state_dict`
/// between them → `flyer_finish`. Only one flight lives on an instance; a
/// finished one is cleared, so `flyer_start` again is legal.
#[derive(GodotClass)]
#[class(base = RefCounted)]
struct DarterFlyer {
    /// The flight in progress; `flyer_finish` consumes and clears it. Every
    /// method guards on None instead of unwrapping across the FFI boundary.
    flyer: Option<Flyer>,
    /// Simulation input for the next (and, sticky, every) `flyer_start`:
    /// the sensor model, set by `flyer_set_sensors`. Consumed-config pattern
    /// DarterQuad's `setup`/`set_throttle` use, minus the per-flight clear —
    /// a persistent instance keeps its configured inputs across flights.
    sensors_pending: Option<SensorConfig>,
    /// Likewise sticky: `flyer_set_radio_pocket(true)` wires the next (and
    /// every) `flyer_start` to read the radio's joypad axes from Godot's
    /// Input singleton — the M2d RadioMaster Pocket HID path.
    radio_pocket: bool,
}

#[godot_api]
impl IRefCounted for DarterFlyer {
    fn init(_base: Base<RefCounted>) -> Self {
        Self {
            flyer: None,
            sensors_pending: None,
            radio_pocket: false,
        }
    }
}

#[godot_api]
impl DarterFlyer {
    /// Spawn the SITL child, apply + diff-verify the profile, open the
    /// record, arm. Returns the empty string on success, else the failure
    /// text — errors cannot panic across the FFI boundary. `profile` "" =
    /// the runner default; a nonpositive `*_until` = stick held for the
    /// whole flight (GDScript cannot express infinity). Record is written as
    /// `flight.jsonl` under `work_dir`, which is also the SITL's fresh cwd.
    #[func]
    fn flyer_start(
        &mut self,
        sitl_bin: GString,
        work_dir: GString,
        duration_s: f64,
        seed: i64,
        throttle: f64,
        yaw: f64,
        yaw_until: f64,
        profile: GString,
    ) -> String {
        if self.flyer.is_some() {
            let m = "flyer_start while a flight is already running".to_string();
            godot_error!("DarterFlyer: {m}");
            return m;
        }
        let seed = match u64::try_from(seed) {
            Ok(s) => s,
            Err(_) => {
                let m = format!("seed must be non-negative (got {seed})");
                godot_error!("DarterFlyer: {m}");
                return m;
            }
        };
        let work = work_dir.to_string();
        if work.is_empty() {
            let m = "flyer_start: empty work dir".to_string();
            godot_error!("DarterFlyer: {m}");
            return m;
        }
        let profile = profile.to_string();
        let mut cfg = FlyerConfig::new(sitl_bin.to_string(), seed, duration_s);
        cfg.throttle = throttle;
        cfg.yaw = yaw;
        cfg.yaw_until = if yaw_until > 0.0 { yaw_until } else { f64::INFINITY };
        cfg.profile = if profile.is_empty() { None } else { Some(profile) };
        if let Some(mut s) = self.sensors_pending {
            // sim_run's --sensors seed rule, class-side: the sensor model
            // follows the run seed (the class has no separate seed spec, so
            // there is nothing to leave pinned).
            s.seed = seed;
            cfg.sensor_cfg = Some(s);
        }
        if self.radio_pocket {
            cfg.input = Some(LiveInputSource {
                name: "pocket_hid",
                provide: Box::new(|| pocket_input()),
            });
        }
        match Flyer::start(cfg, std::path::Path::new(&work), "flight.jsonl") {
            Ok(f) => {
                self.flyer = Some(f);
                String::new()
            }
            Err(e) => {
                godot_error!("DarterFlyer: flyer_start: {e}");
                e
            }
        }
    }

    /// Enable (true) or disable (false) the sensor model for the next
    /// flights — the equivalent of sim_run's bare `--sensors`:
    /// `SensorConfig::DEFAULT`, its seed following each flight's run seed
    /// (see flyer_start). Sticky until changed.
    #[func]
    fn flyer_set_sensors(&mut self, enabled: bool) {
        self.sensors_pending = if enabled { Some(SensorConfig::DEFAULT) } else { None };
    }

    /// Route (or unroute) `flyer_start`'s stick input through the
    /// RadioMaster Pocket HID mapping (M2d): each pumped tick, the runner
    /// consults a hook that reads Godot's Input singleton's device-0 axis
    /// state — the same table the physical radio feeds — instead of the
    /// sim-time stick script. Sticky across flights, like
    /// `flyer_set_sensors`.
    #[func]
    fn flyer_set_radio_pocket(&mut self, enabled: bool) {
        self.radio_pocket = enabled;
    }

    /// Advance at most `n` more 250 Hz ticks (no-op past the plan). Returns
    /// the empty string on success, else the failure text.
    #[func]
    fn flyer_pump(&mut self, n: i64) -> String {
        if n < 0 {
            let m = format!("flyer_pump got negative {n}");
            godot_error!("DarterFlyer: {m}");
            return m;
        }
        let Some(f) = self.flyer.as_mut() else {
            let m = "flyer_pump before flyer_start".to_string();
            godot_error!("DarterFlyer: {m}");
            return m;
        };
        match f.pump(n.unsigned_abs() as usize) {
            Ok(()) => String::new(),
            Err(e) => {
                godot_error!("DarterFlyer: flyer_pump: {e}");
                e
            }
        }
    }

    /// Live view of the flight for the render loop: position and velocity in
    /// metres, orientation quaternion, state of charge and bus voltage
    /// (`soc` -1.0 means no flight is open), sim time the ARM box went up
    /// (null before arming), progress counters, max altitude so far.
    #[func]
    fn flyer_state_dict(&mut self) -> Dictionary<GString, Variant> {
        let mut d = Dictionary::new();
        match self.flyer.as_ref() {
            None => {
                d.set("soc", -1.0);
                d.set("vbus", -1.0);
                d.set("armed_at", &Variant::nil());
                d.set("ticks_done", 0);
                d.set("ticks_total", 0);
                d.set("max_alt", 0.0);
            }
            Some(f) => {
                let s = f.snapshot();
                d.set("pos", Vector3::new(s.pos.x as f32, s.pos.y as f32, s.pos.z as f32));
                d.set("vel", Vector3::new(s.vel.x as f32, s.vel.y as f32, s.vel.z as f32));
                d.set(
                    "quat",
                    Quaternion::new(
                        s.quat.x as f32,
                        s.quat.y as f32,
                        s.quat.z as f32,
                        s.quat.w as f32,
                    ),
                );
                d.set("soc", s.soc as f32);
                d.set("vbus", s.vbus as f32);
                match s.armed_at {
                    Some(t) => d.set("armed_at", t),
                    None => d.set("armed_at", &Variant::nil()),
                }
                d.set("ticks_done", s.ticks_done as i64);
                d.set("ticks_total", s.ticks_total as i64);
                d.set("max_alt", s.max_alt);
            }
        }
        d
    }

    /// Stop the telemetry thread, close the record and return its 16-hex
    /// fnv1a64 hash — the same value sim_run prints for a closed run. Empty
    /// string on failure (a godot error was logged). The flight is consumed;
    /// the SITL child is killed here so the fixed MSP port frees.
    #[func]
    fn flyer_finish(&mut self) -> String {
        let Some(f) = self.flyer.as_mut() else {
            godot_error!("DarterFlyer: flyer_finish with no flight running");
            return String::new();
        };
        let result = f.finish();
        self.flyer = None; // record closed; drop kills the child, frees the port
        match result {
            Ok(st) => format!("{:016x}", st.record_hash),
            Err(e) => {
                godot_error!("DarterFlyer: record finish: {e}");
                String::new()
            }
        }
    }
}

fn to_godot_packed(v: &[f64]) -> PackedFloat32Array {
    PackedFloat32Array::from(v.iter().map(|x| *x as f32).collect::<Vec<f32>>())
}

/// One tick of the RadioMaster Pocket HID mapping, read live from Godot's
/// Input singleton. Measured against the physical radio over USB (guided
/// captures, 2026-10-04, eight rounds; scratch logs not committed). The
/// axis slots below are Input.get_joy_axis indexes for this device — Godot
/// passes raw HID report slots through unremapped, so they are NOT the
/// standard controller semantics (the right stick is slots 0/1, the left
/// stick LR is slot 2; there is no LEFT_Y on this device):
///
///   slot 0  right-stick LR   -1 left .. +1 right   (aileron -> roll)
///   slot 1  right-stick UD   +1 = stick away       (elevator -> pitch)
///   slot 2  left-stick  LR   -1 left .. +1 right   (rudder -> yaw)
///   slot 3  SA switch        -1 / 0 / +1; +1 = arm end (-> aux3 2000 us)
///   slot 4  throttle         0 at bottom .. +1 at top
///   slot 5  SB switch        -1 / 0 / +1
///
/// SC (3-pos) and the SF trim dial are HID-silent; SD = button 1; SE, the
/// only push button, = button 0 — none of these feed the RC path. The
/// same facts are in tests/godot_flyer.rs's pocket test (the mapping's
/// acceptance test) and docs/physics.md section 11.
fn pocket_input() -> LiveInput {
    let input = godot::classes::Input::singleton();
    // Device 0, read unconditionally — NOT gated on get_connected_joypads.
    // Measured (gdext scratch rig, 2026-10-04): synthetic JoypadMotion axis
    // state persists in Godot's axis table, but a synthetic device NEVER
    // appears in the connected-pads list, so a pad-list gate would make the
    // padless CI probe read neutral forever. With no events injected the
    // table reads 0.0, so a radioless phone still lands in the plain arm
    // timeout (SA never reaches +1) — the single-Pocket assumption is
    // device 0; naming/enumeration of other pads is a later rung.
    let slot =
        |n: i64| -> JoyAxis { JoyAxis::try_from_ord(n as i32).unwrap_or(JoyAxis::INVALID) };
    let ax = |n: i64| -> f64 { input.get_joy_axis(0, slot(n)) as f64 };
    let sa = ax(3); // -1 / 0 / +1; the +1 end is arm
    let aux3_us = (1500.0_f64 + sa * 500.0).clamp(1000.0, 2000.0) as u16;
    LiveInput {
        roll: ax(0),
        pitch: ax(1),
        yaw: ax(2),
        // Measured range: slot 4 sits at 0 at the throttle's bottom, +1 at
        // the top — not the +-1 span the sticks swing.
        throttle: ax(4).clamp(0.0, 1.0),
        aux3_us,
    }
}