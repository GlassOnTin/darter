//! Headless flight harness: runs the physics core alone (core mode, for
//! determinism and calibration) or the full closed loop against a spawned
//! Betaflight SITL child (closed mode), recording both to the darter_record
//! JSONL format plus a summary.json of facts and provenance.
//!
//! The closed-mode runner itself lives in darter_core::flyer (M2b), shared
//! verbatim with the Godot DarterFlyer class; its contract notes are there.
//!
//! Loop-time instrumentation: per-tick wall duration is collected and written
//! to summary.json as p50/p99/max (VISION M0: frame-time instrumentation from
//! the first commit).

use std::io::{BufWriter, Write};
use std::path::{Path, PathBuf};

use darter_core::air::RHO_0;
use darter_core::flight::{CoreFlight, CoreSetup, SUBSTEP_DT, TICK_DT};
use darter_core::flyer::{DEFAULT_SITL_BIN, DEFAULT_THROTTLE, Flyer, FlyerConfig};
use darter_core::preset::Preset;
use darter_core::quad::hover_throttle;
use darter_core::sha256::sha256_hex;
use darter_core::sensor::SensorConfig;
use darter_core::terrain::TerrainGrid;
use darter_core::wind::WindConfig;
use darter_core::DVec3;

mod track;

/// Default closed duration covers the 5 s boot grace, the arming window,
/// the throttle ramp, and ~5 s of powered flight.
const CLOSED_DEFAULT_DURATION: f64 = 12.0;

struct Args {
    mode: String,
    seed: u64,
    duration: Option<f64>,
    out: String,
    profile: Option<String>,
    determinism_check: bool,
    bin: String,
    throttle: f64,
    /// Core-mode throttle override: Some means `--throttle` was given, so
    /// core mode scripts that value instead of the solved hover throttle
    /// (0 = motors-off tests).
    core_throttle: Option<f64>,
    /// Core-mode spawn altitude in m (default 0.5 m, just off the ground).
    alt: f64,
    /// Core-mode spawn offsets in m (default 0): x east, y north, relative
    /// to the origin. Lets a scripted core flight start inside an area pack
    /// (the record frame is the same ENU frame the packs use).
    x: f64,
    y: f64,
    /// Core-mode initial velocity in m/s (default zero): with hover throttle
    /// the quad coasts level while drag bleeds the speed off (~1% / s for
    /// the calibrated 5" drag), which is the honest physics of a level
    /// transit — no attitude controller exists in core mode.
    vx: f64,
    vy: f64,
    vz: f64,
    sensors: bool,
    sensor_cfg: Option<SensorConfig>,
    wind_cfg: Option<WindConfig>,
    /// Yaw stick value (Fly phase), for yaw-axis stability probes.
    yaw: f64,
    /// Sim time after which the yaw stick returns to zero.
    yaw_until: f64,
    /// Pitch stick value (Fly phase), for pitch-axis stability probes.
    pitch: f64,
    /// Sim time after which the pitch stick returns to zero.
    pitch_until: f64,
    /// Roll stick value (Fly phase), for roll-axis stability probes.
    roll: f64,
    /// Sim time after which the roll stick returns to zero.
    roll_until: f64,
    /// Feed the GPS-stale sentinel the SITL looks for (sitl.c skips
    /// setVirtualGPS when |lat|>90 or |lon|>180): the virtual GPS never
    /// fixes, isolating baro+inertial altitude behaviour.
    gps_stale: bool,
    /// Path to a terrain grid sidecar (area_pack.py's terrain.bin): ground
    /// contact follows the DEM heights, --alt/--x/--y stay AGL offsets in
    /// the record convention. None = flat ground at z = 0. A missing or bad
    /// file is a hard error, never a silent flat fallback.
    terrain: Option<String>,
    /// Path to a darter_track JSON (schema v1, see file track.rs): the flown
    /// record is evaluated post-hoc against the gate planes and the events
    /// land in summary.json. A missing or bad file is a hard error before
    /// anything flies. None = no track evaluation.
    track: Option<String>,
}

fn parse_args() -> Result<Args, String> {
    let mut a = Args {
        mode: "core".into(),
        seed: 1,
        duration: None,
        out: "run".into(),
        profile: None,
        determinism_check: false,
        bin: DEFAULT_SITL_BIN.into(),
        throttle: DEFAULT_THROTTLE,
        core_throttle: None,
        alt: 0.5,
        x: 0.0,
        y: 0.0,
        vx: 0.0,
        vy: 0.0,
        vz: 0.0,
        sensors: false,
        sensor_cfg: None,
        wind_cfg: None,
        yaw: 0.0,
        yaw_until: f64::INFINITY,
        pitch: 0.0,
        pitch_until: f64::INFINITY,
        roll: 0.0,
        roll_until: f64::INFINITY,
        gps_stale: false,
        terrain: None,
        track: None,
    };
    // Set when the --sensors spec pinned a seed, so a bare --sensors follows
    // the run seed (same --seed reproduces the same noise stream).
    let mut sensor_seed_given = false;
    // Same convention for --wind.
    let mut wind_seed_given = false;
    let mut it = std::env::args().skip(1);
    while let Some(arg) = it.next() {
        let mut val = |name: &str| -> Result<String, String> {
            it.next().ok_or_else(|| format!("--{name} needs a value"))
        };
        match arg.as_str() {
            "--mode" => a.mode = val("mode")?,
            "--seed" => a.seed = val("seed")?.parse().map_err(|_| "--seed wants an integer")?,
            "--duration" => {
                a.duration = Some(val("duration")?.parse().map_err(|_| "--duration wants seconds")?)
            }
            "--out" => a.out = val("out")?,
            "--profile" => a.profile = Some(val("profile")?),
            "--bin" => a.bin = val("bin")?,
            "--yaw" => a.yaw = val("yaw")?.parse().map_err(|_| "--yaw wants -1..1")?,
            "--yaw-until" => {
                a.yaw_until = val("yaw-until")?.parse().map_err(|_| "--yaw-until wants seconds")?
            }
            "--pitch" => a.pitch = val("pitch")?.parse().map_err(|_| "--pitch wants -1..1")?,
            "--pitch-until" => {
                a.pitch_until = val("pitch-until")?.parse().map_err(|_| "--pitch-until wants seconds")?
            }
            "--roll" => a.roll = val("roll")?.parse().map_err(|_| "--roll wants -1..1")?,
            "--roll-until" => {
                a.roll_until = val("roll-until")?.parse().map_err(|_| "--roll-until wants seconds")?
            }
            "--throttle" => {
                a.throttle = val("throttle")?.parse().map_err(|_| "--throttle wants 0..1")?;
                a.core_throttle = Some(a.throttle);
            }
            "--alt" => a.alt = val("alt")?.parse().map_err(|_| "--alt wants metres")?,
            "--x" => a.x = val("x")?.parse().map_err(|_| "--x wants metres")?,
            "--y" => a.y = val("y")?.parse().map_err(|_| "--y wants metres")?,
            "--vx" => a.vx = val("vx")?.parse().map_err(|_| "--vx wants m/s")?,
            "--vy" => a.vy = val("vy")?.parse().map_err(|_| "--vy wants m/s")?,
            "--vz" => a.vz = val("vz")?.parse().map_err(|_| "--vz wants m/s")?,
            "--determinism-check" => a.determinism_check = true,
            "--gps-stale" => a.gps_stale = true,
            "--terrain" => a.terrain = Some(val("terrain")?),
            "--track" => a.track = Some(val("track")?),
            s if s == "--wind" || s.starts_with("--wind=") => {
                // Bare --wind: standard weather (mean calm, W20 moderate).
                let mut cfg = WindConfig {
                    mean: DVec3::ZERO,
                    w20_ms: 15.43,
                    seed: 1,
                };
                if let Some(spec) = arg.strip_prefix("--wind=") {
                    for kv in spec.split(',') {
                        let (k, v) = kv.split_once('=')
                            .ok_or_else(|| format!("--wind spec wants key=value, got {kv}"))?;
                        match k {
                            "ex" => cfg.mean.x = v.parse().map_err(|_| format!("--wind {k} wants f64"))?,
                            "ny" => cfg.mean.y = v.parse().map_err(|_| format!("--wind {k} wants f64"))?,
                            "uz" => cfg.mean.z = v.parse().map_err(|_| format!("--wind {k} wants f64"))?,
                            "w20" => cfg.w20_ms = v.parse().map_err(|_| format!("--wind {k} wants f64"))?,
                            "seed" => {
                                cfg.seed = v.parse().map_err(|_| "--wind seed wants u64")?;
                                wind_seed_given = true;
                                continue;
                            }
                            other => return Err(format!("--wind unknown key {other}")),
                        }
                    }
                }
                a.wind_cfg = Some(cfg);
            }
            s if s == "--sensors" || s.starts_with("--sensors=") => {
                a.sensors = true;
                let mut cfg = a.sensor_cfg.unwrap_or(SensorConfig::DEFAULT);
                if let Some(spec) = arg.strip_prefix("--sensors=") {
                    for kv in spec.split(',') {
                        let (k, v) = kv.split_once('=')
                            .ok_or_else(|| format!("--sensors spec wants key=value, got {kv}"))?;
                        let f = match k {
                            "gyro_noise" => &mut cfg.gyro_noise_std,
                            "gyro_bias" => &mut cfg.gyro_bias_std,
                            "gyro_rw" => &mut cfg.gyro_bias_rw_std,
                            "accel_noise" => &mut cfg.accel_noise_std,
                            "vib_accel" => &mut cfg.vib_accel_amp,
                            "vib_gyro" => &mut cfg.vib_gyro_amp,
                            "vib2" => &mut cfg.vib2_scale,
                            "seed" => {
                                cfg.seed = v.parse().map_err(|_| "--sensors seed wants u64")?;
                                sensor_seed_given = true;
                                continue;
                            }
                            other => return Err(format!("--sensors unknown key {other}")),
                        };
                        *f = v.parse().map_err(|_| format!("--sensors {k} wants f64"))?;
                    }
                }
                a.sensor_cfg = Some(cfg);
            }
            other => return Err(format!("unknown argument {other}")),
        }
    }
    // The sensor model follows the run seed unless the spec pinned one, so
    // the same --seed reproduces the same noise stream (recorded inputs).
    if let Some(cfg) = &mut a.sensor_cfg {
        if !sensor_seed_given {
            cfg.seed = a.seed;
        }
    }
    if let Some(cfg) = &mut a.wind_cfg {
        if !wind_seed_given {
            cfg.seed = a.seed;
        }
    }
    if a.mode != "core" && a.mode != "closed" {
        return Err(format!("--mode must be core or closed, got {}", a.mode));
    }
    if a.determinism_check && a.mode != "core" {
        return Err("--determinism-check is core-only (closed loops are not bit-reproducible)".into());
    }
    let scripted_spawn = a.x != 0.0 || a.y != 0.0 || a.vx != 0.0 || a.vy != 0.0 || a.vz != 0.0;
    if scripted_spawn && a.mode != "core" {
        return Err("--x/--y/--vx/--vy/--vz are core-only (closed mode is flown by the FC)".into());
    }
    Ok(a)
}

struct SitlInfo {
    bin: String,
    sha256: String,
    version: String,
    #[allow(dead_code)] // the applied lines ride the record header; summary keeps only the verdict below
    profile: Vec<String>,
    profile_readback_ok: bool,
}

/// Provenance for the terrain grid, when one flew (summary.json only — the
/// record header is a fixed wire contract and stays untouched).
struct TerrainInfo {
    path: String,
    sha256: String,
}

/// Provenance + results for the track, when one was evaluated
/// (summary.json only). Events are computed post-hoc from the written
/// record — physics never sees the track.
struct TrackInfo {
    path: String,
    sha256: String,
    name: String,
    is_loop: bool,
    n_cps: usize,
    events: Vec<(usize, f64)>,
    lap_splits: Option<Vec<f64>>,
}

struct RunOutcome {
    record_hash: u64,
    ticks: usize,
    wall_s: f64,
    final_alt: f64,
    max_alt: f64,
    final_soc: f64,
    final_vbus: f64,
    final_i_bus: f64,
    rpm_end: [f64; 4],
    att_samples: usize,
    /// Sim time the ARM box first registered as armed (flight_flags bit 0).
    armed_at_s: Option<f64>,
    status_samples: Vec<(f64, u32, u32)>, // t, arming_disable, flight_flags
    loop_p50_ms: f64,
    loop_p99_ms: f64,
    loop_max_ms: f64,
    // Closed mode only.
    servo_packets: u64,
    msp_errors: u64,
    sensors: bool,
    wind: bool,
    sitl: Option<SitlInfo>,
    /// Set only when the run flew with a terrain grid (`--terrain`).
    terrain: Option<TerrainInfo>,
    /// Set only when the run evaluated a track (`--track`).
    track: Option<TrackInfo>,
}

fn run() -> Result<(), String> {
    let mut args = parse_args()?;
    // Resolve the per-mode default duration up front so the summary records
    // what actually ran.
    if args.duration.is_none() {
        args.duration = Some(if args.mode == "core" { 2.0 } else { CLOSED_DEFAULT_DURATION });
    }
    // Load the terrain grid before anything flies: a bad --terrain path is a
    // hard error here, not a silent flat fallback mid-run.
    let (terrain, terrain_info) = match &args.terrain {
        Some(path) => {
            let bytes =
                std::fs::read(path).map_err(|e| format!("terrain {path}: read: {e}"))?;
            let grid = TerrainGrid::parse(&bytes).map_err(|e| format!("terrain {path}: {e}"))?;
            println!(
                "[sim_run] terrain {path}: grid {}x{}, step {:.0} m, z [{:.1}, {:.1}]",
                grid.cols, grid.rows, grid.step, grid.z_min, grid.z_max
            );
            let info = TerrainInfo {
                path: path.clone(),
                sha256: sha256_hex(&bytes),
            };
            (Some(grid), Some(info))
        }
        None => (None, None),
    };
    // Same load-before-fly contract for the track file. It feeds only the
    // post-hoc event evaluation below — physics and record bytes never see
    // it, so --track cannot change a flight.
    let (track, track_prov) = match &args.track {
        Some(path) => {
            let bytes = std::fs::read(path).map_err(|e| format!("track {path}: read: {e}"))?;
            let t = track::parse(&bytes).map_err(|e| format!("track {path}: {e}"))?;
            println!(
                "[sim_run] track {path}: \"{}\", {} checkpoints, {}, spawn {}",
                t.name,
                t.cps.len(),
                if t.is_loop { "loop" } else { "open" },
                match t.spawn {
                    Some([x, y, z]) => format!("({x:.1}, {y:.1}, {z:.1})"),
                    None => "none".to_string(),
                }
            );
            (Some(t), Some((path.clone(), sha256_hex(&bytes))))
        }
        None => (None, None),
    };

    let mut outcome = match args.mode.as_str() {
        "core" => {
            let o1 = run_core(&args, "flight.jsonl", terrain.as_ref())?;
            if args.determinism_check {
                let o2 = run_core(&args, "flight2.jsonl", terrain.as_ref())?;
                if o1.record_hash != o2.record_hash {
                    return Err(format!(
                        "determinism check FAILED: hashes differ {:016x} vs {:016x}",
                        o1.record_hash, o2.record_hash
                    ));
                }
                println!("[sim_run] determinism check passed: both runs {:016x}", o1.record_hash);
            }
            o1
        }
        "closed" => run_closed(&args, terrain.as_ref())?,
        _ => unreachable!(),
    };
    outcome.terrain = terrain_info;

    let out_dir = PathBuf::from(&args.out);
    // Events are computed post-hoc from the written record — with
    // --determinism-check only flight.jsonl is measured (flight2.jsonl is
    // written, evaluated, hashed, and its data discarded for output).
    if let Some(t) = &track {
        let (path, sha256) = track_prov.expect("track provenance held alongside the flight");
        let rows = track::read_record_positions(&out_dir.join("flight.jsonl"))
            .map_err(|e| format!("track {path}: {e}"))?;
        let (events, splits) = track::compute_events(t, &rows);
        outcome.track = Some(TrackInfo {
            path,
            sha256,
            name: t.name.clone(),
            is_loop: t.is_loop,
            n_cps: t.cps.len(),
            events,
            // Open tracks report no laps at all (null); a loop reports an
            // array, which is empty on a one-way transit with a single
            // start crossing.
            lap_splits: if t.is_loop { Some(splits) } else { None },
        });
    }
    write_summary(&out_dir, &args, &outcome)?;
    println!(
        "[sim_run] done: {} ticks in {:.2}s wall, record hash {:016x}, final alt {:.3} m, p99 loop {:.2} ms",
        outcome.ticks, outcome.wall_s, outcome.record_hash, outcome.final_alt, outcome.loop_p99_ms
    );
    Ok(())
}

fn main() {
    if let Err(e) = run() {
        eprintln!("sim_run: {e}");
        std::process::exit(1);
    }
}

/// Core-mode flight: scripted throttle (the solved hover value, or
/// `--throttle` when given — 0 = motors-off tests), no SITL, record every
/// tick. Identical inputs produce identical record hashes. With terrain the
/// spawn z is AGL over the DEM height (--alt stays an above-ground offset)
/// and ground contact is the grid. The loop itself lives in
/// darter_core::flight so the Godot extension flies the identical shape.
fn run_core(
    args: &Args,
    record_name: &str,
    terrain: Option<&TerrainGrid>,
) -> Result<RunOutcome, String> {
    let duration = args.duration.unwrap_or(2.0);
    let out_dir = PathBuf::from(&args.out);
    std::fs::create_dir_all(&out_dir).map_err(|e| format!("out dir: {e}"))?;

    let preset = Preset::FREESTYLE_5IN;
    let thr = args
        .core_throttle
        .unwrap_or_else(|| hover_throttle(&preset, preset.battery, 1.0, RHO_0));
    println!("[sim_run] core mode: throttle {:.4}, {} s", thr, duration);
    let mut flight = CoreFlight::new(
        CoreSetup {
            preset,
            seed: args.seed,
            duration_s: duration,
            throttle: args.core_throttle,
            alt_agl: args.alt,
            x: args.x,
            y: args.y,
            vel: DVec3::new(args.vx, args.vy, args.vz),
            wind_cfg: args.wind_cfg.clone(),
            terrain: terrain.cloned(),
        },
        &out_dir.join(record_name),
    )?;
    flight.advance_ticks((duration / TICK_DT).floor() as usize)?;
    let st = flight.finish()?;

    Ok(RunOutcome {
        record_hash: st.record_hash,
        ticks: st.ticks,
        wall_s: st.wall_s,
        final_alt: st.final_alt,
        max_alt: st.max_alt,
        final_soc: st.final_soc,
        final_vbus: st.final_vbus,
        final_i_bus: st.final_i_bus,
        rpm_end: st.rpm_end,
        att_samples: 0,
        armed_at_s: None,
        status_samples: Vec::new(),
        loop_p50_ms: st.loop_p50_ms,
        loop_p99_ms: st.loop_p99_ms,
        loop_max_ms: st.loop_max_ms,
        servo_packets: 0,
        msp_errors: 0,
        sensors: false,
        wind: args.wind_cfg.is_some(),
        sitl: None,
        terrain: None,
        track: None,
    })
}

/// Closed-mode flight: the runner now lives in darter_core::flyer (lifted
/// from this binary in M2b so the Godot DarterFlyer class drives the
/// identical code); this wrapper only maps the CLI args onto FlyerConfig.
fn run_closed(args: &Args, terrain: Option<&TerrainGrid>) -> Result<RunOutcome, String> {
    let duration = args.duration.unwrap_or(CLOSED_DEFAULT_DURATION);
    let out_dir = PathBuf::from(&args.out);
    let cfg = FlyerConfig {
        sitl_bin: args.bin.clone().into(),
        profile: args.profile.clone(),
        seed: args.seed,
        duration_s: duration,
        throttle: args.throttle,
        yaw: args.yaw,
        yaw_until: args.yaw_until,
        pitch: args.pitch,
        pitch_until: args.pitch_until,
        roll: args.roll,
        roll_until: args.roll_until,
        gps_stale: args.gps_stale,
        sensor_cfg: args.sensor_cfg.clone(),
        wind_cfg: args.wind_cfg,
        terrain: terrain.cloned(),
    };
    let mut flyer = Flyer::start(&cfg, &out_dir, "flight.jsonl")?;
    flyer.pump((duration / TICK_DT).floor() as usize)?;
    let st = flyer.finish()?;
    Ok(RunOutcome {
        record_hash: st.record_hash,
        ticks: st.ticks,
        wall_s: st.wall_s,
        final_alt: st.final_alt,
        max_alt: st.max_alt,
        final_soc: st.final_soc,
        final_vbus: st.final_vbus,
        final_i_bus: st.final_i_bus,
        rpm_end: st.rpm_end,
        att_samples: st.att_samples,
        armed_at_s: st.armed_at_s,
        status_samples: st.status_samples,
        loop_p50_ms: st.loop_p50_ms,
        loop_p99_ms: st.loop_p99_ms,
        loop_max_ms: st.loop_max_ms,
        servo_packets: st.servo_packets,
        msp_errors: st.msp_errors,
        sensors: st.sensors,
        wind: st.wind,
        sitl: Some(SitlInfo {
            bin: st.sitl.bin,
            sha256: st.sitl.sha256,
            version: st.sitl.version,
            profile: st.sitl.profile,
            profile_readback_ok: st.sitl.profile_readback_ok,
        }),
        terrain: None,
        track: None,
    })
}

/// summary.json: run facts, provenance, and the quantitative acceptance
/// numbers. Hand-formatted like the record (fixed field order).
fn write_summary(out_dir: &Path, args: &Args, o: &RunOutcome) -> Result<(), String> {
    let have_track = o.track.is_some();
    let mut s = String::with_capacity(2048);
    s.push_str("{\n");
    s.push_str(&format!("  \"mode\": \"{}\",\n", args.mode));
    s.push_str(&format!("  \"seed\": {},\n", args.seed));
    s.push_str(&format!(
        "  \"duration_s\": {:.3},\n",
        args.duration.unwrap_or(0.0)
    ));
    s.push_str(&format!("  \"preset\": \"{}\",\n", Preset::FREESTYLE_5IN.name));
    s.push_str(&format!("  \"tick_dt_s\": {TICK_DT},\n"));
    s.push_str(&format!("  \"substep_dt_s\": {SUBSTEP_DT},\n"));
    s.push_str(&format!("  \"ticks\": {},\n", o.ticks));
    s.push_str(&format!("  \"wall_s\": {:.3},\n", o.wall_s));
    s.push_str(&format!("  \"record_hash\": \"0x{:016x}\",\n", o.record_hash));
    s.push_str("  \"final\": {\n");
    s.push_str(&format!("    \"alt_m\": {:.6},\n", o.final_alt));
    s.push_str(&format!("    \"soc\": {:.6},\n", o.final_soc));
    s.push_str(&format!("    \"vbus_v\": {:.4},\n", o.final_vbus));
    s.push_str(&format!("    \"i_bus_a\": {:.4},\n", o.final_i_bus));
    s.push_str(&format!(
        "    \"rpm\": [{:.1}, {:.1}, {:.1}, {:.1}]\n",
        o.rpm_end[0], o.rpm_end[1], o.rpm_end[2], o.rpm_end[3]
    ));
    s.push_str("  },\n");
    s.push_str(&format!("  \"max_alt_m\": {:.6},\n", o.max_alt));
    s.push_str("  \"loop_ms\": {\n");
    s.push_str(&format!("    \"p50\": {:.3},\n", o.loop_p50_ms));
    s.push_str(&format!("    \"p99\": {:.3},\n", o.loop_p99_ms));
    s.push_str(&format!("    \"max\": {:.3}\n", o.loop_max_ms));
    s.push_str("  },\n");
    s.push_str(&format!("  \"sensors\": {},\n", o.sensors));
    s.push_str(&format!("  \"wind\": {},\n", o.wind));
    s.push_str(&format!("  \"att_samples\": {},\n", o.att_samples));
    match o.armed_at_s {
        Some(t) => s.push_str(&format!("  \"armed_at_s\": {:.3},\n", t)),
        None => s.push_str("  \"armed_at_s\": null,\n"),
    }
    s.push_str("  \"status_samples\": [");
    for (i, (t, arm, flags)) in o.status_samples.iter().enumerate() {
        if i > 0 {
            s.push_str(", ");
        }
        s.push_str(&format!("[{:.3}, {}, {}]", t, arm, flags));
    }
    s.push_str("],\n");
    if let Some(sitl) = &o.sitl {
        s.push_str(&format!("  \"servo_packets\": {},\n", o.servo_packets));
        s.push_str(&format!("  \"msp_errors\": {},\n", o.msp_errors));
        s.push_str("  \"sitl\": {\n");
        s.push_str(&format!("    \"bin\": \"{}\",\n", json_escape(&sitl.bin)));
        s.push_str(&format!("    \"sha256\": \"{}\",\n", json_escape(&sitl.sha256)));
        s.push_str(&format!("    \"version\": \"{}\",\n", json_escape(&sitl.version)));
        s.push_str(&format!(
            "    \"profile_readback_ok\": {}\n",
            sitl.profile_readback_ok
        ));
        if o.terrain.is_some() || have_track {
            s.push_str("  },\n");
        } else {
            s.push_str("  }\n");
        }
    } else if o.terrain.is_none() && !have_track {
        // Trim the trailing comma from the status_samples line.
        if s.ends_with("],\n") {
            s.pop();
            s.pop();
        }
        s.push_str("\n");
    }
    if let Some(terr) = &o.terrain {
        s.push_str("  \"terrain\": {\n");
        s.push_str(&format!("    \"path\": \"{}\",\n", json_escape(&terr.path)));
        s.push_str(&format!("    \"sha256\": \"{}\"\n", json_escape(&terr.sha256)));
        // The track block always follows when present, so the terrain
        // object closes with a comma for it. The comma lives on the close
        // line only — the last key line must never carry a trailing comma.
        if have_track {
            s.push_str("  },\n");
        } else {
            s.push_str("  }\n");
        }
    }
    if let Some(trk) = &o.track {
        s.push_str("  \"track\": {\n");
        s.push_str(&format!(
            "    \"schema\": \"darter_track\", \"version\": 1, \"name\": \"{}\", \"path\": \"{}\",\n",
            json_escape(&trk.name),
            json_escape(&trk.path)
        ));
        s.push_str(&format!(
            "    \"sha256\": \"{}\", \"checkpoints\": {}, \"loop\": {},\n",
            json_escape(&trk.sha256),
            trk.n_cps,
            trk.is_loop
        ));
        s.push_str("    \"events\": [");
        for (i, (idx, t)) in trk.events.iter().enumerate() {
            if i > 0 {
                s.push_str(", ");
            }
            s.push_str(&format!("[{}, {:.3}]", idx, t));
        }
        s.push_str("], ");
        match &trk.lap_splits {
            Some(splits) => {
                s.push_str("\"lap_splits\": [");
                for (i, t) in splits.iter().enumerate() {
                    if i > 0 {
                        s.push_str(", ");
                    }
                    s.push_str(&format!("{:.3}", t));
                }
                s.push_str("]\n");
            }
            None => s.push_str("\"lap_splits\": null\n"),
        }
        s.push_str("  }\n");
    }
    s.push_str("}\n");
    let path = out_dir.join("summary.json");
    let mut f = BufWriter::new(std::fs::File::create(&path).map_err(|e| format!("summary: {e}"))?);
    f.write_all(s.as_bytes()).map_err(|e| format!("summary: {e}"))?;
    f.flush().map_err(|e| format!("summary: {e}"))?;
    println!("[sim_run] summary: {}", path.display());
    Ok(())
}

fn json_escape(s: &str) -> String {
    let mut out = String::with_capacity(s.len() + 2);
    for c in s.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            _ => out.push(c),
        }
    }
    out
}