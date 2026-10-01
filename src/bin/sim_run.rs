//! Headless flight harness: runs the physics core alone (core mode, for
//! determinism and calibration) or the full closed loop against a spawned
//! Betaflight SITL child (closed mode), recording both to the darter_record
//! JSONL format plus a summary.json of facts and provenance.
//!
//! Closed-mode contract:
//! - The SITL binary is spawned as a child in a FRESH working directory
//!   (<out>/sitl_cwd): the SITL writes eeprom.bin into its cwd, so a fresh
//!   directory boots factory defaults and profile application cannot leak
//!   between runs.
//! - The config profile is applied over the MSPv2 CLI **without save** (RAM
//!   only), then read back with `diff`; the run refuses to start flying until
//!   every profile line is present in the readback.
//! - Loop shape: 250 Hz fdm/RC ticks, 32 x 125 us core substeps per tick,
//!   wall-clock paced. MSP telemetry runs on a dedicated thread that owns
//!   the link (the SITL serves exactly one TCP client per UART — verified
//!   serial_tcp.c:76): ATTITUDE+MOTOR at 25 Hz, STATUS at 4 Hz. Polls cost
//!   15-30 ms each on this build, far over the 4 ms tick budget, so inline
//!   polling stretched the loop to 2x wall time (observed 2026-09-28);
//!   off-thread the flight loop keeps real time.
//! - RC script is a state machine driven by observed FC status, not a fixed
//!   timeline: hold the arm box DOWN until arming_disable clears (the FC
//!   blocks arming for pwr_on_arm_grace = 5 s after boot; holding the box
//!   active during any disable flag latches ARMING_DISABLED_ARM_SWITCH,
//!   observed), raise the box, wait for ARM box + flags clear, then ramp
//!   throttle (default 0.16, near hover) over 0.5 s.
//!
//! Loop-time instrumentation: per-tick wall duration is collected and written
//! to summary.json as p50/p99/max (VISION M0: frame-time instrumentation from
//! the first commit).

use std::io::{self, BufWriter, Write};
use std::net::{Ipv4Addr, SocketAddr, TcpStream};
use std::path::{Path, PathBuf};
use std::process::{Child, Command, Stdio};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use darter_core::air::RHO_0;
use darter_core::msp::{
    decode_attitude, decode_estimated_altitude, decode_motor, decode_raw_imu, decode_status,
    MspLink, MSP_ALTITUDE, MSP_ATTITUDE, MSP_MOTOR, MSP_RAW_IMU, MSP_STATUS,
};
use darter_core::preset::Preset;
use darter_core::quad::{hover_throttle, Quad};
use darter_core::record::{FcSample, RecordHeader, RecordWriter, Sample, SitlProvenance};
use darter_core::sha256::sha256_hex;
use darter_core::sensor::{SensorConfig, SensorModel};
use darter_core::sitl::{fdm_from_state, fdm_from_state_imu, rc_packet, SimLink};
use darter_core::terrain::{Ground, TerrainGrid};
use darter_core::wind::{WindConfig, WindModel};
use darter_core::DVec3;

const SUBSTEP_DT: f64 = 125e-6; // 8 kHz core step
const SUBSTEPS_PER_TICK: usize = 32; // 4 ms tick = 250 Hz fdm/RC
const TICK_DT: f64 = SUBSTEP_DT * SUBSTEPS_PER_TICK as f64;

const MSP_ADDR: &str = "127.0.0.1:5761";
const SITL_MSP_PORT: u16 = 5761;

const DEFAULT_BIN: &str = "/tmp/betaflight/obj/betaflight_2026.12.0-alpha_SITL";
const DEFAULT_PROFILE: &str = "aux 0 0 2 1700 2100 0 0";
const DEFAULT_THROTTLE: f64 = 0.16; // closed-loop scripted throttle (hover ~0.155 at full pack)
const ARMED_US: u16 = 2000;
const DISARMED_US: u16 = 1000;

/// Default closed duration covers the 5 s boot grace, the arming window,
/// the throttle ramp, and ~5 s of powered flight.
const CLOSED_DEFAULT_DURATION: f64 = 12.0;

/// Telemetry cadence (dedicated thread).
const TELEM_ATT_PERIOD: Duration = Duration::from_millis(40); // 25 Hz
const TELEM_STATUS_PERIOD: Duration = Duration::from_millis(250); // 4 Hz

/// Arming state-machine timeouts (sim time).
const GRACE_TIMEOUT_S: f64 = 20.0; // grace clears at 5 s FC time; 4x slack
const ARM_TIMEOUT_S: f64 = 10.0; // from box-up to armed

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
}

fn parse_args() -> Result<Args, String> {
    let mut a = Args {
        mode: "core".into(),
        seed: 1,
        duration: None,
        out: "run".into(),
        profile: None,
        determinism_check: false,
        bin: DEFAULT_BIN.into(),
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
    profile: Vec<String>,
    profile_readback_ok: bool,
}

/// Provenance for the terrain grid, when one flew (summary.json only — the
/// record header is a fixed wire contract and stays untouched).
struct TerrainInfo {
    path: String,
    sha256: String,
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
/// and ground contact is the grid.
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
    // AGL convention kept: --alt is an offset above the ground under (x, y).
    // Flat h is a literal 0.0, so the no-terrain spawn is bit-identical.
    let mut quad = Quad::new(
        preset,
        DVec3::new(args.x, args.y, args.alt + terrain.map_or(0.0, |g| g.h_at(args.x, args.y))),
    );
    if let Some(grid) = terrain {
        quad.ground = Ground::Grid(grid.clone());
    }
    quad.throttle = [thr; 4];
    // Scripted initial velocity (level transit; see the --vx doc comment).
    if args.vx != 0.0 || args.vy != 0.0 || args.vz != 0.0 {
        quad.state.vel = DVec3::new(args.vx, args.vy, args.vz);
    }
    let mut wind = args.wind_cfg.map(WindModel::new);
    if let Some(cfg) = &wind {
        quad.wind = cfg.config().mean;
    }

    let mut record = RecordWriter::create(&out_dir.join(record_name))
        .map_err(|e| format!("record: {e}"))?;
    record
        .write_header(&RecordHeader {
            mode: "core",
            seed: args.seed,
            duration_s: duration,
            preset: preset.name,
            profile: {
                let mut p = vec![
                    format!("throttle={thr:.6}"),
                    format!("alt={:.3}", args.alt),
                    format!("spawn=({:.3},{:.3})", args.x, args.y),
                ];
                if args.vx != 0.0 || args.vy != 0.0 || args.vz != 0.0 {
                    p.push(format!("vel=({:.3},{:.3},{:.3})", args.vx, args.vy, args.vz));
                }
                p
            },
            sitl: None,
            sensors: None,
            wind: args.wind_cfg,
        })
        .map_err(|e| format!("record header: {e}"))?;

    let total_ticks = (duration / TICK_DT).floor() as usize;
    let started = Instant::now();
    let mut loop_ms = Vec::with_capacity(total_ticks);
    let mut max_alt = 0.0f64;
    for tick in 0..total_ticks {
        let tick_start = Instant::now();
        // Wind advances once per tick at the current altitude, then holds
        // across the 32 substeps (the gust filter runs at tick rate).
        if let Some(w) = wind.as_mut() {
            quad.wind = w.step(TICK_DT, quad.state.pos.z);
        }
        for _ in 0..SUBSTEPS_PER_TICK {
            quad.step(SUBSTEP_DT);
        }
        let t = (tick + 1) as f64 * TICK_DT;
        // No sensor model yet (T3): the quad state is the telemetry, no FC.
        write_sample(&mut record, t, &quad, wind.is_some().then_some(quad.wind), None)
            .map_err(|e| format!("record: {e}"))?;
        max_alt = max_alt.max(quad.state.pos.z);
        loop_ms.push(tick_start.elapsed().as_secs_f64() * 1e3);
    }
    let wall_s = started.elapsed().as_secs_f64();
    let record_hash = record.finish().map_err(|e| format!("record finish: {e}"))?;

    loop_ms.sort_by(|a, b| a.partial_cmp(b).unwrap());
    Ok(RunOutcome {
        record_hash,
        ticks: total_ticks,
        wall_s,
        final_alt: quad.state.pos.z,
        max_alt,
        final_soc: quad.soc(),
        final_vbus: quad.bus_voltage(),
        final_i_bus: quad.bus_current(),
        rpm_end: quad.rpm,
        att_samples: 0,
        armed_at_s: None,
        status_samples: Vec::new(),
        loop_p50_ms: pct(&loop_ms, 0.50),
        loop_p99_ms: pct(&loop_ms, 0.99),
        loop_max_ms: *loop_ms.last().unwrap_or(&0.0),
        servo_packets: 0,
        msp_errors: 0,
        sensors: false,
        wind: args.wind_cfg.is_some(),
        sitl: None,
        terrain: None,
    })
}

fn write_sample(
    w: &mut RecordWriter,
    t: f64,
    quad: &Quad,
    wind: Option<DVec3>,
    fc: Option<FcSample>,
) -> io::Result<()> {
    w.write_sample(&Sample {
        t,
        pos: quad.state.pos,
        vel: quad.state.vel,
        quat: quad.state.quat,
        omega: quad.state.omega,
        rpm: quad.rpm,
        i_mot: quad.i_mot,
        vbus: quad.bus_voltage(),
        soc: quad.soc(),
        wind,
        fc,
    })
}

/// Closed-mode flight: spawn + supervise a SITL child, apply the profile in
/// RAM, read it back, fly the scripted RC against the closed loop, record.
fn run_closed(args: &Args, terrain: Option<&TerrainGrid>) -> Result<RunOutcome, String> {
    let duration = args.duration.unwrap_or(8.0);
    let out_dir = PathBuf::from(&args.out);
    std::fs::create_dir_all(&out_dir).map_err(|e| format!("out dir: {e}"))?;

    // Provenance: hash of the binary that will run.
    let bin_bytes =
        std::fs::read(&args.bin).map_err(|e| format!("SITL binary {}: {e}", args.bin))?;
    let bin_sha = sha256_hex(&bin_bytes);
    println!("[sim_run] SITL binary {} sha256 {bin_sha}", args.bin);

    // A leftover SITL on the fixed ports would eat our fdm/RC/MSP traffic.
    if TcpStream::connect_timeout(
        &SocketAddr::from((Ipv4Addr::LOCALHOST, SITL_MSP_PORT)),
        Duration::from_millis(200),
    )
    .is_ok()
    {
        return Err(format!("port {SITL_MSP_PORT} already in use: another SITL is running; stop it first"));
    }

    // Fresh working directory -> factory defaults (eeprom.bin lives in cwd).
    let sitl_cwd = out_dir.join("sitl_cwd");
    if sitl_cwd.exists() {
        std::fs::remove_dir_all(&sitl_cwd).map_err(|e| format!("sitl cwd wipe: {e}"))?;
    }
    std::fs::create_dir(&sitl_cwd).map_err(|e| format!("sitl cwd: {e}"))?;
    let log_out = std::fs::File::create(out_dir.join("sitl_stdout.log"))
        .map_err(|e| format!("sitl log: {e}"))?;
    let log_err = log_out.try_clone().map_err(|e| format!("sitl log: {e}"))?;

    let mut proc_guard = SitlProc {
        child: Some(
            Command::new(&args.bin)
                .current_dir(&sitl_cwd)
                .stdout(Stdio::from(log_out))
                .stderr(Stdio::from(log_err))
                .spawn()
                .map_err(|e| format!("spawn SITL: {e}"))?,
        ),
    };
    let pid = proc_guard.child.as_ref().unwrap().id();
    println!("[sim_run] SITL pid {pid}, cwd {}", sitl_cwd.display());

    let mut link = wait_msp_ready(&mut proc_guard)?;

    // Profile: RAM only (no save; the fresh cwd has defaults on disk anyway).
    let profile: Vec<String> = args
        .profile
        .as_deref()
        .unwrap_or(DEFAULT_PROFILE)
        .split(';')
        .map(|l| l.trim().to_string())
        .filter(|l| !l.is_empty() && !l.starts_with('#'))
        .collect();
    for line in &profile {
        link.cli(line).map_err(|e| format!("profile apply {line:?}: {e}"))?;
    }

    // Readback before flying: the profile must be verifiably live.
    let diff = link.cli("diff").map_err(|e| format!("diff readback: {e}"))?;
    let version = link.cli("version").map_err(|e| format!("version readback: {e}"))?;
    let mut profile_readback_ok = true;
    for line in &profile {
        if !diff.contains(line.as_str()) {
            eprintln!("[sim_run] profile line missing from diff readback: {line}");
            profile_readback_ok = false;
        }
    }
    let version_line = version
        .lines()
        .find(|l| l.contains("Betaflight /"))
        .unwrap_or("")
        .trim()
        .to_string();
    if version_line.is_empty() {
        return Err(format!("no version line in CLI readback: {version:?}"));
    }
    println!("[sim_run] readback: {version_line}");
    if !profile_readback_ok {
        return Err("profile readback failed; refusing to fly with unverified config".into());
    }

    let info = SitlInfo {
        bin: args.bin.clone(),
        sha256: bin_sha,
        version: version_line,
        profile,
        profile_readback_ok,
    };
    fly_closed(args, link, duration, &out_dir, info, terrain)
}

/// Latest-snapshot store for the telemetry thread.
#[derive(Default)]
struct TelemState {
    latest: Option<FcSample>,
    seq: u64, // bumped whenever `latest` changes
    status: Vec<(f64, u32, u32)>,
    errors: u64,
}

/// MSP telemetry thread: owns the link (the SITL serves one TCP client per
/// UART) and polls ATTITUDE+MOTOR at 25 Hz and STATUS at 4 Hz. Each request
/// costs 15-30 ms on this build — far over the 4 ms tick budget — so the
/// flight loop only reads this thread's shared snapshot. Runs until `stop`.
fn spawn_telemetry(
    mut link: MspLink,
    started: Instant,
    stop: Arc<AtomicBool>,
) -> (Arc<Mutex<TelemState>>, std::thread::JoinHandle<()>) {
    let shared = Arc::new(Mutex::new(TelemState::default()));
    let sh = Arc::clone(&shared);
    let handle = std::thread::spawn(move || {
        let mut next_att = started + TELEM_ATT_PERIOD;
        let mut next_status = started + TELEM_STATUS_PERIOD;
        let norm = |v: u16| ((v as f64 - 1000.0) / 1000.0).clamp(0.0, 1.0) as f32;
        loop {
            if stop.load(Ordering::Relaxed) {
                break;
            }
            let now = Instant::now();
            if now >= next_att {
                next_att += TELEM_ATT_PERIOD;
                let att = link.request_v1(MSP_ATTITUDE).map(|p| decode_attitude(&p));
                let motors = link.request_v1(MSP_MOTOR).map(|p| decode_motor(&p));
                let imu = link.request_v1(MSP_RAW_IMU).map(|p| decode_raw_imu(&p));
                let alt = link.request_v1(MSP_ALTITUDE).map(|p| decode_estimated_altitude(&p));
                let mut st = sh.lock().unwrap();
                if let (Ok(Some(a)), Ok(m), Ok(Some(i)), Ok(Some(alt))) = (att, motors, imu, alt) {
                    let (arm, flags) = st.status.last().map(|s| (s.1, s.2)).unwrap_or((0, 0));
                    st.latest = Some(FcSample {
                        att_cdeg: [a.roll_decdeg as i32, a.pitch_decdeg as i32, a.yaw_deg as i32],
                        motors: [
                            norm(*m.first().unwrap_or(&1000)),
                            norm(*m.get(1).unwrap_or(&1000)),
                            norm(*m.get(2).unwrap_or(&1000)),
                            norm(*m.get(3).unwrap_or(&1000)),
                        ],
                        gyro_raw: [
                            i.gyro_raw[0] as f64,
                            i.gyro_raw[1] as f64,
                            i.gyro_raw[2] as f64,
                        ],
                        alt_cm: alt.alt_cm,
                        vario_cms: alt.vario_cms,
                        arming_disable: arm,
                        flight_flags: flags,
                    });
                    st.seq += 1;
                } else {
                    st.errors += 1;
                }
            }
            if now >= next_status {
                next_status += TELEM_STATUS_PERIOD;
                match link.request_v1(MSP_STATUS) {
                    Ok(p) => {
                        let mut st = sh.lock().unwrap();
                        match decode_status(&p) {
                            Some(s) => {
                                let t = started.elapsed().as_secs_f64();
                                st.status.push((t, s.arming_disable_flags, s.flight_flags));
                            }
                            None => st.errors += 1,
                        }
                    }
                    Err(_) => {
                        sh.lock().unwrap().errors += 1;
                    }
                }
            }
            std::thread::sleep(Duration::from_millis(5));
        }
    });
    (shared, handle)
}

#[derive(Clone, Copy, PartialEq)]
enum ArmPhase {
    WaitGrace,
    WaitArmed,
    Fly,
}

/// Throttle ramp after the ARM box goes up.
const SETTLE_S: f64 = 0.25;
const RAMP_S: f64 = 0.5;

/// Fly the closed loop from the already-configured link, which is moved into
/// the telemetry thread. Split from run_closed so the spawn/config plumbing
/// stays readable.
fn fly_closed(
    args: &Args,
    link: MspLink,
    duration: f64,
    out_dir: &Path,
    sitl_info: SitlInfo,
    terrain: Option<&TerrainGrid>,
) -> Result<RunOutcome, String> {
    let sim_link = SimLink::new().map_err(|e| format!("udp bind: {e}"))?;

    let mut record =
        RecordWriter::create(&out_dir.join("flight.jsonl")).map_err(|e| format!("record: {e}"))?;
    record
        .write_header(&RecordHeader {
            mode: "closed",
            seed: args.seed,
            duration_s: duration,
            preset: Preset::FREESTYLE_5IN.name,
            profile: sitl_info.profile.clone(),
            sitl: Some(SitlProvenance {
                path: sitl_info.bin.clone(),
                sha256: sitl_info.sha256.clone(),
                version: sitl_info.version.clone(),
            }),
            sensors: args.sensor_cfg.clone(),
            wind: args.wind_cfg,
        })
        .map_err(|e| format!("record header: {e}"))?;

    let started = Instant::now();
    let stop = Arc::new(AtomicBool::new(false));
    let (telem, telem_handle) = spawn_telemetry(link, started, Arc::clone(&stop));

    let run_res = fly_loop(args, record, duration, &sim_link, &telem, terrain);
    stop.store(true, Ordering::Relaxed);
    if telem_handle.join().is_err() {
        eprintln!("[sim_run] telemetry thread panicked");
    }
    let telem_state = telem.lock().unwrap();
    let (status_samples, msp_errors) = (telem_state.status.clone(), telem_state.errors);
    drop(telem_state);
    run_res.map(|lo| {
        let wall_s = started.elapsed().as_secs_f64();
        let mut loop_ms = lo.loop_ms;
        loop_ms.sort_by(|a, b| a.partial_cmp(b).unwrap());
        RunOutcome {
            record_hash: lo.record_hash,
            ticks: (duration / TICK_DT).floor() as usize,
            wall_s,
            final_alt: lo.final_alt,
            max_alt: lo.max_alt,
            final_soc: lo.final_soc,
            final_vbus: lo.final_vbus,
            final_i_bus: lo.final_i_bus,
            rpm_end: lo.rpm_end,
            att_samples: lo.att_samples,
            armed_at_s: lo.armed_at,
            status_samples,
            loop_p50_ms: pct(&loop_ms, 0.50),
            loop_p99_ms: pct(&loop_ms, 0.99),
            loop_max_ms: *loop_ms.last().unwrap_or(&0.0),
            servo_packets: lo.servo_packets,
            msp_errors,
            sensors: args.sensors,
            wind: args.wind_cfg.is_some(),
            sitl: Some(sitl_info),
            terrain: None,
        }
    })
}

/// What the flight loop itself measured and produced.
struct LoopOutcome {
    record_hash: u64,
    att_samples: usize,
    armed_at: Option<f64>,
    max_alt: f64,
    final_alt: f64,
    final_soc: f64,
    final_vbus: f64,
    final_i_bus: f64,
    rpm_end: [f64; 4],
    servo_packets: u64,
    loop_ms: Vec<f64>,
}

/// The flight loop proper: RC arming state machine, 8 kHz substeps over UDP,
/// servo feedback, record writing, wall pacing. With terrain the closed-mode
/// spawn sits half a metre above the DEM height at the origin; ground
/// contact follows the grid from the first substep.
fn fly_loop(
    args: &Args,
    mut record: RecordWriter,
    duration: f64,
    sim_link: &SimLink,
    telem: &Arc<Mutex<TelemState>>,
    terrain: Option<&TerrainGrid>,
) -> Result<LoopOutcome, String> {
    // --gps-stale feeds an out-of-range lat/lon the SITL treats as the GPS
    // sentinel (sitl.c: skip the update so the virtual GPS goes stale).
    let (origin_lat, origin_lon) = if args.gps_stale { (999.0, 999.0) } else { (47.6, -122.3) };
    let total_ticks = (duration / TICK_DT).floor() as usize;
    let mut loop_ms = Vec::with_capacity(total_ticks);
    let mut max_alt = 0.0f64;
    let mut att_samples = 0usize;
    let mut last_seq = 0u64;
    let mut servo_packets = 0u64;
    let mut phase = ArmPhase::WaitGrace;
    let mut phase_entered = 0.0f64;
    let mut armed_at: Option<f64> = None;
    let spawn_z = 0.5 + terrain.map_or(0.0, |g| g.h_at(0.0, 0.0));
    let mut quad = Quad::new(Preset::FREESTYLE_5IN, DVec3::new(0.0, 0.0, spawn_z));
    if let Some(grid) = terrain {
        quad.ground = Ground::Grid(grid.clone());
    }
    // Sensor model seeded from the run seed; None keeps the fdm path
    // bit-identical to the pre-sensor harness.
    let mut sensor = args.sensor_cfg.as_ref().map(|c| SensorModel::new(*c));
    // Wind advances once per tick; None keeps the physics path bit-identical
    // to the pre-wind harness (Quad::wind stays zero).
    let mut wind = args.wind_cfg.map(WindModel::new);
    if let Some(cfg) = &wind {
        quad.wind = cfg.config().mean;
    }

    for tick in 0..total_ticks {
        let tick_start = Instant::now();
        let t = (tick + 1) as f64 * TICK_DT;
        if let Some(w) = wind.as_mut() {
            quad.wind = w.step(TICK_DT, quad.state.pos.z);
        }

        // Latest FC status drives the arming state machine.
        let (arm, flags) = {
            let st = telem.lock().unwrap();
            st.status.last().map(|s| (s.1, s.2)).unwrap_or((u32::MAX, 0))
        };
        let (thr, aux3, yaw, pitch, roll, next_phase) = match phase {
            ArmPhase::WaitGrace => {
                if t - phase_entered > GRACE_TIMEOUT_S {
                    return Err(format!(
                        "arming grace never cleared: arming_disable={arm:#x} at t={t:.1}"
                    ));
                }
                if arm == 0 {
                    (0.0, DISARMED_US, 0.0, 0.0, 0.0, Some(ArmPhase::WaitArmed))
                } else {
                    (0.0, DISARMED_US, 0.0, 0.0, 0.0, None)
                }
            }
            ArmPhase::WaitArmed => {
                if t - phase_entered > ARM_TIMEOUT_S {
                    return Err(format!(
                        "did not arm: arming_disable={arm:#x} flight_flags={flags:#x} at t={t:.1}"
                    ));
                }
                if arm == 0 && flags & 1 != 0 {
                    (0.0, ARMED_US, 0.0, 0.0, 0.0, Some(ArmPhase::Fly))
                } else {
                    (0.0, ARMED_US, 0.0, 0.0, 0.0, None)
                }
            }
            ArmPhase::Fly => {
                let dt = t - armed_at.unwrap_or(0.0);
                let thr = if dt < SETTLE_S {
                    0.0
                } else if dt < SETTLE_S + RAMP_S {
                    args.throttle * (dt - SETTLE_S) / RAMP_S
                } else {
                    args.throttle
                };
                let yaw = if t < args.yaw_until { args.yaw } else { 0.0 };
                let pitch = if t < args.pitch_until { args.pitch } else { 0.0 };
                let roll = if t < args.roll_until { args.roll } else { 0.0 };
                (thr, ARMED_US, yaw, pitch, roll, None)
            }
        };
        if let Some(next) = next_phase {
            if next == ArmPhase::Fly {
                armed_at = Some(t);
                println!("[sim_run] armed at t={t:.2} s");
            }
            phase = next;
            phase_entered = t;
        }
        sim_link
            .send_rc(&rc_packet(roll, pitch, yaw, thr, aux3))
            .map_err(|e| format!("send rc: {e}"))?;

        for _ in 0..SUBSTEPS_PER_TICK {
            quad.step(SUBSTEP_DT);
            let pkt = match sensor.as_mut() {
                Some(s) => {
                    let (g, a) = quad.imu();
                    let rpm_mean = quad.rpm.iter().sum::<f64>() / 4.0;
                    let thr_mean = quad.throttle.iter().sum::<f64>() / 4.0;
                    let (gn, an) = s.step(SUBSTEP_DT, g, a, rpm_mean, thr_mean);
                    fdm_from_state_imu(&quad, gn, an, origin_lat, origin_lon, t)
                }
                None => fdm_from_state(&quad, origin_lat, origin_lon, t),
            };
            sim_link.send_fdm(&pkt).map_err(|e| format!("send fdm: {e}"))?;
        }
        let (motors, n) = sim_link.try_recv_motors();
        servo_packets += n;
        if let Some(m) = motors {
            for i in 0..4 {
                quad.throttle[i] = (m.motor_speed[i] as f64).clamp(0.0, 1.0);
            }
        }

        // Record the newest telemetry snapshot once per tick (the thread
        // produces ~25 Hz; the record keeps every new snapshot).
        let fc = {
            let st = telem.lock().unwrap();
            if st.seq != last_seq {
                last_seq = st.seq;
                att_samples += 1;
                st.latest.clone()
            } else {
                None
            }
        };

        write_sample(&mut record, t, &quad, wind.is_some().then_some(quad.wind), fc)
            .map_err(|e| format!("record: {e}"))?;
        max_alt = max_alt.max(quad.state.pos.z);

        // Wall-clock pacing: sleep off what is left of this 4 ms tick.
        let elapsed = tick_start.elapsed();
        let budget = Duration::from_secs_f64(TICK_DT);
        if elapsed < budget {
            std::thread::sleep(budget - elapsed);
        }
        loop_ms.push(elapsed.as_secs_f64() * 1e3);
    }
    let record_hash = record.finish().map_err(|e| format!("record finish: {e}"))?;
    Ok(LoopOutcome {
        record_hash,
        att_samples,
        armed_at,
        max_alt,
        final_alt: quad.state.pos.z,
        final_soc: quad.soc(),
        final_vbus: quad.bus_voltage(),
        final_i_bus: quad.bus_current(),
        rpm_end: quad.rpm,
        servo_packets,
        loop_ms,
    })
}

/// Owns the SITL child process; kills it on drop so a failed run cannot
/// leave an orphan holding the ports.
struct SitlProc {
    child: Option<Child>,
}

impl Drop for SitlProc {
    fn drop(&mut self) {
        if let Some(mut c) = self.child.take() {
            let _ = c.kill();
            let _ = c.wait();
        }
    }
}

/// Poll-connect to the SITL's MSP serial bridge until it accepts (up to 15 s),
/// then confirm STATUS answers. A child that exits early is reported, not
/// waited on blindly.
fn wait_msp_ready(guard: &mut SitlProc) -> Result<MspLink, String> {
    let deadline = Instant::now() + Duration::from_secs(15);
    loop {
        if let Ok(l) = MspLink::connect(MSP_ADDR) {
            let mut l = l;
            match l.request_v1(MSP_STATUS) {
                Ok(payload) => {
                    if decode_status(&payload).is_none() {
                        return Err("STATUS answered but the payload does not decode".into());
                    }
                    return Ok(l);
                }
                Err(_) => {
                    // Port open before the firmware serves MSP: drop and retry.
                    drop(l);
                }
            }
        }
        if let Some(c) = guard.child.as_mut() {
            if let Ok(Some(status)) = c.try_wait() {
                return Err(format!("SITL exited early: {status}"));
            }
        }
        if Instant::now() > deadline {
            return Err("timed out waiting for the SITL MSP port".into());
        }
        std::thread::sleep(Duration::from_millis(100));
    }
}

fn pct(sorted: &[f64], q: f64) -> f64 {
    if sorted.is_empty() {
        return 0.0;
    }
    let i = ((q * (sorted.len() - 1) as f64) as usize).min(sorted.len() - 1);
    sorted[i]
}

/// summary.json: run facts, provenance, and the quantitative acceptance
/// numbers. Hand-formatted like the record (fixed field order).
fn write_summary(out_dir: &Path, args: &Args, o: &RunOutcome) -> Result<(), String> {
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
        if o.terrain.is_some() {
            s.push_str("  },\n");
        } else {
            s.push_str("  }\n");
        }
    } else if o.terrain.is_none() {
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