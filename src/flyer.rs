//! The closed-loop flight runner: spawn + supervise a real Betaflight SITL
//! child, apply the config profile over the MSPv2 CLI with a diff-readback
//! gate, fly the scripted-RC arming state machine over the two links, and
//! record the darter_record schema. Lifted verbatim from sim_run's closed
//! mode (M2b) so the Godot GDExtension class `DarterFlyer` (tools/gdext) and
//! `sim_run --mode closed` drive the identical runner; the CLI is now a thin
//! wrapper.
//!
//! Contract notes carried from the sim_run harness:
//! - The SITL binary is spawned as a child in a FRESH working directory
//!   (<out>/sitl_cwd): it writes eeprom.bin into its cwd, so a fresh
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
//! NOT reproducible: the SITL's own PID loop runs on wall-clock time, so its
//! dt carries UDP jitter and identical seeds vary run-to-run (section 11 of
//! docs/physics.md, tests/sitl_loop.rs). Callers compare behaviour, never
//! bytes, across closed-mode runs.

use std::net::{Ipv4Addr, SocketAddr, TcpStream};
use std::path::{Path, PathBuf};
use std::process::{Child, Command, Stdio};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use crate::flight::{pct, write_sample, SUBSTEPS_PER_TICK, SUBSTEP_DT, TICK_DT};
use crate::msp::{
    decode_attitude, decode_estimated_altitude, decode_motor, decode_raw_imu, decode_status,
    MspLink, MSP_ALTITUDE, MSP_ATTITUDE, MSP_MOTOR, MSP_RAW_IMU, MSP_STATUS,
};
use crate::preset::Preset;
use crate::quad::Quad;
use crate::record::{FcSample, RecordHeader, RecordWriter, SitlProvenance};
use crate::sha256::sha256_hex;
use crate::sensor::{SensorConfig, SensorModel};
use crate::sitl::{fdm_from_state, fdm_from_state_imu, rc_packet, SimLink};
use crate::terrain::{Ground, TerrainGrid};
use crate::wind::{WindConfig, WindModel};
use crate::{DQuat, DVec3};

/// The SITL's MSP serial bridge (one TCP client per UART).
pub const MSP_ADDR: &str = "127.0.0.1:5761";
/// The MSP TCP port a leftover SITL would hold; start() refuses when busy.
pub const SITL_MSP_PORT: u16 = 5761;

/// Default SITL binary (a `make SITL_TARGET` build of upstream Betaflight
/// checked out under /tmp/betaflight); the gdext caller overrides it.
pub const DEFAULT_SITL_BIN: &str = "/tmp/betaflight/obj/betaflight_2026.12.0-alpha_SITL";
/// Default profile: arms AUX3 (RC7) at 1700-2100 us — the scripted arm box.
pub const DEFAULT_PROFILE: &str = "aux 0 0 2 1700 2100 0 0";
/// Closed-loop scripted throttle, near hover (~0.155 at full pack). sim_run
/// --throttle overrides it for core mode too.
pub const DEFAULT_THROTTLE: f64 = 0.16;

const ARMED_US: u16 = 2000;
const DISARMED_US: u16 = 1000;

/// Telemetry cadence (dedicated thread).
const TELEM_ATT_PERIOD: Duration = Duration::from_millis(40); // 25 Hz
const TELEM_STATUS_PERIOD: Duration = Duration::from_millis(250); // 4 Hz

/// Arming state-machine timeouts (sim time).
const GRACE_TIMEOUT_S: f64 = 20.0; // grace clears at 5 s FC time; 4x slack
const ARM_TIMEOUT_S: f64 = 10.0; // from box-up to armed

/// Throttle ramp after the ARM box goes up.
const SETTLE_S: f64 = 0.25;
const RAMP_S: f64 = 0.5;

/// What start() established and read back from the running SITL, carried
/// into the record header and returned to the caller for summary reporting.
#[derive(Clone)]
pub struct SitlFacts {
    pub bin: String,
    pub sha256: String,
    pub version: String,
    /// The profile lines actually applied (split + # lines filtered out).
    pub profile: Vec<String>,
    pub profile_readback_ok: bool,
}

/// Everything a closed flight needs before it starts. Sim-time stick scripts
/// (`yaw` until `yaw_until`, etc.) drive the Fly phase; `*_until =
/// f64::INFINITY` means "stick held for the whole flight".
pub struct FlyerConfig {
    pub sitl_bin: PathBuf,
    /// ';'-joined profile lines; None = DEFAULT_PROFILE.
    pub profile: Option<String>,
    pub seed: u64,
    pub duration_s: f64,
    /// Fly-phase throttle (0..1), reached over SETTLE_S + RAMP_S.
    pub throttle: f64,
    pub yaw: f64,
    pub yaw_until: f64,
    pub pitch: f64,
    pub pitch_until: f64,
    pub roll: f64,
    pub roll_until: f64,
    /// Feed the GPS-stale sentinel the SITL looks for (sitl.c skips
    /// setVirtualGPS when |lat|>90 or |lon|>180), isolating baro+inertial
    /// altitude behaviour.
    pub gps_stale: bool,
    pub sensor_cfg: Option<SensorConfig>,
    pub wind_cfg: Option<WindConfig>,
    /// DEM sidecar (already parsed): spawn half a metre above the height at
    /// the origin and follow the grid for ground contact. None = flat at 0.
    pub terrain: Option<TerrainGrid>,
}

impl FlyerConfig {
    /// sim_run's closed-mode defaults for everything but the values the
    /// caller must always name.
    pub fn new(sitl_bin: impl Into<PathBuf>, seed: u64, duration_s: f64) -> Self {
        Self {
            sitl_bin: sitl_bin.into(),
            profile: None,
            seed,
            duration_s,
            throttle: DEFAULT_THROTTLE,
            yaw: 0.0,
            yaw_until: f64::INFINITY,
            pitch: 0.0,
            pitch_until: f64::INFINITY,
            roll: 0.0,
            roll_until: f64::INFINITY,
            gps_stale: false,
            sensor_cfg: None,
            wind_cfg: None,
            terrain: None,
        }
    }
}

/// One caller-readable instant of the flight.
#[derive(Clone, Copy)]
pub struct FlyerSnapshot {
    pub pos: DVec3,
    pub vel: DVec3,
    pub quat: DQuat,
    pub soc: f64,
    pub vbus: f64,
    pub ticks_done: usize,
    pub ticks_total: usize,
    pub armed_at: Option<f64>,
    pub max_alt: f64,
}

/// What finish() measured and produced. sim_run maps this field-for-field
/// onto its RunOutcome (summary.json), the gdext class hands the caller a
/// dictionary.
pub struct FlyerStats {
    pub record_hash: u64,
    pub ticks: usize,
    pub wall_s: f64,
    pub final_alt: f64,
    pub max_alt: f64,
    pub final_soc: f64,
    pub final_vbus: f64,
    pub final_i_bus: f64,
    pub rpm_end: [f64; 4],
    pub att_samples: usize,
    /// Sim time the ARM box first registered as armed (flight_flags bit 0).
    pub armed_at_s: Option<f64>,
    pub status_samples: Vec<(f64, u32, u32)>, // t, arming_disable, flight_flags
    pub loop_p50_ms: f64,
    pub loop_p99_ms: f64,
    pub loop_max_ms: f64,
    pub servo_packets: u64,
    pub msp_errors: u64,
    pub sensors: bool,
    pub wind: bool,
    pub sitl: SitlFacts,
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

/// The closed-flight runner. Built by `start` (spawn + config + readback
/// gate), advanced by `pump` (wall-paced, the caller may chunk), measured by
/// `finish`. Dropping it kills the SITL child and stops the telemetry thread
/// even on an un-finished, errored flight.
pub struct Flyer {
    cfg_facts: SitlFacts,
    #[allow(dead_code)] // held for its Drop: kills the SITL child
    proc_guard: SitlProc,
    sim_link: SimLink,
    telem: Arc<Mutex<TelemState>>,
    telem_handle: Option<std::thread::JoinHandle<()>>,
    stop: Arc<AtomicBool>,
    started: Instant,
    record: Option<RecordWriter>,
    quad: Quad,
    sensor: Option<SensorModel>,
    wind: Option<WindModel>,
    wind_on: bool,
    origin_lat: f64,
    origin_lon: f64,
    ticks_total: usize,
    tick_next: usize,
    // Fly-phase stick script, lifted out of the CLI Args.
    cfg_throttle: f64,
    cfg_yaw: f64,
    cfg_yaw_until: f64,
    cfg_pitch: f64,
    cfg_pitch_until: f64,
    cfg_roll: f64,
    cfg_roll_until: f64,
    loop_ms: Vec<f64>,
    max_alt: f64,
    att_samples: usize,
    last_seq: u64,
    servo_packets: u64,
    phase: ArmPhase,
    phase_entered: f64,
    armed_at: Option<f64>,
}

impl Flyer {
    /// Spawn the SITL, apply + verify the profile, open the record and the
    /// links. The quad spawns half a metre above the DEM height at the origin
    /// (flat ground without a grid); ground contact follows the grid from the
    /// first substep when a grid is set.
    pub fn start(cfg: &FlyerConfig, out_dir: &Path, record_name: &str) -> Result<Flyer, String> {
        std::fs::create_dir_all(out_dir).map_err(|e| format!("out dir: {e}"))?;

        // Provenance: hash of the binary that will run.
        let bin_bytes =
            std::fs::read(&cfg.sitl_bin).map_err(|e| format!("SITL binary {}: {e}", cfg.sitl_bin.display()))?;
        let bin_sha = sha256_hex(&bin_bytes);
        println!("[flyer] SITL binary {} sha256 {bin_sha}", cfg.sitl_bin.display());

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
                Command::new(&cfg.sitl_bin)
                    .current_dir(&sitl_cwd)
                    .stdout(Stdio::from(log_out))
                    .stderr(Stdio::from(log_err))
                    .spawn()
                    .map_err(|e| format!("spawn SITL: {e}"))?,
            ),
        };
        let pid = proc_guard.child.as_ref().unwrap().id();
        println!("[flyer] SITL pid {pid}, cwd {}", sitl_cwd.display());

        let mut link = wait_msp_ready(&mut proc_guard)?;

        // Profile: RAM only (no save; the fresh cwd has defaults on disk anyway).
        let profile: Vec<String> = cfg
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
                eprintln!("[flyer] profile line missing from diff readback: {line}");
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
        println!("[flyer] readback: {version_line}");
        if !profile_readback_ok {
            return Err("profile readback failed; refusing to fly with unverified config".into());
        }
        let cfg_facts = SitlFacts {
            bin: cfg.sitl_bin.display().to_string(),
            sha256: bin_sha,
            version: version_line,
            profile,
            profile_readback_ok,
        };

        let sim_link = SimLink::new().map_err(|e| format!("udp bind: {e}"))?;
        let mut record =
            RecordWriter::create(&out_dir.join(record_name)).map_err(|e| format!("record: {e}"))?;
        record
            .write_header(&RecordHeader {
                mode: "closed",
                seed: cfg.seed,
                duration_s: cfg.duration_s,
                preset: Preset::FREESTYLE_5IN.name,
                profile: cfg_facts.profile.clone(),
                sitl: Some(SitlProvenance {
                    path: cfg_facts.bin.clone(),
                    sha256: cfg_facts.sha256.clone(),
                    version: cfg_facts.version.clone(),
                }),
                sensors: cfg.sensor_cfg.clone(),
                wind: cfg.wind_cfg,
            })
            .map_err(|e| format!("record header: {e}"))?;

        let started = Instant::now();
        let stop = Arc::new(AtomicBool::new(false));
        let (telem, telem_handle) = spawn_telemetry(link, started, Arc::clone(&stop));

        // The flight loop's own state, hoisted from the tick loop's locals.
        let (origin_lat, origin_lon) =
            if cfg.gps_stale { (999.0, 999.0) } else { (47.6, -122.3) };
        let ticks_total = (cfg.duration_s / TICK_DT).floor() as usize;
        let spawn_z = 0.5 + cfg.terrain.as_ref().map_or(0.0, |g| g.h_at(0.0, 0.0));
        let mut quad = Quad::new(Preset::FREESTYLE_5IN, DVec3::new(0.0, 0.0, spawn_z));
        if let Some(grid) = &cfg.terrain {
            quad.ground = Ground::Grid(grid.clone());
        }
        // Sensor model seeded from the run seed; None keeps the fdm path
        // bit-identical to the pre-sensor harness.
        let sensor = cfg.sensor_cfg.as_ref().map(|c| SensorModel::new(*c));
        // Wind advances once per tick; None keeps the physics path
        // bit-identical to the pre-wind harness (Quad::wind stays zero).
        let wind = cfg.wind_cfg.map(WindModel::new);
        if let Some(w) = &wind {
            quad.wind = w.config().mean;
        }
        Ok(Flyer {
            cfg_facts,
            proc_guard,
            sim_link,
            telem,
            telem_handle: Some(telem_handle),
            stop,
            started,
            record: Some(record),
            quad,
            sensor,
            wind,
            wind_on: cfg.wind_cfg.is_some(),
            origin_lat,
            origin_lon,
            ticks_total,
            tick_next: 0,
            cfg_throttle: cfg.throttle,
            cfg_yaw: cfg.yaw,
            cfg_yaw_until: cfg.yaw_until,
            cfg_pitch: cfg.pitch,
            cfg_pitch_until: cfg.pitch_until,
            cfg_roll: cfg.roll,
            cfg_roll_until: cfg.roll_until,
            loop_ms: Vec::with_capacity(ticks_total),
            max_alt: 0.0,
            att_samples: 0,
            last_seq: 0,
            servo_packets: 0,
            phase: ArmPhase::WaitGrace,
            phase_entered: 0.0,
            armed_at: None,
        })
    }

    /// Advance at most `max_ticks` ticks, each wall-paced inside (the caller
    /// may therefore pump in chunks without changing per-tick pacing; gaps
    /// between chunks simply run the sim slower than real time, and the
    /// record stays sim-time indexed). A no-op once the plan is flown.
    pub fn pump(&mut self, max_ticks: usize) -> Result<(), String> {
        // Post-finish pumping took the record; refuse instead of unwrapping.
        if self.record.is_none() {
            return Err("flyer already finished".into());
        }
        let mut done = 0usize;
        while self.tick_next < self.ticks_total && done < max_ticks {
            let tick = self.tick_next;
            let tick_start = Instant::now();
            let t = (tick + 1) as f64 * TICK_DT;
            if let Some(w) = self.wind.as_mut() {
                self.quad.wind = w.step(TICK_DT, self.quad.state.pos.z);
            }

            // Latest FC status drives the arming state machine.
            let (arm, flags) = {
                let st = self.telem.lock().unwrap();
                st.status.last().map(|s| (s.1, s.2)).unwrap_or((u32::MAX, 0))
            };
            let (thr, aux3, yaw, pitch, roll, next_phase) = match self.phase {
                ArmPhase::WaitGrace => {
                    if t - self.phase_entered > GRACE_TIMEOUT_S {
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
                    if t - self.phase_entered > ARM_TIMEOUT_S {
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
                    let dt = t - self.armed_at.unwrap_or(0.0);
                    let thr = if dt < SETTLE_S {
                        0.0
                    } else if dt < SETTLE_S + RAMP_S {
                        self.cfg_throttle * (dt - SETTLE_S) / RAMP_S
                    } else {
                        self.cfg_throttle
                    };
                    let yaw = if t < self.cfg_yaw_until { self.cfg_yaw } else { 0.0 };
                    let pitch = if t < self.cfg_pitch_until { self.cfg_pitch } else { 0.0 };
                    let roll = if t < self.cfg_roll_until { self.cfg_roll } else { 0.0 };
                    (thr, ARMED_US, yaw, pitch, roll, None)
                }
            };
            if let Some(next) = next_phase {
                if next == ArmPhase::Fly {
                    self.armed_at = Some(t);
                    println!("[flyer] armed at t={t:.2} s");
                }
                self.phase = next;
                self.phase_entered = t;
            }
            self.sim_link
                .send_rc(&rc_packet(roll, pitch, yaw, thr, aux3))
                .map_err(|e| format!("send rc: {e}"))?;

            for _ in 0..SUBSTEPS_PER_TICK {
                self.quad.step(SUBSTEP_DT);
                let pkt = match self.sensor.as_mut() {
                    Some(s) => {
                        let (g, a) = self.quad.imu();
                        let rpm_mean = self.quad.rpm.iter().sum::<f64>() / 4.0;
                        let thr_mean = self.quad.throttle.iter().sum::<f64>() / 4.0;
                        let (gn, an) = s.step(SUBSTEP_DT, g, a, rpm_mean, thr_mean);
                        fdm_from_state_imu(
                            &self.quad,
                            gn,
                            an,
                            self.origin_lat,
                            self.origin_lon,
                            t,
                        )
                    }
                    None => fdm_from_state(&self.quad, self.origin_lat, self.origin_lon, t),
                };
                self.sim_link.send_fdm(&pkt).map_err(|e| format!("send fdm: {e}"))?;
            }
            let (motors, n) = self.sim_link.try_recv_motors();
            self.servo_packets += n;
            if let Some(m) = motors {
                for i in 0..4 {
                    self.quad.throttle[i] = (m.motor_speed[i] as f64).clamp(0.0, 1.0);
                }
            }

            // Record the newest telemetry snapshot once per tick (the thread
            // produces ~25 Hz; the record keeps every new snapshot).
            let fc = {
                let st = self.telem.lock().unwrap();
                if st.seq != self.last_seq {
                    self.last_seq = st.seq;
                    self.att_samples += 1;
                    st.latest.clone()
                } else {
                    None
                }
            };

            write_sample(
                self.record.as_mut().expect("record open across pump"),
                t,
                &self.quad,
                self.wind_on.then_some(self.quad.wind),
                fc,
            )
            .map_err(|e| format!("record: {e}"))?;
            self.max_alt = self.max_alt.max(self.quad.state.pos.z);

            // Wall-clock pacing: sleep off what is left of this 4 ms tick.
            let elapsed = tick_start.elapsed();
            let budget = Duration::from_secs_f64(TICK_DT);
            if elapsed < budget {
                std::thread::sleep(budget - elapsed);
            }
            self.loop_ms.push(elapsed.as_secs_f64() * 1e3);

            self.tick_next = tick + 1;
            done += 1;
        }
        Ok(())
    }

    /// Caller-readable instant of the flight.
    pub fn snapshot(&self) -> FlyerSnapshot {
        FlyerSnapshot {
            pos: self.quad.state.pos,
            vel: self.quad.state.vel,
            quat: self.quad.state.quat,
            soc: self.quad.soc(),
            vbus: self.quad.bus_voltage(),
            ticks_done: self.tick_next,
            ticks_total: self.ticks_total,
            armed_at: self.armed_at,
            max_alt: self.max_alt,
        }
    }

    pub fn ticks_total(&self) -> usize {
        self.ticks_total
    }

    pub fn facts(&self) -> &SitlFacts {
        &self.cfg_facts
    }

    /// Stop the telemetry thread, close the record and collect the stats.
    /// Callable exactly once; an un-finished Flyer still tears itself down
    /// (telemetry stopped, SITL child killed) on drop.
    pub fn finish(&mut self) -> Result<FlyerStats, String> {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(h) = self.telem_handle.take() {
            if h.join().is_err() {
                eprintln!("[flyer] telemetry thread panicked");
            }
        }
        let record = self
            .record
            .take()
            .ok_or_else(|| "flyer already finished".to_string())?;
        let record_hash = record.finish().map_err(|e| format!("record finish: {e}"))?;
        let (status_samples, msp_errors) = {
            let st = self.telem.lock().unwrap();
            (st.status.clone(), st.errors)
        };
        let wall_s = self.started.elapsed().as_secs_f64();
        let mut loop_ms = std::mem::take(&mut self.loop_ms);
        loop_ms.sort_by(|a, b| a.partial_cmp(b).unwrap());
        Ok(FlyerStats {
            record_hash,
            ticks: self.ticks_total,
            wall_s,
            final_alt: self.quad.state.pos.z,
            max_alt: self.max_alt,
            final_soc: self.quad.soc(),
            final_vbus: self.quad.bus_voltage(),
            final_i_bus: self.quad.bus_current(),
            rpm_end: self.quad.rpm,
            att_samples: self.att_samples,
            armed_at_s: self.armed_at,
            status_samples,
            loop_p50_ms: pct(&loop_ms, 0.50),
            loop_p99_ms: pct(&loop_ms, 0.99),
            loop_max_ms: *loop_ms.last().unwrap_or(&0.0),
            servo_packets: self.servo_packets,
            msp_errors,
            sensors: self.sensor.is_some(),
            wind: self.wind_on,
            sitl: self.cfg_facts.clone(),
        })
    }
}

impl Drop for Flyer {
    fn drop(&mut self) {
        if self.telem_handle.is_some() {
            self.stop.store(true, Ordering::Relaxed);
            if let Some(h) = self.telem_handle.take() {
                if h.join().is_err() {
                    eprintln!("[flyer] telemetry thread panicked");
                }
            }
        }
        // The SITL child dies with proc_guard's own drop, right after the
        // telemetry thread has been told to stop.
    }
}