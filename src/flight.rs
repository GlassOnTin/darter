//! Core-mode flight loop, shared by the sim_run CLI and the live plant.
//!
//! The loop shape — 250 Hz ticks of 32 x 125 us core substeps, one
//! darter_record sample per tick, the header written before the first
//! substep — is the repo's determinism contract: identical inputs produce
//! byte-identical record files. This module is the loop's single home:
//! `sim_run --mode core` runs all its ticks in one `advance_ticks` call,
//! and the Godot extension's DarterQuad (tools/gdext) steps the identical
//! flight through per-call chunks, so the loop cannot drift between them.
//! The closed-mode rung (M2b) reuses the sample/step constants here too.

use std::io;
use std::path::Path;
use std::time::Instant;

use crate::air::RHO_0;
use crate::preset::Preset;
use crate::quad::{hover_throttle, Quad};
use crate::record::{FcSample, RecordHeader, RecordWriter, Sample};
use crate::terrain::{Ground, TerrainGrid};
use crate::wind::{WindConfig, WindModel};
use crate::DVec3;

/// Core substep: 125 us, the 8 kHz rate a real FC's PID loop runs at.
pub const SUBSTEP_DT: f64 = 125e-6;
/// Substeps per 250 Hz tick.
pub const SUBSTEPS_PER_TICK: usize = 32;
/// The tick period record samples land on (the record's time axis).
pub const TICK_DT: f64 = SUBSTEP_DT * SUBSTEPS_PER_TICK as f64;

/// One core-mode flight's fixed inputs. `throttle = None` solves the hover
/// value at full charge (the sim_run default); `vel` is the scripted initial
/// velocity (zero = spawn at rest); `terrain` enables grid ground contact and
/// the DEM spawn height (--alt stays an above-ground offset).
pub struct CoreSetup {
    pub preset: Preset,
    pub seed: u64,
    pub duration_s: f64,
    pub throttle: Option<f64>,
    pub alt_agl: f64,
    pub x: f64,
    pub y: f64,
    pub vel: DVec3,
    pub wind_cfg: Option<WindConfig>,
    pub terrain: Option<TerrainGrid>,
}

/// Results reported at finish(). Wall and loop percentiles are
/// instrumentation only — they never affect record bytes.
pub struct CoreStats {
    pub record_hash: u64,
    pub ticks: usize,
    pub wall_s: f64,
    pub final_alt: f64,
    pub max_alt: f64,
    pub final_soc: f64,
    pub final_vbus: f64,
    pub final_i_bus: f64,
    pub rpm_end: [f64; 4],
    pub loop_p50_ms: f64,
    pub loop_p99_ms: f64,
    pub loop_max_ms: f64,
}

/// The driven core-mode flight: `new` (spawn + record header), then
/// `advance_ticks` chunks, then `finish` (commit + stats). The record is
/// committed only at `finish`; a flight abandoned before it writes nothing.
pub struct CoreFlight {
    quad: Quad,
    wind: Option<WindModel>,
    record: Option<RecordWriter>,
    tick: usize,
    total_ticks: usize,
    max_alt: f64,
    loop_ms: Vec<f64>,
    started: Instant,
}

impl CoreFlight {
    /// Build the quad per the core-mode spawn convention (AGL offset over
    /// the DEM height under (x, y), ground contact from the grid), fix the
    /// throttle, and write the record header from the settled inputs.
    pub fn new(setup: CoreSetup, record_path: &Path) -> Result<Self, String> {
        let thr = match setup.throttle {
            Some(t) => t,
            None => hover_throttle(&setup.preset, setup.preset.battery, 1.0, RHO_0),
        };
        let h = setup.terrain.as_ref().map_or(0.0, |g| g.h_at(setup.x, setup.y));
        let mut quad = Quad::new(setup.preset, DVec3::new(setup.x, setup.y, setup.alt_agl + h));
        if let Some(grid) = setup.terrain {
            quad.ground = Ground::Grid(grid);
        }
        quad.throttle = [thr; 4];
        if setup.vel != DVec3::ZERO {
            quad.state.vel = setup.vel;
        }
        let wind = setup.wind_cfg.map(WindModel::new);
        if let Some(w) = &wind {
            quad.wind = w.config().mean;
        }

        let mut record =
            RecordWriter::create(record_path).map_err(|e| format!("record: {e}"))?;
        record
            .write_header(&RecordHeader {
                mode: "core",
                seed: setup.seed,
                duration_s: setup.duration_s,
                preset: setup.preset.name,
                profile: {
                    let mut p = vec![
                        format!("throttle={thr:.6}"),
                        format!("alt={:.3}", setup.alt_agl),
                        format!("spawn=({:.3},{:.3})", setup.x, setup.y),
                    ];
                    if setup.vel != DVec3::ZERO {
                        p.push(format!(
                            "vel=({:.3},{:.3},{:.3})",
                            setup.vel.x, setup.vel.y, setup.vel.z
                        ));
                    }
                    p
                },
                sitl: None,
                sensors: None,
                wind: setup.wind_cfg,
            })
            .map_err(|e| format!("record header: {e}"))?;

        let total_ticks = (setup.duration_s / TICK_DT).floor() as usize;
        Ok(Self {
            quad,
            wind,
            record: Some(record),
            tick: 0,
            total_ticks,
            max_alt: 0.0,
            loop_ms: Vec::with_capacity(total_ticks),
            started: Instant::now(),
        })
    }

    /// Run `n` more ticks (clamped at `total_ticks`): wind advances once per
    /// tick at the current altitude and holds across the 32 substeps, then
    /// one sample is written per tick. Errors only on record I/O.
    pub fn advance_ticks(&mut self, n: usize) -> Result<(), String> {
        for _ in 0..n {
            if self.tick >= self.total_ticks {
                return Ok(());
            }
            let tick_start = Instant::now();
            if let Some(w) = self.wind.as_mut() {
                self.quad.wind = w.step(TICK_DT, self.quad.state.pos.z);
            }
            for _ in 0..SUBSTEPS_PER_TICK {
                self.quad.step(SUBSTEP_DT);
            }
            let t = (self.tick + 1) as f64 * TICK_DT;
            // No sensor model yet (T3): the quad state is the telemetry, no FC.
            let w = self.wind.is_some().then_some(self.quad.wind);
            write_sample(
                self.record.as_mut().expect("writer held until finish"),
                t,
                &self.quad,
                w,
                None,
            )
            .map_err(|e| format!("record: {e}"))?;
            self.max_alt = self.max_alt.max(self.quad.state.pos.z);
            self.tick += 1;
            self.loop_ms.push(tick_start.elapsed().as_secs_f64() * 1e3);
        }
        Ok(())
    }

    pub fn ticks_done(&self) -> usize {
        self.tick
    }

    pub fn ticks_total(&self) -> usize {
        self.total_ticks
    }

    /// Live read-back of the flight's current state (the render loop's view;
    /// f64 end-of-last-tick values).
    pub fn state(&self) -> (DVec3, DVec3, crate::DQuat, usize) {
        (
            self.quad.state.pos,
            self.quad.state.vel,
            self.quad.state.quat,
            self.tick,
        )
    }

    pub fn rpm_view(&self) -> [f64; 4] {
        self.quad.rpm
    }

    pub fn soc_view(&self) -> f64 {
        self.quad.soc()
    }

    pub fn vbus_view(&self) -> f64 {
        self.quad.bus_voltage()
    }

    /// Commit the record and report the stats. Final-state numbers are read
    /// before the writer is finished; the wall clock excludes the sort and
    /// the hash (same as sim_run measured it).
    pub fn finish(&mut self) -> Result<CoreStats, String> {
        let final_alt = self.quad.state.pos.z;
        let wall_s = self.started.elapsed().as_secs_f64();
        let record_hash = self
            .record
            .take()
            .expect("finish called once")
            .finish()
            .map_err(|e| format!("record finish: {e}"))?;
        let mut loop_ms = std::mem::take(&mut self.loop_ms);
        loop_ms.sort_by(|a, b| a.partial_cmp(b).unwrap());
        Ok(CoreStats {
            record_hash,
            ticks: self.tick,
            wall_s,
            final_alt,
            max_alt: self.max_alt,
            final_soc: self.quad.soc(),
            final_vbus: self.quad.bus_voltage(),
            final_i_bus: self.quad.bus_current(),
            rpm_end: self.quad.rpm,
            loop_p50_ms: pct(&loop_ms, 0.50),
            loop_p99_ms: pct(&loop_ms, 0.99),
            loop_max_ms: *loop_ms.last().unwrap_or(&0.0),
        })
    }
}

/// One record sample from the quad's end-of-tick state (the wire shape both
/// core and closed modes write; closed mode passes the latest FC telemetry).
pub fn write_sample(
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

/// Sorted-array percentile at quantile `q` (summary instrumentation;
/// truncating index — the exact convention the sim_run summary has always
/// reported).
pub fn pct(sorted: &[f64], q: f64) -> f64 {
    if sorted.is_empty() {
        return 0.0;
    }
    let i = ((q * (sorted.len() - 1) as f64) as usize).min(sorted.len() - 1);
    sorted[i]
}