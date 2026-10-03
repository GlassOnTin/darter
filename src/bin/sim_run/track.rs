//! darter_track: checkpoint/race track files, parsed and evaluated by sim_run.
//!
//! Schema v1 (JSON, "darter_track", version 1). Coordinates are world metres
//! ENU absolute — x east, y north, z up, origin at source.origin_lat/lon, the
//! same frame literal as pack.json v3. A track file is independent of any pack.
//!
//! ```json
//! {
//!   "schema": "darter_track",
//!   "version": 1,
//!   "name": "corridor_slalom",
//!   "spawn": {"x": 0.0, "y": 0.0, "z": 35.0},
//!   "checkpoints": [
//!     {"kind": "gate", "x": -15.0, "y": 0.0, "z": 34.41, "radius_m": 4.0},
//!     {"kind": "start", "x": 0.0, "y": 0.0, "z": 35.0, "radius_m": 4.0}
//!   ]
//! }
//! ```
//!
//! - Top-level keys, exactly: `schema`, `version`, `name`, `spawn` (optional),
//!   `checkpoints`. Checkpoint keys, exactly: `kind`, `x`, `y`, `z`,
//!   `radius_m`. Unknown keys are errors (typo protection beats forward
//!   compat; a future editor bumps `version`).
//! - `spawn` is advisory metadata only: it documents where a pilot is expected
//!   to start, is validated as three finite numbers, and is never read for
//!   physics, orientation, or flying. There is no coupling with the
//!   --x/--y/--alt flight-spawn flags.
//! - `kind`: "gate" or "start". A start checkpoint makes the track a loop and
//!   must be the first listed entry; at most one start; a loop needs at least
//!   3 checkpoints. An open track (no start) has at least 2.
//! - v1 lap semantics are a PASSIVE event log. `compute_events` reports every
//!   valid crossing: a record segment that crosses the checkpoint's plane from
//!   the behind side, in the forward direction, with the crossing point inside
//!   the radius. Wrong order, missed gates, skips, and repeated cross-side
//!   drift produce nothing — no chain enforcement, no failures. Lap splits for
//!   loops are the differences between consecutive start-gate crossing times
//!   (the first start crossing opens lap 1 and carries no split). Strict
//!   progress enforcement is later work with different schema needs.
//!
//! Checkpoint plane orientation (both sim_run and the GDScript mirror must
//! implement identically):
//!
//! ```text
//! loop  = (a "start" checkpoint exists)            # enforced: index 0
//! prev_i = pos[(i+n-1) % n] if loop else pos[max(i-1, 0)]
//! next_i = pos[(i+1)   % n] if loop else pos[min(i+1, n-1)]
//! n_raw  = next - prev;  n = n_raw / |n_raw|       # |n_raw| >= 1e-6 else error
//! ```
//!
//! Open tracks clamp the neighbour index, so the first gate's plane normal is
//! P1-P0 (its own travel direction) and the last gate's is Pn-1-Pn-2. Loops
//! wrap, which encodes the direction of travel through the start gate.
//!
//! Crossing test per directed record segment A(row k) -> B(row k+1), times
//! strictly increasing:
//!
//! ```text
//! dA = (A-C)·n;  dB = (B-C)·n          # C = checkpoint centre
//! crossed iff dB > 0 and dA <= 0        # from behind side, forward only
//! s = dA / (dA - dB)                    # in [0,1); hit = A + (B-A)*s
//! hit   iff |hit-C| <= radius
//! t_cross = tA + (tB - tA) * s
//! ```
//!
//! Events are sorted by (t, index). All maths f64 on the Rust side; the
//! GDScript mirror (tools/godot_smoke/pack_replay.gd) repeats this with plain
//! scalar floats only — its Vector3 is f32.
//!
//! v1 semantics note: a vehicle sitting exactly ON the plane (dA == 0) that
//! then moves to the forward side counts as a crossing at its row time; drift
//! back and forth re-fires on each behind->forward transition. Deliberate for
//! a passive event log.

use std::fs;
use std::path::Path;

use serde_json::Value;

#[derive(Debug, Clone)]
pub(crate) struct TrackCp {
    pub kind: String, // "gate" | "start"
    pub pos: [f64; 3],
    pub radius_m: f64,
}

#[derive(Debug, Clone)]
pub(crate) struct Track {
    pub name: String,
    pub spawn: Option<[f64; 3]>,
    pub cps: Vec<TrackCp>,
    pub is_loop: bool,
}

/// Parse a track file. Pure function of the bytes; the call site prefixes
/// errors with `track {path}: `. Loud rejections on every shape violation —
/// user-authored JSON must never misparse silently.
pub(crate) fn parse(bytes: &[u8]) -> Result<Track, String> {
    let root: Value = serde_json::from_slice(bytes).map_err(|e| format!("parse: {e}"))?;
    let obj = root.as_object().ok_or("root must be a JSON object")?;
    for key in obj.keys() {
        if !matches!(key.as_str(), "schema" | "version" | "name" | "spawn" | "checkpoints") {
            return Err(format!("unknown key \"{key}\""));
        }
    }
    let schema = obj
        .get("schema")
        .and_then(|v| v.as_str())
        .ok_or("schema must be the string \"darter_track\"")?;
    if schema != "darter_track" {
        return Err(format!("schema must be \"darter_track\", got \"{schema}\""));
    }
    let version = obj
        .get("version")
        .and_then(|v| v.as_f64())
        .ok_or("version must be 1")?;
    if version != 1.0 {
        return Err("version must be 1".to_string());
    }
    let name = obj
        .get("name")
        .and_then(|v| v.as_str())
        .filter(|s| !s.is_empty())
        .ok_or("name must be a non-empty string")?
        .to_string();
    let spawn = match obj.get("spawn") {
        None | Some(Value::Null) => None,
        Some(v) => {
            let o = v
                .as_object()
                .ok_or("spawn must be an object with numeric x/y/z")?;
            let mut p = [0.0f64; 3];
            for (i, k) in ["x", "y", "z"].iter().enumerate() {
                let n = o
                    .get(*k)
                    .and_then(|v| v.as_f64())
                    .ok_or("spawn must be an object with numeric x/y/z")?;
                if !n.is_finite() {
                    return Err("spawn must be an object with numeric x/y/z".to_string());
                }
                p[i] = n;
            }
            Some(p)
        }
    };
    let cps_v = obj
        .get("checkpoints")
        .and_then(|v| v.as_array())
        .ok_or("checkpoints must be an array with at least 2 entries")?;
    if cps_v.len() < 2 {
        return Err("checkpoints must be an array with at least 2 entries".to_string());
    }
    let mut cps = Vec::with_capacity(cps_v.len());
    for (i, v) in cps_v.iter().enumerate() {
        let o = v
            .as_object()
            .ok_or(format!("checkpoint {i} must be an object"))?;
        for key in o.keys() {
            if !matches!(key.as_str(), "kind" | "x" | "y" | "z" | "radius_m") {
                return Err(format!("checkpoint {i}: unknown key \"{key}\""));
            }
        }
        let kind = o
            .get("kind")
            .and_then(|v| v.as_str())
            .filter(|k| *k == "gate" || *k == "start")
            .ok_or(format!("checkpoint {i}: kind must be \"gate\" or \"start\""))?
            .to_string();
        let mut pos = [0.0f64; 3];
        for (j, k) in ["x", "y", "z"].iter().enumerate() {
            let n = o
                .get(*k)
                .and_then(|v| v.as_f64())
                .ok_or(format!("checkpoint {i}: {k} must be a number"))?;
            if !n.is_finite() {
                return Err(format!("checkpoint {i}: {k} must be a finite number"));
            }
            pos[j] = n;
        }
        let radius = o
            .get("radius_m")
            .and_then(|v| v.as_f64())
            .ok_or(format!(
                "checkpoint {i}: radius_m must be a number > 0 and <= 1000"
            ))?;
        if !radius.is_finite() || radius <= 0.0 || radius > 1000.0 {
            return Err(format!(
                "checkpoint {i}: radius_m must be a number > 0 and <= 1000"
            ));
        }
        cps.push(TrackCp { kind, pos, radius_m: radius });
    }
    let starts = cps.iter().filter(|c| c.kind == "start").count();
    if starts > 1 {
        return Err("more than one start checkpoint".to_string());
    }
    if starts == 1 && cps[0].kind != "start" {
        return Err("a start checkpoint must be the first entry".to_string());
    }
    let is_loop = starts == 1;
    if is_loop && cps.len() < 3 {
        return Err(format!(
            "a loop track needs at least 3 checkpoints, got {}",
            cps.len()
        ));
    }
    distinctness(&cps, is_loop)?;
    Ok(Track { name, spawn, cps, is_loop })
}

/// Adjacent checkpoints must differ by >= 1e-3 m; loops also last-vs-first.
/// Guard against zero-length normals at the neighbourhood level, e.g. a loop
/// like A,B,A,B whose neighbours of A coincide even though adjacent entries
/// differ.
fn distinctness(cps: &[TrackCp], is_loop: bool) -> Result<(), String> {
    let n = cps.len();
    let pairs: Vec<(usize, usize)> = if is_loop {
        (0..n).map(|i| (i, (i + 1) % n)).collect()
    } else {
        (0..n - 1).map(|i| (i, i + 1)).collect()
    };
    for (a, b) in &pairs {
        if dist(cps[*a].pos, cps[*b].pos) < 1e-3 {
            return Err(format!("checkpoints {a} and {b} coincide"));
        }
    }
    for i in 0..n {
        let prev = if is_loop {
            cps[(i + n - 1) % n].pos
        } else {
            cps[i.saturating_sub(1)].pos
        };
        let next = if is_loop {
            cps[(i + 1) % n].pos
        } else {
            cps[(i + 1).min(n - 1)].pos
        };
        if norm3(sub3(next, prev)) < 1e-6 {
            return Err(format!(
                "checkpoint {i} has a degenerate normal (neighbours coincide)"
            ));
        }
    }
    Ok(())
}

fn sub3(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn norm3(a: [f64; 3]) -> f64 {
    (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt()
}

fn dist(a: [f64; 3], b: [f64; 3]) -> f64 {
    norm3(sub3(a, b))
}

/// Plane normal (unit) per checkpoint: neighbour midpoint rule, loop wrap,
/// open ends clamped. Same formulas as the header comment. Call only on
/// tracks that passed parse() — this divides by the neighbour span.
pub(crate) fn normals(track: &Track) -> Vec<[f64; 3]> {
    let n = track.cps.len();
    let mut out = Vec::with_capacity(n);
    for i in 0..n {
        let prev = if track.is_loop {
            track.cps[(i + n - 1) % n].pos
        } else {
            track.cps[i.saturating_sub(1)].pos
        };
        let next = if track.is_loop {
            track.cps[(i + 1) % n].pos
        } else {
            track.cps[(i + 1).min(n - 1)].pos
        };
        let raw = sub3(next, prev);
        let len = norm3(raw);
        out.push([raw[0] / len, raw[1] / len, raw[2] / len]);
    }
    out
}

#[derive(Debug, Clone, Copy)]
pub(crate) struct RecPoint {
    pub t: f64,
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

/// Read the t/px/py/pz columns of a darter_record JSONL file. Header lines
/// (starting {"schema) are skipped; times must be strictly increasing.
pub(crate) fn read_record_positions(path: &Path) -> Result<Vec<RecPoint>, String> {
    let text = fs::read_to_string(path).map_err(|e| format!("read: {e}"))?;
    let mut rows = Vec::new();
    let mut prev_t = f64::NEG_INFINITY;
    for (ln, line) in text.lines().enumerate() {
        if line.starts_with("{\"schema") {
            continue;
        }
        let v: Value = serde_json::from_str(line)
            .map_err(|e| format!("record line {}: parse: {e}", ln + 1))?;
        let f = |k: &str| -> Result<f64, String> {
            v.get(k)
                .and_then(|x| x.as_f64())
                .ok_or(format!("record line {}: missing {k}", ln + 1))
        };
        let t = f("t")?;
        if t <= prev_t {
            return Err(format!("record line {}: t not increasing", ln + 1));
        }
        rows.push(RecPoint { t, x: f("px")?, y: f("py")?, z: f("pz")? });
        prev_t = t;
    }
    Ok(rows)
}

/// Every valid crossing of every checkpoint, chronological. Returns the
/// events sorted by (t, checkpoint index) and the lap splits (empty for open
/// tracks; consecutive start-gate crossing time differences for loops).
pub(crate) fn compute_events(
    track: &Track,
    rows: &[RecPoint],
) -> (Vec<(usize, f64)>, Vec<f64>) {
    let norms = normals(track);
    let mut events: Vec<(usize, f64)> = Vec::new();
    for w in 0..rows.len().saturating_sub(1) {
        let a = &rows[w];
        let b = &rows[w + 1];
        for (i, cp) in track.cps.iter().enumerate() {
            let n = norms[i];
            let d_a = (a.x - cp.pos[0]) * n[0]
                + (a.y - cp.pos[1]) * n[1]
                + (a.z - cp.pos[2]) * n[2];
            let d_b = (b.x - cp.pos[0]) * n[0]
                + (b.y - cp.pos[1]) * n[1]
                + (b.z - cp.pos[2]) * n[2];
            if !(d_b > 0.0 && d_a <= 0.0) {
                continue;
            }
            let s = d_a / (d_a - d_b);
            let hx = a.x + (b.x - a.x) * s - cp.pos[0];
            let hy = a.y + (b.y - a.y) * s - cp.pos[1];
            let hz = a.z + (b.z - a.z) * s - cp.pos[2];
            if hx * hx + hy * hy + hz * hz <= cp.radius_m * cp.radius_m {
                events.push((i, a.t + (b.t - a.t) * s));
            }
        }
    }
    events.sort_by(|a, b| {
        a.1.partial_cmp(&b.1)
            .unwrap_or(std::cmp::Ordering::Equal)
            .then(a.0.cmp(&b.0))
    });
    let mut splits = Vec::new();
    if track.is_loop {
        let start_ts: Vec<f64> = events
            .iter()
            .filter(|(i, _)| track.cps[*i].kind == "start")
            .map(|(_, t)| *t)
            .collect();
        for w in 1..start_ts.len() {
            splits.push(start_ts[w] - start_ts[w - 1]);
        }
    }
    (events, splits)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn parse_str(s: &str) -> Result<Track, String> {
        parse(s.as_bytes())
    }

    const BASE: &str = r#"{
        "schema": "darter_track",
        "version": 1,
        "name": "t",
        "spawn": {"x": 0.0, "y": 0.0, "z": 35.0},
        "checkpoints": [
            {"kind": "gate", "x": -15.0, "y": 0.0, "z": 34.41, "radius_m": 4.0},
            {"kind": "gate", "x": -40.0, "y": 0.0, "z": 33.5, "radius_m": 4.0}
        ]
    }"#;

    /// The BASE fixture is fixed text, so every replacement must hit.
    fn replace(json: &str, from: &str, to: &str) -> String {
        assert!(json.contains(from), "fixture text not found: {from}");
        json.replace(from, to)
    }

    fn err_of(s: &str) -> String {
        parse_str(s).expect_err("expected rejection")
    }

    #[test]
    fn track_parses_v1_fields() {
        let t = parse_str(BASE).expect("parses");
        assert_eq!(t.name, "t");
        assert!(!t.is_loop);
        assert_eq!(t.spawn, Some([0.0, 0.0, 35.0]));
        assert_eq!(t.cps.len(), 2);
        assert_eq!(t.cps[0].pos, [-15.0, 0.0, 34.41]);
        assert_eq!(t.cps[1].radius_m, 4.0);
        // version 1.0 is the same number 1
        let v10 = replace(BASE, "\"version\": 1,", "\"version\": 1.0,");
        parse_str(&v10).expect("version 1.0 accepted");
        // spawn may be absent
        let no_spawn = replace(BASE, "\"spawn\": {\"x\": 0.0, \"y\": 0.0, \"z\": 35.0},", "");
        let t2 = parse_str(&no_spawn).expect("parses without spawn");
        assert_eq!(t2.spawn, None);
    }

    #[test]
    fn track_rejects_wrong_schema() {
        let s = replace(BASE, "\"darter_track\",", "\"nope\",");
        assert!(err_of(&s).contains("schema must be \"darter_track\""));
        let nonstr = replace(BASE, "\"schema\": \"darter_track\"", "\"schema\": 7");
        assert!(err_of(&nonstr).contains("schema must be the string"));
    }

    #[test]
    fn track_rejects_bad_version() {
        let s = replace(BASE, "\"version\": 1,", "\"version\": 2,");
        assert!(err_of(&s).contains("version must be 1"));
    }

    #[test]
    fn track_rejects_unknown_keys() {
        let top = replace(BASE, "\"name\": \"t\",", "\"name\": \"t\", \"oops\": 1,");
        assert!(err_of(&top).contains("unknown key \"oops\""));
        let cp = replace(BASE, "\"z\": 34.41,", "\"z\": 34.41, \"oops\": 1,");
        assert!(err_of(&cp).contains("checkpoint 0: unknown key \"oops\""));
    }

    #[test]
    fn track_rejects_bad_name() {
        let empty = replace(BASE, "\"name\": \"t\",", "\"name\": \"\",");
        assert!(err_of(&empty).contains("name must be a non-empty string"));
        let missing = replace(BASE, "\"name\": \"t\",", "");
        assert!(err_of(&missing).contains("name must be a non-empty string"));
    }

    #[test]
    fn track_rejects_bad_spawn() {
        let s = replace(
            BASE,
            "\"x\": 0.0, \"y\": 0.0, \"z\": 35.0",
            "\"x\": 0.0, \"y\": \"a\", \"z\": 35.0",
        );
        assert!(err_of(&s).contains("spawn must be an object with numeric x/y/z"));
    }

    #[test]
    fn track_rejects_bad_checkpoint() {
        let kind = replace(BASE, "\"kind\": \"gate\"", "\"kind\": \"foo\"");
        assert!(err_of(&kind).contains("checkpoint 0: kind must be \"gate\" or \"start\""));
        let rad0 = replace(BASE, "\"radius_m\": 4.0", "\"radius_m\": 0.0");
        assert!(err_of(&rad0).contains("radius_m must be a number > 0 and <= 1000"));
        let radbig = replace(BASE, "\"radius_m\": 4.0", "\"radius_m\": 1001.0");
        assert!(err_of(&radbig).contains("radius_m must be a number > 0 and <= 1000"));
        let noz = replace(BASE, "\"z\": 34.41,", "\"x\": -15.0,");
        assert!(err_of(&noz).contains("checkpoint 0: z must be a number"));
        let nonum = replace(BASE, "\"x\": -15.0,", "\"x\": \"a\",");
        assert!(err_of(&nonum).contains("checkpoint 0: x must be a number"));
        // A single checkpoint entry is rejected outright.
        let one = replace(
            BASE,
            "{\"kind\": \"gate\", \"x\": -15.0, \"y\": 0.0, \"z\": 34.41, \"radius_m\": 4.0},",
            "",
        );
        assert!(err_of(&one).contains("at least 2 entries"));
    }

    #[test]
    fn track_rejects_coincident_checkpoints() {
        // The whole checkpoint is replaced: coincident means the 3D distance
        // is under 1e-3 m, so changing x alone (with z still 0.91 m apart)
        // is legitimately accepted.
        let cp1 = "{\"kind\": \"gate\", \"x\": -40.0, \"y\": 0.0, \"z\": 33.5, \"radius_m\": 4.0}";
        let exact = replace(
            BASE,
            cp1,
            "{\"kind\": \"gate\", \"x\": -15.0, \"y\": 0.0, \"z\": 34.41, \"radius_m\": 4.0}",
        );
        assert!(err_of(&exact).contains("checkpoints 0 and 1 coincide"));
        // Below the 1e-3 m threshold counts as coincident too.
        let close = replace(
            BASE,
            cp1,
            "{\"kind\": \"gate\", \"x\": -15.0001, \"y\": 0.0, \"z\": 34.41, \"radius_m\": 4.0}",
        );
        assert!(err_of(&close).contains("checkpoints 0 and 1 coincide"));
    }

    #[test]
    fn track_rejects_start_not_first() {
        let s = replace(
            BASE,
            "\"kind\": \"gate\", \"x\": -40.0,",
            "\"kind\": \"start\", \"x\": -40.0,",
        );
        assert!(err_of(&s).contains("a start checkpoint must be the first entry"));
    }

    #[test]
    fn track_rejects_second_start() {
        let late = replace(
            BASE,
            "\"kind\": \"gate\", \"x\": -40.0,",
            "\"kind\": \"start\", \"x\": -40.0,",
        );
        let both = replace(
            &late,
            "\"kind\": \"gate\", \"x\": -15.0,",
            "\"kind\": \"start\", \"x\": -15.0,",
        );
        assert!(err_of(&both).contains("more than one start checkpoint"));
    }

    #[test]
    fn track_rejects_short_loop() {
        let s = replace(
            BASE,
            "\"kind\": \"gate\", \"x\": -15.0,",
            "\"kind\": \"start\", \"x\": -15.0,",
        );
        assert!(err_of(&s).contains("a loop track needs at least 3 checkpoints"));
    }

    #[test]
    fn track_rejects_degenerate_normal() {
        // Loop A,B,A,B: adjacent entries differ but checkpoint 0's neighbours
        // are both B.
        let s = r#"{"schema":"darter_track","version":1,"name":"t","checkpoints":[
            {"kind":"start","x":0.0,"y":0.0,"z":0.0,"radius_m":4.0},
            {"kind":"gate","x":10.0,"y":0.0,"z":0.0,"radius_m":4.0},
            {"kind":"gate","x":0.0,"y":0.0,"z":0.0,"radius_m":4.0},
            {"kind":"gate","x":10.0,"y":0.0,"z":0.0,"radius_m":4.0}
        ]}"#;
        assert!(err_of(s).contains("degenerate normal"));
    }

    #[test]
    fn gate_normals_open_ends() {
        // Three gates on the x axis flown eastward: every normal points +x,
        // including both ends (first = P1-P0, last = Pn-1-Pn-2).
        let mk = |x: f64| format!(r#"{{"kind":"gate","x":{x},"y":0.0,"z":0.0,"radius_m":2.0}}"#);
        let s = format!(
            r#"{{"schema":"darter_track","version":1,"name":"t","checkpoints":[{},{},{}]}}"#,
            mk(0.0),
            mk(10.0),
            mk(20.0)
        );
        let t = parse_str(&s).expect("parses");
        for n in normals(&t) {
            assert!((n[0] - 1.0).abs() < 1e-12, "normal {n:?}");
            assert!(n[1].abs() < 1e-12 && n[2].abs() < 1e-12);
        }
    }

    #[test]
    fn gate_normals_loop_wrap() {
        // Rectangle loop 0->1->2->3: checkpoint 1's normal is P2-P0 (the
        // mid-neighbour rule), checkpoint 0's is P1-P3 (the wrap).
        let s = r#"{"schema":"darter_track","version":1,"name":"t","checkpoints":[
            {"kind":"start","x":0.0,"y":0.0,"z":0.0,"radius_m":2.0},
            {"kind":"gate","x":10.0,"y":0.0,"z":0.0,"radius_m":2.0},
            {"kind":"gate","x":10.0,"y":10.0,"z":0.0,"radius_m":2.0},
            {"kind":"gate","x":0.0,"y":10.0,"z":0.0,"radius_m":2.0}
        ]}"#;
        let t = parse_str(s).expect("parses");
        assert!(t.is_loop);
        let ns = normals(&t);
        let inv = (10.0f64 * 10.0 + 10.0 * 10.0).sqrt();
        // cp1: next - prev = P2 - P0 = (10,10,0)
        assert!((ns[1][0] - 10.0 / inv).abs() < 1e-12);
        assert!((ns[1][1] - 10.0 / inv).abs() < 1e-12);
        // cp0: next - prev = P1 - P3 = (10,-10,0)
        assert!((ns[0][0] - 10.0 / inv).abs() < 1e-12);
        assert!((ns[0][1] + 10.0 / inv).abs() < 1e-12);
    }

    /// Build a Track struct directly (open, no spawn) for crossing tests.
    fn track_with_cps(cps: &str) -> Track {
        let v: Value = serde_json::from_str(cps).expect("cps json");
        let arr = v.as_array().expect("array");
        let mut out = Vec::new();
        for c in arr {
            out.push(TrackCp {
                kind: c["kind"].as_str().unwrap_or("gate").to_string(),
                pos: [
                    c["x"].as_f64().unwrap(),
                    c["y"].as_f64().unwrap(),
                    c["z"].as_f64().unwrap(),
                ],
                radius_m: c["radius_m"].as_f64().unwrap(),
            });
        }
        Track { name: "t".into(), spawn: None, cps: out, is_loop: false }
    }

    fn rows(pts: &[(f64, f64, f64, f64)]) -> Vec<RecPoint> {
        pts.iter()
            .map(|(t, x, y, z)| RecPoint { t: *t, x: *x, y: *y, z: *z })
            .collect()
    }

    #[test]
    fn crossing_detects_direction_and_sign_change() {
        let t = track_with_cps(
            r#"[
            {"kind":"gate","x":0.0,"y":0.0,"z":0.0,"radius_m":2.0},
            {"kind":"gate","x":10.0,"y":0.0,"z":0.0,"radius_m":2.0}
        ]"#,
        );
        // Eastward through gate 0 at the midpoint: event at t 0.5.
        let (ev, _) = compute_events(&t, &rows(&[(0.0, -1.0, 0.0, 0.0), (1.0, 1.0, 0.0, 0.0)]));
        assert_eq!(ev, vec![(0, 0.5)]);
        // Westward through the plane: no event (direction enforced).
        let (ev, _) = compute_events(&t, &rows(&[(0.0, 1.0, 0.0, 0.0), (1.0, -1.0, 0.0, 0.0)]));
        assert!(ev.is_empty());
        // Parallel drift along the plane: no event.
        let (ev, _) = compute_events(&t, &rows(&[(0.0, 5.0, 1.0, 0.0), (1.0, 5.0, 2.0, 0.0)]));
        assert!(ev.is_empty());
        // Start exactly on the plane, moving away from the forward side: no
        // event.
        let (ev, _) = compute_events(&t, &rows(&[(0.0, 0.0, 0.0, 0.0), (1.0, -6.0, 0.0, 0.0)]));
        assert!(ev.is_empty());
    }

    #[test]
    fn crossing_requires_radius_hit() {
        let t = track_with_cps(
            r#"[
            {"kind":"gate","x":0.0,"y":0.0,"z":0.0,"radius_m":2.0},
            {"kind":"gate","x":10.0,"y":0.0,"z":0.0,"radius_m":2.0}
        ]"#,
        );
        // Plane crossed 100 m off-centre: outside the aperture.
        let (ev, _) = compute_events(&t, &rows(&[(0.0, -1.0, 100.0, 0.0), (1.0, 1.0, 100.0, 0.0)]));
        assert!(ev.is_empty());
        // Exactly at the radius edge counts (<=). Exact arithmetic: the hit
        // point is (0, 2, 0) with no rounding.
        let (ev, _) = compute_events(&t, &rows(&[(0.0, -1.0, 2.0, 0.0), (1.0, 1.0, 2.0, 0.0)]));
        assert_eq!(ev, vec![(0, 0.5)]);
    }

    #[test]
    fn crossing_time_is_interpolated() {
        let t = track_with_cps(
            r#"[
            {"kind":"gate","x":0.0,"y":0.0,"z":0.0,"radius_m":2.0},
            {"kind":"gate","x":10.0,"y":0.0,"z":0.0,"radius_m":2.0}
        ]"#,
        );
        let (ev, _) = compute_events(&t, &rows(&[(10.0, -2.0, 0.0, 0.0), (11.0, 2.0, 0.0, 0.0)]));
        assert_eq!(ev, vec![(0, 10.5)]);
    }

    #[test]
    fn crossing_tiebreak_sorted_by_t_then_index() {
        // Equal crossing times sort by checkpoint index. Exact equality
        // needs exact arithmetic — diagonal normals carry ulp noise in the
        // dot products, so "same instant" crossings differ in the last bits
        // and sort by time. Axis-aligned planes with the record row lying
        // exactly ON both planes (the documented dA == 0 sitting-on-the-
        // plane semantics) give s == 0 and both events carry the row time.
        // cp0's plane is x=0 (n0 = P1-P0 = (10,0,0) -> unit (1,0,0)); cp1's
        // plane is y=0 (n1 = P2-P0 = (0,30,0) -> unit (0,1,0)); the row
        // (0,0,0) sits on both and is 10.0 m from cp1 -> radius 11 admits
        // the hit.
        let t = track_with_cps(
            r#"[
            {"kind":"gate","x":0.0,"y":0.0,"z":0.0,"radius_m":11.0},
            {"kind":"gate","x":10.0,"y":0.0,"z":0.0,"radius_m":11.0},
            {"kind":"gate","x":0.0,"y":30.0,"z":0.0,"radius_m":11.0}
        ]"#,
        );
        // cp2's plane is diagonal, dA and dB both negative: it never
        // crosses forward. Exactly two events, both at t 0.0, index order.
        let (ev, _) = compute_events(&t, &rows(&[(0.0, 0.0, 0.0, 0.0), (1.0, 0.0001, 0.0001, 0.0)]));
        assert_eq!(ev, vec![(0, 0.0), (1, 0.0)]);
    }

    #[test]
    fn loop_laps_after_repeat_start() {
        // Loop C0=(0,0) start, C1=(0,-10), C2=(10,-10), flown by jumping
        // segment-to-segment through the far waypoint W=(5000,5000) — any
        // plane crossing on a teleport leg is kilometres outside the 2 m
        // radius. Sequence start,1,2,start,1,2,start = two completed laps
        // plus the final start recross.
        let s = r#"{"schema":"darter_track","version":1,"name":"t","checkpoints":[
            {"kind":"start","x":0.0,"y":0.0,"z":0.0,"radius_m":2.0},
            {"kind":"gate","x":0.0,"y":-10.0,"z":0.0,"radius_m":2.0},
            {"kind":"gate","x":10.0,"y":-10.0,"z":0.0,"radius_m":2.0}
        ]}"#;
        let t = parse_str(s).expect("parses");
        assert!(t.is_loop);
        let ns = normals(&t);
        let crossing_order = [0usize, 1, 2, 0, 1, 2, 0];
        let mut pts: Vec<(f64, f64, f64, f64)> = Vec::new();
        let mut seg_starts = Vec::new(); // expected event time origin per crossing
        let mut tt = 0.0f64;
        for &gi in &crossing_order {
            let cp = &t.cps[gi];
            let n = ns[gi];
            // Crossing segment spans [tt, tt+0.2]: A and B symmetric about
            // the centre (hit == centre, s == 0.5), so the event lands at
            // ~tt+0.1.
            pts.push((
                tt,
                cp.pos[0] - 1.0 * n[0],
                cp.pos[1] - 1.0 * n[1],
                cp.pos[2] - 1.0 * n[2],
            ));
            seg_starts.push(tt);
            tt += 0.2;
            pts.push((
                tt,
                cp.pos[0] + 1.0 * n[0],
                cp.pos[1] + 1.0 * n[1],
                cp.pos[2] + 1.0 * n[2],
            ));
            tt += 1.0;
            pts.push((tt, 5000.0, 5000.0, 0.0));
        }
        let (ev, splits) = compute_events(&t, &rows(&pts));
        assert_eq!(ev.len(), crossing_order.len(), "unexpected events: {ev:?}");
        for (k, e) in ev.iter().enumerate() {
            assert_eq!(e.0, crossing_order[k], "event {k}: {ev:?}");
            assert!(
                (e.1 - (seg_starts[k] + 0.1)).abs() < 1e-9,
                "event {k} time {:?} vs {}",
                e.1,
                seg_starts[k] + 0.1
            );
        }
        // Start crossings at k = 0, 3, 6: two lap splits of ~3.6 s.
        assert_eq!(splits.len(), 2, "splits {splits:?}");
        for s in &splits {
            assert!((s - 3.6).abs() < 1e-9, "split {s}");
        }
    }

    #[test]
    fn open_track_has_no_laps() {
        let t = track_with_cps(
            r#"[
            {"kind":"gate","x":0.0,"y":0.0,"z":0.0,"radius_m":2.0},
            {"kind":"gate","x":10.0,"y":0.0,"z":0.0,"radius_m":2.0}
        ]"#,
        );
        let (ev, splits) = compute_events(
            &t,
            &rows(&[(-1.0, -1.0, 0.0, 0.0), (1.0, 1.0, 0.0, 0.0), (2.0, 11.0, 0.0, 0.0)]),
        );
        assert_eq!(ev.len(), 2);
        assert!(splits.is_empty());
    }
}