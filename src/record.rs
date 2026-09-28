//! JSONL flight record: one header line, then one line per sample, with the
//! fields in the fixed order below. Output is hand-formatted (no serde) so
//! the bytes are deterministic by construction: fixed field order, fixed
//! float precision, no map iteration.
//!
//! The run-to-run change detector is FNV-1a 64-bit over every written byte
//! — enough to say "these two runs produced identical records", explicitly
//! NOT cryptographic (binary provenance uses sha256, src/sha256.rs).
//!
//! Line schemas (field order is the wire contract; appending fields is a
//! version bump, reordering is a new schema):
//!   header: {"schema":"darter_record","version":3,"mode":..,"seed":..,
//!            "duration_s":..,"preset":..,"profile":[..],
//!            "sensors":{..}|null,"sitl":{..}|null}
//!     (v2 added sensors provenance, v3 added the g*/alt/vario FC fields)
//!   sample: {"t":..,"px":..,"py":..,"pz":..,"vx":..,"vy":..,"vz":..,
//!            "qw":..,"qx":..,"qy":..,"qz":..,"wx":..,"wy":..,"wz":..,
//!            "r0":..,"r1":..,"r2":..,"r3":..,"i0":..,"i1":..,"i2":..,"i3":..,
//!            "vbus":..,"soc":..,
//!            "att_r":..,"att_p":..,"att_y":..,"m0":..,"m1":..,"m2":..,"m3":..,
//!            "g0":..,"g1":..,"g2":..,"alt":..,"vario":..,"arm":..,"flags":..}
//! The FC telemetry fields (att_*, m*, g*, alt, vario, arm, flags) are omitted
//! when no telemetry was sampled for that tick; the core fields are always
//! present.
//!
//! FC field units (all measured against the SITL build pinned in the header):
//! - att_r/att_p: MSP 108 roll/pitch in DECIDEGREES (imu.c writes 1800/pi
//!   steps); att_y: MSP 108 yaw in DEGREES 0..360 (msp.c passes it through
//!   DECIDEGREES_TO_DEGREES). All three are the FC's own Mahony estimate,
//!   not the fed truth — see src/msp.rs Attitude for the measured limits.
//! - g0/g1/g2: MSP 102 filtered gyro in RAW-COUNT units (gyroADCf/scale);
//!   multiply by 0.061035 for dps. NOT dps despite the source symbol
//!   gyroRateDps (gyro_init.c:823).
//! - alt/vario: MSP 109 fused altitude (cm, KF estimate relative to the
//!   disarmed baro capture) and vario (cm/s).

use std::fs::File;
use std::io::{self, BufWriter, Write};
use std::path::Path;

pub const SCHEMA_NAME: &str = "darter_record";
pub const SCHEMA_VERSION: u32 = 3;

/// FNV-1a 64-bit (offset 14695981039346656037, prime 1099511628211).
pub fn fnv1a64(data: &[u8], mut hash: u64) -> u64 {
    for &b in data {
        hash ^= u64::from(b);
        hash = hash.wrapping_mul(0x100000001b3);
    }
    hash
}

pub const FNV1A64_INIT: u64 = 0xcbf2_9ce4_8422_2325;

pub struct SitlProvenance {
    pub path: String,
    pub sha256: String,
    pub version: String,
}

pub struct RecordHeader {
    pub mode: &'static str,
    pub seed: u64,
    pub duration_s: f64,
    pub preset: &'static str,
    pub profile: Vec<String>,
    pub sitl: Option<SitlProvenance>,
    /// Sensor model provenance (None = noise off, pure rigid-body truth).
    pub sensors: Option<crate::sensor::SensorConfig>,
}

/// One sampled tick. FC telemetry is optional (None in core-only runs and
/// on ticks without an MSP poll).
pub struct Sample {
    pub t: f64,
    pub pos: crate::DVec3,
    pub vel: crate::DVec3,
    pub quat: crate::DQuat,
    pub omega: crate::DVec3,
    pub rpm: [f64; 4],
    pub i_mot: [f64; 4],
    pub vbus: f64,
    pub soc: f64,
    pub fc: Option<FcSample>,
}

#[derive(Clone)]
pub struct FcSample {
    /// MSP_RAW_IMU (102) filtered gyro in RAW-COUNT units (x0.061035 = dps);
    /// the same signal the PID loop sees.
    pub gyro_raw: [f64; 3],
    /// MSP_ATTITUDE (108): roll/pitch in decidegrees, yaw in degrees —
    /// the FC's own Mahony estimate, not the fed truth (see src/msp.rs
    /// Attitude for the measured limits of that estimate).
    pub att_cdeg: [i32; 3],
    /// FC motor outputs, normalised 0..1, M1..M4.
    pub motors: [f32; 4],
    /// MSP_ALTITUDE (109): fused KF altitude estimate, cm (relative to the
    /// disarmed baro capture), plus vario cm/s (0 when USE_VARIO is off).
    pub alt_cm: i32,
    pub vario_cms: i16,
    /// MSP_STATUS arming-disable bitfield.
    pub arming_disable: u32,
    /// MSP_STATUS first-32 flight-mode box bits.
    pub flight_flags: u32,
}

pub struct RecordWriter {
    out: BufWriter<File>,
    hash: u64,
}

impl RecordWriter {
    pub fn create(path: &Path) -> io::Result<Self> {
        Ok(Self {
            out: BufWriter::new(File::create(path)?),
            hash: FNV1A64_INIT,
        })
    }

    fn write_line(&mut self, line: &str) -> io::Result<()> {
        self.out.write_all(line.as_bytes())?;
        self.out.write_all(b"\n")?;
        self.hash = fnv1a64(line.as_bytes(), self.hash);
        self.hash = fnv1a64(b"\n", self.hash);
        Ok(())
    }

    pub fn write_header(&mut self, h: &RecordHeader) -> io::Result<()> {
        let mut s = String::with_capacity(256);
        s.push_str("{");
        s.push_str(&format!("\"schema\":\"{SCHEMA_NAME}\",\"version\":{SCHEMA_VERSION}"));
        s.push_str(&format!(",\"mode\":\"{}\"", h.mode));
        s.push_str(&format!(",\"seed\":{}", h.seed));
        s.push_str(&format!(",\"duration_s\":{:.3}", h.duration_s));
        s.push_str(&format!(",\"preset\":\"{}\"", h.preset));
        s.push_str(",\"profile\":[");
        for (i, line) in h.profile.iter().enumerate() {
            if i > 0 {
                s.push(',');
            }
            s.push_str(&json_str(line));
        }
        s.push(']');
        match &h.sensors {
            Some(c) => {
                s.push_str(&format!(
                    ",\"sensors\":{{\"gyro_noise_std\":{:.3e},\"gyro_bias_std\":{:.3e},\"gyro_bias_rw_std\":{:.3e},\"accel_noise_std\":{:.3e},\"vib_accel_amp\":{:.3},\"vib_gyro_amp\":{:.3},\"vib2_scale\":{:.3},\"seed\":{}}}",
                    c.gyro_noise_std, c.gyro_bias_std, c.gyro_bias_rw_std,
                    c.accel_noise_std, c.vib_accel_amp, c.vib_gyro_amp,
                    c.vib2_scale, c.seed
                ));
            }
            None => s.push_str(",\"sensors\":null"),
        }
        match &h.sitl {
            Some(p) => {
                s.push_str(&format!(
                    ",\"sitl\":{{\"path\":{},\"sha256\":{},\"version\":{}}}",
                    json_str(&p.path),
                    json_str(&p.sha256),
                    json_str(&p.version)
                ));
            }
            None => s.push_str(",\"sitl\":null"),
        }
        s.push('}');
        self.write_line(&s)
    }

    pub fn write_sample(&mut self, s: &Sample) -> io::Result<()> {
        let mut b = String::with_capacity(512);
        b.push_str(&format!(
            "{{\"t\":{:.3},\"px\":{:.6},\"py\":{:.6},\"pz\":{:.6},\"vx\":{:.6},\"vy\":{:.6},\"vz\":{:.6},\"qw\":{:.9},\"qx\":{:.9},\"qy\":{:.9},\"qz\":{:.9},\"wx\":{:.6},\"wy\":{:.6},\"wz\":{:.6},\"r0\":{:.2},\"r1\":{:.2},\"r2\":{:.2},\"r3\":{:.2},\"i0\":{:.3},\"i1\":{:.3},\"i2\":{:.3},\"i3\":{:.3},\"vbus\":{:.4},\"soc\":{:.6}",
            s.t,
            s.pos.x, s.pos.y, s.pos.z,
            s.vel.x, s.vel.y, s.vel.z,
            s.quat.w, s.quat.x, s.quat.y, s.quat.z,
            s.omega.x, s.omega.y, s.omega.z,
            s.rpm[0], s.rpm[1], s.rpm[2], s.rpm[3],
            s.i_mot[0], s.i_mot[1], s.i_mot[2], s.i_mot[3],
            s.vbus, s.soc,
        ));
        if let Some(fc) = &s.fc {
            b.push_str(&format!(
                ",\"att_r\":{},\"att_p\":{},\"att_y\":{},\"m0\":{:.4},\"m1\":{:.4},\"m2\":{:.4},\"m3\":{:.4},\"g0\":{:.1},\"g1\":{:.1},\"g2\":{:.1},\"alt\":{},\"vario\":{},\"arm\":{},\"flags\":{}",
                fc.att_cdeg[0], fc.att_cdeg[1], fc.att_cdeg[2],
                fc.motors[0], fc.motors[1], fc.motors[2], fc.motors[3],
                fc.gyro_raw[0], fc.gyro_raw[1], fc.gyro_raw[2],
                fc.alt_cm, fc.vario_cms,
                fc.arming_disable, fc.flight_flags,
            ));
        }
        b.push('}');
        self.write_line(&b)
    }

    /// Flush and close; returns the running FNV-1a hash of all written bytes.
    pub fn finish(mut self) -> io::Result<u64> {
        self.out.flush()?;
        Ok(self.hash)
    }
}

/// Minimal JSON string encoding for ASCII provenance values (path, version).
fn json_str(s: &str) -> String {
    let mut out = String::with_capacity(s.len() + 2);
    out.push('"');
    for c in s.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            c if (c as u32) < 0x20 => out.push_str(&format!("\\u{:04x}", c as u32)),
            c => out.push(c),
        }
    }
    out.push('"');
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample(t: f64, z: f64) -> Sample {
        Sample {
            t,
            pos: crate::DVec3::new(1.0, 2.0, z),
            vel: crate::DVec3::new(0.0, 0.0, 0.5),
            quat: crate::DQuat::IDENTITY,
            omega: crate::DVec3::ZERO,
            rpm: [100.0; 4],
            i_mot: [1.5; 4],
            vbus: 25.08,
            soc: 1.0,
            fc: None,
        }
    }

    /// Field order is fixed and the header lands first.
    #[test]
    fn record_bytes_are_ordered() {
        let dir = std::env::temp_dir().join(format!("darter-rec-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("flight.jsonl");
        let mut w = RecordWriter::create(&path).unwrap();
        w.write_header(&RecordHeader {
            mode: "core",
            seed: 7,
            duration_s: 2.0,
            preset: "FREESTYLE_5IN",
            profile: vec!["aux 0 0 2 1700 2100 0 0".into()],
            sitl: None,
            sensors: None,
        })
        .unwrap();
        w.write_sample(&sample(0.0, 0.5)).unwrap();
        w.write_sample(&sample(0.004, 0.5)).unwrap();
        let hash = w.finish().unwrap();

        let text = std::fs::read_to_string(&path).unwrap();
        let mut lines = text.lines();
        let header = lines.next().unwrap();
        assert!(header.starts_with("{\"schema\":\"darter_record\",\"version\":3,\"mode\":\"core\",\"seed\":7,\"duration_s\":2.000,\"preset\":\"FREESTYLE_5IN\",\"profile\":[\"aux 0 0 2 1700 2100 0 0\"],\"sensors\":null,\"sitl\":null}"));
        let s0 = lines.next().unwrap();
        assert!(
            s0.starts_with("{\"t\":0.000,\"px\":1.000000,\"py\":2.000000,\"pz\":0.500000,"),
            "first fields not in contract order: {s0}"
        );
        assert!(s0.ends_with("\"vbus\":25.0800,\"soc\":1.000000}"), "no fc fields: {s0}");
        let hash2 = {
            let mut w = RecordWriter::create(&path).unwrap();
            w.write_header(&RecordHeader {
                mode: "core",
                seed: 7,
                duration_s: 2.0,
                preset: "FREESTYLE_5IN",
                profile: vec!["aux 0 0 2 1700 2100 0 0".into()],
                sitl: None,
                sensors: None,
            })
            .unwrap();
            w.write_sample(&sample(0.0, 0.5)).unwrap();
            w.write_sample(&sample(0.004, 0.5)).unwrap();
            w.finish().unwrap()
        };
        assert_eq!(hash, hash2, "same content, different hash");
        std::fs::remove_dir_all(&dir).unwrap();
    }

    /// Identical bytes -> identical FNV; one changed byte -> different.
    #[test]
    fn fnv1a64_separates_and_repeats() {
        let a = fnv1a64(b"flight line one\n", FNV1A64_INIT);
        assert_eq!(fnv1a64(b"flight line one\n", FNV1A64_INIT), a);
        assert_ne!(fnv1a64(b"flight line two\n", FNV1A64_INIT), a);
        // Reference value for the empty stream (hand-checked).
        assert_eq!(FNV1A64_INIT, 0xcbf29ce484222325);
    }

    /// Provenance strings with quotes/backslashes survive the escape.
    #[test]
    fn json_str_escapes() {
        assert_eq!(json_str(r"a\b"), r#""a\\b""#);
        assert_eq!(json_str("q\"x"), "\"q\\\"x\"");
    }
}