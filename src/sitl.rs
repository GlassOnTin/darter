//! UDP bridge to the Betaflight SITL target (src/platform/SIMULATOR in
//! upstream Betaflight). Replicates the wire behaviour of the Gazebo
//! BetaflightPlugin so the SITL consumes our fdm packets exactly as it
//! consumes Gazebo's.
//!
//! Ports (SITL side): fdm in on 9003 (server), RC in on 9004 (server),
//! servo_packet (normalised motors, one per received fdm packet) out on 9002.
//!
//! All values are little-endian; the SITL is a native process.

use std::io;
use std::net::{SocketAddr, UdpSocket};

use crate::quad::Quad;

pub const SITL_HOST: &str = "127.0.0.1";
pub const PORT_MOTORS: u16 = 9002;
pub const PORT_STATE: u16 = 9003;
pub const PORT_RC: u16 = 9004;

/// Sim-to-FC state packet, fields 1:1 with `fdm_packet`
/// (src/platform/SIMULATOR/target/SITL/target.h). 18 doubles, 144 bytes.
///
/// Frame conventions (what the Gazebo plugin puts on the wire):
/// - `imu_angular_velocity_rpy` / `imu_linear_acceleration_xyz` are the IMU
///   sensor samples in the FRD frame (x forward, y right, z down), i.e. the
///   FLU body frame flipped by Rx(pi). sitl.c negates the accel components
///   itself and maps gyro polarity via sitlGyroBodyFromSim().
/// - `imu_orientation_quat` is (w, x, y, z) conjugated by Rx(pi): qy/qz
///   negated. sitl.c undoes that and rotates the world by Rz(pi/2)
///   (ENU -> NWU) to get its internal attitude.
/// - `velocity_xyz` is ENU: east, north, up.
/// - `position_xyz` is (longitude, latitude, altitude). sitl.c mirrors
///   horizontal position around the first packet's origin (a Gazebo quirk it
///   works around), so we pre-mirror the same way around our origin.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FdmPacket {
    pub timestamp: f64,
    pub imu_angular_velocity_rpy: [f64; 3],
    pub imu_linear_acceleration_xyz: [f64; 3],
    pub imu_orientation_quat: [f64; 4],
    pub velocity_xyz: [f64; 3],
    pub position_xyz: [f64; 3],
    pub pressure: f64,
}

/// FC-to-sim motor packet, fields 1:1 with `servo_packet`. One packet per
/// received fdm packet; `motor_speed` is normalised 0..1 (3D mode: -1..1) in
/// Betaflight motor order M1..M4 (props-in: M1 RR, M2 FR, M3 RL, M4 FL).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ServoPacket {
    pub motor_speed: [f32; 4],
}

/// RC-in packet, fields 1:1 with `rc_packet`. `channels` are pulse widths in
/// microseconds (1000..2000) in the receiver order Betaflight's default map
/// ("AETR1234") expects: aileron/roll, elevator/pitch, THROTTLE, rudder/yaw,
/// then AUX1... (rc channel aliases ROLL=0, PITCH=1, THROTTLE=2, YAW=3 —
/// throttle and yaw are swapped relative to the alias enum.)
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RcPacket {
    pub timestamp: f64,
    pub channels: [u16; 16],
}

pub const FDM_WIRE_LEN: usize = 144;
pub const RC_WIRE_LEN: usize = 40;
pub const SERVO_WIRE_LEN: usize = 16;

impl FdmPacket {
    pub fn to_wire(&self) -> [u8; FDM_WIRE_LEN] {
        let mut buf = [0u8; FDM_WIRE_LEN];
        let mut off = 0;
        for v in [self.timestamp] {
            put_f64(&mut buf, &mut off, v);
        }
        for v in self.imu_angular_velocity_rpy {
            put_f64(&mut buf, &mut off, v);
        }
        for v in self.imu_linear_acceleration_xyz {
            put_f64(&mut buf, &mut off, v);
        }
        for v in self.imu_orientation_quat {
            put_f64(&mut buf, &mut off, v);
        }
        for v in self.velocity_xyz {
            put_f64(&mut buf, &mut off, v);
        }
        for v in self.position_xyz {
            put_f64(&mut buf, &mut off, v);
        }
        put_f64(&mut buf, &mut off, self.pressure);
        assert_eq!(off, FDM_WIRE_LEN);
        buf
    }
}

impl RcPacket {
    pub fn to_wire(&self) -> [u8; RC_WIRE_LEN] {
        let mut buf = [0u8; RC_WIRE_LEN];
        put_f64(&mut buf, &mut 0, self.timestamp);
        let mut off = 8;
        for c in self.channels {
            buf[off..off + 2].copy_from_slice(&c.to_le_bytes());
            off += 2;
        }
        assert_eq!(off, RC_WIRE_LEN);
        buf
    }
}

/// Parse a servo_packet payload; None unless it is exactly 16 bytes.
pub fn parse_servo(buf: &[u8]) -> Option<ServoPacket> {
    if buf.len() != SERVO_WIRE_LEN {
        return None;
    }
    let mut motor_speed = [0f32; 4];
    for (i, m) in motor_speed.iter_mut().enumerate() {
        let b: [u8; 4] = buf[i * 4..i * 4 + 4].try_into().ok()?;
        *m = f32::from_le_bytes(b);
    }
    Some(ServoPacket { motor_speed })
}

fn put_f64(buf: &mut [u8], off: &mut usize, v: f64) {
    buf[*off..*off + 8].copy_from_slice(&v.to_le_bytes());
    *off += 8;
}

/// WGS84 equatorial radius; spherical lat/lon from ENU metres is plenty for
/// sim GPS feeds.
const R_EARTH: f64 = 6378137.0;
const DEG: f64 = std::f64::consts::PI / 180.0;

/// Sea-level standard pressure (Pa). The gazebo bridge ignores the packet's
/// pressure and derives its own from altitude, so any plausible value works.
const SEA_LEVEL_PRESSURE: f64 = 101325.0;

/// Build the fdm packet for a sim state. `origin_lat`/`origin_lon` are the
/// lat/lon the sim world origin maps to; horizontal position is pre-mirrored
/// around them so the SITL's own mirror correction recovers the true position.
/// The sim world must start with the vehicle at the origin, matching sitl.c's
/// assumption that the first packet arrives at the spawn position.
pub fn fdm_from_state(quad: &Quad, origin_lat: f64, origin_lon: f64, timestamp: f64) -> FdmPacket {
    let (gyro, accel) = quad.imu(); // FLU body frame

    // FLU -> FRD is Rx(pi): x unchanged, y and z negated.
    let imu_angular_velocity_rpy = [gyro.x, -gyro.y, -gyro.z];
    let imu_linear_acceleration_xyz = [accel.x, -accel.y, -accel.z];

    // Plugin convention: q conjugated by Rx(pi) == qy/qz negated.
    let q = quad.state.quat;
    let imu_orientation_quat = [q.w, q.x, -q.y, -q.z];

    let pos = quad.state.pos;
    let lat = origin_lat + pos.y / R_EARTH / DEG;
    let lon = origin_lon + pos.x / (R_EARTH * (lat * DEG).cos()) / DEG;
    // Pre-mirror: sitl.c computes corrected = 2*origin - sent.
    let position_xyz = [
        2.0 * origin_lon - lon, // longitude
        2.0 * origin_lat - lat, // latitude
        pos.z,                  // altitude
    ];

    let vel = quad.state.vel; // ENU: x east, y north, z up
    FdmPacket {
        timestamp,
        imu_angular_velocity_rpy,
        imu_linear_acceleration_xyz,
        imu_orientation_quat,
        velocity_xyz: [vel.x, vel.y, vel.z],
        position_xyz,
        pressure: SEA_LEVEL_PRESSURE,
    }
}

/// Build an RC packet from normalised stick values, in AETR receiver order
/// (see `RcPacket`). Roll/pitch/yaw are -1..1 (centred 1500), throttle 0..1;
/// `aux3` is a raw pulse width written to AUX3 (CH7) — the arm switch slot,
/// matching the reference ArduPilot config where RC7 arms.
pub fn rc_packet(roll: f64, pitch: f64, yaw: f64, throttle: f64, aux3: u16) -> RcPacket {
    let stick = |v: f64| -> u16 { (1500.0 + (v.clamp(-1.0, 1.0) * 500.0)).round() as u16 };
    let thr = (1000.0 + (throttle.clamp(0.0, 1.0) * 1000.0)).round() as u16;
    let mut channels = [1000u16; 16];
    channels[0] = stick(roll);     // A: aileron / roll
    channels[1] = stick(pitch);    // E: elevator / pitch
    channels[2] = thr;             // T: throttle
    channels[3] = stick(yaw);      // R: rudder / yaw
    channels[6] = aux3;            // AUX3: arm switch
    RcPacket { timestamp: 0.0, channels }
}

/// UDP link to a Betaflight SITL process on localhost.
pub struct SimLink {
    /// Bound to PORT_MOTORS; the SITL sends servo packets here.
    motor_sock: UdpSocket,
    fdm_peer: SocketAddr,
    rc_peer: SocketAddr,
}

impl SimLink {
    pub fn new() -> io::Result<Self> {
        let motor_sock = UdpSocket::bind((SITL_HOST, PORT_MOTORS))?;
        motor_sock.set_nonblocking(true)?;
        let fdm_peer = SocketAddr::new(SITL_HOST.parse().unwrap(), PORT_STATE);
        let rc_peer = SocketAddr::new(SITL_HOST.parse().unwrap(), PORT_RC);
        Ok(Self { motor_sock, fdm_peer, rc_peer })
    }

    pub fn send_fdm(&self, pkt: &FdmPacket) -> io::Result<usize> {
        self.motor_sock.send_to(&pkt.to_wire(), self.fdm_peer)
    }

    pub fn send_rc(&self, pkt: &RcPacket) -> io::Result<usize> {
        self.motor_sock.send_to(&pkt.to_wire(), self.rc_peer)
    }

    /// Drain queued servo packets and return the newest motor set plus how
    /// many packets were drained (zeros included), or `(None, 0)`.
    pub fn try_recv_motors(&self) -> (Option<ServoPacket>, u64) {
        let mut last = None;
        let mut count = 0u64;
        let mut buf = [0u8; SERVO_WIRE_LEN];
        while let Ok(n) = self.motor_sock.recv(&mut buf) {
            count += 1;
            if n == SERVO_WIRE_LEN {
                last = parse_servo(&buf);
            }
        }
        (last, count)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::preset::Preset;
    use crate::quad::Quad;
    use crate::DVec3;

    /// Golden bytes from a C program compiled against the real
    /// target.h (see tools/wire_dump.c) — proves our manual packing matches
    /// the compiler's struct layout exactly.
    fn golden_fdm() -> Vec<u8> {
        include_bytes!("../tests/data/fdm_golden.bin").to_vec()
    }

    #[test]
    fn fdm_wire_matches_c_layout() {
        let pkt = FdmPacket {
            timestamp: 1.5,
            imu_angular_velocity_rpy: [0.1, 1.1, 2.1],
            imu_linear_acceleration_xyz: [4.0, 5.0, 6.0],
            imu_orientation_quat: [7.0, 8.0, 9.0, 10.0],
            velocity_xyz: [11.0, 12.0, 13.0],
            position_xyz: [14.0, 15.0, 16.0],
            pressure: 17.0,
        };
        assert_eq!(pkt.to_wire(), golden_fdm()[..FDM_WIRE_LEN]);
    }

    #[test]
    fn rc_wire_matches_c_layout() {
        let golden = include_bytes!("../tests/data/rc_golden.bin").to_vec();
        let mut channels = [1000u16; 16];
        for (i, c) in channels.iter_mut().enumerate() {
            *c = 1000 + 10 * i as u16;
        }
        let pkt = RcPacket { timestamp: 2.5, channels };
        assert_eq!(pkt.to_wire(), golden[..RC_WIRE_LEN]);
    }

    #[test]
    fn servo_parses_known_bytes() {
        let golden = include_bytes!("../tests/data/servo_golden.bin").to_vec();
        let s = parse_servo(&golden).expect("servo packet");
        assert_eq!(s.motor_speed, [0.1, 0.2, 0.3, 0.4]);
        assert!(parse_servo(&buf_short()).is_none());
    }

    fn buf_short() -> Vec<u8> {
        vec![0u8; SERVO_WIRE_LEN - 1]
    }

    #[test]
    fn fdm_from_state_level_hover() {
        // Spawn at the ground so one step lands in the ground branch and
        // accel holds the grounded +1 g value.
        let mut quad = Quad::new(Preset::FREESTYLE_5IN, DVec3::new(0.0, 0.0, 0.0));
        quad.throttle = [0.0; 4];
        quad.step(0.001);
        let pkt = fdm_from_state(&quad, 47.0, -122.0, 5.0);
        // Identity quat -> plugin quat still identity.
        assert_eq!(pkt.imu_orientation_quat, [1.0, 0.0, 0.0, 0.0]);
        // Hover/grounded specific force: +g on FLU z -> -g on FRD z.
        assert!((pkt.imu_linear_acceleration_xyz[2] + crate::quad::G).abs() < 1e-9);
        assert!(pkt.imu_linear_acceleration_xyz[0].abs() < 1e-9);
        // At the origin, position mirrors back to the origin itself.
        assert!((pkt.position_xyz[0] + 122.0).abs() < 1e-9); // lon
        assert!((pkt.position_xyz[1] - 47.0).abs() < 1e-9); // lat
        assert!(pkt.position_xyz[2].abs() < 1e-9);
        // ENU velocity passthrough (ground branch leaves a small bounce).
        assert!(pkt.velocity_xyz[2].abs() < 0.01);
    }

    #[test]
    fn rc_packet_maps_sticks_aetr() {
        // AETR receiver order: ch2 is throttle, ch3 is yaw.
        let rc = rc_packet(0.0, 1.0, -1.0, 0.5, 2000);
        assert_eq!(rc.channels[0], 1500); // roll neutral
        assert_eq!(rc.channels[1], 2000); // pitch +
        assert_eq!(rc.channels[2], 1500); // throttle 0.5
        assert_eq!(rc.channels[3], 1000); // yaw -
        assert_eq!(rc.channels[4], 1000); // aux1 untouched
        assert_eq!(rc.channels[6], 2000); // aux3: arm switch
    }
}