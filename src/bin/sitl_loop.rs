//! M0 bring-up loop: step the darter-core physics at 8 kHz on the wall
//! clock, feed Betaflight SITL at 250 Hz over UDP, apply the SITL's motor
//! outputs, and send RC. Closed loop: Betaflight's own PIDs fly the quad.
//!
//! Run Betaflight SITL first (no arguments), then this binary. `--ramp`
//! plays a scripted arm/throttle profile; without it the loop sends neutral
//! RC and stays disarmed, which is enough to verify traffic both ways.
//! `--radio` reads live stick values from the RadioMaster Pocket (js0)
//! instead of any scripted profile.
//!
//! usage: sitl_loop [--lat <deg>] [--lon <deg>] [--ramp] [--radio]
//!     [--terrain <terrain.bin>]

use darter_core::preset::Preset;
use darter_core::quad::Quad;
use darter_core::radio::Radio;
use darter_core::sitl::{fdm_from_state, rc_packet, RcPacket, SimLink};
use darter_core::terrain::{Ground, TerrainGrid};
use darter_core::DVec3;
use std::time::{Duration, Instant};

/// Physics substep (s): 125 us, 8 kHz.
const DT: f64 = 125e-6;
/// fdm packet rate (Hz).
const FDM_HZ: f64 = 250.0;
/// Physics substeps per fdm packet (8 kHz / 250 Hz).
const STEPS_PER_FDM: usize = 32;

/// Betaflight pulse-width units.
const CH_MIN: u16 = 1000;
const CH_MAX: u16 = 2000;

fn main() {
    let mut origin_lat = 47.0;
    let mut origin_lon = -122.0;
    let mut ramp = false;
    let mut radio_path: Option<String> = None;
    let mut terrain_path: Option<String> = None;
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        match a.as_str() {
            "--lat" => origin_lat = args.next().and_then(|v| v.parse().ok()).unwrap_or(origin_lat),
            "--lon" => origin_lon = args.next().and_then(|v| v.parse().ok()).unwrap_or(origin_lon),
            "--ramp" => ramp = true,
            "--radio" => radio_path = Some("/dev/input/js0".to_string()),
            "--terrain" => {
                terrain_path = Some(args.next().expect("--terrain needs a path"));
            }
            other => {
                eprintln!(
                    "unknown arg {other}; usage: sitl_loop [--lat <deg>] [--lon <deg>] [--ramp] [--radio] [--terrain <terrain.bin>]"
                );
                std::process::exit(2);
            }
        }
    }
    // Missing or malformed file = hard error here: no silent flat fallback.
    let terrain = match &terrain_path {
        Some(p) => {
            let g = TerrainGrid::load(p).unwrap_or_else(|e| {
                eprintln!("sitl_loop: {e}");
                std::process::exit(1);
            });
            println!(
                "terrain {p}: grid {}x{}, step {:.0} m, z [{:.1}, {:.1}]",
                g.cols, g.rows, g.step, g.z_min, g.z_max
            );
            Some(g)
        }
        None => None,
    };

    let link = SimLink::new().expect("bind UDP 9002");
    let spawn_z = terrain.as_ref().map_or(0.0, |g| g.h_at(0.0, 0.0));
    let mut quad = Quad::new(Preset::FREESTYLE_5IN, DVec3::new(0.0, 0.0, spawn_z));
    if let Some(g) = terrain {
        quad.ground = Ground::Grid(g);
    }
    let mut radio =
        radio_path.as_ref().map(|p| Radio::open(p).expect("open radio js device"));
    if radio.is_some() {
        println!("radio: live RC from {radio_path:?} (axes 0-3 -> AETR, axes 4-7 -> AUX1-4)");
    }
    println!(
        "darter sitl_loop: origin lat {origin_lat} lon {origin_lon}, physics 8 kHz, fdm {FDM_HZ} Hz"
    );

    let start = Instant::now();
    let mut fdm_ticks: u64 = 0;
    let mut last_status = 0.0;
    let mut motors = [0.0f64; 4]; // newest motor commands from the FC
    let mut servo_total = 0u64; // servo packets received (zeros included)
    let mut last_printed_axes = [0i16; 8];

    loop {
        let t = start.elapsed().as_secs_f64();

        // RC source: live radio, else the scripted profile. The FC's ARM box
        // sits on AUX3 (CH7), the switch the reference ArduPilot config arms
        // with; both RC sources drive it there.
        let (rc, arm_b, thr_f) = if let Some(r) = radio.as_mut() {
            let _ = r.poll();
            let channels = r.channels();
            let arm = channels[6] >= 1700;
            let thr = (channels[2] as f64 - 1000.0) / 1000.0;
            (RcPacket { timestamp: 0.0, channels }, arm, thr)
        } else {
            let (roll_u, pitch_u, yaw_u, thr_u, aux3_u) = rc_profile(t, ramp);
            let thr = thr_u as f64;
            let arm = aux3_u >= 1700;
            (rc_packet(roll_u, pitch_u, yaw_u, thr_u, aux3_u), arm, thr)
        };

        // Step physics one fdm period.
        for _ in 0..STEPS_PER_FDM {
            quad.step(DT);
        }

        // Apply the newest motor commands from the FC, then report state.
        let (servo, n) = link.try_recv_motors();
        servo_total += n;
        if let Some(s) = servo {
            motors = s.motor_speed.map(f64::from);
        }
        quad.throttle = motors;
        let _ = link.send_fdm(&fdm_from_state(&quad, origin_lat, origin_lon, t));
        let _ = link.send_rc(&rc);
        fdm_ticks += 1;

        if t - last_status >= 1.0 {
            last_status = t;
            let (roll, pitch, yaw) = euler_deg(quad.state.quat);
            let p = quad.state.pos;
            println!(
                "t={t:5.1} arm={:5} thr={thr_f:.3} z={:+7.3} vz={:+6.2} rpy=({:+6.1},{:+6.1},{:+6.1}) rpm={:6.0} n={servo_total} cmd=({:.2},{:.2},{:.2},{:.2}) bus={:5.2}V {:5.2}A soc={:.3}",
                arm_b,
                p.z,
                quad.state.vel.z,
                roll,
                pitch,
                yaw,
                quad.rpm[0],
                motors[0],
                motors[1],
                motors[2],
                motors[3],
                quad.bus_voltage(),
                quad.bus_current(),
                quad.soc(),
            );
            if let Some(r) = &radio {
                let axes: [i16; 8] = core::array::from_fn(|i| r.axis(i));
                if axes != last_printed_axes {
                    last_printed_axes = axes;
                    println!("        js axes: {axes:?}");
                }
            }
        }

        // Pace to wall clock: fdm tick i lands at start + i/FDM_HZ.
        let target = start + Duration::from_secs_f64(fdm_ticks as f64 / FDM_HZ);
        let now = Instant::now();
        if target > now {
            let d = target - now;
            if d > Duration::from_millis(2) {
                std::thread::sleep(d - Duration::from_millis(1));
            }
            while Instant::now() < target {
                std::hint::spin_loop();
            }
        }
    }
}

/// Scripted RC for the bring-up ramp. Roll/pitch/yaw neutral, throttle
/// profile: idle to 2 s, 0->0.6 over 2..7 s, hold to 15 s, down by 18 s.
/// AUX3 (arm switch, range 1700-2100) goes high at 1 s and back low at 18 s.
/// Without `--ramp`: everything neutral, disarmed.
fn rc_profile(t: f64, ramp: bool) -> (f64, f64, f64, f64, u16) {
    if !ramp {
        return (0.0, 0.0, 0.0, 0.0, CH_MIN);
    }
    let thr = if t < 2.0 {
        0.0
    } else if t < 7.0 {
        0.6 * (t - 2.0) / 5.0
    } else if t < 15.0 {
        0.6
    } else if t < 18.0 {
        0.6 * (1.0 - (t - 15.0) / 3.0)
    } else {
        0.0
    };
    let aux3 = if (1.0..18.0).contains(&t) { CH_MAX } else { CH_MIN };
    (0.0, 0.0, 0.0, thr, aux3)
}

fn euler_deg(q: darter_core::DQuat) -> (f64, f64, f64) {
    let (qw, qx, qy, qz) = (q.w, q.x, q.y, q.z);
    let roll = (2.0 * (qw * qx + qy * qz)).atan2(1.0 - 2.0 * (qx * qx + qy * qy));
    let sin_p = (2.0 * (qw * qy - qz * qx)).clamp(-1.0, 1.0);
    let pitch = sin_p.asin();
    let yaw = (2.0 * (qw * qz + qx * qy)).atan2(1.0 - 2.0 * (qy * qy + qz * qz));
    let deg = 180.0 / std::f64::consts::PI;
    (roll * deg, pitch * deg, yaw * deg)
}