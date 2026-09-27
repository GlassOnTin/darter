use darter_core::air::density;
use darter_core::{preset::Preset, quad::hover_throttle, quad::Quad, DVec3};

const DT: f64 = 125e-6; // 8 kHz, the PID-loop rate this core targets

fn level_quad() -> Quad {
    Quad::new(Preset::FREESTYLE_5IN, DVec3::new(0.0, 0.0, 0.5))
}

/// Freeze the pack at full charge before every step so tests exercise the
/// actuator chain at a known bus voltage instead of drifting with the
/// coulomb counter.
fn freeze_battery(quad: &mut Quad) {
    quad.battery.soc = 1.0;
}

/// Free fall: zero throttle reads zero proper acceleration (to within the
/// drag deceleration) and falls at ~g.
#[test]
fn free_fall() {
    let mut quad = level_quad();
    quad.state.pos.z = 2.0;
    for _ in 0..1600 {
        quad.step(DT); // 0.2 s
    }
    let (gyro, accel) = quad.imu();
    // Drag on a ~1.96 m/s fall decelerates by ~0.09 m/s^2, so the speed is a
    // few mm/s under the vacuum value.
    assert!((quad.state.vel.z - (-9.80665 * 0.2)).abs() < 0.03);
    assert!(accel.length() < 0.12, "proper accel {:?} != 0", accel);
    assert_eq!(gyro.length(), 0.0);
}

/// The numeric hover solve holds level hover: seeded at rest with the solved
/// throttle, the quad neither climbs nor sinks once the rotors have spun up.
/// The pack is frozen because the solve targets one battery state.
#[test]
fn hover_holds() {
    let mut quad = level_quad();
    // Solve at the hold altitude: thrust scales with density, so a
    // sea-level solve at 0.5 m would sink (drift ~ g*(1-rho/rho0)*t^2/2,
    // measured -0.66 mm at 0.5 m and -93 mm at 50 m before the fix).
    let hover = hover_throttle(&quad.preset, quad.preset.battery, 1.0, density(0.5));
    quad.throttle = [hover; 4];
    // Spin up at the hold altitude itself (rotors need ~20 tau; from rest the
    // airframe sinks ~2.5 cm while spooling, still above the ground plane),
    // then re-level. Pre-spinning at a different altitude leaves rpm at that
    // altitude's equilibrium (measured +0.15 mm drift for a 5 m pre-spin, the
    // excess decaying over ~3 rpm time constants), so hold-altitude spin-up
    // is what makes the solve's density target match the flight.
    quad.state.pos.z = 0.5;
    for _ in 0..8_000 {
        freeze_battery(&mut quad);
        quad.step(DT); // 1 s
    }
    quad.state.pos.z = 0.5;
    quad.state.vel = DVec3::ZERO;
    for _ in 0..16_000 {
        freeze_battery(&mut quad);
        quad.step(DT); // 2 s
    }
    let s = &quad.state;
    println!(
        "hover hold: z drift {:+.9} m over 2 s at throttle {:.4}",
        s.pos.z - 0.5,
        hover
    );
    assert!(
        (s.pos.z - 0.5).abs() < 1e-4,
        "z drift {} m over 2 s at hover throttle {:.4}",
        s.pos.z - 0.5,
        hover
    );
    assert!(s.vel.length() < 1e-5, "hover velocity {:?}", s.vel);
    assert!(s.omega.length() < 1e-6);
}

/// Full throttle reaches a steady rotor speed at static conditions: the
/// bench curve scaled by pack and motor sag, current from the power model.
/// The velocity is zeroed every step so v_ax = 0 (otherwise the quad climbs
/// ~36 m/s in the 2 s window and the equilibrium is the climbing one, with
/// the advance-ratio falloff and induced power in play — measured separately
/// at 30,968 rpm / 30.07 A, which is not this quantity). Hand-iterated fixed
/// point on the model formulas (fresh 6S pack: v_bus = 25.08 - 4*I*0.008,
/// rpm = 30991*(v_bus - I*0.067)/(24.51 - 35.70*0.067), I = P_shaft/v_bus +
/// 1.28 at v_ax = 0): 30,370 rpm, 34.40 A/motor, bus 23.979 V, sag 1.101 V.
/// The bench table itself reads 30,991 rpm at 24.51 V; the sim flies a
/// 25.08 V OCV pack, hence the difference.
#[test]
fn rotor_reaches_calibrated_steady_state() {
    let mut quad = level_quad();
    quad.throttle = [1.0; 4];
    for _ in 0..16_000 {
        quad.state.vel = DVec3::ZERO; // hold static (v_ax = 0)
        freeze_battery(&mut quad);
        quad.step(DT); // 2 s = 40 tau
    }
    let expected_rpm = 30_370.0;
    println!(
        "static full throttle: rpm {:.1} (expected {expected_rpm}), i {:.2} A, bus {:.3} V, sag {:.3} V",
        quad.rpm[0],
        quad.i_mot[0],
        quad.bus_voltage(),
        quad.preset.battery.ocv_v(1.0) - quad.bus_voltage()
    );
    assert!(
        (quad.rpm[0] - expected_rpm).abs() / expected_rpm < 0.01,
        "steady rpm {} vs hand-iterated {}",
        quad.rpm[0],
        expected_rpm
    );
    assert!(
        (quad.i_mot[0] - 34.40).abs() < 0.5,
        "steady current {} vs hand-iterated 34.40 A",
        quad.i_mot[0]
    );
    // Rotor lag: cut the throttle and check the e^(-t/tau) decay one tau
        // later (discrete form (1 - dt/tau)^(tau/dt), no target movement since
    // d = 0 drives the target to exactly zero).
    quad.throttle = [0.0; 4];
    let tau_steps = (quad.preset.rpm_tau_s / DT).round() as usize; // 400
    let from = quad.rpm[0];
    for _ in 0..tau_steps {
        quad.step(DT);
    }
    let expected = from * (1.0 - 1.0 / tau_steps as f64).powi(tau_steps as i32);
    assert!(
        (quad.rpm[0] - expected).abs() / expected < 0.01,
        "lagged rpm {} vs first-order {}",
        quad.rpm[0],
        expected
    );
}

/// Props-in layout: rear-right + front-left (both CW) yaw counter-clockwise.
#[test]
fn yaw_torque_sign() {
    let mut quad = level_quad();
    quad.throttle = [0.8, 0.0, 0.0, 0.8];
    for _ in 0..400 {
        quad.step(DT); // 0.05 s
    }
    assert!(quad.state.omega.z > 0.0, "expected CCW yaw, got {:?}", quad.state.omega);
}

/// Right-side thrust lifts the right side, which is roll left: negative
/// omega.x in the FLU frame.
#[test]
fn roll_torque_sign() {
    let mut quad = level_quad();
    quad.throttle = [0.8, 0.8, 0.0, 0.0];
    for _ in 0..400 {
        quad.step(DT);
    }
    assert!(quad.state.omega.x < 0.0, "expected right side up, got {:?}", quad.state.omega);
}

/// Front thrust raises the nose: nose up is negative omega.y in this frame.
#[test]
fn pitch_torque_sign() {
    let mut quad = level_quad();
    quad.throttle = [0.0, 0.8, 0.0, 0.8];
    for _ in 0..400 {
        quad.step(DT);
    }
    assert!(quad.state.omega.y < 0.0, "expected nose up, got {:?}", quad.state.omega);
}

/// Identical inputs reproduce identical trajectories, bit for bit.
#[test]
fn deterministic() {
    let mut a = level_quad();
    let mut b = level_quad();
    let steps = 10_000;
    for n in 0..steps {
        let t = n as f64 * DT;
        let thr = 0.3 + 0.2 * (t * 7.0).sin();
        a.throttle = [thr, thr * 0.9, thr * 1.1, thr];
        b.throttle = [thr, thr * 0.9, thr * 1.1, thr];
        a.step(DT);
        b.step(DT);
    }
    let sa = &a.state;
    let sb = &b.state;
    let pa = sa.pos.to_array();
    let pb = sb.pos.to_array();
    for (x, y) in pa.iter().zip(pb.iter()) {
        assert_eq!(x.to_bits(), y.to_bits());
    }
    assert_eq!(a.rpm, b.rpm);
    assert_eq!(a.i_mot, b.i_mot);
}

/// Quaternion stays unit-length under sustained rotation rates.
#[test]
fn quat_stays_normalized() {
    let mut quad = level_quad();
    quad.throttle = [0.8, 0.1, 0.1, 0.8];
    for _ in 0..80_000 {
        quad.step(DT); // 10 s of aggressive torque
    }
    let q = quad.state.quat;
    let norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    assert!((norm - 1.0).abs() < 1e-9, "quat norm {norm}");
}