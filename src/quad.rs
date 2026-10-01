use crate::air::density;
use crate::battery::{Battery, BatterySpec};
use crate::preset::Preset;
use crate::terrain::Ground;
use glam::{DQuat, DVec3};

/// Standard gravity (m/s^2).
pub const G: f64 = 9.80665;

/// rpm to rad/s.
const RPM_TO_RAD: f64 = std::f64::consts::PI / 30.0;

/// Body frame is FLU: x forward, y left, z up — the right-handed triple used
/// by Gazebo and by the Betaflight SITL's internal (NWU) axes, so IMU samples
/// convert to the SITL wire format with a single Rx(pi) flip. World frame is
/// ENU with z up. Motor order follows the Betaflight mixer: M1 rear-right,
/// M2 front-right, M3 rear-left, M4 front-left. Spin directions are the
/// "props in" layout: M1/M4 clockwise, M2/M3 counter-clockwise, viewed from
/// above.
///
/// Sign conventions, verified by tests: positive omega.x rolls right (right
/// side down), positive omega.y pitches the nose down, positive omega.z yaws
/// counter-clockwise seen from above.
const MOTOR_POS: [[f64; 2]; 4] = [
    [-1.0, -1.0], // M1 rear-right
    [1.0, -1.0],  // M2 front-right
    [-1.0, 1.0],  // M3 rear-left
    [1.0, 1.0],   // M4 front-left
];
const SPIN_CCW: [f64; 4] = [0.0, 1.0, 1.0, 0.0];

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct State {
    /// World position (m), ENU, z up.
    pub pos: DVec3,
    /// World velocity (m/s).
    pub vel: DVec3,
    /// Body-to-world orientation.
    pub quat: DQuat,
    /// Angular velocity (rad/s), body frame.
    pub omega: DVec3,
}

impl State {
    pub fn level_at(pos: DVec3) -> Self {
        Self {
            pos,
            vel: DVec3::ZERO,
            quat: DQuat::IDENTITY,
            omega: DVec3::ZERO,
        }
    }
}

/// Rotor rpm at the bench voltage for a commanded throttle: piecewise-linear
/// through the measured (throttle, rpm) points, power law `rpm0 * (d/d0)^k`
/// below the first point (the bench table starts at d = 0.5).
pub fn rpm_nl(p: &Preset, d: f64) -> f64 {
    let curve = p.rpm_curve;
    if d <= 0.0 {
        return 0.0;
    }
    let (d0, r0) = curve[0];
    if d < d0 {
        return r0 * (d / d0).powf(p.rpm_sub_k);
    }
    let last = curve.len() - 1;
    if d >= curve[last].0 {
        return curve[last].1;
    }
    for k in 0..last {
        if d <= curve[k + 1].0 {
            let t = (d - curve[k].0) / (curve[k + 1].0 - curve[k].0);
            return curve[k].1 + t * (curve[k + 1].1 - curve[k].1);
        }
    }
    curve[last].1
}

/// Bench-table value (current or voltage) interpolated on the rpm_curve
/// throttle grid, clamped at both ends. Below the first point the bench
/// measured no current/voltage, so the first point's value carries over.
fn bench_value(values: &[f64], curve: &[(f64, f64)], d: f64) -> f64 {
    debug_assert_eq!(values.len(), curve.len());
    if d <= curve[0].0 {
        return values[0];
    }
    let last = curve.len() - 1;
    if d >= curve[last].0 {
        return values[last];
    }
    for k in 0..last {
        if d <= curve[k + 1].0 {
            let t = (d - curve[k].0) / (curve[k + 1].0 - curve[k].0);
            return values[k] + t * (values[k + 1] - values[k]);
        }
    }
    values[last]
}

/// Static (still-air) thrust and shaft power of one prop at rotor speed
/// `omega_rad`: the Glauert induced-velocity quadratic for the power path and
/// the advance-ratio falloff law for the thrust path.
///
/// Thrust: `T = T0 (1 - J/J0)` for axial inflow `v_ax > 0` (climbing through
/// still air), `J = v_ax / (rev_s * D)`, `J0 = pitch/diameter`, clamped at
/// zero; `T = T0` for `v_ax <= 0`. The descent branch is a placeholder — real
/// props keep thrust in the vortex-ring state but this model does not attempt
/// it (unvalidated, see the calibration report). `T0 = ct0 * rho * D^4 *
/// omega^2` matches the bench static thrust column within ~3.5%.
///
/// Power: induced `T * (v_ax + v_i) / FoM` plus profile `c_p * rho * D^5 *
/// omega^3`, with `v_i` the positive Glauert root of
/// `v_i^2 + v_ax*v_i - T/(2 rho A) = 0`. `v_i * (v_ax + v_i) >= 0` for every
/// `v_ax`, so shaft power never goes negative.
pub fn thrust_power(p: &Preset, omega_rad: f64, v_ax: f64, rho: f64) -> (f64, f64) {
    let diam = p.prop_diameter_m;
    let disc_area = std::f64::consts::PI * diam * diam / 4.0;
    let t0 = p.prop_ct0 * rho * diam.powi(4) * omega_rad * omega_rad;
    let (thrust, v_i0_sq) = if v_ax <= 0.0 {
        (t0, t0 / (2.0 * rho * disc_area))
    } else {
        let n = omega_rad / (2.0 * std::f64::consts::PI);
        let j = if n > 0.0 { v_ax / (n * diam) } else { f64::INFINITY };
        let t = (t0 * (1.0 - j / p.prop_j0)).max(0.0);
        (t, t / (2.0 * rho * disc_area))
    };
    let v_i = 0.5 * (-v_ax + (v_ax * v_ax + 4.0 * v_i0_sq).sqrt());
    let p_induced = thrust * (v_ax + v_i);
    let p_shaft = p_induced / p.prop_fom + p.prop_cp * rho * diam.powi(5) * omega_rad.powi(3);
    (thrust, p_shaft)
}

/// Quasi-steady rotor rpm for throttle `d` at bus voltage `v_bus`: the bench
/// curve scaled by the resistive sag ratio `(V - I R_m) / (V_bench - I_bench
/// R_m)`. The motor current `I` enters through the power model, so the pair
/// is solved by a 2-iteration fixed point seeded from the previous step's
/// current (fixed iteration count — deterministic).
fn omega_target_rpm(p: &Preset, d: f64, v_bus: f64, i_seed: f64, v_ax: f64, rho: f64) -> f64 {
    let rpm_nl = rpm_nl(p, d);
    if rpm_nl <= 0.0 {
        return 0.0;
    }
    let v_bench = bench_value(p.rpm_v_bench, p.rpm_curve, d);
    let i_bench = bench_value(p.rpm_cur_bench, p.rpm_curve, d);
    let denom = v_bench - i_bench * p.motor_r_m_ohm;
    let mut i = i_seed;
    let mut omega = 0.0;
    for _ in 0..2 {
        let num = v_bus - i * p.motor_r_m_ohm;
        omega = if num > 0.0 { rpm_nl * num / denom } else { 0.0 };
        let (_, p_shaft) = thrust_power(p, omega * RPM_TO_RAD, v_ax, rho);
        i = p_shaft / v_bus + p.motor_i_idle_a;
    }
    omega.max(0.0)
}

/// Rigid-body quad with a bench-calibrated actuator chain (throttle ->
/// rotor-speed curve -> sag -> thrust/power -> current -> battery), quadratic
/// per-axis drag on air-relative velocity, and a ground plane (flat by
/// default, or the terrain grid a harness installs).
/// Deterministic: f64 and fixed operation order throughout, so identical
/// inputs reproduce identical trajectories.
pub struct Quad {
    pub preset: Preset,
    pub state: State,
    /// Rotor speed (rpm), lagged toward the quasi-steady target.
    pub rpm: [f64; 4],
    /// Commanded throttle per motor, 0..1.
    pub throttle: [f64; 4],
    /// Pack state: coulomb count and bus sag. Nothing on the SITL bridge sees
    /// this (no virtual battery sensor upstream); it feeds motor sag and
    /// sim-side telemetry.
    pub battery: Battery,
    /// Wind velocity (m/s), world frame. Zero unless a wind source sets it;
    /// drag always acts on air-relative velocity.
    pub wind: DVec3,
    /// Last per-motor electrical current (A). Also the fixed-point seed.
    pub i_mot: [f64; 4],
    /// Last proper acceleration (m/s^2, FLU body frame) for the IMU output.
    accel: DVec3,
    /// Ground plane under the quad: Flat (z = 0) by default, a DEM grid when
    /// the harness loads one. Kept out of `new()` so every existing call
    /// site and its trajectories stay byte-identical.
    pub ground: Ground,
}

impl Quad {
    pub fn new(preset: Preset, pos: DVec3) -> Self {
        Self {
            battery: Battery::new(preset.battery),
            preset,
            state: State::level_at(pos),
            rpm: [0.0; 4],
            throttle: [0.0; 4],
            wind: DVec3::ZERO,
            i_mot: [0.0; 4],
            accel: DVec3::ZERO,
            ground: Ground::Flat,
        }
    }

    /// Pack terminal voltage (V) under the last bus current.
    pub fn bus_voltage(&self) -> f64 {
        self.battery.bus_voltage()
    }

    /// Pack state of charge, 0..1.
    pub fn soc(&self) -> f64 {
        self.battery.soc
    }

    /// Total bus current of the last step (A).
    pub fn bus_current(&self) -> f64 {
        self.battery.i_bus
    }

    /// Advance one fixed step of `dt` seconds (spike target: 125 us, 8 kHz).
    pub fn step(&mut self, dt: f64) {
        let p = &self.preset;
        let rho = density(self.state.pos.z);
        // Bus voltage as of the previous step's current: one step of lag in
        // the pack response, which keeps the per-step update order fixed.
        let v_bus = self.battery.bus_voltage().max(1.0);

        // Air-relative velocity in the body frame (aircraft minus wind).
        let v_air_body = self.state.quat.conjugate() * (self.state.vel - self.wind);
        // Axial inflow (m/s, + = climbing through still air): the same for
        // all four discs in this model.
        let v_ax = v_air_body.z;

        // Thrust and torque in the body frame. r x F with F along +z gives
        // tau_x = y*T and tau_y = -x*T per motor.
        let mut force = DVec3::ZERO;
        let mut torque = DVec3::ZERO;
        let mut i_bus = 0.0;
        for i in 0..4 {
            let d = self.throttle[i].clamp(0.0, 1.0);
            let target = omega_target_rpm(p, d, v_bus, self.i_mot[i], v_ax, rho);
            self.rpm[i] += (target - self.rpm[i]) * (dt / p.rpm_tau_s).min(1.0);

            let omega = self.rpm[i] * RPM_TO_RAD;
            let (thrust, p_shaft) = thrust_power(p, omega, v_ax, rho);
            // Electrical current: shaft power over bus voltage plus the ESC
            // idle draw; motor+ESC efficiency is absorbed into FoM/c_p. A
            // stopped motor (d = 0) draws nothing — the idle draw is a
            // spinning/armed-motor value.
            let cur = if d > 0.0 { p_shaft / v_bus + p.motor_i_idle_a } else { 0.0 };
            self.i_mot[i] = cur;
            i_bus += cur;

            force.z += thrust;
            torque.x += p.arm_m * MOTOR_POS[i][1] * thrust;
            torque.y += -p.arm_m * MOTOR_POS[i][0] * thrust;
            // Reaction torque Q = P_shaft / omega along the spin axis.
            if omega > 1e-6 {
                torque.z += (1.0 - 2.0 * SPIN_CCW[i]) * p_shaft / omega;
            }
        }

        // Per-axis quadratic drag force on the air-relative velocity (the
        // opposing sign is already in the expression).
        let drag = DVec3::new(
            -0.5 * rho * p.cda[0] * v_air_body.x * v_air_body.x.abs(),
            -0.5 * rho * p.cda[1] * v_air_body.y * v_air_body.y.abs(),
            -0.5 * rho * p.cda[2] * v_air_body.z * v_air_body.z.abs(),
        );
        force += drag;

        // Linear motion.
        let a_body = force / p.mass_kg;
        let a_world = self.state.quat * a_body - DVec3::new(0.0, 0.0, G);
        // Proper acceleration for the IMU: a - g, so free fall reads zero
        // and a grounded airframe reads +1 g.
        self.accel = self.state.quat.conjugate() * (a_world + DVec3::new(0.0, 0.0, G));
        self.state.vel += a_world * dt;
        self.state.pos += self.state.vel * dt;

        // Angular motion, diagonal inertia, gyroscopic cross term included.
        let w = self.state.omega;
        let inertia = p.inertia;
        let gyro = DVec3::new(
            (inertia[1] - inertia[2]) * w.y * w.z,
            (inertia[2] - inertia[0]) * w.z * w.x,
            (inertia[0] - inertia[1]) * w.x * w.y,
        );
        let w_dot = DVec3::new(
            (torque.x - gyro.x) / inertia[0],
            (torque.y - gyro.y) / inertia[1],
            (torque.z - gyro.z) / inertia[2],
        );
        self.state.omega += w_dot * dt;

        // Orientation: q' = 0.5 * q * (0, omega) for body-frame rates,
        // written out because glam has no scalar-multiply for DQuat.
        let wx = self.state.omega.x;
        let wy = self.state.omega.y;
        let wz = self.state.omega.z;
        let q = self.state.quat;
        let dqx = 0.5 * (q.w * wx + q.y * wz - q.z * wy);
        let dqy = 0.5 * (q.w * wy - q.x * wz + q.z * wx);
        let dqz = 0.5 * (q.w * wz + q.x * wy - q.y * wx);
        let dqw = -0.5 * (q.x * wx + q.y * wy + q.z * wz);
        self.state.quat = DQuat::from_xyzw(
            q.x + dqx * dt,
            q.y + dqy * dt,
            q.z + dqz * dt,
            q.w + dqw * dt,
        )
        .normalize();

        // Discharge the pack with this step's bus current.
        self.battery.step(dt, i_bus);

        // Ground contact: rest at the ground height under the craft (Flat is
        // z = 0, the pre-M1 form; height() returns a literal 0.0 there), damp
        // motion. A grounded airframe's proper acceleration is +1 g, which
        // the free-fall expression misses.
        let gh = self.ground.height(self.state.pos.x, self.state.pos.y);
        if self.state.pos.z <= gh {
            self.state.pos.z = gh;
            if self.state.vel.z < 0.0 {
                self.state.vel.z *= -0.2;
            }
            self.state.vel.x *= 0.9;
            self.state.vel.y *= 0.9;
            self.state.omega *= 0.8;
            self.accel = self.state.quat.conjugate() * DVec3::new(0.0, 0.0, G);
        }
    }

    /// Simulated IMU read: body-frame gyro (rad/s) and proper acceleration
    /// (m/s^2). Noise and bias belong to the sensor model, not to this state.
    pub fn imu(&self) -> (DVec3, DVec3) {
        (self.state.omega, self.accel)
    }
}

/// Throttle that holds a level hover in still air at battery state `soc`
/// and density `rho` (pass `RHO_0` for sea level, `density(alt)` for an
/// altitude), solved by bisection on the quasi-static model (converged
/// current fixed point). Deterministic; ~50 bisection steps.
pub fn hover_throttle(p: &Preset, battery: BatterySpec, soc: f64, rho: f64) -> f64 {
    let v_ocv = battery.ocv_v(soc);
    let mg = p.mass_kg * G;
    let net_thrust = |d: f64| -> f64 {
        // Joint fixed point over (rotor speed, motor current) at the bus
        // voltage the hover current itself implies. Converged (24 rounds).
        let mut i = p.motor_i_idle_a;
        let mut thrust = 0.0;
        for _ in 0..24 {
            let v_bus = (v_ocv - 4.0 * i * battery.r_pack).max(1.0);
            let omega = omega_target_rpm(p, d, v_bus, i, 0.0, rho) * RPM_TO_RAD;
            let (t, p_shaft) = thrust_power(p, omega, 0.0, rho);
            thrust = t;
            i = p_shaft / v_bus + p.motor_i_idle_a;
        }
        thrust
    };
    let mut lo = 0.02f64;
    let mut hi = 0.48f64;
    for _ in 0..50 {
        let mid = 0.5 * (lo + hi);
        if 4.0 * net_thrust(mid) < mg {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    0.5 * (lo + hi)
}