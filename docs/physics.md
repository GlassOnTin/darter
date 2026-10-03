# The flight model: maths, implementation, and status

This document describes what the physics core computes and how, where every
number comes from, and what the model does not attempt. It describes the
implementation in this commit, checked against `src/` and the test suites; it
is not a spec of an ideal model. Section 13 names the known errors.

VISION.md requires that "the model and its known errors are published". This
document and `tests/calibration.rs` (the calibration report) are that
publication.

Every preset number carries a provenance label from `src/preset.rs`:

- `Measured`: bench data or a specification sheet.
- `Estimated`: a judgment call, bounded where possible.
- `Derived`: fitted from recorded bench data through a documented model.

`tests/calibration.rs::every_field_has_provenance` machine-checks that each
field of the preset carries one of the three labels, in a pinned order.

The core is one rigid body with four rotors, stepped in f64. Architecture in
one sentence: a bench-calibrated actuator chain converts throttle to rotor
speed to thrust and shaft power, the rigid body integrates those forces, and
the same state feeds both the built-in physics tests and a UDP bridge that
real Betaflight/INAV firmware flies against.

## 1. Frames, state, and conventions

Two frames (`src/quad.rs`):

- **Body**: FLU (x forward, y left, z up), the same orientation Gazebo's
  GazeboX4 plugins and Betaflight-SITL expect. Wiring to the upstream
  firmware flips this once, at the bridge (section 11), never inside the
  physics.
- **World**: ENU, z up.

`State { pos, vel, quat, omega }`: position and velocity in the world frame,
`quat` rotates body vectors into the world frame, `omega` is the body-frame
angular rate. `G = 9.80665 m/s^2`, `RPM_TO_RAD = pi/30`.

Motor order is Betaflight's mixer order: M1 rear-right, M2 front-right, M3
rear-left, M4 front-left, with `MOTOR_POS = [[-1,-1],[1,-1],[-1,1],[1,1]]`
(x forward, y left, in metres scaled by the arm length). Props-in: M1 and M4
spin clockwise (`SPIN_CCW = [0,1,1,0]`, so M2 and M3 spin counter-clockwise).

Sign conventions are pinned by `tests/physics.rs`, not by comments alone:
positive `omega.x` rolls right (right side down), positive `omega.y` pitches
nose-down, positive `omega.z` yaws counter-clockwise seen from above.

## 2. Time stepping

The core runs at 8 kHz (`SUBSTEP_DT = 125 us`, the rate a real FC's PID loop
runs), 32 substeps per 4 ms tick; the tick is the FDM/record/wind rate (250
Hz). `src/bin/sim_run/main.rs` owns this cadence.

Each substep, `Quad::step` runs one fixed sequence (order matters for
reproducibility; forces are evaluated on the start-of-substep state):

1. Air density `rho = density(pos.z)` (section 6).
2. Bus voltage `v_bus`: the battery's terminal voltage from the **previous**
   substep, floored at 1.0 V. One-step lag, deliberate, keeps the update
   order acyclic (section 7).
3. Air velocity in the body frame: `v_air = quat^-1 * (vel - wind)`. All four
   rotor discs share the axial component `v_ax = v_air.z` (section 13, gap 5).
4. Per motor: commanded duty `d` (clamped 0..1), rotor-speed target
   (section 3.2), first-order rotor lag (section 3.3), then thrust and shaft
   power (section 3.4), then electrical current `i = P_shaft/v_bus + I_idle`
   for `d > 0`, zero otherwise.
5. Forces and moments accumulate (section 4).
6. Drag: quadratic, per body axis, on air-relative velocity (section 4).
7. World acceleration `a = quat * (F/m) - g*zhat`; proper acceleration
   `a_prop = quat^-1 * (a + g*zhat)` (free fall reads 0, standing on the
   ground reads +1 g) is what the IMU output carries.
8. Velocity integration then position integration, using the **new**
   velocity (semi-implicit Euler). `vel += a*dt; pos += vel_new*dt`.
9. Angular: a gyroscopic cross term is computed from the **old** `omega`
   (`((I1-I2)*wy*wz, (I2-I0)*wz*wx, (I0-I1)*wx*wy)`), `omega` advances with
   explicit Euler, and the quaternion derivative `q' = 0.5 * q * (0, omega)`
   uses the **new** `omega` (written out in four components because glam has
   no DQuat scalar multiply), then normalized.
10. Battery coulomb count with the currents from step 4 (section 7).
11. Ground contact (section 5).

Integration scheme: explicit Euler on the rotational states and
battery/state variables, semi-implicit on translation. Nothing implicit, no
multi-stage integrator; with a 125 us step the rotor (tau 50 ms, section 3.3)
and the fastest aerodynamic time scales sit well above the step size, and
fixed order keeps trajectories byte-reproducible (section 10). The calibration
gates in `tests/calibration.rs` and `tests/physics.rs` are the check that the
scheme is adequate at this resolution (hover drift 5.2 um over 2 s; see
section 12).

## 3. The actuator chain

Throttle to thrust passes through five models, each pinned against bench
data where the data exists. The reference airframe is `FREESTYLE_5IN`
("5in-freestyle-6s"): T-HOBBY Velox V2306.5 V2 KV1950 motors, T-HOBBY
T5143S 5.1x4.3 tri-blade props, 6S 1300 mAh LiPo, 0.650 kg all-up mass. A
second fixture (iFlight XING2 2306 1755KV + Nazgul 5140) exists only as a
cross-check in `tests/calibration.rs`; it is not a playable preset.

### 3.1 Bench rotor-speed map

`rpm_nl(p, d)` interpolates 11 recorded bench points (duty 0.50..1.00,
rotor speed 20,195..30,991 rpm, bus voltage 25.05..24.51 V, current per
motor 7.51..35.70 A) piecewise-linearly, and extends below the lowest
measured duty with a power law `rpm(0.5) * (d/0.5)^k`, `k = 0.6179`
(labelled `Derived` in the preset). **Gap**: the repo records no fit input
for that exponent; it predates the provenance discipline and entered with
the core commit 7a0ff2a. Grep and `git log -S` find only its definition.
Section 13 repeats this.

### 3.2 Rotor-speed set-point with battery lag

`omega_target_rpm` converts a command into a rotor speed set-point through a
quasi-steady DC-motor model. The bench map is assumed to hold at the bench
operating point; running away from it, the rotor speed scales with applied
winding voltage:

```
i      := previous substep's current          (fixed-point seed)
denom  = v_bench(d) - i_bench(d) * R_m        (bench operating point)
repeat exactly twice (fixed count, for determinism):
    num    = v_bus - i * R_m                  (motor terminal voltage now)
    omega  = num > 0 ? rpm_nl(d) * num/denom : 0
    _, p_shaft = thrust_power(omega, v_ax, rho)
    i      = p_shaft / v_bus + I_idle
```

Two iterations, a fixed count, no convergence tolerance; the seed makes the
map stateful (previous current) but deterministic. The sag term
`(v_bus - i*R_m)/(v_bench - i_bench*R_m)` is what ties rotor speed to the
battery terminal voltage, so a sagging pack costs thrust exactly as a real
one does at full throttle.

### 3.3 Rotor lag

Measured rpm relaxes toward the set-point with a first-order lag:

```
rpm += (target - rpm) * min(1, dt/tau)        tau = 0.050 s (preset)
```

`tests/physics.rs::rotor_reaches_calibrated_steady_state` checks the
discrete decay constant against the analytic `(1 - dt/tau)^(tau/dt)` after a
throttle cut (1% gate), and the lag is why the hover test pre-spins rotors
for ~20 tau before checking drift.

### 3.4 Thrust law

Momentum-theory propeller with a linear climb de-rating, at axial inflow
`v_ax` and density `rho`:

```
T0 = ct0 * rho * D^4 * omega^2                (static/zero-advance)
                                             ct0 = 4.296e-3 (Derived)
climb      (v_ax > 0):  J = v_ax/(n*D), n = omega/(2*pi)
                        T  = max(0, T0 * (1 - J/j0))
                        j0 = 4.3/5.1 ~= 0.843, geometric pitch/diameter
                             (Estimated; UIUC 5x3.75 tri-blade polar
                             cross-check: zero-thrust J ~ 0.77 vs 0.75
                             geometric)
descent    (v_ax <= 0): T = T0
```

The descent branch is a **placeholder**, not a validated model. The source
comment says it plainly: real props keep thrust in the vortex-ring state but
this model does not attempt it (unvalidated; see `tests/calibration.rs`).
Measured consequence: terminal fall comes out -19.84 m/s, inside the
expected [15, 25] m/s gate, but the curve shape inside the vortex-ring
regime is not the real one. Section 13, gap 1; section 14.2 for the
replacement plan.

### 3.5 Induced velocity and shaft power

From momentum theory with a figure of merit and a profile-power term:

```
v_i0^2 = T / (2 * rho * A)                    A = pi*D^2/4
v_i    = ( -v_ax + sqrt(v_ax^2 + 4*v_i0^2) ) / 2
P_ind  = T * (v_ax + v_i)
P_shaft = P_ind / FoM + cp * rho * D^5 * omega^3
                       FoM = 0.70 (Estimated)
                       cp  = 2.315e-4 (Derived, fit)
```

`v_i * (v_ax + v_i) >= 0` for every `v_ax`, so induced power is never
negative and shaft power never reverses. `cp` was fitted so that
full-throttle electrical current matches the bench (section 12). The
mid-table power column of the bench sheet is NOT reproduced by any single
(FoM, cp) pair; that gap is documented in the calibration report, not
asserted away.

### 3.6 Yaw reaction torque

Rotor drag on the frame, from shaft power over angular rate:

```
tau_z += (1 - 2*spin_ccw_i) * P_shaft_i / omega_i      omega > 1e-6 guard
```

CW rotors (+1) and CCW rotors (-1) drag the frame in opposite senses, so
differential shaft power is the yaw actuator, exactly as on a real frame.
Torque reversal below the omega guard is ignored (throttle 0, omega 0).

### 3.7 What the current model does not include

No motor/ESC efficiency split beyond `I_idle` and the profile-power term, no
motor temperature derating, no ESC switching dynamics, no per-motor induced
velocity interaction (section 13, gaps 4-5, 14).

## 4. Forces, moments, and integration of motion

Total forces and moments in the body frame at the start of the substep:

```
F   = (sum_i 0, sum_i 0, sum_i T_i)                  (thrust, body z up)
tau_x = sum_i  arm * y_i * T_i                       (roll)
tau_y = -sum_i arm * x_i * T_i                       (pitch)
tau_z = sum_i (1 - 2*spin_ccw_i) * P_shaft_i/omega_i (yaw, section 3.6)
```

plus quadratic per-axis drag on the air-relative body velocity:

```
a_drag_i = -0.5 * rho * cda_axis * v_i * |v_i|
cda = [0.005, 0.005, 0.026] m^2 (Derived; vertical area dominates)
```

World-frame acceleration, then proper acceleration for the IMU:

```
a_world = quat * (F/m) - (0, 0, G)
a_prop  = quat^-1 * (a_world + (0, 0, G))    free fall -> 0, grounded -> +1 g
```

Angular integration (explicit Euler with a gyroscopic correction computed
from the old omega):

```
omega += (tau - gyro_cross) / I_ii * dt        per axis, I = [0.004, 0.004, 0.009] kg m^2
q      = normalize(q + 0.5 * q * (0, omega_new) * dt)
```

There is no attitude controller in the core: it is a bare rigid body.
Response to a ±0.005 differential step is a divergent angle by design
(~2 rad/s after 0.3 s), measured in `tests/calibration.rs`; controller-side
checks belong to the closed-loop tier (section 11) where real firmware does
the controlling.

## 5. Ground contact

Ground is `z = 0` (Flat) or a terrain grid. When `pos.z <= ground_height`:

- position clamps to the surface,
- descending vertical velocity reflects with restitution -0.2,
- horizontal velocity damps x0.9 per contact substep,
- omega damps x0.8 per contact substep,
- proper acceleration reads +1 g (the IMU sees contact, as it should).

`Ground::Flat` is the pre-M1 exact branch (`z = 0` always); `Ground::Grid`
wraps `TerrainGrid` (`src/terrain.rs`): the terrain.bin format (56-byte
header + row-major f64 heights) and `h_at` mirror
`tools/area_pack.py::terrain_h_at` op-for-op including the u >= v triangle
split, so the drawn surface and the contact surface are the same surface.

No contact friction model, no tip-over dynamics, no ground-effect
(aerodynamics): contact is a kinematic clamp with these factors (section 13,
gaps 3 and 15).

## 6. Air density

Isa atmosphere, derived not fitted:

```
rho(h) = 1.225 * (1 - 2.25577e-5 * h)^4.25588   (h in m)
```

clamped to h in [0, 11000] m; above 11 km the 11-km value is returned
rather than a garbage extrapolation. `src/air.rs` tests pin 1.1116 kg/m^3 at
1000 m and 0.3639 kg/m^3 at 11 km. Everything downstream (thrust, drag,
wind, hover solve) consumes this.

## 7. Battery

`src/battery.rs`: pack-only coulomb counter plus series resistance:

```
SoC'   = SoC - I_bus * dt / Q_As               Q = 4680 A*s (6S1300, Estimated)
V_bus  = OCV(SoC) - I_bus * R_pack             R = 0.008 Ohm
```

OCV is a piecewise-linear per-cell curve 3.30..4.18 V: an **estimated**
representative LiPo curve, not a measurement of any specific pack. The
battery's visible effects: motor-voltage sag through section 3.2, and
sim-side telemetry (V_bus, SoC in the record).

Honest gap: **nothing on the SITL bridge sees this battery** (no virtual
battery sensor exists upstream), so "the FC behaves correctly when the pack
is nearly empty" is untestable in closed-loop today. The battery affects
only the motor chain and sim-side telemetry. Section 11, section 13 gap 9.

`tests/calibration.rs` carries a hand-computed integration check: 1 s at
142.8 A on 4680 A*s must land SoC at exactly 0.96949, per-cell 4.14338 V,
pack 24.86031 V before sag, and shows the expected sag.

## 8. Sensor model

`src/sensor.rs` (optional layer; `SensorConfig::OFF` is bit-identical
passthrough so the fdm path stays deterministic without it):

- gyro noise sigma 0.0055 rad/s (0.005 dps/sqrt(Hz) at 8 kHz), turn-on bias
  sigma 0.0087 rad/s (0.5 dps), random-walk sigma 1.7e-6 rad/sqrt(s)
  (0.01 dps/sqrt(s)).
- accel noise sigma 0.25 m/s^2 (400 ug/sqrt(Hz)).
- vibration: `amp * thr_mean * (sin(phase) + 0.5*sin(2*phase))` added to
  accel and gyro, phase keyed to blade-pass frequency (mean rpm, 2-blade -> 2x
  rotor Hz), amplitude throttle-linear (`amp = 2.0` ~ 0.2 g at full
  throttle).

All magnitudes are `Estimated` (an assumed MPU-6000-class part at 8 kHz).
The vibration **shape** is a labelled placeholder: an isotropic per-axis
sinusoid pair, not a validated frame response. Section 13 gap 7; section
14.1 for the replacement plan.

Inline tests (`src/sensor.rs`): bit-identity under OFF, sigma within +/-5%
of the specified value at 80k samples, random-walk sigma growing as sqrt(t)
across an ensemble, Goertzel power at blade-pass and 2x blade-pass (and zero
at 401 Hz), zero vibration at zero throttle. RNG: xoshiro256** seeded via
splitmix64, one stream per source (section 10).

## 9. Wind

`src/wind.rs`: MIL-F-8785C Dryden gusts, 10-1000 ft band. Sigma_w = 0.1*W20
with W20 the 20-ft-wind speed (7.7 / 15.4 / 23.2 m/s at light / moderate /
severe). The altitude profile follows the spec's lateral/vertical channels:

```
sigma_h = sigma_w / (0.177 + 0.000823*h)^0.4
L_w     = h
L_h     = h / (0.177 + 0.000823*h)^1.2
Phi(omega) = sigma^2*L/(pi*V) * (1 + 3*(L*omega/V)^2) / (1 + (L*omega/V)^2)^2
```

Adaptation, labelled as such: a hovering quad has no longitudinal axis, so
east and north use the **lateral** spectrum as independent streams
(horizontal isotropy), the vertical channel uses the vertical spectrum, and
the longitudinal channel is dropped. Exact discrete 2-state cascade of
`H(s) = K'(1 + sqrt(3)*tau*s)/(1 + tau*s)^2`:

```
e = exp(-dt/tau)
x1' = e*x1 + (1-e)*w
x2' = e*x2 + (1-e)*x1'
y   = K' * (sqrt(3)*x1 + (1-sqrt(3))*x2)      K' = sigma*sqrt(L/(V*dt))
```

Stepped once per 4 ms tick and held across the 32 substeps. RNG stream is
independent (section 10); with gusts off (W20 <= 0) no randomness is
consumed at all, so wind-on/off never perturbs other random draws.

Gaps (section 13 gap 8): the 1000-2000 ft interpolation band is not
implemented; no terrain wind shadowing; advection speed V is floored at
1 m/s (Estimated). `tests/wind.rs` checks the realized spectrum against the
analytic one with a Welch estimator.

## 10. Determinism

The core is deterministic: f64 arithmetic, fixed operation order (section 2),
fixed iteration counts (2 for the set-point, 24 for the hover solve's joint
fixed point, 50 bisection steps), no wall clock in the physics, and seeded
RNG streams (`src/rng.rs`: xoshiro256** via splitmix64, one stream per
model, Box-Muller with the cos branch consumed first so the spare is
pinned). `tests/physics.rs::deterministic` runs 10,000 steps twice and
requires bit-identical states (`to_bits`), and `quat_stays_normalized`
requires 1e-9 after 10 s of aggressive input.

Records (`src/record.rs`) print fixed float precisions per field, so a
re-run's JSONL is byte-identical where the state is. The run-to-run change
detector is FNV-1a 64 over every line of the file; explicitly **not**
cryptographic. Binary provenance (fixtures, downloaded binaries, APK bytes)
uses sha256 (`src/sha256.rs`). Neither is a security boundary; both answer
"did anything change".

Measured: x86_64 and aarch64 produce byte-identical records (OPPO CPH2655,
Adreno 830, 2026-09 runs) because every operation is IEEE f64 with fixed
order; no platform-specific reassociation exists in the code.

Closed-loop is NOT reproducible: real Betaflight SITL carries its own
state and scheduling; the same seed and profile can limit-cycle or disarm at
the yaw burst depending on run (section 11, `tests/sitl_loop.rs`). The
determinism boundary is the core, and `--determinism-check` (core mode only)
exercises it in CI-adjacent runs by writing flight.jsonl and flight2.jsonl
and comparing FNV hashes.

## 11. The closed loop against real firmware

`src/bin/sim_run/main.rs` in closed mode spawns the real Betaflight SITL
binary as a child process and flies it over two links:

- **Servo/FDM over UDP**: the sim sends FdmPacket (144 B little-endian, 18
  f64: timestamp, gyro, accel, quaternion, velocity, lon/lat/alt, pressure)
  on 9003, receives ServoPacket (4 x f32 0..1 motor thrusts) on 9002, and RC
  (16 x u16 microseconds, AETR receiver order) on 9004.
- **MSP over TCP 5761** (upstream serial_tcp serves exactly one client per
  UART): CLI-over-MSPv2 for profile setup with a diff-readback gate, then
  telemetry (ATTITUDE/MOTOR at 25 Hz, STATUS at 4 Hz) on a dedicated thread
  that owns the link, because a poll costs 15-30 ms and inline polling
  stretched the loop past 2x wall time when it was first wired (observed
  2026-09-28).

Frame mapping (`src/sitl.rs::fdm_from_state_imu`), FLU body to FRD, one
Rx(pi) at the bridge: gyro `[wx, -wy, wz]`, accel `[ax, -ay, -az]`,
quaternion `(qw, qx, -qy, -qz)`. The yaw component deliberately keeps FLU
polarity (+CCW from above) instead of the plugin's FRD CW-positive: with an
FRD-polarity feed this SITL build's yaw PID+mixer chain runs positive
feedback and diverges (observed 2026-09-28); FLU polarity keeps the loop
bounded and a right-stick burst yaws CW like a real props-in quad, which is
what `tests/sitl_loop.rs` checks. The residual is measurable and documented:
after the burst the yaw loop either limit-cycles about the burst heading or
escalates into a spin the FC disarms via RUNAWAY_TAKEOFF, varying run to run
at the same seed. Documented, not fixed.

Two further quirks of the upstream plugin, absorbed in the bridge and each
verified upstream: position is pre-mirrored around the origin (sitl.c
computes `corrected = 2*origin - sent`), and the pressure field is ignored
upstream (the bridge sends a constant 101325 Pa). Velocity passes through
in ENU.

What the FC never sees, on purpose or otherwise:

- **Battery**: no virtual battery sensor upstream; the sim's battery state
  (section 7) cannot reach the FC, so closed-loop low-battery behaviour is
  untestable today (section 13 gap 9).
- **Attitude truth**: MSP ATTITUDE is the FC's own Mahony estimate; the sim
  never feeds it truth. MSP 102 gyro fields arrive in raw-count units
  (multiply 0.061035 for dps; `src/msp.rs` has the measured limits and the
  gyro_init.c citation), and MSP 109 alt/vario is the FC's fused KF relative
  to the disarmed baro capture, all units measured against the pinned SITL
  build and recorded in `src/record.rs`'s schema header.

Arming/flight sequencing (WaitGrace 5 s arm-grace, WaitArmed, 0.25 s settle +
0.5 s ramp at a throttle just above the solved hover) lives in the harness,
not the physics; the harness is the only place wall-clock and real-firmware
nondeterminism enter a run.

## 12. Calibration status

Policy (`tests/calibration.rs`): the held-out anchor. `ct0` is fit from the
full-throttle bench point alone; hover throttle, steady full-throttle
current, climb rate at 60% throttle, and terminal fall rate are **held-out
checks** against it, not re-fitted. Static-thrust gates at the two mid-table
bench points sit at a 6% tolerance.

Measured 2026-10-03 (`cargo test --test calibration --test physics`, 11 + 8
tests, all green; the same values as the original core commit 7a0ff2a's
dated evidence):

- static thrust: +3.4% at 20,195 rpm (675.8 vs 653.80 g bench),
  +1.0% at 25,784 rpm (1101.7 vs 1090.67 g),
  full-throttle anchor 1591.45 g exact by construction.
- full-throttle electrical: 875.18 W / 35.71 A vs bench 875.15 W / 35.70 A.
- hover throttle, fresh pack: 0.1547.
- full-throttle static equilibrium: 34.39 A/motor, bus 23.979 V,
  sag 1.101 V, rpm 30,370 (hand-iterated fixed point matched by the
  physics test to the printed digit).
- climb at 60% throttle: 23.31 m/s (bus 24.67 V, 12.95 A/motor).
- terminal fall: -19.84 m/s.
- hover hold: z drift +0.000005235 m over 2 s at throttle 0.1548
  (solved at `density(0.5)` because sea-level solve would sink; see
  the physics test comment).

Bands, and why they are bands: climb is gated [10, 35) m/s because the
falloff strength is set by measured UIUC CT(J) data, which cannot produce
18-19 m/s climbs; terminal fall is [15, 25] m/s; hover throttle [0.15,
0.30] with the expectation that it lands at the bottom of the band, as it
does (concave rpm map).

Documented, not fixed:

- The bench sheet's mid-table power column is not reproduced by any single
  (FoM, cp) pair. The calibration report says so; the fit anchors full
  throttle and accepts the mid-table error (section 3.5).
- The yaw-polarity divergence in closed loop (section 11).
- `k = 0.6179` for the sub-half-throttle rpm law has no recorded fit input
  (section 3.1).

Not yet done: flight-log residuals against real quad logs (the M4 milestone
item), which would turn several estimated labels into measured ones.

## 13. Known errors and gaps

In order of physics significance, all verifiable in source:

1. **Descent thrust is a placeholder** (`T = T0` for `v_ax <= 0`): the
   vortex-ring regime's thrust curve is not modelled; terminal velocity
   still lands in a plausible band. Replacement path in section 14.2.
2. **`rpm_sub_k` provenance**: the power-law exponent below duty 0.5 is
   labelled `Derived` with no recorded fit input (section 3.1).
3. **No ground effect**: thrust near the ground ignores in-ground-plane
   flow changes; the physics is the same 1 m above the deck as 50 m up.
4. **Shared axial inflow**: all four discs see the same `v_ax = v_air.z`;
   no oblique inflow from body lateral/forward velocity, no
   rotor-rotor wash (downwash of rear onto front under acceleration).
5. **No body aerodynamic moments or lift**: drag is the only body
   aerodynamic force, quadratic per axis; no pitching moment from the
   fuselage, no Magnus, no prop-wash-on-frame drag.
6. **Battery current one-step lag**: `v_bus` is previous-substep's value
   (deliberate, for update-order simplicity; section 2 step 2).
7. **Vibration shape is a placeholder** (isotropic sinusoid pair,
   section 8); only the magnitude is estimated, the shape is invented.
8. **Wind gaps**: 1000-2000 ft interpolation band, terrain shadowing,
   point winds (thermals, rotor wash turbulence) all missing; advection
   speed floored at 1 m/s.
9. **Battery is pack-only with an estimated OCV curve**, and the FC never
   sees it (section 7, section 11).
10. **Sensor magnitudes are estimated**; no datasheet measurement of any
    specific IMU part.
11. **Rigid body only**: no airframe flex, no prop flexibility, no
    aeroelastic coupling. Section 14.1 is the direction.
12. **One playable preset**; the cross-check fixture exists only inside
    `tests/calibration.rs`.
13. **Motor thermal and ESC dynamics** absent (no winding-temperature
    derate, no PWM switching loss beyond the profile-power term).
14. **No contact friction model / tip-over dynamics**; landing at an angle
    is not specially handled.
15. **The bridge's yaw polarity divergence** is a fidelity gap accepted for
    stability (documented in section 11).
16. **Pressure field is constant** into the FDM; unused upstream (barometer
    fidelity in closed loop comes only through the FC's own estimate chain
    if at all).

## 14. Raising fidelity: where an offline FEM (fenics-rs) fits

Compute constraint first: the core steps f64 at 8 kHz with fixed operation
order, on a phone, with renderer headroom. Nothing a variational FEM solve
produces is affordable inside that loop, and it shouldn't try to be. The
architecture that fits this project is: **offline compute against a preset,
landing as preset data** (a coefficient, a table, a frequency list) the
runtime consumes through the same analytic paths the bench data already
feeds. That is exactly how the bench curve became `rpm_curve`: recorded
once, stored, consumed forever after at zero runtime cost.

fenics-rs (github.com/GlassOnTin/fenics-rs, AGPL-3.0, licence-compatible
with darter) is a pure-Rust variational FEM framework: simplicial meshes,
Lagrange P1-P3, DG, Raviart-Thomas and Nedelec elements, sparse LU and CG
over faer-rs, Newton-Raphson for nonlinear systems, a modal eigensolver
(K u = omega^2 M u), stabilized Stokes, transient heat, neo-Hookean
deformations, cross-checked against DOLFINx 0.10.0 within its published
tolerances. Three directions from darter's needs, honestly scoped:

### 14.1 Frame rigidity via modal analysis (nearest fit)

Today's vibration model is an invented shape (section 8). What modal data
would give: the preset gains a frame-resonance table, the sensor's vibration
placeholder is replaced by resonances excited at blade-pass harmonics
(deterministically, from the same mean-rpm phase input), and Betaflight's
RPM-filter/notch tuning behaviour emerges for the right physical reason: the
FC is, in real life, tuned to filter exactly these frequencies. That last
point matters for the vision of this repo as a virtual hardware test
platform; notch placement on a real frame is a standard Betaflight task and
a sim that needs no notch tuning is lying about the rigidity of airframes.

fenics-rs has the pieces this needs (tetrahedral meshing, elasticity,
eigenvalue solve). The real work is geometry (a credible 5-inch frame as a
tet mesh, including arms as beams not boxes) and honest material properties;
the resonance table stays `Estimated` until a real frame is tapped and
measured, which is a cheap physical test (hammer tap + the accelerometer
already in a quad). Convergence point: the M4 vibration milestone.

### 14.2 Wake and inflow (honest scoping, cheap wins first)

A wake solve is **unavailable regardless of budget**: fenics-rs solves
linear, Poisson, elasticity, heat and Stokes-class systems; it has no
convective incompressible (Navier-Stokes) solver in the tree, and a
rotor-wake problem is convective at any Reynolds number a quad flies at.
That does not block the actual gap:

- The descent/vortex-ring placeholder (section 13 gap 1) has published
  replacement curves; a `T_descent(v_ax)` table in the preset, taken from
  the literature and gated against the terminal-fall band, is a literature
  step with no FEM at all. This is the cheap win and it comes first.
- Oblique inflow (a real correction when the body moves laterally, section
  13 gap 4) likewise has standard propeller-theory corrections. No FEM.
- fenics-rs *could* validate such a fitted table afterwards: an
  axisymmetric momentum-sink disc solve is Stokes-adjacent and inside what
  the library does. Optional validation of a fitted table, not generation
  of the physics.

### 14.3 EM antenna patterns (far term, name the gap)

fenics-rs already has Nedelec H(curl) elements, the right element family
for electromagnetics. What it lacks for this use: a time-harmonic
Maxwell/Helmholtz solver (the library solves real-valued systems), complex
arithmetic in its forms, a PML/radiation boundary, and a far-field
transform. None of those exist today; an antenna-pattern capability is
therefore a solver build-out, not a configuration. The cheaper path stays:
measured patterns or textbook dipole/monopole formulas feeding the future RC
link model, so failsafes trip for the right physical reason (link budget
with a real radiation pattern) rather than an invented one. Section 14.4's
compute constraint applies on top.

### 14.4 Milestone fit

All three directions converge at **M4** (vibration, residuals, weather).
Until then none of this changes M1-M3 scope, and fenics-rs stays out of
darter's dependency tree: the core library keeps its glam+libc-only
guarantee, and if offline FEM tooling ever runs inside this repo it joins as
a bin-local dependency (the established pattern, like serde_json in the
track rung). The wasm build is interesting for a future standalone
education/geometry tool, not for the flight loop.

## 15. Where the numbers live

- Preset values and labels: `src/preset.rs` (the 18-field provenance table,
  machine-checked).
- Bench fixtures: the rpm/current/voltage curves inside `src/preset.rs`,
  cross-check fixture in `tests/calibration.rs`.
- Physics constants: `src/quad.rs` (G, RPM_TO_RAD), `src/air.rs` (ISA).
- Sensor constants: `src/sensor.rs` (`SensorConfig::DEFAULT`).
- Wind model: `src/wind.rs`.
- Wire formats and quirks: `src/sitl.rs`, `src/msp.rs`, `src/record.rs`.
- Harness cadence/sequencing: `src/bin/sim_run/main.rs`.
- The calibration report: `tests/calibration.rs`.
- The physics gates: `tests/physics.rs`, wind spectrum check in
  `tests/wind.rs`, closed-loop yaw residual in `tests/sitl_loop.rs`.