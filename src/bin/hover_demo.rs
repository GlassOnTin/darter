use darter_core::air::RHO_0;
use darter_core::{preset::Preset, quad::hover_throttle, quad::Quad};
use glam::DVec3;

/// Headless hover demo: 5" preset holding hover for 5 s at 8 kHz steps.
fn main() {
    let mut quad = Quad::new(Preset::FREESTYLE_5IN, DVec3::new(0.0, 0.0, 0.1));
    let hover = hover_throttle(&quad.preset, quad.preset.battery, 1.0, RHO_0);
    quad.throttle = [hover; 4];

    let dt = 125e-6; // 8 kHz, matches a 5" PID loop
    let steps = (5.0 / dt) as usize;
    println!(
        "preset={}  hover_throttle={:.4}  dt={}s",
        quad.preset.name, hover, dt
    );
    for n in 0..=steps {
        // Hold the pack at full charge so the trace shows the actuator
        // equilibrium, not battery drift.
        quad.battery.soc = 1.0;
        quad.step(dt);
        if n % 2000 == 0 {
            let s = &quad.state;
            println!(
                "t={:5.2}  z={:+8.5}  vel_z={:+7.4}  |omega|={:7.4}  rpm={:7.0}  bus={:5.2}V",
                n as f64 * dt,
                s.pos.z,
                s.vel.z,
                s.omega.length(),
                quad.rpm[0],
                quad.bus_voltage()
            );
        }
    }
}