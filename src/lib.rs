//! darter-core: flight physics core for the Darter drone simulator.
//!
//! Deterministic, f64, fixed-step. This crate owns no I/O: inputs are plain
//! numbers, so the same core serves the desktop spike, the Godot client, and
//! the headless research binary. Sensor imperfection (noise, bias, vibration)
//! is layered on by a sensor model, never baked into the rigid-body state.

pub mod air;
pub mod battery;
pub mod preset;
pub mod quad;
pub mod radio;
pub mod sitl;

pub use glam::{DQuat, DVec3};