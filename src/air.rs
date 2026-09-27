//! Standard-atmosphere air properties for the thrust and drag paths.

/// Sea-level standard density (kg/m^3) at 15 C, 101325 Pa.
pub const RHO_0: f64 = 1.225;

/// ISA troposphere air density (kg/m^3) at geometric altitude `h` (m).
/// Layer-bounded (valid 0..11 km, linear-temperature layer); above the
/// layer top it returns the 11 km value rather than extrapolating garbage.
/// Values are derived, not fitted: rho = rho0 * (1 - L*h/T0)^(g/(R*L)-1).
pub fn density(h: f64) -> f64 {
    let h = h.clamp(0.0, 11_000.0);
    RHO_0 * (1.0 - 2.25577e-5 * h).powf(4.25588)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sea_level_and_altitude() {
        assert!((density(0.0) - RHO_0).abs() < 1e-9);
        // Standard atmosphere: 1.1116 kg/m^3 at 1000 m; at the 11 km
        // tropopause p = 22632 Pa, T = 216.65 K -> p/(R T) = 0.3639 kg/m^3.
        assert!((density(1000.0) - 1.1116).abs() < 2e-3);
        assert!((density(11_000.0) - 0.3639).abs() < 2e-3);
        // Beyond the layer: clamped, no extrapolation blowup.
        assert!((density(20_000.0) - density(11_000.0)).abs() < 1e-12);
    }
}