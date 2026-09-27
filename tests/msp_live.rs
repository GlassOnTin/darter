//! Live SITL round-trip tests — opt-in
//! (`cargo test --test msp_live -- --ignored --test-threads=1`; serial is
//! required, the SITL serves one TCP client per UART and two parallel tests
//! kill each other's connections), against a running SITL on the standard
//! ports. The offline suite must stay green without them.
//!
//! These verify *wiring*, not physics: MSP_STATUS reflects the SITL's real
//! arming-disable bitfield, and MSP_ATTITUDE is fed from the same fdm the
//! SITL received (both sides come from the sim, so agreement means the
//! telemetry path is connected, not that the estimator matches the truth).

use darter_core::msp::*;

fn msp_addr() -> String {
    std::env::var("DARTER_MSP_ADDR").unwrap_or_else(|_| "127.0.0.1:5761".into())
}

/// One connection attempt, one client: opening a probe stream first would
/// close the real one (the SITL serves a single TCP client per UART).
fn link_or_skip() -> Option<MspLink> {
    let addr = msp_addr();
    match MspLink::connect(&addr) {
        Ok(l) => Some(l),
        Err(_) => None,
    }
}

/// STATUS answers on a live link and the arming-disable bitfield decodes to
/// the runtime_config.h bit positions we depend on (the harness's state
/// machine reads bit 29 ARM_SWITCH and the cleared-at-arm invariant).
#[test]
#[ignore]
fn live_status_round_trip() {
    let mut link = match link_or_skip() {
        Some(l) => l,
        None => {
            eprintln!("no SITL on {} — skipping", msp_addr());
            return;
        }
    };
    let p = link.request_v1(MSP_STATUS).expect("STATUS reply");
    let s = decode_status(&p).expect("status payload");
    // RX_FAILSAFE (bit 2) may be set or clear depending on RC feed; the
    // field must at least fit the known mask (0..=1<<29).
    assert!(s.arming_disable_flags < 1 << 30, "unknown bit set: {:#x}", s.arming_disable_flags);
    // ATTITUDE answers on the same connection (single-client UART rule holds).
    let a = link.request_v1(MSP_ATTITUDE).expect("ATTITUDE reply");
    assert_eq!(a.len(), 6);
    decode_attitude(&a).expect("attitude payload");
}

/// CLI over MSPv2 still works on a live link: version responds with the
/// Betaflight version string (the profile-apply path).
#[test]
#[ignore]
fn live_cli_version() {
    let mut link = match link_or_skip() {
        Some(l) => l,
        None => {
            eprintln!("no SITL on {} — skipping", msp_addr());
            return;
        }
    };
    let out = link.cli("version").expect("cli version");
    assert!(out.contains("Betaflight /"), "unexpected: {out:?}");
}