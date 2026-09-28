//! MSP client tests, all offline against captured SITL frames.
//!
//! The goldens in tests/data/msp_golden/ are replies from the live SITL at
//! rest (capture session 2026-09-28, Betaflight 2026.12.0-alpha), not
//! synthesised: they pin the framing (markers, checksum field order) and the
//! payload layouts the harness depends on. Any change here means the wire
//! contract changed.

use darter_core::msp::*;

fn golden(name: &str) -> Vec<u8> {
    let path = format!("tests/data/msp_golden/{name}.bin");
    std::fs::read(&path).unwrap_or_else(|e| panic!("missing fixture {path}: {e}"))
}

/// CRC-8/DVB-S2 catalogue check value.
#[test]
fn crc8_dvb_s2_check_value() {
    assert_eq!(crc8_dvb_s2(b"123456789"), 0xBC);
}

#[test]
fn crc8_xor_check() {
    assert_eq!(crc8_xor(&[0u8, 101]), 101); // the $M< request crc for STATUS
    assert_eq!(crc8_xor(&[27, 101]), 27 ^ 101 ^ 0); // len ^ cmd ^ payload(0)
}

#[test]
fn golden_status_v1() {
    let (d, f, n) = parse_reply(&golden("status_v1")).unwrap().unwrap();
    assert_eq!(d, b'M');
    assert_eq!(n, 33);
    assert_eq!(f.cmd, MSP_STATUS);
    let s = decode_status(&f.payload).expect("status payload");
    assert_eq!(s.cycle_time_us, 1002);
    assert_eq!(s.i2c_errors, 0);
    assert_eq!(s.sensors, 0x27); // ACC | BARO | MAG | GYRO
    assert_eq!(s.flight_flags, 0);
    assert_eq!(s.pid_profile, 0);
    assert_eq!(s.load_percent, 0);
    assert_eq!(s.arming_disable_flags, 0x04); // RX_FAILSAFE at rest (no RC)
    assert_eq!(s.rate_profile_count, 4);
    assert_eq!(s.battery_profile_count, 3);
    assert_eq!(s.battery_profile, 0);
}

/// MSP_STATUS_EX (150) differs from 101 only at [13:15]; the decoder's
/// shared tail must read the same.
#[test]
fn golden_status_ex_v1() {
    let (_, f, _) = parse_reply(&golden("status_ex_v1")).unwrap().unwrap();
    assert_eq!(f.cmd, MSP_STATUS_EX);
    let s = decode_status(&f.payload).expect("status payload");
    assert_eq!(s.sensors, 0x27);
    assert_eq!(s.arming_disable_flags, 0x04);
    assert_eq!(s.rate_profile_count, 4);
}

#[test]
fn golden_status_v2() {
    let (d, f, n) = parse_reply(&golden("status_v2")).unwrap().unwrap();
    assert_eq!(d, b'X');
    assert_eq!(n, 36);
    assert_eq!(f.cmd, MSP_STATUS);
    assert_eq!(f.flags, 0);
    let s = decode_status(&f.payload).expect("status payload");
    assert_eq!(s.sensors, 0x27);
    assert_eq!(s.arming_disable_flags, 0x04);
}

#[test]
fn golden_attitude() {
    for name in ["attitude_v1", "attitude_v2"] {
        let (_, f, _) = parse_reply(&golden(name)).unwrap().unwrap();
        assert_eq!(f.cmd, MSP_ATTITUDE);
        let a = decode_attitude(&f.payload).expect("attitude payload");
        // Captured with the craft level and disarmed.
        assert_eq!((a.roll_decdeg, a.pitch_decdeg, a.yaw_deg), (0, 0, 0));
    }
}

#[test]
fn golden_motor() {
    for name in ["motor_v1", "motor_v2"] {
        let (_, f, _) = parse_reply(&golden(name)).unwrap().unwrap();
        assert_eq!(f.cmd, MSP_MOTOR);
        assert_eq!(decode_motor(&f.payload), vec![1000, 1000, 1000, 1000, 0, 0, 0, 0]);
    }
}

#[test]
fn golden_rc() {
    let (_, f, _) = parse_reply(&golden("rc_v1")).unwrap().unwrap();
    assert_eq!(f.cmd, MSP_RC);
    let r = decode_rc(&f.payload);
    // AETR + the live capture state: roll/pitch/yaw centred, throttle low,
    // AUX channels at their default positions.
    assert_eq!(&r[..4], &[1500, 1500, 1500, 885]);
    assert_eq!(r.len(), 16);
    let (_, f2, _) = parse_reply(&golden("raw_rc_v2")).unwrap().unwrap();
    assert_eq!(decode_rc(&f2.payload), r);
    // MSP_RC over v2 carries an empty payload on this build — useless,
    // kept as a fixture so that stays true.
    let (_, f3, _) = parse_reply(&golden("rc_v2")).unwrap().unwrap();
    assert_eq!(f3.cmd, 0x41);
    assert!(f3.payload.is_empty());
}

/// MSP2_GET_SETTING (0x1003) is refused by this SITL build with an $X!
/// error frame — the CLI is the only setting readback surface.
#[test]
fn golden_get_setting_rejected() {
    let err = parse_reply(&golden("get_setting_v2")).unwrap().unwrap_err();
    assert!(matches!(err, MspError::Rejected(0x1003)));
}

/// Bad checksums are rejected, not decoded.
#[test]
fn bad_checksum_rejected() {
    let mut f = golden("status_v1");
    let n = f.len();
    f[n - 2] ^= 0xFF; // corrupt the last payload byte
    let err = parse_reply(&f).unwrap().unwrap_err();
    assert!(matches!(err, MspError::BadReply(_)));
}

/// Frames split across reads decode byte-at-a-time (TCP gives no framing).
#[test]
fn incremental_feed() {
    let full = golden("status_v1");
    let mut buf = Vec::new();
    let mut got = None;
    for &b in &full {
        buf.push(b);
        if let Some(Ok((_, f, _))) = parse_reply(&buf) {
            got = Some(f);
            break;
        }
    }
    assert_eq!(got.expect("frame after full feed").cmd, MSP_STATUS);
}

/// Multiple frames in one buffer: parse, drain, parse again — the drain
/// length must carry us to the next frame exactly.
#[test]
fn back_to_back_frames() {
    let mut buf = golden("status_v1");
    buf.extend_from_slice(&golden("motor_v1"));
    buf.extend_from_slice(&golden("attitude_v1"));
    let seq = [
        (b'M', MSP_STATUS),
        (b'M', MSP_MOTOR),
        (b'M', MSP_ATTITUDE),
    ];
    for (d, cmd) in seq {
        let (dialect, f, n) = parse_reply(&buf).unwrap().unwrap();
        assert_eq!((dialect, f.cmd), (d, cmd));
        buf.drain(..n);
    }
    assert!(buf.is_empty(), "leftover bytes {buf:02x?}");
}

/// Garbage before a marker is skipped by the drain length.
#[test]
fn garbage_prefix_skipped() {
    let mut buf = vec![0xDE, 0xAD, 0x00, 0x24]; // 4 junk bytes, no marker
    buf.extend_from_slice(&golden("attitude_v1"));
    let (d, f, n) = parse_reply(&buf).unwrap().unwrap();
    assert_eq!((d, f.cmd), (b'M', MSP_ATTITUDE));
    assert_eq!(n, 4 + 12);
}

/// A partial frame (or a partial $X!) returns None, never an error: the
/// remaining bytes may arrive with the next read.
#[test]
fn partial_frame_is_none() {
    let full = golden("status_v2");
    for cut in [0usize, 3, 5, 7, full.len() - 1] {
        let got = parse_reply(&full[..cut]);
        if cut < 3 {
            assert!(got.is_none(), "cut {cut} parsed");
        }
        // A cut that exposes a complete header but a short payload is None.
        if cut >= 3 && cut < full.len() {
            assert!(got.is_none(), "cut {cut} parsed as complete");
        }
    }
    let err = golden("get_setting_v2");
    for cut in [0, 3, 8] {
        assert!(parse_reply(&err[..cut]).is_none(), "partial $X! at cut {cut}");
    }
}

/// decode_status on truncated payloads returns None instead of panicking.
#[test]
fn status_decoder_resists_truncation() {
    let raw = golden("status_v1")[5..32].to_vec();
    for cut in 0..16 {
        assert!(decode_status(&raw[..cut]).is_none(), "decoded {cut} bytes");
    }
    assert!(decode_status(&raw).is_some());
}

/// The exti flag bytes (byteCount > 0) shift the arming-flags tail; the
/// decoder must account for them. Synthesised from the real payload: the
/// SITL capture has byteCount = 0 (no extended flight-mode bits).
#[test]
fn status_decoder_handles_ext_bytes() {
    let raw = golden("status_v1")[5..32].to_vec();
    let mut with_ext = raw[..16].to_vec();
    with_ext[15] = 2; // byteCount: 2 exti bytes follow
    with_ext.extend_from_slice(&[0xAA, 0xBB]);
    with_ext.extend_from_slice(&raw[16..]);
    let s = decode_status(&with_ext).expect("decoded with ext bytes");
    assert_eq!(s.arming_disable_flags, 0x04);
    assert_eq!(s.sensors, 0x27);
}
/// The request checksum covers flags+cmd+size+payload (the same range the
/// reply parser verifies). Hashing the header alone produced valid-looking
/// frames that the SITL silently dropped for every non-empty payload —
/// STATUS (empty payload) worked while every CLI command timed out.
#[test]
fn request_v2_checksum_covers_payload() {
    // CLI request for "version" with offset 0, CRC precomputed against the
    // reference python client and the msp_serial.c covered range.
    let mut payload = b"version\x00".to_vec();
    payload.extend_from_slice(&0u16.to_le_bytes());
    let frame = build_v2_request(MSP2_CLI_COMMAND, &payload);
    assert_eq!(
        &frame[..8],
        &[b'$', b'X', b'<', 0x00, 0x12, 0x30, 0x0A, 0x00],
        "marker, flags, cmd LE, size LE"
    );
    assert_eq!(&frame[8..8 + payload.len()], &payload[..]);
    assert_eq!(frame[8 + payload.len()], 0x5D, "crc over hdr+payload");
    // Symmetry: the sent frame's checksum field equals what our own reply
    // parser would verify over the same byte range.
    let crc_field = frame.len() - 1;
    assert_eq!(frame[crc_field], crc8_dvb_s2(&frame[3..crc_field]));
    // And the empty-payload v1 STATUS request stays byte-stable.
    assert_eq!(request_v1_frame_bytes(MSP_STATUS), vec![b'$', b'M', b'<', 0, 101, 101]);
}

fn request_v1_frame_bytes(cmd: u16) -> Vec<u8> {
    let cmd = u8::try_from(cmd).unwrap();
    [b"$M<" as &[u8], &[0u8], &[cmd], &[crc8_xor(&[0, cmd])]].concat()
}

/// MSP 102 RAW_IMU: 3 acc i16, 3 gyro-dps i16, 3 mag i16 — the gyro triplet is
/// the filtered signal the PID loop sees.
#[test]
fn raw_imu_decoder_layout() {
    let mut p = Vec::new();
    for v in [-100i16, 2, 250, -950, 0, 31, -400, 1, 999] {
        p.extend_from_slice(&v.to_le_bytes());
    }
    let r = decode_raw_imu(&p).expect("decoded");
    assert_eq!(r.acc, [-100, 2, 250]);
    assert_eq!(r.gyro_raw, [-950, 0, 31]);
    assert_eq!(r.mag, [-400, 1, 999]);
    assert!(decode_raw_imu(&p[..17]).is_none(), "short payload rejected");
}

/// MSP 109 ALTITUDE: i32 LE relative altitude cm, i16 LE vario cm/s
/// (msp.c `case MSP_ALTITUDE`: getEstimatedAltitudeCm, getEstimatedVario).
#[test]
fn estimated_altitude_decoder_layout() {
    let mut p = Vec::new();
    p.extend_from_slice(&(-13_000i32).to_le_bytes());
    p.extend_from_slice(&(-412i16).to_le_bytes());
    let r = decode_estimated_altitude(&p).expect("decoded");
    assert_eq!(r.alt_cm, -13_000);
    assert_eq!(r.vario_cms, -412);
    assert!(decode_estimated_altitude(&p[..5]).is_none(), "short payload rejected");
    // Positive climb, vario zero (USE_VARIO off on real hardware is legal).
    let mut q = Vec::new();
    q.extend_from_slice(&40_000i32.to_le_bytes());
    q.extend_from_slice(&0i16.to_le_bytes());
    let r2 = decode_estimated_altitude(&q).expect("decoded");
    assert_eq!(r2.alt_cm, 40_000);
    assert_eq!(r2.vario_cms, 0);
}
