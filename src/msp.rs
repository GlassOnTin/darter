//! MSP client over the SITL's TCP serial bridge (UART1, tcp://127.0.0.1:5761).
//!
//! Two dialects share one framing: MSPv1 (`$M<` request / `$M>` reply, XOR
//! checksum over len+cmd+payload, 1-byte command) and MSPv2 (`$X<` / `$X>`,
//! CRC8-DVB-S2 over flags+cmd+size+payload, 2-byte command, flag byte).
//! The SITL answers both on the same port; unsupported MSPv2 commands get an
//! `$X!` error frame (same header, zero length). On top of these, MSPv2 CLI
//! command (0x3012) drives the Betaflight CLI programmatically — the config
//! surface for the harness (the MSP2_GET_SETTING data command is refused on
//! this port, golden fixture `get_setting_v2.bin`).
//!
//! Wire layouts verified against the running SITL and its source
//! (msp.c MSP_STATUS, msp_serial.c frame writer, msp_protocol_v2_betaflight.h
//! flag constants); fixtures in tests/data/msp_golden/ are captured replies,
//! not synthesised.

use std::io::{self, Read, Write};
use std::net::TcpStream;
use std::time::{Duration, Instant};

pub const MSP_STATUS: u16 = 101;
pub const MSP_MOTOR: u16 = 104;
pub const MSP_RC: u16 = 105;
pub const MSP_ATTITUDE: u16 = 108;
pub const MSP_STATUS_EX: u16 = 150;
pub const MSP2_CLI_COMMAND: u16 = 0x3012;

/// MSPv2 CLI reply flag: output exceeded the pageable buffer (msp_protocol_v2_betaflight.h).
pub const CLI_FLAG_TRUNCATED: u8 = 1 << 0;
/// MSPv2 CLI reply flag: command refused or paging session mismatch.
pub const CLI_FLAG_REFUSED: u8 = 1 << 1;

/// CRC8-DVB-S2 (poly 0xD5, init 0), used by MSPv2 frames.
pub fn crc8_dvb_s2(data: &[u8]) -> u8 {
    let mut crc: u8 = 0;
    for &b in data {
        crc ^= b;
        for _ in 0..8 {
            crc = if crc & 0x80 != 0 { (crc << 1) ^ 0xD5 } else { crc << 1 };
        }
    }
    crc
}

/// MSPv1 checksum: XOR of length, command, and payload bytes.
pub fn crc8_xor(data: &[u8]) -> u8 {
    data.iter().fold(0u8, |a, &b| a ^ b)
}

/// `$X<` request frame: marker, flags(0), cmd u16 LE, size u16 LE, payload,
/// CRC8-DVB-S2 over flags+cmd+size+payload (msp_serial.c request path, and
/// byte-identical to the reference python client). The checksum covers the
/// payload too — hashing the header alone silently drops every CLI command
/// with a non-empty payload (observed 2026-09-28).
pub fn build_v2_request(cmd: u16, payload: &[u8]) -> Vec<u8> {
    let mut frame = Vec::with_capacity(8 + payload.len());
    frame.extend_from_slice(b"$X<");
    frame.push(0); // flags: none set
    frame.extend_from_slice(&cmd.to_le_bytes());
    frame.extend_from_slice(&(payload.len() as u16).to_le_bytes());
    frame.extend_from_slice(payload);
    let crc = crc8_dvb_s2(&frame[3..]);
    frame.push(crc);
    frame
}

/// A parsed reply frame. `flags` is the MSPv2 flag byte (0 for MSPv1).
#[derive(Clone, Debug, PartialEq)]
pub struct Frame {
    pub cmd: u16,
    pub flags: u8,
    pub payload: Vec<u8>,
}

#[derive(Debug)]
pub enum MspError {
    Io(io::Error),
    /// `$X!` error frame (unsupported command, invalid argument, ...).
    Rejected(u16),
    /// CLI command refused via the in-band refused flag.
    CliRefused,
    /// Malformed or timed-out reply.
    BadReply(&'static str),
}

impl From<io::Error> for MspError {
    fn from(e: io::Error) -> Self {
        MspError::Io(e)
    }
}

impl std::fmt::Display for MspError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            MspError::Io(e) => write!(f, "io error: {e}"),
            MspError::Rejected(cmd) => write!(f, "MSP command {cmd:#x} rejected ($X! error frame)"),
            MspError::CliRefused => write!(f, "CLI command refused (in-band flag)"),
            MspError::BadReply(why) => write!(f, "bad MSP reply: {why}"),
        }
    }
}

pub struct MspLink {
    inner: TcpStream,
    buf: Vec<u8>,
}

impl MspLink {
    pub fn connect(addr: &str) -> io::Result<Self> {
        let inner = TcpStream::connect(addr)?;
        inner.set_read_timeout(Some(Duration::from_millis(1000)))?;
        Ok(Self { inner, buf: Vec::new() })
    }

    /// MSPv1 request: send `$M<` and wait for the matching `$M>` reply,
    /// skipping frames in between (other dialect or command).
    pub fn request_v1(&mut self, cmd: u16) -> Result<Vec<u8>, MspError> {
        let cmd = u8::try_from(cmd).map_err(|_| MspError::BadReply("v1 command out of u8 range"))?;
        let frame = [b"$M<" as &[u8], &[0u8], &[cmd], &[crc8_xor(&[0, cmd])]].concat();
        self.inner.write_all(&frame)?;
        self.recv_frame(b'M', u16::from(cmd), Duration::from_secs(2)).map(|f| f.payload)
    }

    /// MSPv2 request: send `$X<` and wait for the matching `$X>` reply.
    pub fn request_v2(&mut self, cmd: u16, payload: &[u8]) -> Result<Frame, MspError> {
        let frame = build_v2_request(cmd, payload);
        self.inner.write_all(&frame)?;
        self.recv_frame(b'X', cmd, Duration::from_secs(2))
    }

    /// Run one Betaflight CLI command over MSP2_CLI_COMMAND (0x3012),
    /// following pagination until the accumulated text reaches the reported
    /// total length. Returns the full text. A refused command (unknown
    /// command, or paging a session after a different command ran) is
    /// CliRefused.
    pub fn cli(&mut self, line: &str) -> Result<String, MspError> {
        let line = line.as_bytes();
        let mut text = String::new();
        let mut offset: u16 = 0;
        loop {
            let mut payload = line.to_vec();
            payload.push(0);
            payload.extend_from_slice(&offset.to_le_bytes());
            let frame = self.request_v2(MSP2_CLI_COMMAND, &payload)?;
            if frame.payload.len() < 3 {
                return Err(MspError::BadReply("cli reply shorter than 3 bytes"));
            }
            let total = u16::from_le_bytes([frame.payload[0], frame.payload[1]]) as usize;
            let pflags = frame.payload[2];
            if pflags & CLI_FLAG_REFUSED != 0 {
                return Err(MspError::CliRefused);
            }
            let window = frame.payload.len() - 3;
            if window == 0 {
                // No progress but the total is unmet: stop rather than spin.
                break;
            }
            text.push_str(&String::from_utf8_lossy(&frame.payload[3..]));
            offset = offset.saturating_add(window as u16);
            if text.len() >= total || offset == u16::MAX {
                break;
            }
        }
        Ok(text)
    }

    /// Read frames until one matching `dialect` and `cmd` arrives, skipping
    /// anything else. A malformed frame or `$X!` error aborts the wait.
    fn recv_frame(&mut self, dialect: u8, cmd: u16, timeout: Duration) -> Result<Frame, MspError> {
        let deadline = Instant::now() + timeout;
        loop {
            if let Some(parsed) = parse_reply(&self.buf) {
                match parsed {
                    Ok((d, frame, n)) => {
                        self.buf.drain(..n);
                        if d == dialect && frame.cmd == cmd {
                            return Ok(frame);
                        }
                    }
                    Err(e) => {
                        self.buf.clear();
                        return Err(e);
                    }
                }
            } else if self.buf.len() > 64 * 1024 {
                // No marker in a large buffer: the peer is not speaking MSP.
                self.buf.clear();
            }
            if Instant::now() > deadline {
                return Err(MspError::BadReply("timeout waiting for reply"));
            }
            let mut chunk = [0u8; 512];
            let n = match self.inner.read(&mut chunk) {
                Ok(0) => return Err(MspError::BadReply("connection closed")),
                Ok(n) => n,
                Err(e) if e.kind() == io::ErrorKind::WouldBlock || e.kind() == io::ErrorKind::TimedOut => {
                    continue
                }
                Err(e) => return Err(MspError::Io(e)),
            };
            self.buf.extend_from_slice(&chunk[..n]);
        }
    }
}

/// Parse the first complete reply frame in `buf`: `("M"/"X", frame, total
/// length to drain)`. Returns None when no marker is present or the frame at
/// the first marker is still incomplete — never skip past an incomplete
/// frame, it may complete with the next bytes. `$X!` decodes to
/// Err(Rejected(cmd)) once its 9 bytes are present.
///
/// The checksum covers length+cmd+payload (v1, XOR) or
/// flags+cmd+size+payload (v2, CRC8-DVB-S2) — both start at buf[i+3].
pub fn parse_reply(buf: &[u8]) -> Option<Result<(u8, Frame, usize), MspError>> {
    let i = buf.windows(3).position(|w| {
        (w[0] == b'$' && w[1] == b'M' && w[2] == b'>')
            || (w[0] == b'$' && w[1] == b'X' && (w[2] == b'>' || w[2] == b'!'))
    })?;
    let dialect = buf[i + 1];
    let marker = buf[i + 2];
    if marker == b'!' {
        // $X! + flags + cmd(2) + size(2) + payload(0) + crc = 9 bytes.
        if buf.len() < i + 9 {
            return None;
        }
        let cmd = u16::from_le_bytes([buf[i + 4], buf[i + 5]]);
        return Some(Err(MspError::Rejected(cmd)));
    }
    let (hdr_len, len, cmd, flags) = if dialect == b'M' {
        if buf.len() < i + 5 {
            return None;
        }
        (5usize, buf[i + 3] as usize, buf[i + 4] as u16, 0u8)
    } else {
        if buf.len() < i + 8 {
            return None;
        }
        (
            8usize,
            u16::from_le_bytes([buf[i + 6], buf[i + 7]]) as usize,
            u16::from_le_bytes([buf[i + 4], buf[i + 5]]),
            buf[i + 3],
        )
    };
    let total = i + hdr_len + len + 1;
    if buf.len() < total {
        return None;
    }
    let ok = if dialect == b'M' {
        crc8_xor(&buf[i + 3..total - 1]) == buf[total - 1]
    } else {
        crc8_dvb_s2(&buf[i + 3..total - 1]) == buf[total - 1]
    };
    if !ok {
        return Some(Err(MspError::BadReply("checksum mismatch")));
    }
    let payload = buf[i + hdr_len..i + hdr_len + len].to_vec();
    Some(Ok((dialect, Frame { cmd, flags, payload }, total)))
}

/// Attitude from MSP 108: euler angles in centidegrees.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Attitude {
    pub roll_cdeg: i16,
    pub pitch_cdeg: i16,
    pub yaw_cdeg: i16,
}

/// MSP 108 payload (3 x i16 LE).
pub fn decode_attitude(p: &[u8]) -> Option<Attitude> {
    if p.len() < 6 {
        return None;
    }
    Some(Attitude {
        roll_cdeg: i16::from_le_bytes([p[0], p[1]]),
        pitch_cdeg: i16::from_le_bytes([p[2], p[3]]),
        yaw_cdeg: i16::from_le_bytes([p[4], p[5]]),
    })
}

fn decode_u16_vec(p: &[u8], count: usize) -> Vec<u16> {
    p.chunks_exact(2).take(count).map(|c| u16::from_le_bytes([c[0], c[1]])).collect()
}

/// MSP 104 payload: up to 8 motor values (u16 LE).
pub fn decode_motor(p: &[u8]) -> Vec<u16> {
    decode_u16_vec(p, 8)
}

/// MSP 105 payload: up to 16 RC channel values (u16 LE).
pub fn decode_rc(p: &[u8]) -> Vec<u16> {
    decode_u16_vec(p, 16)
}

/// MSP 101/150 payload, laid out per msp.c `case MSP_STATUS`. The [13:15]
/// pair differs by command (gyro cycle for 101, profile indices for 150) and
/// is not exposed; everything after the byteCount header is shared.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Status {
    pub cycle_time_us: u16,
    pub i2c_errors: u16,
    /// Sensor presence bitfield: ACC 1, BARO 2, MAG 4, GPS 8, RF 16, GYRO 32,
    /// OPTFLOW 64, PITOT 128 (msp.c writes sensors(SENSOR_x) << n).
    pub sensors: u16,
    /// First 32 bits of the flight-mode box bitmask (0 = no boxes active).
    pub flight_flags: u32,
    pub pid_profile: u8,
    pub load_percent: u16,
    /// Arming-disable flag bitfield (bit 2 = RX_FAILSAFE on this build).
    pub arming_disable_flags: u32,
    pub rate_profile_count: u8,
    pub battery_profile_count: u8,
    pub battery_profile: u8,
}

pub fn decode_status(p: &[u8]) -> Option<Status> {
    if p.len() < 16 {
        return None;
    }
    let ext_count = p[15] as usize;
    // Fixed tail after the exti bytes: flag count (1) + flags (4) + reboot
    // (1) + cpu temp (2) = 9 bytes; the trailing profile counts are optional.
    if p.len() < 16 + ext_count + 9 {
        return None;
    }
    let mut o = 16 + ext_count;
    let _flag_count = p[o];
    o += 1;
    let arming_disable_flags = u32::from_le_bytes([p[o], p[o + 1], p[o + 2], p[o + 3]]);
    o += 5; // flags u32 + reboot u8
    let _cpu_temp = u16::from_le_bytes([p[o], p[o + 1]]);
    o += 2;
    Some(Status {
        cycle_time_us: u16::from_le_bytes([p[0], p[1]]),
        i2c_errors: u16::from_le_bytes([p[2], p[3]]),
        sensors: u16::from_le_bytes([p[4], p[5]]),
        flight_flags: u32::from_le_bytes([p[6], p[7], p[8], p[9]]),
        pid_profile: p[10],
        load_percent: u16::from_le_bytes([p[11], p[12]]),
        arming_disable_flags,
        rate_profile_count: p.get(o).copied().unwrap_or(0),
        battery_profile_count: p.get(o + 1).copied().unwrap_or(0),
        battery_profile: p.get(o + 2).copied().unwrap_or(0),
    })
}