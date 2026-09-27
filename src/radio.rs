//! Radio input via the Linux joystick interface (`/dev/input/js*`).
//!
//! The reference radio is a RadioMaster Pocket running EdgeTX, which exposes a
//! USB HID gamepad over USB/IP. Through the js* interface EdgeTX's default
//! "USB Joystick" template maps:
//! - axes 0..3 to model channels 1..4 (A, E, T, R — matches our AETR
//!   rc_packet ordering), each -32767..+32767,
//! - axes 4..7 to model channels 5..8 (AUX1..AUX4) as switch axes,
//! - the remaining controls as buttons (unused for RC).
//!
//! Channel values are pulse widths in microseconds (1000..2000), the same
//! wire domain as `sitl::RcPacket`.
//!
//! Axis-to-stick identity is a hypothesis until the user moves the sticks and
//! the observed axis changes match (`--radio` prints them).

use std::fs::File;
use std::io;
use std::io::Read;
use std::os::fd::AsRawFd;
use std::os::unix::fs::OpenOptionsExt;
use std::time::{Duration, Instant};

/// Linux js event: (time, value, type, code).
const JS_EVENT_BUTTON: u8 = 0x01;
const JS_EVENT_AXIS: u8 = 0x02;
const JS_EVENT_INIT: u8 = 0x80;
const JSIOCGAXES: u32 = 0x8001_6a11;

/// Number of axes the radio exposes (Pocket EdgeTX: 8).
const MAX_AXES: usize = 16;

/// Live state of a joystick device: axis values (i16) and button states.
///
/// The device can vanish and reappear while we run (the usbip forward drops
/// and re-attaches on the phone), so `poll` reopens the path when reads fail
/// permanently. A freshly opened js fd delivers a JS_EVENT_INIT snapshot of
/// the current state, which `read_events` records like any other event.
pub struct Radio {
    path: String,
    dev: File,
    axes: [i16; MAX_AXES],
    buttons: [bool; 64],
    last_retry: Option<Instant>,
}

const RETRY_INTERVAL: Duration = Duration::from_millis(500);

impl Radio {
    /// Open a js device in nonblocking mode and read its initial state (the
    /// first read delivers JS_EVENT_INIT snapshots of every axis and button).
    pub fn open(path: &str) -> io::Result<Self> {
        let dev = std::fs::OpenOptions::new()
            .read(true)
            .custom_flags(libc::O_NONBLOCK)
            .open(path)?;
        let mut radio =
            Self { path: path.to_string(), dev, axes: [0; MAX_AXES], buttons: [false; 64], last_retry: None };
        // Drain the initial snapshot.
        while radio.read_events()? > 0 {}
        Ok(radio)
    }

    fn open_fd(path: &str) -> io::Result<File> {
        std::fs::OpenOptions::new().read(true).custom_flags(libc::O_NONBLOCK).open(path)
    }

    /// Drain pending events, updating axis/button state. Returns how many
    /// events were consumed. On a dead device (forward dropped), retries the
    /// open every `RETRY_INTERVAL`; a stale fd is never reported as an error
    /// to the caller because RC keeps flowing from the last known state.
    pub fn poll(&mut self) -> io::Result<usize> {
        match self.read_events() {
            Ok(n) => return Ok(n),
            Err(e) if e.kind() == io::ErrorKind::WouldBlock => return Ok(0),
            Err(_) => {} // device gone; reopen below
        }
        if self.last_retry.map(|t| t.elapsed() >= RETRY_INTERVAL).unwrap_or(true) {
            self.last_retry = Some(Instant::now());
            match Self::open_fd(&self.path) {
                Ok(dev) => {
                    self.dev = dev;
                    // Drain the fresh INIT snapshot.
                    while let Ok(n) = self.read_events() {
                        if n == 0 {
                            break;
                        }
                    }
                }
                Err(_) => return Ok(0),
            }
        }
        Ok(0)
    }

    fn read_events(&mut self) -> io::Result<usize> {
        let mut buf = [0u8; 8 * 64];
        let n = match self.dev.read(&mut buf) {
            Ok(n) => n,
            Err(e) if e.kind() == io::ErrorKind::WouldBlock => return Ok(0),
            Err(e) => return Err(e),
        };
        let mut count = 0;
        for chunk in buf[..n].chunks_exact(8) {
            count += 1;
            let value = i16::from_le_bytes([chunk[4], chunk[5]]);
            let typ = chunk[6];
            let code = chunk[7] as usize;
            match typ & !JS_EVENT_INIT {
                JS_EVENT_AXIS if code < MAX_AXES => self.axes[code] = value,
                JS_EVENT_BUTTON if code < 64 => self.buttons[code] = value != 0,
                _ => {}
            }
        }
        Ok(count)
    }

    pub fn axis(&self, i: usize) -> i16 {
        self.axes[i]
    }

    /// Map an axis value (-32767..+32767) to a pulse width (1000..2000 us).
    /// EdgeTX centers at 0 -> 1500.
    fn axis_to_us(value: i16) -> u16 {
        let norm = value.clamp(-32767, 32767) as f64 / 32767.0; // -1..1
        (1500.0 + norm * 500.0).round() as u16
    }

    /// Current channel pulse widths in AETR receiver order: roll, pitch,
    /// throttle, yaw, AUX1..AUX4 from axes 4..7.
    pub fn channels(&self) -> [u16; 16] {
        let mut channels = [1000u16; 16];
        for i in 0..8 {
            channels[i] = Self::axis_to_us(self.axes[i]);
        }
        channels
    }
}

/// Read the axis count from a js device (for the smoke test / diagnostics).
pub fn axis_count(path: &str) -> io::Result<u8> {
    let f = File::open(path)?;
    let mut buf = [0u8; 1];
    // SAFETY: JSIOCGAXES ioctl writes one byte into buf.
    let r = unsafe { libc::ioctl(f.as_raw_fd(), JSIOCGAXES as libc::c_ulong, &mut buf) };
    if r < 0 {
        return Err(io::Error::last_os_error());
    }
    Ok(buf[0])
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn axis_to_us_endpoints_and_center() {
        assert_eq!(Radio::axis_to_us(-32767), 1000);
        assert_eq!(Radio::axis_to_us(0), 1500);
        assert_eq!(Radio::axis_to_us(32767), 2000);
        // Saturated values clamp, not wrap.
        assert_eq!(Radio::axis_to_us(-32768), 1000);
        assert_eq!(Radio::axis_to_us(16383), 1750);
    }
}