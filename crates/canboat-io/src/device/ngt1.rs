// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Actisense NGT-1 codec adapter for [`super::run`].
//!
//! Wraps [`canboat_core::format::ngt1`]: the device reader pushes
//! incoming bytes through `Ngt1Decoder`, the writer encodes
//! [`RawFrame`]s as `N2K_MSG_SEND` (0x94) frames, and a 20-second
//! keepalive resends the NGT-1 startup ping while the channel is
//! quiet. Synthetic PGNs (`>= 0x40000`) are skipped on the write
//! path, matching canboat's behaviour.

use std::io::{Read, Write};
use std::time::Duration;

use canboat_core::RawFrame;
use canboat_core::format::ikonvert::{NetworkStatus, build_network_status};
use canboat_core::format::{
    encode_n2k_send_frame, encode_startup_ping,
    ngt1::{Ngt1Decoder, NgtEvent},
};

use super::{DeviceDecoder, DeviceEncoder, DeviceEvent, DeviceHandle};

/// Re-ping the NGT-1 startup sequence every 20 s — matches the C
/// `actisense-serial` keepalive.
pub const KEEPALIVE_INTERVAL: Duration = Duration::from_secs(20);

/// Synthetic-PGN marker. Frames at or above this PGN are emitted by
/// internal canboat tooling and must never hit the bus.
pub const ACTISENSE_SYNTHETIC_PGN: u32 = 0x40000;

/// `Actisense: System status` — `ACTISENSE_BEM + 0xf2`. The NGT-1 sends
/// it about once a second, but only when P-codes are enabled for the
/// port, so its two fields may never arrive.
const ACTISENSE_SYSTEM_STATUS_PGN: u32 = ACTISENSE_SYNTHETIC_PGN + 0xf2;

/// How often to emit the synthetic gateway network status.
const NETWORK_STATUS_INTERVAL_MS: u64 = 5_000;

/// Start the NGT-1 reader/writer threads. See [`super::run`].
pub fn run(reader: Box<dyn Read + Send>, writer: Box<dyn Write + Send>) -> DeviceHandle {
    super::run(Decoder::new(), Encoder, reader, writer)
}

/// Decoder wrapper that adapts [`Ngt1Decoder`] to [`DeviceDecoder`].
pub struct Decoder {
    inner: Ngt1Decoder,
    /// State for the synthetic `NMEA 2000 gateway: network status`
    /// (PGN 262400) this driver emits alongside the real traffic, so a
    /// consumer sees the same per-gateway record whichever gateway is
    /// feeding it. canboat C's `actisense-serial` does the same.
    net: NetworkStatusState,
}

struct NetworkStatusState {
    start_ms: u64,
    next_ms: u64,
    /// Distinct source addresses seen, for the device count.
    seen: [bool; 256],
    /// Ch1 Rx Load and Error ID, cached from the last System Status.
    /// Stay `None` — and ride the canboat sentinel — until one arrives.
    load_pct: Option<u8>,
    errors: Option<u32>,
}

impl Decoder {
    pub fn new() -> Self {
        let now = now_ms();
        Self {
            inner: Ngt1Decoder::new(),
            net: NetworkStatusState {
                start_ms: now,
                next_ms: now + NETWORK_STATUS_INTERVAL_MS,
                seen: [false; 256],
                load_pct: None,
                errors: None,
            },
        }
    }
}

impl Default for Decoder {
    fn default() -> Self {
        Self::new()
    }
}

impl DeviceDecoder for Decoder {
    fn decode(&mut self, bytes: &[u8], events: &mut Vec<DeviceEvent>) {
        // Wall-clock fallback, for a gateway whose P-codes are off and
        // so never sends a System Status to trigger on. Driven by
        // arriving bytes rather than a timer thread, which on a live
        // bus is close enough — the C uses a real timer in its main
        // loop.
        if now_ms() >= self.net.next_ms {
            self.emit_network_status(events);
        }
        for ev in self.inner.push_bytes(bytes) {
            match ev {
                NgtEvent::Message(msg) => {
                    if let Some(frame) = msg.to_raw_frame() {
                        self.note_frame(frame, events);
                    }
                }
                NgtEvent::Error(e) => events.push(DeviceEvent::Error(e.to_string())),
                // EBL header records (timestamp etc.) only appear when
                // the decoder is in EBL mode; this generic NGT-1 device
                // decoder never enables that, so this arm is dead.
                NgtEvent::Header(_) => {}
            }
        }
    }
}

impl Decoder {
    /// Per-frame bookkeeping for the synthetic network status: note the
    /// source address, and harvest the two fields `Actisense: System
    /// status` carries. Pushes the frame itself, plus a fresh status
    /// record when that message is what arrived.
    fn note_frame(&mut self, frame: RawFrame, events: &mut Vec<DeviceEvent>) {
        self.net.seen[frame.src as usize] = true;
        // Frame data drops the subcommand byte, so the C's msg[8..11]
        // and msg[14] are data[7..10] and data[13] here.
        if frame.pgn == ACTISENSE_SYSTEM_STATUS_PGN {
            if frame.data.len() >= 11 {
                self.net.errors = Some(u32::from_le_bytes([
                    frame.data[7],
                    frame.data[8],
                    frame.data[9],
                    frame.data[10],
                ]));
            }
            if frame.data.len() >= 14 {
                self.net.load_pct = Some(frame.data[13]);
            }
            events.push(DeviceEvent::Frame(frame));
            // A System Status arrival is the authoritative trigger for
            // a fresh record, as it is in the C — which also makes this
            // work during a file replay, where the wall-clock fallback
            // below never fires.
            self.emit_network_status(events);
            return;
        }
        events.push(DeviceEvent::Frame(frame));
    }

    /// Emit the synthetic PGN 262400 record and re-arm the timer.
    fn emit_network_status(&mut self, events: &mut Vec<DeviceEvent>) {
        let now = now_ms();
        let device_count = self.net.seen.iter().filter(|s| **s).count().min(255) as u8;
        let frame = build_network_status(
            NetworkStatus {
                load_pct: self.net.load_pct,
                errors: self.net.errors,
                device_count: Some(device_count),
                uptime_s: Some(now.saturating_sub(self.net.start_ms).div_euclid(1000) as u32),
                // The NGT-1 reports neither its own address nor any
                // rejected-TX count, so both stay at the canboat
                // "no data" sentinel.
                gateway_addr: 0xff,
                rejected_tx: None,
            },
            Some(now_iso_ms()),
        );
        events.push(DeviceEvent::Frame(frame));
        self.net.next_ms = now + NETWORK_STATUS_INTERVAL_MS;
    }
}

/// ISO-8601 UTC with milliseconds — the timestamp form every canboat
/// text format uses. Same shape as the ikonvert and maretron drivers'.
fn now_iso_ms() -> String {
    let ms_total = now_ms();
    let secs = (ms_total / 1000) as i64;
    let ms = (ms_total % 1000) as u32;
    let days = secs.div_euclid(86_400);
    let day_secs = secs.rem_euclid(86_400) as u32;
    let (h, m, s) = (day_secs / 3600, (day_secs / 60) % 60, day_secs % 60);
    let (y, mo, d) = canboat_core::format::days_to_ymd(days);
    format!("{y:04}-{mo:02}-{d:02}T{h:02}:{m:02}:{s:02}.{ms:03}")
}

fn now_ms() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_millis() as u64)
        .unwrap_or(0)
}

/// Stateless encoder.
pub struct Encoder;

impl DeviceEncoder for Encoder {
    fn init_bytes(&self) -> Vec<u8> {
        encode_startup_ping()
    }

    fn keepalive(&self) -> Option<(Duration, Vec<u8>)> {
        Some((KEEPALIVE_INTERVAL, encode_startup_ping()))
    }

    fn encode_frame(&self, frame: &RawFrame) -> Option<Vec<u8>> {
        if frame.pgn >= ACTISENSE_SYNTHETIC_PGN {
            log::debug!("ngt1: skipping synthetic PGN {}", frame.pgn);
            return None;
        }
        Some(encode_n2k_send_frame(frame))
    }
}

#[cfg(test)]
mod network_status_tests {
    use super::*;

    /// The NGT-1 driver must produce the same synthetic PGN 262400
    /// record the iKonvert and SocketCAN drivers do — canboat C's
    /// `actisense-serial` emits it, and a consumer should see one
    /// uniform per-gateway status whichever gateway is feeding it.
    #[test]
    fn system_status_yields_a_network_status_record() {
        let mut d = Decoder::new();
        let mut events = Vec::new();

        // `Actisense: System status` (ACTISENSE_BEM + 0xf2) as the
        // decoder hands it over: the subcommand byte is stripped, so
        // Error ID sits at data[7..10] and Ch1 Rx Load at data[13].
        let mut data = vec![0u8; 16];
        data[7..11].copy_from_slice(&1234u32.to_le_bytes());
        data[13] = 42; // 42 % bus load
        d.net.seen[3] = true;
        d.net.seen[9] = true;
        d.note_frame(
            RawFrame {
                timestamp: None,
                prio: 0,
                pgn: ACTISENSE_SYSTEM_STATUS_PGN,
                src: 0,
                dst: 0,
                data: data.into(),
            },
            &mut events,
        );

        let status = events
            .iter()
            .filter_map(|e| match e {
                DeviceEvent::Frame(f) if f.pgn == 0x40100 => Some(f),
                _ => None,
            })
            .next()
            .expect("a network status frame");
        assert_eq!(status.data.len(), 15);
        assert_eq!(status.data[0], 42, "Ch1 Rx Load");
        assert_eq!(
            u32::from_le_bytes([
                status.data[1],
                status.data[2],
                status.data[3],
                status.data[4]
            ]),
            1234,
            "Error ID"
        );
        // src 0, 3 and 9 seen by now.
        assert_eq!(status.data[5], 3, "device count");
        // The NGT-1 knows neither of these.
        assert_eq!(status.data[10], 0xff, "gateway address sentinel");
        assert_eq!(&status.data[11..15], &[0xff; 4], "rejected TX sentinel");
    }

    /// With no System Status ever seen — P-codes off — the two fields
    /// it carries must ride the sentinel rather than read as zero.
    #[test]
    fn without_system_status_load_and_errors_are_sentinel() {
        let mut d = Decoder::new();
        let mut events = Vec::new();
        d.emit_network_status(&mut events);
        let DeviceEvent::Frame(f) = &events[0] else {
            panic!("expected a frame")
        };
        assert_eq!(f.data[0], 0xff, "load sentinel");
        assert_eq!(&f.data[1..5], &[0xff; 4], "errors sentinel");
    }
}
