// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Maretron IPG100/200 codec adapter for [`super::run`].
//!
//! Wraps [`canboat_core::format::maretron_ipg`]. Unlike NGT-1 or
//! iKonvert, Maretron has a handshake driven by *reception*: after
//! the writer sends `CONNECT`, the runner waits for the device's
//! `CONNECTED\t<serial>\0` reply before flipping to binary. The
//! decoder emits the post-`CONNECTED` `SET_MODE BINARY` bytes as a
//! [`DeviceEvent::SendBytes`] so the writer thread picks it up
//! without needing a sideband state-sync.

use std::io::{Read, Write};
use std::time::{SystemTime, UNIX_EPOCH};

use canboat_core::RawFrame;
use canboat_core::format::days_to_ymd;
use canboat_core::format::maretron_ipg::{
    MaretronFrame, ParseOutcome, RX_CONNECTED, RX_DETAILED_LICENSES_USED, RX_INSTANCE_DATA,
    RX_LICENSES_USED, RX_NO, RX_SERVER_VERSION, SessionState, build_connect, build_frame,
    build_set_mode_binary, parse,
};

use super::{DeviceDecoder, DeviceEncoder, DeviceEvent, DeviceHandle};

/// Synthetic-PGN marker.
pub const MARETRON_SYNTHETIC_PGN: u32 = 0x40000;

/// Maretron session configuration.
#[derive(Debug, Clone, Default)]
pub struct Config {
    /// Login password. IPG units without security accept the empty
    /// string.
    pub password: String,
    /// Optional fixed timestamp to stamp on every emitted frame —
    /// matches canboat C's `-fixtime` (deterministic-output tests).
    /// When `None`, the host clock is used.
    pub fixtime: Option<String>,
}

/// Start the Maretron reader/writer threads.
pub fn run(
    reader: Box<dyn Read + Send>,
    writer: Box<dyn Write + Send>,
    config: Config,
) -> DeviceHandle {
    let password = config.password.clone();
    let decoder = Decoder::new(config.fixtime);
    super::run(decoder, Encoder { password }, reader, writer)
}

pub struct Decoder {
    acc: Vec<u8>,
    state: SessionState,
    fixtime: Option<String>,
}

impl Decoder {
    pub fn new(fixtime: Option<String>) -> Self {
        Self {
            acc: Vec::with_capacity(4096),
            state: SessionState::AwaitHandshake,
            fixtime,
        }
    }
}

impl DeviceDecoder for Decoder {
    fn decode(&mut self, bytes: &[u8], events: &mut Vec<DeviceEvent>) {
        self.acc.extend_from_slice(bytes);
        loop {
            let outcome = parse(&self.acc, self.state);
            match outcome {
                ParseOutcome::NeedMore => return,
                ParseOutcome::Drop { consumed } => {
                    self.acc.drain(..consumed);
                }
                ParseOutcome::Text { line, consumed } => {
                    self.acc.drain(..consumed);
                    self.handle_text(&line, events);
                }
                ParseOutcome::Frame { frame, consumed } => {
                    self.acc.drain(..consumed);
                    events.push(DeviceEvent::Frame(self.to_raw(&frame)));
                }
            }
        }
    }
}

impl Decoder {
    fn handle_text(&mut self, line: &str, events: &mut Vec<DeviceEvent>) {
        let (head, rest) = line.split_once('\t').unwrap_or((line, ""));
        match head {
            RX_CONNECTED => {
                log::info!("maretron: connected (serial {rest})");
                events.push(DeviceEvent::SendBytes(build_set_mode_binary()));
                self.state = SessionState::Streaming;
            }
            RX_NO => {
                events.push(DeviceEvent::Error(
                    "maretron: authentication rejected".to_string(),
                ));
            }
            RX_SERVER_VERSION => log::info!("maretron: server version {rest}"),
            RX_INSTANCE_DATA => log::info!("maretron: instance data {rest}"),
            RX_LICENSES_USED | RX_DETAILED_LICENSES_USED => log::debug!("{head}\t{rest}"),
            _ => log::debug!("maretron control: {head} {rest}"),
        }
    }

    fn to_raw(&self, frame: &MaretronFrame) -> RawFrame {
        let ts = self.fixtime.clone().unwrap_or_else(now_iso_ms);
        frame.to_raw(Some(ts))
    }
}

pub struct Encoder {
    password: String,
}

impl DeviceEncoder for Encoder {
    fn init_bytes(&self) -> Vec<u8> {
        build_connect(&self.password)
    }

    fn encode_frame(&self, frame: &RawFrame) -> Option<Vec<u8>> {
        if frame.pgn >= MARETRON_SYNTHETIC_PGN {
            log::debug!("maretron: skipping synthetic PGN {}", frame.pgn);
            return None;
        }
        match build_frame(frame.pgn, frame.prio, frame.dst, &frame.data) {
            Some(b) => Some(b),
            None => {
                log::warn!(
                    "maretron: cannot encode PGN {} (payload too large)",
                    frame.pgn
                );
                None
            }
        }
    }
}

/// `YYYY-MM-DDTHH:MM:SS.mmm` from the host clock in UTC. Lifted from
/// the old `maretron-ipg` binary so the device crate doesn't pull in
/// `chrono`.
fn now_iso_ms() -> String {
    let dur = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default();
    let secs = dur.as_secs() as i64;
    let ms = dur.subsec_millis();
    let days = secs.div_euclid(86_400);
    let day_secs = secs.rem_euclid(86_400) as u32;
    let h = day_secs / 3600;
    let m = (day_secs / 60) % 60;
    let s = day_secs % 60;
    let (y, mo, d) = days_to_ymd(days);
    format!("{y:04}-{mo:02}-{d:02}T{h:02}:{m:02}:{s:02}.{ms:03}")
}

#[cfg(test)]
mod tests {
    use super::*;
    use canboat_core::format::maretron_ipg::{F1_SYNC_BIT, FRAME_SYNC};

    /// Drive a decoder through the handshake so the session is in
    /// binary-streaming state, and return it ready for frame bytes.
    fn streaming_decoder() -> Decoder {
        let mut d = Decoder::new(Some("2026-05-29T19:16:04.826Z".to_string()));
        let mut events = Vec::new();
        d.decode(b"CONNECTED\t1234567\0", &mut events);
        d
    }

    /// One `A5` binary frame: prio 6, msg_type 1, PF 0xF1, PS 0x01
    /// (PDU2, so PS folds in: PGN 0xF101 = 61697), SA 0x17.
    fn binary_frame(payload: &[u8]) -> Vec<u8> {
        let mut v = vec![
            FRAME_SYNC,
            F1_SYNC_BIT | (6 << 4) | (1 << 1),
            0xF1,
            0x01,
            0x17,
            payload.len() as u8,
        ];
        v.extend_from_slice(payload);
        v
    }

    /// `CONNECTED` is the reception-driven half of the handshake: the
    /// decoder answers with `SET_MODE BINARY` for the writer thread.
    #[test]
    fn connected_reply_asks_the_writer_for_binary_mode() {
        let mut d = Decoder::new(None);
        let mut events = Vec::new();
        d.decode(b"CONNECTED\t1234567\0", &mut events);
        assert_eq!(events.len(), 1);
        match &events[0] {
            DeviceEvent::SendBytes(b) => assert_eq!(b, &build_set_mode_binary()),
            other => panic!("expected SendBytes, got {other:?}"),
        }
        assert_eq!(d.state, SessionState::Streaming);
    }

    /// Until `CONNECTED` arrives the session stays in text mode, so a
    /// byte sequence that would be a binary frame is read as text.
    #[test]
    fn handshake_state_holds_until_connected() {
        let mut d = Decoder::new(None);
        let mut events = Vec::new();
        d.decode(b"SERVER_VERSION\t2.1.4\0", &mut events);
        assert!(events.is_empty(), "version banner is informational");
        assert_eq!(d.state, SessionState::AwaitHandshake);
    }

    /// A rejected password surfaces as an error event rather than a
    /// silent stall on a session that will never stream.
    #[test]
    fn rejected_login_reports_an_error() {
        let mut d = Decoder::new(None);
        let mut events = Vec::new();
        d.decode(b"NO\tbad password\0", &mut events);
        assert_eq!(events.len(), 1);
        match &events[0] {
            DeviceEvent::Error(m) => assert!(m.contains("authentication")),
            other => panic!("expected Error, got {other:?}"),
        }
        assert_eq!(
            d.state,
            SessionState::AwaitHandshake,
            "a rejected login must not flip to binary"
        );
    }

    /// After the handshake, `A5` frames decode to `RawFrame`s. PF
    /// >= 0xF0 is PDU2, so PS folds into the PGN and dst is global.
    #[test]
    fn binary_frame_decodes_as_pdu2_broadcast() {
        let mut d = streaming_decoder();
        let mut events = Vec::new();
        d.decode(&binary_frame(&[1, 2, 3, 4, 5, 6, 7, 8]), &mut events);
        assert_eq!(events.len(), 1);
        match &events[0] {
            DeviceEvent::Frame(f) => {
                assert_eq!(f.pgn, 0xF101);
                assert_eq!(f.prio, 6);
                assert_eq!(f.src, 0x17);
                assert_eq!(f.dst, 0xFF);
                assert_eq!(&f.data[..], &[1, 2, 3, 4, 5, 6, 7, 8]);
            }
            other => panic!("expected Frame, got {other:?}"),
        }
    }

    /// The accumulator holds a partial frame across reads instead of
    /// emitting a truncated one.
    #[test]
    fn partial_frame_waits_for_the_rest() {
        let mut d = streaming_decoder();
        let bytes = binary_frame(&[0xaa; 8]);
        let (head, tail) = bytes.split_at(4);
        let mut events = Vec::new();
        d.decode(head, &mut events);
        assert!(events.is_empty(), "half a frame must not emit");
        d.decode(tail, &mut events);
        assert_eq!(events.len(), 1);
    }

    /// Several frames arriving in one read all come out, in order.
    #[test]
    fn back_to_back_frames_all_emit() {
        let mut d = streaming_decoder();
        let mut bytes = binary_frame(&[1]);
        bytes.extend(binary_frame(&[2]));
        bytes.extend(binary_frame(&[3]));
        let mut events = Vec::new();
        d.decode(&bytes, &mut events);
        let payloads: Vec<u8> = events
            .iter()
            .map(|e| match e {
                DeviceEvent::Frame(f) => f.data[0],
                other => panic!("expected Frame, got {other:?}"),
            })
            .collect();
        assert_eq!(payloads, vec![1, 2, 3]);
    }

    /// IPG video and ASCII records share the stream; they are dropped
    /// without disturbing the frames around them.
    #[test]
    fn video_records_are_dropped_between_frames() {
        let mut d = streaming_decoder();
        let mut bytes = b"3some video record\0".to_vec();
        bytes.extend(binary_frame(&[9]));
        let mut events = Vec::new();
        d.decode(&bytes, &mut events);
        assert_eq!(events.len(), 1, "only the frame survives: {events:?}");
    }

    /// `fixtime` replaces the host clock so output is reproducible.
    #[test]
    fn fixtime_stamps_every_frame() {
        let mut d = streaming_decoder();
        let mut events = Vec::new();
        d.decode(&binary_frame(&[7]), &mut events);
        match &events[0] {
            DeviceEvent::Frame(f) => {
                assert_eq!(f.timestamp.as_deref(), Some("2026-05-29T19:16:04.826Z"))
            }
            other => panic!("expected Frame, got {other:?}"),
        }
    }

    /// Without `fixtime` the host clock supplies an ISO-8601 stamp.
    #[test]
    fn host_clock_stamp_is_iso_8601() {
        let ts = now_iso_ms();
        assert_eq!(ts.len(), 23, "YYYY-MM-DDTHH:MM:SS.mmm, got {ts}");
        assert_eq!(&ts[4..5], "-");
        assert_eq!(&ts[10..11], "T");
        assert_eq!(&ts[19..20], ".");
        assert!(ts[..4].parse::<u32>().unwrap() >= 2026);
    }

    /// The writer opens the session with CONNECT carrying the password.
    #[test]
    fn init_bytes_carry_the_password() {
        let e = Encoder {
            password: "hunter2".to_string(),
        };
        let init = e.init_bytes();
        assert_eq!(init, build_connect("hunter2"));
        assert!(init.ends_with(&[0]), "CONNECT is NUL-terminated");
        assert!(String::from_utf8_lossy(&init).contains("\"hunter2\""));
    }

    /// Synthetic PGNs are canboat-internal and must never reach the bus.
    #[test]
    fn synthetic_pgns_are_not_encoded() {
        let e = Encoder {
            password: String::new(),
        };
        let frame = RawFrame::new(None, 6, MARETRON_SYNTHETIC_PGN, 17, 255, [0u8; 8]);
        assert!(e.encode_frame(&frame).is_none());
    }

    /// A payload past the fast-packet ceiling can't be framed, so the
    /// encoder declines rather than truncating it.
    #[test]
    fn oversized_payload_is_declined() {
        let e = Encoder {
            password: String::new(),
        };
        let mut frame = RawFrame::new(None, 6, 126996, 17, 255, [0u8; 8]);
        frame.data = std::iter::repeat_n(0xabu8, 224).collect();
        assert!(e.encode_frame(&frame).is_none());
    }

    /// What the encoder writes, the decoder reads back — PGN, priority
    /// and payload survive the round trip. SA is the IPG's to assign,
    /// so it goes out as 0xFF.
    #[test]
    fn encoder_round_trips_through_decoder() {
        let e = Encoder {
            password: String::new(),
        };
        let original = RawFrame::new(
            None,
            3,
            127508,
            17,
            255,
            [0xff, 0x5e, 0x7d, 0x00, 0x00, 0xff, 0xff, 0xff],
        );
        let bytes = e.encode_frame(&original).expect("encodes");

        let mut d = streaming_decoder();
        let mut events = Vec::new();
        d.decode(&bytes, &mut events);
        assert_eq!(events.len(), 1);
        match &events[0] {
            DeviceEvent::Frame(f) => {
                assert_eq!(f.pgn, original.pgn);
                assert_eq!(f.prio, original.prio);
                assert_eq!(f.data, original.data);
                assert_eq!(f.src, 0xFF, "the IPG substitutes its own source");
            }
            other => panic!("expected Frame, got {other:?}"),
        }
    }
}
