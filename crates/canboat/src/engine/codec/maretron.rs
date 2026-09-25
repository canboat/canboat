// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Maretron IPG100/200 gateway codec.
//!
//! Wraps [`crate::engine::format::maretron_ipg`]. Unlike NGT-1 or
//! iKonvert, Maretron has a handshake driven by *reception*: after
//! [`Codec::open`] sends `CONNECT`, the codec waits for the device's
//! `CONNECTED\t<serial>\0` reply, answers it with `SET_MODE BINARY` as an
//! [`Event::Send`], and only then reads binary frames.

use crate::engine::RawFrame;
use crate::engine::format::maretron_ipg::{
    MaretronFrame, ParseOutcome, RX_CONNECTED, RX_DETAILED_LICENSES_USED, RX_INSTANCE_DATA,
    RX_LICENSES_USED, RX_NO, RX_SERVER_VERSION, SessionState, build_connect, build_frame,
    build_set_mode_binary, parse,
};

use super::{Codec, Event, Refused, SYNTHETIC_PGN_START, iso_ms};

/// Maretron session configuration.
#[derive(Debug, Clone, Default)]
pub struct Config {
    /// Login password. IPG units without security accept the empty
    /// string.
    pub password: String,
}

/// The Maretron IPG protocol; see the [module docs](self).
pub struct Maretron {
    acc: Vec<u8>,
    state: SessionState,
    password: String,
}

impl Maretron {
    /// A codec with [`Config`].
    pub fn new(config: Config) -> Self {
        Self {
            acc: Vec::with_capacity(4096),
            state: SessionState::AwaitHandshake,
            password: config.password,
        }
    }

    fn handle_text(&mut self, line: &str, events: &mut Vec<Event>) {
        let (head, rest) = line.split_once('\t').unwrap_or((line, ""));
        match head {
            RX_CONNECTED => {
                log::info!("maretron: connected (serial {rest})");
                events.push(Event::Send(build_set_mode_binary()));
                self.state = SessionState::Streaming;
            }
            RX_NO => {
                events.push(Event::Error(
                    "maretron: authentication rejected".to_string(),
                ));
            }
            RX_SERVER_VERSION => log::info!("maretron: server version {rest}"),
            RX_INSTANCE_DATA => log::info!("maretron: instance data {rest}"),
            RX_LICENSES_USED | RX_DETAILED_LICENSES_USED => log::debug!("{head}\t{rest}"),
            _ => log::debug!("maretron control: {head} {rest}"),
        }
    }
}

impl Codec for Maretron {
    fn open(&mut self) -> Vec<u8> {
        build_connect(&self.password)
    }

    fn receive(&mut self, bytes: &[u8], now_ms: u64, events: &mut Vec<Event>) {
        self.acc.extend_from_slice(bytes);
        loop {
            match parse(&self.acc, self.state) {
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
                    events.push(Event::Frame(to_raw(&frame, now_ms)));
                }
            }
        }
    }

    fn send(&mut self, frame: &RawFrame) -> Result<Vec<u8>, Refused> {
        if frame.pgn >= SYNTHETIC_PGN_START {
            log::debug!("maretron: skipping synthetic PGN {}", frame.pgn);
            return Err(Refused::Synthetic);
        }
        build_frame(frame.pgn, frame.prio, frame.dst, &frame.data).ok_or_else(|| {
            log::warn!(
                "maretron: cannot encode PGN {} (payload too large)",
                frame.pgn
            );
            Refused::TooLarge
        })
    }
}

fn to_raw(frame: &MaretronFrame, now_ms: u64) -> RawFrame {
    frame.to_raw(Some(iso_ms(now_ms)))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::engine::format::maretron_ipg::{F1_SYNC_BIT, FRAME_SYNC};

    /// Drive a decoder through the handshake so the session is in
    /// binary-streaming state, and return it ready for frame bytes.
    const NOW: u64 = 1_780_082_164_826;

    fn streaming_decoder() -> Maretron {
        let mut d = Maretron::new(Config::default());
        let mut events = Vec::new();
        d.receive(b"CONNECTED\t1234567\0", NOW, &mut events);
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
    /// codec answers with `SET_MODE BINARY` to write back.
    #[test]
    fn connected_reply_asks_the_writer_for_binary_mode() {
        let mut d = Maretron::new(Config::default());
        let mut events = Vec::new();
        d.receive(b"CONNECTED\t1234567\0", NOW, &mut events);
        assert_eq!(events.len(), 1);
        match &events[0] {
            Event::Send(b) => assert_eq!(b, &build_set_mode_binary()),
            other => panic!("expected SendBytes, got {other:?}"),
        }
        assert_eq!(d.state, SessionState::Streaming);
    }

    /// Until `CONNECTED` arrives the session stays in text mode, so a
    /// byte sequence that would be a binary frame is read as text.
    #[test]
    fn handshake_state_holds_until_connected() {
        let mut d = Maretron::new(Config::default());
        let mut events = Vec::new();
        d.receive(b"SERVER_VERSION\t2.1.4\0", NOW, &mut events);
        assert!(events.is_empty(), "version banner is informational");
        assert_eq!(d.state, SessionState::AwaitHandshake);
    }

    /// A rejected password surfaces as an error event rather than a
    /// silent stall on a session that will never stream.
    #[test]
    fn rejected_login_reports_an_error() {
        let mut d = Maretron::new(Config::default());
        let mut events = Vec::new();
        d.receive(b"NO\tbad password\0", NOW, &mut events);
        assert_eq!(events.len(), 1);
        match &events[0] {
            Event::Error(m) => assert!(m.contains("authentication")),
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
        d.receive(&binary_frame(&[1, 2, 3, 4, 5, 6, 7, 8]), NOW, &mut events);
        assert_eq!(events.len(), 1);
        match &events[0] {
            Event::Frame(f) => {
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
        d.receive(head, NOW, &mut events);
        assert!(events.is_empty(), "half a frame must not emit");
        d.receive(tail, NOW, &mut events);
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
        d.receive(&bytes, NOW, &mut events);
        let payloads: Vec<u8> = events
            .iter()
            .map(|e| match e {
                Event::Frame(f) => f.data[0],
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
        d.receive(&bytes, NOW, &mut events);
        assert_eq!(events.len(), 1, "only the frame survives: {events:?}");
    }

    /// Frames carry the caller's time.
    #[test]
    fn frames_are_stamped_with_the_callers_time() {
        let mut d = streaming_decoder();
        let mut events = Vec::new();
        d.receive(&binary_frame(&[7]), NOW, &mut events);
        match &events[0] {
            Event::Frame(f) => assert_eq!(f.timestamp.as_deref(), Some("2026-05-29T19:16:04.826")),
            other => panic!("expected Frame, got {other:?}"),
        }
    }

    /// `open` starts the session with CONNECT carrying the password.
    #[test]
    fn open_carries_the_password() {
        let mut e = Maretron::new(Config {
            password: "hunter2".to_string(),
        });
        let init = e.open();
        assert_eq!(init, build_connect("hunter2"));
        assert!(init.ends_with(&[0]), "CONNECT is NUL-terminated");
        assert!(String::from_utf8_lossy(&init).contains("\"hunter2\""));
    }

    /// Synthetic PGNs are canboat-internal and must never reach the bus.
    #[test]
    fn synthetic_pgns_are_not_encoded() {
        let mut e = Maretron::new(Config::default());
        let frame = RawFrame::new(None, 6, SYNTHETIC_PGN_START, 17, 255, [0u8; 8]);
        assert_eq!(e.send(&frame), Err(Refused::Synthetic));
    }

    /// A payload past the fast-packet ceiling can't be framed, so the
    /// codec declines rather than truncating it.
    #[test]
    fn oversized_payload_is_declined() {
        let mut e = Maretron::new(Config::default());
        let mut frame = RawFrame::new(None, 6, 126996, 17, 255, [0u8; 8]);
        frame.data = std::iter::repeat_n(0xabu8, 224).collect();
        assert_eq!(e.send(&frame), Err(Refused::TooLarge));
    }

    /// What the codec sends, it reads back — PGN, priority
    /// and payload survive the round trip. SA is the IPG's to assign,
    /// so it goes out as 0xFF.
    #[test]
    fn send_round_trips_through_receive() {
        let mut e = Maretron::new(Config::default());
        let original = RawFrame::new(
            None,
            3,
            127508,
            17,
            255,
            [0xff, 0x5e, 0x7d, 0x00, 0x00, 0xff, 0xff, 0xff],
        );
        let bytes = e.send(&original).expect("encodes");

        let mut d = streaming_decoder();
        let mut events = Vec::new();
        d.receive(&bytes, NOW, &mut events);
        assert_eq!(events.len(), 1);
        match &events[0] {
            Event::Frame(f) => {
                assert_eq!(f.pgn, original.pgn);
                assert_eq!(f.prio, original.prio);
                assert_eq!(f.data, original.data);
                assert_eq!(f.src, 0xFF, "the IPG substitutes its own source");
            }
            other => panic!("expected Frame, got {other:?}"),
        }
    }
}
