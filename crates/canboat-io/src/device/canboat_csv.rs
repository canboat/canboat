// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! "Canboat Raw CSV" (PLAIN/FAST) codec — talks to another
//! `canboat-pipeline` instance over its bidirectional CSV TCP port
//! (default 2603).
//!
//! This driver lets one `canboat-pipeline` chain into another: the
//! upstream instance owns the physical N2K gateway (NGT-1 / iKonvert
//! / Maretron) and exposes its CSV R/W port; the downstream instance
//! consumes that port as if it were a device of its own. Symmetric
//! traffic — outbound `RawFrame`s get serialised back as canboat
//! PLAIN/FAST and travel the same socket.
//!
//! Wire format (both directions):
//!
//!   2026-05-29T19:16:04.826Z,2,127251,14,255,8,ff,5e,7d,00,00,ff,ff,ff
//!
//! On open the codec writes a `# format=FAST\r\n` header so the
//! receiver knows the stream is pre-coalesced (matches canboat's
//! convention; the canboat-pipeline CSV port emits the same header
//! on its end too).
//!
//! Comment lines and stray iKonvert sentences (`$PDGY,…`, `!PDGY,…`)
//! that might leak through some upstream are silently dropped on
//! the read side — they're not PLAIN/FAST.

use std::io::{Read, Write};

use canboat_core::RawFrame;
use canboat_core::format::{parse_plain, plain::ParseError, write_plain};

use super::{DeviceDecoder, DeviceEncoder, DeviceEvent, DeviceHandle};

/// Synthetic-PGN marker. Frames at or above this PGN are emitted by
/// internal canboat tooling and must never be sent upstream.
pub const CANBOAT_SYNTHETIC_PGN: u32 = 0x40000;

/// Start the canboat-CSV reader/writer threads.
pub fn run(reader: Box<dyn Read + Send>, writer: Box<dyn Write + Send>) -> DeviceHandle {
    super::run(Decoder::new(), Encoder, reader, writer)
}

/// Line-buffered decoder. Accumulates UTF-8 across reads, splits on
/// `\n`, and routes each line through `parse_plain`.
pub struct Decoder {
    acc: String,
}

impl Decoder {
    pub fn new() -> Self {
        Self {
            acc: String::with_capacity(1024),
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
        self.acc.push_str(&String::from_utf8_lossy(bytes));
        while let Some(eol) = self.acc.find('\n') {
            let line: String = self.acc.drain(..=eol).collect();
            let trimmed = line.trim_end_matches(['\r', '\n']);
            if trimmed.is_empty() || trimmed.starts_with('#') {
                // canboat comment line or empty — includes the
                // `# format=FAST` header the upstream emits.
                continue;
            }
            if trimmed.starts_with('$') || trimmed.starts_with('!') {
                // iKonvert-style sentences that some upstreams leak
                // (NavLink2 init, etc.). Not PLAIN/FAST; drop.
                log::debug!("canboat-csv: skipping non-PLAIN/FAST line: {trimmed}");
                continue;
            }
            match parse_plain(trimmed) {
                Ok(frame) => events.push(DeviceEvent::Frame(frame)),
                Err(ParseError::Empty) => {}
                Err(e) => events.push(DeviceEvent::Error(format!("canboat-csv: {e}"))),
            }
        }
    }
}

pub struct Encoder;

impl DeviceEncoder for Encoder {
    fn init_bytes(&self) -> Vec<u8> {
        // Declare our format so the receiver (or any analyzer in the
        // middle) skips reassembly. Matches what the standalone
        // canboat reader binaries emit on stdout.
        b"# format=FAST\r\n".to_vec()
    }

    fn encode_frame(&self, frame: &RawFrame) -> Option<Vec<u8>> {
        if frame.pgn >= CANBOAT_SYNTHETIC_PGN {
            log::debug!("canboat-csv: skipping synthetic PGN {}", frame.pgn);
            return None;
        }
        let mut line = String::with_capacity(128);
        if write_plain(&mut line, frame).is_err() {
            return None;
        }
        line.push_str("\r\n");
        Some(line.into_bytes())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn decoder_parses_one_line_per_event() {
        let mut d = Decoder::new();
        let mut events = Vec::new();
        d.decode(
            b"2026-05-29T19:16:04.826Z,2,127251,14,255,8,ff,5e,7d,00,00,ff,ff,ff\r\n",
            &mut events,
        );
        assert_eq!(events.len(), 1);
        match &events[0] {
            DeviceEvent::Frame(f) => {
                assert_eq!(f.pgn, 127251);
                assert_eq!(f.src, 14);
                assert_eq!(f.dst, 255);
                assert_eq!(f.data.len(), 8);
            }
            _ => panic!("expected Frame"),
        }
    }

    #[test]
    fn decoder_skips_comment_and_ikonvert_lines() {
        let mut d = Decoder::new();
        let mut events = Vec::new();
        d.decode(
            b"# format=FAST\n$PDGY,N2NET_OFFLINE\n!PDGY,65350,2,27,FF\n",
            &mut events,
        );
        assert!(
            events.is_empty(),
            "header / iKonvert lines must be silently dropped, got {events:?}"
        );
    }

    #[test]
    fn decoder_handles_split_across_reads() {
        let mut d = Decoder::new();
        let mut events = Vec::new();
        d.decode(
            b"2026-05-29T19:16:04.826Z,2,127251,14,255,8,ff,5e,7d,",
            &mut events,
        );
        assert!(events.is_empty(), "partial line should not emit yet");
        d.decode(b"00,00,ff,ff,ff\r\n", &mut events);
        assert_eq!(events.len(), 1);
    }

    #[test]
    fn encoder_round_trips_through_decoder() {
        let original = RawFrame::new(
            Some("2026-05-29T19:16:04.826Z".to_string()),
            2,
            127251,
            14,
            255,
            [0xff, 0x5e, 0x7d, 0x00, 0x00, 0xff, 0xff, 0xff],
        );
        let bytes = Encoder.encode_frame(&original).expect("encode");
        let mut d = Decoder::new();
        let mut events = Vec::new();
        d.decode(&bytes, &mut events);
        assert_eq!(events.len(), 1);
        match &events[0] {
            DeviceEvent::Frame(f) => {
                assert_eq!(f.pgn, original.pgn);
                assert_eq!(f.prio, original.prio);
                assert_eq!(f.src, original.src);
                assert_eq!(f.dst, original.dst);
                assert_eq!(f.data.as_slice(), original.data.as_slice());
                assert_eq!(f.timestamp.as_deref(), Some("2026-05-29T19:16:04.826Z"));
            }
            _ => panic!("expected Frame"),
        }
    }

    #[test]
    fn encoder_drops_synthetic_pgns() {
        let f = RawFrame::new(None, 2, CANBOAT_SYNTHETIC_PGN, 0, 255, []);
        assert!(Encoder.encode_frame(&f).is_none());
    }

    #[test]
    fn encoder_init_bytes_emits_fast_header() {
        let bytes = Encoder.init_bytes();
        assert_eq!(bytes, b"# format=FAST\r\n");
    }
}
