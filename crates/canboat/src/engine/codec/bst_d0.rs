// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Actisense BST-D0 messages over TCP: a W2K-1 or PRO-NDC-1E2K in its
//! Actisense data mode (#993). See [`crate::engine::format::bst_d0`] for the
//! message layout.
//!
//! The link needs no handshake. A message carries the device's own clock,
//! so each frame is stamped with the time it is received, as the NGT-1
//! codec does.

use crate::engine::RawFrame;
use crate::engine::format::bst_d0::{BstD0Decoder, encode_frame};
use crate::engine::format_iso_ms;

use super::{Codec, Event, Refused, SYNTHETIC_PGN_START};

/// The W2K-1's Actisense mode. See the [module docs](self).
#[derive(Debug, Default)]
pub struct BstD0 {
    decoder: BstD0Decoder,
}

impl BstD0 {
    pub fn new() -> Self {
        Self::default()
    }
}

impl Codec for BstD0 {
    fn receive(&mut self, bytes: &[u8], now_ms: u64, events: &mut Vec<Event>) {
        let skipped = self.decoder.skipped;
        for mut frame in self.decoder.push_bytes(bytes) {
            frame.timestamp = Some(format_iso_ms(now_ms));
            events.push(Event::Frame(frame));
        }
        let skipped = self.decoder.skipped - skipped;
        if skipped > 0 {
            events.push(Event::Error(format!(
                "skipped {skipped} bytes that are not a BST-D0 message"
            )));
        }
    }

    fn send(&mut self, frame: &RawFrame) -> Result<Vec<u8>, Refused> {
        if frame.pgn >= SYNTHETIC_PGN_START {
            return Err(Refused::Synthetic);
        }
        encode_frame(frame).ok_or(Refused::TooLarge)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn frame(pgn: u32, dst: u8, data: &[u8]) -> RawFrame {
        RawFrame::new(None, 2, pgn, 3, dst, data.iter().copied())
    }

    #[test]
    fn frames_are_stamped_with_the_time_received() {
        let sent = frame(129025, 255, &[1, 2, 3, 4, 5, 6, 7, 8]);
        let mut codec = BstD0::new();
        let bytes = codec.send(&sent).unwrap();
        let mut events = Vec::new();
        codec.receive(&bytes, 1_759_536_000_000, &mut events);
        let [Event::Frame(got)] = events.as_slice() else {
            panic!("{events:?}");
        };
        assert_eq!(got.timestamp.as_deref(), Some("2025-10-04T00:00:00.000Z"));
        assert_eq!(
            (got.pgn, got.src, got.dst, got.data.as_slice()),
            (129025, 3, 255, &sent.data[..])
        );
    }

    #[test]
    fn skipped_bytes_are_reported() {
        let mut events = Vec::new();
        BstD0::new().receive(&[0x55; 7], 0, &mut events);
        assert!(matches!(events.as_slice(), [Event::Error(e)] if e.contains("skipped 7 bytes")));
    }

    #[test]
    fn a_synthetic_pgn_is_not_sent() {
        assert_eq!(
            BstD0::new().send(&frame(0x40000, 255, &[0])),
            Err(Refused::Synthetic)
        );
    }
}
