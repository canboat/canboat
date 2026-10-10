// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Actisense BST-95 raw CAN frames: what a PRO-NDC-1E2K or W2K-1 sends in
//! its "CAN Actisense" mode, and an NGX in its CAN Packet operating mode.
//! See [`crate::engine::format::bst`] for the message layout.
//!
//! Every message is one CAN frame, so the codec does what the bus does not:
//! it joins fast-packets on the way in, and splits a message into
//! fast-packet frames on the way out. Each frame carries the device's 16-bit
//! clock, so frames are stamped with the time they are received. A frame on
//! its way to the bus — the gateway echoing one we sent — is skipped, as the
//! message it belongs to was handed to the gateway already.

use std::collections::HashMap;

use crate::engine::format::bst::{BdtpDecoder, bst95_message, encode_bst95};
use crate::engine::reassembly::{Reassembled, Reassembler};
use crate::engine::{BusProtocol, FramePacketType, RawFrame, fastpacket, format_iso_ms};

use super::{Codec, Event, Refused, SYNTHETIC_PGN_START};

/// BST-95, raw CAN frames. See the [module docs](self).
pub struct Bst95 {
    bdtp: BdtpDecoder,
    /// Decides which frames are fast-packets.
    protocol: BusProtocol,
    reassembler: Reassembler,
    /// Per-(pgn, src) fast-packet sequence counters, mod 8.
    seq: HashMap<(u32, u8), u8>,
}

impl Default for Bst95 {
    fn default() -> Self {
        Self::new(BusProtocol::Nmea2000)
    }
}

impl Bst95 {
    /// A codec for a bus that carries `protocol`.
    pub fn new(protocol: BusProtocol) -> Self {
        Self {
            bdtp: BdtpDecoder::default(),
            protocol,
            reassembler: Reassembler::new(),
            seq: HashMap::new(),
        }
    }

    fn next_seq(&mut self, pgn: u32, src: u8) -> u8 {
        let e = self.seq.entry((pgn, src)).or_insert(0);
        let s = *e;
        *e = (*e + 1) & 0x07;
        s
    }
}

impl Codec for Bst95 {
    fn receive(&mut self, bytes: &[u8], now_ms: u64, events: &mut Vec<Event>) {
        let dropped = self.bdtp.dropped;
        for &b in bytes {
            let Some(m) = self.bdtp.push_byte(b) else {
                continue;
            };
            // Another BST message (a BEM reply, say) is not a CAN frame.
            let Some((mut frame, to_bus)) = bst95_message(&m) else {
                continue;
            };
            if to_bus {
                continue;
            }
            frame.timestamp = Some(format_iso_ms(now_ms));
            let pt = self.protocol.packet_type(frame.pgn);
            match self.reassembler.push(frame, pt) {
                Reassembled::PassThrough(f) | Reassembled::Complete(f) => {
                    events.push(Event::Frame(f))
                }
                Reassembled::Partial => {}
                Reassembled::Error(e) => events.push(Event::Error(e.to_string())),
            }
        }
        let dropped = self.bdtp.dropped - dropped;
        if dropped > 0 {
            events.push(Event::Error(format!(
                "dropped {dropped} damaged BDTP frame(s)"
            )));
        }
    }

    fn send(&mut self, frame: &RawFrame) -> Result<Vec<u8>, Refused> {
        if frame.pgn >= SYNTHETIC_PGN_START {
            return Err(Refused::Synthetic);
        }
        let mut out = Vec::new();
        let mut put =
            |data: &[u8]| encode_bst95(frame.prio, frame.pgn, frame.src, frame.dst, data, &mut out);
        if self.protocol.packet_type(frame.pgn) == FramePacketType::Fast {
            let seq = self.next_seq(frame.pgn, frame.src);
            let chunks = fastpacket::fragment(seq, &frame.data).ok_or(Refused::TooLarge)?;
            for chunk in &chunks {
                put(chunk);
            }
        } else if frame.data.len() <= 8 {
            put(&frame.data);
        } else {
            // One BST-95 message is one CAN frame; a longer single-frame
            // PGN would need ISO transport, which this codec does not do.
            return Err(Refused::TooLarge);
        }
        Ok(out)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::engine::format::bst::encode_bst95_message;

    const NOW: u64 = 1_780_082_164_826;

    /// What a gateway sends for the frames the codec sent, had they come
    /// from the bus: the same messages, direction cleared.
    fn as_received(wire: &[u8]) -> Vec<u8> {
        let mut d = BdtpDecoder::default();
        let mut out = Vec::new();
        for m in wire.iter().filter_map(|&b| d.push_byte(b)) {
            let (f, _) = bst95_message(&m).unwrap();
            encode_bst95_message(f.prio, f.pgn, f.src, f.dst, &f.data, false, &mut out);
        }
        out
    }

    fn frame(pgn: u32, data: &[u8]) -> RawFrame {
        RawFrame::new(None, 2, pgn, 0x23, 255, data.iter().copied())
    }

    #[test]
    fn a_fast_packet_goes_out_in_frames_and_comes_back_whole() {
        let sent = frame(129029, &(0u8..43).collect::<Vec<_>>());
        let mut codec = Bst95::default();
        let wire = codec.send(&sent).unwrap();
        let frames = wire.windows(2).filter(|w| w == &[0x10, 0x02]).count();
        assert_eq!(frames, 7, "43 bytes: 6 + 6 × 7 → 7 frames");

        let mut events = Vec::new();
        codec.receive(&as_received(&wire), NOW, &mut events);
        let [Event::Frame(got)] = &events[..] else {
            panic!("{events:?}");
        };
        assert_eq!(
            (got.pgn, got.src, got.data.as_slice()),
            (129029, 0x23, &sent.data[..])
        );
        assert_eq!(got.timestamp.as_deref(), Some("2026-05-29T19:16:04.826Z"));
    }

    /// The gateway echoes what we send, marked as going to the bus.
    #[test]
    fn our_own_frames_are_not_received() {
        let mut codec = Bst95::default();
        let wire = codec
            .send(&frame(127250, &[1, 2, 3, 4, 5, 6, 7, 8]))
            .unwrap();
        let mut events = Vec::new();
        codec.receive(&wire, NOW, &mut events);
        assert!(events.is_empty(), "{events:?}");
    }

    #[test]
    fn what_cannot_go_is_refused() {
        let mut codec = Bst95::default();
        assert_eq!(codec.send(&frame(0x40100, &[1])), Err(Refused::Synthetic));
        assert_eq!(codec.send(&frame(127250, &[0; 9])), Err(Refused::TooLarge));
        assert_eq!(
            codec.send(&frame(129029, &[0; 224])),
            Err(Refused::TooLarge)
        );
    }

    #[test]
    fn fast_packet_sequences_count_per_pgn_and_source() {
        let mut codec = Bst95::default();
        assert_eq!(codec.next_seq(129029, 1), 0);
        assert_eq!(codec.next_seq(129029, 1), 1);
        assert_eq!(codec.next_seq(129029, 2), 0);
    }
}
