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
//!
//! A longer message that is not a fast-packet, 9 to 1785 bytes, goes out by
//! ISO TP ([`crate::engine::iso_tp`]): a BAM to global, its data packets
//! [`BAM_GAP_MS`](crate::engine::iso_tp::BAM_GAP_MS) apart at
//! [`Codec::next_deadline`]; RTS/CTS to one address, driven by the Clear To
//! Send frames the bus delivers.

use std::collections::HashMap;

use crate::engine::format::bst::{BdtpDecoder, bst95_message, encode_bst95};
use crate::engine::iso_tp::{PGN_TP_CM, TP_MAX_SIZE, TpError, TpSender};
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
    /// The ISO TP transfers being sent.
    tp: TpSender,
    /// The time of the last receive or tick, for [`Codec::send`].
    now_ms: u64,
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
            tp: TpSender::new(),
            now_ms: 0,
        }
    }

    /// The gateway's bytes for ISO TP `frames`.
    fn encode(frames: &[RawFrame]) -> Vec<u8> {
        let mut out = Vec::new();
        for f in frames {
            encode_bst95(f.prio, f.pgn, f.src, f.dst, &f.data, &mut out);
        }
        out
    }

    /// What ISO TP has due at `now_ms`, as bytes to write.
    fn poll_tp(&mut self, now_ms: u64, events: &mut Vec<Event>) {
        let due = self.tp.poll(now_ms);
        if !due.is_empty() {
            events.push(Event::Send(Self::encode(&due)));
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
    /// A quiet bus still gets its minute's summary of incomplete
    /// fast-packets, and the ISO TP packets that are due.
    fn tick(&mut self, now_ms: u64, events: &mut Vec<Event>) {
        self.now_ms = now_ms;
        self.reassembler.tick(now_ms);
        self.poll_tp(now_ms, events);
    }

    fn next_deadline(&self) -> Option<u64> {
        self.tp.next_deadline()
    }

    fn receive(&mut self, bytes: &[u8], now_ms: u64, events: &mut Vec<Event>) {
        self.now_ms = now_ms;
        let dropped = self.bdtp.dropped;
        self.reassembler.tick(now_ms);
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
            // A Clear To Send, End Of Message or Abort for what we send.
            if frame.pgn == PGN_TP_CM {
                let answer = self.tp.on_frame(now_ms, &frame);
                if !answer.is_empty() {
                    events.push(Event::Send(Self::encode(&answer)));
                }
            }
            let pt = self.protocol.packet_type(frame.pgn);
            match self.reassembler.push(frame, pt) {
                Reassembled::PassThrough(f) | Reassembled::Complete(f) => {
                    events.push(Event::Frame(f))
                }
                Reassembled::Partial => {}
                Reassembled::Error(e) => events.push(Event::Error(e.to_string())),
            }
        }
        // The window a Clear To Send asked for.
        self.poll_tp(now_ms, events);
        let dropped = self.bdtp.dropped - dropped;
        if dropped > 0 {
            events.push(Event::Error(format!(
                "dropped {dropped} damaged BDTP frame(s)"
            )));
        }
    }

    fn send(&mut self, frame: &RawFrame) -> Result<Vec<u8>, Refused> {
        self.send_at(frame, self.now_ms)
    }

    fn send_at(&mut self, frame: &RawFrame, now_ms: u64) -> Result<Vec<u8>, Refused> {
        self.now_ms = now_ms;
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
        } else if frame.data.len() <= TP_MAX_SIZE {
            // One BST-95 message is one CAN frame: a longer one goes by ISO
            // TP, its announcement now and its data at the deadlines.
            let announce = self
                .tp
                .send(now_ms, frame.pgn, frame.src, frame.dst, &frame.data)
                .map_err(|e| match e {
                    TpError::QueueFull => Refused::Busy,
                    _ => Refused::TooLarge,
                })?;
            return Ok(Self::encode(&announce));
        } else {
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
        assert_eq!(
            codec.send(&frame(129029, &[0; 224])),
            Err(Refused::TooLarge)
        );
    }

    /// The frames in what the codec writes.
    fn frames_in(wire: &[u8]) -> Vec<RawFrame> {
        let mut d = BdtpDecoder::default();
        wire.iter()
            .filter_map(|&b| d.push_byte(b))
            .map(|m| bst95_message(&m).unwrap().0)
            .collect()
    }

    /// The bytes the codec asks to write.
    fn sent(events: &[Event]) -> Vec<RawFrame> {
        events
            .iter()
            .flat_map(|e| match e {
                Event::Send(b) => frames_in(b),
                _ => Vec::new(),
            })
            .collect()
    }

    /// A TP.CM from `src` to `dst`, as the bus delivers it.
    fn cm(src: u8, dst: u8, data: [u8; 8]) -> Vec<u8> {
        let mut out = Vec::new();
        encode_bst95_message(7, PGN_TP_CM, src, dst, &data, false, &mut out);
        out
    }

    #[test]
    fn a_long_message_to_global_goes_out_as_a_bam_at_its_deadlines() {
        use crate::engine::iso_tp::{BAM_GAP_MS, PGN_TP_DT};
        let mut codec = Bst95::new(BusProtocol::J1939);
        let vin: Vec<u8> = (1..=20).collect();
        let wire = codec.send_at(&frame(65260, &vin), NOW).unwrap();
        let announce = frames_in(&wire);
        assert_eq!(announce.len(), 1);
        assert_eq!((announce[0].pgn, announce[0].data[0]), (PGN_TP_CM, 32));

        let mut packets = Vec::new();
        let mut at = Vec::new();
        while let Some(due) = codec.next_deadline() {
            let mut events = Vec::new();
            codec.tick(due, &mut events);
            at.push(due);
            packets.extend(sent(&events));
        }
        assert_eq!(packets.len(), 3);
        assert!(packets.iter().all(|f| f.pgn == PGN_TP_DT && f.src == 0x23));
        assert_eq!(
            at,
            [NOW + BAM_GAP_MS, NOW + 2 * BAM_GAP_MS, NOW + 3 * BAM_GAP_MS]
        );
        assert_eq!(&packets[2].data[..], &[3, 15, 16, 17, 18, 19, 20, 0xFF]);
    }

    #[test]
    fn a_clear_to_send_releases_its_window_and_end_of_message_finishes() {
        use crate::engine::iso_tp::PGN_TP_DT;
        let mut codec = Bst95::new(BusProtocol::J1939);
        let mut to_peer = frame(65260, &[7; 20]);
        to_peer.dst = 0x30;
        let rts = frames_in(&codec.send_at(&to_peer, NOW).unwrap());
        assert_eq!((rts[0].data[0], rts[0].dst), (16, 0x30));

        // CTS: two packets, from the first.
        let [p0, p1, p2] = [0xEC, 0xFE, 0x00];
        let mut events = Vec::new();
        codec.receive(
            &cm(0x30, 0x23, [17, 2, 1, 0xFF, 0xFF, p0, p1, p2]),
            NOW + 10,
            &mut events,
        );
        let window = sent(&events);
        assert_eq!(window.len(), 2);
        assert!(window.iter().all(|f| f.pgn == PGN_TP_DT && f.dst == 0x30));
        assert_eq!((window[0].data[0], window[1].data[0]), (1, 2));

        let mut events = Vec::new();
        codec.receive(
            &cm(0x30, 0x23, [17, 1, 3, 0xFF, 0xFF, p0, p1, p2]),
            NOW + 20,
            &mut events,
        );
        assert_eq!(sent(&events).len(), 1);

        let mut events = Vec::new();
        codec.receive(
            &cm(0x30, 0x23, [19, 20, 0, 3, 0xFF, p0, p1, p2]),
            NOW + 30,
            &mut events,
        );
        assert!(sent(&events).is_empty());
        assert_eq!(codec.next_deadline(), None, "the transfer is done");
    }

    #[test]
    fn a_silent_receiver_gets_an_abort() {
        use crate::engine::iso_tp::T3_MS;
        let mut codec = Bst95::new(BusProtocol::J1939);
        let mut to_peer = frame(65260, &[7; 20]);
        to_peer.dst = 0x30;
        codec.send_at(&to_peer, NOW).unwrap();
        assert_eq!(codec.next_deadline(), Some(NOW + T3_MS));
        let mut events = Vec::new();
        codec.tick(NOW + T3_MS, &mut events);
        let abort = sent(&events);
        assert_eq!((abort.len(), abort[0].data[0]), (1, 255));
        assert_eq!(codec.next_deadline(), None);
    }

    /// The gateway's echoes of the TP frames we send are neither
    /// reassembled nor taken for the receiver's answers.
    #[test]
    fn our_own_tp_frames_are_not_received() {
        let mut codec = Bst95::new(BusProtocol::J1939);
        let mut wire = codec.send_at(&frame(65260, &[7; 20]), NOW).unwrap();
        while let Some(due) = codec.next_deadline() {
            let mut events = Vec::new();
            codec.tick(due, &mut events);
            for e in events {
                if let Event::Send(b) = e {
                    wire.extend(b);
                }
            }
        }
        let mut events = Vec::new();
        codec.receive(&wire, NOW + 1000, &mut events);
        assert!(events.is_empty(), "{events:?}");
    }

    #[test]
    fn fast_packet_sequences_count_per_pgn_and_source() {
        let mut codec = Bst95::default();
        assert_eq!(codec.next_seq(129029, 1), 0);
        assert_eq!(codec.next_seq(129029, 1), 1);
        assert_eq!(codec.next_seq(129029, 2), 0);
    }
}
