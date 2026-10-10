// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! CANalyst-II raw CAN frames: the Waveshare USB-CAN-B, and the other
//! "Chuangxin Tech USBCAN/CANalyst-II" adapters (`04d8:0053`).
//!
//! The adapter moves CAN frames over USB bulk endpoints in 64-byte
//! packets: a count, then up to three 21-byte messages. Each message is
//! the CAN identifier (u32, little-endian), a timestamp in 100 µs units
//! (u32), a time flag, a send type, the remote and extended flags, the
//! data length and eight data bytes. The layout is the one
//! [python-canalystii](https://github.com/projectgus/python-canalystii)
//! documents. Opening the channel (bit rate, start) happens on a separate
//! command endpoint, outside this codec — see `io::canalyst`.
//!
//! Every message is one CAN frame, so the codec joins fast-packets on the
//! way in and splits a message into fast-packet frames on the way out,
//! as [`super::bst95`] does. Frames are stamped with the time they are
//! received. Standard (11-bit) and remote frames are skipped.

use std::collections::HashMap;

use crate::engine::format::{iso11783_compose, iso11783_decompose};
use crate::engine::reassembly::{Reassembled, Reassembler};
use crate::engine::{BusProtocol, FramePacketType, RawFrame, fastpacket, format_iso_ms};

use super::{Codec, Event, Refused, SYNTHETIC_PGN_START};

/// Bytes in one USB packet: a count and three messages.
pub const PACKET_LEN: usize = 64;
/// Messages in one packet.
const MESSAGES_PER_PACKET: usize = 3;
/// Bytes in one message.
const MESSAGE_LEN: usize = 21;

/// One CAN data frame from a packet. Remote frames are left out.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct Message<'a> {
    /// 29 bits when `extended`, else 11.
    pub id: u32,
    pub extended: bool,
    pub data: &'a [u8],
}

/// The data frames in `packet`, or the message count it claims when that
/// is more than a packet holds.
pub(crate) fn messages(packet: &[u8; PACKET_LEN]) -> Result<Vec<Message<'_>>, usize> {
    let count = usize::from(packet[0]);
    if count > MESSAGES_PER_PACKET {
        return Err(count);
    }
    Ok(packet[1..]
        .as_chunks::<MESSAGE_LEN>()
        .0
        .iter()
        .take(count)
        .filter_map(|m| {
            let id = u32::from_le_bytes([m[0], m[1], m[2], m[3]]);
            let (remote, extended, len) = (m[10] != 0, m[11] != 0, usize::from(m[12]));
            (!remote && len <= 8).then(|| Message {
                id: id & if extended { 0x1FFF_FFFF } else { 0x7FF },
                extended,
                data: &m[13..13 + len],
            })
        })
        .collect())
}

/// Append `frames` as packets of up to three messages.
pub(crate) fn encode_packets(frames: &[Message<'_>], out: &mut Vec<u8>) {
    for chunk in frames.chunks(MESSAGES_PER_PACKET) {
        let start = out.len();
        out.resize(start + PACKET_LEN, 0);
        let packet = &mut out[start..];
        packet[0] = chunk.len() as u8;
        for (i, f) in chunk.iter().enumerate() {
            debug_assert!(f.data.len() <= 8);
            let m = &mut packet[1 + i * MESSAGE_LEN..1 + (i + 1) * MESSAGE_LEN];
            m[0..4].copy_from_slice(&f.id.to_le_bytes());
            // Timestamp, time flag, send type (0: retry until sent) and
            // the remote flag stay 0.
            m[11] = u8::from(f.extended);
            m[12] = f.data.len() as u8;
            m[13..13 + f.data.len()].copy_from_slice(f.data);
        }
    }
}

/// Whole packets out of a byte stream that may split them.
#[derive(Debug, Default)]
pub(crate) struct Packets {
    partial: Vec<u8>,
}

impl Packets {
    /// Call `each` for every packet `bytes` completes.
    pub fn push(&mut self, mut bytes: &[u8], mut each: impl FnMut(&[u8; PACKET_LEN])) {
        if !self.partial.is_empty() {
            let take = (PACKET_LEN - self.partial.len()).min(bytes.len());
            self.partial.extend_from_slice(&bytes[..take]);
            bytes = &bytes[take..];
            if self.partial.len() < PACKET_LEN {
                return;
            }
            let packet: [u8; PACKET_LEN] = self.partial[..].try_into().expect("a whole packet");
            self.partial.clear();
            each(&packet);
        }
        let (packets, rest) = bytes.as_chunks::<PACKET_LEN>();
        packets.iter().for_each(each);
        self.partial.extend_from_slice(rest);
    }
}

/// An extended frame's message.
fn extended(id: u32, data: &[u8]) -> Message<'_> {
    Message {
        id,
        extended: true,
        data,
    }
}

/// CANalyst-II, raw CAN frames. See the [module docs](self).
pub struct Canalyst {
    packets: Packets,
    /// Decides which frames are fast-packets.
    protocol: BusProtocol,
    reassembler: Reassembler,
    /// Per-(pgn, src) fast-packet sequence counters, mod 8.
    seq: HashMap<(u32, u8), u8>,
}

impl Default for Canalyst {
    fn default() -> Self {
        Self::new(BusProtocol::Nmea2000)
    }
}

impl Canalyst {
    /// A codec for a bus that carries `protocol`.
    pub fn new(protocol: BusProtocol) -> Self {
        Self {
            packets: Packets::default(),
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

    fn packet(
        protocol: BusProtocol,
        reassembler: &mut Reassembler,
        packet: &[u8; PACKET_LEN],
        now_ms: u64,
        events: &mut Vec<Event>,
    ) {
        let messages = match messages(packet) {
            Ok(m) => m,
            Err(count) => {
                events.push(Event::Error(format!(
                    "CANalyst packet claims {count} messages"
                )));
                return;
            }
        };
        for m in messages.iter().filter(|m| m.extended) {
            let (prio, pgn, src, dst) = iso11783_decompose(m.id);
            let mut frame = RawFrame::new(None, prio, pgn, src, dst, m.data.iter().copied());
            frame.timestamp = Some(format_iso_ms(now_ms));
            let pt = protocol.packet_type(frame.pgn);
            match reassembler.push(frame, pt) {
                Reassembled::PassThrough(f) | Reassembled::Complete(f) => {
                    events.push(Event::Frame(f))
                }
                Reassembled::Partial => {}
                Reassembled::Error(e) => events.push(Event::Error(e.to_string())),
            }
        }
    }
}

impl Codec for Canalyst {
    fn receive(&mut self, bytes: &[u8], now_ms: u64, events: &mut Vec<Event>) {
        let (protocol, reassembler) = (self.protocol, &mut self.reassembler);
        self.packets.push(bytes, |packet| {
            Self::packet(protocol, reassembler, packet, now_ms, events)
        });
    }

    fn send(&mut self, frame: &RawFrame) -> Result<Vec<u8>, Refused> {
        if frame.pgn >= SYNTHETIC_PGN_START {
            return Err(Refused::Synthetic);
        }
        let id = iso11783_compose(frame.prio & 7, frame.pgn, frame.src, frame.dst);
        let mut out = Vec::new();
        if self.protocol.packet_type(frame.pgn) == FramePacketType::Fast {
            let seq = self.next_seq(frame.pgn, frame.src);
            let chunks = fastpacket::fragment(seq, &frame.data).ok_or(Refused::TooLarge)?;
            let frames: Vec<Message> = chunks.iter().map(|c| extended(id, c)).collect();
            encode_packets(&frames, &mut out);
        } else if frame.data.len() <= 8 {
            encode_packets(&[extended(id, &frame.data)], &mut out);
        } else {
            // One message is one CAN frame. A longer message needs ISO TP
            // (J1939), which this codec receives but does not send.
            return Err(Refused::TooLarge);
        }
        Ok(out)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const NOW: u64 = 1_780_082_164_826;

    fn frame(pgn: u32, data: &[u8]) -> RawFrame {
        RawFrame::new(None, 2, pgn, 0x23, 255, data.iter().copied())
    }

    fn receive(codec: &mut Canalyst, wire: &[u8]) -> Vec<Event> {
        let mut events = Vec::new();
        codec.receive(wire, NOW, &mut events);
        events
    }

    /// A packet as python-canalystii's `MessageBuffer` lays it out.
    #[test]
    fn a_packet_carries_three_messages() {
        let wire = Canalyst::default()
            .send(&frame(129029, &(0u8..20).collect::<Vec<_>>()))
            .unwrap();
        // 20 bytes: 6 + 7 + 7 → 3 frames, one packet.
        assert_eq!(wire.len(), PACKET_LEN);
        assert_eq!(wire[0], 3);
        let first = &wire[1..1 + MESSAGE_LEN];
        assert_eq!(
            u32::from_le_bytes(first[0..4].try_into().unwrap()),
            0x09F8_0523
        );
        assert_eq!(&first[8..13], &[0, 0, 0, 1, 8]);
        assert_eq!(&first[13..], &[0x00, 20, 0, 1, 2, 3, 4, 5]);
    }

    #[test]
    fn a_fast_packet_goes_out_in_frames_and_comes_back_whole() {
        let sent = frame(129029, &(0u8..43).collect::<Vec<_>>());
        let mut codec = Canalyst::default();
        let wire = codec.send(&sent).unwrap();
        // 43 bytes: 7 frames, so three packets (3 + 3 + 1).
        assert_eq!(wire.len(), 3 * PACKET_LEN);
        assert_eq!((wire[0], wire[PACKET_LEN], wire[2 * PACKET_LEN]), (3, 3, 1));

        let events = receive(&mut codec, &wire);
        let [Event::Frame(got)] = &events[..] else {
            panic!("{events:?}");
        };
        assert_eq!(
            (got.prio, got.pgn, got.src, got.dst, got.data.as_slice()),
            (2, 129029, 0x23, 255, sent.data.as_slice())
        );
        assert_eq!(got.timestamp.as_deref(), Some(format_iso_ms(NOW).as_str()));
    }

    #[test]
    fn a_single_frame_has_a_destination() {
        let sent = RawFrame::new(None, 6, 59904, 3, 0x23, [0x14, 0xf0, 0x01]);
        let mut codec = Canalyst::default();
        let wire = codec.send(&sent).unwrap();
        let events = receive(&mut codec, &wire);
        let [Event::Frame(got)] = &events[..] else {
            panic!("{events:?}");
        };
        assert_eq!(
            (got.prio, got.pgn, got.src, got.dst, got.data.as_slice()),
            (6, 59904, 3, 0x23, &[0x14, 0xf0, 0x01][..])
        );
    }

    #[test]
    fn a_packet_split_across_reads_is_joined() {
        let mut codec = Canalyst::default();
        let wire = codec
            .send(&frame(127250, &[1, 2, 3, 4, 5, 6, 7, 8]))
            .unwrap();
        let mut events = receive(&mut codec, &wire[..10]);
        assert!(events.is_empty());
        events = receive(&mut codec, &wire[10..]);
        assert!(matches!(&events[..], [Event::Frame(f)] if f.pgn == 127250));
    }

    #[test]
    fn standard_and_remote_frames_are_skipped() {
        let mut wire = Canalyst::default()
            .send(&frame(127250, &[1, 2, 3, 4, 5, 6, 7, 8]))
            .unwrap();
        let mut standard = wire.clone();
        standard[1 + 11] = 0;
        wire[1 + 10] = 1;
        wire.extend(standard);
        assert!(receive(&mut Canalyst::default(), &wire).is_empty());
    }

    #[test]
    fn a_bad_count_is_reported() {
        let mut wire = vec![0u8; PACKET_LEN];
        wire[0] = 4;
        let events = receive(&mut Canalyst::default(), &wire);
        assert!(matches!(&events[..], [Event::Error(_)]), "{events:?}");
    }

    #[test]
    fn synthetic_and_oversized_frames_are_refused() {
        let mut codec = Canalyst::default();
        assert_eq!(codec.send(&frame(0x40100, &[1])), Err(Refused::Synthetic));
        assert_eq!(
            codec.send(&frame(129029, &[0; 224])),
            Err(Refused::TooLarge)
        );
        assert_eq!(codec.send(&frame(59904, &[0; 9])), Err(Refused::TooLarge));
    }
}
