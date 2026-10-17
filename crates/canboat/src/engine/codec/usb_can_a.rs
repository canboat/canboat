// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Waveshare USB-CAN-A raw CAN frames: a CH340 USB serial chip (`1a86:7523`)
//! at 2 000 000 baud in front of an STM32 CAN controller.
//!
//! The host first sends a 20-byte configuration packet: `AA 55 02`, which
//! selects the fixed 20-byte framing, the CAN bit rate as a code, the frame
//! type, a filter and mask (0: everything), the mode (0: normal) and
//! automatic retransmission (0: on), then a checksum. The adapter keeps it
//! across power cycles, and does not answer it.
//!
//! Frames then travel both ways as 20 bytes: `AA 55 01`, standard (1) or
//! extended (2), data (1) or remote (2), the identifier (u32,
//! little-endian), the length, eight data bytes (zero-padded), a 0, and a
//! checksum: the low byte of the sum of bytes 2 to 18. The checksum and
//! fixed length let a reader find the frames again when the serial link
//! drops a byte, which the variable-length framing the adapter also has
//! (`AA 55 12`) cannot. Waveshare documents the protocol in "USB (Serial
//! port) to CAN protocol defines".
//!
//! Every frame is one CAN frame, so the codec joins fast-packets on the way
//! in and splits a message into fast-packet frames on the way out, as
//! [`super::canalyst`] does. Frames are stamped with the time they are
//! received. Remote frames are skipped, and so are the frames of the other
//! kind: a Quick bus carries 11-bit standard frames, NMEA 2000 and J1939
//! 29-bit extended ones.

use smallvec::SmallVec;

use crate::engine::reassembly::Reassembler;
use crate::engine::{BusProtocol, RawFrame};

use super::raw_can::{self, CanFrame, FastPacketSeq};
use super::{Codec, Event, Refused};

/// Bytes in a frame, and in the configuration packet.
pub const FRAME_LEN: usize = 20;

/// The serial speed the adapter runs at.
pub const BAUD: u32 = 2_000_000;

/// The CAN bit rates the adapter runs at, and their codes.
const BITRATES: [(u32, u8); 12] = [
    (1_000_000, 0x01),
    (800_000, 0x02),
    (500_000, 0x03),
    (400_000, 0x04),
    (250_000, 0x05),
    (200_000, 0x06),
    (125_000, 0x07),
    (100_000, 0x08),
    (50_000, 0x09),
    (20_000, 0x0a),
    (10_000, 0x0b),
    (5_000, 0x0c),
];

/// The CAN bit rates the adapter runs at.
pub fn bitrates() -> impl Iterator<Item = u32> {
    BITRATES.iter().map(|&(rate, _)| rate)
}

/// The least time from one frame written to the next on a bus at
/// `bitrate`: the adapter loses frames written faster than it sends them.
/// A frame's time on the bus and a margin: 175 bit times, and never less
/// than 0.7 ms. Written back to back at 250 kbit/s, 100 messages of 29
/// frames each came through 100 times at 0.7 ms, 97 times at 0.5 ms and 12
/// times with no gap at all.
pub(crate) fn write_gap(bitrate: u32) -> std::time::Duration {
    use std::time::Duration;
    let bits = Duration::from_micros(175_000_000 / u64::from(bitrate.max(1)));
    bits.max(Duration::from_micros(700))
}

/// The low byte of the sum of bytes 2 to 18.
fn checksum(packet: &[u8]) -> u8 {
    packet[2..19]
        .iter()
        .fold(0u8, |sum, &b| sum.wrapping_add(b))
}

/// The configuration packet for a bus at `bitrate` carrying `protocol`, or
/// `None` for a bit rate the adapter does not have. Normal mode, automatic
/// retransmission on, and no filter.
pub fn config_packet(bitrate: u32, protocol: BusProtocol) -> Option<[u8; FRAME_LEN]> {
    let code = BITRATES.iter().find(|&&(rate, _)| rate == bitrate)?.1;
    let mut packet = [0u8; FRAME_LEN];
    packet[..4].copy_from_slice(&[0xaa, 0x55, 0x02, code]);
    // The frame type: it sends either kind, and passes on both.
    packet[4] = if protocol.standard_frames() { 1 } else { 2 };
    packet[19] = checksum(&packet);
    Some(packet)
}

/// Append the frame that carries `frame`.
pub(crate) fn encode_frame(frame: &CanFrame, out: &mut Vec<u8>) {
    debug_assert!(frame.data.len() <= 8);
    let start = out.len();
    out.resize(start + FRAME_LEN, 0);
    let packet = &mut out[start..];
    packet[..5].copy_from_slice(&[0xaa, 0x55, 0x01, 1 + u8::from(frame.extended), 0x01]);
    packet[5..9].copy_from_slice(&frame.id.to_le_bytes());
    packet[9] = frame.data.len() as u8;
    packet[10..10 + frame.data.len()].copy_from_slice(&frame.data);
    packet[19] = checksum(packet);
}

/// What a 20-byte packet holds.
enum Packet {
    Data(CanFrame),
    /// A remote frame, or another packet with a good checksum: skipped.
    Other,
}

/// The packet at the start of `p`, or `None` when that is no packet.
fn packet(p: &[u8; FRAME_LEN]) -> Option<Packet> {
    if p[..2] != [0xaa, 0x55] || p[19] != checksum(p) {
        return None;
    }
    if p[2] != 0x01 {
        return Some(Packet::Other);
    }
    let (kind, remote, len) = (p[3], p[4], usize::from(p[9]));
    if !matches!(kind, 1 | 2) || !matches!(remote, 1 | 2) || len > 8 || p[18] != 0 {
        return None;
    }
    if remote == 2 {
        return Some(Packet::Other);
    }
    let extended = kind == 2;
    let id = u32::from_le_bytes([p[5], p[6], p[7], p[8]]);
    Some(Packet::Data(CanFrame {
        id: id & if extended { 0x1fff_ffff } else { 0x7ff },
        extended,
        data: SmallVec::from_slice(&p[10..10 + len]),
    }))
}

/// The frames in a byte stream that may split them, or lose a byte.
#[derive(Debug, Default)]
pub(crate) struct Frames {
    buf: Vec<u8>,
    /// Bytes skipped to find the next frame.
    pub skipped: u64,
}

impl Frames {
    /// Call `each` for every data frame `bytes` completes.
    pub fn push(&mut self, bytes: &[u8], mut each: impl FnMut(CanFrame)) {
        self.buf.extend_from_slice(bytes);
        let mut at = 0;
        while self.buf.len() - at >= FRAME_LEN {
            let p: &[u8; FRAME_LEN] = self.buf[at..at + FRAME_LEN].try_into().expect("20 bytes");
            match packet(p) {
                Some(Packet::Data(frame)) => {
                    each(frame);
                    at += FRAME_LEN;
                }
                Some(Packet::Other) => at += FRAME_LEN,
                // Not a packet: look for the next one a byte further on.
                None => {
                    at += 1;
                    self.skipped += 1;
                }
            }
        }
        self.buf.drain(..at);
    }
}

/// Waveshare USB-CAN-A, raw CAN frames. See the [module docs](self).
pub struct UsbCanA {
    config: [u8; FRAME_LEN],
    frames: Frames,
    /// Decides which frames are fast-packets.
    protocol: BusProtocol,
    reassembler: Reassembler,
    seq: FastPacketSeq,
}

impl Default for UsbCanA {
    fn default() -> Self {
        Self::new(BusProtocol::Nmea2000)
    }
}

impl UsbCanA {
    /// A codec for a 250 kbit/s bus that carries `protocol`.
    pub fn new(protocol: BusProtocol) -> Self {
        Self::with_bitrate(protocol, 250_000).expect("250 kbit/s is one of the adapter's rates")
    }

    /// A codec for a bus at `bitrate` that carries `protocol`, or `None` for
    /// a bit rate the adapter does not have (see [`bitrates`]).
    pub fn with_bitrate(protocol: BusProtocol, bitrate: u32) -> Option<Self> {
        Some(Self {
            config: config_packet(bitrate, protocol)?,
            frames: Frames::default(),
            protocol,
            reassembler: Reassembler::new(),
            seq: FastPacketSeq::default(),
        })
    }
}

impl Codec for UsbCanA {
    /// A quiet bus still gets its minute's summary of incomplete
    /// fast-packets.
    fn tick(&mut self, now_ms: u64, _events: &mut Vec<Event>) {
        self.reassembler.tick(now_ms);
    }

    /// The configuration packet.
    fn open(&mut self) -> Vec<u8> {
        self.config.to_vec()
    }

    fn receive(&mut self, bytes: &[u8], now_ms: u64, events: &mut Vec<Event>) {
        let (protocol, reassembler) = (self.protocol, &mut self.reassembler);
        self.frames.push(bytes, |f| {
            raw_can::receive(
                protocol,
                reassembler,
                f.id,
                f.extended,
                &f.data,
                Some(now_ms),
                events,
            )
        });
    }

    fn send(&mut self, frame: &RawFrame) -> Result<Vec<u8>, Refused> {
        let mut out = Vec::new();
        for f in raw_can::split(self.protocol, &mut self.seq, frame)? {
            encode_frame(&f, &mut out);
        }
        Ok(out)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::engine::format_iso_ms;

    const NOW: u64 = 1_780_082_164_826;

    fn hex(s: &str) -> Vec<u8> {
        s.split_whitespace()
            .map(|b| u8::from_str_radix(b, 16).unwrap())
            .collect()
    }

    fn receive(codec: &mut UsbCanA, wire: &[u8]) -> Vec<Event> {
        let mut events = Vec::new();
        codec.receive(wire, NOW, &mut events);
        events
    }

    fn frames(wire: &[u8]) -> Vec<CanFrame> {
        let mut out = Vec::new();
        Frames::default().push(wire, |f| out.push(f));
        out
    }

    #[test]
    fn the_config_packet() {
        assert_eq!(
            config_packet(250_000, BusProtocol::Nmea2000).unwrap()[..],
            hex("AA 55 02 05 02 00 00 00 00 00 00 00 00 00 00 00 00 00 00 09")
        );
        assert_eq!(
            config_packet(500_000, BusProtocol::Quick).unwrap()[..],
            hex("AA 55 02 03 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 06")
        );
        assert_eq!(config_packet(300_000, BusProtocol::Nmea2000), None);
        assert!(UsbCanA::with_bitrate(BusProtocol::J1939, 300_000).is_none());
        assert_eq!(UsbCanA::default().open()[3], 0x05);
    }

    /// The frames of #1016: one written from Waveshare's description, two
    /// captured off a bus by tomer-w/nmea2000, and the checksums of
    /// Waveshare's own examples.
    #[test]
    fn the_golden_frames() {
        let ours = UsbCanA::default()
            .send(&RawFrame::new(None, 2, 127250, 0x23, 255, 0..8))
            .unwrap();
        assert_eq!(
            ours,
            hex("AA 55 01 02 01 23 12 F1 09 08 00 01 02 03 04 05 06 07 00 57")
        );

        let heave = hex("aa 55 01 02 01 09 00 ff 1c 08 3f 9f dc ff ff ff ff ff 00 e5");
        let events = receive(&mut UsbCanA::default(), &heave);
        let [Event::Frame(f)] = &events[..] else {
            panic!("{events:?}");
        };
        assert_eq!((f.prio, f.pgn, f.src), (7, 65280, 0x09));
        assert_eq!(
            f.data.as_slice(),
            &[0x3f, 0x9f, 0xdc, 0xff, 0xff, 0xff, 0xff, 0xff]
        );
        assert_eq!(f.timestamp.as_deref(), Some(format_iso_ms(NOW).as_str()));

        let rot = hex("aa 55 01 02 01 01 13 f1 09 08 ff fa c2 ff ff ff ff ff 00 d0");
        let events = receive(&mut UsbCanA::default(), &rot);
        assert!(matches!(&events[..], [Event::Frame(f)] if (f.pgn, f.src) == (127251, 1)));

        let mut out = Vec::new();
        encode_frame(
            &CanFrame {
                id: 0x123,
                extended: false,
                data: SmallVec::from_slice(&[0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88]),
            },
            &mut out,
        );
        assert_eq!(out[19], 0x93);
        out.clear();
        encode_frame(
            &CanFrame {
                id: 0x1234_5678,
                extended: true,
                data: SmallVec::from_slice(&[1, 2, 3, 4, 5, 6, 7, 8]),
            },
            &mut out,
        );
        assert_eq!(out[19], 0x44);
    }

    #[test]
    fn a_fast_packet_goes_out_in_frames_and_comes_back_whole() {
        let sent = RawFrame::new(None, 2, 129029, 0x23, 255, 0u8..43);
        let mut codec = UsbCanA::default();
        let wire = codec.send(&sent).unwrap();
        // 43 bytes: 7 frames of 20 bytes, each with a good checksum.
        assert_eq!(wire.len(), 7 * FRAME_LEN);
        for p in wire.as_chunks::<FRAME_LEN>().0 {
            assert_eq!(p[19], checksum(p));
        }
        let events = receive(&mut codec, &wire);
        let [Event::Frame(got)] = &events[..] else {
            panic!("{events:?}");
        };
        assert_eq!(
            (got.prio, got.pgn, got.src, got.dst, got.data.as_slice()),
            (2, 129029, 0x23, 255, sent.data.as_slice())
        );
    }

    #[test]
    fn a_frame_split_across_reads_is_joined() {
        let mut codec = UsbCanA::default();
        let wire = codec
            .send(&RawFrame::new(None, 2, 127250, 0x23, 255, 0..8))
            .unwrap();
        assert!(receive(&mut codec, &wire[..7]).is_empty());
        let events = receive(&mut codec, &wire[7..]);
        assert!(matches!(&events[..], [Event::Frame(f)] if f.pgn == 127250));
    }

    /// A lost byte, a bad checksum, an `AA 55` inside the data, and noise
    /// before the first frame: only whole, good frames come out.
    #[test]
    fn a_reader_finds_the_frames_again() {
        let frame = |data: &[u8]| {
            let mut out = Vec::new();
            encode_frame(
                &CanFrame {
                    id: 0x09f1_1223,
                    extended: true,
                    data: SmallVec::from_slice(data),
                },
                &mut out,
            );
            out
        };
        let mut wire = vec![0x55, 0xaa, 0x00, 0xaa];
        let mut lost = frame(&[1, 2, 3, 4, 5, 6, 7, 8]);
        lost.remove(12);
        wire.extend(lost);
        let mut bad = frame(&[9; 8]);
        bad[19] ^= 0xff;
        wire.extend(bad);
        wire.extend(frame(&[0xaa, 0x55, 0x01, 0x02, 0x01, 0xaa, 0x55, 0x00]));
        wire.extend(frame(&[4, 5]));
        let mut f = Frames::default();
        let mut got = Vec::new();
        f.push(&wire, |frame| got.push(frame.data.to_vec()));
        assert_eq!(
            got,
            [
                vec![0xaa, 0x55, 0x01, 0x02, 0x01, 0xaa, 0x55, 0x00],
                vec![4, 5]
            ]
        );
        assert_eq!(f.skipped, 4 + 19 + 20);
    }

    /// A status packet, a remote frame and frames of the other kind pass
    /// nothing on.
    #[test]
    fn other_packets_and_frames_are_skipped() {
        let mut status = [0u8; FRAME_LEN];
        status[..3].copy_from_slice(&[0xaa, 0x55, 0x04]);
        status[19] = checksum(&status);
        let mut remote = UsbCanA::default()
            .send(&RawFrame::new(None, 2, 127250, 0x23, 255, 0..8))
            .unwrap();
        remote[4] = 2;
        remote[19] = checksum(&remote);
        let quick = UsbCanA::new(BusProtocol::Quick)
            .send(&RawFrame::new(None, 0, 0x6c1, 0, 255, [1, 2]))
            .unwrap();
        let mut wire = status.to_vec();
        wire.extend(&remote);
        wire.extend(&quick);
        assert!(frames(&wire[..2 * FRAME_LEN]).is_empty());
        assert!(receive(&mut UsbCanA::default(), &wire).is_empty());
    }

    #[test]
    fn a_quick_bus_carries_11_bit_frames_both_ways() {
        let mut codec = UsbCanA::new(BusProtocol::Quick);
        let q = RawFrame::new(None, 0, 0x6c1, 0, 255, [0xc1, 0x18, 0x6a]);
        let wire = codec.send(&q).unwrap();
        assert_eq!(&wire[..10], &hex("AA 55 01 01 01 C1 06 00 00 03")[..]);
        let events = receive(&mut codec, &wire);
        assert!(matches!(&events[..], [Event::Frame(f)]
            if (f.prio, f.pgn, f.src, f.dst) == (0, 0x6c1, 0, 255)
                && f.data.as_slice() == [0xc1, 0x18, 0x6a]));
        let n2k = UsbCanA::default()
            .send(&RawFrame::new(None, 2, 127250, 0x23, 255, 0..8))
            .unwrap();
        assert!(receive(&mut codec, &n2k).is_empty());
        assert_eq!(
            codec.send(&RawFrame::new(None, 0, 0x800, 0, 255, [0])),
            Err(Refused::TooLarge)
        );
    }

    #[test]
    fn synthetic_and_oversized_frames_are_refused() {
        let mut codec = UsbCanA::default();
        let frame = |pgn, len| RawFrame::new(None, 2, pgn, 0x23, 255, vec![0; len]);
        assert_eq!(codec.send(&frame(0x40100, 1)), Err(Refused::Synthetic));
        assert_eq!(codec.send(&frame(129029, 224)), Err(Refused::TooLarge));
        assert_eq!(codec.send(&frame(59904, 9)), Err(Refused::TooLarge));
    }
}
