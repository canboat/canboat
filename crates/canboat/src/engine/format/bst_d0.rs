// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.
//! Actisense BST-D0 messages: reassembled NMEA 2000 messages, as an
//! Actisense W2K-1 or PRO-NDC-1E2K sends them in its "Actisense" data mode
//! (#993). Specified in the Actisense SDK:
//! <https://github.com/Actisense/SDK/blob/main/docs/DataFormats/Binary/bst-detail/BST-D0.md>.
//!
//! A message travels in [BDTP] framing, `DLE STX … DLE ETX`, with every
//! `DLE` inside doubled. Unframed, it is
//!
//! ```text
//! D0 <L:u16 LE> <D> <S> <PDUS> <PDUF> <DPP> <C> <T:u32 LE> <data…> <checksum>
//! ```
//!
//! - `L` is the length of everything before the checksum, `13 + data`.
//! - `D` is the destination, `S PDUS PDUF DPP` the 29-bit CAN identifier
//!   (little-endian: source, PDU specific, PDU format, then data page and
//!   priority), so the PGN, priority and (for a PDU1 PGN) destination come
//!   from those four bytes.
//! - `C` is the control byte: message type in bits 0–1 (0 single frame,
//!   1 fast-packet, 2 ISO multi-packet), direction in bit 3 (1 = to the
//!   bus), internally generated in bit 4, fast-packet sequence in bits 5–7.
//! - `T` is the device's millisecond clock. It is not used.
//! - `checksum` makes the 8-bit sum of the whole message zero.
//!
//! `data` is a whole message: fast-packets and ISO transport already joined.
//! A frame with a bad checksum, length or escape is dropped, as the SDK's
//! BDTP parser does, and decoding resumes at the next `DLE STX`. So is one
//! with more data than [`MAX_DATA_LEN`], which no NMEA 2000 message has. Other BST
//! messages in the stream (a BEM reply, say) are skipped.
//!
//! [BDTP]: https://github.com/Actisense/SDK/blob/main/docs/DataProtocols/bdtp-protocol.md

use smallvec::SmallVec;

use super::common::{iso11783_compose, iso11783_decompose};
use crate::engine::fastpacket;
use crate::engine::{FASTPACKET_MAX_SIZE, FramePacketType, RAWFRAME_MAX_SIZE, RawFrame};

/// BST message ID of a BST-D0 message.
pub const BST_D0: u8 = 0xD0;

const DLE: u8 = 0x10;
const STX: u8 = 0x02;
const ETX: u8 = 0x03;

/// `D0`, `L`, `D`, `S PDUS PDUF DPP`, `C` and `T`: everything before the data.
const HEADER_LEN: usize = 13;

/// The largest data a message can carry. `L` is 16 bits and would allow
/// more, but no NMEA 2000 message is longer than ISO transport's 255
/// packets of 7 bytes, and a [`RawFrame`] holds no more.
pub const MAX_DATA_LEN: usize = RAWFRAME_MAX_SIZE;

/// Most unframed bytes collected for one message before it is given up
/// on: a header, the largest data and the checksum.
const MAX_COLLECT: usize = HEADER_LEN + MAX_DATA_LEN + 1;

/// The largest PGN a CAN identifier carries: 18 bits.
const MAX_PGN: u32 = 0x3ffff;

/// Control byte: the message is going to the bus.
const C_TO_BUS: u8 = 1 << 3;

/// `frame` as one BDTP-framed BST-D0 message for the gateway to send, or
/// `None` when its data is longer than [`MAX_DATA_LEN`] or its PGN does not
/// fit the 18 bits of a CAN identifier (canboat's synthetic PGNs do not).
///
/// The control byte tells the gateway how to send it: as one frame, as a
/// fast-packet, or with ISO transport when it is too long for either. The
/// fast-packet sequence and the time are left 0, as the SDK allows.
pub fn encode_frame(frame: &RawFrame) -> Option<Vec<u8>> {
    let n = frame.data.len();
    if n > MAX_DATA_LEN || frame.pgn > MAX_PGN {
        return None;
    }
    let message_type = match fastpacket::packet_type(frame.pgn) {
        _ if n > FASTPACKET_MAX_SIZE => 2,
        FramePacketType::Fast => 1,
        _ if n > 8 => 2,
        _ => 0,
    };
    let canid = iso11783_compose(frame.prio, frame.pgn, frame.src, frame.dst);
    let mut m = Vec::with_capacity(HEADER_LEN + n + 1);
    m.push(BST_D0);
    m.extend_from_slice(&((HEADER_LEN + n) as u16).to_le_bytes());
    m.push(frame.dst);
    m.extend_from_slice(&canid.to_le_bytes());
    m.push(message_type | C_TO_BUS);
    m.extend_from_slice(&[0, 0, 0, 0]);
    m.extend_from_slice(&frame.data);
    m.push(0u8.wrapping_sub(m.iter().fold(0u8, |s, &b| s.wrapping_add(b))));

    let mut out = Vec::with_capacity(m.len() + 8);
    out.extend_from_slice(&[DLE, STX]);
    for b in m {
        out.push(b);
        if b == DLE {
            out.push(DLE);
        }
    }
    out.extend_from_slice(&[DLE, ETX]);
    Some(out)
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
enum State {
    /// Outside a frame, looking for `DLE STX`.
    #[default]
    Idle,
    /// Outside a frame, just after a `DLE`.
    IdleDle,
    /// Inside a frame, collecting bytes.
    InFrame,
    /// Inside a frame, just after a `DLE`.
    InFrameDle,
}

/// Incremental decoder for a stream of BST-D0 messages. Feed it bytes as
/// they arrive with [`push_bytes`](Self::push_bytes); it returns each
/// message as a [`RawFrame`] once all of it is in.
#[derive(Debug, Default)]
pub struct BstD0Decoder {
    state: State,
    /// The current frame, unframed and with its DLEs undoubled.
    buf: Vec<u8>,
    /// Bytes on the wire that the current frame took so far.
    wire: u64,
    /// Bytes thrown away: outside any frame, in a frame that was dropped,
    /// or in a BST message other than BST-D0.
    pub skipped: u64,
}

impl BstD0Decoder {
    pub fn new() -> Self {
        Self::default()
    }

    /// Add `bytes` and return every message they complete.
    pub fn push_bytes(&mut self, bytes: &[u8]) -> Vec<RawFrame> {
        let mut frames = Vec::new();
        for &b in bytes {
            if let Some(f) = self.push_byte(b) {
                frames.push(f);
            }
        }
        frames
    }

    fn push_byte(&mut self, b: u8) -> Option<RawFrame> {
        match self.state {
            State::Idle => {
                self.skipped += 1;
                if b == DLE {
                    self.state = State::IdleDle;
                }
            }
            State::IdleDle => match b {
                STX => {
                    // The DLE counted as skipped belongs to this frame.
                    self.skipped -= 1;
                    self.start();
                }
                DLE => self.skipped += 1,
                _ => {
                    self.skipped += 1;
                    self.state = State::Idle;
                }
            },
            State::InFrame => {
                self.wire += 1;
                if b == DLE {
                    self.state = State::InFrameDle;
                } else {
                    self.collect(b);
                }
            }
            State::InFrameDle => {
                self.wire += 1;
                match b {
                    DLE => {
                        self.state = State::InFrame;
                        self.collect(DLE);
                    }
                    ETX => {
                        let frame = to_frame(&self.buf);
                        if frame.is_none() {
                            self.skipped += self.wire;
                        }
                        self.state = State::Idle;
                        self.buf.clear();
                        return frame;
                    }
                    STX => {
                        // A new frame before this one ended: drop this one,
                        // but not the `DLE STX` that starts the next.
                        self.skipped += self.wire - 2;
                        self.start();
                    }
                    _ => self.drop_frame(),
                }
            }
        }
        None
    }

    /// Begin a frame after its `DLE STX`.
    fn start(&mut self) {
        self.state = State::InFrame;
        self.buf.clear();
        self.wire = 2;
    }

    fn collect(&mut self, b: u8) {
        if self.buf.len() >= MAX_COLLECT {
            self.drop_frame();
        } else {
            self.buf.push(b);
        }
    }

    fn drop_frame(&mut self) {
        self.skipped += self.wire;
        self.state = State::Idle;
        self.buf.clear();
    }
}

/// The frame in an unframed BST message, or `None` when it is not a
/// well-formed BST-D0 message: another BST ID, a length that does not
/// match, or a bad checksum.
pub(crate) fn to_frame(m: &[u8]) -> Option<RawFrame> {
    if m.len() < HEADER_LEN + 1 || m[0] != BST_D0 {
        return None;
    }
    let len = u16::from_le_bytes([m[1], m[2]]) as usize;
    if len < HEADER_LEN || len + 1 != m.len() || len - HEADER_LEN > MAX_DATA_LEN {
        return None;
    }
    if m.iter().fold(0u8, |s, &b| s.wrapping_add(b)) != 0 {
        return None;
    }
    let canid = u32::from_le_bytes([m[4], m[5], m[6], m[7]]) & 0x1fff_ffff;
    let (prio, pgn, src, dst) = iso11783_decompose(canid);
    Some(RawFrame {
        timestamp: None,
        prio,
        pgn,
        src,
        dst,
        data: SmallVec::from_slice(&m[HEADER_LEN..len]),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A BDTP-framed BST-D0 message received from the bus, built by hand
    /// from the SDK's description.
    fn message(dst: u8, canid: u32, data: &[u8]) -> Vec<u8> {
        let mut m = vec![BST_D0];
        m.extend_from_slice(&((HEADER_LEN + data.len()) as u16).to_le_bytes());
        m.push(dst);
        m.extend_from_slice(&canid.to_le_bytes());
        m.push(0); // control: single frame, from the bus
        m.extend_from_slice(&0x1234_5678u32.to_le_bytes());
        m.extend_from_slice(data);
        let sum = m.iter().fold(0u8, |s, &b| s.wrapping_add(b));
        m.push(0u8.wrapping_sub(sum));
        let mut out = vec![DLE, STX];
        for b in m {
            out.push(b);
            if b == DLE {
                out.push(DLE);
            }
        }
        out.extend_from_slice(&[DLE, ETX]);
        out
    }

    /// 129025 Position, Rapid Update from source 3, priority 2.
    const POSITION: u32 = 0x09F8_0103;
    const POSITION_DATA: [u8; 8] = [0x50, 0x9b, 0x3a, 0x1e, 0x2e, 0xd7, 0x0a, 0x03];

    fn frame(canid: u32, dst: u8, data: &[u8]) -> RawFrame {
        let (prio, pgn, src, _) = iso11783_decompose(canid);
        RawFrame {
            timestamp: None,
            prio,
            pgn,
            src,
            dst,
            data: SmallVec::from_slice(data),
        }
    }

    #[test]
    fn decodes_a_message() {
        let frames = BstD0Decoder::new().push_bytes(&message(255, POSITION, &POSITION_DATA));
        assert_eq!(frames.len(), 1);
        let f = &frames[0];
        assert_eq!((f.prio, f.pgn, f.src, f.dst), (2, 129025, 3, 255));
        assert_eq!(f.data.as_slice(), &POSITION_DATA);
    }

    /// The SDK's Example 2: DPP 0x18, PDUF 0xEA, PDUS 0x1F is an ISO
    /// Request at priority 6 to address 0x1F; the destination of a PDU1
    /// PGN comes from the CAN id.
    #[test]
    fn a_pdu1_destination_comes_from_the_can_id() {
        let frames =
            BstD0Decoder::new().push_bytes(&message(0x1f, 0x18EA_1F03, &[0x14, 0xf0, 0x01]));
        let f = &frames[0];
        assert_eq!((f.prio, f.pgn, f.src, f.dst), (6, 59904, 3, 0x1f));
    }

    /// The SDK's Example 3: data page 1. DPP 0x09, PDUF 0xFD, PDUS 0x08 is
    /// PGN 130312 at priority 2.
    #[test]
    fn data_page_one() {
        let frames = BstD0Decoder::new().push_bytes(&message(255, 0x09FD_0823, &[1, 2, 3]));
        assert_eq!((frames[0].prio, frames[0].pgn), (2, 130312));
    }

    /// A DLE anywhere in the message is sent doubled and read back once:
    /// in the CAN id and the data here.
    #[test]
    fn dles_are_undoubled() {
        // Source 0x10, PDU specific 0x10, 16 data bytes of 0x10.
        let canid = 0x09F8_1010;
        let data = [DLE; 16];
        let bytes = message(255, canid, &data);
        assert!(bytes.windows(2).filter(|w| *w == [DLE, DLE]).count() >= 18);
        let frames = BstD0Decoder::new().push_bytes(&bytes);
        assert_eq!(frames.len(), 1);
        assert_eq!((frames[0].pgn, frames[0].src), (129040, 0x10));
        assert_eq!(frames[0].data.as_slice(), &data);
    }

    /// A whole fast-packet message comes in one BST-D0 message.
    #[test]
    fn a_long_message() {
        let data: Vec<u8> = (0u8..=200).collect();
        let frames = BstD0Decoder::new().push_bytes(&message(255, 0x0DF8_0503, &data));
        assert_eq!(frames[0].pgn, 129029);
        assert_eq!(frames[0].data.as_slice(), data.as_slice());
    }

    /// Bytes arrive in any pieces, a cut between a DLE and its pair
    /// included.
    #[test]
    fn a_message_split_across_reads() {
        let mut stream = message(255, POSITION, &POSITION_DATA);
        stream.extend(message(255, 0x09F8_1010, &[DLE; 8]));
        for cut in 0..stream.len() {
            let mut d = BstD0Decoder::new();
            let mut frames = d.push_bytes(&stream[..cut]);
            frames.extend(d.push_bytes(&stream[cut..]));
            assert_eq!(frames.len(), 2, "cut at {cut}");
            assert_eq!(d.skipped, 0, "cut at {cut}");
        }
    }

    /// A bad checksum, a wrong length, a bad escape and a frame cut short
    /// by the next `DLE STX` each cost that frame only.
    #[test]
    fn drops_bad_frames_and_resyncs() {
        let good = message(255, 0x18EA_1F03, &[0x14, 0xf0, 0x01]);

        let mut bad_checksum = message(255, POSITION, &POSITION_DATA);
        let n = bad_checksum.len();
        bad_checksum[n - 3] ^= 0x01;

        let mut bad_length = message(255, POSITION, &POSITION_DATA);
        bad_length[3] += 1;

        let mut bad_escape = message(255, POSITION, &POSITION_DATA);
        bad_escape.insert(8, DLE);
        bad_escape.insert(9, 0x55);

        let cut_short = message(255, POSITION, &POSITION_DATA)[..10].to_vec();

        let mut stream = vec![0xaa, DLE, 0x55];
        for bad in [&bad_checksum, &bad_length, &bad_escape, &cut_short] {
            stream.extend(bad);
            stream.extend(&good);
        }
        let mut d = BstD0Decoder::new();
        let frames = d.push_bytes(&stream);
        assert_eq!(frames.len(), 4);
        assert!(frames.iter().all(|f| f.pgn == 59904));
        assert!(d.skipped >= 3 + bad_checksum.len() as u64 + bad_length.len() as u64);
    }

    /// Another BST message in the stream, such as a BEM reply, is skipped.
    #[test]
    fn other_bst_messages_are_skipped() {
        // A0 (BEM response), length 3, three bytes, checksum.
        let mut bem = vec![0xA0u8, 3, 0x11, 0x02, 0x00];
        let sum = bem.iter().fold(0u8, |s, &b| s.wrapping_add(b));
        bem.push(0u8.wrapping_sub(sum));
        let mut stream = vec![DLE, STX];
        stream.extend(&bem);
        stream.extend([DLE, ETX]);
        stream.extend(message(255, POSITION, &POSITION_DATA));
        let mut d = BstD0Decoder::new();
        let frames = d.push_bytes(&stream);
        assert_eq!(frames.len(), 1);
        assert_eq!(d.skipped, 10);
    }

    /// What [`encode_frame`] writes is a well-formed message going to the
    /// bus, with the message type the gateway needs, and decodes to the
    /// same frame.
    #[test]
    fn encode_round_trips() {
        for (canid, dst, data, message_type) in [
            (POSITION, 255u8, POSITION_DATA.to_vec(), 0u8),
            (0x18EA_2303, 0x23, vec![0x14, 0xf0, 0x01], 0),
            (0x0DF8_0503, 255, (0u8..43).collect(), 1),
            // A fast-packet PGN is sent as one even when short.
            (0x19F0_1410, 255, vec![DLE; 4], 1),
            // Too long for a fast-packet: ISO transport.
            (0x19F0_1410, 255, vec![0x55; 300], 2),
        ] {
            let f = frame(canid, dst, &data);
            let bytes = encode_frame(&f).unwrap();
            let mut d = BstD0Decoder::new();
            assert_eq!(d.push_bytes(&bytes), vec![f.clone()]);
            assert_eq!(d.skipped, 0);
            // Undo the framing to look at the control byte and length.
            let mut m = Vec::new();
            let mut i = 2;
            while i < bytes.len() - 2 {
                m.push(bytes[i]);
                i += if bytes[i] == DLE { 2 } else { 1 };
            }
            assert_eq!(m[8], message_type | C_TO_BUS, "{:x}", f.pgn);
            assert_eq!(u16::from_le_bytes([m[1], m[2]]) as usize, 13 + data.len());
        }
    }

    /// A message with more data than [`MAX_DATA_LEN`] is dropped, not
    /// passed on truncated, and the next message still decodes.
    #[test]
    fn an_oversized_message_is_dropped() {
        let big = message(255, 0x0DF8_0503, &vec![0x55; MAX_DATA_LEN + 1]);
        let mut stream = big.clone();
        stream.extend(message(255, POSITION, &POSITION_DATA));
        let mut d = BstD0Decoder::new();
        let frames = d.push_bytes(&stream);
        assert_eq!(frames.len(), 1);
        assert_eq!(frames[0].pgn, 129025);
        assert_eq!(d.skipped, big.len() as u64);
        // The largest message there can be still decodes.
        let max = message(255, 0x0DF8_0503, &vec![0x55; MAX_DATA_LEN]);
        assert_eq!(
            BstD0Decoder::new().push_bytes(&max)[0].data.len(),
            MAX_DATA_LEN
        );
        assert!(encode_frame(&frame(0x0DF8_0503, 255, &[0x55; MAX_DATA_LEN])).is_some());
    }

    /// A PGN that does not fit 18 bits is refused: composing the CAN id
    /// would shift it into the priority and send a different PGN.
    #[test]
    fn a_pgn_beyond_18_bits_is_refused() {
        let mut f = frame(POSITION, 255, &POSITION_DATA);
        f.pgn = 0x40000;
        assert!(encode_frame(&f).is_none());
        f.pgn = 0x3ffff;
        assert!(encode_frame(&f).is_some());
    }

    /// A stream with no message in it does not grow the buffer.
    #[test]
    fn noise_is_not_kept() {
        let mut d = BstD0Decoder::new();
        for _ in 0..100 {
            assert!(d.push_bytes(&[0x55; 1000]).is_empty());
        }
        assert!(d.buf.is_empty());
        assert_eq!(d.skipped, 100_000);
    }
}
