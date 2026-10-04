// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.
//! Actisense binary "N2K" messages: BST ID `0xD0`, as an Actisense W2K-1
//! sends them over TCP in its "Actisense" data mode (#993).
//!
//! One message is
//!
//! ```text
//! DLE STX D0 <len:u16 LE> <dst> <canid:u32 LE> <time:u32 LE> <mhs> <data…> <x> DLE ETX
//! ```
//!
//! where `len` counts every byte after itself, the closing `DLE ETX`
//! included, so a message is `5 + len` bytes long. `canid` is the 29-bit
//! ISO 11783 identifier, which carries the priority, PGN, source and (for
//! a PDU1 PGN) destination. `time` is the device's own clock and `mhs` a
//! status byte; neither is used. `data` is a whole message, fast-packets
//! already joined. The byte before `DLE ETX` is not read.
//!
//! This follows canboatjs's `readN2KActisense`, the reference for the
//! format: the message is framed by its length alone, with no DLE
//! stuffing undone, and the destination is taken from the CAN id (255 for
//! a PDU2 PGN) rather than from the `dst` byte. Unlike canboatjs, a
//! message split across reads is kept until the rest arrives, and a
//! malformed one costs a resync to the next `DLE STX D0`, not the rest of
//! the read.
//!
//! [`encode_frame`] writes a message as canboatjs's `encodeN2KActisense`
//! does: device time, status and the byte before `DLE ETX` all 0.
//! canboatjs never sends in this mode, so this is not yet proven on a
//! W2K-1.

use smallvec::SmallVec;

use super::common::{iso11783_compose, iso11783_decompose};
use crate::engine::RawFrame;

/// BST message ID of an N2K message.
pub const BST_N2K: u8 = 0xD0;

const DLE: u8 = 0x10;
const STX: u8 = 0x02;
const ETX: u8 = 0x03;

/// `DLE STX D0` and the two length bytes.
const PREFIX_LEN: usize = 5;
/// `dst`, `canid`, `time` and `mhs`.
const HEADER_LEN: usize = 10;
/// The byte before `DLE ETX`, and `DLE ETX`.
const TRAILER_LEN: usize = 3;
/// The smallest `len`: a message with no data.
const MIN_LEN: usize = HEADER_LEN + TRAILER_LEN;

/// The largest data a message can carry: `len` is 16 bits.
pub const MAX_DATA_LEN: usize = u16::MAX as usize - MIN_LEN;

/// `frame` as one BST `0xD0` message, or `None` when its data is longer
/// than [`MAX_DATA_LEN`].
pub fn encode_frame(frame: &RawFrame) -> Option<Vec<u8>> {
    if frame.data.len() > MAX_DATA_LEN {
        return None;
    }
    let canid = iso11783_compose(frame.prio, frame.pgn, frame.src, frame.dst);
    let mut m = Vec::with_capacity(PREFIX_LEN + MIN_LEN + frame.data.len());
    m.extend_from_slice(&[DLE, STX, BST_N2K]);
    m.extend_from_slice(&((MIN_LEN + frame.data.len()) as u16).to_le_bytes());
    m.push(frame.dst);
    m.extend_from_slice(&canid.to_le_bytes());
    m.extend_from_slice(&[0, 0, 0, 0, 0]);
    m.extend_from_slice(&frame.data);
    m.extend_from_slice(&[0, DLE, ETX]);
    Some(m)
}

/// Incremental decoder for a stream of BST `0xD0` messages. Feed it bytes
/// as they arrive with [`push_bytes`](Self::push_bytes); it returns each
/// message as a [`RawFrame`] once all of it is in.
#[derive(Debug, Default)]
pub struct ActisenseN2kDecoder {
    buf: Vec<u8>,
    /// Bytes thrown away while looking for the start of a message, or
    /// because a message was malformed.
    pub skipped: u64,
}

impl ActisenseN2kDecoder {
    pub fn new() -> Self {
        Self::default()
    }

    /// Add `bytes` and return every message they complete.
    pub fn push_bytes(&mut self, bytes: &[u8]) -> Vec<RawFrame> {
        self.buf.extend_from_slice(bytes);
        let mut frames = Vec::new();
        let mut pos = 0;
        while let Some(start) = find_start(&self.buf[pos..]) {
            self.skipped += start as u64;
            pos += start;
            let rest = &self.buf[pos..];
            if rest.len() < PREFIX_LEN {
                break;
            }
            let len = u16::from_le_bytes([rest[3], rest[4]]) as usize;
            if len < MIN_LEN {
                // Not a message after all: look again past this DLE.
                self.skipped += 1;
                pos += 1;
                continue;
            }
            let total = PREFIX_LEN + len;
            if rest.len() < total {
                break;
            }
            if rest[total - 2] != DLE || rest[total - 1] != ETX {
                self.skipped += 1;
                pos += 1;
                continue;
            }
            frames.push(to_frame(&rest[PREFIX_LEN..total - TRAILER_LEN]));
            pos += total;
        }
        if find_start(&self.buf[pos..]).is_none() {
            // Nothing to wait for, except perhaps a `DLE STX D0` cut short
            // at the very end.
            let keep = partial_start(&self.buf[pos..]);
            let drop = self.buf.len() - pos - keep;
            self.skipped += drop as u64;
            pos += drop;
        }
        self.buf.drain(..pos);
        frames
    }
}

/// The offset of the next `DLE STX D0` in `buf`.
fn find_start(buf: &[u8]) -> Option<usize> {
    buf.windows(3).position(|w| w == [DLE, STX, BST_N2K])
}

/// How many bytes at the end of `buf` could begin a `DLE STX D0`.
fn partial_start(buf: &[u8]) -> usize {
    match buf {
        [.., DLE, STX] => 2,
        [.., DLE] => 1,
        _ => 0,
    }
}

/// The frame in a message's `dst canid time mhs data`.
fn to_frame(body: &[u8]) -> RawFrame {
    let canid = u32::from_le_bytes([body[1], body[2], body[3], body[4]]) & 0x1fff_ffff;
    let (prio, pgn, src, dst) = iso11783_decompose(canid);
    RawFrame {
        timestamp: None,
        prio,
        pgn,
        src,
        dst,
        data: SmallVec::from_slice(&body[HEADER_LEN..]),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A message as canboatjs's `encodeN2KActisense` writes it.
    fn message(dst: u8, canid: u32, data: &[u8]) -> Vec<u8> {
        let mut m = vec![DLE, STX, BST_N2K];
        m.extend_from_slice(&((MIN_LEN + data.len()) as u16).to_le_bytes());
        m.push(dst);
        m.extend_from_slice(&canid.to_le_bytes());
        m.extend_from_slice(&0x1234_5678u32.to_le_bytes());
        m.push(0);
        m.extend_from_slice(data);
        m.extend_from_slice(&[0, DLE, ETX]);
        m
    }

    /// 129025 Position, Rapid Update from source 3, priority 2.
    const POSITION: u32 = 0x09F8_0103;
    const POSITION_DATA: [u8; 8] = [0x50, 0x9b, 0x3a, 0x1e, 0x2e, 0xd7, 0x0a, 0x03];

    #[test]
    fn decodes_a_message() {
        let frames = ActisenseN2kDecoder::new().push_bytes(&message(255, POSITION, &POSITION_DATA));
        assert_eq!(frames.len(), 1);
        let f = &frames[0];
        assert_eq!((f.prio, f.pgn, f.src, f.dst), (2, 129025, 3, 255));
        assert_eq!(f.data.as_slice(), &POSITION_DATA);
    }

    /// The destination of a PDU1 PGN is in the CAN id: 59904 ISO Request
    /// from 3 to 0x23.
    #[test]
    fn a_pdu1_destination_comes_from_the_can_id() {
        let frames =
            ActisenseN2kDecoder::new().push_bytes(&message(0x23, 0x18EA_2303, &[0x14, 0xf0, 0x01]));
        assert_eq!(
            (frames[0].pgn, frames[0].src, frames[0].dst),
            (59904, 3, 0x23)
        );
    }

    /// A whole fast-packet message comes in one BST message, and a DLE in
    /// the data is just data.
    #[test]
    fn a_long_message_and_a_dle_in_the_data() {
        let data: Vec<u8> = (0u8..=40).map(|b| b ^ 0x10).collect();
        let frames = ActisenseN2kDecoder::new().push_bytes(&message(255, 0x0DF8_0503, &data));
        assert_eq!(frames[0].pgn, 129029);
        assert_eq!(frames[0].data.as_slice(), data.as_slice());
    }

    /// Bytes arrive in any pieces: a message split across reads is kept.
    #[test]
    fn a_message_split_across_reads() {
        let mut stream = message(255, POSITION, &POSITION_DATA);
        stream.extend(message(255, POSITION, &POSITION_DATA));
        for cut in 0..stream.len() {
            let mut d = ActisenseN2kDecoder::new();
            let mut frames = d.push_bytes(&stream[..cut]);
            frames.extend(d.push_bytes(&stream[cut..]));
            assert_eq!(frames.len(), 2, "cut at {cut}");
            assert_eq!(d.skipped, 0, "cut at {cut}");
        }
    }

    /// Noise, and a message whose length does not end on `DLE ETX`, cost
    /// a resync to the next message, which still decodes.
    #[test]
    fn resyncs_after_noise_and_a_bad_message() {
        let mut bad = message(255, POSITION, &POSITION_DATA);
        let n = bad.len();
        bad[n - 1] = 0x00;
        let mut stream = vec![0xaa, 0x10, 0x55];
        stream.extend(&bad);
        stream.extend(message(255, 0x18EA_2303, &[0x14, 0xf0, 0x01]));
        let mut d = ActisenseN2kDecoder::new();
        let frames = d.push_bytes(&stream);
        assert_eq!(frames.len(), 1);
        assert_eq!(frames[0].pgn, 59904);
        assert!(d.skipped > 0);
    }

    /// What [`encode_frame`] writes decodes to the same frame, and matches
    /// canboatjs's `encodeN2KActisense` but for its device time.
    #[test]
    fn encode_round_trips() {
        for (canid, dst, data) in [
            (POSITION, 255u8, POSITION_DATA.to_vec()),
            (0x18EA_2303, 0x23, vec![0x14, 0xf0, 0x01]),
            (0x0DF8_0503, 255, (0u8..43).collect()),
        ] {
            let (prio, pgn, src, _) = iso11783_decompose(canid);
            let frame = RawFrame {
                timestamp: None,
                prio,
                pgn,
                src,
                dst,
                data: SmallVec::from_slice(&data),
            };
            let bytes = encode_frame(&frame).unwrap();
            let mut want = message(dst, canid, &data);
            want[10..14].fill(0);
            assert_eq!(bytes, want);
            let back = ActisenseN2kDecoder::new().push_bytes(&bytes);
            assert_eq!(back, vec![frame]);
        }
    }

    /// A stream with no message in it does not grow the buffer.
    #[test]
    fn noise_is_not_kept() {
        let mut d = ActisenseN2kDecoder::new();
        for _ in 0..100 {
            assert!(d.push_bytes(&[0x55; 1000]).is_empty());
        }
        assert!(d.buf.is_empty());
        assert_eq!(d.skipped, 100_000);
    }
}
