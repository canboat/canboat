// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Actisense BST messages, whatever their type, and the BDTP framing they
//! travel in. Specified in the Actisense SDK:
//! [BST](https://github.com/Actisense/SDK/blob/main/docs/DataFormats/Binary/BST.md),
//! [BDTP](https://github.com/Actisense/SDK/blob/main/docs/DataProtocols/bdtp-protocol.md).
//!
//! A BST message is its ID, a length, its data and a zero-sum checksum.
//! IDs `D0`–`DF` ("BST type 2") have a 16-bit length that counts the whole
//! message but the checksum; the others an 8-bit length of the data alone.
//! On a link a message is framed `DLE STX … DLE ETX`, with every `DLE`
//! inside doubled.
//!
//! [`to_raw_frame`] turns the messages that carry NMEA 2000 traffic into
//! frames: BST-93 (an NGT-1's received message), BST-94 (a message for an
//! NGT-1 to send), BST-95 (a raw CAN frame) and BST-D0 (a W2K-1's
//! reassembled message).

use smallvec::SmallVec;

use super::bst_d0::{self, BST_D0};
use super::common::iso11783_decompose;
use super::ngt1::{N2K_MSG_RECEIVED, N2K_MSG_SEND, NgtMessage};
use crate::engine::RawFrame;

const DLE: u8 = 0x10;
const STX: u8 = 0x02;
const ETX: u8 = 0x03;

/// BST message ID of a BST-95 raw CAN frame.
pub const BST_95: u8 = 0x95;

/// Most bytes collected for one message before it is given up on: a type 2
/// message is at most a 16-bit length long, but none that canboat reads is
/// near that.
const MAX_COLLECT: usize = 2048;

/// A BDTP framer: feed it bytes, and it hands back each message it finds,
/// unframed and un-stuffed, checksum included. It checks nothing but the
/// framing; [`message_body`] checks the length and checksum.
#[derive(Debug, Default)]
pub struct BdtpDecoder {
    state: State,
    buf: Vec<u8>,
    /// Frames given up on: a bad escape, or too long.
    pub dropped: u64,
}

#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
enum State {
    #[default]
    Idle,
    IdleDle,
    InFrame,
    InFrameDle,
}

impl BdtpDecoder {
    /// Feed one byte; a message comes out at its `DLE ETX`.
    pub fn push_byte(&mut self, b: u8) -> Option<Vec<u8>> {
        match (self.state, b) {
            (State::Idle, DLE) => self.state = State::IdleDle,
            (State::Idle, _) => {}
            (State::IdleDle, STX) | (State::InFrameDle, STX) => {
                if self.state == State::InFrameDle {
                    self.dropped += 1;
                }
                self.buf.clear();
                self.state = State::InFrame;
            }
            (State::IdleDle, DLE) => {}
            (State::IdleDle, _) => self.state = State::Idle,
            (State::InFrame, DLE) => self.state = State::InFrameDle,
            (State::InFrame, _) => self.collect(b),
            (State::InFrameDle, DLE) => {
                self.state = State::InFrame;
                self.collect(DLE);
            }
            (State::InFrameDle, ETX) => {
                self.state = State::Idle;
                return Some(std::mem::take(&mut self.buf));
            }
            (State::InFrameDle, _) => {
                self.dropped += 1;
                self.buf.clear();
                self.state = State::Idle;
            }
        }
        None
    }

    fn collect(&mut self, b: u8) {
        if self.buf.len() >= MAX_COLLECT {
            self.dropped += 1;
            self.buf.clear();
            self.state = State::Idle;
        } else {
            self.buf.push(b);
        }
    }
}

/// A BST message's length: `(header bytes, total bytes without the
/// checksum)`, or `None` when `m` is too short to say.
fn message_len(m: &[u8]) -> Option<(usize, usize)> {
    match m.first()? {
        0xD0..=0xDF => Some((3, u16::from_le_bytes([*m.get(1)?, *m.get(2)?]) as usize)),
        _ => Some((2, 2 + *m.get(1)? as usize)),
    }
}

/// The ID and data of the unframed BST message `m`, or `None` when its
/// length does not match or its checksum is wrong. A message without its
/// checksum is accepted when `checksum_optional`: an EBL file's
/// BSTRawFrame carries some that way.
pub fn message_body(m: &[u8], checksum_optional: bool) -> Option<(u8, &[u8])> {
    let (header, len) = message_len(m)?;
    if len < header {
        return None;
    }
    let with_checksum = m.len() == len + 1 && m.iter().fold(0u8, |s, &b| s.wrapping_add(b)) == 0;
    let without = checksum_optional && m.len() == len;
    (with_checksum || without).then(|| (m[0], &m[header..len]))
}

/// The NMEA 2000 frame the unframed BST message `m` carries, if it is a
/// well-formed BST-93, BST-94, BST-95 or BST-D0 message. A BST-94 message
/// has no source; it comes out as source 0, as canboat C has it. Its timestamp is left
/// `None`: these messages carry only the device's own clock.
pub fn to_raw_frame(m: &[u8], checksum_optional: bool) -> Option<RawFrame> {
    let (id, data) = message_body(m, checksum_optional)?;
    match id {
        N2K_MSG_RECEIVED => {
            let mut frame = NgtMessage {
                command: id,
                payload: data.to_vec(),
            }
            .to_raw_frame()?;
            frame.timestamp = None;
            Some(frame)
        }
        N2K_MSG_SEND => bst94_frame(data),
        BST_95 => bst95_frame(data),
        BST_D0 => {
            let mut whole = m[..message_len(m)?.1].to_vec();
            let sum = whole.iter().fold(0u8, |s, &b| s.wrapping_add(b));
            whole.push(0u8.wrapping_sub(sum));
            bst_d0::to_frame(&whole)
        }
        _ => None,
    }
}

/// A BST-94 message's frame
/// ([SDK](https://github.com/Actisense/SDK/blob/main/docs/DataFormats/Binary/bst-detail/BST-94-NGT.md)):
/// priority, PGN (3 bytes), destination, data length, data.
fn bst94_frame(d: &[u8]) -> Option<RawFrame> {
    let (&[prio, p0, p1, p2, dst, len], data) = d.split_first_chunk::<6>()?;
    (data.len() == len as usize).then(|| RawFrame {
        timestamp: None,
        prio,
        pgn: u32::from_le_bytes([p0, p1, p2, 0]),
        src: 0,
        dst,
        data: SmallVec::from_slice(data),
    })
}

/// A BST-95 message's frame
/// ([SDK](https://github.com/Actisense/SDK/blob/main/docs/DataFormats/Binary/bst-detail/BST-95-can-frame.md)):
/// a 16-bit timestamp, then the CAN identifier as source, PDU specific,
/// PDU format and a byte with the data page (bits 0–1), the priority (bits
/// 2–4), the timestamp resolution (bits 5–6) and the direction (bit 7),
/// then up to 8 bytes of data.
fn bst95_frame(d: &[u8]) -> Option<RawFrame> {
    if d.len() < 6 || d.len() > 14 {
        return None;
    }
    let canid = u32::from(d[2])
        | u32::from(d[3]) << 8
        | u32::from(d[4]) << 16
        | u32::from(d[5] & 0x1f) << 24;
    let (prio, pgn, src, dst) = iso11783_decompose(canid);
    Some(RawFrame {
        timestamp: None,
        prio,
        pgn,
        src,
        dst,
        data: SmallVec::from_slice(&d[6..]),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn checksummed(m: &[u8]) -> Vec<u8> {
        let mut v = m.to_vec();
        let sum = v.iter().fold(0u8, |s, &b| s.wrapping_add(b));
        v.push(0u8.wrapping_sub(sum));
        v
    }

    /// The BST-95 message in `samples/pgn1.ebl` (a BSTRawFrame, without a
    /// checksum): PGN 129025 from source 0, priority 2.
    #[test]
    fn bst95_carries_a_can_frame() {
        let m = [
            0x95, 0x0e, 0x28, 0x9a, 0x00, 0x01, 0xf8, 0x09, 0x3d, 0x0d, 0xb3, 0x22, 0x48, 0x32,
            0x59, 0x0d,
        ];
        assert!(to_raw_frame(&m, false).is_none(), "no checksum");
        let f = to_raw_frame(&m, true).unwrap();
        assert_eq!((f.prio, f.pgn, f.src, f.dst), (2, 129025, 0, 255));
        assert_eq!(f.data.as_slice(), &m[8..]);
        let f = to_raw_frame(&checksummed(&m), false).unwrap();
        assert_eq!(f.pgn, 129025);
    }

    /// A PDU1 PGN's PDU specific byte is the destination.
    #[test]
    fn bst95_pdu1_has_a_destination() {
        // ISO Request (59904) from 3 to 0x23, priority 6, 3 bytes.
        let m = checksummed(&[0x95, 9, 0, 0, 0x03, 0x23, 0xea, 6 << 2, 0x14, 0xf0, 0x01]);
        let f = to_raw_frame(&m, false).unwrap();
        assert_eq!((f.prio, f.pgn, f.src, f.dst), (6, 59904, 3, 0x23));
    }

    /// The SDK's worked BST-93 message from its EBL example, with the
    /// checksum corrected: the SDK has `bc`, which does not zero the sum.
    #[test]
    fn bst93_carries_a_message() {
        let m = [
            0x93, 0x0c, 0x02, 0x00, 0xee, 0x01, 0xff, 0xee, 0, 0, 0, 0, 0x01, 0x42, 0x40,
        ];
        let f = to_raw_frame(&m, false).unwrap();
        assert_eq!((f.prio, f.pgn, f.src, f.dst), (2, 126464, 0xee, 0xff));
        assert_eq!(f.data.as_slice(), &[0x42]);
        assert!(f.timestamp.is_none());
    }

    /// A BST-94 message from `samples/pgn127504.ebl`: PGN 127503 for the
    /// gateway to send.
    #[test]
    fn bst94_carries_a_message_to_send() {
        let mut m = vec![0x94, 0x1a, 0x06, 0x0f, 0xf2, 0x01, 0xff, 0x14];
        m.extend_from_slice(&[0; 20]);
        let f = to_raw_frame(&checksummed(&m), false).unwrap();
        assert_eq!(
            (f.prio, f.pgn, f.src, f.dst, f.data.len()),
            (6, 127503, 0, 255, 20)
        );
    }

    #[test]
    fn a_bad_checksum_or_length_is_refused() {
        let mut m = checksummed(&[0x95, 7, 0, 0, 1, 2, 0xf8, 9, 0xaa]);
        assert!(to_raw_frame(&m, false).is_some());
        m[8] ^= 1;
        assert!(to_raw_frame(&m, false).is_none());
        let short = checksummed(&[0x95, 9, 0, 0, 1, 2, 0xf8, 9, 0xaa]);
        assert!(to_raw_frame(&short, true).is_none());
    }

    #[test]
    fn bdtp_unstuffs_and_resyncs() {
        let mut d = BdtpDecoder::default();
        let wire = [
            0x55, DLE, STX, 1, DLE, DLE, 2, DLE, ETX, DLE, STX, 9, DLE, 0x44, DLE, STX, 3, DLE, ETX,
        ];
        let got: Vec<Vec<u8>> = wire.iter().filter_map(|&b| d.push_byte(b)).collect();
        assert_eq!(got, [vec![1, DLE, 2], vec![3]]);
        assert_eq!(d.dropped, 1);
    }
}
