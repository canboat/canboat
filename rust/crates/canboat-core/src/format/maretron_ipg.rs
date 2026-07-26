// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Maretron IPG100 binary frame codec (sans-I/O).
//!
//! The IPG100 ships NMEA 2000 frames over TCP after a brief text-mode
//! handshake. Each binary frame on the wire is:
//!
//! ```text
//!   0xA5  F1  PF  PS  SA  LEN [LEN_HI]  <payload bytes>
//! ```
//!
//!   - **0xA5**: sync byte.
//!   - **F1**: must have bit 7 set; bits 6..4 = priority,
//!     bits 2..1 = message type (1 = single, 2 = fast, 3 = ext.),
//!     bit 0 = data-page.
//!   - **PF / PS**: ISO 11783 PDU specifier bytes.
//!   - **SA**: source address.
//!   - **LEN**: payload length. For `msg_type == 3` it's a 16-bit LE
//!     length spanning bytes 5 and 6; otherwise an 8-bit length in
//!     byte 5 only.
//!
//! Reference: `canboat/maretron-ipg/maretron-ipg.c` —
//! `parseMaretronFrame` and `buildMaretronFrame`.

use smallvec::SmallVec;

use crate::frame::{FASTPACKET_MAX_SIZE, RawFrame};

/// Sync byte that introduces every binary frame.
pub const FRAME_SYNC: u8 = 0xA5;
/// F1 must always have bit 7 set on a valid frame.
pub const F1_SYNC_BIT: u8 = 0x80;
/// IPG default TCP port (bus 0). Bus 1 uses :6553.
pub const DEFAULT_TCP_PORT: &str = "6543";

/// CONNECT message — `%s` is the password (empty string is valid).
pub const TX_CONNECT_MSG_FMT: &str = "CONNECT\t\"{password}\"\t\tMOBILE";
/// Move the session from text to binary mode after the handshake.
pub const TX_SET_MODE_BINARY_MSG: &str = "SET_MODE\tBINARY";

/// First-word strings the IPG sends in the text-mode handshake.
pub const RX_CONNECTED: &str = "CONNECTED";
pub const RX_NO: &str = "NO";
pub const RX_SERVER_VERSION: &str = "SERVER_VERSION";
pub const RX_INSTANCE_DATA: &str = "INSTANCE_DATA";
pub const RX_LICENSES_USED: &str = "LICENSES_USED";
pub const RX_DETAILED_LICENSES_USED: &str = "DETAILED_LICENSES_USED";

/// Prefix bytes the IPG can interleave with binary frames in stream
/// mode (video / ASCII pass-through). Both are followed by a NUL.
pub const VIDEO_PREFIX: u8 = 0x33; // '3'
pub const ASCII_PREFIX: u8 = 0x32; // '2'

/// What the parser reports when consuming a slice of bytes off the
/// wire. Bytes past the consumed prefix are still in the caller's
/// buffer.
#[derive(Debug)]
pub enum ParseOutcome {
    /// One complete binary frame, plus the number of bytes consumed.
    Frame {
        frame: MaretronFrame,
        consumed: usize,
    },
    /// One NUL-terminated text-mode line (handshake message), plus
    /// the byte count *including* the trailing NUL.
    Text { line: String, consumed: usize },
    /// Not enough bytes to decide — the caller should accumulate more.
    NeedMore,
    /// Bytes drop on the floor — out-of-sync. The caller should
    /// discard `consumed` bytes from the front of its buffer and try
    /// again.
    Drop { consumed: usize },
}

/// Decoded binary frame fields.
#[derive(Debug, Clone)]
pub struct MaretronFrame {
    pub pgn: u32,
    pub prio: u8,
    pub src: u8,
    pub dst: u8,
    pub msg_type: u8,
    pub payload: Vec<u8>,
}

impl MaretronFrame {
    /// Convert to a canboat [`RawFrame`] with the given timestamp.
    pub fn to_raw(&self, timestamp: Option<String>) -> RawFrame {
        let mut data: SmallVec<[u8; 8]> =
            SmallVec::with_capacity(self.payload.len().min(FASTPACKET_MAX_SIZE));
        let take = self.payload.len().min(FASTPACKET_MAX_SIZE);
        data.extend_from_slice(&self.payload[..take]);
        RawFrame {
            timestamp,
            prio: self.prio,
            pgn: self.pgn,
            src: self.src,
            dst: self.dst,
            data,
        }
    }
}

/// Parse one event out of the front of `buf`. The caller is
/// responsible for accumulating bytes between calls — pass the same
/// `state` value back to track whether the session is still in the
/// text-mode handshake.
pub fn parse(buf: &[u8], state: SessionState) -> ParseOutcome {
    if buf.is_empty() {
        return ParseOutcome::NeedMore;
    }
    let first = buf[0];

    if state == SessionState::AwaitHandshake {
        return parse_nul_terminated_text(buf);
    }

    if first == FRAME_SYNC {
        return parse_binary_frame(buf);
    }
    if first == VIDEO_PREFIX || first == ASCII_PREFIX {
        return match buf.iter().position(|&b| b == 0) {
            Some(idx) => ParseOutcome::Drop { consumed: idx + 1 },
            None => ParseOutcome::NeedMore,
        };
    }
    if first & 0x80 == 0 {
        // Likely another text-mode handshake message — IPG can re-send
        // server-version etc. mid-stream.
        return parse_nul_terminated_text(buf);
    }
    // Out-of-sync — drop one byte and let the caller try again.
    ParseOutcome::Drop { consumed: 1 }
}

/// Whether the session is still negotiating in text mode, or has
/// flipped to binary frames.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SessionState {
    AwaitHandshake,
    Streaming,
}

fn parse_nul_terminated_text(buf: &[u8]) -> ParseOutcome {
    let Some(nul_idx) = buf.iter().position(|&b| b == 0) else {
        return ParseOutcome::NeedMore;
    };
    let line = String::from_utf8_lossy(&buf[..nul_idx]).into_owned();
    ParseOutcome::Text {
        line,
        consumed: nul_idx + 1,
    }
}

fn parse_binary_frame(buf: &[u8]) -> ParseOutcome {
    // sync + F1 + PF + PS + SA + LEN — minimum 6 bytes.
    if buf.len() < 6 {
        return ParseOutcome::NeedMore;
    }
    let f1 = buf[1];
    if f1 & F1_SYNC_BIT == 0 {
        return ParseOutcome::Drop { consumed: 1 };
    }
    let pf = buf[2];
    let ps = buf[3];
    let sa = buf[4];
    let prio = (f1 >> 4) & 0x07;
    let msg_type = (f1 >> 1) & 0x03;
    let dp = f1 & 0x01;

    let (payload_start, payload_len) = if msg_type == 3 {
        if buf.len() < 7 {
            return ParseOutcome::NeedMore;
        }
        let len = (buf[5] as u16) | ((buf[6] as u16) << 8);
        (7usize, len as usize)
    } else {
        (6usize, buf[5] as usize)
    };

    let total = payload_start + payload_len;
    if buf.len() < total {
        return ParseOutcome::NeedMore;
    }

    let (pgn, dst) = if pf < 0xF0 {
        ((dp as u32) << 16 | (pf as u32) << 8, ps)
    } else {
        ((dp as u32) << 16 | (pf as u32) << 8 | ps as u32, 0xFFu8)
    };

    let payload = buf[payload_start..total].to_vec();
    ParseOutcome::Frame {
        frame: MaretronFrame {
            pgn,
            prio,
            src: sa,
            dst,
            msg_type,
            payload,
        },
        consumed: total,
    }
}

/// Encode an outgoing N2K message as a Maretron binary frame ready
/// for the wire. `msg_type` is 1 for single-frame messages
/// (`payload_len <= 8`) and 2 for fast-packet payloads; SA on the
/// wire is always `0xFF` (the IPG substitutes its claimed source
/// address). Mirrors canboat's `buildMaretronFrame`.
pub fn build_frame(pgn: u32, prio: u8, dst: u8, payload: &[u8]) -> Option<Vec<u8>> {
    if payload.len() > FASTPACKET_MAX_SIZE {
        return None;
    }
    let msg_type: u8 = if payload.len() <= 8 { 1 } else { 2 };
    let dp = ((pgn >> 16) & 0x01) as u8;
    let pf = ((pgn >> 8) & 0xFF) as u8;
    let ps = if pf < 0xF0 { dst } else { (pgn & 0xFF) as u8 };
    let mut out = Vec::with_capacity(6 + payload.len());
    out.push(FRAME_SYNC);
    out.push(F1_SYNC_BIT | ((prio & 0x07) << 4) | ((msg_type & 0x03) << 1) | (dp & 0x01));
    out.push(pf);
    out.push(ps);
    out.push(0xFF);
    out.push(payload.len() as u8);
    out.extend_from_slice(payload);
    Some(out)
}

/// Format the CONNECT handshake string with the given password. The
/// trailing NUL is appended for the caller (canboat sends a NUL-
/// terminated frame).
pub fn build_connect(password: &str) -> Vec<u8> {
    let mut s = format!("CONNECT\t\"{}\"\t\tMOBILE", password).into_bytes();
    s.push(0);
    s
}

/// `SET_MODE\tBINARY\0` — sent right after we see `CONNECTED`.
pub fn build_set_mode_binary() -> Vec<u8> {
    let mut s = TX_SET_MODE_BINARY_MSG.as_bytes().to_vec();
    s.push(0);
    s
}

/// Ensure `url` carries an explicit port; default to [`DEFAULT_TCP_PORT`]
/// (6543) when none is set. Strips the leading `tcp://` if present.
pub fn ensure_default_port(url: &str) -> String {
    let body = url.strip_prefix("tcp://").unwrap_or(url);
    // Bracketed IPv6 already supplies its own port-delimiter context.
    if body.starts_with('[') || body.contains(':') {
        body.to_string()
    } else {
        format!("{body}:{DEFAULT_TCP_PORT}")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_single_frame() {
        // sync=A5, F1=11000100 (prio=4, msg=2, dp=0), PF=0xFD, PS=0x02,
        // SA=0x16, LEN=4, payload AA BB CC DD.
        let buf = [
            0xA5, 0b11000100, 0xFD, 0x02, 0x16, 0x04, 0xAA, 0xBB, 0xCC, 0xDD,
        ];
        let out = parse(&buf, SessionState::Streaming);
        match out {
            ParseOutcome::Frame { frame, consumed } => {
                assert_eq!(consumed, 10);
                assert_eq!(frame.pgn, 0xFD02);
                assert_eq!(frame.prio, 4);
                assert_eq!(frame.src, 0x16);
                assert_eq!(frame.dst, 0xFF);
                assert_eq!(frame.payload, vec![0xAA, 0xBB, 0xCC, 0xDD]);
            }
            other => panic!("expected Frame, got {other:?}"),
        }
    }

    #[test]
    fn need_more_on_partial_frame() {
        let buf = [0xA5, 0xC4, 0xFD, 0x02, 0x16, 0x04, 0xAA, 0xBB];
        assert!(matches!(
            parse(&buf, SessionState::Streaming),
            ParseOutcome::NeedMore
        ));
    }

    #[test]
    fn handshake_message_is_text() {
        let buf = b"CONNECTED\tABC123\0extra";
        let out = parse(buf, SessionState::AwaitHandshake);
        match out {
            ParseOutcome::Text { line, consumed } => {
                assert_eq!(line, "CONNECTED\tABC123");
                assert_eq!(consumed, 17);
            }
            other => panic!("expected Text, got {other:?}"),
        }
    }

    #[test]
    fn build_then_parse_roundtrip() {
        let frame = build_frame(0xFD02, 3, 0xFF, &[1, 2, 3, 4]).unwrap();
        match parse(&frame, SessionState::Streaming) {
            ParseOutcome::Frame { frame, .. } => {
                assert_eq!(frame.pgn, 0xFD02);
                assert_eq!(frame.prio, 3);
                assert_eq!(frame.payload, vec![1, 2, 3, 4]);
                assert_eq!(frame.msg_type, 1);
            }
            o => panic!("expected Frame, got {o:?}"),
        }
    }

    #[test]
    fn ensure_default_port_works() {
        assert_eq!(ensure_default_port("tcp://ipg.local"), "ipg.local:6543");
        assert_eq!(
            ensure_default_port("tcp://ipg.local:6553"),
            "ipg.local:6553"
        );
        assert_eq!(ensure_default_port("192.168.1.100"), "192.168.1.100:6543");
        assert_eq!(ensure_default_port("[fe80::1]:6543"), "[fe80::1]:6543");
    }
}
