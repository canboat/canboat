// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Canboat PLAIN / FAST line format.
//!
//! ```text
//! <timestamp>,<prio>,<pgn>,<src>,<dst>,<len>,<hex0>,<hex1>,...,<hexN>
//! ```
//!
//! `len` is the payload byte count (`0..=223`). When `len <= 8` the
//! line is conventionally called PLAIN (a single CAN frame); otherwise
//! it is FAST (a pre-coalesced fast-packet payload). Both share the
//! same syntax — this module parses and writes both.

use std::fmt;

use smallvec::SmallVec;

use crate::frame::{RAWFRAME_MAX_SIZE, RawFrame};

/// Byte → hex nibble lookup. `0xff` sentinel for non-hex bytes.
/// Used by the PLAIN/FAST payload decoder to skip the cost of
/// `u8::from_str_radix` + tokenisation per byte.
const HEX_NIBBLE: [u8; 256] = {
    let mut t = [0xffu8; 256];
    let mut i = 0u8;
    while i < 10 {
        t[(b'0' + i) as usize] = i;
        i += 1;
    }
    let mut i = 0u8;
    while i < 6 {
        t[(b'a' + i) as usize] = 10 + i;
        t[(b'A' + i) as usize] = 10 + i;
        i += 1;
    }
    t
};

#[derive(Debug, thiserror::Error, PartialEq, Eq)]
pub enum ParseError {
    #[error("line is empty")]
    Empty,
    #[error("missing timestamp separator")]
    MissingTimestamp,
    #[error("expected {expected} comma-separated header fields, found {found}")]
    BadHeader { expected: usize, found: usize },
    #[error("malformed integer in field {field}: {value:?}")]
    BadInteger {
        field: &'static str,
        value: String,
        /// Byte offset within the original line where the field
        /// started. `None` when the producing parser doesn't track
        /// offsets (the non-PLAIN format parsers — airmar, chetco,
        /// ydwg02, etc. — use `str::Split` and don't bother).
        offset: Option<usize>,
    },
    #[error("declared length {len} exceeds max {max}", max = RAWFRAME_MAX_SIZE)]
    LengthTooLarge { len: usize },
    #[error("expected {expected} hex bytes, found {found}")]
    BadPayloadCount { expected: usize, found: usize },
    #[error("malformed hex byte {value:?} at index {index}")]
    BadHexByte {
        index: usize,
        value: String,
        offset: Option<usize>,
    },
    #[error("line ends after the length field; no payload section")]
    MissingPayload,
}

impl ParseError {
    /// Byte offset within the line where the parse failed, when the
    /// producing parser tracks it. Callers can use this to render a
    /// caret-style pointer beneath the offending line.
    pub fn byte_offset(&self) -> Option<usize> {
        match self {
            ParseError::BadInteger { offset, .. } | ParseError::BadHexByte { offset, .. } => {
                *offset
            }
            _ => None,
        }
    }
}

/// Parse one PLAIN/FAST line.
///
/// The input may have a trailing newline; it is ignored. The timestamp
/// is preserved verbatim (whatever appears before the first comma).
pub fn parse_line(line: &str) -> Result<RawFrame, ParseError> {
    let line = line.trim_end_matches(['\r', '\n']);
    if line.is_empty() {
        return Err(ParseError::Empty);
    }

    // Split off the timestamp (everything up to the first comma).
    let (ts, rest) = line.split_once(',').ok_or(ParseError::MissingTimestamp)?;
    let timestamp = if ts.is_empty() {
        None
    } else {
        Some(ts.to_string())
    };

    // Byte offset of `rest` within the original `line` — used to
    // translate `cursor` (which is relative to `rest`) into a
    // line-relative offset on errors.
    let rest_base = ts.len() + 1;

    // Header: prio, pgn, src, dst, len. Parse the five tokens by
    // walking byte-by-byte rather than using `str::Split` so we can
    // recover the byte offset of the payload start without paying
    // for an extra slice/iterator chain.
    let rest_bytes = rest.as_bytes();
    let mut cursor = 0usize;
    let prio = take_uint_until_comma::<u8>(rest_bytes, &mut cursor, rest_base, "prio")?;
    let pgn = take_uint_until_comma::<u32>(rest_bytes, &mut cursor, rest_base, "pgn")?;
    let src = take_uint_until_comma::<u8>(rest_bytes, &mut cursor, rest_base, "src")?;
    let dst = take_uint_until_comma::<u8>(rest_bytes, &mut cursor, rest_base, "dst")?;
    let len = take_uint_until_comma::<usize>(rest_bytes, &mut cursor, rest_base, "len")?;

    if len > RAWFRAME_MAX_SIZE {
        return Err(ParseError::LengthTooLarge { len });
    }

    // canboat requires a sixth comma — the one terminating `len` and
    // opening the payload — whatever `len` itself says
    // (`findOccurrence(p, ',', 6)` in `parseRawFormatFast`,
    // common/parse.c). Captures contain lines that stop right after the
    // length, `2020-08-22T13:52:36.981Z,0,16712965,255,0,0`, and the C
    // rejects them outright with "cannot find sixth comma". Accepting
    // them produced a zero-byte frame that reached the decoder and came
    // out as a fields-less record for a PGN that does not exist.
    if cursor == 0 || rest_bytes[cursor - 1] != b',' {
        return Err(ParseError::MissingPayload);
    }

    // Payload: exactly `len` hex bytes, comma-separated. Fast path:
    // walk the remaining bytes ourselves, decoding two hex nibbles
    // per byte via a precomputed table. Beats
    // `from_str_radix(tok.trim(), 16)` per byte by roughly 3× —
    // canboat samples spend the largest chunk of their parse time
    // here.
    let payload_start = cursor;
    let payload_bytes = &rest_bytes[cursor..];
    let mut data: SmallVec<[u8; 8]> = SmallVec::with_capacity(len);
    let mut i = 0usize;
    let mut decoded = 0usize;
    while decoded < len && i < payload_bytes.len() {
        // Tolerate leading whitespace inside a token (canboat samples
        // occasionally use ` 04` style).
        while i < payload_bytes.len() && payload_bytes[i] == b' ' {
            i += 1;
        }
        if i + 2 > payload_bytes.len() {
            return Err(ParseError::BadHexByte {
                index: decoded,
                value: String::from_utf8_lossy(&payload_bytes[i..]).into_owned(),
                offset: Some(rest_base + payload_start + i),
            });
        }
        let hi = HEX_NIBBLE[payload_bytes[i] as usize];
        let lo = HEX_NIBBLE[payload_bytes[i + 1] as usize];
        if hi == 0xff || lo == 0xff {
            return Err(ParseError::BadHexByte {
                index: decoded,
                value: String::from_utf8_lossy(&payload_bytes[i..i + 2]).into_owned(),
                offset: Some(rest_base + payload_start + i),
            });
        }
        data.push((hi << 4) | lo);
        decoded += 1;
        i += 2;
        // Skip the separator. Trailing whitespace allowed for canboat
        // samples like `... 03 ,04`.
        while i < payload_bytes.len() && payload_bytes[i] == b' ' {
            i += 1;
        }
        if i < payload_bytes.len() && payload_bytes[i] == b',' {
            i += 1;
        }
    }

    if data.len() != len {
        return Err(ParseError::BadPayloadCount {
            expected: len,
            found: data.len(),
        });
    }

    Ok(RawFrame {
        timestamp,
        prio,
        pgn,
        src,
        dst,
        data,
    })
}

/// Read a decimal unsigned integer up to the next comma in `bytes`,
/// advancing `cursor` past the integer and the comma. Skips a
/// leading space, matching the canboat sample style `01, 02`.
///
/// `base` is the offset of `bytes[0]` within the original line, so
/// any returned `BadInteger` carries a *line-relative* byte offset
/// for caret-style error rendering.
fn take_uint_until_comma<T>(
    bytes: &[u8],
    cursor: &mut usize,
    base: usize,
    field: &'static str,
) -> Result<T, ParseError>
where
    T: TryFrom<u64>,
{
    while *cursor < bytes.len() && bytes[*cursor] == b' ' {
        *cursor += 1;
    }
    let start = *cursor;
    let mut acc: u64 = 0;
    let mut saw_digit = false;
    while *cursor < bytes.len() {
        let b = bytes[*cursor];
        if b.is_ascii_digit() {
            acc = acc * 10 + (b - b'0') as u64;
            saw_digit = true;
            *cursor += 1;
            continue;
        }
        break;
    }
    if !saw_digit {
        return Err(ParseError::BadInteger {
            field,
            value: String::from_utf8_lossy(&bytes[start..*cursor]).into_owned(),
            offset: Some(base + start),
        });
    }
    // Skip trailing whitespace then the comma.
    while *cursor < bytes.len() && bytes[*cursor] == b' ' {
        *cursor += 1;
    }
    if *cursor < bytes.len() && bytes[*cursor] == b',' {
        *cursor += 1;
    }
    T::try_from(acc).map_err(|_| ParseError::BadInteger {
        field,
        value: acc.to_string(),
        offset: Some(base + start),
    })
}

/// Write one PLAIN/FAST line to `w`. The timestamp is canonicalised via
/// [`normalize_timestamp`](super::normalize_timestamp) (empty if `None`)
/// so a comma-millis value never corrupts the comma-delimited line. Hex
/// bytes are lowercase, two digits each.
///
/// The output never includes a trailing newline; the caller decides.
pub fn write_line<W: fmt::Write>(w: &mut W, frame: &RawFrame) -> fmt::Result {
    if let Some(ts) = &frame.timestamp {
        w.write_str(&super::normalize_timestamp(ts))?;
    }
    write!(
        w,
        ",{prio},{pgn},{src},{dst},{len}",
        prio = frame.prio,
        pgn = frame.pgn,
        src = frame.src,
        dst = frame.dst,
        len = frame.data.len(),
    )?;
    for b in &frame.data {
        write!(w, ",{:02x}", b)?;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_plain_8byte_frame() {
        let line = "2011-04-25-06:25:03.603,3,129029,36,255,8,e6,f1,3a,80,9c,c6,0d,b3";
        let f = parse_line(line).unwrap();
        assert_eq!(f.timestamp.as_deref(), Some("2011-04-25-06:25:03.603"));
        assert_eq!(f.prio, 3);
        assert_eq!(f.pgn, 129029);
        assert_eq!(f.src, 36);
        assert_eq!(f.dst, 255);
        assert_eq!(
            &f.data[..],
            &[0xe6, 0xf1, 0x3a, 0x80, 0x9c, 0xc6, 0x0d, 0xb3]
        );
    }

    #[test]
    fn parses_fast_43byte_payload() {
        let mut line = String::from("2011-04-25-06:25:03.603,3,129029,36,255,43");
        for i in 0..43 {
            line.push_str(&format!(",{:02x}", i as u8));
        }
        let f = parse_line(&line).unwrap();
        assert_eq!(f.data.len(), 43);
        assert_eq!(f.data[0], 0);
        assert_eq!(f.data[42], 42);
    }

    #[test]
    fn writes_round_trip() {
        // A canonical timestamp round-trips unchanged.
        let frame = RawFrame::new(
            Some("2011-04-25T06:25:03.603Z".into()),
            3,
            129029,
            36,
            255,
            vec![0xe6, 0xf1, 0x3a, 0x80, 0x9c, 0xc6, 0x0d, 0xb3],
        );
        let mut out = String::new();
        write_line(&mut out, &frame).unwrap();
        let again = parse_line(&out).unwrap();
        assert_eq!(again, frame);
    }

    #[test]
    fn write_canonicalises_classic_timestamp() {
        // The classic canboat log form (dash before the time, no `Z`) is
        // rewritten to ISO-8601 UTC on emit.
        let frame = RawFrame::new(
            Some("2011-04-25-06:25:03.603".into()),
            3,
            129029,
            36,
            255,
            vec![0xe6, 0xf1, 0x3a, 0x80, 0x9c, 0xc6, 0x0d, 0xb3],
        );
        let mut out = String::new();
        write_line(&mut out, &frame).unwrap();
        assert!(out.starts_with("2011-04-25T06:25:03.603Z,"), "got {out}");
    }

    #[test]
    fn rejects_empty_line() {
        assert!(matches!(parse_line(""), Err(ParseError::Empty)));
    }

    #[test]
    fn rejects_oversized_len() {
        // RAWFRAME_MAX_SIZE is 255 * 7 = 1785 (ISO-TP maximum).
        let line = "ts,3,129029,36,255,1786";
        assert!(matches!(
            parse_line(line),
            Err(ParseError::LengthTooLarge { .. })
        ));
        // 250 declared bytes is in range since ISO-TP support. The line
        // stops after the length, so it is rejected for having no
        // payload section at all — the C's "cannot find sixth comma" —
        // before the byte count is ever counted.
        let line = "ts,3,129029,36,255,250";
        assert!(matches!(parse_line(line), Err(ParseError::MissingPayload)));
    }

    #[test]
    fn rejects_a_line_that_stops_after_the_length() {
        // Real capture lines (samples/susteranna2020.raw and friends)
        // that carry no data at all. canboat rejects them outright;
        // accepting them yielded a zero-byte frame that decoded into a
        // fields-less record for a PGN that does not exist.
        let line = "2020-08-22T13:52:36.981Z,0,16712965,255,0,0";
        assert!(matches!(parse_line(line), Err(ParseError::MissingPayload)));
        // A declared-zero-length line *with* the payload comma is still
        // accepted, matching the C's parser.
        let ok = parse_line("2020-08-22T13:52:36.981Z,0,127250,255,0,0,").expect("parses");
        assert!(ok.data.is_empty());
    }

    #[test]
    fn rejects_short_payload() {
        // Declared 8 bytes, only 3 supplied.
        let line = "ts,3,129029,36,255,8,01,02,03";
        assert!(matches!(
            parse_line(line),
            Err(ParseError::BadPayloadCount {
                expected: 8,
                found: 3
            })
        ));
    }

    #[test]
    fn empty_timestamp_field() {
        let line = ",3,129029,36,255,1,ff";
        let f = parse_line(line).unwrap();
        assert!(f.timestamp.is_none());
        assert_eq!(&f.data[..], &[0xff]);
    }
}
