// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Actisense N2K ASCII line format.
//!
//! Example line:
//!
//! ```text
//!   A173321.107 23FF7 1F119 01 02 ...
//! ```
//!
//! Layout (whitespace-separated):
//!   1. `A<HHMMSS.mmm>` — timestamp (UTC time-of-day; no date).
//!   2. `<SDP>` in hex — encoded as `(src << 12) | (dst << 4) | prio`.
//!   3. `<PGN>` in hex.
//!   4. Subsequent tokens are hex data bytes.
//!
//! Mirrors `parseRawFormatActisenseN2KAscii` in canboat/common/parse.c.

use std::fmt;

use smallvec::SmallVec;

use crate::format::plain::ParseError;
use crate::format::timestamp::time_of_day;
use crate::frame::RawFrame;

/// Encode `frame` as an Actisense N2K-ASCII line (no trailing newline):
/// `A<HHMMSS.mmm> <SDP:X> <PGN:X> <contiguous upper-case hex>`, where
/// `SDP = (src << 12) | (dst << 4) | prio`. The inverse of [`parse_line`];
/// data bytes are packed as one hex run, as canboat emits them.
pub fn write_line<W: fmt::Write>(w: &mut W, frame: &RawFrame) -> fmt::Result {
    // Actisense's time token has no colons: HHMMSS.mmm.
    let mut tod = time_of_day(frame.timestamp.as_deref());
    tod.retain(|c| c != ':');
    let sdp = ((frame.src as u32) << 12) | ((frame.dst as u32) << 4) | (frame.prio as u32);
    write!(w, "A{tod} {sdp:X} {pgn:X}", pgn = frame.pgn)?;
    if !frame.data.is_empty() {
        w.write_char(' ')?;
        for b in &frame.data {
            write!(w, "{b:02X}")?;
        }
    }
    Ok(())
}

pub fn parse_line(line: &str) -> Result<RawFrame, ParseError> {
    let line = line.trim_end_matches(['\r', '\n']);
    if line.is_empty() {
        return Err(ParseError::Empty);
    }
    let mut toks = line.split_whitespace();

    // Token 1: A + 6-digit time, optionally with .mmm fraction.
    let ts_tok = toks.next().ok_or(ParseError::MissingTimestamp)?;
    if !ts_tok.starts_with('A') || ts_tok.len() < 7 {
        return Err(ParseError::BadInteger {
            field: "timestamp",
            value: ts_tok.to_string(),
            offset: None,
        });
    }
    let timestamp = format_actisense_timestamp(&ts_tok[1..])?;

    // Token 2: SDP packed hex.
    let sdp_tok = toks.next().ok_or(ParseError::BadHeader {
        expected: 3,
        found: 1,
    })?;
    let sdp = u32::from_str_radix(sdp_tok, 16).map_err(|_| ParseError::BadInteger {
        field: "sdp",
        value: sdp_tok.to_string(),
        offset: None,
    })?;
    let prio = (sdp & 0xf) as u8;
    let dst = ((sdp >> 4) & 0xff) as u8;
    let src = ((sdp >> 12) & 0xff) as u8;

    // Token 3: PGN.
    let pgn_tok = toks.next().ok_or(ParseError::BadHeader {
        expected: 3,
        found: 2,
    })?;
    let pgn = u32::from_str_radix(pgn_tok, 16).map_err(|_| ParseError::BadInteger {
        field: "pgn",
        value: pgn_tok.to_string(),
        offset: None,
    })?;

    // Remaining tokens carry payload data. Actisense packs the bytes
    // as one contiguous hex string (e.g. "3F9FDCFFFFFFFFFF") so each
    // remaining token is split into 2-char hex pairs. Multiple tokens
    // are concatenated to be permissive.
    let mut data: SmallVec<[u8; 8]> = SmallVec::new();
    for t in toks {
        let bytes = t.as_bytes();
        if bytes.len() % 2 != 0 {
            return Err(ParseError::BadHexByte {
                index: data.len(),
                value: t.to_string(),
                offset: None,
            });
        }
        for pair in bytes.chunks(2) {
            let s = std::str::from_utf8(pair).map_err(|_| ParseError::BadHexByte {
                index: data.len(),
                value: t.to_string(),
                offset: None,
            })?;
            let b = u8::from_str_radix(s, 16).map_err(|_| ParseError::BadHexByte {
                index: data.len(),
                value: s.to_string(),
                offset: None,
            })?;
            data.push(b);
        }
    }

    Ok(RawFrame {
        timestamp: Some(timestamp),
        prio,
        pgn,
        src,
        dst,
        data,
    })
}

/// Convert Actisense's `HHMMSS[.mmm]` to canboat's `HH:MM:SS,mmm`
/// style.
fn format_actisense_timestamp(rest: &str) -> Result<String, ParseError> {
    let (h_str, mm_str, ss_str, ms_str) = if let Some((time, frac)) = rest.split_once('.') {
        if time.len() < 6 {
            return Err(ParseError::BadInteger {
                field: "timestamp",
                value: rest.to_string(),
                offset: None,
            });
        }
        (&time[..2], &time[2..4], &time[4..6], frac)
    } else if rest.len() >= 6 {
        (&rest[..2], &rest[2..4], &rest[4..6], "000")
    } else {
        return Err(ParseError::BadInteger {
            field: "timestamp",
            value: rest.to_string(),
            offset: None,
        });
    };
    let _h: u32 = h_str.parse().map_err(|_| ParseError::BadInteger {
        field: "timestamp",
        value: rest.to_string(),
        offset: None,
    })?;
    let _m: u32 = mm_str.parse().map_err(|_| ParseError::BadInteger {
        field: "timestamp",
        value: rest.to_string(),
        offset: None,
    })?;
    let _s: u32 = ss_str.parse().map_err(|_| ParseError::BadInteger {
        field: "timestamp",
        value: rest.to_string(),
        offset: None,
    })?;
    // canboat formats with comma + 3-digit milliseconds.
    let ms: u32 = ms_str.parse().map_err(|_| ParseError::BadInteger {
        field: "timestamp",
        value: rest.to_string(),
        offset: None,
    })?;
    Ok(format!("{h_str}:{mm_str}:{ss_str},{:03}", ms % 1000))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_actisense_ascii_line() {
        // Pulled from canboat/common/parse.c documentation. Synthetic:
        //   time 17:33:21.107, src 0x23, dst 0xff, prio 0x7,
        //   pgn 0x1f119 = 127257.
        let line = "A173321.107 23FF7 1F119 01 02 03 04 05 06 07 08";
        let f = parse_line(line).unwrap();
        assert_eq!(f.timestamp.as_deref(), Some("17:33:21,107"));
        assert_eq!(f.prio, 7);
        assert_eq!(f.dst, 0xff);
        assert_eq!(f.src, 0x23);
        assert_eq!(f.pgn, 0x1f119);
        assert_eq!(&f.data[..], &[1, 2, 3, 4, 5, 6, 7, 8]);
    }

    #[test]
    fn rejects_missing_timestamp_prefix() {
        let line = "173321.107 23FF7 1F119";
        assert!(parse_line(line).is_err());
    }

    #[test]
    fn write_line_round_trips_through_parse() {
        let frame = RawFrame {
            timestamp: Some("2026-01-01T17:33:21.107Z".into()),
            prio: 7,
            pgn: 0x1f119,
            src: 0x23,
            dst: 0xff,
            data: [1, 2, 3, 4, 5, 6, 7, 8].into_iter().collect(),
        };
        let mut line = String::new();
        write_line(&mut line, &frame).unwrap();
        assert_eq!(line, "A173321.107 23FF7 1F119 0102030405060708");
        let back = parse_line(&line).unwrap();
        assert_eq!(
            (back.prio, back.pgn, back.src, back.dst),
            (7, 0x1f119, 0x23, 0xff)
        );
        assert_eq!(&back.data[..], &[1, 2, 3, 4, 5, 6, 7, 8]);
        // Timestamp survives as the same instant (parser stores comma form).
        assert_eq!(back.timestamp.as_deref(), Some("17:33:21,107"));
    }

    #[test]
    fn write_line_empty_data() {
        let frame = RawFrame {
            timestamp: None,
            prio: 3,
            pgn: 0x1f119,
            src: 1,
            dst: 2,
            data: SmallVec::new(),
        };
        let mut line = String::new();
        write_line(&mut line, &frame).unwrap();
        assert_eq!(line, "A000000.000 1023 1F119");
    }
}
