// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Linux SocketCAN `candump` text formats.
//!
//! The standard capture tool on a J1939 or NMEA 2000 CAN interface is
//! `candump` from can-utils, and it prints two common shapes:
//!
//! ```text
//!   can0  18EEFF00   [8]  8E F2 DD E8 00 96 64 40      (pretty, default)
//! (1436509053.762905) can0 18EEFF00#8EF2DDE800966440   (log, -l / -L)
//! ```
//!
//! Both carry one raw CAN frame per line: a 29-bit ISO 11783 identifier
//! and up to 8 data bytes. The pretty form has no timestamp at all
//! (`RawFrame.timestamp` stays `None` — the caller stamps receive
//! time); the log form's epoch seconds are converted to the ISO shape
//! downstream emitters expect, by pure arithmetic so wasm32 (no host
//! clock) takes the same path.
//!
//! canboat C handles these through the separate `candump2analyzer`
//! converter (whose parser this mirrors); here they are first-class
//! input formats, so `canboat convert` / `analyzer --j1939` read a
//! candump capture directly.

use smallvec::SmallVec;

use crate::format::common::iso11783_decompose;
use crate::format::plain::ParseError;
use crate::format::timestamp::days_to_ymd;
use crate::frame::RawFrame;

/// True when `line` looks like a candump *pretty* line:
/// `<iface> <hex-canid> [<len>] <bytes…>`.
pub(crate) fn looks_like_pretty(line: &str) -> bool {
    let mut toks = line.split_whitespace();
    let Some(_iface) = toks.next() else {
        return false;
    };
    let Some(id) = toks.next() else { return false };
    let Some(len) = toks.next() else { return false };
    (3..=8).contains(&id.len())
        && id.bytes().all(|b| b.is_ascii_hexdigit())
        && len.len() >= 3
        && len.starts_with('[')
        && len.ends_with(']')
        && len[1..len.len() - 1].bytes().all(|b| b.is_ascii_digit())
}

/// True when `line` looks like a candump *log* line:
/// `(<epoch.frac>) <iface> <hex-canid>#<hexbytes>`.
pub(crate) fn looks_like_log(line: &str) -> bool {
    let Some(rest) = line.strip_prefix('(') else {
        return false;
    };
    let Some(close) = rest.find(')') else {
        return false;
    };
    let ts = &rest[..close];
    ts.bytes().all(|b| b.is_ascii_digit() || b == b'.')
        && !ts.is_empty()
        && rest[close + 1..].trim_start().contains('#')
}

/// Parse either candump shape into a [`RawFrame`].
pub fn parse_line(line: &str) -> Result<RawFrame, ParseError> {
    let line = line.trim_end_matches(['\r', '\n']);
    let t = line.trim_start();
    if t.is_empty() {
        return Err(ParseError::Empty);
    }
    if t.starts_with('(') {
        parse_log(t)
    } else {
        parse_pretty(t)
    }
}

/// `  can0  18EEFF00   [8]  8E F2 DD E8 00 96 64 40`
fn parse_pretty(line: &str) -> Result<RawFrame, ParseError> {
    let mut toks = line.split_whitespace();
    let _iface = toks.next().ok_or(ParseError::Empty)?;
    let id_tok = toks.next().ok_or(ParseError::BadHeader {
        expected: 3,
        found: 1,
    })?;
    let canid = u32::from_str_radix(id_tok, 16).map_err(|_| ParseError::BadInteger {
        field: "canid",
        value: id_tok.to_string(),
        offset: None,
    })?;
    let len_tok = toks.next().ok_or(ParseError::BadHeader {
        expected: 3,
        found: 2,
    })?;
    let declared: usize = len_tok
        .strip_prefix('[')
        .and_then(|s| s.strip_suffix(']'))
        .and_then(|s| s.parse().ok())
        .ok_or_else(|| ParseError::BadInteger {
            field: "len",
            value: len_tok.to_string(),
            offset: None,
        })?;
    // A classic CAN frame carries at most 8 bytes; candump never
    // prints more. A longer declaration (CAN-FD) is not N2K/J1939
    // traffic — reject it rather than truncate silently.
    if declared > 8 {
        return Err(ParseError::BadInteger {
            field: "len",
            value: len_tok.to_string(),
            offset: None,
        });
    }

    let (prio, pgn, src, dst) = iso11783_decompose(canid);
    let mut data: SmallVec<[u8; 8]> = SmallVec::new();
    for (i, tok) in toks.take(declared).enumerate() {
        let b = u8::from_str_radix(tok, 16).map_err(|_| ParseError::BadHexByte {
            index: i,
            value: tok.to_string(),
            offset: None,
        })?;
        data.push(b);
    }
    Ok(RawFrame {
        timestamp: None,
        prio,
        pgn,
        src,
        dst,
        data,
    })
}

/// `(1436509053.762905) can0 18EEFF00#8EF2DDE800966440`
fn parse_log(line: &str) -> Result<RawFrame, ParseError> {
    let rest = line.strip_prefix('(').ok_or(ParseError::Empty)?;
    let close = rest.find(')').ok_or(ParseError::BadHeader {
        expected: 3,
        found: 0,
    })?;
    let timestamp = epoch_to_iso(&rest[..close]);
    let mut toks = rest[close + 1..].split_whitespace();
    let _iface = toks.next().ok_or(ParseError::BadHeader {
        expected: 3,
        found: 1,
    })?;
    let frame_tok = toks.next().ok_or(ParseError::BadHeader {
        expected: 3,
        found: 2,
    })?;
    let (id_part, data_part) = frame_tok.split_once('#').ok_or(ParseError::BadHeader {
        expected: 3,
        found: 2,
    })?;
    let canid = u32::from_str_radix(id_part, 16).map_err(|_| ParseError::BadInteger {
        field: "canid",
        value: id_part.to_string(),
        offset: None,
    })?;
    // Remote frames (`R`) and CAN-FD flag digits after `##` are not
    // N2K/J1939 traffic; reject anything but plain hex pairs.
    let (prio, pgn, src, dst) = iso11783_decompose(canid);
    let mut data: SmallVec<[u8; 8]> = SmallVec::new();
    let bytes = data_part.as_bytes();
    if bytes.len() % 2 != 0 {
        return Err(ParseError::BadHexByte {
            index: bytes.len() / 2,
            value: data_part.to_string(),
            offset: None,
        });
    }
    // More than 8 payload bytes means CAN-FD — reject rather than
    // truncate silently into a wrong-layout decode.
    if bytes.len() > 16 {
        return Err(ParseError::BadHexByte {
            index: 8,
            value: data_part.to_string(),
            offset: None,
        });
    }
    // `as_chunks` rather than `chunks_exact`: the length parity was checked
    // above, so the remainder is always empty (clippy::chunks_exact_to_as_chunks).
    for (i, pair) in bytes.as_chunks::<2>().0.iter().enumerate() {
        let s = std::str::from_utf8(pair).map_err(|_| ParseError::BadHexByte {
            index: i,
            value: data_part.to_string(),
            offset: None,
        })?;
        let b = u8::from_str_radix(s, 16).map_err(|_| ParseError::BadHexByte {
            index: i,
            value: s.to_string(),
            offset: None,
        })?;
        data.push(b);
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

/// `1436509053.762905` → `2015-07-10T06:17:33.762Z` by pure integer
/// arithmetic (no host clock — safe on wasm32, stable in tests).
fn epoch_to_iso(ts: &str) -> String {
    let (secs_str, frac_str) = ts.split_once('.').unwrap_or((ts, ""));
    let secs: i64 = secs_str.parse().unwrap_or(0);
    let millis: u32 = {
        let mut buf = [b'0'; 3];
        for (i, b) in frac_str.bytes().take(3).enumerate() {
            buf[i] = b;
        }
        std::str::from_utf8(&buf)
            .ok()
            .and_then(|s| s.parse().ok())
            .unwrap_or(0)
    };
    let days = secs.div_euclid(86_400);
    let day_secs = secs.rem_euclid(86_400) as u32;
    let (y, mo, d) = days_to_ymd(days);
    let (h, m, s) = (day_secs / 3600, (day_secs / 60) % 60, day_secs % 60);
    format!("{y:04}-{mo:02}-{d:02}T{h:02}:{m:02}:{s:02}.{millis:03}Z")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_pretty_line() {
        let f = parse_line("  can0  18EEFF00   [8]  8E F2 DD E8 00 96 64 40").unwrap();
        assert_eq!(f.pgn, 60928);
        assert_eq!(f.prio, 6);
        assert_eq!(f.src, 0);
        assert_eq!(f.dst, 255);
        assert_eq!(f.timestamp, None);
        assert_eq!(
            f.data.as_slice(),
            &[0x8E, 0xF2, 0xDD, 0xE8, 0x00, 0x96, 0x64, 0x40]
        );
    }

    #[test]
    fn parses_pretty_short_frame() {
        let f = parse_line("  vcan0  0CF00300   [3]  FC 00 FF").unwrap();
        assert_eq!(f.pgn, 61443);
        assert_eq!(f.data.len(), 3);
    }

    #[test]
    fn parses_log_line_with_epoch_timestamp() {
        let f = parse_line("(1436509053.762905) can0 18EEFF00#8EF2DDE800966440").unwrap();
        assert_eq!(f.pgn, 60928);
        assert_eq!(f.timestamp.as_deref(), Some("2015-07-10T06:17:33.762Z"));
        assert_eq!(f.data.len(), 8);
        assert_eq!(f.data[0], 0x8E);
    }

    #[test]
    fn detects_pretty_not_plain() {
        assert!(looks_like_pretty("  can0  18EEFF00   [8]  8E F2"));
        assert!(!looks_like_pretty(
            "2026-08-06T00:00:00.000Z,3,61444,0,255,8,ff,ff"
        ));
        // An interface name where the id should be is not hex.
        assert!(!looks_like_pretty("hello world [x] zz"));
    }

    #[test]
    fn detects_log_shape() {
        assert!(looks_like_log("(1436509053.762905) can0 18EEFF00#8E"));
        assert!(!looks_like_log("(not a timestamp) can0 x"));
        assert!(!looks_like_log("plain,1,2,3"));
    }

    #[test]
    fn rejects_odd_hex_in_log_payload() {
        assert!(parse_line("(1.0) can0 18EEFF00#8EF").is_err());
    }

    #[test]
    fn rejects_can_fd_payloads() {
        // Pretty shape declaring more than a classic frame carries.
        assert!(parse_line("  can0  18EEFF00  [12]  00 11 22 33 44 55 66 77 88 99 AA BB").is_err());
        // Log shape with more than 8 payload bytes.
        assert!(parse_line("(1.0) can0 18EEFF00#00112233445566778899").is_err());
    }
}
