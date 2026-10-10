// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Linux SocketCAN `candump` text formats.
//!
//! The standard capture tool on a J1939 or NMEA 2000 CAN interface is
//! `candump` from can-utils, and it prints these shapes:
//!
//! ```text
//!   can0  18EEFF00   [8]  8E F2 DD E8 00 96 64 40      (pretty, default)
//! (1436509053.762905) can0 18EEFF00#8EF2DDE800966440   (log, -l / -L)
//! <0x18eeff01> [8] 05 a0 be 1c 00 a0 a0 c0             (Angstrom, old can-utils)
//! ```
//!
//! All carry one raw CAN frame per line: a 29-bit ISO 11783 identifier
//! and up to 8 data bytes. candump prints a 29-bit identifier with 8 hex
//! digits and an 11-bit one (Quick PCS, other non-ISO devices) with 3, so
//! the identifier's width says which kind a line holds. The pretty and
//! Angstrom forms have no timestamp at all
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

use crate::engine::format::common::iso11783_decompose;
use crate::engine::format::plain::ParseError;
use crate::engine::format::timestamp::days_to_ymd;
use crate::engine::frame::RawFrame;

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

/// True when `line` looks like a candump *Angstrom* line, as the
/// can-utils of the Angstrom distribution print it:
/// `<0x<hex-canid>> [<len>] <bytes…>`.
pub(crate) fn looks_like_angstrom(line: &str) -> bool {
    let mut toks = line.split_whitespace();
    let Some(id) = toks.next().and_then(angstrom_id) else {
        return false;
    };
    let Some(len) = toks.next() else { return false };
    (3..=8).contains(&id.len())
        && id.bytes().all(|b| b.is_ascii_hexdigit())
        && len.len() >= 3
        && len.starts_with('[')
        && len.ends_with(']')
        && len[1..len.len() - 1].bytes().all(|b| b.is_ascii_digit())
}

/// True for the banner the Angstrom candump prints before its first
/// frame: `interface = can0, family = 29, type = 3, proto = 1`. It holds
/// commas, so format detection would otherwise take it for PLAIN.
pub(crate) fn is_banner(line: &str) -> bool {
    line.starts_with("interface = ") && line.contains(", family = ")
}

/// The hex digits of an Angstrom `<0x…>` identifier token.
fn angstrom_id(tok: &str) -> Option<&str> {
    tok.strip_prefix("<0x")
        .or_else(|| tok.strip_prefix("<0X"))?
        .strip_suffix('>')
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

/// Parse any candump shape into a [`RawFrame`], for a 29-bit ISO 11783
/// bus. A line with an 11-bit identifier is refused as a bad `canid`.
#[cfg(test)]
fn parse_line(line: &str) -> Result<RawFrame, ParseError> {
    parse_line_for(line, false)?.ok_or_else(|| ParseError::BadInteger {
        field: "canid",
        value: "an 11-bit identifier".to_string(),
        offset: None,
    })
}

/// Parse any candump shape, keeping only the frames of one kind:
/// 11-bit identifiers when `standard` is set (Quick PCS), 29-bit ISO 11783
/// ones when not. `Ok(None)` for a frame of the other kind.
///
/// An 11-bit identifier is a message type with no priority, source or
/// destination: it becomes the frame's `pgn`, with priority 0, source 0
/// and destination 255.
pub fn parse_line_for(line: &str, standard: bool) -> Result<Option<RawFrame>, ParseError> {
    let line = line.trim_end_matches(['\r', '\n']);
    let t = line.trim_start();
    if t.is_empty() {
        return Err(ParseError::Empty);
    }
    if t.starts_with('(') {
        parse_log(t, standard)
    } else if is_banner(t) {
        Ok(None)
    } else if t.starts_with('<') {
        parse_angstrom(t, standard)
    } else {
        parse_pretty(t, standard)
    }
}

/// The frame header from candump's identifier column, or `None` when the
/// identifier is not of the `standard` kind asked for.
fn header(id_tok: &str, standard: bool) -> Result<Option<(u8, u32, u8, u8)>, ParseError> {
    let canid = u32::from_str_radix(id_tok, 16).map_err(|_| ParseError::BadInteger {
        field: "canid",
        value: id_tok.to_string(),
        offset: None,
    })?;
    // candump's `%03X` for a standard frame, `%08X` for an extended one.
    if (id_tok.len() <= 3) != standard {
        return Ok(None);
    }
    Ok(Some(if standard {
        (0, canid, 0, 255)
    } else {
        iso11783_decompose(canid)
    }))
}

/// `  can0  18EEFF00   [8]  8E F2 DD E8 00 96 64 40`
fn parse_pretty(line: &str, standard: bool) -> Result<Option<RawFrame>, ParseError> {
    let mut toks = line.split_whitespace();
    let _iface = toks.next().ok_or(ParseError::Empty)?;
    let id_tok = toks.next().ok_or(ParseError::BadHeader {
        expected: 3,
        found: 1,
    })?;
    parse_len_and_bytes(id_tok, toks, 2, standard)
}

/// `<0x18eeff01> [8] 05 a0 be 1c 00 a0 a0 c0`: the pretty shape with the
/// identifier in `<0x…>` and no interface column.
fn parse_angstrom(line: &str, standard: bool) -> Result<Option<RawFrame>, ParseError> {
    let mut toks = line.split_whitespace();
    let tok = toks.next().ok_or(ParseError::Empty)?;
    let id_tok = angstrom_id(tok).ok_or_else(|| ParseError::BadInteger {
        field: "canid",
        value: tok.to_string(),
        offset: None,
    })?;
    parse_len_and_bytes(id_tok, toks, 1, standard)
}

/// The rest of a pretty or Angstrom line after the identifier: `[<len>]`
/// and that many hex bytes. `header_tokens` is how many tokens came
/// before the length, for the error when it is missing.
fn parse_len_and_bytes<'a>(
    id_tok: &str,
    mut toks: impl Iterator<Item = &'a str>,
    header_tokens: usize,
    standard: bool,
) -> Result<Option<RawFrame>, ParseError> {
    let Some((prio, pgn, src, dst)) = header(id_tok, standard)? else {
        return Ok(None);
    };
    let len_tok = toks.next().ok_or(ParseError::BadHeader {
        expected: header_tokens + 1,
        found: header_tokens,
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

    let mut data: SmallVec<[u8; 8]> = SmallVec::new();
    for (i, tok) in toks.take(declared).enumerate() {
        let b = u8::from_str_radix(tok, 16).map_err(|_| ParseError::BadHexByte {
            index: i,
            value: tok.to_string(),
            offset: None,
        })?;
        data.push(b);
    }
    // A line cut short: decoding the bytes it has as the whole frame
    // would read the missing ones as field values.
    if data.len() < declared {
        return Err(ParseError::BadPayloadCount {
            expected: declared,
            found: data.len(),
        });
    }
    Ok(Some(RawFrame {
        timestamp: None,
        prio,
        pgn,
        src,
        dst,
        data,
    }))
}

/// `(1436509053.762905) can0 18EEFF00#8EF2DDE800966440`
fn parse_log(line: &str, standard: bool) -> Result<Option<RawFrame>, ParseError> {
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
    let Some((prio, pgn, src, dst)) = header(id_part, standard)? else {
        return Ok(None);
    };
    // Remote frames (`R`) and CAN-FD flag digits after `##` are not
    // N2K/J1939 traffic; reject anything but plain hex pairs.
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
    Ok(Some(RawFrame {
        timestamp: Some(timestamp),
        prio,
        pgn,
        src,
        dst,
        data,
    }))
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
    fn an_identifier_of_3_digits_is_an_11_bit_frame() {
        let line = "(1702468372.909137) can0 6C1#C1186A0000000200";
        let f = parse_line_for(line, true).unwrap().unwrap();
        assert_eq!((f.prio, f.pgn, f.src, f.dst), (0, 0x6c1, 0, 255));
        assert_eq!(f.data.as_slice(), &[0xc1, 0x18, 0x6a, 0, 0, 0, 2, 0]);
        let pretty = parse_line_for("  can0  6C1   [8]  C1 18 6A 00 00 00 02 00", true).unwrap();
        assert_eq!(pretty.unwrap().pgn, 0x6c1);

        // Each bus keeps its own kind of frame and skips the other.
        assert_eq!(parse_line_for(line, false).unwrap(), None);
        assert_eq!(
            parse_line_for("(1.0) can0 18EEFF00#8EF2DDE800966440", true).unwrap(),
            None
        );
        // An 11-bit identifier is not read as a 29-bit one.
        assert!(parse_line(line).is_err());
    }

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
    fn parses_angstrom_line() {
        let f = parse_line("<0x18eeff01> [8] 05 a0 be 1c 00 a0 a0 c0").unwrap();
        assert_eq!((f.prio, f.pgn, f.src, f.dst), (6, 60928, 1, 255));
        assert_eq!(f.timestamp, None);
        assert_eq!(
            f.data.as_slice(),
            &[0x05, 0xa0, 0xbe, 0x1c, 0x00, 0xa0, 0xa0, 0xc0]
        );
        // A trailing space, as the Angstrom candump prints one.
        let f = parse_line("<0x15fd0723> [3] 00 ff ff ").unwrap();
        assert_eq!((f.pgn, f.src), (130311, 0x23));
        assert_eq!(f.data.as_slice(), &[0x00, 0xff, 0xff]);
        // An 11-bit identifier, printed with 3 digits.
        let f = parse_line_for("<0x6c1> [2] c1 18", true).unwrap().unwrap();
        assert_eq!(f.pgn, 0x6c1);
        assert_eq!(parse_line_for("<0x6c1> [2] c1 18", false).unwrap(), None);
    }

    #[test]
    fn detects_angstrom_shape() {
        assert!(looks_like_angstrom("<0x18eeff01> [8] 05 a0 be 1c"));
        assert!(!looks_like_angstrom("<0x18eeff01 [8] 05"));
        assert!(!looks_like_angstrom("<0xzz> [8] 05"));
        assert!(!looks_like_angstrom("  can0  18EEFF00   [8]  8E F2"));
        assert!(!looks_like_pretty("<0x18eeff01> [8] 05 a0 be 1c"));
    }

    #[test]
    fn the_angstrom_banner_is_candump_but_no_frame() {
        let banner = "interface = can0, family = 29, type = 3, proto = 1";
        assert!(is_banner(banner));
        assert_eq!(parse_line_for(banner, false).unwrap(), None);
        assert_eq!(
            crate::engine::format::detect(banner),
            Some(crate::engine::format::InputFormat::Candump)
        );
    }

    #[test]
    fn rejects_bad_angstrom_lines() {
        assert!(parse_line("<0x18eeff01> [12] 00 11 22 33 44 55 66 77 88 99 aa bb").is_err());
        assert!(parse_line("<0x18eeff01>").is_err());
        assert!(parse_line("<18eeff01> [1] 00").is_err());
        // Fewer bytes than declared, in both shapes that declare a length.
        assert!(matches!(
            parse_line("<0x18eeff01> [8] 05"),
            Err(ParseError::BadPayloadCount {
                expected: 8,
                found: 1
            })
        ));
        assert!(parse_line("  can0  18EEFF00   [8]  8E").is_err());
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
