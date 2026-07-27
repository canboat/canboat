// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Chetco N2K-USB line format.
//!
//! Layout (one NMEA-style sentence per line):
//!
//! ```text
//!   $PCDIN,<PGN:hex>,<tstamp:hex>,<SRC:hex>,<data_hex>...*<XX>
//! ```
//!
//!   - `PGN` — 24-bit PGN number, hex.
//!   - `tstamp` — milliseconds since Unix epoch, hex (variable width;
//!     canboat C's parser uses `sscanf %x` which stops at the first
//!     non-hex char).
//!   - `SRC` — source address, hex.
//!   - data bytes follow as a continuous hex string (no separators).
//!   - The trailing `*XX` is an XOR checksum (not verified here, only
//!     used as the data-end marker).
//!
//! Mirrors `parseRawFormatChetco` in canboat/common/parse.c. Output
//! timestamp shape matches canboat: `YYYY-MM-DDTHH:MM:SS,mmm`
//! (UTC seconds + 3-digit ms).
//!
//! Priority is always emitted as 0 (canboat does the same — Chetco
//! captures don't carry one), and the destination is set to 255
//! (broadcast).
//!
//! Multi-packet messages arrive pre-coalesced on a single line, so
//! the caller should treat this format as `MULTIPACKETS_COALESCED`.

use smallvec::SmallVec;

use crate::format::plain::ParseError;
use crate::format::timestamp::days_to_ymd;
use crate::frame::{FASTPACKET_MAX_SIZE, RawFrame};

pub fn parse_line(line: &str) -> Result<RawFrame, ParseError> {
    let line = line.trim_end_matches(['\r', '\n']);
    if line.is_empty() {
        return Err(ParseError::Empty);
    }
    let rest = line
        .strip_prefix("$PCDIN,")
        .ok_or(ParseError::MissingTimestamp)?;
    let mut iter = rest.splitn(4, ',');
    let pgn_tok = iter.next().ok_or(ParseError::BadHeader {
        expected: 4,
        found: 1,
    })?;
    let tstamp_tok = iter.next().ok_or(ParseError::BadHeader {
        expected: 4,
        found: 2,
    })?;
    let src_tok = iter.next().ok_or(ParseError::BadHeader {
        expected: 4,
        found: 3,
    })?;
    let data_tok = iter.next().ok_or(ParseError::BadHeader {
        expected: 4,
        found: 4,
    })?;

    let pgn = u32::from_str_radix(pgn_tok, 16).map_err(|_| ParseError::BadInteger {
        field: "pgn",
        value: pgn_tok.to_string(),
        offset: None,
    })?;
    // Chetco's tstamp is hex milliseconds; canboat's `sscanf %x`
    // stops at any non-hex byte, so accept a trailing `!` (seen in
    // canboat's own parse comment example: `089C77D!`) by taking
    // only the leading hex run.
    let tstamp_hex: String = tstamp_tok
        .chars()
        .take_while(|c| c.is_ascii_hexdigit())
        .collect();
    let tstamp_ms = u64::from_str_radix(&tstamp_hex, 16).map_err(|_| ParseError::BadInteger {
        field: "tstamp",
        value: tstamp_tok.to_string(),
        offset: None,
    })?;
    let src = u8::from_str_radix(src_tok, 16).map_err(|_| ParseError::BadInteger {
        field: "src",
        value: src_tok.to_string(),
        offset: None,
    })?;

    // Data: continuous hex pairs, terminated by `*XX` checksum.
    let data_str = data_tok.split('*').next().unwrap_or("");
    if data_str.len() % 2 != 0 {
        return Err(ParseError::BadHexByte {
            index: data_str.len() / 2,
            value: data_str.chars().last().unwrap_or(' ').to_string(),
            offset: None,
        });
    }
    let mut data: SmallVec<[u8; 8]> = SmallVec::new();
    for (i, chunk) in data_str.as_bytes().chunks(2).enumerate() {
        if i >= FASTPACKET_MAX_SIZE {
            break;
        }
        let s = std::str::from_utf8(chunk).map_err(|_| ParseError::BadHexByte {
            index: i,
            value: String::from_utf8_lossy(chunk).into_owned(),
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
        timestamp: Some(format_chetco_timestamp(tstamp_ms)),
        prio: 0,
        pgn,
        src,
        dst: 0xff,
        data,
    })
}

/// `<ms-since-epoch>` → `YYYY-MM-DDTHH:MM:SS,mmm`. Matches the format
/// string canboat assembles in `parseRawFormatChetco`:
///
/// ```c
/// strftime(ts, n, "%Y-%m-%dT%H:%M:%S", localtime_r(&t, &tm));
/// sprintf(ts + strlen(ts), ",%3.3u", tstamp_ms % 1000);
/// ```
fn format_chetco_timestamp(tstamp_ms: u64) -> String {
    let secs = (tstamp_ms / 1000) as i64;
    let ms = tstamp_ms % 1000;
    let days = secs.div_euclid(86_400);
    let day_seconds = secs.rem_euclid(86_400) as u32;
    let h = day_seconds / 3600;
    let m = (day_seconds / 60) % 60;
    let s = day_seconds % 60;
    let (year, mo, d) = days_to_ymd(days);
    format!("{year:04}-{mo:02}-{d:02}T{h:02}:{m:02}:{s:02},{ms:03}")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_chetco_sentence() {
        // Synthesised from canboat's own parse comment:
        //   $PCDIN,01F801,FB12345,07,A0B1C2D3E4F506*XX
        // PGN = 0x01F801 = 129025; src = 7; 7 data bytes;
        // tstamp = 0xFB12345 = 263 267 141 ms = 263 267 s + 141 ms
        //                    = 3d + 1h 7m 47s + 141 ms
        //                    → 1970-01-04T01:07:47,141.
        let line = "$PCDIN,01F801,FB12345,07,A0B1C2D3E4F506*55";
        let f = parse_line(line).unwrap();
        assert_eq!(f.pgn, 0x1f801);
        assert_eq!(f.src, 0x07);
        assert_eq!(f.dst, 0xff);
        assert_eq!(f.prio, 0);
        assert_eq!(
            f.data.as_slice(),
            &[0xa0, 0xb1, 0xc2, 0xd3, 0xe4, 0xf5, 0x06]
        );
        assert_eq!(f.timestamp.as_deref(), Some("1970-01-04T01:07:47,141"));
    }

    #[test]
    fn rejects_missing_prefix() {
        assert!(parse_line("BOGUS,01,02").is_err());
    }
}
